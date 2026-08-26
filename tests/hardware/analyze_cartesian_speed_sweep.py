#!/usr/bin/env python3
"""Rank a Yunyi Cartesian speed sweep with smoothness as the primary goal."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def circle_arc(start: np.ndarray, via: np.ndarray, end: np.ndarray) -> np.ndarray:
    chord1 = via - start
    chord2 = end - start
    e1 = chord1 / np.linalg.norm(chord1)
    normal = np.cross(chord1, chord2)
    normal /= np.linalg.norm(normal)
    e2 = np.cross(normal, e1)
    bx = float(np.dot(chord1, e1))
    cx = float(np.dot(chord2, e1))
    cy = float(np.dot(chord2, e2))
    ux = bx / 2.0
    uy = (float(np.dot(chord2, chord2)) - 2.0 * cx * ux) / (2.0 * cy)
    center = start + ux * e1 + uy * e2

    def angle(point: np.ndarray) -> float:
        radius = point - center
        return math.atan2(float(np.dot(radius, e2)), float(np.dot(radius, e1)))

    a0, av, a1 = angle(start), angle(via), angle(end)
    ccw_total = (a1 - a0) % (2.0 * math.pi)
    ccw_via = (av - a0) % (2.0 * math.pi)
    delta = ccw_total if ccw_via <= ccw_total else -((a0 - a1) % (2.0 * math.pi))
    theta = np.linspace(a0, a0 + delta, 1001)
    radius = np.linalg.norm(start - center)
    return center + radius * (
        np.cos(theta)[:, None] * e1 + np.sin(theta)[:, None] * e2
    )


def planned_geometry(data: dict[str, object]) -> np.ndarray:
    start = np.asarray(data["start"][:3], dtype=float)
    end = np.asarray(data["end"][:3], dtype=float)
    if data["motion"] == "linear":
        alpha = np.linspace(0.0, 1.0, 1001)[:, None]
        return start + alpha * (end - start)
    return circle_arc(start, np.asarray(data["via"][:3], dtype=float), end)


def actual_path(data: dict[str, object]) -> np.ndarray:
    samples = [sample for sample in data["samples"] if sample["phase"] == "running"]
    xyz = np.asarray([sample["pose"][:3] for sample in samples], dtype=float)
    start = np.asarray(data["start"][:3], dtype=float)
    start_index = int(np.argmin(np.linalg.norm(xyz - start, axis=1)))
    end_index = len(samples) - 1
    for index in range(start_index, len(samples)):
        if samples[index].get("motion_state") == "COMPLETED":
            end_index = index
            break
    return xyz[start_index : end_index + 1]


def geometry_deviation(actual: np.ndarray, planned: np.ndarray) -> np.ndarray:
    output = np.empty(len(actual), dtype=float)
    for index, point in enumerate(actual):
        output[index] = np.min(np.linalg.norm(planned - point, axis=1))
    return output


def highpass_rms(values: np.ndarray, sample_hz: float, cutoff_hz: float = 5.0) -> float:
    window = max(5, int(round(sample_hz / cutoff_hz)))
    if window % 2 == 0:
        window += 1
    if len(values) <= window * 2:
        return float("nan")
    smooth = np.convolve(values, np.ones(window) / window, mode="valid")
    half = window // 2
    residual = values[half : len(values) - half] - smooth
    return float(np.sqrt(np.mean(residual * residual)))


def read_path_trace(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    valid = []
    for row in rows:
        try:
            tracking = float(row["tracking_position_error"])
            command = float(row["command_q_r-joint5"])
            actual = float(row["actual_q_r-joint5"])
        except (KeyError, ValueError):
            continue
        if math.isfinite(command) and math.isfinite(actual):
            valid.append(row)
    start_candidates = [
        index for index, row in enumerate(valid)
        if float(row["tracking_position_error"]) > 0.0
    ]
    if start_candidates:
        valid = valid[max(0, start_candidates[0] - 1) :]
    timestamp = np.asarray([float(row["timestamp_ns"]) for row in valid])
    result: dict[str, np.ndarray] = {
        "time": (timestamp - timestamp[0]) * 1.0e-9,
        "tracking_time_scale": np.asarray(
            [float(row["tracking_time_scale"]) for row in valid]
        ),
        "tracking_position_error": np.asarray(
            [float(row["tracking_position_error"]) for row in valid]
        ),
    }
    for joint in range(1, 8):
        result[f"command_{joint}"] = np.asarray(
            [float(row[f"command_q_r-joint{joint}"]) for row in valid]
        )
        result[f"actual_{joint}"] = np.asarray(
            [float(row[f"actual_q_r-joint{joint}"]) for row in valid]
        )
        result[f"velocity_{joint}"] = np.asarray(
            [float(row[f"actual_dq_r-joint{joint}"]) for row in valid]
        )
        result[f"pv_v_{joint}"] = np.asarray(
            [float(row[f"pv_velocity_limit_r-joint{joint}"]) for row in valid]
        )
    return result


def analyze_run(record: dict[str, object]) -> dict[str, object]:
    result_path = Path(record["result"])
    trace_path = Path(record["trace"])
    data = json.loads(result_path.read_text())
    planned = planned_geometry(data)
    actual = actual_path(data)
    deviation_mm = geometry_deviation(actual, planned) * 1000.0
    trace = read_path_trace(trace_path)
    duration = max(1e-9, float(trace["time"][-1] - trace["time"][0]))
    trace_hz = (len(trace["time"]) - 1) / duration

    joint_hp: list[float] = []
    joint_rms: list[float] = []
    joint_peak: list[float] = []
    for joint in range(1, 8):
        error_mrad = (trace[f"actual_{joint}"] - trace[f"command_{joint}"]) * 1000.0
        joint_hp.append(highpass_rms(error_mrad, trace_hz))
        joint_rms.append(float(np.sqrt(np.mean(error_mrad * error_mrad))))
        joint_peak.append(float(np.max(np.abs(error_mrad))))

    hold_p2p = max(
        float(joint["hold_position_peak_to_peak_rad"])
        for joint in data["metrics"]["joints"]
    )
    scale = trace["tracking_time_scale"]
    pv_v = np.concatenate([trace[f"pv_v_{joint}"] for joint in range(1, 8)])
    return {
        "run": int(record["run"]),
        "speed_percent": float(record["speed_percent"]),
        "settled_s": float(record["settled_s"]),
        "sample_hz": float(record["sample_hz"]),
        "max_temperature_c": float(record["max_temperature_c"]),
        "xyz_error_mean_mm": float(np.mean(deviation_mm)),
        "xyz_error_p95_mm": float(np.percentile(deviation_mm, 95)),
        "xyz_error_max_mm": float(np.max(deviation_mm)),
        "xyz_error_highpass_rms_mm": highpass_rms(
            deviation_mm, float(record["sample_hz"])
        ),
        "joint_tracking_highpass_mean_mrad": float(np.mean(joint_hp)),
        "joint_tracking_highpass_max_mrad": float(np.max(joint_hp)),
        "joint_tracking_rms_mean_mrad": float(np.mean(joint_rms)),
        "joint_tracking_rms_max_mrad": float(np.max(joint_rms)),
        "joint_tracking_peak_max_mrad": float(np.max(joint_peak)),
        "j5_tracking_highpass_rms_mrad": joint_hp[4],
        "j5_tracking_rms_mrad": joint_rms[4],
        "j5_tracking_peak_mrad": joint_peak[4],
        "tracking_scale_median": float(np.median(scale)),
        "tracking_scale_p05": float(np.percentile(scale, 5)),
        "pv_velocity_median_rad_s": float(np.nanmedian(pv_v)),
        "hold_position_p2p_max_rad": hold_p2p,
    }


def normalized(values: np.ndarray) -> np.ndarray:
    lower = float(np.min(values))
    upper = float(np.max(values))
    if upper <= lower:
        return np.zeros_like(values)
    return (values - lower) / (upper - lower)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    args = parser.parse_args()

    manifest = json.loads(args.manifest.read_text())
    runs = [analyze_run(record) for record in manifest["runs"]]
    tool_hp = np.asarray([run["xyz_error_highpass_rms_mm"] for run in runs])
    joint_mean = np.asarray(
        [run["joint_tracking_highpass_mean_mrad"] for run in runs]
    )
    joint_max = np.asarray(
        [run["joint_tracking_highpass_max_mrad"] for run in runs]
    )
    path_p95 = np.asarray([run["xyz_error_p95_mm"] for run in runs])
    duration = np.asarray([run["settled_s"] for run in runs])
    score = (
        0.40 * normalized(tool_hp)
        + 0.30 * normalized(joint_mean)
        + 0.20 * normalized(joint_max)
        + 0.07 * normalized(path_p95)
        + 0.03 * normalized(duration)
    )
    for run, value in zip(runs, score):
        run["smoothness_score"] = float(value)
    runs.sort(key=lambda run: run["smoothness_score"])
    for rank, run in enumerate(runs, start=1):
        run["smoothness_rank"] = rank

    args.output_json.write_text(json.dumps(runs, indent=2) + "\n")
    with args.output_csv.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(runs[0]))
        writer.writeheader()
        writer.writerows(runs)
    print(json.dumps(runs, indent=2), flush=True)


if __name__ == "__main__":
    main()

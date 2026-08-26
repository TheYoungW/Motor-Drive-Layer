#!/usr/bin/env python3
"""Rank native joint-trajectory parameters with right-arm smoothness first."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def highpass_rms(values: np.ndarray, sample_hz: float, cutoff_hz: float = 5.0) -> float:
    window = max(5, int(round(sample_hz / cutoff_hz)))
    if window % 2 == 0:
        window += 1
    if len(values) <= window * 2:
        return float("nan")
    smooth = np.convolve(values, np.ones(window) / window, mode="valid")
    half = window // 2
    residual = values[half:len(values) - half] - smooth
    return float(np.sqrt(np.mean(residual * residual)))


def read_formal_trace(
    path: Path,
) -> tuple[list[dict[str, str]], np.ndarray, float]:
    with path.open(newline="") as stream:
        rows = [
            row for row in csv.DictReader(stream)
            if int(row["planned_valid_mask"]) != 0 and
            int(row["motion_state"]) == 1 and
            float(row["progress"]) < 0.999999
        ]
    if len(rows) < 10:
        raise RuntimeError(f"trajectory trace is incomplete: {path}")
    timestamps = np.asarray([float(row["timestamp_ns"]) for row in rows])
    duration = (timestamps[-1] - timestamps[0]) * 1.0e-9
    return rows, timestamps * 1.0e-9, (len(rows) - 1) / max(duration, 1.0e-9)


def analyze(record: dict[str, object]) -> dict[str, object]:
    data = json.loads(Path(record["result"]).read_text())
    rows, timestamps_s, sample_hz = read_formal_trace(Path(record["trace"]))
    moving_joints = [
        index for index, joint in enumerate(data["metrics"]["joints"], start=1)
        if abs(float(joint["travel_rad"])) >= 0.02
    ]
    tracking_rms: list[float] = []
    tracking_hp: list[float] = []
    velocity_hp: list[float] = []
    planned_peak_acceleration: list[float] = []
    plan_command_peak: list[float] = []
    for joint in moving_joints:
        planned = np.asarray([
            float(row[f"planned_q_r-joint{joint}"]) for row in rows
        ])
        planned_velocity = np.asarray([
            float(row[f"planned_dq_r-joint{joint}"]) for row in rows
        ])
        command = np.asarray([
            float(row[f"command_q_r-joint{joint}"]) for row in rows
        ])
        actual = np.asarray([
            float(row[f"actual_q_r-joint{joint}"]) for row in rows
        ])
        actual_velocity = np.asarray([
            float(row[f"actual_dq_r-joint{joint}"]) for row in rows
        ])
        error_mrad = (actual - command) * 1000.0
        tracking_rms.append(float(np.sqrt(np.mean(error_mrad * error_mrad))))
        tracking_hp.append(highpass_rms(error_mrad, sample_hz))
        velocity_hp.append(highpass_rms(actual_velocity * 1000.0, sample_hz))
        acceleration = np.diff(planned_velocity) / np.diff(timestamps_s)
        planned_peak_acceleration.append(float(np.max(np.abs(acceleration))))
        plan_command_peak.append(float(np.max(np.abs(planned - command))) * 1000.0)
    hold_p2p = max(
        float(data["metrics"]["joints"][joint - 1][
            "hold_position_peak_to_peak_rad"
        ]) * 1000.0
        for joint in moving_joints
    )
    hold_peak_velocity = max(
        float(data["metrics"]["joints"][joint - 1][
            "hold_peak_abs_velocity_rad_s"
        ])
        for joint in moving_joints
    )
    return {
        "run": int(record["run"]),
        "duration_s": float(record["duration_s"]),
        "pv_velocity_limit_rad_s": float(record["pv_velocity_limit_rad_s"]),
        "completed_s": float(record["completed_s"]),
        "sample_hz": float(record["sample_hz"]),
        "max_temperature_c": float(record["max_temperature_c"]),
        "tracking_rms_mean_mrad": float(np.mean(tracking_rms)),
        "tracking_highpass_mean_mrad": float(np.mean(tracking_hp)),
        "tracking_highpass_max_mrad": float(np.max(tracking_hp)),
        "velocity_highpass_mean_mrad_s": float(np.mean(velocity_hp)),
        "planned_peak_acceleration_rad_s2": float(
            np.max(planned_peak_acceleration)
        ),
        "planned_command_peak_mrad": float(np.max(plan_command_peak)),
        "hold_position_p2p_max_mrad": hold_p2p,
        "hold_peak_abs_velocity_rad_s": hold_peak_velocity,
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
    runs = [analyze(record) for record in manifest["runs"]]
    score = (
        0.30 * normalized(np.asarray([
            run["tracking_highpass_mean_mrad"] for run in runs
        ]))
        + 0.25 * normalized(np.asarray([
            run["velocity_highpass_mean_mrad_s"] for run in runs
        ]))
        + 0.15 * normalized(np.asarray([
            run["planned_peak_acceleration_rad_s2"] for run in runs
        ]))
        + 0.15 * normalized(np.asarray([
            run["hold_position_p2p_max_mrad"] for run in runs
        ]))
        + 0.10 * normalized(np.asarray([
            run["tracking_rms_mean_mrad"] for run in runs
        ]))
        + 0.05 * normalized(np.asarray([
            run["completed_s"] for run in runs
        ]))
    )
    for run, value in zip(runs, score):
        run["smoothness_score"] = float(value)
    runs.sort(key=lambda item: item["smoothness_score"])
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

#!/usr/bin/env python3
"""Plot planned Cartesian geometry and measured Yunyi tool0 feedback."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


PLANNED_COLOR = "#2563eb"
COMMAND_COLOR = "#16a34a"
ACTUAL_COLOR = "#f97316"
ERROR_COLOR = "#dc2626"
ORIENTATION_COLOR = "#7c3aed"
NEUTRAL_COLOR = "#374151"


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


def actual_path(data: dict[str, object]) -> tuple[np.ndarray, np.ndarray]:
    samples = [sample for sample in data["samples"] if sample["phase"] == "running"]
    start = np.asarray(data["start"][:3], dtype=float)
    xyz = np.asarray([sample["pose"][:3] for sample in samples], dtype=float)
    start_index = int(np.argmin(np.linalg.norm(xyz - start, axis=1)))
    end_index = len(samples) - 1
    for index in range(start_index, len(samples)):
        if samples[index]["motion_state"] == "COMPLETED":
            end_index = index
            break
    selected = samples[start_index : end_index + 1]
    return (
        np.asarray([sample["pose"][:3] for sample in selected], dtype=float),
        np.asarray([sample["pose"][3:] for sample in selected], dtype=float),
    )


def nearest_geometry(actual: np.ndarray, planned: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    nearest_index = np.empty(len(actual), dtype=int)
    deviation = np.empty(len(actual), dtype=float)
    for index, point in enumerate(actual):
        distances = np.linalg.norm(planned - point, axis=1)
        nearest_index[index] = int(np.argmin(distances))
        deviation[index] = distances[nearest_index[index]]
    return nearest_index / (len(planned) - 1), deviation


def rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def orientation_errors(actual_rpy: np.ndarray, expected_rpy: np.ndarray) -> np.ndarray:
    expected = rpy_matrix(expected_rpy)
    errors = []
    for rpy in actual_rpy:
        relative = expected.T @ rpy_matrix(rpy)
        cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
        errors.append(math.acos(float(cosine)))
    return np.asarray(errors)


def read_joint_trace(path: Path) -> dict[str, np.ndarray]:
    rows: list[tuple[float, float, float, float, float]] = []
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            planned = float(row["planned_q_r-joint5"])
            command = float(row["command_q_r-joint5"])
            actual = float(row["actual_q_r-joint5"])
            if not all(math.isfinite(value) for value in (planned, command, actual)):
                continue
            tracking_error = float(row.get("tracking_position_error", "nan"))
            rows.append(
                (float(row["timestamp_ns"]), planned, command, actual, tracking_error)
            )
    values = np.asarray(rows)
    # New traces record zero tracking error during the automatic joint approach
    # and a measured value during the declared Linear/Circular path. Showing
    # only that path keeps the joint chart aligned with the Cartesian charts.
    measured = np.flatnonzero(np.isfinite(values[:, 4]) & (values[:, 4] > 0.0))
    if measured.size:
        values = values[max(0, int(measured[0]) - 1) :]
    return {
        "time": (values[:, 0] - values[0, 0]) * 1.0e-9,
        "planned": values[:, 1],
        "command": values[:, 2],
        "actual": values[:, 3],
    }


def equal_3d_axes(axis: object, points: np.ndarray) -> None:
    center = (points.min(axis=0) + points.max(axis=0)) / 2.0
    radius = max(float(np.ptp(points[:, dimension])) for dimension in range(3)) / 2.0
    radius = max(radius, 0.01)
    axis.set_xlim(center[0] - radius, center[0] + radius)
    axis.set_ylim(center[1] - radius, center[1] + radius)
    axis.set_zlim(center[2] - radius, center[2] + radius)


def plot_one(json_path: Path, trace_path: Path, output: Path) -> dict[str, float]:
    data = json.loads(json_path.read_text())
    planned = planned_geometry(data)
    actual, actual_rpy = actual_path(data)
    path_progress, deviation = nearest_geometry(actual, planned)
    orientation = orientation_errors(actual_rpy, np.asarray(data["start"][3:], dtype=float))
    trace = read_joint_trace(trace_path)

    figure = plt.figure(figsize=(14, 14), constrained_layout=True)
    grid = figure.add_gridspec(3, 2)
    spatial = figure.add_subplot(grid[0, 0], projection="3d")
    spatial.plot(*planned.T, color=PLANNED_COLOR, linewidth=2.5, label="Planned path")
    spatial.plot(*actual.T, color=ACTUAL_COLOR, linewidth=1.2, alpha=0.9, label="Measured tool0")
    spatial.scatter(*planned[0], color=NEUTRAL_COLOR, s=45, marker="o", label="Start")
    spatial.scatter(*planned[-1], color=NEUTRAL_COLOR, s=55, marker="x", label="End")
    spatial.set_xlabel("X (m)")
    spatial.set_ylabel("Y (m)")
    spatial.set_zlabel("Z (m)")
    spatial.set_title("1. Tool0 path in 3D")
    equal_3d_axes(spatial, np.vstack((planned, actual)))
    spatial.legend(fontsize=8)

    plane = figure.add_subplot(grid[0, 1])
    if data["motion"] == "linear":
        plane.plot(planned[:, 2], planned[:, 1], color=PLANNED_COLOR, linewidth=2.5, label="Planned path")
        plane.plot(actual[:, 2], actual[:, 1], color=ACTUAL_COLOR, linewidth=1.2, label="Measured tool0")
        plane.set_xlabel("Z (m)")
        plane.set_ylabel("Y (m)")
        plane.set_title("2. Tool0 path projected on Y-Z")
    else:
        plane.plot(planned[:, 1], planned[:, 2], color=PLANNED_COLOR, linewidth=2.5, label="Planned path")
        plane.plot(actual[:, 1], actual[:, 2], color=ACTUAL_COLOR, linewidth=1.2, label="Measured tool0")
        plane.set_xlabel("Y (m)")
        plane.set_ylabel("Z (m)")
        plane.set_title("2. Tool0 path projected on Y-Z")
    plane.axis("equal")
    plane.grid(alpha=0.25)
    plane.legend()

    error_axis = figure.add_subplot(grid[1, 0])
    xyz_error_mm = deviation * 1000.0
    error_axis.plot(path_progress * 100.0, xyz_error_mm, color=ERROR_COLOR, linewidth=1.1, label="XYZ path error")
    error_axis.set_xlabel("Path progress (%)")
    error_axis.set_ylabel("XYZ path error (mm)")
    error_axis.grid(alpha=0.25)
    error_axis.legend(loc="upper right")
    error_axis.set_title("3. XYZ path error — lower is better")
    error_axis.text(
        0.02,
        0.96,
        f"P95 {np.percentile(xyz_error_mm, 95):.2f} mm   Max {np.max(xyz_error_mm):.2f} mm",
        transform=error_axis.transAxes,
        va="top",
        color=ERROR_COLOR,
    )

    orientation_axis = figure.add_subplot(grid[1, 1])
    orientation_deg = np.degrees(orientation)
    orientation_axis.plot(
        path_progress * 100.0,
        orientation_deg,
        color=ORIENTATION_COLOR,
        linewidth=1.0,
        label="Orientation error",
    )
    orientation_axis.set_xlabel("Path progress (%)")
    orientation_axis.set_ylabel("Orientation error (deg)")
    orientation_axis.grid(alpha=0.25)
    orientation_axis.legend(loc="upper right")
    orientation_axis.set_title("4. Orientation error — lower is better")
    orientation_axis.text(
        0.02,
        0.96,
        f"P95 {np.percentile(orientation_deg, 95):.2f}°   Max {np.max(orientation_deg):.2f}°",
        transform=orientation_axis.transAxes,
        va="top",
        color=ORIENTATION_COLOR,
    )

    joint = figure.add_subplot(grid[2, 0])
    joint.plot(trace["time"], trace["planned"], color=PLANNED_COLOR, linewidth=2.2, label="Planned q")
    joint.plot(trace["time"], trace["command"], color=COMMAND_COLOR, linewidth=1.4, linestyle="--", label="Sent command q")
    joint.plot(trace["time"], trace["actual"], color=ACTUAL_COLOR, linewidth=1.0, label="Measured q")
    joint.set_xlabel("Path execution time (s)")
    joint.set_ylabel("Right J5 angle (rad)")
    joint.grid(alpha=0.25)
    joint.legend(loc="upper left", fontsize=8)
    joint.set_title("5. Right J5 angle — planned, sent, and measured")

    tracking = figure.add_subplot(grid[2, 1])
    tracking_error_mrad = (trace["actual"] - trace["command"]) * 1000.0
    tracking.plot(
        trace["time"],
        tracking_error_mrad,
        color=ERROR_COLOR,
        linewidth=0.9,
        label="Measured q − sent command q",
    )
    tracking.axhline(0.0, color=NEUTRAL_COLOR, linewidth=0.8, alpha=0.6)
    tracking.set_xlabel("Path execution time (s)")
    tracking.set_ylabel("Right J5 tracking error (mrad)")
    tracking.grid(alpha=0.25)
    tracking.legend(loc="upper right", fontsize=8)
    tracking.set_title("6. Right J5 tracking error — closer to zero is better")
    tracking.text(
        0.02,
        0.96,
        f"RMS {np.sqrt(np.mean(tracking_error_mrad ** 2)):.2f} mrad   Peak {np.max(np.abs(tracking_error_mrad)):.2f} mrad",
        transform=tracking.transAxes,
        va="top",
        color=ERROR_COLOR,
    )

    title = f"Yunyi right-arm {str(data['motion']).upper()} at {data['speed_percent']:.0f}%"
    color_key = (
        "Blue = planned   Green = sent command   Orange = measured feedback   "
        "Red = error   Purple = orientation error"
    )
    figure.suptitle(f"{title}\n{color_key}", fontsize=15, fontweight="bold")
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180)
    plt.close(figure)

    return {
        "actual_samples": float(len(actual)),
        "xyz_deviation_mean_mm": float(np.mean(deviation) * 1000.0),
        "xyz_deviation_p95_mm": float(np.percentile(deviation, 95) * 1000.0),
        "xyz_deviation_max_mm": float(np.max(deviation) * 1000.0),
        "orientation_error_p95_deg": float(np.percentile(orientation_deg, 95)),
        "orientation_error_max_deg": float(np.max(orientation_deg)),
        "j5_tracking_error_rms_mrad": float(
            np.sqrt(np.mean((trace["actual"] - trace["command"]) ** 2)) * 1000.0
        ),
        "j5_tracking_error_peak_mrad": float(
            np.max(np.abs(trace["actual"] - trace["command"])) * 1000.0
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", type=Path, required=True)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    metrics = plot_one(args.json, args.trace, args.output)
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()

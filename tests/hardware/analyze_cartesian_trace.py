#!/usr/bin/env python3
"""Summarize planned/reference/feedback reversals in a native control trace."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


ROLES = [f"r-joint{index}" for index in range(1, 8)]


def finite_values(rows: list[dict[str, str]], column: str) -> list[float]:
    values = [float(row[column]) for row in rows]
    return [value for value in values if math.isfinite(value)]


def reversals(values: list[float], threshold: float) -> int:
    active = [1 if value > threshold else -1 for value in values if abs(value) > threshold]
    return sum(a != b for a, b in zip(active, active[1:]))


def highpass_peak_to_peak(values: list[float], half_window: int = 25) -> float:
    if len(values) <= 2 * half_window:
        return 0.0
    prefix = [0.0]
    for value in values:
        prefix.append(prefix[-1] + value)
    filtered = []
    for index in range(half_window, len(values) - half_window):
        first = index - half_window
        last = index + half_window + 1
        average = (prefix[last] - prefix[first]) / (last - first)
        filtered.append(values[index] - average)
    return max(filtered) - min(filtered)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--trajectory-id", type=int, default=1)
    args = parser.parse_args()
    with args.trace.open(newline="") as source:
        all_rows = list(csv.DictReader(source))
    rows = [
        row for row in all_rows
        if int(row.get("motion_id", row.get("trajectory_id", "0"))) ==
        args.trajectory_id
        and int(row["planned_valid_mask"]) != 0
    ]
    if not rows:
        raise SystemExit("trace has no valid planned trajectory rows")
    timestamps = [int(row["timestamp_ns"]) for row in rows]
    result: dict[str, object] = {
        "trace": str(args.trace),
        "samples": len(rows),
        "duration_s": (timestamps[-1] - timestamps[0]) / 1e9,
        "sample_hz": (len(rows) - 1) * 1e9 / (timestamps[-1] - timestamps[0]),
        "progress_range": [float(rows[0]["progress"]), float(rows[-1]["progress"])],
        "joints": [],
    }
    joints: list[dict[str, object]] = []
    for role in ROLES:
        planned_q = finite_values(rows, f"planned_q_{role}")
        planned_dq = finite_values(rows, f"planned_dq_{role}")
        command_q = finite_values(rows, f"command_q_{role}")
        actual_q = finite_values(rows, f"actual_q_{role}")
        actual_dq = finite_values(rows, f"actual_dq_{role}")
        command_steps = [b - a for a, b in zip(command_q, command_q[1:])]
        tracking_error = [actual - command for actual, command in zip(actual_q, command_q)]
        planned_acceleration = [
            (b - a) / max(1e-9, (timestamps[i + 1] - timestamps[i]) / 1e9)
            for i, (a, b) in enumerate(zip(planned_dq, planned_dq[1:]))
        ]
        joints.append({
            "role": role,
            "planned_dq_reversals_gt_0_01": reversals(planned_dq, 0.01),
            "actual_dq_reversals_gt_0_01": reversals(actual_dq, 0.01),
            "command_step_reversals_gt_2e_5": reversals(command_steps, 2e-5),
            "planned_peak_abs_dq": max(abs(value) for value in planned_dq),
            "actual_peak_abs_dq": max(abs(value) for value in actual_dq),
            "peak_abs_command_step": max(abs(value) for value in command_steps),
            "peak_abs_planned_acceleration": max(abs(value) for value in planned_acceleration),
            "planned_q_range": max(planned_q) - min(planned_q),
            "actual_q_range": max(actual_q) - min(actual_q),
            "actual_q_highpass_10hz_peak_to_peak": highpass_peak_to_peak(actual_q),
            "tracking_error_highpass_10hz_peak_to_peak": highpass_peak_to_peak(tracking_error),
            "peak_abs_tracking_error": max(abs(value) for value in tracking_error),
        })
    result["joints"] = joints
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()

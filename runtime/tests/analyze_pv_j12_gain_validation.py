#!/usr/bin/env python3
"""Analyze J1/J2 PV gain validation traces with motion and hold metrics."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np


JOINTS = (1, 2)


def highpass(values: np.ndarray, window: int = 51) -> tuple[float, float]:
    if len(values) < window:
        return 0.0, 0.0
    radius = window // 2
    smooth = np.convolve(values, np.ones(window) / window, mode="valid")
    residual = values[radius : len(values) - radius] - smooth
    return float(np.ptp(residual)), float(np.sqrt(np.mean(residual**2)))


def reversals(values: np.ndarray, threshold: float = 0.01) -> int:
    signs = np.sign(values[np.abs(values) > threshold])
    return int(np.sum(signs[1:] != signs[:-1])) if len(signs) > 1 else 0


def motion_start(command: np.ndarray, end: int, target: np.ndarray) -> int:
    """Find departure from the last non-target stable reference plateau."""
    delta = np.zeros(end)
    delta[1:] = np.max(np.abs(command[1:end] - command[: end - 1]), axis=1)
    stable = delta <= 1.0e-7
    runs: list[tuple[int, int]] = []
    first: int | None = None
    for index, value in enumerate(stable):
        if value and first is None:
            first = index
        if first is not None and (not value or index == end - 1):
            last = index - 1 if not value else index
            plateau = np.median(command[first : last + 1], axis=0)
            if last - first >= 50 and np.max(np.abs(plateau - target)) > 0.01:
                runs.append((first, last))
            first = None
    if not runs:
        raise RuntimeError("no stable source-reference plateau before motion")
    return runs[-1][1] + 1


def load_trace(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    timestamp = np.asarray([int(row["timestamp_ns"]) for row in rows])
    command = np.asarray(
        [
            [float(row[f"command_q_l-joint{joint}"]) for joint in range(1, 8)]
            for row in rows
        ]
    )
    actual = np.asarray(
        [
            [float(row[f"actual_q_l-joint{joint}"]) for joint in range(1, 8)]
            for row in rows
        ]
    )
    velocity = np.asarray(
        [
            [float(row[f"actual_dq_l-joint{joint}"]) for joint in range(1, 8)]
            for row in rows
        ]
    )
    return timestamp, command, actual, velocity


def load_holds(path: Path) -> dict[tuple[str, int], list[dict[str, str]]]:
    holds: dict[tuple[str, int], list[dict[str, str]]] = defaultdict(list)
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            holds[(row["profile"], int(row["point_index"]))].append(row)
    return holds


def analyze(args: argparse.Namespace) -> dict[str, object]:
    timestamp, command, actual, velocity = load_trace(args.control_trace)
    holds = load_holds(args.hold_samples)
    legs: list[dict[str, object]] = []
    profile_order: list[str] = []
    for profile, _ in holds:
        if profile not in profile_order:
            profile_order.append(profile)

    for (profile, point), samples in holds.items():
        hold_start_ns = int(samples[0]["timestamp_ns"])
        hold_end_ns = int(samples[-1]["timestamp_ns"])
        hold_start = int(np.searchsorted(timestamp, hold_start_ns))
        hold_end = int(np.searchsorted(timestamp, hold_end_ns, side="right"))
        target = np.median(command[hold_start:hold_end], axis=0)
        start = motion_start(command, hold_start, target)
        initial = command[start - 1]
        motion = slice(start, hold_start)
        hold = slice(hold_start, hold_end)
        result: dict[str, object] = {
            "profile": profile,
            "point": point,
            "settled_s": (timestamp[hold_start - 1] - timestamp[start]) * 1.0e-9,
        }
        for joint in JOINTS:
            index = joint - 1
            tracking = actual[motion, index] - command[motion, index]
            highpass_p2p, highpass_rms = highpass(actual[motion, index])
            direction = 1.0 if target[index] - initial[index] >= 0.0 else -1.0
            prefix = f"j{joint}_"
            result[prefix + "endpoint_abs_mrad"] = (
                abs(actual[hold_start, index] - target[index]) * 1000.0
            )
            result[prefix + "tracking_rms_mrad"] = (
                float(np.sqrt(np.mean(tracking**2))) * 1000.0
            )
            result[prefix + "tracking_peak_mrad"] = (
                float(np.max(np.abs(tracking))) * 1000.0
            )
            result[prefix + "actual_hp_p2p_deg"] = math.degrees(highpass_p2p)
            result[prefix + "actual_hp_rms_mrad"] = highpass_rms * 1000.0
            result[prefix + "peak_velocity_rad_s"] = float(
                np.max(np.abs(velocity[motion, index]))
            )
            result[prefix + "overshoot_mrad"] = (
                max(
                    0.0,
                    float(
                        np.max(direction * (actual[motion, index] - target[index]))
                    ),
                )
                * 1000.0
            )
            result[prefix + "velocity_reversals"] = reversals(
                velocity[motion, index]
            )
            result[prefix + "hold_p2p_mrad"] = (
                float(np.ptp(actual[hold, index])) * 1000.0
            )
            result[prefix + "hold_rms_mrad"] = (
                float(np.std(actual[hold, index])) * 1000.0
            )
            result[prefix + "hold_peak_velocity_rad_s"] = float(
                np.max(np.abs(velocity[hold, index]))
            )
        legs.append(result)

    profiles: list[dict[str, object]] = []
    for profile in profile_order:
        selected = [leg for leg in legs if leg["profile"] == profile]

        def values(suffix: str) -> list[float]:
            return [
                float(leg[f"j{joint}_{suffix}"])
                for leg in selected
                for joint in JOINTS
            ]

        profiles.append(
            {
                "profile": profile,
                "legs": len(selected),
                "settled_s_mean": float(
                    np.mean([float(leg["settled_s"]) for leg in selected])
                ),
                "endpoint_abs_error_mean_mrad": float(
                    np.mean(values("endpoint_abs_mrad"))
                ),
                "endpoint_abs_error_max_mrad": float(
                    np.max(values("endpoint_abs_mrad"))
                ),
                "tracking_rms_mean_mrad": float(
                    np.mean(values("tracking_rms_mrad"))
                ),
                "tracking_peak_max_mrad": float(
                    np.max(values("tracking_peak_mrad"))
                ),
                "actual_hp_p2p_mean_deg": float(
                    np.mean(values("actual_hp_p2p_deg"))
                ),
                "actual_hp_p2p_max_deg": float(
                    np.max(values("actual_hp_p2p_deg"))
                ),
                "actual_hp_rms_mean_mrad": float(
                    np.mean(values("actual_hp_rms_mrad"))
                ),
                "peak_velocity_max_rad_s": float(
                    np.max(values("peak_velocity_rad_s"))
                ),
                "overshoot_max_mrad": float(np.max(values("overshoot_mrad"))),
                "velocity_reversals_total": int(
                    np.sum(values("velocity_reversals"))
                ),
                "hold_p2p_max_mrad": float(np.max(values("hold_p2p_mrad"))),
                "hold_rms_max_mrad": float(np.max(values("hold_rms_mrad"))),
                "hold_peak_velocity_max_rad_s": float(
                    np.max(values("hold_peak_velocity_rad_s"))
                ),
            }
        )
    return {"profiles": profiles, "legs": legs}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("control_trace", type=Path)
    parser.add_argument("hold_samples", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = analyze(args)
    rendered = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.write_text(rendered)
    print(rendered, end="")


if __name__ == "__main__":
    main()

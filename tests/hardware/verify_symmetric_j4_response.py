#!/usr/bin/env python3
"""Compare left/right J4 motor-side PV response on real Yunyi hardware."""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

from arx_d_can import ArxDCanDualArm


@dataclass(frozen=True)
class Sample:
    elapsed_s: float
    left_q: float
    right_q: float
    left_dq: float
    right_dq: float


def collect_leg(
    robot: ArxDCanDualArm,
    left_target: list[float],
    right_target: list[float],
    timeout_s: float = 8.0,
    hold_s: float = 3.0,
) -> tuple[list[Sample], float]:
    started = time.monotonic()
    robot.set_joint_pv(left=left_target, right=right_target)
    samples: list[Sample] = []
    last_sequence = -1
    stable_since: float | None = None
    settled_at: float | None = None
    while time.monotonic() - started < timeout_s:
        health = robot.get_health()
        if health.safe_stopped or health.fault_reason:
            raise RuntimeError(f"unsafe Runtime state: {health}")
        state = robot.read_state()
        if state.sequence == last_sequence:
            time.sleep(0.0005)
            continue
        last_sequence = state.sequence
        elapsed = time.monotonic() - started
        sample = Sample(
            elapsed,
            state.left.positions[3],
            state.right.positions[3],
            state.left.arm.velocities[3],
            state.right.arm.velocities[3],
        )
        samples.append(sample)
        stable = (
            abs(sample.left_q - left_target[3]) <= 0.01
            and abs(sample.right_q - right_target[3]) <= 0.01
            and abs(sample.left_dq) <= 0.05
            and abs(sample.right_dq) <= 0.05
        )
        if stable:
            stable_since = stable_since or elapsed
            if settled_at is None and elapsed - stable_since >= 0.4:
                settled_at = elapsed
        else:
            stable_since = None
        if settled_at is not None and elapsed - settled_at >= hold_s:
            return samples, settled_at
    raise RuntimeError("symmetric J4 leg did not settle within the timeout")


def metrics(
    samples: list[Sample], settled_at: float, target: float,
    side: str, direction: float,
) -> dict[str, float | int]:
    q_name = f"{side}_q"
    dq_name = f"{side}_dq"
    positions = [getattr(sample, q_name) for sample in samples]
    velocities = [getattr(sample, dq_name) for sample in samples]
    hold = [sample for sample in samples if sample.elapsed_s >= settled_at]
    hold_q = [getattr(sample, q_name) for sample in hold]
    hold_dq = [getattr(sample, dq_name) for sample in hold]
    overshoot = (
        max(0.0, max(positions) - target)
        if direction > 0.0
        else max(0.0, target - min(positions))
    )
    signs = [1 if value > 0.01 else -1 if value < -0.01 else 0 for value in velocities]
    active_signs = [value for value in signs if value]
    reversals = sum(
        current != previous
        for previous, current in zip(active_signs, active_signs[1:])
    )
    mean_q = sum(hold_q) / len(hold_q)
    return {
        "samples": len(samples),
        "settled_s": settled_at,
        "endpoint_error_rad": positions[-1] - target,
        "overshoot_rad": overshoot,
        "peak_abs_velocity_rad_s": max(abs(value) for value in velocities),
        "velocity_direction_reversals": reversals,
        "hold_samples": len(hold),
        "hold_position_peak_to_peak_rad": max(hold_q) - min(hold_q),
        "hold_position_rms_about_mean_rad": math.sqrt(
            sum((value - mean_q) ** 2 for value in hold_q) / len(hold_q)
        ),
        "hold_peak_abs_velocity_rad_s": max(abs(value) for value in hold_dq),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--i-understand-both-arms-will-move", action="store_true")
    parser.add_argument("--delta-rad", type=float, default=0.1)
    parser.add_argument("--speed", type=float, default=20.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not args.i_understand_both_arms_will_move:
        parser.error("--i-understand-both-arms-will-move is required")
    if not 0.02 <= args.delta_rad <= 0.2:
        parser.error("--delta-rad must be within 0.02..0.2")

    robot = ArxDCanDualArm(control_mode="pv")
    connected = False
    result: dict[str, object] = {
        "speed_percent": args.speed,
        "delta_rad": args.delta_rad,
        "reference_side": "right",
        "measurement_scope": "motor-side encoder feedback",
    }
    try:
        robot.connect()
        connected = True
        robot.enable()
        robot.set_max_speed(args.speed)
        initial = robot.read_state()
        common = [
            0.5 * (left + right)
            for left, right in zip(
                initial.left.positions, initial.right.positions
            )
        ]
        left_initial = common.copy()
        right_initial = common.copy()
        collect_leg(robot, left_initial, right_initial, hold_s=0.5)

        left_inward = left_initial.copy()
        right_inward = right_initial.copy()
        left_inward[3] -= args.delta_rad
        right_inward[3] -= args.delta_rad
        inward_samples, inward_settled = collect_leg(
            robot, left_inward, right_inward
        )
        outward_samples, outward_settled = collect_leg(
            robot, left_initial, right_initial
        )
        result["initial_left_q"] = left_initial
        result["initial_right_q"] = right_initial
        result["inward"] = {
            "left": metrics(
                inward_samples, inward_settled, left_inward[3], "left", -1.0
            ),
            "right": metrics(
                inward_samples, inward_settled, right_inward[3], "right", -1.0
            ),
        }
        result["outward"] = {
            "left": metrics(
                outward_samples, outward_settled, left_initial[3], "left", 1.0
            ),
            "right": metrics(
                outward_samples, outward_settled, right_initial[3], "right", 1.0
            ),
        }
        result["health"] = str(robot.get_health())
        before_disable = robot.read_state()
        result["before_disable_j4"] = {
            "left": before_disable.left.positions[3],
            "right": before_disable.right.positions[3],
        }
    finally:
        if connected:
            robot.disconnect()

    time.sleep(1.0)
    verifier = ArxDCanDualArm(control_mode="pv")
    try:
        verifier.connect()
        disabled = verifier.read_state()
        health = verifier.get_health()
        result["after_disable_j4"] = {
            "left": disabled.left.positions[3],
            "right": disabled.right.positions[3],
        }
        result["passive_sag_rad"] = {
            "left": disabled.left.positions[3]
            - float(result["before_disable_j4"]["left"]),
            "right": disabled.right.positions[3]
            - float(result["before_disable_j4"]["right"]),
        }
        result["disconnect_verified"] = bool(
            health.disable_confirmed
            and not any(disabled.left.arm.enabled)
            and not any(disabled.right.arm.enabled)
        )
    finally:
        verifier.disconnect()

    print(json.dumps(result, indent=2), flush=True)
    if args.output:
        args.output.write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    main()

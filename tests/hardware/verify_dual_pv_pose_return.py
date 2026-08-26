#!/usr/bin/env python3
"""Exercise a mirrored Yunyi dual-arm PV target and return-to-zero on hardware."""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

from arx_d_can import ArxDCanDualArm


JOINT_NAMES = tuple(
    [f"left/l-joint{i}" for i in range(1, 8)]
    + [f"right/r-joint{i}" for i in range(1, 8)]
)
POSITION_TOLERANCE_RAD = 0.01
VELOCITY_TOLERANCE_RAD_S = 0.05
SETTLE_CONFIRM_S = 0.4


@dataclass(frozen=True)
class Sample:
    elapsed_s: float
    q: tuple[float, ...]
    dq: tuple[float, ...]


def read_q_dq(state: object) -> tuple[tuple[float, ...], tuple[float, ...]]:
    q = tuple(state.left.positions) + tuple(state.right.positions)
    dq = tuple(state.left.arm.velocities) + tuple(state.right.arm.velocities)
    return q, dq


def collect_leg(
    robot: ArxDCanDualArm,
    left_target: list[float],
    right_target: list[float],
    *,
    timeout_s: float = 15.0,
    hold_s: float = 3.0,
) -> tuple[list[Sample], float, tuple[float, ...]]:
    initial_state = robot.read_state()
    initial_q, _ = read_q_dq(initial_state)
    started = time.monotonic()
    robot.set_joint_pv(left=left_target, right=right_target)
    target = tuple(left_target + right_target)
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
            time.sleep(0.0002)
            continue
        last_sequence = state.sequence
        elapsed = time.monotonic() - started
        q, dq = read_q_dq(state)
        samples.append(Sample(elapsed, q, dq))

        stable = all(
            abs(position - goal) <= POSITION_TOLERANCE_RAD
            and abs(velocity) <= VELOCITY_TOLERANCE_RAD_S
            for position, velocity, goal in zip(q, dq, target)
        )
        if stable:
            stable_since = elapsed if stable_since is None else stable_since
            if settled_at is None and elapsed - stable_since >= SETTLE_CONFIRM_S:
                settled_at = elapsed
        else:
            stable_since = None
        if settled_at is not None and elapsed - settled_at >= hold_s:
            return samples, settled_at, initial_q

    raise RuntimeError("PV leg did not settle within timeout")


def direction_reversals(values: list[float], threshold: float = 0.01) -> int:
    signs = [1 if value > threshold else -1 if value < -threshold else 0 for value in values]
    active = [value for value in signs if value]
    return sum(current != previous for previous, current in zip(active, active[1:]))


def crossing_time(
    samples: list[Sample], joint: int, start: float, target: float, fraction: float
) -> float | None:
    distance = target - start
    if abs(distance) < 0.02:
        return None
    for sample in samples:
        progress = (sample.q[joint] - start) / distance
        if progress >= fraction:
            return sample.elapsed_s
    return None


def highpass_metrics(values: list[float], window: int = 51) -> tuple[float, float]:
    if len(values) < window:
        return 0.0, 0.0
    radius = window // 2
    residuals: list[float] = []
    running = sum(values[:window])
    for center in range(radius, len(values) - radius):
        if center > radius:
            running += values[center + radius] - values[center - radius - 1]
        residuals.append(values[center] - running / window)
    mean_square = sum(value * value for value in residuals) / len(residuals)
    return max(residuals) - min(residuals), math.sqrt(mean_square)


def joint_metrics(
    samples: list[Sample], settled_at: float, initial_q: tuple[float, ...], target: tuple[float, ...]
) -> dict[str, dict[str, float | int | None]]:
    motion = [sample for sample in samples if sample.elapsed_s < settled_at]
    hold = [sample for sample in samples if sample.elapsed_s >= settled_at]
    result: dict[str, dict[str, float | int | None]] = {}
    for joint, name in enumerate(JOINT_NAMES):
        motion_q = [sample.q[joint] for sample in motion]
        motion_dq = [sample.dq[joint] for sample in motion]
        hold_q = [sample.q[joint] for sample in hold]
        hold_dq = [sample.dq[joint] for sample in hold]
        distance = target[joint] - initial_q[joint]
        direction = 1.0 if distance >= 0.0 else -1.0
        overshoot = max(
            0.0,
            max(direction * (value - target[joint]) for value in motion_q),
        )
        hp_p2p, hp_rms = highpass_metrics(motion_q)
        hold_mean = sum(hold_q) / len(hold_q)
        t10 = crossing_time(motion, joint, initial_q[joint], target[joint], 0.1)
        t50 = crossing_time(motion, joint, initial_q[joint], target[joint], 0.5)
        t90 = crossing_time(motion, joint, initial_q[joint], target[joint], 0.9)
        midpath = (
            [sample for sample in motion if t10 <= sample.elapsed_s <= t90]
            if t10 is not None and t90 is not None
            else []
        )
        late = (
            [sample for sample in motion if sample.elapsed_s > t90]
            if t90 is not None
            else []
        )
        midpath_opposite = [
            -direction * sample.dq[joint]
            for sample in midpath
            if direction * sample.dq[joint] < -0.01
        ]
        result[name] = {
            "start_rad": initial_q[joint],
            "target_rad": target[joint],
            "travel_rad": distance,
            "t10_s": t10,
            "t50_s": t50,
            "t90_s": t90,
            "endpoint_error_rad": samples[-1].q[joint] - target[joint],
            "overshoot_rad": overshoot,
            "motion_peak_abs_velocity_rad_s": max(abs(value) for value in motion_dq),
            "motion_velocity_direction_reversals": direction_reversals(motion_dq),
            "midpath_velocity_direction_reversals": direction_reversals(
                [sample.dq[joint] for sample in midpath]
            ),
            "midpath_opposite_velocity_samples": len(midpath_opposite),
            "midpath_peak_opposite_velocity_rad_s": (
                max(midpath_opposite) if midpath_opposite else 0.0
            ),
            "late_velocity_direction_reversals": direction_reversals(
                [sample.dq[joint] for sample in late]
            ),
            "motion_position_highpass_p2p_rad": hp_p2p,
            "motion_position_highpass_rms_rad": hp_rms,
            "hold_position_peak_to_peak_rad": max(hold_q) - min(hold_q),
            "hold_position_rms_rad": math.sqrt(
                sum((value - hold_mean) ** 2 for value in hold_q) / len(hold_q)
            ),
            "hold_peak_abs_velocity_rad_s": max(abs(value) for value in hold_dq),
            "hold_velocity_direction_reversals": direction_reversals(hold_dq),
        }
    return result


def leg_result(
    samples: list[Sample], settled_at: float, initial_q: tuple[float, ...], target: list[float]
) -> dict[str, object]:
    duration = samples[-1].elapsed_s - samples[0].elapsed_s
    return {
        "sample_count": len(samples),
        "sample_rate_hz": (len(samples) - 1) / duration,
        "settled_s": settled_at,
        "position_tolerance_rad": POSITION_TOLERANCE_RAD,
        "velocity_tolerance_rad_s": VELOCITY_TOLERANCE_RAD_S,
        "settle_confirmation_s": SETTLE_CONFIRM_S,
        "joints": joint_metrics(samples, settled_at, initial_q, tuple(target)),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--i-understand-both-arms-will-move", action="store_true")
    parser.add_argument("--speed", type=float, default=50.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not args.i_understand_both_arms_will_move:
        parser.error("--i-understand-both-arms-will-move is required")

    left_target = [-math.pi / 4, math.pi / 4, 0.0, math.pi / 2, 0.0, 0.0, 0.0]
    right_target = [-math.pi / 4, -math.pi / 4, 0.0, math.pi / 2, 0.0, 0.0, 0.0]
    zero = [0.0] * 7
    result: dict[str, object] = {
        "control_mode": "pv",
        "speed_percent": args.speed,
        "left_target_rad": left_target,
        "right_target_rad": right_target,
        "reference_side": "right",
        "measurement_scope": "motor-side encoder feedback",
    }

    robot = ArxDCanDualArm(control_mode="pv")
    connected = False
    try:
        robot.connect()
        connected = True
        before_enable = robot.read_state()
        result["before_enable"] = {
            "left_q": list(before_enable.left.positions),
            "right_q": list(before_enable.right.positions),
            "left_enabled": list(before_enable.left.arm.enabled),
            "right_enabled": list(before_enable.right.arm.enabled),
        }
        robot.enable()
        robot.set_max_speed(2.0 * args.speed / 100.0)
        robot.set_max_acceleration(4.0)

        outbound, outbound_settled, outbound_initial = collect_leg(
            robot, left_target, right_target
        )
        result["outbound"] = leg_result(
            outbound, outbound_settled, outbound_initial, left_target + right_target
        )

        returned, returned_settled, returned_initial = collect_leg(robot, zero, zero)
        result["return_to_zero"] = leg_result(
            returned, returned_settled, returned_initial, zero + zero
        )
        result["health_before_disconnect"] = str(robot.get_health())
    finally:
        if connected:
            robot.disconnect()

    time.sleep(0.5)
    verifier = ArxDCanDualArm(control_mode="pv")
    try:
        verifier.connect()
        state = verifier.read_state()
        health = verifier.get_health()
        result["after_disconnect"] = {
            "left_q": list(state.left.positions),
            "right_q": list(state.right.positions),
            "left_enabled": list(state.left.arm.enabled),
            "right_enabled": list(state.right.arm.enabled),
            "disable_confirmed": health.disable_confirmed,
            "fault_reason": health.fault_reason,
        }
    finally:
        verifier.disconnect()

    rendered = json.dumps(result, indent=2)
    print(rendered, flush=True)
    if args.output:
        args.output.write_text(rendered + "\n")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""真机验证 Yunyi 普通 PV/MIT 双臂相对关节跟随。"""
from __future__ import annotations

import argparse
import json
import math
import time

from arx_d_can import ArxDCanDualArm, BimanualFollowPhase


SAFE_Q = (0.0, 0.0, 0.0, math.pi / 2.0, 0.0, 0.0, 0.0)


def arm_positions(robot: ArxDCanDualArm, side: str) -> tuple[float, ...]:
    state = robot.read_state()
    source = state.left.arm if side == "left" else state.right.arm
    return tuple(source.positions)


def send_positions(
    robot: ArxDCanDualArm,
    mode: str,
    left: tuple[float, ...],
    right: tuple[float, ...],
    speed: float,
) -> None:
    if mode == "pv":
        robot.set_joint_pv(left=left, right=right)
    else:
        robot.set_joint_mit(left=left, right=right, velocity=speed)


def wait_stable(
    robot: ArxDCanDualArm,
    expected_left: tuple[float, ...],
    expected_right: tuple[float, ...],
    position_tolerance: float,
    timeout_s: float = 25.0,
) -> None:
    deadline = time.monotonic() + timeout_s
    stable_since: float | None = None
    last_error = math.inf
    last_speed = math.inf
    last_position_errors: tuple[float, ...] = ()
    while time.monotonic() < deadline:
        state = robot.read_state()
        positions = (*state.left.arm.positions, *state.right.arm.positions)
        velocities = (*state.left.arm.velocities, *state.right.arm.velocities)
        expected = (*expected_left, *expected_right)
        last_error = max(abs(a - b) for a, b in zip(positions, expected))
        last_position_errors = tuple(a - b for a, b in zip(positions, expected))
        last_speed = max(abs(value) for value in velocities)
        if last_error <= position_tolerance and last_speed <= 0.06:
            stable_since = stable_since or time.monotonic()
            if time.monotonic() - stable_since >= 0.30:
                return
        else:
            stable_since = None
        health = robot.get_health()
        if health.safe_stopped or health.fault_reason:
            raise RuntimeError(
                health.safety_reason or health.fault_reason or
                f"Runtime entered {health.state.name}"
            )
        time.sleep(0.005)
    raise RuntimeError(
        f"position did not settle: max_error={last_error:.6f} rad, "
        f"max_velocity={last_speed:.6f} rad/s, "
        f"position_errors={last_position_errors}"
    )


def run_mode(args: argparse.Namespace, mode: str) -> dict[str, object]:
    robot = ArxDCanDualArm(control_mode=mode, with_grippers=True)
    started = False
    try:
        robot.connect()
        robot.enable()
        if mode == "pv":
            robot.set_max_speed(args.speed)
        send_positions(robot, mode, SAFE_Q, SAFE_Q, args.speed)
        tolerance = 0.05 if mode == "pv" else 0.06
        wait_stable(robot, SAFE_Q, SAFE_Q, tolerance)

        baseline_left = arm_positions(robot, "left")
        baseline_right = arm_positions(robot, "right")
        robot.start_bimanual_follow(leader=args.leader)
        started = True
        status0 = robot.bimanual_follow_status
        if status0.phase is not BimanualFollowPhase.ACTIVE:
            raise RuntimeError("bimanual follow did not become ACTIVE immediately")

        leader_target = list(
            baseline_left if args.leader == "left" else baseline_right
        )
        leader_target[0] += math.radians(args.delta_deg)
        leader_target_tuple = tuple(leader_target)
        # Deliberately leave the follower argument at SAFE_Q. Runtime must
        # ignore it while follow is active and preserve the captured relation.
        left_command = leader_target_tuple if args.leader == "left" else SAFE_Q
        right_command = leader_target_tuple if args.leader == "right" else SAFE_Q
        send_positions(robot, mode, left_command, right_command, args.speed)

        expected_left = list(baseline_left)
        expected_right = list(baseline_right)
        expected_left[0] += math.radians(args.delta_deg)
        expected_right[0] += math.radians(args.delta_deg)
        wait_stable(
            robot, tuple(expected_left), tuple(expected_right), tolerance
        )

        samples: list[dict[str, object]] = []
        sample_started = time.monotonic()
        while time.monotonic() - sample_started < args.sample_seconds:
            state = robot.read_state()
            status = robot.bimanual_follow_status
            samples.append({
                "t": time.monotonic() - sample_started,
                "left": list(state.left.arm.positions),
                "right": list(state.right.arm.positions),
                "tracking_error": status.max_tracking_error,
                "cycles": status.control_cycles,
            })
            time.sleep(0.002)

        send_positions(robot, mode, SAFE_Q, SAFE_Q, args.speed)
        wait_stable(robot, baseline_left, baseline_right, tolerance)
        robot.stop_bimanual_follow()
        started = False
        if robot.bimanual_follow_status.active:
            raise RuntimeError("bimanual follow remained active after stop")

        first = samples[0]
        last = samples[-1]
        duration = float(last["t"]) - float(first["t"])
        cycles = int(last["cycles"]) - int(first["cycles"])
        final = robot.read_state()
        health = robot.get_health()
        return {
            "mode": mode,
            "leader": args.leader,
            "delta_deg": args.delta_deg,
            "native_control_hz": cycles / duration,
            "samples": len(samples),
            "maximum_reported_tracking_error_rad": max(
                float(item["tracking_error"]) for item in samples
            ),
            "left_j1_delta_rad":
                float(samples[-1]["left"][0]) - baseline_left[0],
            "right_j1_delta_rad":
                float(samples[-1]["right"][0]) - baseline_right[0],
            "stop_hold_left": list(final.left.arm.positions),
            "stop_hold_right": list(final.right.arm.positions),
            "health_state": health.state.name,
            "fault_reason": health.fault_reason,
        }
    finally:
        if robot.connected and started:
            try:
                robot.stop_bimanual_follow()
            except Exception:
                pass
        if robot.connected:
            robot.disconnect()


def main(args: argparse.Namespace) -> None:
    modes = ("pv", "mit") if args.mode == "both" else (args.mode,)
    results = [run_mode(args, mode) for mode in modes]
    print(json.dumps(results, ensure_ascii=False, indent=2))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("pv", "mit", "both"), default="both")
    parser.add_argument("--leader", choices=("left", "right"), default="right")
    parser.add_argument("--speed", type=float, default=30.0)
    parser.add_argument("--delta-deg", type=float, default=8.0)
    parser.add_argument("--sample-seconds", type=float, default=2.0)
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())

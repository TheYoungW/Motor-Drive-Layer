#!/usr/bin/env python3
"""True-hardware verification for native approach-PTP Cartesian paths."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

from arx_d_can import ArxDCanDualArm, CartesianMotionState


START = (0.403537, 0.231892, 0.381638, 0.0, -1.570796, 0.0)
LINEAR_END = (0.403537, 0.331892, 0.381638, 0.0, -1.570796, 0.0)
CIRCULAR_VIA = (0.423537, 0.281892, 0.381638, 0.0, -1.570796, 0.0)
CIRCULAR_END = LINEAR_END


def quaternion_from_rpy(pose: list[float] | tuple[float, ...]) -> tuple[float, ...]:
    roll, pitch, yaw = pose[3:]
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def pose_error(actual: list[float], target: tuple[float, ...]) -> tuple[float, float]:
    position = math.sqrt(sum((actual[index] - target[index]) ** 2 for index in range(3)))
    actual_q = quaternion_from_rpy(actual)
    target_q = quaternion_from_rpy(target)
    dot = min(1.0, abs(sum(left * right for left, right in zip(actual_q, target_q))))
    orientation = 2.0 * math.acos(dot)
    return position, orientation


def wait_joint_target(
    robot: ArxDCanDualArm, target: tuple[float, ...], timeout_s: float
) -> None:
    deadline = time.monotonic() + timeout_s
    stable = 0
    while time.monotonic() < deadline:
        state = robot.read_state()
        position_error = max(
            abs(actual - expected)
            for actual, expected in zip(
                (*state.left.positions, *state.right.positions), target
            )
        )
        speed = max(
            abs(value)
            for value in (*state.left.arm.velocities, *state.right.arm.velocities)
        )
        stable = stable + 1 if position_error <= 0.02 and speed <= 0.05 else 0
        if stable >= 20:
            return
        time.sleep(0.01)
    raise RuntimeError("joint target did not settle")


def wait_motion(
    robot: ArxDCanDualArm,
    motion_id: int,
    declared_start: tuple[float, ...],
    final_pose: tuple[float, ...],
    timeout_s: float = 30.0,
) -> dict[str, object]:
    deadline = time.monotonic() + timeout_s
    start_seen_at: float | None = None
    start_seen_progress: float | None = None
    submitted_at = time.monotonic()
    samples = 0
    while time.monotonic() < deadline:
        status = robot.get_cartesian_motion_status(motion_id)
        state = robot.read_state()
        pose = robot.get_pose("left")
        samples += 1
        start_position_error, start_orientation_error = pose_error(pose, declared_start)
        actual_speed = max(abs(value) for value in state.left.arm.velocities)
        if (
            start_seen_at is None
            and start_position_error <= 0.005
            and start_orientation_error <= 0.035
            and actual_speed <= 0.05
        ):
            start_seen_at = time.monotonic() - submitted_at
            start_seen_progress = status.progress
        if status.state is CartesianMotionState.COMPLETED:
            final_actual = robot.get_pose("left")
            final_error = pose_error(final_actual, final_pose)
            return {
                "motion_id": motion_id,
                "duration_s": time.monotonic() - submitted_at,
                "samples": samples,
                "start_seen_at_s": start_seen_at,
                "start_seen_progress": start_seen_progress,
                "final_pose": final_actual,
                "final_position_error_m": final_error[0],
                "final_orientation_error_rad": final_error[1],
                "status": status.state.value,
            }
        if status.state in (CartesianMotionState.CANCELLED, CartesianMotionState.FAULT):
            raise RuntimeError(f"motion {motion_id} ended as {status.state.value}: {status.error}")
        time.sleep(0.004)
    raise TimeoutError(f"motion {motion_id} did not complete")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--i-understand-robot-will-move",
        action="store_true",
        help="required acknowledgement before connecting and enabling the robot",
    )
    args = parser.parse_args()
    if not args.i_understand_robot_will_move:
        parser.error("--i-understand-robot-will-move is required")

    robot = ArxDCanDualArm(control_mode="pv")
    results: dict[str, object] = {}
    connected = False
    try:
        robot.connect()
        connected = True
        robot.enable()
        robot.set_max_acceleration(4.0)

        home = (0.0,) * 14
        robot.set_joint_pv(left=home[:7], right=home[7:], velocity=50.0)
        wait_joint_target(robot, home, 10.0)
        initial_pose = robot.get_pose("left")
        initial_error = pose_error(initial_pose, START)
        results["initial_pose"] = initial_pose
        results["initial_start_position_error_m"] = initial_error[0]
        results["initial_start_orientation_error_rad"] = initial_error[1]

        linear_id = robot.move_linear(
            side="left", start_pose=START, end_pose=LINEAR_END, speed_percent=20.0
        )
        results["linear"] = wait_motion(robot, linear_id, START, LINEAR_END)

        circular_id = robot.move_circular(
            side="left",
            start_pose=START,
            via_pose=CIRCULAR_VIA,
            end_pose=CIRCULAR_END,
            speed_percent=20.0,
        )
        results["circular"] = wait_motion(robot, circular_id, START, CIRCULAR_END)

        robot.set_joint_pv(left=home[:7], right=home[7:], velocity=50.0)
        wait_joint_target(robot, home, 10.0)
        cancel_id = robot.move_linear(
            side="left", start_pose=START, end_pose=LINEAR_END, speed_percent=20.0
        )
        time.sleep(0.5)
        before_cancel = robot.read_state().left.positions
        robot.cancel_cartesian_motion()
        cancel_status = robot.get_cartesian_motion_status(cancel_id)
        settle_deadline = time.monotonic() + 5.0
        stable = 0
        while time.monotonic() < settle_deadline:
            state = robot.read_state()
            stable = stable + 1 if max(
                abs(value) for value in state.left.arm.velocities
            ) <= 0.05 else 0
            if stable >= 20:
                break
            time.sleep(0.01)
        hold_samples = []
        hold_deadline = time.monotonic() + 1.0
        while time.monotonic() < hold_deadline:
            hold_samples.append(robot.read_state().left.positions)
            time.sleep(0.004)
        after_cancel = hold_samples[-1]
        hold_range = max(
            max(sample[index] for sample in hold_samples)
            - min(sample[index] for sample in hold_samples)
            for index in range(7)
        )
        results["cancel"] = {
            "motion_id": cancel_id,
            "state": cancel_status.state.value,
            "catch_up_after_cancel_rad": max(
                abs(after - before)
                for after, before in zip(after_cancel, before_cancel)
            ),
            "settled_hold_range_rad": hold_range,
        }
        health = robot.get_health()
        results["health_state"] = health.state.name
        results["health_error"] = (
            health.last_operation_error or health.safety_reason or health.fault_reason
        )
        print(json.dumps(results, ensure_ascii=False, indent=2), flush=True)
        output = Path("/home/ubuntu/motorbridge/tests/results/yunyi_composite_cartesian_2026-08-25.json")
        output.write_text(json.dumps(results, ensure_ascii=False, indent=2) + "\n")
    finally:
        if connected:
            robot.disconnect()


if __name__ == "__main__":
    main()

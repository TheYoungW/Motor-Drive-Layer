#!/usr/bin/env python3
"""Verify dual-arm set_pose as the first PV command after enable."""

from __future__ import annotations

import argparse
import math
import time

from arx_d_can import ArxDCanDualArm


LEFT_TARGET = (0.403537, 0.231892, 0.381638, 0.0, -1.570796, 0.0)
RIGHT_TARGET = (0.403537, -0.231889, 0.381639, 0.0, -1.570796, 0.0)


def pose_error(actual: list[float], target: tuple[float, ...]) -> tuple[float, float]:
    position = math.sqrt(sum((actual[i] - target[i]) ** 2 for i in range(3)))
    def rotation(values: list[float] | tuple[float, ...]) -> tuple[float, ...]:
        roll, pitch, yaw = values[3:]
        sr, cr = math.sin(roll), math.cos(roll)
        sp, cp = math.sin(pitch), math.cos(pitch)
        sy, cy = math.sin(yaw), math.cos(yaw)
        return (
            cy * cp,
            cy * sp * sr - sy * cr,
            cy * sp * cr + sy * sr,
            sy * cp,
            sy * sp * sr + cy * cr,
            sy * sp * cr - cy * sr,
            -sp,
            cp * sr,
            cp * cr,
        )

    actual_rotation = rotation(actual)
    target_rotation = rotation(target)
    trace = sum(a * b for a, b in zip(actual_rotation, target_rotation))
    orientation = math.acos(max(-1.0, min(1.0, 0.5 * (trace - 1.0))))
    return position, orientation


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--i-understand-robot-will-move", action="store_true")
    parser.add_argument("--speed", type=float, default=50.0)
    args = parser.parse_args()
    if not args.i_understand_robot_will_move:
        parser.error("--i-understand-robot-will-move is required")

    robot = ArxDCanDualArm(control_mode="pv")
    connected = False
    try:
        robot.connect()
        connected = True
        robot.enable()
        started = time.monotonic()
        robot.set_pose(
            left_target_pose=LEFT_TARGET,
            right_target_pose=RIGHT_TARGET,
            speed_percent=args.speed,
        )
        installed = time.monotonic()
        print(f"set_pose_call_s={installed - started:.6f}", flush=True)

        deadline = started + 10.0
        stable_samples = 0
        while time.monotonic() < deadline:
            health = robot.get_health()
            if health.safe_stopped or health.fault_reason:
                raise RuntimeError(f"Runtime safety transition: {health}")
            state = robot.read_state()
            left_error = pose_error(robot.get_pose("left"), LEFT_TARGET)
            right_error = pose_error(robot.get_pose("right"), RIGHT_TARGET)
            velocity = max(
                abs(value)
                for value in (
                    *state.left.arm.velocities,
                    *state.right.arm.velocities,
                )
            )
            settled = (
                left_error[0] <= 0.005
                and left_error[1] <= 0.035
                and right_error[0] <= 0.005
                and right_error[1] <= 0.035
                and velocity <= 0.05
            )
            stable_samples = stable_samples + 1 if settled else 0
            if stable_samples >= 20:
                break
            time.sleep(0.01)
        state = robot.read_state()
        print(f"settled_s={time.monotonic() - started:.6f}", flush=True)
        print(f"left_q={list(state.left.positions)}", flush=True)
        print(f"right_q={list(state.right.positions)}", flush=True)
        print(f"left_pose={robot.get_pose('left')}", flush=True)
        print(f"right_pose={robot.get_pose('right')}", flush=True)
        print(f"health={robot.get_health()}", flush=True)
        if stable_samples < 20:
            raise RuntimeError("dual-arm set_pose did not settle within 10 seconds")
    finally:
        if connected:
            robot.disconnect()


if __name__ == "__main__":
    main()

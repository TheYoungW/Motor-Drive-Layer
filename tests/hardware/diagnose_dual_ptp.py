#!/usr/bin/env python3
"""Verify atomic dual-arm ordinary PV PTP on a Yunyi dual arm."""

from __future__ import annotations

import argparse
import math
import time

from arx_d_can import ArxDCanDualArm


LEFT_TARGET = (0.403537, 0.231892, 0.381638, 0.0, -1.570796, 0.0)
RIGHT_TARGET = (0.403537, -0.231889, 0.381639, 0.0, -1.570796, 0.0)


def pose_error(actual: list[float], target: tuple[float, ...]) -> tuple[float, float]:
    xyz = math.sqrt(sum((actual[index] - target[index]) ** 2 for index in range(3)))
    rpy = math.sqrt(sum((actual[index] - target[index]) ** 2 for index in range(3, 6)))
    return xyz, rpy


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
    connected = False
    try:
        robot.connect()
        connected = True
        robot.enable()
        robot.set_max_speed(50.0)
        home = (0.0,) * 7
        robot.set_joint_pv(left=home, right=home)
        home_deadline = time.monotonic() + 8.0
        stable = 0
        while time.monotonic() < home_deadline:
            state = robot.read_state()
            home_ready = (
                max(abs(value) for value in (*state.left.positions, *state.right.positions))
                <= 0.02
                and max(
                    abs(value)
                    for value in (
                        *state.left.arm.velocities,
                        *state.right.arm.velocities,
                    )
                )
                <= 0.05
            )
            stable = stable + 1 if home_ready else 0
            if stable >= 20:
                break
            time.sleep(0.01)
        if stable < 20:
            raise RuntimeError("dual arm did not settle at the zero test start")
        start = robot.read_state()
        print("start left q", list(start.left.positions), flush=True)
        print("start right q", list(start.right.positions), flush=True)
        print("start left pose", robot.get_pose("left"), flush=True)
        print("start right pose", robot.get_pose("right"), flush=True)

        submitted_at = time.monotonic()
        robot.move_pose(
            left_target_pose=LEFT_TARGET,
            right_target_pose=RIGHT_TARGET,
            speed_percent=50.0,
        )
        installed_at = time.monotonic()
        print("dual IK and atomic install s", installed_at - submitted_at, flush=True)

        samples: list[tuple[float, tuple[float, ...], tuple[float, ...]]] = []
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            state = robot.read_state()
            samples.append(
                (time.monotonic() - submitted_at, state.left.positions, state.right.positions)
            )
            time.sleep(0.002)

        final = robot.read_state()
        left_pose = robot.get_pose("left")
        right_pose = robot.get_pose("right")
        print("final left q", list(final.left.positions), flush=True)
        print("final right q", list(final.right.positions), flush=True)
        print("final left pose", left_pose, "error", pose_error(left_pose, LEFT_TARGET), flush=True)
        print("final right pose", right_pose, "error", pose_error(right_pose, RIGHT_TARGET), flush=True)
        left_ranges = [
            max(sample[1][index] for sample in samples) - min(sample[1][index] for sample in samples)
            for index in range(7)
        ]
        right_ranges = [
            max(sample[2][index] for sample in samples) - min(sample[2][index] for sample in samples)
            for index in range(7)
        ]
        target_q = (0.0, 0.0, 0.0, math.pi / 2.0, 0.0, 0.0, 0.0)
        arrival_times: list[float | None] = [None, None]
        for elapsed, left_q, right_q in samples:
            for side, q in enumerate((left_q, right_q)):
                if arrival_times[side] is None and max(
                    abs(q[index] - target_q[index]) for index in range(7)
                ) <= 0.02:
                    arrival_times[side] = elapsed
        print("left q ranges", left_ranges, flush=True)
        print("right q ranges", right_ranges, flush=True)
        print("left/right joint arrival s", arrival_times, flush=True)
        if all(value is not None for value in arrival_times):
            print("arrival difference s", abs(arrival_times[0] - arrival_times[1]), flush=True)
        print("health", robot.get_health(), flush=True)
    finally:
        if connected:
            robot.disconnect()


if __name__ == "__main__":
    main()

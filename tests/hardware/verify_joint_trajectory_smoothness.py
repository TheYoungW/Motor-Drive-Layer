#!/usr/bin/env python3
"""Measure one native 14-joint quintic PV trajectory on Yunyi hardware."""

from __future__ import annotations

import argparse
import ctypes
import json
import math
import time
from pathlib import Path

from arx_d_can import ArxDCanDualArm

from verify_cartesian_smoothness import temperatures


DOF = 7
PRODUCT_DOF = 14
MODE_PV = 1
INTERPOLATION_QUINTIC = 1
TRAJECTORY_RUNNING = 1
TRAJECTORY_COMPLETED = 2
TRAJECTORY_CANCELLED = 3
TRAJECTORY_FAULT = 4
START = [0.0, 0.0, 0.0, math.pi / 2.0, 0.0, 0.0, 0.0]
RIGHT_TARGET = [
    -math.pi / 4.0, -math.pi / 4.0, 0.0,
    math.pi / 2.0, 0.0, 0.0, 0.0,
]


class TrajectoryWaypoint(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("time_s", ctypes.c_double),
        ("left_positions", ctypes.c_float * DOF),
        ("right_positions", ctypes.c_float * DOF),
        ("left_velocities", ctypes.c_float * DOF),
        ("right_velocities", ctypes.c_float * DOF),
        ("left_accelerations", ctypes.c_float * DOF),
        ("right_accelerations", ctypes.c_float * DOF),
        ("velocity_valid_mask", ctypes.c_uint32),
        ("acceleration_valid_mask", ctypes.c_uint32),
    ]


class TrajectoryConfig(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("interpolation", ctypes.c_int32),
        ("control_mode", ctypes.c_int32),
        ("mit_kp", ctypes.c_float * PRODUCT_DOF),
        ("mit_kd", ctypes.c_float * PRODUCT_DOF),
        ("mit_feedforward_torque", ctypes.c_float * PRODUCT_DOF),
        ("pv_velocity_limits", ctypes.c_float * PRODUCT_DOF),
    ]


class MotionStatus(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("motion_id", ctypes.c_uint64),
        ("motion_type", ctypes.c_int32),
        ("state", ctypes.c_int32),
        ("active_segment", ctypes.c_uint32),
        ("waypoint_count", ctypes.c_uint32),
        ("elapsed_s", ctypes.c_double),
        ("duration_s", ctypes.c_double),
        ("progress", ctypes.c_float),
        ("error", ctypes.c_char * 512),
    ]


def native_functions(robot: ArxDCanDualArm) -> tuple[object, object]:
    library = robot._runtime._runtime_abi.lib
    start = library.articore_runtime_move_joint_trajectory
    start.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(TrajectoryWaypoint),
        ctypes.c_uint32,
        ctypes.POINTER(TrajectoryConfig),
        ctypes.POINTER(ctypes.c_uint64),
    ]
    start.restype = ctypes.c_int32
    status = library.articore_runtime_get_motion_status
    status.argtypes = [
        ctypes.c_void_p, ctypes.c_uint64, ctypes.POINTER(MotionStatus)
    ]
    status.restype = ctypes.c_int32
    return start, status


def last_error(robot: ArxDCanDualArm) -> str:
    value = robot._runtime._runtime_abi.lib.articore_runtime_last_error()
    return value.decode(errors="replace") if value else "unknown Runtime error"


def native_status(
    robot: ArxDCanDualArm, function: object, motion_id: int,
) -> MotionStatus:
    output = MotionStatus()
    output.struct_size = ctypes.sizeof(output)
    result = int(function(
        robot._runtime._require_open(), motion_id, ctypes.byref(output)
    ))
    if result != 0:
        raise RuntimeError(f"get_motion_status failed: {last_error(robot)}")
    return output


def wait_joint_target(
    robot: ArxDCanDualArm,
    left: list[float],
    right: list[float],
    *,
    timeout_s: float = 15.0,
    hold_s: float = 0.4,
) -> None:
    started = time.monotonic()
    robot.set_joint_pv(left=left, right=right, velocity=50.0)
    stable_since: float | None = None
    last_sequence = -1
    last_positions: tuple[float, ...] = ()
    last_velocities: tuple[float, ...] = ()
    while time.monotonic() - started < timeout_s:
        health = robot.get_health()
        if health.safe_stopped or health.fault_reason:
            raise RuntimeError(f"unsafe Runtime state during approach: {health}")
        state = robot.read_state()
        if state.sequence == last_sequence:
            time.sleep(0.0005)
            continue
        last_sequence = state.sequence
        positions = (*state.left.positions, *state.right.positions)
        velocities = (*state.left.arm.velocities, *state.right.arm.velocities)
        last_positions = tuple(float(value) for value in positions)
        last_velocities = tuple(float(value) for value in velocities)
        targets = (*left, *right)
        stable = all(
            abs(float(position) - target) <= 0.02 and
            abs(float(velocity)) <= 0.05
            for position, velocity, target in zip(
                positions, velocities, targets
            )
        )
        now = time.monotonic()
        if stable:
            stable_since = now if stable_since is None else stable_since
            if now - stable_since >= hold_s:
                return
        else:
            stable_since = None
    if last_positions:
        worst_position = max(
            range(PRODUCT_DOF),
            key=lambda index: abs(last_positions[index] - (*left, *right)[index]),
        )
        worst_velocity = max(
            range(PRODUCT_DOF), key=lambda index: abs(last_velocities[index])
        )
        raise RuntimeError(
            "ordinary PV approach did not settle: "
            f"worst_position_joint={worst_position + 1} "
            f"error={last_positions[worst_position] - (*left, *right)[worst_position]:.6f}rad, "
            f"worst_velocity_joint={worst_velocity + 1} "
            f"velocity={last_velocities[worst_velocity]:.6f}rad/s"
        )
    raise RuntimeError("ordinary PV approach did not receive feedback")


def build_request(
    duration_s: float, pv_velocity_limit: float,
) -> tuple[ctypes.Array[TrajectoryWaypoint], TrajectoryConfig]:
    waypoints = (TrajectoryWaypoint * 2)()
    for waypoint, time_s, right in (
        (waypoints[0], 0.0, START),
        (waypoints[1], duration_s, RIGHT_TARGET),
    ):
        waypoint.struct_size = ctypes.sizeof(TrajectoryWaypoint)
        waypoint.time_s = time_s
        waypoint.left_positions[:] = START
        waypoint.right_positions[:] = right
        waypoint.velocity_valid_mask = 0
        waypoint.acceleration_valid_mask = 0
    config = TrajectoryConfig()
    config.struct_size = ctypes.sizeof(config)
    config.interpolation = INTERPOLATION_QUINTIC
    config.control_mode = MODE_PV
    config.pv_velocity_limits[:] = [pv_velocity_limit] * PRODUCT_DOF
    return waypoints, config


def sample(robot: ArxDCanDualArm, started: float, status: MotionStatus) -> dict[str, object]:
    state = robot.read_state()
    return {
        "monotonic_ns": time.monotonic_ns(),
        "elapsed_s": time.monotonic() - started,
        "sequence": state.sequence,
        "trajectory_state": int(status.state),
        "trajectory_elapsed_s": float(status.elapsed_s),
        "trajectory_progress": float(status.progress),
        "left_q": list(state.left.positions),
        "left_dq": list(state.left.arm.velocities),
        "right_q": list(state.right.positions),
        "right_dq": list(state.right.arm.velocities),
    }


def direction_reversals(values: list[float], threshold: float = 0.01) -> int:
    signs = [1 if value > threshold else -1 if value < -threshold else 0
             for value in values]
    active = [value for value in signs if value]
    return sum(previous != current
               for previous, current in zip(active, active[1:]))


def summarize(
    samples: list[dict[str, object]], completed_s: float,
) -> dict[str, object]:
    moving = [item for item in samples
              if float(item["elapsed_s"]) <= completed_s]
    hold = [item for item in samples
            if float(item["elapsed_s"]) > completed_s]
    if not moving or not hold:
        raise RuntimeError("trajectory capture is missing motion or hold samples")
    duration = (int(samples[-1]["monotonic_ns"]) -
                int(samples[0]["monotonic_ns"])) * 1.0e-9
    joints: list[dict[str, object]] = []
    for joint in range(DOF):
        moving_q = [float(item["right_q"][joint]) for item in moving]
        moving_dq = [float(item["right_dq"][joint]) for item in moving]
        hold_q = [float(item["right_q"][joint]) for item in hold]
        hold_dq = [float(item["right_dq"][joint]) for item in hold]
        target = RIGHT_TARGET[joint]
        direction = 1.0 if target >= START[joint] else -1.0
        joints.append({
            "joint": joint + 1,
            "travel_rad": target - START[joint],
            "endpoint_error_rad": hold_q[-1] - target,
            "overshoot_rad": max(
                0.0,
                max(direction * (value - target) for value in moving_q),
            ),
            "motion_peak_abs_velocity_rad_s": max(
                abs(value) for value in moving_dq
            ),
            "motion_velocity_direction_reversals": direction_reversals(
                moving_dq
            ),
            "hold_position_peak_to_peak_rad": max(hold_q) - min(hold_q),
            "hold_peak_abs_velocity_rad_s": max(abs(value) for value in hold_dq),
            "hold_velocity_direction_reversals": direction_reversals(hold_dq),
        })
    return {
        "sample_count": len(samples),
        "sample_rate_hz": (len(samples) - 1) / max(duration, 1.0e-9),
        "completed_s": completed_s,
        "joints": joints,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--pv-v", type=float, required=True)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--hold-seconds", type=float, default=2.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--i-understand-both-arms-will-move", action="store_true"
    )
    args = parser.parse_args()
    if not args.i_understand_both_arms_will_move:
        parser.error("--i-understand-both-arms-will-move is required")
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        parser.error("--duration must be positive and finite")
    if not math.isfinite(args.pv_v) or args.pv_v <= 0.0:
        parser.error("--pv-v must be positive and finite")

    result: dict[str, object] = {
        "control_mode": "pv",
        "interpolation": "quintic",
        "duration_s": args.duration,
        "pv_velocity_limit_rad_s": args.pv_v,
        "left_start": START,
        "left_target": START,
        "right_start": START,
        "right_target": RIGHT_TARGET,
    }
    robot = ArxDCanDualArm(control_mode="pv", with_grippers=True)
    connected = False
    try:
        robot.connect()
        connected = True
        robot.enable()
        robot.set_max_acceleration(4.0)
        wait_joint_target(robot, START, START, timeout_s=args.timeout)
        start_function, status_function = native_functions(robot)
        waypoints, config = build_request(args.duration, args.pv_v)
        started = time.monotonic()
        motion_id = ctypes.c_uint64()
        code = int(start_function(
            robot._runtime._require_open(), waypoints, len(waypoints),
            ctypes.byref(config), ctypes.byref(motion_id),
        ))
        if code != 0:
            raise RuntimeError(
                f"move_joint_trajectory failed: {last_error(robot)}"
            )
        samples: list[dict[str, object]] = []
        last_sequence = -1
        completed_s: float | None = None
        while time.monotonic() - started < args.timeout:
            health = robot.get_health()
            if health.safe_stopped or health.fault_reason:
                raise RuntimeError(f"unsafe Runtime state: {health}")
            status = native_status(robot, status_function, motion_id.value)
            item = sample(robot, started, status)
            if item["sequence"] != last_sequence:
                last_sequence = int(item["sequence"])
                samples.append(item)
            if status.state in (TRAJECTORY_CANCELLED, TRAJECTORY_FAULT):
                error = bytes(status.error).split(b"\0", 1)[0].decode(
                    errors="replace"
                )
                raise RuntimeError(
                    f"trajectory ended state={status.state}: {error}"
                )
            if status.state == TRAJECTORY_COMPLETED and completed_s is None:
                completed_s = float(item["elapsed_s"])
            if completed_s is not None and (
                float(item["elapsed_s"]) - completed_s >= args.hold_seconds
            ):
                break
        if completed_s is None:
            raise RuntimeError("native trajectory did not complete")
        result["metrics"] = summarize(samples, completed_s)
        result["temperatures"] = temperatures(robot)
        result["health"] = str(robot.get_health())
        result["samples"] = samples
        wait_joint_target(robot, START, START, timeout_s=args.timeout)
    finally:
        if connected:
            robot.disconnect()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    compact = dict(result)
    compact.pop("samples", None)
    print(json.dumps(compact, indent=2), flush=True)


if __name__ == "__main__":
    main()

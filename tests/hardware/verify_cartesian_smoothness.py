#!/usr/bin/env python3
"""Measure set_pose, Linear, and Circular smoothness on Yunyi hardware."""

from __future__ import annotations

import argparse
import ctypes
import json
import math
import os
import time
from pathlib import Path

from arx_d_can import ArxDCanDualArm


RIGHT_CENTER = [0.403537, -0.231889, 0.381639, 0.0, -1.570796, 0.0]
RIGHT_CIRCLE_START = [0.403537, -0.231889, 0.301639, 0.0, -1.570796, 0.0]
RIGHT_CIRCLE_VIA = [0.403537, -0.311889, 0.381639, 0.0, -1.570796, 0.0]
RIGHT_CIRCLE_END = [0.403537, -0.231889, 0.461639, 0.0, -1.570796, 0.0]


class _ArmState(ctypes.Structure):
    _fields_ = [
        ("positions", ctypes.c_float * 7),
        ("velocities", ctypes.c_float * 7),
        ("torques", ctypes.c_float * 7),
        ("mos_temperatures", ctypes.c_float * 7),
        ("rotor_temperatures", ctypes.c_float * 7),
        ("enabled_mask", ctypes.c_uint32),
        ("enabled_valid_mask", ctypes.c_uint32),
        ("temperature_valid_mask", ctypes.c_uint32),
    ]


class _ProductState(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("has_grippers", ctypes.c_int32),
        ("left", _ArmState),
        ("right", _ArmState),
        ("left_gripper_available", ctypes.c_int32),
        ("right_gripper_available", ctypes.c_int32),
        ("left_gripper_opening", ctypes.c_float),
        ("right_gripper_opening", ctypes.c_float),
        ("left_gripper_level", ctypes.c_int32),
        ("right_gripper_level", ctypes.c_int32),
        ("left_gripper_enabled", ctypes.c_int32),
        ("right_gripper_enabled", ctypes.c_int32),
        ("left_gripper_enabled_valid", ctypes.c_int32),
        ("right_gripper_enabled_valid", ctypes.c_int32),
        ("left_gripper_mos_temperature", ctypes.c_float),
        ("left_gripper_rotor_temperature", ctypes.c_float),
        ("right_gripper_mos_temperature", ctypes.c_float),
        ("right_gripper_rotor_temperature", ctypes.c_float),
        ("left_gripper_temperature_valid", ctypes.c_int32),
        ("right_gripper_temperature_valid", ctypes.c_int32),
        ("timestamp_ns", ctypes.c_uint64),
        ("sequence", ctypes.c_uint64),
    ]


def temperatures(robot: ArxDCanDualArm) -> dict[str, object]:
    runtime = robot._runtime
    function = runtime._runtime_abi.lib.articore_runtime_get_state
    original_argtypes = function.argtypes
    original_restype = function.restype
    try:
        function.argtypes = [ctypes.c_void_p, ctypes.POINTER(_ProductState)]
        function.restype = ctypes.c_int32
        native = _ProductState()
        native.struct_size = ctypes.sizeof(native)
        result = int(function(runtime._require_open(), ctypes.byref(native)))
    finally:
        function.argtypes = original_argtypes
        function.restype = original_restype
    if result != 0:
        raise RuntimeError("articore_runtime_get_state failed")

    def arm(value: _ArmState) -> dict[str, list[float | None]]:
        def values(source: object) -> list[float | None]:
            return [
                float(source[index])
                if value.temperature_valid_mask & (1 << index)
                else None
                for index in range(7)
            ]
        return {
            "mos_c": values(value.mos_temperatures),
            "rotor_c": values(value.rotor_temperatures),
        }

    return {"left": arm(native.left), "right": arm(native.right)}


def wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def rpy_quaternion(pose: list[float]) -> tuple[float, float, float, float]:
    roll, pitch, yaw = pose[3:]
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def pose_error(actual: list[float], target: list[float]) -> tuple[float, float]:
    position = math.sqrt(sum((actual[i] - target[i]) ** 2 for i in range(3)))
    actual_q = rpy_quaternion(actual)
    target_q = rpy_quaternion(target)
    dot = min(1.0, abs(sum(a * b for a, b in zip(actual_q, target_q))))
    orientation = 2.0 * math.acos(dot)
    return position, orientation


def sample(robot: ArxDCanDualArm, started: float, phase: str) -> dict[str, object]:
    state = robot.read_state()
    return {
        "elapsed_s": time.monotonic() - started,
        "phase": phase,
        "sequence": state.sequence,
        "q": list(state.right.positions),
        "dq": list(state.right.arm.velocities),
        "pose": robot.get_pose("right"),
    }


def wait_ptp(
    robot: ArxDCanDualArm,
    started: float,
    target: list[float],
    samples: list[dict[str, object]],
    timeout_s: float = 20.0,
) -> float:
    stable_since: float | None = None
    last_sequence = -1
    while time.monotonic() - started < timeout_s:
        health = robot.get_health()
        if health.safe_stopped or health.fault_reason:
            raise RuntimeError(f"unsafe Runtime state during set_pose: {health}")
        item = sample(robot, started, "running")
        if item["sequence"] == last_sequence:
            time.sleep(0.0005)
            continue
        last_sequence = int(item["sequence"])
        samples.append(item)
        position_error, orientation_error = pose_error(item["pose"], target)
        stable = (
            position_error <= 0.005
            and orientation_error <= 0.035
            and max(abs(value) for value in item["dq"]) <= 0.05
        )
        if stable:
            stable_since = stable_since or float(item["elapsed_s"])
            if float(item["elapsed_s"]) - stable_since >= 0.4:
                return float(item["elapsed_s"])
        else:
            stable_since = None
    raise RuntimeError("set_pose did not settle within timeout")


def wait_native_motion(
    robot: ArxDCanDualArm,
    motion_id: int,
    started: float,
    samples: list[dict[str, object]],
    timeout_s: float = 45.0,
) -> tuple[float, dict[str, object]]:
    last_sequence = -1
    while time.monotonic() - started < timeout_s:
        status = robot.get_motion_status(motion_id)
        item = sample(robot, started, "running")
        if item["sequence"] != last_sequence:
            last_sequence = int(item["sequence"])
            item["motion_state"] = status.state.name
            item["progress"] = status.progress
            samples.append(item)
        if status.state.name == "COMPLETED":
            return float(item["elapsed_s"]), {
                "state": status.state.name,
                "motion_id": status.motion_id,
                "duration_s": status.duration_s,
                "elapsed_s": status.elapsed_s,
                "progress": status.progress,
                "error": status.error,
            }
        if status.state.name in {"FAULT", "CANCELLED"}:
            raise RuntimeError(f"Cartesian motion ended as {status}")
    raise RuntimeError("Cartesian motion did not complete within timeout")


def collect_hold(
    robot: ArxDCanDualArm,
    started: float,
    samples: list[dict[str, object]],
    duration_s: float = 3.0,
) -> None:
    hold_started = time.monotonic()
    last_sequence = -1
    while time.monotonic() - hold_started < duration_s:
        item = sample(robot, started, "hold")
        if item["sequence"] == last_sequence:
            time.sleep(0.0005)
            continue
        last_sequence = int(item["sequence"])
        samples.append(item)


def direction_reversals(values: list[float], threshold: float = 0.01) -> int:
    signs = [1 if value > threshold else -1 if value < -threshold else 0 for value in values]
    active = [value for value in signs if value]
    return sum(a != b for a, b in zip(active, active[1:]))


def summarize(samples: list[dict[str, object]], settled_s: float) -> dict[str, object]:
    running = [item for item in samples if item["phase"] == "running"]
    hold = [item for item in samples if item["phase"] == "hold"]
    output: dict[str, object] = {
        "running_samples": len(running),
        "hold_samples": len(hold),
        "settled_s": settled_s,
        "running_sample_hz": (
            len(running) / max(1e-9, float(running[-1]["elapsed_s"]) - float(running[0]["elapsed_s"]))
            if len(running) > 1
            else 0.0
        ),
        "joints": [],
    }
    joints: list[dict[str, float | int]] = []
    for index in range(7):
        running_q = [float(item["q"][index]) for item in running]
        running_dq = [float(item["dq"][index]) for item in running]
        hold_q = [float(item["q"][index]) for item in hold]
        hold_dq = [float(item["dq"][index]) for item in hold]
        joints.append({
            "joint": index + 1,
            "running_position_range_rad": max(running_q) - min(running_q),
            "running_peak_abs_velocity_rad_s": max(abs(value) for value in running_dq),
            "running_velocity_direction_reversals": direction_reversals(running_dq),
            "hold_position_peak_to_peak_rad": max(hold_q) - min(hold_q),
            "hold_peak_abs_velocity_rad_s": max(abs(value) for value in hold_dq),
            "hold_velocity_direction_reversals": direction_reversals(hold_dq),
        })
    output["joints"] = joints
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--motion", choices=("set_pose", "linear", "circular"), required=True
    )
    parser.add_argument("--speed", type=float, default=50.0)
    parser.add_argument(
        "--linear-distance-m",
        type=float,
        default=0.07,
        help="Linear +Z distance in metres, within (0, 0.14] (default: 0.07)",
    )
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--hold-seconds", type=float, default=1.0)
    parser.add_argument("--i-understand-the-right-arm-will-move", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not args.i_understand_the_right_arm_will_move:
        parser.error("--i-understand-the-right-arm-will-move is required")
    if (
        not math.isfinite(args.linear_distance_m)
        or args.linear_distance_m <= 0.0
        or args.linear_distance_m > 0.14
    ):
        parser.error("--linear-distance-m must be within (0, 0.14]")

    robot = ArxDCanDualArm(control_mode="pv", with_grippers=True)
    samples: list[dict[str, object]] = []
    result: dict[str, object] = {
        "motion": args.motion,
        "side": "right",
        "speed_percent": args.speed,
        "runtime_library": os.environ.get("ARTICORE_RUNTIME_LIB"),
    }
    connected = False
    try:
        robot.connect()
        connected = True
        result["initial_pose"] = robot.get_pose("right")
        result["initial_temperatures"] = temperatures(robot)
        robot.enable()
        if args.motion in ("linear", "circular"):
            path_start = (
                RIGHT_CENTER if args.motion == "linear" else RIGHT_CIRCLE_START
            )
            preposition_started = time.monotonic()
            robot.set_pose(
                left_target_pose=robot.get_pose("left"),
                right_target_pose=path_start,
                speed_percent=50.0,
            )
            preposition_samples: list[dict[str, object]] = []
            result["preposition_settled_s"] = wait_ptp(
                robot,
                preposition_started,
                path_start,
                preposition_samples,
                timeout_s=args.timeout,
            )
        robot.set_speed_percent(args.speed)
        started = time.monotonic()
        if args.motion == "set_pose":
            result["target"] = RIGHT_CENTER
            robot.set_pose(
                left_target_pose=robot.get_pose("left"),
                right_target_pose=RIGHT_CENTER,
                speed_percent=args.speed,
            )
            settled_s = wait_ptp(
                robot, started, RIGHT_CENTER, samples, timeout_s=args.timeout
            )
        elif args.motion == "linear":
            linear_end = list(RIGHT_CENTER)
            linear_end[2] += args.linear_distance_m
            result["start"] = RIGHT_CENTER
            result["end"] = linear_end
            result["linear_distance_m"] = args.linear_distance_m
            motion_id = robot.move_linear_trajectory(
                side="right", start_pose=RIGHT_CENTER,
                end_pose=linear_end,
            )
            settled_s, result["native_status"] = wait_native_motion(
                robot, motion_id, started, samples, timeout_s=args.timeout
            )
        else:
            result["start"] = RIGHT_CIRCLE_START
            result["via"] = RIGHT_CIRCLE_VIA
            result["end"] = RIGHT_CIRCLE_END
            motion_id = robot.move_circular_trajectory(
                side="right", start_pose=RIGHT_CIRCLE_START,
                via_pose=RIGHT_CIRCLE_VIA, end_pose=RIGHT_CIRCLE_END,
            )
            settled_s, result["native_status"] = wait_native_motion(
                robot, motion_id, started, samples, timeout_s=args.timeout
            )
        collect_hold(robot, started, samples, duration_s=args.hold_seconds)
        result["final_pose"] = robot.get_pose("right")
        result["final_temperatures"] = temperatures(robot)
        result["health"] = str(robot.get_health())
        result["metrics"] = summarize(samples, settled_s)
        result["samples"] = samples
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

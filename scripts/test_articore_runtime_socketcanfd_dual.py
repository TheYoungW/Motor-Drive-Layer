#!/usr/bin/env python3
"""Real-hardware acceptance test for dual-arm ArticoreRuntime at 500 Hz.

The test holds all fourteen arm joints at their measured positions while both
product grippers hold their measured opening. It enables real hardware and
always runs the Runtime disable transaction before releasing native handles.
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from collections import Counter
from dataclasses import asdict
from pathlib import Path
from typing import Sequence

from motor_drive_layer import (
    ArticoreRuntime,
    CommandLifetime,
    Controller,
    ControllerGroup,
    GripperProductBinding,
    JointControlConfig,
    RuntimeConfig,
    RuntimeControlMode,
    RuntimeMitCommand,
    RuntimeMotor,
    SafetyState,
)
from motor_drive_layer.stress import MotorSpec, parse_motor_spec

from test_socketcanfd_brs_dual_channel import (
    _capture_feedback,
    _counter_delta,
    _interface_stats,
)


_MODEL_LIMITS = {
    "8009": (12.5, 45.0, 54.0),
    "4340P": (12.5, 10.0, 28.0),
    "4310": (12.5, 30.0, 10.0),
}


def run_test(
    specs: Sequence[MotorSpec],
    *,
    duration_s: float,
    minimum_feedback_ratio: float,
    hold_kp: float,
    hold_kd: float,
    tx_gap_us: int,
) -> dict[str, object]:
    grouped: dict[int, list[MotorSpec]] = {}
    for spec in specs:
        grouped.setdefault(spec.channel, []).append(spec)
    if set(grouped) != {0, 1} or any(len(values) != 8 for values in grouped.values()):
        raise ValueError(
            "hardware acceptance requires exactly 8 motors on channel 0 and 8 on channel 1"
        )
    if duration_s <= 0 or not 0 < minimum_feedback_ratio <= 1:
        raise ValueError("duration must be positive and feedback ratio must be in (0, 1]")
    if hold_kp < 0 or hold_kd < 0 or tx_gap_us < 0:
        raise ValueError("hold gains and tx_gap_us must be non-negative")
    for channel, values in grouped.items():
        if sorted(spec.motor_id for spec in values) != list(range(1, 9)):
            raise ValueError(f"channel {channel} must contain motor IDs 1..8")
        if len({spec.feedback_id for spec in values}) != 8:
            raise ValueError(f"channel {channel} contains duplicate feedback IDs")
        for spec in values:
            if spec.model not in _MODEL_LIMITS:
                raise ValueError(f"unsupported acceptance-test model: {spec.model}")

    ordered_specs = tuple(
        sorted(grouped[0], key=lambda item: item.motor_id)
        + sorted(grouped[1], key=lambda item: item.motor_id)
    )
    channel_names = {0: "can0", 1: "can1"}
    before_interfaces = {
        name: _interface_stats(name) for name in channel_names.values()
    }
    controllers: dict[int, Controller] = {}
    motors = []
    group: ControllerGroup | None = None
    runtime: ArticoreRuntime | None = None
    runtime_control_hz = 0
    enable_report = None
    disable_report = None
    final_health = None
    post_disable_statuses: list[int | None] = []
    cleanup_errors: list[str] = []
    raw_counts: Counter[tuple[int, int]] = Counter()
    raw_brs_counts: Counter[tuple[int, int]] = Counter()
    capture_stop = threading.Event()
    capture_threads: list[threading.Thread] = []
    elapsed_s = 0.0
    feedback_hz: list[float] = []
    initial_counts: list[int] = []
    final_counts: list[int] = []
    integrity = []
    try:
        for channel in (0, 1):
            controller = Controller.from_socketcanfd(
                channel_names[channel], enable_brs=True
            )
            capabilities = controller.transport_capabilities()
            if (
                capabilities.transport != "socketcanfd"
                or not capabilities.can_fd
                or not capabilities.can_fd_brs
            ):
                raise RuntimeError(
                    f"{channel_names[channel]} is not active SocketCAN-FD+BRS: "
                    f"{capabilities}"
                )
            controllers[channel] = controller
            for spec in sorted(grouped[channel], key=lambda item: item.motor_id):
                motors.append(
                    controller.add_damiao_motor(
                        spec.motor_id, spec.feedback_id, spec.model
                    )
                )
            controller.set_tx_gap_us(tx_gap_us)

        group = ControllerGroup(tuple(controllers.values()))
        runtime_motors = [
            RuntimeMotor(
                motor,
                side=spec.channel,
                name=("left" if spec.channel == 0 else "right")
                + ("-gripper" if spec.motor_id == 8 else f"-joint{spec.motor_id}"),
                is_gripper=spec.motor_id == 8,
                safe_kp=0.0 if spec.motor_id == 8 else hold_kp,
                safe_kd=0.0 if spec.motor_id == 8 else hold_kd,
            )
            for spec, motor in zip(ordered_specs, motors)
        ]
        runtime = ArticoreRuntime(
            RuntimeConfig(
                control_hz=500,
                command_timeout_ms=250,
                enable_grace_ms=2000,
                safe_hold_hz=100,
                feedback_check_hz=100,
                feedback_failure_threshold=3,
                feedback_max_age_ms=300,
                safe_hold_failure_threshold=3,
                disable_feedback_timeout_ms=100,
                safe_pv_velocity_limit=0.2,
                gripper_control_hz=500,
            ),
            group,
            controllers[0],
            controllers[1],
            runtime_motors,
        )
        runtime_control_hz = runtime.control_hz
        if runtime_control_hz != 500:
            raise RuntimeError(
                f"transport-aware ArticoreRuntime returned {runtime_control_hz} Hz"
            )
        runtime.configure_joints(
            [
                JointControlConfig(
                    motor=motor,
                    lower_position=-_MODEL_LIMITS[spec.model][0],
                    upper_position=_MODEL_LIMITS[spec.model][0],
                    velocity_limit=_MODEL_LIMITS[spec.model][1],
                    torque_limit=_MODEL_LIMITS[spec.model][2],
                    mit_kp=hold_kp,
                    mit_kd=hold_kd,
                )
                for spec, motor in zip(ordered_specs, motors)
                if spec.motor_id != 8
            ]
        )
        runtime.configure_gripper_products(
            [
                GripperProductBinding(motor, "yunyi_gripper_v1")
                for spec, motor in zip(ordered_specs, motors)
                if spec.motor_id == 8
            ]
        )
        runtime.connect()
        positions = []
        for spec, motor in zip(ordered_specs, motors):
            state = motor.get_state()
            if state is None:
                raise RuntimeError(
                    f"missing initial state for channel {spec.channel} motor {spec.motor_id}"
                )
            positions.append(state.pos)

        capture_threads = [
            threading.Thread(
                target=_capture_feedback,
                args=(
                    channel,
                    channel_names[channel],
                    capture_stop,
                    raw_counts,
                    raw_brs_counts,
                ),
                daemon=True,
            )
            for channel in (0, 1)
        ]
        for thread in capture_threads:
            thread.start()
        time.sleep(0.05)

        enable_report = runtime.enable(RuntimeControlMode.MIT)
        runtime.submit_mit(
            [
                RuntimeMitCommand(
                    motor=motor,
                    position=position,
                    velocity=0.0,
                    kp=hold_kp,
                    kd=hold_kd,
                    feedforward_torque=0.0,
                )
                for spec, motor, position in zip(ordered_specs, motors, positions)
                if spec.motor_id != 8
            ],
            CommandLifetime.HOLD_UNTIL_REPLACED,
        )
        initial_counts = [
            motor.get_feedback_stats().update_count for motor in motors
        ]
        started = time.monotonic()
        deadline = started + duration_s
        while True:
            now = time.monotonic()
            if now >= deadline:
                break
            health = runtime.health
            if health.state is not SafetyState.RUNNING:
                raise RuntimeError(
                    f"Runtime left RUNNING state: {health.state.name}; "
                    f"reason={health.fault_reason}"
                )
            time.sleep(min(0.1, deadline - now))
        elapsed_s = time.monotonic() - started
        final_health = runtime.health
        final_counts = [
            motor.get_feedback_stats().update_count for motor in motors
        ]
        integrity = [motor.get_feedback_integrity_stats() for motor in motors]
        feedback_hz = [
            (after - before) / elapsed_s
            for before, after in zip(initial_counts, final_counts)
        ]
    finally:
        capture_stop.set()
        for thread in capture_threads:
            thread.join(timeout=1.0)
        if runtime is not None:
            try:
                disable_report = runtime.disable()
            except Exception as exc:
                cleanup_errors.append(f"runtime disable: {exc}")
            for motor in motors:
                try:
                    state = motor.get_state()
                    post_disable_statuses.append(
                        None if state is None else int(state.status_code)
                    )
                except Exception as exc:
                    cleanup_errors.append(f"post-disable state: {exc}")
            try:
                runtime.close()
            except Exception as exc:
                cleanup_errors.append(f"runtime close: {exc}")
        if group is not None:
            try:
                group.close()
            except Exception as exc:
                cleanup_errors.append(f"group close: {exc}")
        for motor in reversed(motors):
            try:
                motor.close()
            except Exception as exc:
                cleanup_errors.append(f"motor close: {exc}")
        for controller in reversed(tuple(controllers.values())):
            try:
                controller.close_bus()
                controller.close()
            except Exception as exc:
                cleanup_errors.append(f"controller close: {exc}")

    after_interfaces = {
        name: _interface_stats(name) for name in channel_names.values()
    }
    interface_deltas = {
        name: _counter_delta(before_interfaces[name], after_interfaces[name])
        for name in channel_names.values()
    }
    minimum_feedback_hz = runtime_control_hz * minimum_feedback_ratio
    feedback = [
        {
            "spec": asdict(spec),
            "driver_hz": rate,
            "raw_bus_hz": raw_counts[(spec.channel, spec.motor_id)] / elapsed_s,
            "raw_bus_brs_hz": raw_brs_counts[(spec.channel, spec.motor_id)]
            / elapsed_s,
            "rejected_frames": item.rejected_frame_count,
            "identity_mismatches": item.identity_mismatch_count,
            "implausible_jumps": item.implausible_position_jump_count,
        }
        for spec, rate, item in zip(ordered_specs, feedback_hz, integrity)
    ]
    low_feedback = [
        item for item in feedback if item["driver_hz"] < minimum_feedback_hz
    ]
    bad_interfaces = {
        name: {"state": after_interfaces[name]["state"], **delta}
        for name, delta in interface_deltas.items()
        if after_interfaces[name]["state"] != "ERROR-ACTIVE" or any(delta.values())
    }
    runtime_errors = (
        final_health is None
        or final_health.state is not SafetyState.RUNNING
        or final_health.consecutive_send_failures != 0
        or final_health.consecutive_feedback_failures != 0
        or final_health.left_transport.send_errors != 0
        or final_health.left_transport.receive_errors != 0
        or final_health.right_transport.send_errors != 0
        or final_health.right_transport.receive_errors != 0
    )
    disable_ok = (
        disable_report is not None
        and disable_report.success
        and disable_report.barrier_confirmed
        and disable_report.expected_count == 16
        and disable_report.disabled_count == 16
        and post_disable_statuses == [0] * 16
    )
    passed = (
        runtime_control_hz == 500
        and not low_feedback
        and not bad_interfaces
        and not runtime_errors
        and disable_ok
        and not cleanup_errors
    )
    return {
        "passed": passed,
        "runtime_control_hz": runtime_control_hz,
        "duration_s": elapsed_s,
        "minimum_feedback_ratio": minimum_feedback_ratio,
        "feedback": feedback,
        "low_feedback": low_feedback,
        "enable_report": None if enable_report is None else asdict(enable_report),
        "final_health": None if final_health is None else asdict(final_health),
        "disable_report": None if disable_report is None else asdict(disable_report),
        "post_disable_statuses": post_disable_statuses,
        "interfaces": after_interfaces,
        "interface_counter_deltas": interface_deltas,
        "bad_interfaces": bad_interfaces,
        "runtime_errors": runtime_errors,
        "cleanup_errors": cleanup_errors,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--motor",
        action="append",
        type=parse_motor_spec,
        required=True,
        metavar="CHANNEL:MOTOR_ID:FEEDBACK_ID:MODEL",
    )
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--minimum-feedback-ratio", type=float, default=0.95)
    parser.add_argument("--hold-kp", type=float, default=5.0)
    parser.add_argument("--hold-kd", type=float, default=1.0)
    parser.add_argument("--tx-gap-us", type=int, default=200)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--i-understand-motors-will-be-enabled",
        action="store_true",
        required=True,
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    result = run_test(
        args.motor,
        duration_s=args.duration_s,
        minimum_feedback_ratio=args.minimum_feedback_ratio,
        hold_kp=args.hold_kp,
        hold_kd=args.hold_kd,
        tx_gap_us=args.tx_gap_us,
    )
    rendered = json.dumps(result, ensure_ascii=False, indent=2)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

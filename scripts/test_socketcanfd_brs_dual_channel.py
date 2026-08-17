#!/usr/bin/env python3
"""Destructive-hardware opt-in test for dual-channel SocketCAN-FD+BRS.

The test enables every configured motor, holds the initial measured positions
with configurable low MIT gains and zero feed-forward torque, and always
attempts to disable all motors before closing. It refuses to start unless the
caller explicitly acknowledges that real hardware will be enabled.
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import subprocess
import threading
import time
from collections import Counter
from dataclasses import asdict
from pathlib import Path
from typing import Sequence

from motor_drive_layer import Controller, ControllerGroup
from motor_drive_layer.stress import MotorSpec, parse_motor_spec


def _interface_stats(channel: str) -> dict[str, int | str]:
    completed = subprocess.run(
        ["ip", "-details", "-statistics", "-json", "link", "show", "dev", channel],
        check=True,
        capture_output=True,
        text=True,
    )
    raw = json.loads(completed.stdout)[0]
    xstats = raw.get("linkinfo", {}).get("info_xstats", {})
    stats64 = raw.get("stats64", {})
    rx = stats64.get("rx", {})
    tx = stats64.get("tx", {})
    return {
        "state": raw.get("linkinfo", {}).get("info_data", {}).get("state", "UNKNOWN"),
        "bus_error": int(xstats.get("bus_error", 0)),
        "error_warning": int(xstats.get("error_warning", 0)),
        "error_passive": int(xstats.get("error_passive", 0)),
        "bus_off": int(xstats.get("bus_off", 0)),
        "rx_errors": int(rx.get("errors", 0)),
        "rx_dropped": int(rx.get("dropped", 0)),
        "tx_errors": int(tx.get("errors", 0)),
        "tx_dropped": int(tx.get("dropped", 0)),
    }


def _counter_delta(before: dict[str, int | str], after: dict[str, int | str]) -> dict[str, int]:
    return {
        key: int(after[key]) - int(before[key])
        for key in (
            "bus_error",
            "error_warning",
            "error_passive",
            "bus_off",
            "rx_errors",
            "rx_dropped",
            "tx_errors",
            "tx_dropped",
        )
    }


def _capture_feedback(
    channel: int,
    interface: str,
    stop: threading.Event,
    counts: Counter[tuple[int, int]],
    brs_counts: Counter[tuple[int, int]],
) -> None:
    receiver = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    receiver.setsockopt(socket.SOL_CAN_RAW, 5, 1)  # CAN_RAW_FD_FRAMES
    receiver.settimeout(0.05)
    receiver.bind((interface,))
    try:
        while not stop.is_set():
            try:
                raw = receiver.recv(72)
            except TimeoutError:
                continue
            if len(raw) != 72:
                continue
            can_id = struct.unpack_from("=I", raw)[0] & 0x1FFFFFFF
            is_standard_feedback = 0x11 <= can_id <= 0x18
            is_dm_feedback = 0x201 <= can_id <= 0x208
            if not (is_standard_feedback or is_dm_feedback):
                continue
            key = (channel, can_id & 0x0F)
            counts[key] += 1
            if raw[5] & 0x01:
                brs_counts[key] += 1
    finally:
        receiver.close()


def run_test(
    specs: Sequence[MotorSpec],
    *,
    control_hz: float,
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
        raise ValueError("hardware acceptance requires exactly 8 motors on channel 0 and 8 on channel 1")
    if (
        control_hz <= 0
        or duration_s <= 0
        or not 0 < minimum_feedback_ratio <= 1
        or hold_kp < 0
        or hold_kd < 0
        or tx_gap_us < 0
    ):
        raise ValueError(
            "control_hz/duration_s must be positive, feedback ratio in (0, 1], "
            "hold gains non-negative, and tx_gap_us non-negative"
        )
    for channel, values in grouped.items():
        ids = [item.motor_id for item in values]
        feedback_ids = [item.feedback_id for item in values]
        if len(set(ids)) != 8 or len(set(feedback_ids)) != 8:
            raise ValueError(f"channel {channel} contains duplicate motor or feedback IDs")
    ordered_specs = tuple(spec for channel in (0, 1) for spec in grouped[channel])

    channel_names = {0: "can0", 1: "can1"}
    before_interfaces = {
        name: _interface_stats(name) for name in channel_names.values()
    }
    controllers: dict[int, Controller] = {}
    motors = []
    disable_errors: list[str] = []
    disabled_count = 0
    raw_counts: Counter[tuple[int, int]] = Counter()
    raw_brs_counts: Counter[tuple[int, int]] = Counter()
    capture_stop = threading.Event()
    capture_threads: list[threading.Thread] = []
    try:
        for channel in (0, 1):
            controller = Controller.from_socketcanfd(
                channel_names[channel], enable_brs=True
            )
            capabilities = controller.transport_capabilities()
            if not capabilities.can_fd or not capabilities.can_fd_brs:
                raise RuntimeError(
                    f"{channel_names[channel]} does not report active CAN-FD+BRS: "
                    f"{capabilities}"
                )
            controllers[channel] = controller
            for spec in grouped[channel]:
                motors.append(
                    controller.add_damiao_motor(
                        spec.motor_id, spec.feedback_id, spec.model
                    )
                )
            controller.set_tx_gap_us(tx_gap_us)

        for controller in controllers.values():
            controller.request_feedback_all(100)
        positions = []
        for motor in motors:
            state = motor.get_state()
            if state is None:
                raise RuntimeError("missing initial motor state")
            positions.append(state.pos)
        initial_counts = [motor.get_feedback_stats().update_count for motor in motors]

        for controller in controllers.values():
            controller.enable_all()

        capture_threads = [
            threading.Thread(
                target=_capture_feedback,
                args=(channel, channel_names[channel], capture_stop, raw_counts, raw_brs_counts),
                daemon=True,
            )
            for channel in (0, 1)
        ]
        for thread in capture_threads:
            thread.start()
        time.sleep(0.05)
        interval_ns = round(1_000_000_000 / control_hz)
        started_ns = time.monotonic_ns()
        deadline_ns = started_ns
        cycles = 0
        with ControllerGroup(tuple(controllers.values())) as group:
            batch = group.prepare_mit(motors)
            while time.monotonic_ns() - started_ns < duration_s * 1_000_000_000:
                batch.send(positions, 0.0, hold_kp, hold_kd, 0.0)
                cycles += 1
                deadline_ns += interval_ns
                remaining_ns = deadline_ns - time.monotonic_ns()
                if remaining_ns > 0:
                    time.sleep(remaining_ns / 1_000_000_000)
        elapsed_s = (time.monotonic_ns() - started_ns) / 1_000_000_000
        capture_stop.set()
        for thread in capture_threads:
            thread.join(timeout=1.0)
        completed_hz = cycles / elapsed_s
        final_counts = [motor.get_feedback_stats().update_count for motor in motors]
        integrity = [motor.get_feedback_integrity_stats() for motor in motors]
        feedback_hz = [
            (after - before) / elapsed_s
            for before, after in zip(initial_counts, final_counts)
        ]
        minimum_feedback_hz = completed_hz * minimum_feedback_ratio
        low_feedback = [
            {
                "channel": spec.channel,
                "motor_id": spec.motor_id,
                "feedback_hz": rate,
            }
            for spec, rate in zip(ordered_specs, feedback_hz)
            if rate < minimum_feedback_hz
        ]
    finally:
        capture_stop.set()
        for thread in capture_threads:
            thread.join(timeout=1.0)
        for channel, controller in reversed(tuple(controllers.items())):
            try:
                controller.disable_all()
            except Exception as exc:  # continue disabling the other channel
                disable_errors.append(f"{channel_names[channel]}: {exc}")
        for controller in controllers.values():
            try:
                controller.request_feedback_all(100)
            except Exception as exc:
                disable_errors.append(f"post-disable feedback: {exc}")
        for motor in motors:
            try:
                state = motor.get_state()
                disabled_count += int(state is not None and state.status_code == 0)
            except Exception as exc:
                disable_errors.append(f"post-disable state: {exc}")
        for motor in reversed(motors):
            motor.close()
        for controller in reversed(tuple(controllers.values())):
            controller.close_bus()
            controller.close()

    after_interfaces = {
        name: _interface_stats(name) for name in channel_names.values()
    }
    interface_deltas = {
        name: _counter_delta(before_interfaces[name], after_interfaces[name])
        for name in channel_names.values()
    }
    bad_interfaces = {
        name: {"state": after_interfaces[name]["state"], **delta}
        for name, delta in interface_deltas.items()
        if after_interfaces[name]["state"] != "ERROR-ACTIVE" or any(delta.values())
    }
    passed = not low_feedback and not bad_interfaces and not disable_errors and disabled_count == 16
    return {
        "passed": passed,
        "control_hz_requested": control_hz,
        "control_hz_completed": completed_hz,
        "cycles": cycles,
        "elapsed_s": elapsed_s,
        "minimum_feedback_ratio": minimum_feedback_ratio,
        "hold_kp": hold_kp,
        "hold_kd": hold_kd,
        "tx_gap_us": tx_gap_us,
        "feedback_hz": [
            {
                "spec": asdict(spec),
                "driver_hz": rate,
                "raw_bus_hz": raw_counts[(spec.channel, spec.motor_id)] / elapsed_s,
                "raw_bus_brs_hz": raw_brs_counts[(spec.channel, spec.motor_id)] / elapsed_s,
                "rejected_frames": item.rejected_frame_count,
                "identity_mismatches": item.identity_mismatch_count,
                "implausible_jumps": item.implausible_position_jump_count,
            }
            for spec, rate, item in zip(ordered_specs, feedback_hz, integrity)
        ],
        "low_feedback": low_feedback,
        "interfaces": after_interfaces,
        "interface_counter_deltas": interface_deltas,
        "bad_interfaces": bad_interfaces,
        "disabled_count": disabled_count,
        "disable_errors": disable_errors,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--motor", action="append", type=parse_motor_spec, required=True,
        metavar="CHANNEL:MOTOR_ID:FEEDBACK_ID:MODEL",
    )
    parser.add_argument("--control-hz", type=float, default=400.0)
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--minimum-feedback-ratio", type=float, default=0.95)
    parser.add_argument("--hold-kp", type=float, default=5.0)
    parser.add_argument("--hold-kd", type=float, default=1.0)
    parser.add_argument("--tx-gap-us", type=int, default=200)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--i-understand-motors-will-be-enabled", action="store_true", required=True
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    result = run_test(
        args.motor,
        control_hz=args.control_hz,
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

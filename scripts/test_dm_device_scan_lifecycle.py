#!/usr/bin/env python3
"""Feedback-only DM_Device scanner and cross-process lifecycle test.

The parent alternates a scanner subprocess and a fresh CH0/CH1 reader
subprocess. Neither worker enables, disables, or commands a motor.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from collections import defaultdict

from motor_drive_layer import Controller, MotorCandidate, PresencePolicy, PresenceState
from motor_drive_layer.stress import MotorSpec, parse_motor_spec


def _grouped(specs: tuple[MotorSpec, ...]) -> dict[int, list[MotorSpec]]:
    grouped: dict[int, list[MotorSpec]] = defaultdict(list)
    for spec in specs:
        grouped[spec.channel].append(spec)
    if not grouped:
        raise ValueError("at least one --motor is required")
    return dict(sorted(grouped.items()))


def _close_read_only(controllers: list[Controller], motors: list[object]) -> None:
    first_error: Exception | None = None
    for motor in reversed(motors):
        close = getattr(motor, "close", None)
        if close is not None:
            try:
                close()
            except Exception as exc:
                if first_error is None:
                    first_error = exc
    for controller in reversed(controllers):
        try:
            controller.close_bus()
        except Exception as exc:
            if first_error is None:
                first_error = exc
        try:
            controller.close()
        except Exception as exc:
            if first_error is None:
                first_error = exc
    if first_error is not None:
        raise first_error


def _scan_worker(
    specs: tuple[MotorSpec, ...],
    device: str,
    bitrate: int,
    data_bitrate: int,
    timeout_ms: int,
    retries: int,
) -> None:
    controllers: list[Controller] = []
    motors: list[object] = []
    try:
        for channel, channel_specs in _grouped(specs).items():
            controller = Controller.from_dm_device(
                device=device,
                channel=channel,
                bitrate=bitrate,
                data_bitrate=data_bitrate,
            )
            controllers.append(controller)
            candidates = tuple(
                MotorCandidate(
                    role=f"channel-{channel}-motor-0x{spec.motor_id:X}",
                    motor_id=spec.motor_id,
                    feedback_id=spec.feedback_id,
                    model=spec.model,
                    policy=PresencePolicy.OPTIONAL,
                )
                for spec in channel_specs
            )
            results = controller.discover_damiao_motors(
                candidates,
                timeout_ms=timeout_ms,
                retries=retries,
            )
            motors.extend(result.motor for result in results if result.motor is not None)
            missing = [
                result.motor_id
                for result in results
                if result.state != PresenceState.PRESENT
            ]
            if missing:
                raise RuntimeError(f"CH{channel} scan missed motor IDs {missing}")
    finally:
        _close_read_only(controllers, motors)


def _read_worker(
    specs: tuple[MotorSpec, ...],
    device: str,
    bitrate: int,
    data_bitrate: int,
    timeout_ms: int,
) -> None:
    controllers: list[Controller] = []
    motors: list[object] = []
    try:
        for channel, channel_specs in _grouped(specs).items():
            controller = Controller.from_dm_device(
                device=device,
                channel=channel,
                bitrate=bitrate,
                data_bitrate=data_bitrate,
            )
            controllers.append(controller)
            channel_motors = [
                controller.add_damiao_motor(
                    spec.motor_id,
                    spec.feedback_id,
                    spec.model,
                )
                for spec in channel_specs
            ]
            motors.extend(channel_motors)

        # Both channels are open before either request, matching dual-arm use.
        for controller in controllers:
            controller.request_feedback_all(timeout_ms)
        if any(getattr(motor, "get_state")() is None for motor in motors):
            raise RuntimeError("reader did not cache every requested motor state")
    finally:
        _close_read_only(controllers, motors)


def _worker_command(args: argparse.Namespace, worker: str) -> list[str]:
    command = [
        sys.executable,
        os.path.abspath(__file__),
        "--worker",
        worker,
        "--device",
        args.device,
        "--bitrate",
        str(args.bitrate),
        "--data-bitrate",
        str(args.data_bitrate),
        "--timeout-ms",
        str(args.timeout_ms),
        "--retries",
        str(args.retries),
    ]
    for spec in args.motor:
        command.extend(
            [
                "--motor",
                f"{spec.channel}:{spec.motor_id}:{spec.feedback_id}:{spec.model}",
            ]
        )
    return command


def _run_parent(args: argparse.Namespace) -> None:
    started = time.monotonic()
    environment = os.environ.copy()
    if args.abi:
        environment["MOTOR_DM_DEVICE_ABI"] = args.abi
        environment.pop("MOTOR_DM_DEVICE_LIB", None)

    def run_worker(worker: str) -> None:
        completed = subprocess.run(
            _worker_command(args, worker),
            check=False,
            env=environment,
            text=True,
            capture_output=True,
        )
        if completed.returncode != 0:
            raise RuntimeError(
                f"{worker} subprocess failed with exit code {completed.returncode}\n"
                f"stdout:\n{completed.stdout}\n"
                f"stderr:\n{completed.stderr}"
            )

    for cycle in range(1, args.cycles + 1):
        run_worker("scan")
        run_worker("read")
        if cycle == 1 or cycle % 10 == 0 or cycle == args.cycles:
            print(f"scan->exit->CH0/CH1 read: {cycle}/{args.cycles}", flush=True)
    print(
        f"passed {args.cycles} cross-process scan/read cycles in "
        f"{time.monotonic() - started:.3f} s"
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--motor", action="append", type=parse_motor_spec, required=True)
    parser.add_argument("--device", default="usb2canfd-dual")
    parser.add_argument("--bitrate", type=int, default=1_000_000)
    parser.add_argument("--data-bitrate", type=int, default=5_000_000)
    parser.add_argument("--timeout-ms", type=int, default=150)
    parser.add_argument("--retries", type=int, default=1)
    parser.add_argument("--cycles", type=int, default=100)
    parser.add_argument("--abi", choices=("v1.0", "v1.1"), default="v1.1")
    parser.add_argument("--worker", choices=("scan", "read"), help=argparse.SUPPRESS)
    return parser


def main() -> int:
    args = _parser().parse_args()
    specs = tuple(args.motor)
    if args.cycles <= 0:
        raise ValueError("cycles must be positive")
    if args.worker == "scan":
        _scan_worker(
            specs,
            args.device,
            args.bitrate,
            args.data_bitrate,
            args.timeout_ms,
            args.retries,
        )
    elif args.worker == "read":
        _read_worker(
            specs,
            args.device,
            args.bitrate,
            args.data_bitrate,
            args.timeout_ms,
        )
    else:
        _run_parent(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

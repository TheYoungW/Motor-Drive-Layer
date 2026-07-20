#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Sequence
from typing import TypeVar

from motor_drive_layer import Controller, Mode


MOTOR_COUNT = 7
T = TypeVar("T")


def _parse_can_id(text: str) -> int:
    value = int(text, 0)
    if not 0 <= value <= 0x7FF:
        raise argparse.ArgumentTypeError("CAN ID must be in 0x000..0x7FF")
    return value


def _one_or_seven(values: list[T], name: str) -> list[T]:
    if len(values) == 1:
        return values * MOTOR_COUNT
    if len(values) == MOTOR_COUNT:
        return values
    raise ValueError(f"{name} expects one value or exactly {MOTOR_COUNT} values")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Send Damiao position-velocity (PV/POS_VEL) frames to seven motors "
            "over dm-serial"
        )
    )
    parser.add_argument("--serial-port", default="/dev/ttyACM0")
    parser.add_argument("--serial-baud", type=int, default=1_000_000)
    parser.add_argument(
        "--models",
        nargs="+",
        default=["4310"],
        help="one shared model or seven models",
    )
    parser.add_argument(
        "--motor-ids",
        nargs=MOTOR_COUNT,
        type=_parse_can_id,
        default=list(range(0x01, 0x08)),
    )
    parser.add_argument(
        "--feedback-ids",
        nargs=MOTOR_COUNT,
        type=_parse_can_id,
        default=list(range(0x11, 0x18)),
    )
    parser.add_argument(
        "--positions",
        nargs=MOTOR_COUNT,
        type=float,
        required=True,
        help="seven target positions in radians (required)",
    )
    parser.add_argument(
        "--velocity-limits",
        nargs="+",
        type=float,
        default=[1.0],
        help="one shared limit or seven limits in rad/s",
    )
    parser.add_argument("--cycles", type=int, default=50)
    parser.add_argument("--period-ms", type=int, default=20)
    parser.add_argument("--tx-gap-us", type=int, default=120)
    parser.add_argument("--ensure-timeout-ms", type=int, default=1000)
    return parser


def run(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    if args.serial_baud <= 0:
        parser.error("--serial-baud must be positive")
    if any(value < 0 for value in args.velocity_limits):
        parser.error("--velocity-limits must not contain negative values")
    if args.cycles <= 0:
        parser.error("--cycles must be positive")
    if args.period_ms < 0:
        parser.error("--period-ms must not be negative")
    if args.tx_gap_us < 0:
        parser.error("--tx-gap-us must not be negative")
    if args.ensure_timeout_ms <= 0:
        parser.error("--ensure-timeout-ms must be positive")

    try:
        models = _one_or_seven(args.models, "--models")
        velocity_limits = _one_or_seven(
            args.velocity_limits, "--velocity-limits"
        )
    except ValueError as error:
        parser.error(str(error))

    print(
        f"transport=dm-serial serial={args.serial_port}@{args.serial_baud} "
        f"mode=pos-vel motor_ids={[hex(value) for value in args.motor_ids]} "
        f"feedback_ids={[hex(value) for value in args.feedback_ids]} "
        f"positions={args.positions} velocity_limits={velocity_limits}"
    )

    motors = []
    try:
        with Controller.from_dm_serial(args.serial_port, args.serial_baud) as controller:
            try:
                controller.set_tx_gap_us(args.tx_gap_us)
                motors = [
                    controller.add_damiao_motor(motor_id, feedback_id, model)
                    for motor_id, feedback_id, model in zip(
                        args.motor_ids, args.feedback_ids, models
                    )
                ]
                controller.enable_all()
                time.sleep(0.2)
                for motor in motors:
                    motor.ensure_mode(Mode.POS_VEL, args.ensure_timeout_ms)

                for cycle in range(1, args.cycles + 1):
                    # Each cycle sends one PV/POS_VEL frame to each of the seven motors.
                    for motor, position, velocity_limit in zip(
                        motors, args.positions, velocity_limits
                    ):
                        motor.send_pos_vel(position, velocity_limit)

                    if args.period_ms > 0:
                        time.sleep(args.period_ms / 1000.0)

                    states = []
                    for motor_id, motor in zip(args.motor_ids, motors):
                        state = motor.get_state()
                        if state is None:
                            states.append(f"0x{motor_id:X}:no-feedback")
                        else:
                            states.append(
                                f"0x{motor_id:X}:pos={state.pos:+.3f},"
                                f"vel={state.vel:+.3f},status={state.status_code}"
                            )
                    print(f"cycle={cycle} " + " | ".join(states))
            finally:
                try:
                    controller.disable_all()
                except Exception:
                    pass
                for motor in motors:
                    motor.close()
    except Exception as error:
        print(f"FAIL {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(run())

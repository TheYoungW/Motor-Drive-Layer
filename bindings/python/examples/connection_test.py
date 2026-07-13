#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Sequence

from motorbridge import Controller


def _parse_id(text: str) -> int:
    value = int(text, 0)
    if not 0 <= value <= 0x7FF:
        raise argparse.ArgumentTypeError("CAN ID must be in 0x000..0x7FF")
    return value


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Safe Damiao connectivity test: disable one motor and verify fresh feedback"
        )
    )
    parser.add_argument(
        "--transport",
        choices=["socketcan", "socketcanfd", "dm-serial", "dm-device"],
        default="socketcan",
    )
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--serial-port", default="/dev/ttyACM0")
    parser.add_argument("--serial-baud", type=int, default=921600)
    parser.add_argument("--dm-device-type", default="usb2canfd-dual")
    parser.add_argument("--dm-channel", default="0")
    parser.add_argument("--model", default="4340P")
    parser.add_argument("--motor-id", type=_parse_id, default=0x01)
    parser.add_argument("--feedback-id", type=_parse_id, default=0x11)
    parser.add_argument("--attempts", type=int, default=5)
    parser.add_argument("--interval-ms", type=int, default=100)
    parser.add_argument("--max-age-ms", type=int, default=1000)
    return parser


def _open_controller(args: argparse.Namespace) -> Controller:
    if args.transport == "socketcan":
        return Controller(args.channel)
    if args.transport == "socketcanfd":
        return Controller.from_socketcanfd(args.channel)
    if args.transport == "dm-serial":
        return Controller.from_dm_serial(args.serial_port, args.serial_baud)
    return Controller.from_dm_device(args.dm_device_type, args.dm_channel)


def run(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    if args.serial_baud <= 0:
        parser.error("--serial-baud must be positive")
    if args.attempts <= 0:
        parser.error("--attempts must be positive")
    if args.interval_ms < 0:
        parser.error("--interval-ms must not be negative")
    if args.max_age_ms <= 0:
        parser.error("--max-age-ms must be positive")

    print(
        f"transport={args.transport} model={args.model} "
        f"motor_id=0x{args.motor_id:X} feedback_id=0x{args.feedback_id:X}"
    )

    try:
        with _open_controller(args) as controller:
            motor = controller.add_damiao_motor(
                args.motor_id, args.feedback_id, args.model
            )
            try:
                motor.disable()

                for attempt in range(1, args.attempts + 1):
                    motor.request_feedback()
                    if args.interval_ms:
                        time.sleep(args.interval_ms / 1000.0)

                    state = motor.get_state()
                    stats = motor.get_feedback_stats()
                    age_ms = stats.age_ns / 1_000_000.0
                    if state is None or not stats.has_feedback:
                        print(f"attempt={attempt} feedback=missing")
                        continue
                    if age_ms > args.max_age_ms:
                        print(f"attempt={attempt} feedback=stale age_ms={age_ms:.2f}")
                        continue

                    print(
                        f"PASS feedback_count={stats.update_count} age_ms={age_ms:.2f} "
                        f"status={state.status_code} pos={state.pos:+.3f} "
                        f"vel={state.vel:+.3f} torq={state.torq:+.3f} "
                        f"mos_c={state.t_mos:.1f} rotor_c={state.t_rotor:.1f}"
                    )
                    if state.status_code != 0:
                        print(
                            f"FAIL motor returned non-disabled status {state.status_code}",
                            file=sys.stderr,
                        )
                        return 1
                    return 0

                print(
                    "FAIL no fresh feedback; verify the transport, interface, IDs, "
                    "model, wiring, and power",
                    file=sys.stderr,
                )
                return 1
            finally:
                try:
                    motor.disable()
                except Exception:
                    pass
                motor.close()
    except Exception as error:
        print(f"FAIL {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(run())

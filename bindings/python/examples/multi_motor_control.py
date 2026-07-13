#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time

from motorbridge import Controller, Mode


def _split(values: list[str]) -> list[str]:
    return [item.strip() for value in values for item in value.split(",") if item.strip()]


def _ids(values: list[str]) -> list[int]:
    return [int(value, 0) for value in _split(values)]


def _floats(values: list[str]) -> list[float]:
    return [float(value) for value in _split(values)]


def _per_motor(
    values: list[float], count: int, name: str, default: float | None = None
) -> list[float]:
    if not values:
        if default is None:
            raise ValueError(f"{name} is required for the selected mode")
        return [default] * count
    if len(values) == 1:
        return values * count
    if len(values) == count:
        return values
    raise ValueError(f"{name} expects one value or {count} values")


def _feedback_ids(motor_ids: list[int], values: list[str]) -> list[int]:
    parsed = _ids(values)
    if not parsed:
        return [0x10 + (motor_id & 0x0F) for motor_id in motor_ids]
    if len(parsed) == 1:
        return parsed * len(motor_ids)
    if len(parsed) == len(motor_ids):
        return parsed
    raise ValueError("feedback-id expects one value or one value per motor")


def main() -> None:
    parser = argparse.ArgumentParser(description="Multi-motor SocketCAN control example")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--model", default="4310")
    parser.add_argument("--ids", nargs="+", required=True, help="for example: 0x04 0x07")
    parser.add_argument(
        "--feedback-ids",
        nargs="+",
        default=[],
        help="defaults to 0x10 + low nibble of each motor ID",
    )
    parser.add_argument(
        "--mode", choices=["mit", "pos-vel", "vel", "force-pos"], default="mit"
    )
    parser.add_argument("--pos", nargs="+", default=[])
    parser.add_argument("--vel", nargs="+", default=[])
    parser.add_argument("--kp", nargs="+", default=["2.0"])
    parser.add_argument("--kd", nargs="+", default=["0.2"])
    parser.add_argument("--tau", nargs="+", default=["0.0"])
    parser.add_argument("--vlim", nargs="+", default=["1.0"])
    parser.add_argument("--ratio", nargs="+", default=["0.2"])
    parser.add_argument("--loop", type=int, default=100)
    parser.add_argument("--dt-ms", type=int, default=20)
    parser.add_argument("--ensure-timeout-ms", type=int, default=1000)
    args = parser.parse_args()

    motor_ids = _ids(args.ids)
    if not motor_ids:
        raise ValueError("at least one motor ID is required")
    feedback_ids = _feedback_ids(motor_ids, args.feedback_ids)
    count = len(motor_ids)

    position_required = args.mode in {"mit", "pos-vel", "force-pos"}
    positions = _per_motor(
        _floats(args.pos), count, "pos", None if position_required else 0.0
    )
    velocities = _per_motor(_floats(args.vel), count, "vel", 0.0)
    kp = _per_motor(_floats(args.kp), count, "kp", 2.0)
    kd = _per_motor(_floats(args.kd), count, "kd", 0.2)
    torque = _per_motor(_floats(args.tau), count, "tau", 0.0)
    velocity_limits = _per_motor(_floats(args.vlim), count, "vlim", 1.0)
    ratios = _per_motor(_floats(args.ratio), count, "ratio", 0.2)
    modes = {
        "mit": Mode.MIT,
        "pos-vel": Mode.POS_VEL,
        "vel": Mode.VEL,
        "force-pos": Mode.FORCE_POS,
    }

    print(
        f"channel={args.channel} model={args.model} mode={args.mode} "
        f"ids={[hex(value) for value in motor_ids]} "
        f"feedback_ids={[hex(value) for value in feedback_ids]}"
    )

    motors = []
    with Controller(args.channel) as controller:
        try:
            motors = [
                controller.add_damiao_motor(motor_id, feedback_id, args.model)
                for motor_id, feedback_id in zip(motor_ids, feedback_ids)
            ]
            controller.enable_all()
            time.sleep(0.2)
            for motor in motors:
                motor.ensure_mode(modes[args.mode], args.ensure_timeout_ms)

            for iteration in range(max(1, args.loop)):
                for index, motor in enumerate(motors):
                    if args.mode == "mit":
                        motor.send_mit(
                            positions[index],
                            velocities[index],
                            kp[index],
                            kd[index],
                            torque[index],
                        )
                    elif args.mode == "pos-vel":
                        motor.send_pos_vel(positions[index], velocity_limits[index])
                    elif args.mode == "vel":
                        motor.send_vel(velocities[index])
                    else:
                        motor.send_force_pos(
                            positions[index], velocity_limits[index], ratios[index]
                        )

                states = []
                for motor_id, motor in zip(motor_ids, motors):
                    state = motor.get_state()
                    if state is None:
                        states.append(f"0x{motor_id:X}:no-feedback")
                    else:
                        states.append(
                            f"0x{motor_id:X}:pos={state.pos:+.3f},vel={state.vel:+.3f}"
                        )
                print(f"#{iteration} " + " | ".join(states))
                if args.dt_ms > 0:
                    time.sleep(args.dt_ms / 1000.0)
        finally:
            try:
                controller.disable_all()
            except Exception:
                pass
            for motor in motors:
                motor.close()


if __name__ == "__main__":
    main()

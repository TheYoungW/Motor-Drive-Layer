#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from rebot_config import JointConfig, load_rebot_dm_config


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Smoke-test C++ motor_abi with reBot Damiao hardware configuration."
    )
    parser.add_argument(
        "--rebot-config",
        type=Path,
        default=Path("../reBotArm_control_py/config/rebotarm_dm.yaml"),
        help="Path to rebotarm_dm.yaml.",
    )
    parser.add_argument(
        "--transport",
        choices=["dm-serial", "dm-device"],
        default="dm-serial",
        help="Hardware transport to use.",
    )
    parser.add_argument("--channel", help="Serial port for dm-serial; defaults to YAML channel.")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--dm-device-type", default="usb2canfd-dual")
    parser.add_argument("--dm-channel", default="0")
    parser.add_argument("--timeout-s", type=float, default=2.0)
    parser.add_argument("--enable", action="store_true", help="Enable motors during smoke test.")
    parser.add_argument("--dry-run", action="store_true", help="Validate setup without opening hardware.")
    parser.add_argument(
        "--run-hardware",
        action="store_true",
        help="Actually open hardware. Required unless --dry-run is set.",
    )
    args = parser.parse_args()

    channel, joints = load_rebot_dm_config(args.rebot_config)
    channel = args.channel or channel
    if not joints:
        raise RuntimeError(f"no joints found in {args.rebot_config}")
    non_damiao = [j.name for j in joints if j.vendor != "damiao"]
    if non_damiao:
        raise RuntimeError(f"non-Damiao joints in config: {non_damiao}")

    import motorbridge

    print(f"abi_version={motorbridge.abi_version()}")
    print(f"transport={args.transport}")
    print(f"joints={len(joints)}")
    print(f"config={args.rebot_config}")
    if args.transport == "dm-serial":
        print(f"channel={channel}")
    else:
        print(f"dm_device_type={args.dm_device_type}")
        print(f"dm_channel={args.dm_channel}")

    if args.dry_run:
        print("dry_run=ok")
        return 0
    if not args.run_hardware:
        raise RuntimeError("pass --run-hardware to open adapters, or pass --dry-run for validation only")

    ctrl = _open_controller(args, channel)
    try:
        motors = [_add_motor(ctrl, joint) for joint in joints]
        if args.enable:
            ctrl.enable_all()
            time.sleep(0.2)

        deadline = time.monotonic() + args.timeout_s
        seen: dict[str, object] = {}
        while time.monotonic() < deadline and len(seen) < len(motors):
            for joint, motor in zip(joints, motors):
                try:
                    motor.request_feedback()
                except Exception as exc:
                    print(f"{joint.name}:request_feedback_error={exc}")
            ctrl.poll_feedback_once()
            for joint, motor in zip(joints, motors):
                state = motor.get_state()
                if state is not None:
                    seen[joint.name] = state
            time.sleep(0.01)

        for joint in joints:
            state = seen.get(joint.name)
            if state is None:
                print(f"{joint.name}:feedback=missing")
            else:
                print(
                    f"{joint.name}:feedback=ok status={state.status_code} "
                    f"pos={state.pos:+.4f} vel={state.vel:+.4f} torq={state.torq:+.4f}"
                )

        if len(seen) != len(joints):
            raise RuntimeError(f"feedback missing for {len(joints) - len(seen)} joint(s)")
        print("hardware_smoke=ok")
        return 0
    finally:
        if args.enable:
            try:
                ctrl.disable_all()
            except Exception as exc:
                print(f"disable_all_error={exc}")
        try:
            ctrl.shutdown()
        finally:
            ctrl.close()


def _open_controller(args: argparse.Namespace, channel: str):
    from motorbridge import Controller

    if args.transport == "dm-serial":
        return Controller.from_dm_serial(channel, args.baud)
    return Controller.from_dm_device(args.dm_device_type, args.dm_channel)


def _add_motor(ctrl, joint: JointConfig):
    return ctrl.add_damiao_motor(joint.motor_id, joint.feedback_id, joint.model)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"hardware_smoke=failed: {exc}", file=sys.stderr)
        raise SystemExit(1)

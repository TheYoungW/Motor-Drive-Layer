#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from rebot_config import load_rebot_dm_config


REQUIRED_CONTROLLER_METHODS = [
    "from_dm_serial",
    "from_dm_device",
    "add_damiao_motor",
    "enable_all",
    "disable_all",
    "poll_feedback_once",
    "shutdown",
    "close",
]

REQUIRED_MOTOR_METHODS = [
    "ensure_mode",
    "send_mit",
    "send_pos_vel",
    "send_vel",
    "write_register_f32",
    "request_feedback",
    "get_state",
    "set_zero_position",
    "enable",
    "disable",
]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Verify that Python motorbridge loads the C++ ABI and matches reBot DM needs."
    )
    parser.add_argument(
        "--rebot-root",
        type=Path,
        default=Path("../reBotArm_control_py"),
        help="Path to the reBotArm_control_py checkout.",
    )
    parser.add_argument(
        "--skip-hardware-probes",
        action="store_true",
        help="Skip constructor probes that may touch local hardware/OS transports.",
    )
    args = parser.parse_args()

    import motorbridge
    from motorbridge import Controller, Motor

    caps = motorbridge.abi_capabilities()
    version = motorbridge.abi_version()
    if version != "0.2.0-cpp":
        raise RuntimeError(f"expected C++ ABI 0.2.0-cpp, got {version!r}")
    if caps.get("vendors") != ["damiao"]:
        raise RuntimeError(f"expected Damiao-only ABI, got vendors={caps.get('vendors')!r}")
    expected_transports = ["socketcan", "socketcanfd", "dm-serial", "dm-device"]
    if caps.get("transports") != expected_transports:
        raise RuntimeError(f"expected transports={expected_transports!r}, got {caps.get('transports')!r}")

    missing_controller = [m for m in REQUIRED_CONTROLLER_METHODS if not hasattr(Controller, m)]
    missing_motor = [m for m in REQUIRED_MOTOR_METHODS if not hasattr(Motor, m)]
    if missing_controller or missing_motor:
        raise RuntimeError(
            f"missing Python API methods: Controller={missing_controller}, Motor={missing_motor}"
        )

    rebot_cfg = args.rebot_root / "config" / "rebotarm_dm.yaml"
    channel, joints = load_rebot_dm_config(rebot_cfg)
    non_damiao = [j.name for j in joints if j.vendor != "damiao"]
    if non_damiao:
        raise RuntimeError(f"reBot DM config contains non-Damiao joints: {non_damiao}")

    print(f"abi_version={version}")
    print(f"transports={','.join(caps['transports'])}")
    print("python_api=ok")
    print(f"rebot_dm_channel={channel}")
    print(f"rebot_dm_joints={len(joints)}")

    if not args.skip_hardware_probes:
        _probe_transports()

    return 0


def _probe_transports() -> None:
    from motorbridge import Controller

    try:
        ctrl = Controller("can0")
    except Exception as exc:
        print(f"socketcan_probe={type(exc).__name__}:{str(exc)[:120]}")
    else:
        ctrl.close()
        print("socketcan_probe=opened")

    try:
        ctrl = Controller.from_dm_device("usb2canfd-dual", "0")
    except Exception as exc:
        print(f"dm_device_probe={type(exc).__name__}:{str(exc)[:120]}")
    else:
        ctrl.close()
        print("dm_device_probe=opened")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"compat_check=failed: {exc}", file=sys.stderr)
        raise SystemExit(1)

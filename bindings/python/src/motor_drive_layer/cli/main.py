from __future__ import annotations

import argparse
import sys
import time

from .. import get_version
from ..core import Controller
from ..models import Mode
from .platform_hints import preflight_can_runtime


DAMIAO_MODEL_LIMITS: dict[str, tuple[float, float, float]] = {
    "3507": (12.566, 50.0, 5.0),
    "4310": (12.5, 30.0, 10.0),
    "4310P": (12.5, 50.0, 10.0),
    "4340": (12.5, 10.0, 28.0),
    "4340P": (12.5, 10.0, 28.0),
    "4340_v20": (12.5, 20.0, 28.0),
    "6006": (12.5, 45.0, 20.0),
    "8006": (12.5, 45.0, 40.0),
    "8009": (12.5, 45.0, 54.0),
    "10010L": (12.5, 25.0, 200.0),
    "10010": (12.5, 20.0, 200.0),
    "H3510": (12.5, 280.0, 1.0),
    "G6215": (12.5, 45.0, 10.0),
    "H6220": (12.5, 45.0, 10.0),
    "JH11": (12.5, 10.0, 12.0),
    "6248P": (12.566, 20.0, 120.0),
}


def _parse_id(text: str | int) -> int:
    return int(str(text), 0)


def _parse_rids(text: str) -> list[int]:
    return [int(x.strip(), 0) for x in text.split(",") if x.strip()]


def _mode_to_enum(mode: str) -> Mode:
    return {
        "mit": Mode.MIT,
        "pos-vel": Mode.POS_VEL,
        "vel": Mode.VEL,
        "force-pos": Mode.FORCE_POS,
    }[mode]


def _open_controller(args: argparse.Namespace) -> Controller:
    transport = str(args.transport)
    if transport == "dm-serial":
        return Controller.from_dm_serial(args.serial_port, args.serial_baud)
    if transport == "dm-device":
        dm_channel = args.dm_channel
        if dm_channel is None:
            dm_channel = "0"
        return Controller.from_dm_device(args.dm_device_type, dm_channel)
    if transport in ("auto", "socketcan"):
        return Controller(args.channel)
    if transport == "socketcanfd":
        return Controller.from_socketcanfd(args.channel)
    raise ValueError(f"unsupported transport: {transport}")


def _add_common_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--vendor", default="damiao", choices=["damiao"], help="only Damiao is supported")
    p.add_argument("--channel", default="can0", help="SocketCAN/CAN-FD channel")
    p.add_argument(
        "--transport",
        default="auto",
        choices=["auto", "socketcan", "socketcanfd", "dm-serial", "dm-device"],
        help="transport backend",
    )
    p.add_argument("--serial-port", default="/dev/ttyACM0", help="serial port for dm-serial")
    p.add_argument("--serial-baud", type=int, default=1_000_000, help="baud rate for dm-serial")
    p.add_argument("--dm-device-type", default="usb2canfd-dual", help="DM_Device SDK adapter type")
    p.add_argument("--dm-channel", default="0", help="DM_Device SDK channel number")
    p.add_argument("--model", default="4340", help="Damiao model name/hint")
    p.add_argument("--motor-id", default="0x01", help="command/device ID")
    p.add_argument("--feedback-id", default="0x11", help="feedback ID")


def _add_run_args(p: argparse.ArgumentParser) -> None:
    p.add_argument(
        "--mode",
        default="mit",
        choices=["enable", "disable", "clear-error", "mit", "pos-vel", "vel", "force-pos", "read-param", "write-param"],
    )
    p.add_argument("--pos", type=float, default=0.0)
    p.add_argument("--vel", type=float, default=0.0)
    p.add_argument("--kp", type=float, default=2.0)
    p.add_argument("--kd", type=float, default=1.0)
    p.add_argument("--tau", type=float, default=0.0)
    p.add_argument("--vlim", type=float, default=1.0)
    p.add_argument("--ratio", type=float, default=0.5)
    p.add_argument("--loop", type=int, default=1)
    p.add_argument("--dt-ms", type=int, default=20)
    p.add_argument("--ensure-mode", type=int, default=1)
    p.add_argument("--ensure-timeout-ms", type=int, default=1000)
    p.add_argument("--ensure-strict", type=int, default=0)
    p.add_argument("--print-state", type=int, default=1)
    p.add_argument("--feedback-timeout-ms", type=int, default=50)
    p.add_argument("--verify-model", type=int, default=1)
    p.add_argument("--verify-timeout-ms", type=int, default=500)
    p.add_argument("--verify-tol", type=float, default=0.2)
    p.add_argument("--param-id", default="10")
    p.add_argument("--param-value", default="")
    p.add_argument("--type", default="f32", choices=["u32", "f32"])
    p.add_argument("--store", type=int, default=0)
    p.add_argument("--set-motor-id", default="")
    p.add_argument("--set-feedback-id", default="")
    p.add_argument("--verify-id", type=int, default=1)


def _verify_damiao_model(motor, model: str, timeout_ms: int, tol: float) -> None:
    expected = DAMIAO_MODEL_LIMITS.get(model)
    if expected is None:
        raise ValueError(f"unknown Damiao model in catalog: {model}")
    pmax = motor.get_register_f32(21, timeout_ms)
    vmax = motor.get_register_f32(22, timeout_ms)
    tmax = motor.get_register_f32(23, timeout_ms)
    if all(abs(a - b) <= tol for a, b in zip((pmax, vmax, tmax), expected)):
        print(f"[ok] model handshake passed: {model}")
        return
    raise RuntimeError(
        f"model mismatch: {model} expects {expected}, device reports "
        f"({pmax:.3f}, {vmax:.3f}, {tmax:.3f})"
    )


def _add_motor(ctrl: Controller, args: argparse.Namespace):
    return ctrl.add_damiao_motor(_parse_id(args.motor_id), _parse_id(args.feedback_id), args.model)


def _id_set_command(args: argparse.Namespace) -> None:
    motor_id = _parse_id(args.motor_id)
    feedback_id = _parse_id(args.feedback_id)
    new_motor_id = _parse_id(args.new_motor_id) if args.new_motor_id else motor_id
    new_feedback_id = _parse_id(args.new_feedback_id) if args.new_feedback_id else feedback_id
    with _open_controller(args) as ctrl:
        motor = ctrl.add_damiao_motor(motor_id, feedback_id, args.model)
        try:
            if new_feedback_id != feedback_id:
                motor.write_register_u32(7, new_feedback_id)
                print(f"write rid=7 (MST_ID) <= 0x{new_feedback_id:X}")
            if new_motor_id != motor_id:
                motor.write_register_u32(8, new_motor_id)
                print(f"write rid=8 (ESC_ID) <= 0x{new_motor_id:X}")
            if args.store:
                motor.store_parameters()
                print("store_parameters sent")
        finally:
            motor.close()


def _id_dump_command(args: argparse.Namespace) -> None:
    with _open_controller(args) as ctrl:
        motor = _add_motor(ctrl, args)
        try:
            for rid in _parse_rids(args.rids):
                try:
                    value = motor.get_register_u32(rid, args.timeout_ms)
                    print(f"rid={rid:>3} (u32) = {value} (0x{value:X})")
                except Exception as e_u32:
                    try:
                        value_f = motor.get_register_f32(rid, args.timeout_ms)
                        print(f"rid={rid:>3} (f32) = {value_f:.6f}")
                    except Exception:
                        print(f"rid={rid:>3} read failed: {e_u32}")
        finally:
            motor.close()


def _read_param_command(args: argparse.Namespace) -> None:
    with _open_controller(args) as ctrl:
        motor = _add_motor(ctrl, args)
        try:
            param_id = _parse_id(args.param_id)
            if args.type == "u32":
                value = motor.damiao_get_param_u32(param_id, args.timeout_ms)
            else:
                value = motor.damiao_get_param_f32(param_id, args.timeout_ms)
            print(f"param_id=0x{param_id:X} type={args.type} value={value}")
        finally:
            motor.close()


def _write_param_command(args: argparse.Namespace) -> None:
    with _open_controller(args) as ctrl:
        motor = _add_motor(ctrl, args)
        try:
            param_id = _parse_id(args.param_id)
            if args.type == "u32":
                motor.damiao_write_param_u32(param_id, int(args.value, 0))
                verify = motor.damiao_get_param_u32(param_id, args.timeout_ms) if args.verify else None
            else:
                motor.damiao_write_param_f32(param_id, float(args.value))
                verify = motor.damiao_get_param_f32(param_id, args.timeout_ms) if args.verify else None
            if args.store:
                motor.store_parameters()
            print(f"param_id=0x{param_id:X} type={args.type} value={args.value} verify={verify}")
        finally:
            motor.close()


def _run_command(args: argparse.Namespace) -> None:
    if args.set_motor_id or args.set_feedback_id:
        id_args = argparse.Namespace(**vars(args))
        id_args.new_motor_id = args.set_motor_id
        id_args.new_feedback_id = args.set_feedback_id
        id_args.verify = args.verify_id
        _id_set_command(id_args)
        return

    with _open_controller(args) as ctrl:
        motor = _add_motor(ctrl, args)
        try:
            if args.mode == "read-param":
                param_id = _parse_id(args.param_id)
                if args.type == "u32":
                    value = motor.damiao_get_param_u32(param_id, args.verify_timeout_ms)
                else:
                    value = motor.damiao_get_param_f32(param_id, args.verify_timeout_ms)
                print(f"param_id=0x{param_id:X} type={args.type} value={value}")
                return
            if args.mode == "write-param":
                if args.param_value == "":
                    raise ValueError("write-param requires --param-value")
                param_id = _parse_id(args.param_id)
                if args.type == "u32":
                    motor.damiao_write_param_u32(param_id, int(args.param_value, 0))
                    verify = motor.damiao_get_param_u32(param_id, args.verify_timeout_ms)
                else:
                    motor.damiao_write_param_f32(param_id, float(args.param_value))
                    verify = motor.damiao_get_param_f32(param_id, args.verify_timeout_ms)
                if args.store:
                    motor.store_parameters()
                print(f"param_id=0x{param_id:X} type={args.type} value={args.param_value} verify={verify}")
                return
            if args.verify_model and args.mode not in ("enable", "disable"):
                _verify_damiao_model(motor, args.model, args.verify_timeout_ms, args.verify_tol)

            if args.mode not in ("enable", "disable", "clear-error"):
                ctrl.enable_all()
                time.sleep(0.3)
            if args.ensure_mode and args.mode in ("mit", "pos-vel", "vel", "force-pos"):
                try:
                    motor.ensure_mode(_mode_to_enum(args.mode), args.ensure_timeout_ms)
                except Exception:
                    if args.ensure_strict:
                        raise

            for i in range(args.loop):
                if args.mode == "enable":
                    motor.enable()
                elif args.mode == "disable":
                    motor.disable()
                elif args.mode == "clear-error":
                    motor.clear_error()
                    print("[ok] clear-error requested")
                    break
                elif args.mode == "mit":
                    motor.send_mit(args.pos, args.vel, args.kp, args.kd, args.tau)
                elif args.mode == "pos-vel":
                    motor.send_pos_vel(args.pos, args.vlim)
                elif args.mode == "vel":
                    motor.send_vel(args.vel)
                elif args.mode == "force-pos":
                    motor.send_force_pos(args.pos, args.vlim, args.ratio)
                if args.print_state:
                    print(
                        f"#{i} state={motor.request_fresh_state(args.feedback_timeout_ms)}"
                    )
                time.sleep(max(args.dt_ms, 0) / 1000.0)
        finally:
            motor.close()


def _scan_command(args: argparse.Namespace) -> None:
    start_id = _parse_id(args.start_id)
    end_id = _parse_id(args.end_id)
    found = 0
    for mid in range(start_id, end_id + 1):
        scan_args = argparse.Namespace(**vars(args))
        scan_args.motor_id = str(mid)
        scan_args.feedback_id = hex(_parse_id(args.feedback_base) + (mid & 0x0F))
        try:
            with _open_controller(scan_args) as ctrl:
                motor = _add_motor(ctrl, scan_args)
                try:
                    state = motor.request_fresh_state(max(args.timeout_ms, 10))
                    found += 1
                    print(f"[hit] id=0x{mid:X} feedback_id={scan_args.feedback_id} state={state}")
                finally:
                    motor.close()
        except Exception as e:
            print(f"[.. ] id=0x{mid:X} no reply: {e}")
    print(f"[scan] done vendor=damiao hits={found}")


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="motor-drive-layer Damiao-only Python CLI", allow_abbrev=False)
    p.add_argument("-v", "--version", action="version", version=f"motor-drive-layer {get_version()}")
    sub = p.add_subparsers(dest="command")

    run = sub.add_parser("run", help="send Damiao control commands")
    _add_common_args(run)
    _add_run_args(run)

    scan = sub.add_parser("scan", help="scan Damiao motor IDs")
    _add_common_args(scan)
    scan.add_argument("--start-id", default="0x01")
    scan.add_argument("--end-id", default="0x10")
    scan.add_argument("--feedback-base", default="0x10")
    scan.add_argument("--timeout-ms", type=int, default=80)
    scan.set_defaults(dm_channel=None)

    dump = sub.add_parser("id-dump", help="read Damiao ID/mode/limit registers")
    _add_common_args(dump)
    dump.add_argument("--timeout-ms", type=int, default=500)
    dump.add_argument("--rids", default="7,8,9,10,21,22,23")

    set_id = sub.add_parser("id-set", help="change Damiao ESC_ID/MST_ID")
    _add_common_args(set_id)
    set_id.add_argument("--new-motor-id", default="")
    set_id.add_argument("--new-feedback-id", default="")
    set_id.add_argument("--store", type=int, default=1)
    set_id.add_argument("--verify", type=int, default=1)
    set_id.add_argument("--timeout-ms", type=int, default=800)

    for name, writer in (("damiao-read-param", False), ("damiao-write-param", True)):
        cmd = sub.add_parser(name, help=name.replace("-", " "))
        _add_common_args(cmd)
        cmd.add_argument("--param-id", required=True)
        cmd.add_argument("--type", required=True, choices=["u32", "f32"])
        cmd.add_argument("--timeout-ms", type=int, default=500)
        if writer:
            cmd.add_argument("--value", required=True)
            cmd.add_argument("--verify", type=int, default=1)
            cmd.add_argument("--store", type=int, default=0)

    return p


def _parse_args() -> argparse.Namespace:
    parser = _build_parser()
    if len(sys.argv) == 1:
        parser.print_help()
        raise SystemExit(0)
    if (
        len(sys.argv) > 1
        and sys.argv[1].startswith("--")
        and sys.argv[1] not in {"--help", "--version"}
    ):
        legacy = argparse.ArgumentParser(description="motor-drive-layer Damiao-only Python CLI", allow_abbrev=False)
        _add_common_args(legacy)
        _add_run_args(legacy)
        args = legacy.parse_args()
        args.command = "run"
        return args
    args = parser.parse_args()
    if args.command is None:
        args.command = "run"
    return args


def main() -> None:
    args = _parse_args()
    try:
        if args.transport == "auto" and getattr(args, "serial_port", None) != "/dev/ttyACM0":
            args.transport = "dm-serial"
        hint = preflight_can_runtime("motor-drive-layer-cli", args.transport, args.channel)
        if hint:
            raise RuntimeError(hint)

        if args.command == "run":
            _run_command(args)
        elif args.command == "scan":
            _scan_command(args)
        elif args.command == "id-dump":
            _id_dump_command(args)
        elif args.command == "id-set":
            _id_set_command(args)
        elif args.command == "damiao-read-param":
            _read_param_command(args)
        elif args.command == "damiao-write-param":
            _write_param_command(args)
        else:
            raise ValueError(f"unknown command: {args.command}")
    except Exception as e:
        print(f"error: {e}", file=sys.stderr)
        raise SystemExit(1) from e

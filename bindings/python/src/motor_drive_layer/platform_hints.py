from __future__ import annotations

import shlex
import sys
from pathlib import Path


_CAN_TRANSPORTS = {"auto", "socketcan", "socketcanfd"}


def option_is_provided(argv: list[str], name: str) -> bool:
    flag = f"--{name}"
    return any(tok == flag or tok.startswith(f"{flag}=") for tok in argv)


def parse_option_arg(argv: list[str], name: str, default: str = "") -> str:
    flag = f"--{name}"
    value = default
    i = 0
    while i < len(argv):
        tok = argv[i]
        if tok == flag:
            if i + 1 < len(argv):
                value = argv[i + 1]
                i += 2
                continue
            break
        if tok.startswith(f"{flag}="):
            value = tok.split("=", 1)[1]
            i += 1
            continue
        i += 1
    return str(value or default).strip()


def parse_transport_arg(argv: list[str], default: str = "auto") -> str:
    return parse_option_arg(argv, "transport", default).lower()


def parse_vendor_arg(argv: list[str], default: str = "damiao") -> str:
    return parse_option_arg(argv, "vendor", default).lower()


def should_skip_runtime_preflight(argv: list[str]) -> bool:
    return any(a in ("-h", "--help", "-v", "--version") for a in argv)


def effective_transport_for_preflight(argv: list[str], default: str = "auto") -> str:
    transport = parse_transport_arg(argv, default)
    vendor = parse_vendor_arg(argv, "damiao")
    if (
        transport == "auto"
        and vendor == "damiao"
        and option_is_provided(argv, "serial-port")
    ):
        return "dm-serial"
    return transport


def should_infer_dm_serial_transport(argv: list[str], default: str = "auto") -> bool:
    return (
        parse_transport_arg(argv, default) == "auto"
        and effective_transport_for_preflight(argv, default) == "dm-serial"
    )


def with_inferred_dm_serial_transport(argv: list[str], default: str = "auto") -> list[str]:
    args = list(argv)
    if not should_infer_dm_serial_transport(args, default):
        return args

    out: list[str] = []
    replaced = False
    i = 0
    while i < len(args):
        tok = args[i]
        if tok == "--transport":
            out.extend(["--transport", "dm-serial"])
            replaced = True
            i += 2 if i + 1 < len(args) else 1
            continue
        if tok.startswith("--transport="):
            out.append("--transport=dm-serial")
            replaced = True
            i += 1
            continue
        out.append(tok)
        i += 1

    if not replaced:
        out = ["--transport", "dm-serial", *out]
    return out


def transport_needs_can_runtime(transport: str) -> bool:
    return str(transport or "auto").strip().lower() in _CAN_TRANSPORTS


def is_linux() -> bool:
    return sys.platform.startswith("linux")


def parse_channel_arg(argv: list[str], default: str = "can0") -> str:
    return parse_option_arg(argv, "channel", default)


def unsupported_socketcan_hint(tool_name: str, transport: str) -> str:
    return (
        f"[{tool_name}] transport '{transport}' is only available on Linux.\n"
        "Use dm-serial or dm-device on this platform, or run SocketCAN on Linux."
    )


def _linux_iface_state(iface: str) -> tuple[bool, str]:
    base = Path("/sys/class/net") / iface
    if not base.exists():
        return False, "missing"
    state_file = base / "operstate"
    try:
        state = state_file.read_text(encoding="utf-8").strip().lower()
    except OSError:
        return True, "unknown"
    return True, state


def linux_socketcan_hint(tool_name: str, channel: str, transport: str = "socketcan") -> str:
    raw = str(channel or "can0").strip()
    iface = raw.split("@", 1)[0].strip() or "can0"
    shell_iface = shlex.quote(iface)
    normalized_transport = str(transport or "socketcan").strip().lower()
    suffix_tip = ""
    if "@" in raw:
        suffix_tip = (
            f"\nDetected channel '{raw}'. Linux SocketCAN should not include '@bitrate'. "
            f"Use plain interface name like '{iface}'."
        )

    if normalized_transport == "socketcanfd":
        setup = (
            "Example CAN-FD setup (adjust both bitrates for your hardware):\n"
            f"  sudo ip link set {shell_iface} down\n"
            f"  sudo ip link set {shell_iface} type can bitrate 1000000 "
            "dbitrate 5000000 fd on\n"
            f"  sudo ip link set {shell_iface} up\n"
            f"  ip -details link show {shell_iface}"
        )
    else:
        setup = (
            "Example classic CAN setup (adjust bitrate for your hardware):\n"
            f"  sudo ip link set {shell_iface} down\n"
            f"  sudo ip link set {shell_iface} type can bitrate 1000000\n"
            f"  sudo ip link set {shell_iface} up\n"
            f"  ip -details link show {shell_iface}"
        )

    return (
        f"[{tool_name}] Linux CAN interface not ready: {raw}\n"
        "For CAN transport, bring up the SocketCAN interface first, then retry.\n"
        "Quick checks:\n"
        f"  ip link show {shell_iface}\n"
        f"{setup}\n"
        "CANable/candleLight devices must use gs_usb-compatible firmware and appear as a "
        "SocketCAN interface before these commands can work."
        f"{suffix_tip}"
    )


def preflight_can_runtime(tool_name: str, transport: str, channel: str) -> str | None:
    if not transport_needs_can_runtime(transport):
        return None

    if is_linux():
        iface = str(channel or "can0").strip().split("@", 1)[0].strip() or "can0"
        exists, state = _linux_iface_state(iface)
        if (not exists) or state in {"down", "dormant", "lowerlayerdown", "notpresent"}:
            return linux_socketcan_hint(tool_name, channel, transport)
        return None

    return unsupported_socketcan_hint(tool_name, transport)

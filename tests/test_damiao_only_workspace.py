#!/usr/bin/env python3
"""Verify this fork keeps only Damiao runtime crates in the Rust workspace."""

from pathlib import Path
import sys
import tomllib


ROOT = Path(__file__).resolve().parents[1]
ALLOWED_MEMBERS = {
    "motor_core",
    "motor_vendors/damiao",
    "motor_cli",
    "motor_abi",
}
REMOVED_VENDOR_MARKERS = (
    "robstride",
    "robstride_cia402",
    "robstride_mit",
    "myactuator",
    "hexfellow",
    "hightorque",
    "template",
)


def main() -> int:
    data = tomllib.loads((ROOT / "Cargo.toml").read_text())
    members = set(data["workspace"]["members"])
    unexpected = sorted(members - ALLOWED_MEMBERS)
    missing = sorted(ALLOWED_MEMBERS - members)

    errors = []
    if unexpected:
        errors.append(f"unexpected workspace members: {unexpected}")
    if missing:
        errors.append(f"missing workspace members: {missing}")

    vendor_dirs = {p.name for p in (ROOT / "motor_vendors").iterdir() if p.is_dir()}
    removed_left = sorted(marker for marker in REMOVED_VENDOR_MARKERS if marker in vendor_dirs)
    if removed_left:
        errors.append(f"removed vendor directories still present: {removed_left}")

    if errors:
        print("\n".join(errors), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

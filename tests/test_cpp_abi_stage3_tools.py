#!/usr/bin/env python3
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REBOT_ROOT = ROOT.parent / "reBotArm_control_py"


def run_tool(*args: str) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env["PYTHONPATH"] = str(ROOT / "bindings" / "python" / "src")
    return subprocess.run(
        [sys.executable, *args],
        cwd=ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    compat = ROOT / "cpp_damiao" / "tools" / "verify_python_compat.py"
    smoke = ROOT / "cpp_damiao" / "tools" / "rebot_hardware_smoke.py"
    require(compat.exists(), "verify_python_compat.py should exist")
    require(smoke.exists(), "rebot_hardware_smoke.py should exist")

    compat_run = run_tool(
        str(compat),
        "--rebot-root",
        str(REBOT_ROOT),
        "--skip-hardware-probes",
    )
    require(compat_run.returncode == 0, compat_run.stdout)
    require("abi_version=0.2.0-cpp" in compat_run.stdout, compat_run.stdout)
    require("rebot_dm_joints=7" in compat_run.stdout, compat_run.stdout)
    require("python_api=ok" in compat_run.stdout, compat_run.stdout)

    dry_run = run_tool(
        str(smoke),
        "--rebot-config",
        str(REBOT_ROOT / "config" / "rebotarm_dm.yaml"),
        "--dry-run",
    )
    require(dry_run.returncode == 0, dry_run.stdout)
    require("dry_run=ok" in dry_run.stdout, dry_run.stdout)
    require("joints=7" in dry_run.stdout, dry_run.stdout)
    require("transport=dm-serial" in dry_run.stdout, dry_run.stdout)


if __name__ == "__main__":
    main()

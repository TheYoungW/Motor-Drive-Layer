#!/usr/bin/env python3
"""Run a guarded 20-point right-arm Cartesian speed sweep on Yunyi hardware."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

from arx_d_can import ArxDCanDualArm

from verify_cartesian_smoothness import temperatures


DEFAULT_SPEEDS = (
    50.0, 20.0, 80.0, 35.0, 65.0,
    15.0, 90.0, 45.0, 75.0, 25.0,
    100.0, 55.0, 70.0, 30.0, 85.0,
    40.0, 60.0, 22.5, 95.0, 27.5,
)


def temperature_values(snapshot: dict[str, object]) -> list[float]:
    values: list[float] = []
    for side in ("left", "right"):
        arm = snapshot[side]
        for kind in ("mos_c", "rotor_c"):
            values.extend(value for value in arm[kind] if value is not None)
    return values


def verify_disabled_and_temperature() -> tuple[float, dict[str, object]]:
    robot = ArxDCanDualArm(control_mode="pv", with_grippers=True)
    try:
        robot.connect()
        state = robot.read_state()
        enabled = (*state.left.arm.enabled, *state.right.arm.enabled)
        if any(value is not False for value in enabled):
            raise RuntimeError(
                f"post-run motor power is not confirmed disabled: {enabled}"
            )
        snapshot = temperatures(robot)
        values = temperature_values(snapshot)
        if not values:
            raise RuntimeError("post-run motor temperatures are unavailable")
        return max(values), snapshot
    finally:
        robot.disconnect()


def speed_label(speed: float) -> str:
    return f"{speed:05.1f}".replace(".", "p")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--motion", choices=("linear", "circular"), default="circular")
    parser.add_argument("--temperature-limit", type=float, default=70.0)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--hold-seconds", type=float, default=1.0)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    verifier = Path(__file__).with_name("verify_cartesian_smoothness.py")
    manifest: dict[str, object] = {
        "motion": args.motion,
        "speeds": list(DEFAULT_SPEEDS),
        "runs": [],
    }
    manifest_path = args.output_dir / "manifest.json"

    for run_index, speed in enumerate(DEFAULT_SPEEDS, start=1):
        label = speed_label(speed)
        result_path = args.output_dir / f"{run_index:02d}_{args.motion}_speed_{label}.json"
        trace_path = args.output_dir / f"{run_index:02d}_{args.motion}_speed_{label}_trace.csv"
        environment = os.environ.copy()
        environment["ARTICORE_RUNTIME_CONTROL_TRACE"] = str(trace_path)
        command = [
            sys.executable,
            str(verifier),
            "--motion", args.motion,
            "--speed", str(speed),
            "--timeout", str(args.timeout),
            "--hold-seconds", str(args.hold_seconds),
            "--i-understand-the-right-arm-will-move",
            "--output", str(result_path),
        ]
        print(
            f"RUN {run_index:02d}/{len(DEFAULT_SPEEDS)} "
            f"motion={args.motion} speed={speed:.1f}%",
            flush=True,
        )
        started = time.monotonic()
        completed = subprocess.run(
            command,
            env=environment,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        wall_s = time.monotonic() - started
        if completed.returncode != 0:
            print(completed.stdout, flush=True)
            raise RuntimeError(
                f"run {run_index} failed with exit code {completed.returncode}"
            )
        result = json.loads(result_path.read_text())
        maximum_temperature, post_snapshot = verify_disabled_and_temperature()
        native = result.get("native_status", {})
        record = {
            "run": run_index,
            "speed_percent": speed,
            "result": str(result_path),
            "trace": str(trace_path),
            "wall_s": wall_s,
            "settled_s": result["metrics"]["settled_s"],
            "sample_hz": result["metrics"]["running_sample_hz"],
            "motion_state": native.get("state"),
            "max_temperature_c": maximum_temperature,
            "post_temperatures": post_snapshot,
        }
        manifest["runs"].append(record)
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
        print(
            f"PASS {run_index:02d} speed={speed:.1f}% "
            f"settled={record['settled_s']:.2f}s "
            f"sample={record['sample_hz']:.1f}Hz "
            f"max_temp={maximum_temperature:.1f}C disabled=confirmed",
            flush=True,
        )
        if maximum_temperature >= args.temperature_limit:
            raise RuntimeError(
                f"temperature {maximum_temperature:.1f}C reached the "
                f"{args.temperature_limit:.1f}C sweep limit"
            )
        time.sleep(1.0)

    print(f"SWEEP_COMPLETE manifest={manifest_path}", flush=True)


if __name__ == "__main__":
    main()

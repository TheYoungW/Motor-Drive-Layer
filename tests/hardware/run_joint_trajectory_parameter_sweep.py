#!/usr/bin/env python3
"""Run a guarded 20-point native joint-trajectory duration/PV-V sweep."""

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


# Fixed interleaving avoids making duration or V indistinguishable from test
# order, passive joint temperature, or USB/CAN run-to-run drift.
CASES = (
    (1.50, 2.0), (2.50, 1.5), (1.00, 3.0), (2.00, 2.5),
    (1.25, 2.0), (2.50, 3.0), (1.00, 1.5), (2.00, 2.0),
    (1.50, 3.0), (1.25, 1.5), (2.50, 2.5), (1.00, 2.5),
    (2.00, 1.5), (1.50, 2.5), (1.25, 3.0), (2.50, 2.0),
    (1.00, 2.0), (2.00, 3.0), (1.50, 1.5), (1.25, 2.5),
)


def maximum_temperature(snapshot: dict[str, object]) -> float:
    values = [
        float(value)
        for side in ("left", "right")
        for kind in ("mos_c", "rotor_c")
        for value in snapshot[side][kind]
        if value is not None
    ]
    if not values:
        raise RuntimeError("motor temperatures are unavailable")
    return max(values)


def verify_disabled() -> tuple[float, dict[str, object]]:
    robot = ArxDCanDualArm(control_mode="pv", with_grippers=True)
    try:
        robot.connect()
        state = robot.read_state()
        enabled = (*state.left.arm.enabled, *state.right.arm.enabled)
        if any(value is not False for value in enabled):
            raise RuntimeError(f"motors are not confirmed disabled: {enabled}")
        snapshot = temperatures(robot)
        return maximum_temperature(snapshot), snapshot
    finally:
        robot.disconnect()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--temperature-limit", type=float, default=70.0)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--hold-seconds", type=float, default=2.0)
    parser.add_argument("--limit", type=int, default=len(CASES))
    args = parser.parse_args()
    cases = CASES[:args.limit]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    verifier = Path(__file__).with_name(
        "verify_joint_trajectory_smoothness.py"
    )
    manifest: dict[str, object] = {"cases": [], "runs": []}
    manifest_path = args.output_dir / "manifest.json"
    for run, (duration, velocity) in enumerate(cases, start=1):
        label = f"d{duration:g}_v{velocity:g}".replace(".", "p")
        result_path = args.output_dir / f"{run:02d}_{label}.json"
        trace_path = args.output_dir / f"{run:02d}_{label}_trace.csv"
        environment = os.environ.copy()
        environment["ARTICORE_RUNTIME_CONTROL_TRACE"] = str(trace_path)
        command = [
            sys.executable, str(verifier),
            "--duration", str(duration),
            "--pv-v", str(velocity),
            "--timeout", str(args.timeout),
            "--hold-seconds", str(args.hold_seconds),
            "--output", str(result_path),
            "--i-understand-both-arms-will-move",
        ]
        print(
            f"RUN {run:02d}/{len(cases)} duration={duration:g}s "
            f"PV_V={velocity:g}rad/s",
            flush=True,
        )
        started = time.monotonic()
        completed = subprocess.run(
            command, env=environment, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        )
        wall_s = time.monotonic() - started
        if completed.returncode != 0:
            print(completed.stdout, flush=True)
            raise RuntimeError(f"parameter run {run} failed")
        data = json.loads(result_path.read_text())
        max_temp, snapshot = verify_disabled()
        metrics = data["metrics"]
        record = {
            "run": run,
            "duration_s": duration,
            "pv_velocity_limit_rad_s": velocity,
            "result": str(result_path.resolve()),
            "trace": str(trace_path.resolve()),
            "wall_s": wall_s,
            "completed_s": metrics["completed_s"],
            "sample_hz": metrics["sample_rate_hz"],
            "max_temperature_c": max_temp,
            "post_temperatures": snapshot,
        }
        manifest["cases"].append([duration, velocity])
        manifest["runs"].append(record)
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
        print(
            f"PASS completed={metrics['completed_s']:.3f}s "
            f"sample={metrics['sample_rate_hz']:.1f}Hz "
            f"temp={max_temp:.1f}C disabled=confirmed",
            flush=True,
        )
        if max_temp >= args.temperature_limit:
            raise RuntimeError(
                f"temperature {max_temp:.1f}C reached the limit"
            )
        time.sleep(0.5)
    print(f"SWEEP_COMPLETE manifest={manifest_path}", flush=True)


if __name__ == "__main__":
    main()

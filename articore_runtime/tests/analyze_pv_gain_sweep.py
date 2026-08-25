#!/usr/bin/env python3
import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np


TARGETS = {
    2: (0.257305145, -0.189402580),
    17: (-0.308804512, -0.254253387),
    19: (-0.369077682, 0.123407364),
    25: (0.237468719, 0.423247337),
    27: (0.378232956, 0.448042870),
    58: (-0.946249962, 0.043679237),
    61: (-0.790608406, -0.174525261),
    84: (-0.627717972, -0.347715378),
    86: (-0.861181259, -0.277523041),
    87: (-0.887121201, -0.201609612),
    98: (0.135614395, 0.530822754),
    118: (-0.000190735, -0.000190735),
    131: (0.019264221, 0.014305115),
    154: (0.248149872, -0.020790100),
}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("trace", type=Path)
    parser.add_argument("--discard-seconds", type=float, default=2.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    groups = defaultdict(list)
    with args.trace.open(newline="") as stream:
        for row in csv.DictReader(stream):
            key = (row["family"], row["profile"], int(row["point_index"]))
            groups[key].append(row)

    metrics = []
    for (family, profile, point), rows in groups.items():
        tested_joint = int(rows[0]["tested_joint"])
        timestamp = np.asarray([float(row["timestamp_ns"]) for row in rows]) * 1e-9
        elapsed = timestamp - timestamp[0]
        keep = elapsed >= args.discard_seconds
        elapsed = elapsed[keep]
        elapsed -= elapsed[0]
        joints = (1, 2) if tested_joint == 12 else (tested_joint,)
        for joint in joints:
            q = np.asarray([float(row[f"q_left_j{joint}"]) for row in rows])[keep]
            velocity = np.asarray(
                [float(row[f"dq_left_j{joint}"]) for row in rows]
            )[keep]
            slope, intercept = np.polyfit(elapsed, q, 1)
            detrended = q - (slope * elapsed + intercept)
            target = TARGETS[point][joint - 1]
            metrics.append(
                {
                    "tested_joint": joint,
                    "family": family,
                    "profile": profile,
                    "point": point,
                    "samples": int(q.size),
                    "mean_error_rad": float(np.mean(q) - target),
                    "detrended_rms_rad": float(np.sqrt(np.mean(detrended**2))),
                    "peak_to_peak_rad": float(np.ptp(q)),
                    "velocity_rms_rad_s": float(np.sqrt(np.mean(velocity**2))),
                    "velocity_peak_rad_s": float(np.max(np.abs(velocity))),
                    "linear_drift_rad_s": float(slope),
                }
            )

    if args.output:
        args.output.write_text(json.dumps(metrics, indent=2) + "\n")

    columns = (
        "tested_joint",
        "family",
        "profile",
        "point",
        "detrended_rms_rad",
        "peak_to_peak_rad",
        "velocity_rms_rad_s",
        "velocity_peak_rad_s",
        "mean_error_rad",
    )
    print(" ".join(f"{column:>21}" for column in columns))
    for metric in metrics:
        values = []
        for column in columns:
            value = metric[column]
            values.append(
                f"{value:>21}" if isinstance(value, str) else f"{value:>21.9f}"
            )
        print(" ".join(values))


if __name__ == "__main__":
    main()

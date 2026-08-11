from __future__ import annotations

import argparse
import json
import os
import statistics
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Sequence

from .core import Controller


@dataclass(frozen=True)
class MotorSpec:
    """One motor used by the feedback/reconnect diagnostic."""

    channel: int
    motor_id: int
    feedback_id: int
    model: str


def parse_motor_spec(value: str) -> MotorSpec:
    """Parse ``CHANNEL:MOTOR_ID:FEEDBACK_ID:MODEL`` (integers accept hex)."""

    parts = value.split(":", 3)
    if len(parts) != 4 or not parts[3]:
        raise argparse.ArgumentTypeError(
            "motor must be CHANNEL:MOTOR_ID:FEEDBACK_ID:MODEL, for example "
            "0:0x09:0x19:4310"
        )
    try:
        channel, motor_id, feedback_id = (int(part, 0) for part in parts[:3])
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid motor integer: {exc}") from exc
    if channel < 0 or not 0 <= motor_id <= 0xFFFF or not 0 <= feedback_id <= 0xFFFF:
        raise argparse.ArgumentTypeError(
            "channel must be non-negative and CAN IDs must be in 0..=0xffff"
        )
    return MotorSpec(channel, motor_id, feedback_id, parts[3])


def _resource_snapshot() -> dict[str, int | None]:
    def count(path: str) -> int | None:
        try:
            return sum(1 for _ in Path(path).iterdir())
        except OSError:
            return None

    return {
        "file_descriptors": count("/proc/self/fd"),
        "threads": count("/proc/self/task"),
    }


def _percentile(values: Sequence[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * fraction)]


def _latency_summary(values: Sequence[float]) -> dict[str, float]:
    if not values:
        return {"mean_ms": 0.0, "p50_ms": 0.0, "p99_ms": 0.0, "max_ms": 0.0}
    return {
        "mean_ms": statistics.fmean(values),
        "p50_ms": _percentile(values, 0.50),
        "p99_ms": _percentile(values, 0.99),
        "max_ms": max(values),
    }


def run_dm_device_feedback_stress(
    motors: Sequence[MotorSpec],
    *,
    device: str = "usb2canfd-dual",
    bitrate: int = 1_000_000,
    data_bitrate: int = 5_000_000,
    iterations: int = 1_000,
    reconnect_cycles: int = 2,
    timeout_ms: int = 50,
) -> dict[str, object]:
    """Run feedback-only load and same-process reconnect checks.

    This diagnostic never enables a motor and never sends a control command.
    It opens every requested channel, registers the fixed motor layout, calls
    ``request_feedback_all()``, then closes and reopens the device.
    """

    specs = tuple(motors)
    if not specs:
        raise ValueError("motors must not be empty")
    if iterations <= 0 or reconnect_cycles <= 0 or timeout_ms < 0:
        raise ValueError("iterations and reconnect_cycles must be positive; timeout_ms >= 0")
    grouped: dict[int, list[MotorSpec]] = {}
    for spec in specs:
        grouped.setdefault(spec.channel, []).append(spec)
    for channel, channel_specs in grouped.items():
        ids = [spec.motor_id for spec in channel_specs]
        feedback_ids = [spec.feedback_id for spec in channel_specs]
        if len(ids) != len(set(ids)) or len(feedback_ids) != len(set(feedback_ids)):
            raise ValueError(f"channel {channel} has duplicate motor or feedback IDs")

    started = time.monotonic()
    resources_before = _resource_snapshot()
    resources_after_close: list[dict[str, int | None]] = []
    channel_results: dict[str, dict[str, object]] = {
        str(channel): {"requests": 0, "failures": 0, "latencies_ms": [], "errors": []}
        for channel in sorted(grouped)
    }
    capabilities: dict[str, object] = {}

    for _cycle in range(reconnect_cycles):
        controllers: dict[int, Controller] = {}
        motor_handles = []
        try:
            for channel in sorted(grouped):
                controller = Controller.from_dm_device(
                    device=device,
                    channel=channel,
                    bitrate=bitrate,
                    data_bitrate=data_bitrate,
                )
                controllers[channel] = controller
                capabilities[str(channel)] = asdict(controller.transport_capabilities())
                for spec in grouped[channel]:
                    motor_handles.append(
                        controller.add_damiao_motor(spec.motor_id, spec.feedback_id, spec.model)
                    )

            for _iteration in range(iterations):
                for channel, controller in controllers.items():
                    result = channel_results[str(channel)]
                    request_started = time.monotonic_ns()
                    try:
                        controller.request_feedback_all(timeout_ms)
                    except Exception as exc:
                        result["failures"] = int(result["failures"]) + 1
                        errors = result["errors"]
                        if isinstance(errors, list) and len(errors) < 20:
                            errors.append(str(exc))
                    finally:
                        latencies = result["latencies_ms"]
                        if isinstance(latencies, list):
                            latencies.append(
                                (time.monotonic_ns() - request_started) / 1_000_000
                            )
                        result["requests"] = int(result["requests"]) + 1
        finally:
            for motor in reversed(motor_handles):
                motor.close()
            for controller in reversed(tuple(controllers.values())):
                try:
                    controller.close_bus()
                finally:
                    controller.close()
            resources_after_close.append(_resource_snapshot())

    total_failures = 0
    summarized_channels: dict[str, object] = {}
    for channel, raw in channel_results.items():
        failures = int(raw["failures"])
        total_failures += failures
        latencies = raw["latencies_ms"]
        assert isinstance(latencies, list)
        summarized_channels[channel] = {
            "requests": raw["requests"],
            "failures": failures,
            **_latency_summary(latencies),
            "errors": raw["errors"],
        }

    warm = resources_after_close[0]
    final = resources_after_close[-1]
    resource_growth_after_warmup = {
        name: None if warm[name] is None or final[name] is None else final[name] - warm[name]
        for name in warm
    }
    return {
        "device": device,
        "bitrate": bitrate,
        "data_bitrate": data_bitrate,
        "iterations_per_cycle": iterations,
        "reconnect_cycles": reconnect_cycles,
        "timeout_ms": timeout_ms,
        "elapsed_s": time.monotonic() - started,
        "capabilities": capabilities,
        "channels": summarized_channels,
        "total_failures": total_failures,
        "resources": {
            "before": resources_before,
            "after_each_close": resources_after_close,
            "growth_after_warmup": resource_growth_after_warmup,
        },
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Feedback-only DM_Device stress and same-process reconnect diagnostic; "
            "does not enable or command motors"
        )
    )
    parser.add_argument(
        "--motor",
        action="append",
        type=parse_motor_spec,
        required=True,
        metavar="CHANNEL:MOTOR_ID:FEEDBACK_ID:MODEL",
    )
    parser.add_argument("--device", default="usb2canfd-dual")
    parser.add_argument("--bitrate", type=int, default=1_000_000)
    parser.add_argument("--data-bitrate", type=int, default=5_000_000)
    parser.add_argument("--iterations", type=int, default=1_000)
    parser.add_argument("--reconnect-cycles", type=int, default=2)
    parser.add_argument("--timeout-ms", type=int, default=50)
    parser.add_argument("--output", type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    result = run_dm_device_feedback_stress(
        args.motor,
        device=args.device,
        bitrate=args.bitrate,
        data_bitrate=args.data_bitrate,
        iterations=args.iterations,
        reconnect_cycles=args.reconnect_cycles,
        timeout_ms=args.timeout_ms,
    )
    rendered = json.dumps(result, ensure_ascii=False, indent=2)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + os.linesep, encoding="utf-8")
    return 1 if result["total_failures"] else 0


if __name__ == "__main__":
    raise SystemExit(main())

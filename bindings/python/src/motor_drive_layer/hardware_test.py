from __future__ import annotations

import argparse
import sys
import threading
import time
from dataclasses import dataclass
from importlib import resources
from pathlib import Path
from typing import Callable, Sequence

from .hardware_config import ControllerConfig, HardwareConfig, load_hardware_config
from .models import MotorState


class BenchmarkError(RuntimeError):
    pass


@dataclass(frozen=True)
class JointBenchmarkResult:
    name: str
    motor_id: int
    feedback_id: int
    expected_feedback: int
    received_feedback: int
    feedback_percent: float
    feedback_age_ns: int
    state: MotorState


@dataclass(frozen=True)
class ControllerBenchmarkResult:
    name: str
    port: str
    ticks: int
    late_ticks: int
    max_tick_late_us: float
    max_cycle_submit_us: float
    joints: tuple[JointBenchmarkResult, ...]


@dataclass(frozen=True)
class HardwareBenchmarkReport:
    frequency_hz: int
    duration_s: float
    controllers: tuple[ControllerBenchmarkResult, ...]


@dataclass
class _OpenedController:
    config: ControllerConfig
    controller: object
    motors: list[object]
    baseline_counts: list[int]


@dataclass
class _WorkerStats:
    ticks: int = 0
    late_ticks: int = 0
    max_tick_late_us: float = 0.0
    max_cycle_submit_us: float = 0.0


ControllerFactory = Callable[[str, int], object]


def _default_controller_factory(port: str, baud: int):
    from .core import Controller

    return Controller.from_dm_serial(port, baud)


def _sleep_until_ns(deadline_ns: int) -> None:
    remaining_ns = deadline_ns - time.perf_counter_ns()
    if remaining_ns > 0:
        time.sleep(remaining_ns / 1_000_000_000)


def _open_controller(config: ControllerConfig, factory: ControllerFactory) -> _OpenedController:
    controller = factory(config.port, config.baud)
    motors: list[object] = []
    try:
        controller.set_tx_gap_us(config.tx_gap_us)
        for joint in config.joints:
            motors.append(
                controller.add_damiao_motor(joint.motor_id, joint.feedback_id, joint.model)
            )
        controller.disable_all()
        return _OpenedController(config, controller, motors, [])
    except Exception:
        try:
            controller.shutdown()
        finally:
            for motor in motors:
                motor.close()
            controller.close()
        raise


def _establish_disabled_baseline(opened: _OpenedController, settle_s: float) -> None:
    for motor in opened.motors:
        motor.request_feedback()
    if settle_s > 0:
        time.sleep(settle_s)

    baseline_counts: list[int] = []
    for joint, motor in zip(opened.config.joints, opened.motors):
        state = motor.get_state()
        stats = motor.get_feedback_stats()
        if state is None or not stats.has_feedback:
            raise BenchmarkError(
                f"{opened.config.name}.{joint.name} did not return initial feedback"
            )
        if state.status_code != 0:
            raise BenchmarkError(
                f"{opened.config.name}.{joint.name} is not disabled "
                f"(status={state.status_code})"
            )
        baseline_counts.append(stats.update_count)
    opened.baseline_counts = baseline_counts


def _run_worker(
    opened: _OpenedController,
    frequency_hz: int,
    ticks: int,
    start_ns: int,
    stats: _WorkerStats,
    errors: list[BaseException],
    errors_lock: threading.Lock,
) -> None:
    period_ns = max(1, round(1_000_000_000 / frequency_hz))
    try:
        for tick in range(ticks):
            scheduled_ns = start_ns + tick * period_ns
            _sleep_until_ns(scheduled_ns)
            tick_start_ns = time.perf_counter_ns()
            late_us = max(0.0, (tick_start_ns - scheduled_ns) / 1_000.0)
            if late_us > 100.0:
                stats.late_ticks += 1
            stats.max_tick_late_us = max(stats.max_tick_late_us, late_us)

            for motor in opened.motors:
                motor.send_pos_vel(0.0, 0.0)
            cycle_submit_us = (time.perf_counter_ns() - tick_start_ns) / 1_000.0
            stats.max_cycle_submit_us = max(stats.max_cycle_submit_us, cycle_submit_us)
            stats.ticks += 1
    except BaseException as error:
        with errors_lock:
            errors.append(error)


def _cleanup(opened_controllers: Sequence[_OpenedController]) -> None:
    for opened in opened_controllers:
        try:
            opened.controller.shutdown()
        except Exception:
            pass
        finally:
            for motor in opened.motors:
                try:
                    motor.close()
                except Exception:
                    pass
            try:
                opened.controller.close()
            except Exception:
                pass


def run_hardware_test(
    config: HardwareConfig,
    *,
    controller_factory: ControllerFactory | None = None,
) -> HardwareBenchmarkReport:
    factory = controller_factory or _default_controller_factory
    opened_controllers: list[_OpenedController] = []
    worker_stats: list[_WorkerStats] = []
    try:
        for controller_config in config.controllers:
            opened_controllers.append(_open_controller(controller_config, factory))

        settle_s = config.benchmark.feedback_settle_ms / 1_000.0
        for opened in opened_controllers:
            _establish_disabled_baseline(opened, settle_s)

        target_ticks = max(
            1, round(config.benchmark.frequency_hz * config.benchmark.duration_s)
        )
        start_ns = time.perf_counter_ns() + 20_000_000
        errors: list[BaseException] = []
        errors_lock = threading.Lock()
        threads: list[threading.Thread] = []
        for opened in opened_controllers:
            stats = _WorkerStats()
            worker_stats.append(stats)
            thread = threading.Thread(
                target=_run_worker,
                args=(
                    opened,
                    config.benchmark.frequency_hz,
                    target_ticks,
                    start_ns,
                    stats,
                    errors,
                    errors_lock,
                ),
                name=f"motor-drive-layer-{opened.config.name}",
            )
            threads.append(thread)
            thread.start()
        for thread in threads:
            thread.join()
        if errors:
            raise BenchmarkError(f"hardware worker failed: {errors[0]}") from errors[0]

        if settle_s > 0:
            time.sleep(settle_s)

        controller_results: list[ControllerBenchmarkResult] = []
        for opened, timing in zip(opened_controllers, worker_stats):
            joint_results: list[JointBenchmarkResult] = []
            for joint, motor, baseline_count in zip(
                opened.config.joints, opened.motors, opened.baseline_counts
            ):
                state = motor.get_state()
                stats = motor.get_feedback_stats()
                if state is None:
                    raise BenchmarkError(
                        f"{opened.config.name}.{joint.name} has no final state"
                    )
                received = max(0, stats.update_count - baseline_count)
                expected = timing.ticks
                percent = 100.0 * received / expected if expected else 0.0
                joint_results.append(
                    JointBenchmarkResult(
                        name=joint.name,
                        motor_id=joint.motor_id,
                        feedback_id=joint.feedback_id,
                        expected_feedback=expected,
                        received_feedback=received,
                        feedback_percent=percent,
                        feedback_age_ns=stats.age_ns,
                        state=state,
                    )
                )
            controller_results.append(
                ControllerBenchmarkResult(
                    name=opened.config.name,
                    port=opened.config.port,
                    ticks=timing.ticks,
                    late_ticks=timing.late_ticks,
                    max_tick_late_us=timing.max_tick_late_us,
                    max_cycle_submit_us=timing.max_cycle_submit_us,
                    joints=tuple(joint_results),
                )
            )
        return HardwareBenchmarkReport(
            frequency_hz=config.benchmark.frequency_hz,
            duration_s=config.benchmark.duration_s,
            controllers=tuple(controller_results),
        )
    finally:
        _cleanup(opened_controllers)


def format_report(report: HardwareBenchmarkReport) -> str:
    lines = [
        f"frequency_hz={report.frequency_hz} duration_s={report.duration_s:g}"
    ]
    for controller in report.controllers:
        lines.append(
            f"controller={controller.name} port={controller.port} ticks={controller.ticks} "
            f"late_ticks={controller.late_ticks} "
            f"max_tick_late_us={controller.max_tick_late_us:.2f} "
            f"max_cycle_submit_us={controller.max_cycle_submit_us:.2f}"
        )
        for joint in controller.joints:
            lines.append(
                f"  joint={joint.name} motor_id=0x{joint.motor_id:X} "
                f"feedback_id=0x{joint.feedback_id:X} "
                f"feedback={joint.received_feedback}/{joint.expected_feedback} "
                f"percent={joint.feedback_percent:.2f} "
                f"age_us={joint.feedback_age_ns / 1_000.0:.2f} "
                f"status={joint.state.status_code} mos_c={joint.state.t_mos:.0f} "
                f"rotor_c={joint.state.t_rotor:.0f}"
            )
    return "\n".join(lines)


def report_is_successful(report: HardwareBenchmarkReport) -> bool:
    return all(
        joint.received_feedback >= joint.expected_feedback
        for controller in report.controllers
        for joint in controller.joints
    )


def run_cli(
    argv: Sequence[str] | None = None,
    *,
    controller_factory: ControllerFactory | None = None,
) -> int:
    parser = argparse.ArgumentParser(
        description="Safe pure-Python Damiao multi-controller hardware benchmark"
    )
    parser.add_argument("--config", help="Python-layer YAML configuration")
    parser.add_argument(
        "--write-example",
        metavar="PATH",
        help="copy the packaged editable YAML example and exit",
    )
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="validate YAML without opening hardware",
    )
    args = parser.parse_args(argv)
    try:
        if args.write_example:
            destination = Path(args.write_example)
            if destination.exists():
                raise BenchmarkError(f"refusing to overwrite existing file: {destination}")
            example = resources.files("motor_drive_layer").joinpath(
                "configs/damiao_dual_arm.yaml"
            )
            destination.write_text(example.read_text(encoding="utf-8"), encoding="utf-8")
            print(f"example_written={destination}")
            return 0
        if not args.config:
            raise BenchmarkError("--config is required unless --write-example is used")
        config = load_hardware_config(args.config)
        if args.validate_only:
            joint_count = sum(len(controller.joints) for controller in config.controllers)
            print(
                f"config=ok controllers={len(config.controllers)} joints={joint_count} "
                f"frequency_hz={config.benchmark.frequency_hz}"
            )
            return 0
        report = run_hardware_test(config, controller_factory=controller_factory)
        print(format_report(report))
        return 0 if report_is_successful(report) else 1
    except Exception as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


def main() -> None:
    raise SystemExit(run_cli())


if __name__ == "__main__":
    main()

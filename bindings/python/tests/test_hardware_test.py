from __future__ import annotations

from dataclasses import dataclass

import pytest

from motor_drive_layer.hardware_config import (
    BenchmarkConfig,
    ControllerConfig,
    HardwareConfig,
    JointConfig,
)
from motor_drive_layer.hardware_config import load_hardware_config
from motor_drive_layer.hardware_test import (
    BenchmarkError,
    report_is_successful,
    run_cli,
    run_hardware_test,
)
from motor_drive_layer.models import FeedbackStats, MotorState


@dataclass
class FakeMotor:
    status_code: int = 0
    update_count: int = 0
    request_count: int = 0
    control_count: int = 0
    closed: bool = False

    def request_feedback(self) -> None:
        self.request_count += 1
        self.update_count += 1

    def send_pos_vel(self, pos: float, vlim: float) -> None:
        assert pos == 0.0
        assert vlim == 0.0
        self.control_count += 1
        self.update_count += 1

    def get_state(self) -> MotorState:
        return MotorState(1, 0x201, self.status_code, 0.0, 0.0, 0.0, 30.0, 28.0)

    def get_feedback_stats(self) -> FeedbackStats:
        return FeedbackStats(True, self.update_count, 10_000)

    def close(self) -> None:
        self.closed = True


class FakeController:
    def __init__(self, port: str, baud: int, *, status_code: int = 0) -> None:
        self.port = port
        self.baud = baud
        self.status_code = status_code
        self.motors: list[FakeMotor] = []
        self.gap_us: int | None = None
        self.disable_calls = 0
        self.shutdown_calls = 0
        self.closed = False

    def add_damiao_motor(self, motor_id: int, feedback_id: int, model: str) -> FakeMotor:
        motor = FakeMotor(status_code=self.status_code)
        self.motors.append(motor)
        return motor

    def set_tx_gap_us(self, gap_us: int) -> None:
        self.gap_us = gap_us

    def disable_all(self) -> None:
        self.disable_calls += 1

    def shutdown(self) -> None:
        self.shutdown_calls += 1

    def close(self) -> None:
        self.closed = True


class DroppingMotor(FakeMotor):
    def send_pos_vel(self, pos: float, vlim: float) -> None:
        self.control_count += 1


class DroppingController(FakeController):
    def add_damiao_motor(self, motor_id: int, feedback_id: int, model: str) -> FakeMotor:
        motor = DroppingMotor(status_code=self.status_code)
        self.motors.append(motor)
        return motor


class ExtraFeedbackMotor(FakeMotor):
    def send_pos_vel(self, pos: float, vlim: float) -> None:
        super().send_pos_vel(pos, vlim)
        self.update_count += 1


class ExtraFeedbackController(FakeController):
    def add_damiao_motor(self, motor_id: int, feedback_id: int, model: str) -> FakeMotor:
        motor = ExtraFeedbackMotor(status_code=self.status_code)
        self.motors.append(motor)
        return motor


def make_config(*, status_port_count: int = 2) -> HardwareConfig:
    controllers = []
    for index in range(status_port_count):
        controllers.append(
            ControllerConfig(
                name=f"arm_{index}",
                port=f"/dev/fake{index}",
                baud=1_000_000,
                tx_gap_us=120,
                joints=(
                    JointConfig("joint1", 1, 0x201, "4340P"),
                    JointConfig("joint2", 2, 0x202, "4340P"),
                ),
            )
        )
    return HardwareConfig(
        schema_version=1,
        benchmark=BenchmarkConfig(frequency_hz=20, duration_s=0.1, feedback_settle_ms=0),
        controllers=tuple(controllers),
    )


def test_runs_each_controller_without_ever_enabling_motors() -> None:
    created: list[FakeController] = []

    def factory(port: str, baud: int) -> FakeController:
        controller = FakeController(port, baud)
        created.append(controller)
        return controller

    report = run_hardware_test(make_config(), controller_factory=factory)

    assert len(report.controllers) == 2
    for controller, result in zip(created, report.controllers):
        assert controller.gap_us == 120
        assert controller.disable_calls >= 1
        assert controller.shutdown_calls == 1
        assert controller.closed
        assert result.ticks == 2
        for motor, joint in zip(controller.motors, result.joints):
            assert motor.control_count == 2
            assert motor.closed
            assert joint.expected_feedback == 2
            assert joint.received_feedback == 2
            assert joint.feedback_percent == 100.0


def test_non_disabled_feedback_aborts_and_still_cleans_up() -> None:
    created: list[FakeController] = []

    def factory(port: str, baud: int) -> FakeController:
        controller = FakeController(port, baud, status_code=1)
        created.append(controller)
        return controller

    with pytest.raises(BenchmarkError, match="not disabled"):
        run_hardware_test(make_config(status_port_count=1), controller_factory=factory)

    assert created[0].disable_calls >= 1
    assert created[0].shutdown_calls == 1
    assert created[0].closed


def test_report_fails_when_any_expected_feedback_is_missing() -> None:
    report = run_hardware_test(
        make_config(status_port_count=1),
        controller_factory=lambda port, baud: DroppingController(port, baud),
    )

    assert not report_is_successful(report)


def test_report_accepts_additional_feedback_frames() -> None:
    report = run_hardware_test(
        make_config(status_port_count=1),
        controller_factory=lambda port, baud: ExtraFeedbackController(port, baud),
    )

    assert report_is_successful(report)
    assert report.controllers[0].joints[0].received_feedback > report.controllers[0].ticks


def test_cli_writes_packaged_editable_example_without_opening_hardware(tmp_path) -> None:
    destination = tmp_path / "my_robot.yaml"

    exit_code = run_cli(
        ["--write-example", str(destination)],
        controller_factory=lambda port, baud: (_ for _ in ()).throw(
            AssertionError("hardware must not open")
        ),
    )

    assert exit_code == 0
    config = load_hardware_config(destination)
    assert len(config.controllers) == 2
    assert config.controllers[0].joints[0].feedback_id == 0x201

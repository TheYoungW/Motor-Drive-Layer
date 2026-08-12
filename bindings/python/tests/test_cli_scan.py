from __future__ import annotations

import argparse
import importlib

import pytest

from motor_drive_layer.models import (
    MotorDiscoveryResult,
    PresencePolicy,
    PresenceState,
)

cli_module = importlib.import_module("motor_drive_layer.cli.main")


class FakeMotor:
    def __init__(self, motor_id: int) -> None:
        self.motor_id = motor_id
        self.closed = False

    def get_state(self) -> str:
        return f"state-{self.motor_id}"

    def close(self) -> None:
        self.closed = True


class FakeScanController:
    def __init__(self, *, fail: bool = False) -> None:
        self.fail = fail
        self.discovery_calls = []
        self.close_bus_calls = 0
        self.close_calls = 0
        self.shutdown_calls = 0
        self.motors: list[FakeMotor] = []

    def discover_damiao_motors(self, candidates, timeout_ms, retries):
        values = tuple(candidates)
        self.discovery_calls.append((values, timeout_ms, retries))
        if self.fail:
            raise RuntimeError("injected discovery failure")
        results = []
        for candidate in values:
            present = candidate.motor_id == 2
            motor = FakeMotor(candidate.motor_id) if present else None
            if motor is not None:
                self.motors.append(motor)
            results.append(
                MotorDiscoveryResult(
                    role=candidate.role,
                    motor_id=candidate.motor_id,
                    feedback_id=candidate.feedback_id,
                    policy=candidate.policy,
                    state=(PresenceState.PRESENT if present else PresenceState.NOT_INSTALLED),
                    motor=motor,
                    reason=None if present else "fresh feedback timed out",
                )
            )
        return tuple(results)

    def close_bus(self) -> None:
        self.close_bus_calls += 1

    def close(self) -> None:
        self.close_calls += 1

    def shutdown(self) -> None:
        self.shutdown_calls += 1


def scan_args(**overrides) -> argparse.Namespace:
    values = {
        "start_id": "0x01",
        "end_id": "0x03",
        "feedback_base": "0x10",
        "model": "4310",
        "timeout_ms": 50,
        "retries": 1,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


def test_scan_uses_one_controller_one_batch_and_read_only_close(monkeypatch, capsys) -> None:
    controller = FakeScanController()
    opens = []
    monkeypatch.setattr(
        cli_module,
        "_open_controller",
        lambda args: opens.append(args) or controller,
    )

    cli_module._scan_command(scan_args())

    assert len(opens) == 1
    assert len(controller.discovery_calls) == 1
    candidates, timeout_ms, retries = controller.discovery_calls[0]
    assert [candidate.motor_id for candidate in candidates] == [1, 2, 3]
    assert [candidate.feedback_id for candidate in candidates] == [0x11, 0x12, 0x13]
    assert all(candidate.policy == PresencePolicy.OPTIONAL for candidate in candidates)
    assert (timeout_ms, retries) == (50, 1)
    assert controller.close_bus_calls == 1
    assert controller.close_calls == 1
    assert controller.shutdown_calls == 0
    assert controller.motors[0].closed
    output = capsys.readouterr().out
    assert "[hit] id=0x2" in output
    assert "hits=1" in output


def test_scan_closes_bus_without_shutdown_when_discovery_fails(monkeypatch) -> None:
    controller = FakeScanController(fail=True)
    monkeypatch.setattr(cli_module, "_open_controller", lambda _args: controller)

    with pytest.raises(RuntimeError, match="injected discovery failure"):
        cli_module._scan_command(scan_args())

    assert controller.close_bus_calls == 1
    assert controller.close_calls == 1
    assert controller.shutdown_calls == 0


def test_scan_rejects_duplicate_feedback_mapping_before_open(monkeypatch) -> None:
    opened = False

    def open_controller(_args):
        nonlocal opened
        opened = True
        return FakeScanController()

    monkeypatch.setattr(cli_module, "_open_controller", open_controller)
    with pytest.raises(ValueError, match="same feedback ID"):
        cli_module._scan_command(scan_args(start_id="0x01", end_id="0x11"))
    assert not opened

from __future__ import annotations

import ctypes

import pytest

import motor_drive_layer.core as core_module
from motor_drive_layer.abi import CFeedbackStats, CState
from motor_drive_layer.core import Controller, Motor
from motor_drive_layer.errors import CallError
from motor_drive_layer.models import FeedbackStats, MotorState


class FakeLib:
    def __init__(self) -> None:
        self.tx_gap_calls: list[tuple[int, int]] = []
        self.batch_feedback_calls: list[tuple[int, int]] = []
        self.fresh_state_calls: list[tuple[int, int]] = []
        self.freed_motors: list[int] = []

    def motor_handle_free(self, ptr: int) -> None:
        self.freed_motors.append(ptr)

    def motor_controller_set_tx_gap_us(self, ptr: int, gap_us: int) -> int:
        self.tx_gap_calls.append((ptr, gap_us))
        return 0

    def motor_controller_request_feedback_all(self, ptr: int, timeout_ms: int) -> int:
        self.batch_feedback_calls.append((ptr, timeout_ms))
        return 0

    def motor_handle_request_fresh_state(self, ptr: int, timeout_ms: int, out_state) -> int:
        self.fresh_state_calls.append((ptr, timeout_ms))
        state = ctypes.cast(out_state, ctypes.POINTER(CState)).contents
        state.has_value = 1
        state.can_id = 3
        state.arbitration_id = 0x203
        state.status_code = 1
        state.pos = 1.25
        state.vel = 0.5
        state.torq = 0.125
        state.t_mos = 31.0
        state.t_rotor = 29.0
        return 0

    def motor_handle_get_feedback_stats(self, ptr: int, out_stats) -> int:
        stats = ctypes.cast(out_stats, ctypes.POINTER(CFeedbackStats)).contents
        stats.has_feedback = 1
        stats.update_count = 42
        stats.age_ns = 125_000
        return 0


class FakeAbi:
    def __init__(self) -> None:
        self.lib = FakeLib()


def test_controller_forwards_configured_tx_gap() -> None:
    controller = Controller.__new__(Controller)
    controller._abi = FakeAbi()
    controller._ptr = 123

    controller.set_tx_gap_us(120)

    assert controller._abi.lib.tx_gap_calls == [(123, 120)]


def test_controller_requests_fresh_feedback_with_one_deadline() -> None:
    controller = Controller.__new__(Controller)
    controller._abi = FakeAbi()
    controller._ptr = 123

    controller.request_feedback_all(75)

    assert controller._abi.lib.batch_feedback_calls == [(123, 75)]


def test_motor_exposes_feedback_count_and_age() -> None:
    motor = Motor.__new__(Motor)
    motor._abi = FakeAbi()
    motor._ptr = 456

    assert motor.get_feedback_stats() == FeedbackStats(
        has_feedback=True,
        update_count=42,
        age_ns=125_000,
    )


def test_motor_requests_and_returns_a_fresh_state() -> None:
    motor = Motor.__new__(Motor)
    motor._abi = FakeAbi()
    motor._ptr = 456

    state = motor.request_fresh_state(80)

    assert motor._abi.lib.fresh_state_calls == [(456, 80)]
    assert state == MotorState(
        can_id=3,
        arbitration_id=0x203,
        status_code=1,
        pos=1.25,
        vel=0.5,
        torq=0.125,
        t_mos=31.0,
        t_rotor=29.0,
    )


def test_motor_rejects_operations_after_parent_controller_closes_but_can_free() -> None:
    controller = Controller.__new__(Controller)
    controller._ptr = None
    motor = Motor.__new__(Motor)
    motor._abi = FakeAbi()
    motor._ptr = 456
    motor._controller = controller

    with pytest.raises(CallError, match="motor controller is closed"):
        motor.get_feedback_stats()

    motor.close()

    assert motor.closed
    assert motor._abi.lib.freed_motors == [456]


def test_motor_keeps_its_parent_controller_and_supports_context_cleanup(monkeypatch) -> None:
    fake_abi = FakeAbi()
    monkeypatch.setattr(core_module, "get_abi", lambda: fake_abi)
    controller = Controller.__new__(Controller)
    controller._ptr = 123

    with Motor(456, controller) as motor:
        assert motor._controller is controller
        assert not motor.closed
        assert not controller.closed

    assert motor.closed
    assert fake_abi.lib.freed_motors == [456]

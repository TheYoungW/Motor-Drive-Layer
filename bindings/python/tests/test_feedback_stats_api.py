from __future__ import annotations

import ctypes

import pytest

import motor_drive_layer.core as core_module
from motor_drive_layer.abi import (
    CFeedbackReport,
    CFeedbackStats,
    CState,
    CTransportCapabilities,
    CTransportCapabilitiesV2,
    CTransportHealth,
)
from motor_drive_layer.core import Controller, Motor
from motor_drive_layer.errors import (
    CallError,
    FeedbackMotorFaultError,
    FeedbackTimeoutError,
    FeedbackTransportError,
    IncompleteFeedbackError,
)
from motor_drive_layer.models import (
    FeedbackStats,
    MotorErrorCode,
    MotorState,
    TransportCapabilities,
    TransportHealth,
)


class FakeLib:
    def __init__(self) -> None:
        self.tx_gap_calls: list[tuple[int, int]] = []
        self.batch_feedback_calls: list[tuple[int, int]] = []
        self.fresh_state_calls: list[tuple[int, int]] = []
        self.freed_motors: list[int] = []
        self.feedback_code = 0
        self.feedback_expected = 8
        self.feedback_received = 8
        self.feedback_missing_ids: tuple[int, ...] = ()

    def motor_handle_free(self, ptr: int) -> None:
        self.freed_motors.append(ptr)

    def motor_controller_set_tx_gap_us(self, ptr: int, gap_us: int) -> int:
        self.tx_gap_calls.append((ptr, gap_us))
        return 0

    def motor_controller_request_feedback_all_ex(
        self, ptr: int, timeout_ms: int, out_report, missing_ids, capacity: int
    ) -> int:
        self.batch_feedback_calls.append((ptr, timeout_ms))
        report = ctypes.cast(out_report, ctypes.POINTER(CFeedbackReport)).contents
        report.timeout_ms = timeout_ms
        report.expected_count = self.feedback_expected
        report.received_count = self.feedback_received
        report.missing_count = len(self.feedback_missing_ids)
        for index, motor_id in enumerate(self.feedback_missing_ids[:capacity]):
            missing_ids[index] = motor_id
        return self.feedback_code

    def motor_controller_request_feedback_all(self, ptr: int, timeout_ms: int) -> int:
        raise AssertionError("Python Controller must not call the legacy feedback ABI")

    def motor_last_error_message(self) -> bytes:
        return b"fresh feedback timed out; missing motor IDs: 15"

    def motor_controller_get_transport_capabilities(self, ptr: int, out_capabilities) -> int:
        capabilities = ctypes.cast(
            out_capabilities, ctypes.POINTER(CTransportCapabilities)
        ).contents
        capabilities.transport = b"dm-device"
        capabilities.max_payload_bytes = 8
        capabilities.channel_count = 2
        capabilities.can_fd = 1
        capabilities.parallel_batches = 1
        capabilities.hardware_rx_timestamps = 0
        capabilities.reconnect = 1
        capabilities.process_session_reuse = 1
        return 0

    def motor_controller_get_transport_capabilities_v2(self, ptr: int, out_capabilities) -> int:
        capabilities = ctypes.cast(
            out_capabilities, ctypes.POINTER(CTransportCapabilitiesV2)
        ).contents
        assert capabilities.struct_size == ctypes.sizeof(CTransportCapabilitiesV2)
        capabilities.transport = b"socketcanfd"
        capabilities.max_payload_bytes = 8
        capabilities.channel_count = 1
        capabilities.can_fd = 1
        capabilities.parallel_batches = 1
        capabilities.hardware_rx_timestamps = 0
        capabilities.reconnect = 1
        capabilities.process_session_reuse = 0
        capabilities.can_fd_brs = 1
        return 0

    def motor_controller_get_transport_health(self, ptr: int, out_health) -> int:
        health = ctypes.cast(out_health, ctypes.POINTER(CTransportHealth)).contents
        health.connected = 1
        health.healthy = 0
        health.tx_frames = 123
        health.rx_frames = 120
        health.send_errors = 1
        health.receive_errors = 2
        health.last_tx_age_ns = 50_000
        health.last_rx_age_ns = (1 << 64) - 1
        health.last_error = b"injected receive failure"
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
        self.has_transport_capabilities = True
        self.has_transport_capabilities_v2 = True
        self.has_transport_health = True
        self.has_structured_feedback_report = True


def test_controller_forwards_configured_tx_gap() -> None:
    controller = Controller.__new__(Controller)
    controller._abi = FakeAbi()
    controller._ptr = 123
    controller._feedback_motor_count = 8

    controller.set_tx_gap_us(120)

    assert controller._abi.lib.tx_gap_calls == [(123, 120)]


def test_controller_requests_fresh_feedback_with_one_deadline() -> None:
    controller = Controller.__new__(Controller)
    controller._abi = FakeAbi()
    controller._ptr = 123

    controller.request_feedback_all(75)

    assert controller._abi.lib.batch_feedback_calls == [(123, 75)]


def test_controller_raises_structured_incomplete_feedback(monkeypatch) -> None:
    fake_abi = FakeAbi()
    fake_abi.lib.feedback_code = 4
    fake_abi.lib.feedback_received = 7
    fake_abi.lib.feedback_missing_ids = (15,)
    monkeypatch.setattr(core_module, "get_abi", lambda: fake_abi)
    controller = Controller.__new__(Controller)
    controller._abi = fake_abi
    controller._ptr = 123
    controller._feedback_motor_count = 8

    with pytest.raises(IncompleteFeedbackError) as captured:
        controller.request_feedback_all(50)

    error = captured.value
    assert error.error_code is MotorErrorCode.FEEDBACK_INCOMPLETE
    assert error.missing_motor_ids == (15,)
    assert error.timeout_ms == 50
    assert error.expected_count == 8
    assert error.received_count == 7
    assert error.report.missing_count == 1


@pytest.mark.parametrize(
    ("code", "expected_type"),
    (
        (2, FeedbackTransportError),
        (3, FeedbackTimeoutError),
        (5, FeedbackMotorFaultError),
    ),
)
def test_controller_maps_stable_feedback_error_codes(
    monkeypatch, code: int, expected_type: type[CallError]
) -> None:
    fake_abi = FakeAbi()
    fake_abi.lib.feedback_code = code
    fake_abi.lib.feedback_received = 0
    fake_abi.lib.feedback_missing_ids = tuple(range(1, 9))
    monkeypatch.setattr(core_module, "get_abi", lambda: fake_abi)
    controller = Controller.__new__(Controller)
    controller._abi = fake_abi
    controller._ptr = 123
    controller._feedback_motor_count = 8

    with pytest.raises(expected_type) as captured:
        controller.request_feedback_all(50)

    assert int(captured.value.error_code) == code
    assert captured.value.report.expected_count == 8
    assert captured.value.report.received_count == 0
    assert captured.value.report.missing_motor_ids == tuple(range(1, 9))


def test_controller_exposes_per_transport_capabilities() -> None:
    controller = Controller.__new__(Controller)
    controller._abi = FakeAbi()
    controller._ptr = 123

    assert controller.transport_capabilities() == TransportCapabilities(
        transport="socketcanfd",
        max_payload_bytes=8,
        channel_count=1,
        can_fd=True,
        parallel_batches=True,
        hardware_rx_timestamps=False,
        reconnect=True,
        process_session_reuse=False,
        can_fd_brs=True,
    )


def test_controller_exposes_runtime_transport_health() -> None:
    controller = Controller.__new__(Controller)
    controller._abi = FakeAbi()
    controller._ptr = 123

    assert controller.transport_health() == TransportHealth(
        connected=True,
        healthy=False,
        tx_frames=123,
        rx_frames=120,
        send_errors=1,
        receive_errors=2,
        last_tx_age_ns=50_000,
        last_rx_age_ns=None,
        last_error="injected receive failure",
    )


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

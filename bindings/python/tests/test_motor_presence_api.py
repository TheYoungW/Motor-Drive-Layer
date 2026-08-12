from __future__ import annotations

import motor_drive_layer.core as core_module
from motor_drive_layer.core import Controller
from motor_drive_layer.models import (
    MotorCandidate,
    PresencePolicy,
    PresenceState,
)


class FakeLib:
    def __init__(self) -> None:
        self.calls: list[tuple[int, int, int, int]] = []
        self.freed: list[int] = []

    def motor_controller_discover_damiao_motors(
        self,
        controller,
        candidates,
        candidate_count,
        timeout_ms,
        retries,
        results,
        result_capacity,
    ) -> int:
        self.calls.append((candidate_count, timeout_ms, retries, result_capacity))
        assert candidates[0].role == b"joint1"
        assert candidates[0].policy == PresencePolicy.REQUIRED
        assert candidates[1].role == b"gripper"
        assert candidates[1].policy == PresencePolicy.OPTIONAL

        results[0].role = b"joint1"
        results[0].motor_id = 9
        results[0].feedback_id = 0x19
        results[0].policy = PresencePolicy.REQUIRED
        results[0].state = PresenceState.PRESENT
        results[0].motor = 456
        results[1].role = b"gripper"
        results[1].motor_id = 1
        results[1].feedback_id = 0x11
        results[1].policy = PresencePolicy.OPTIONAL
        results[1].state = PresenceState.NOT_INSTALLED
        results[1].reason = b"fresh feedback timed out"
        return 0

    def motor_handle_free(self, motor) -> None:
        self.freed.append(motor)


class FakeAbi:
    def __init__(self) -> None:
        self.lib = FakeLib()
        self.has_motor_presence_discovery = True


def test_discovery_returns_present_handle_and_optional_absence(monkeypatch) -> None:
    controller = Controller.__new__(Controller)
    controller._abi = FakeAbi()
    controller._ptr = 123
    monkeypatch.setattr(core_module, "get_abi", lambda: controller._abi)

    results = controller.discover_damiao_motors(
        (
            MotorCandidate("joint1", 9, 0x19, "4310"),
            MotorCandidate(
                "gripper", 1, 0x11, "4340P", PresencePolicy.OPTIONAL
            ),
        ),
        timeout_ms=25,
        retries=2,
    )

    assert controller._abi.lib.calls == [(2, 25, 2, 2)]
    assert results[0].state is PresenceState.PRESENT
    assert results[0].motor is not None
    assert results[1].state is PresenceState.NOT_INSTALLED
    assert results[1].motor is None
    assert results[1].reason == "fresh feedback timed out"
    results[0].motor.close()
    assert controller._abi.lib.freed == [456]


def test_discovery_rejects_unbounded_wait_parameters() -> None:
    controller = Controller.__new__(Controller)
    controller._abi = FakeAbi()
    controller._ptr = 123
    candidate = MotorCandidate("joint1", 9, 0x19, "4310")

    try:
        controller.discover_damiao_motors((candidate,), timeout_ms=0)
    except ValueError as error:
        assert "positive" in str(error)
    else:
        raise AssertionError("zero discovery timeout was accepted")

    try:
        controller.discover_damiao_motors((candidate,), retries=11)
    except ValueError as error:
        assert "0..=10" in str(error)
    else:
        raise AssertionError("unbounded discovery retry count was accepted")


def test_presence_override_resolves_against_model_default() -> None:
    assert PresencePolicy.resolve(True, PresencePolicy.OPTIONAL) is PresencePolicy.REQUIRED
    assert PresencePolicy.resolve(False, PresencePolicy.OPTIONAL) is PresencePolicy.DISABLED
    assert PresencePolicy.resolve(None, PresencePolicy.OPTIONAL) is PresencePolicy.OPTIONAL

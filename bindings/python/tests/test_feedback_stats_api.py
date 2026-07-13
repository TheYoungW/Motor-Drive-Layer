from __future__ import annotations

import ctypes

from motorbridge.abi import CFeedbackStats
from motorbridge.core import Controller, Motor
from motorbridge.models import FeedbackStats


class FakeLib:
    def __init__(self) -> None:
        self.tx_gap_calls: list[tuple[int, int]] = []

    def motor_controller_set_tx_gap_us(self, ptr: int, gap_us: int) -> int:
        self.tx_gap_calls.append((ptr, gap_us))
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


def test_motor_exposes_feedback_count_and_age() -> None:
    motor = Motor.__new__(Motor)
    motor._abi = FakeAbi()
    motor._ptr = 456

    assert motor.get_feedback_stats() == FeedbackStats(
        has_feedback=True,
        update_count=42,
        age_ns=125_000,
    )

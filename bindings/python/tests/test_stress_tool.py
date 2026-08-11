from __future__ import annotations

import argparse

import pytest

import motor_drive_layer.stress as stress_module
from motor_drive_layer.models import TransportCapabilities
from motor_drive_layer.stress import MotorSpec, parse_motor_spec, run_dm_device_feedback_stress


class FakeMotor:
    def __init__(self) -> None:
        self.closed = False

    def close(self) -> None:
        self.closed = True


class FakeController:
    opened: list[tuple[str, int, int, int]] = []
    closed_buses = 0
    closed_controllers = 0
    feedback_calls: dict[int, int] = {}

    def __init__(self, channel: int) -> None:
        self.channel = channel

    @classmethod
    def from_dm_device(cls, *, device, channel, bitrate, data_bitrate):
        cls.opened.append((device, channel, bitrate, data_bitrate))
        return cls(channel)

    def transport_capabilities(self) -> TransportCapabilities:
        return TransportCapabilities(
            transport="dm-device",
            max_payload_bytes=8,
            channel_count=2,
            can_fd=True,
            parallel_batches=True,
            hardware_rx_timestamps=False,
            reconnect=True,
            process_session_reuse=True,
        )

    def add_damiao_motor(self, motor_id, feedback_id, model):
        assert motor_id > 0 and feedback_id > 0 and model
        return FakeMotor()

    def request_feedback_all(self, timeout_ms):
        assert timeout_ms == 25
        count = self.feedback_calls.get(self.channel, 0) + 1
        self.feedback_calls[self.channel] = count
        if self.channel == 1 and count == 2:
            raise RuntimeError("fresh feedback timed out; missing motor IDs: 15")

    def close_bus(self):
        type(self).closed_buses += 1

    def close(self):
        type(self).closed_controllers += 1


def test_parse_motor_spec_accepts_hex_and_rejects_bad_values() -> None:
    assert parse_motor_spec("1:0x0f:0x1f:4310") == MotorSpec(1, 15, 31, "4310")
    with pytest.raises(argparse.ArgumentTypeError, match="CHANNEL"):
        parse_motor_spec("1:2")
    with pytest.raises(argparse.ArgumentTypeError, match="0xffff"):
        parse_motor_spec("0:0x10000:1:4310")


def test_feedback_stress_reopens_channels_and_reports_failures(monkeypatch) -> None:
    FakeController.opened = []
    FakeController.closed_buses = 0
    FakeController.closed_controllers = 0
    FakeController.feedback_calls = {}
    monkeypatch.setattr(stress_module, "Controller", FakeController)
    snapshots = iter(
        [
            {"file_descriptors": 4, "threads": 1},
            {"file_descriptors": 6, "threads": 2},
            {"file_descriptors": 6, "threads": 2},
        ]
    )
    monkeypatch.setattr(stress_module, "_resource_snapshot", lambda: next(snapshots))

    result = run_dm_device_feedback_stress(
        [MotorSpec(0, 9, 25, "4310"), MotorSpec(1, 15, 31, "4310")],
        iterations=3,
        reconnect_cycles=2,
        timeout_ms=25,
    )

    assert len(FakeController.opened) == 4
    assert FakeController.closed_buses == 4
    assert FakeController.closed_controllers == 4
    assert result["total_failures"] == 1
    assert result["channels"]["0"]["requests"] == 6
    assert result["channels"]["1"]["failures"] == 1
    assert "missing motor IDs: 15" in result["channels"]["1"]["errors"][0]
    assert result["resources"]["growth_after_warmup"] == {
        "file_descriptors": 0,
        "threads": 0,
    }


def test_feedback_stress_rejects_duplicate_channel_layout() -> None:
    with pytest.raises(ValueError, match="duplicate"):
        run_dm_device_feedback_stress(
            [MotorSpec(0, 9, 25, "4310"), MotorSpec(0, 9, 26, "4310")],
            iterations=1,
        )

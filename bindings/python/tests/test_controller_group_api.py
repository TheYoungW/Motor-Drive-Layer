from __future__ import annotations

import pytest

import motor_drive_layer.core as core_module
from motor_drive_layer import (
    Controller,
    ControllerGroup,
    MitCommand,
    Motor,
    PosVelCommand,
)
from motor_drive_layer.errors import CallError


class FakeGroupLib:
    def __init__(self) -> None:
        self.controllers: list[int] = []
        self.mit: list[tuple[int, float, float, float, float, float]] = []
        self.pos_vel: list[tuple[int, float, float]] = []
        self.freed: list[int] = []
        self.fail_pos_vel = False

    def motor_controller_group_new(self, controllers, count: int) -> int:
        self.controllers = [int(controllers[i]) for i in range(count)]
        return 900

    def motor_controller_group_free(self, ptr: int) -> None:
        self.freed.append(ptr)

    def motor_controller_group_send_mit(self, _group, commands, count: int) -> int:
        self.mit = [
            (
                int(commands[i].motor),
                float(commands[i].target_position),
                float(commands[i].target_velocity),
                float(commands[i].stiffness),
                float(commands[i].damping),
                float(commands[i].feedforward_torque),
            )
            for i in range(count)
        ]
        return 0

    def motor_controller_group_send_pos_vel(self, _group, commands, count: int) -> int:
        self.pos_vel = [
            (
                int(commands[i].motor),
                float(commands[i].target_position),
                float(commands[i].velocity_limit),
            )
            for i in range(count)
        ]
        return -1 if self.fail_pos_vel else 0

    def motor_last_error_message(self) -> bytes:
        return b"controller index 1 (CH1), motor ID 15: injected send failure"


class FakeGroupAbi:
    def __init__(self) -> None:
        self.lib = FakeGroupLib()
        self.has_controller_group = True


def controller(ptr: int) -> Controller:
    value = Controller.__new__(Controller)
    value._ptr = ptr
    return value


def test_controller_group_marshals_mit_and_pos_vel_batches(monkeypatch) -> None:
    abi = FakeGroupAbi()
    monkeypatch.setattr(core_module, "get_abi", lambda: abi)
    ch0 = controller(100)
    ch1 = controller(200)
    m0 = Motor(1001, ch0)
    m1 = Motor(2001, ch1)

    with ControllerGroup([ch0, ch1]) as group:
        group.send_mit(
            [
                MitCommand(m0, 1.0, 2.0, 3.0, 4.0, 5.0),
                MitCommand(m1, -1.0, -2.0, 6.0, 7.0, 8.0),
            ]
        )
        group.send_pos_vel(
            [PosVelCommand(m0, 0.5, 1.5), PosVelCommand(m1, -0.5, 2.5)]
        )

    assert abi.lib.controllers == [100, 200]
    assert abi.lib.mit == [
        (1001, 1.0, 2.0, 3.0, 4.0, 5.0),
        (2001, -1.0, -2.0, 6.0, 7.0, 8.0),
    ]
    assert abi.lib.pos_vel == [(1001, 0.5, 1.5), (2001, -0.5, 2.5)]
    assert abi.lib.freed == [900]


def test_controller_group_preserves_native_failure_details(monkeypatch) -> None:
    abi = FakeGroupAbi()
    abi.lib.fail_pos_vel = True
    monkeypatch.setattr(core_module, "get_abi", lambda: abi)
    ch0 = controller(100)
    ch1 = controller(200)
    motor = Motor(2001, ch1)
    group = ControllerGroup([ch0, ch1])

    with pytest.raises(CallError, match="controller index 1.*CH1.*motor ID 15"):
        group.send_pos_vel([PosVelCommand(motor, 0.0, 1.0)])

    group.close()


def test_controller_group_rejects_foreign_or_closed_members(monkeypatch) -> None:
    abi = FakeGroupAbi()
    monkeypatch.setattr(core_module, "get_abi", lambda: abi)
    ch0 = controller(100)
    foreign = controller(300)
    motor = Motor(3001, foreign)
    group = ControllerGroup([ch0])

    with pytest.raises(ValueError, match="does not belong"):
        group.send_pos_vel([PosVelCommand(motor, 0.0, 1.0)])

    ch0._ptr = None
    with pytest.raises(CallError, match="member 0 is closed"):
        group.send_pos_vel([])
    group.close()

from __future__ import annotations

import pytest

import motor_drive_layer.core as core_module
from motor_drive_layer.core import Controller
from motor_drive_layer.errors import CallError


class FakeSocketCanFdLib:
    def __init__(self) -> None:
        self.calls: list[tuple[bytes, int]] = []

    def motor_controller_new_socketcanfd_ex(self, channel: bytes, enable_brs: int) -> int:
        self.calls.append((channel, enable_brs))
        return 123


class FakeSocketCanFdAbi:
    def __init__(self, *, has_ex: bool = True) -> None:
        self.lib = FakeSocketCanFdLib()
        self.has_socketcanfd_ex = has_ex


@pytest.mark.parametrize("enable_brs, expected", [(True, 1), (False, 0)])
def test_socketcanfd_forwards_explicit_brs_option(monkeypatch, enable_brs, expected) -> None:
    abi = FakeSocketCanFdAbi()
    monkeypatch.setattr(core_module, "get_abi", lambda: abi)

    controller = Controller.from_socketcanfd("can1", enable_brs=enable_brs)

    assert controller._ptr == 123
    assert abi.lib.calls == [(b"can1", expected)]


def test_socketcanfd_enables_brs_by_default(monkeypatch) -> None:
    abi = FakeSocketCanFdAbi()
    monkeypatch.setattr(core_module, "get_abi", lambda: abi)

    Controller.from_socketcanfd("can0")

    assert abi.lib.calls == [(b"can0", 1)]


def test_socketcanfd_rejects_non_boolean_brs(monkeypatch) -> None:
    monkeypatch.setattr(core_module, "get_abi", lambda: FakeSocketCanFdAbi())

    with pytest.raises(TypeError, match="enable_brs must be bool"):
        Controller.from_socketcanfd("can0", enable_brs=1)  # type: ignore[arg-type]


def test_socketcanfd_refuses_unsafe_legacy_default(monkeypatch) -> None:
    monkeypatch.setattr(
        core_module, "get_abi", lambda: FakeSocketCanFdAbi(has_ex=False)
    )

    with pytest.raises(CallError, match="cannot guarantee SocketCAN-FD BRS"):
        Controller.from_socketcanfd("can0")

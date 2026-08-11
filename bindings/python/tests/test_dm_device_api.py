from __future__ import annotations

from pathlib import Path

import pytest

import motor_drive_layer.core as core_module
import motor_drive_layer.dm_device_runtime as runtime_module
from motor_drive_layer.core import Controller
from motor_drive_layer.errors import CallError


class FakeDmDeviceLib:
    def __init__(self) -> None:
        self.calls: list[tuple[bytes, bytes, int, int]] = []

    def motor_controller_new_dm_device_ex(
        self, device: bytes, channel: bytes, bitrate: int, data_bitrate: int
    ) -> int:
        self.calls.append((device, channel, bitrate, data_bitrate))
        return 123

    def motor_controller_free(self, _ptr: int) -> None:
        pass


class FakeDmDeviceAbi:
    def __init__(self) -> None:
        self.lib = FakeDmDeviceLib()
        self.has_dm_device_ex = True


def test_from_dm_device_accepts_preferred_keywords_and_rates(monkeypatch) -> None:
    abi = FakeDmDeviceAbi()
    monkeypatch.setattr(core_module, "ensure_dm_device_runtime", lambda **_kwargs: Path("vendor.so"))
    monkeypatch.setattr(core_module, "get_abi", lambda: abi)

    controller = Controller.from_dm_device(
        device="usb2canfd-dual",
        channel=1,
        bitrate=1_000_000,
        data_bitrate=5_000_000,
    )

    assert abi.lib.calls == [(b"usb2canfd-dual", b"1", 1_000_000, 5_000_000)]
    controller.close()


def test_from_dm_device_wraps_runtime_setup_errors(monkeypatch) -> None:
    def fail(**_kwargs):
        raise RuntimeError("missing vendor library")

    monkeypatch.setattr(core_module, "ensure_dm_device_runtime", fail)
    with pytest.raises(CallError, match="runtime setup failed.*missing vendor library"):
        Controller.from_dm_device(channel=0)


def test_from_dm_device_rejects_zero_rates_before_open(monkeypatch) -> None:
    monkeypatch.setattr(core_module, "ensure_dm_device_runtime", lambda **_kwargs: Path("vendor.so"))
    with pytest.raises(ValueError, match="bitrate"):
        Controller.from_dm_device(bitrate=0)


def test_linux_runtime_candidates_include_v10_fallback(monkeypatch) -> None:
    monkeypatch.setattr(runtime_module.sys, "platform", "linux")
    monkeypatch.setattr(runtime_module.platform, "machine", lambda: "x86_64")

    candidates = runtime_module._platform_runtimes()

    assert [item.version for item in candidates] == ["v1.0.0", "v1.1.0"]
    assert candidates[0].relpath == "linux/libdm_device.so"


def test_runtime_download_falls_back_to_v10(monkeypatch, tmp_path) -> None:
    candidates = [
        runtime_module.DmDeviceRuntime("v1.1.0", "linux/x86_64/libdm_device.so", "libdm_device.so"),
        runtime_module.DmDeviceRuntime("v1.0.0", "linux/libdm_device.so", "libdm_device.so"),
    ]
    attempts: list[str] = []

    monkeypatch.delenv("MOTOR_DM_DEVICE_LIB", raising=False)
    monkeypatch.setattr(runtime_module, "_platform_runtimes", lambda: candidates)
    monkeypatch.setattr(runtime_module, "_packaged_runtime_path", lambda item: tmp_path / "packaged" / item.version)
    monkeypatch.setattr(runtime_module, "_source_runtime_path", lambda _item: None)
    monkeypatch.setattr(runtime_module, "_cache_root", lambda: tmp_path / "cache")

    def download(item, dst, _quiet):
        attempts.append(item.version)
        if item.version == "v1.1.0":
            raise RuntimeError("404")
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_bytes(b"legacy runtime")
        return dst

    monkeypatch.setattr(runtime_module, "_download_runtime", download)

    path = runtime_module.ensure_dm_device_runtime(auto_download=True, quiet=True)

    assert attempts == ["v1.1.0", "v1.0.0"]
    assert path.read_bytes() == b"legacy runtime"
    assert path.parts[-3:] == ("v1.0.0", "linux", "libdm_device.so")

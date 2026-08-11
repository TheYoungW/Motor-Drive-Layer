import ctypes
from pathlib import Path

import motor_drive_layer.abi as abi


def test_candidate_paths_support_shallow_installs(monkeypatch) -> None:
    monkeypatch.setattr(abi, "__file__", "/tmp/pkg/abi.py")

    candidates = abi._candidate_lib_paths()

    assert Path("/tmp/pkg/lib/libmotor_abi.so").resolve() in candidates


def test_articore_candidate_paths_support_shallow_installs(monkeypatch) -> None:
    monkeypatch.setattr(abi, "__file__", "/tmp/pkg/abi.py")

    candidates = abi._candidate_articore_runtime_paths()

    expected = Path("/tmp/pkg/lib") / abi._articore_runtime_lib_name()
    assert expected.resolve() in candidates


def test_articore_runtime_library_exposes_versioned_capabilities() -> None:
    library = ctypes.CDLL(abi.articore_runtime_library_path())
    library.articore_runtime_abi_version.restype = ctypes.c_uint32
    library.articore_runtime_capabilities.restype = ctypes.c_uint64

    assert library.articore_runtime_abi_version() == 0x00010000
    assert library.articore_runtime_capabilities() & 0x1F == 0x1F
    assert abi.articore_runtime_abi_version() == "1.0"
    assert abi.articore_runtime_capabilities()["gripper_protection"] is True

from pathlib import Path

import motor_drive_layer.abi as abi


def test_candidate_paths_support_shallow_installs(monkeypatch) -> None:
    monkeypatch.setattr(abi, "__file__", "/tmp/pkg/abi.py")

    candidates = abi._candidate_lib_paths()

    assert Path("/tmp/pkg/lib/libmotor_abi.so") in candidates

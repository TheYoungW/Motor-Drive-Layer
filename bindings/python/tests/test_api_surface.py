from __future__ import annotations

import json
from pathlib import Path


def test_api_surface_includes_binding_parity_metadata() -> None:
    root = Path(__file__).resolve().parents[3]
    surface = json.loads((root / "bindings" / "api_surface.json").read_text(encoding="utf-8"))

    assert surface["schema"] == 1
    assert "motor_abi_version" in surface["abi"]["metadata"]
    assert "motor_abi_capabilities_json" in surface["abi"]["metadata"]
    assert "motorbridge.abi_version()" in surface["bindings"]["python"]["module_metadata"]
    assert "motor_abi_version" in surface["bindings"]["cpp_damiao"]["abi_metadata"]
    assert surface["vendors"] == ["damiao"]
    assert "Controller.add_damiao_motor(motor_id, feedback_id, model)" in surface["bindings"]["controller_methods"]
    assert "Controller.set_tx_gap_us(gap_us)" in surface["bindings"]["controller_methods"]
    assert "Motor.damiao_get_param_f32(param_id, timeout_ms)" in surface["bindings"]["motor_methods"]
    assert "Motor.get_feedback_stats()" in surface["bindings"]["motor_methods"]

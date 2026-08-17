from __future__ import annotations

import json
import re
from pathlib import Path

import motor_drive_layer


def test_api_surface_includes_binding_parity_metadata() -> None:
    root = Path(__file__).resolve().parents[3]
    surface = json.loads((root / "bindings" / "api_surface.json").read_text(encoding="utf-8"))

    assert surface["schema"] == 1
    assert set(surface["native_libraries"]) == {"motor_abi", "articore_runtime"}
    assert "motor_abi_version" in surface["abi"]["metadata"]
    assert "motor_abi_capabilities_json" in surface["abi"]["metadata"]
    assert "motor_drive_layer.abi_version()" in surface["bindings"]["python"]["module_metadata"]
    assert "motor_abi_version" in surface["bindings"]["cpp_damiao"]["abi_metadata"]
    assert "motor_drive_layer.articore_runtime_library_path()" in surface["bindings"]["python"]["module_metadata"]
    assert "motor_drive_layer.articore_runtime_capabilities()" in surface["bindings"]["python"]["module_metadata"]
    assert "articore_runtime_abi_version" in surface["bindings"]["articore_runtime"]["abi_metadata"]
    assert "articore_runtime_start_joint_trajectory_report" not in surface["bindings"]["articore_runtime"]["abi_metadata"]
    assert "articore_runtime_set_joint_mit" in surface["bindings"]["articore_runtime"]["abi_metadata"]
    assert "articore_runtime_set_joint_pv" in surface["bindings"]["articore_runtime"]["abi_metadata"]
    assert "articore_runtime_get_control_hz" in surface["bindings"]["articore_runtime"]["abi_metadata"]
    assert "articore_runtime_configure_gripper_products" in surface["bindings"]["articore_runtime"]["abi_metadata"]
    assert "articore_runtime_configure_joint_safety_limits" in surface["bindings"]["articore_runtime"]["abi_metadata"]
    assert "articore_runtime_configure_gripper_force_profiles" in surface["bindings"]["articore_runtime"]["abi_metadata"]
    assert "articore_runtime_set_gripper_commands" in surface["bindings"]["articore_runtime"]["abi_metadata"]
    assert surface["vendors"] == ["damiao"]
    assert "Controller.add_damiao_motor(motor_id, feedback_id, model)" in surface["bindings"]["controller_methods"]
    assert "Controller.set_tx_gap_us(gap_us)" in surface["bindings"]["controller_methods"]
    assert "Controller.request_feedback_all(timeout_ms)" in surface["bindings"]["controller_methods"]
    assert "Controller.transport_capabilities()" in surface["bindings"]["controller_methods"]
    assert "Controller.transport_health()" in surface["bindings"]["controller_methods"]
    assert "motor_controller_new_dm_device_ex" in surface["abi"]["controller"]
    assert "motor_controller_get_transport_capabilities" in surface["abi"]["controller"]
    assert "motor_controller_get_transport_health" in surface["abi"]["controller"]
    assert "motor_controller_request_feedback_all_ex" in surface["abi"]["controller"]
    assert "motor_controller_group_send_mit" in surface["abi"]["controller_group"]
    assert "motor_controller_group_send_pos_vel" in surface["abi"]["controller_group"]
    assert "ControllerGroup.send_mit(commands)" in surface["bindings"]["controller_group_methods"]
    assert "ControllerGroup.send_pos_vel(commands)" in surface["bindings"]["controller_group_methods"]
    assert "ControllerGroup.prepare_mit(motors)" in surface["bindings"]["controller_group_methods"]
    assert "ControllerGroup.prepare_pos_vel(motors)" in surface["bindings"]["controller_group_methods"]
    assert "ArticoreRuntime.connect()" in surface["bindings"]["runtime_methods"]
    assert "ArticoreRuntime.health" in surface["bindings"]["runtime_methods"]
    assert surface["bindings"]["cpp_damiao"]["raii_target"] == "motorbridge::articore_runtime_cpp"
    assert "MitCommand(motor, pos, vel, kp, kd, tau)" in surface["bindings"]["command_types"]
    assert "Motor.damiao_get_param_f32(param_id, timeout_ms)" in surface["bindings"]["motor_methods"]
    assert "Motor.get_feedback_stats()" in surface["bindings"]["motor_methods"]
    assert "Motor.request_fresh_state(timeout_ms)" in surface["bindings"]["motor_methods"]


def test_runtime_metadata_and_docs_follow_the_canonical_surface() -> None:
    root = Path(__file__).resolve().parents[3]
    surface = json.loads((root / "bindings" / "api_surface.json").read_text(encoding="utf-8"))
    runtime = surface["runtime_abi"]

    assert motor_drive_layer.articore_runtime_abi_version() == runtime["version"]
    enabled = {
        name for name, value in motor_drive_layer.articore_runtime_capabilities().items()
        if value
    }
    assert enabled == set(runtime["capabilities"])

    public_docs = "\n".join(
        (root / name).read_text(encoding="utf-8")
        for name in ("README.md", "README.zh-CN.md")
    )
    for removed in runtime["removed_terms"]:
        assert removed not in public_docs

    header = (root / "articore_runtime" / "include" / "articore" / "runtime_abi.h").read_text(
        encoding="utf-8"
    )
    declared_symbols = set(re.findall(
        r"ARTICORE_RUNTIME_API\s+(?:[^;]*?\s)?(articore_runtime_[a-z0-9_]+)\s*\(",
        header,
        re.DOTALL,
    ))
    assert declared_symbols == set(
        surface["bindings"]["articore_runtime"]["abi_metadata"]
    )

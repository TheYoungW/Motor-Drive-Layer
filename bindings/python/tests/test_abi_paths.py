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

    assert hasattr(library, "articore_runtime_submit_pos_vel_ex")
    assert hasattr(library, "articore_runtime_submit_mit_ex")
    assert hasattr(library, "articore_runtime_start_joint_trajectory_ex")
    assert hasattr(library, "articore_runtime_cancel_trajectory")
    assert hasattr(library, "articore_runtime_configure_trajectory_execution")
    assert hasattr(library, "articore_runtime_configure_joint_safety_limits")
    assert hasattr(library, "articore_runtime_start_joint_trajectory_report")
    assert hasattr(library, "articore_runtime_configure_gripper_force_profiles")
    assert hasattr(library, "articore_runtime_set_gripper_commands")
    assert library.articore_runtime_abi_version() == 0x0001000A
    assert library.articore_runtime_capabilities() & 0xFFFFF == 0xFFFFF
    assert abi.articore_runtime_abi_version() == "1.10"
    assert abi.articore_runtime_capabilities()["gripper_protection"] is True
    assert abi.articore_runtime_capabilities()["current_position_hold"] is True
    assert abi.articore_runtime_capabilities()["realtime_joint_mailbox"] is True
    assert abi.articore_runtime_capabilities()["joint_trajectory"] is True
    assert abi.articore_runtime_capabilities()["atomic_enable"] is True
    assert abi.articore_runtime_capabilities()["command_lifetime"] is True
    assert abi.articore_runtime_capabilities()["nonpreemptive_trajectory"] is True
    assert abi.articore_runtime_capabilities()["protective_fault_hold"] is True
    assert abi.articore_runtime_capabilities()["deterministic_disable"] is True
    assert abi.articore_runtime_capabilities()["trajectory_management"] is True
    assert abi.articore_runtime_capabilities()["trajectory_settling"] is True
    assert abi.articore_runtime_capabilities()["trajectory_replace_or_hold"] is True
    assert abi.articore_runtime_capabilities()["layered_joint_limits"] is True
    assert abi.articore_runtime_capabilities()["gripper_command_profiles"] is True


def test_motor_abi_exposes_structured_feedback_error_codes() -> None:
    native = abi.get_abi()
    assert native.has_structured_feedback_report is True
    report = abi.CFeedbackReport()
    report.struct_size = ctypes.sizeof(report)
    code = native.lib.motor_controller_request_feedback_all_ex(
        None, 50, ctypes.byref(report), None, 0
    )
    assert code == abi.MOTOR_ERROR_INVALID_ARGUMENT
    assert abi.abi_capabilities()["features"]["structured_feedback_report"] is True

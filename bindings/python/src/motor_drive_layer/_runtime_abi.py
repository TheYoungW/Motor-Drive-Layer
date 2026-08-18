from __future__ import annotations

import ctypes
from ctypes import (
    POINTER,
    Structure,
    c_char,
    c_char_p,
    c_float,
    c_int32,
    c_uint8,
    c_uint32,
    c_uint64,
    c_void_p,
)

from .abi import articore_runtime_library_path, get_abi
from .errors import AbiLoadError


class CRuntimeConfig(Structure):
    _fields_ = [
        ("control_hz", c_uint32),
        ("command_timeout_ms", c_uint32),
        ("enable_grace_ms", c_uint32),
        ("safe_hold_hz", c_uint32),
        ("feedback_check_hz", c_uint32),
        ("feedback_failure_threshold", c_uint32),
        ("feedback_max_age_ms", c_uint32),
        ("safe_hold_failure_threshold", c_uint32),
        ("disable_feedback_timeout_ms", c_uint32),
        ("safe_pv_velocity_limit", c_float),
        ("gripper_control_hz", c_uint32),
        ("gripper_fault_action", c_int32),
    ]


class CRuntimeMotorDescriptor(Structure):
    _fields_ = [
        ("motor", c_void_p),
        ("side", c_uint8),
        ("is_gripper", c_uint8),
        ("name", c_char * 64),
        ("safe_kp", c_float),
        ("safe_kd", c_float),
        ("overload_torque", c_float),
        ("retreat_distance", c_float),
        ("contact_torque", c_float),
        ("motion_window_ms", c_uint32),
        ("stall_movement", c_float),
        ("min_position_error", c_float),
        ("contact_hold_ms", c_uint32),
        ("overload_hold_ms", c_uint32),
        ("hold_offset", c_float),
        ("retreat_retry_ms", c_uint32),
        ("open_position", c_float),
        ("closed_position", c_float),
        ("normal_kp", c_float),
        ("normal_kd", c_float),
        ("close_speed", c_float),
        ("max_step_interval_ms", c_uint32),
        ("closing_direction", c_float),
        ("lower_position", c_float),
        ("upper_position", c_float),
    ]


class CRuntimeMotorApi(Structure):
    _fields_ = [
        ("group_send_pos_vel", c_void_p),
        ("group_send_mit", c_void_p),
        ("controller_disable_all", c_void_p),
        ("controller_request_feedback_all_ex", c_void_p),
        ("motor_get_state", c_void_p),
        ("motor_get_feedback_stats", c_void_p),
        ("last_error_message", c_void_p),
        ("controller_get_transport_health", c_void_p),
        ("motor_disable", c_void_p),
    ]


class CRuntimeTransportCapabilities(Structure):
    _fields_ = [
        ("struct_size", c_uint32),
        ("side", c_uint32),
        ("can_fd", c_int32),
        ("can_fd_brs", c_int32),
        ("transport", c_char * 32),
    ]


class CJointControlConfig(Structure):
    _fields_ = [
        ("motor", c_void_p),
        ("lower_position", c_float),
        ("upper_position", c_float),
        ("velocity_limit", c_float),
        ("torque_limit", c_float),
        ("mit_kp", c_float),
        ("mit_kd", c_float),
        ("mit_feedforward_torque", c_float),
    ]


class CJointSafetyLimits(Structure):
    _fields_ = [
        ("struct_size", c_uint32),
        ("motor", c_void_p),
        ("hard_lower_position", c_float),
        ("hard_upper_position", c_float),
        ("soft_lower_position", c_float),
        ("soft_upper_position", c_float),
        ("soft_limit_braking_zone", c_float),
        ("braking_acceleration", c_float),
    ]


class CJointTarget(Structure):
    _fields_ = [("struct_size", c_uint32), ("motor", c_void_p), ("target_position", c_float)]


class CPosVelCommand(Structure):
    _fields_ = [("motor", c_void_p), ("target_position", c_float), ("velocity_limit", c_float)]


class CMitCommand(Structure):
    _fields_ = [
        ("motor", c_void_p),
        ("target_position", c_float),
        ("target_velocity", c_float),
        ("stiffness", c_float),
        ("damping", c_float),
        ("feedforward_torque", c_float),
    ]


class CGripperProductBinding(Structure):
    _fields_ = [("struct_size", c_uint32), ("motor", c_void_p), ("profile_id", c_char * 64)]


class CGravityProductBinding(Structure):
    _fields_ = [
        ("struct_size", c_uint32),
        ("runtime_side", c_uint32),
        ("robot_side", c_uint32),
        ("product_id", c_char * 64),
    ]


class CGravityCompensationConfig(Structure):
    _fields_ = [("struct_size", c_uint32), ("transition_ms", c_uint32)]


class CMotorIdentity(Structure):
    _fields_ = [("struct_size", c_uint32), ("motor", c_void_p), ("can_id", c_uint32)]


class CGripperCommand(Structure):
    _fields_ = [
        ("struct_size", c_uint32),
        ("motor", c_void_p),
        ("opening", c_float),
        ("speed", c_float),
        ("force_level", c_int32),
    ]


class CEnableMotorResult(Structure):
    _fields_ = [
        ("side", c_uint8), ("can_id", c_uint8), ("status_code", c_uint8),
        ("has_feedback", c_uint8), ("feedback_fresh", c_uint8), ("enabled", c_uint8),
        ("name", c_char * 64),
    ]


class CConnectChannelResult(Structure):
    _fields_ = [
        ("side", c_uint8), ("active", c_uint8), ("request_code", c_int32),
        ("expected_count", c_uint32), ("received_count", c_uint32),
        ("missing_count", c_uint32), ("missing_motor_ids", c_uint32 * 32),
        ("error", c_char * 256),
    ]


class CConnectMotorResult(Structure):
    _fields_ = [
        ("side", c_uint8), ("has_feedback", c_uint8),
        ("feedback_fresh", c_uint8), ("feedback_valid", c_uint8),
        ("configured_can_id", c_uint32), ("reported_can_id", c_uint32),
        ("update_count", c_uint64), ("feedback_age_ns", c_uint64),
        ("name", c_char * 64), ("error", c_char * 256),
    ]


class CConnectReport(Structure):
    _fields_ = [
        ("struct_size", c_uint32), ("success", c_int32), ("error_code", c_int32),
        ("expected_count", c_uint32), ("received_count", c_uint32),
        ("missing_count", c_uint32), ("failure_count", c_uint32),
        ("channel_count", c_uint32), ("channels", CConnectChannelResult * 2),
        ("motor_count", c_uint32), ("motors", CConnectMotorResult * 32),
        ("error", c_char * 512),
    ]


class CEnableReport(Structure):
    _fields_ = [
        ("struct_size", c_uint32), ("success", c_int32), ("disable_confirmed", c_int32),
        ("expected_count", c_uint32), ("enabled_count", c_uint32),
        ("missing_count", c_uint32), ("failure_count", c_uint32),
        ("missing_motor_sides", c_uint8 * 32), ("missing_motor_ids", c_uint32 * 32),
        ("motor_count", c_uint32), ("motors", CEnableMotorResult * 32),
        ("error", c_char * 512),
    ]


class CDisableMotorResult(Structure):
    _fields_ = [
        ("side", c_uint8), ("can_id", c_uint8), ("status_code", c_uint8),
        ("has_feedback", c_uint8), ("feedback_fresh", c_uint8), ("disabled", c_uint8),
        ("disable_sent", c_uint8), ("retry_sent", c_uint8), ("name", c_char * 64),
    ]


class CDisableReport(Structure):
    _fields_ = [
        ("struct_size", c_uint32), ("success", c_int32), ("barrier_confirmed", c_int32),
        ("expected_count", c_uint32), ("disabled_count", c_uint32),
        ("missing_count", c_uint32), ("failure_count", c_uint32), ("retry_count", c_uint32),
        ("missing_motor_sides", c_uint8 * 32), ("missing_motor_ids", c_uint32 * 32),
        ("motor_count", c_uint32), ("motors", CDisableMotorResult * 32),
        ("error", c_char * 512),
    ]


class CRuntimeTransportHealth(Structure):
    _fields_ = [
        ("connected", c_int32), ("healthy", c_int32),
        ("consecutive_send_failures", c_uint32), ("consecutive_feedback_failures", c_uint32),
        ("last_feedback_age_ns", c_uint64), ("tx_frames", c_uint64), ("rx_frames", c_uint64),
        ("send_errors", c_uint64), ("receive_errors", c_uint64),
        ("last_tx_age_ns", c_uint64), ("last_rx_age_ns", c_uint64),
        ("last_error", c_char * 256),
    ]


class CGripperHealth(Structure):
    _fields_ = [
        ("available", c_int32), ("side", c_uint8), ("control_state", c_int32),
        ("opening", c_float), ("motor_position", c_float), ("torque", c_float),
        ("contact_detected", c_int32), ("stalled", c_int32), ("overload", c_int32),
        ("has_hold_target", c_int32), ("hold_target", c_float),
        ("feedback_age_ns", c_uint64), ("name", c_char * 64), ("fault_reason", c_char * 256),
    ]


class CSafetyHealth(Structure):
    _fields_ = [
        ("state", c_int32), ("safe_holding", c_int32), ("disable_confirmed", c_int32),
        ("last_successful_command_age_ns", c_uint64), ("last_fresh_feedback_age_ns", c_uint64),
        ("consecutive_send_failures", c_uint32), ("consecutive_feedback_failures", c_uint32),
        ("left_transport", CRuntimeTransportHealth), ("right_transport", CRuntimeTransportHealth),
        ("gripper_count", c_uint32), ("grippers", CGripperHealth * 2),
        ("motor_fault_count", c_uint32), ("motor_faults", (c_char * 64) * 32),
        ("unconfirmed_disable_count", c_uint32), ("unconfirmed_disable", (c_char * 64) * 32),
        ("fault_reason", c_char * 512),
    ]


_MAX_MIT_TORQUE_LIMIT_JOINTS = 32


class CGravityCompensationStatus(Structure):
    _fields_ = [
        ("struct_size", c_uint32),
        ("phase", c_int32),
        ("active", c_int32),
        ("transition_progress", c_float),
        ("control_cycles", c_uint64),
        ("joint_count", c_uint32),
        ("joints", c_void_p * _MAX_MIT_TORQUE_LIMIT_JOINTS),
        ("gravity_feedforward_torque", c_float * _MAX_MIT_TORQUE_LIMIT_JOINTS),
    ]


class CMitTorqueLimitStats(Structure):
    _fields_ = [
        ("struct_size", c_uint32),
        ("torque_limit_activation_count", c_uint64),
        ("torque_limited_joint_mask", c_uint64),
        ("joint_count", c_uint32),
        ("joints", c_void_p * _MAX_MIT_TORQUE_LIMIT_JOINTS),
        ("requested_resultant_torque", c_float * _MAX_MIT_TORQUE_LIMIT_JOINTS),
        ("applied_scale", c_float * _MAX_MIT_TORQUE_LIMIT_JOINTS),
        ("applied_resultant_torque", c_float * _MAX_MIT_TORQUE_LIMIT_JOINTS),
    ]


def _address(function: object) -> int:
    value = ctypes.cast(function, c_void_p).value
    if value is None:
        raise RuntimeError("native motor ABI function has no address")
    return value


class RuntimeAbi:
    def __init__(self) -> None:
        self.motor = get_abi()
        self.lib = ctypes.CDLL(articore_runtime_library_path())
        self.lib.articore_runtime_abi_version.restype = c_uint32
        version = int(self.lib.articore_runtime_abi_version())
        if version < 0x00020008:
            raise AbiLoadError(
                "official ArticoreRuntime binding requires Runtime ABI >= 2.8; "
                f"loaded {version >> 16}.{version & 0xFFFF}"
            )
        self._bind()

    def motor_api(self) -> CRuntimeMotorApi:
        lib = self.motor.lib
        return CRuntimeMotorApi(
            _address(lib.motor_controller_group_send_pos_vel),
            _address(lib.motor_controller_group_send_mit),
            _address(lib.motor_controller_disable_all),
            _address(lib.motor_controller_request_feedback_all_ex),
            _address(lib.motor_handle_get_state),
            _address(lib.motor_handle_get_feedback_stats),
            _address(lib.motor_last_error_message),
            _address(lib.motor_controller_get_transport_health),
            _address(lib.motor_handle_disable),
        )

    def _bind(self) -> None:
        lib = self.lib
        lib.articore_runtime_last_error.restype = c_char_p
        lib.articore_runtime_create_ex.argtypes = [
            POINTER(CRuntimeConfig), POINTER(CRuntimeMotorApi), c_void_p, c_void_p,
            c_void_p, POINTER(CRuntimeMotorDescriptor), c_uint32, c_void_p, c_void_p,
        ]
        lib.articore_runtime_create_ex.restype = c_void_p
        self.has_transport_aware_create = hasattr(
            lib, "articore_runtime_create_ex2"
        )
        if self.has_transport_aware_create:
            lib.articore_runtime_create_ex2.argtypes = [
                POINTER(CRuntimeConfig), POINTER(CRuntimeMotorApi), c_void_p,
                c_void_p, c_void_p, POINTER(CRuntimeMotorDescriptor), c_uint32,
                c_void_p, c_void_p, POINTER(CRuntimeTransportCapabilities),
                c_uint32,
            ]
            lib.articore_runtime_create_ex2.restype = c_void_p
        lib.articore_runtime_free.argtypes = [c_void_p]
        for name in ("connect", "disable", "recover", "close"):
            function = getattr(lib, f"articore_runtime_{name}")
            function.argtypes = [c_void_p]
            function.restype = c_int32
        lib.articore_runtime_enable.argtypes = [c_void_p, c_int32]
        lib.articore_runtime_enable.restype = c_int32
        lib.articore_runtime_estop.argtypes = [c_void_p, c_char_p]
        lib.articore_runtime_estop.restype = c_int32
        lib.articore_runtime_get_control_hz.argtypes = [c_void_p, POINTER(c_uint32)]
        lib.articore_runtime_get_control_hz.restype = c_int32
        lib.articore_runtime_get_health.argtypes = [c_void_p, POINTER(CSafetyHealth)]
        lib.articore_runtime_get_health.restype = c_int32
        lib.articore_runtime_get_mit_torque_limit_stats.argtypes = [
            c_void_p, POINTER(CMitTorqueLimitStats)
        ]
        lib.articore_runtime_get_mit_torque_limit_stats.restype = c_int32
        lib.articore_runtime_get_last_enable_report.argtypes = [c_void_p, POINTER(CEnableReport)]
        lib.articore_runtime_get_last_enable_report.restype = c_int32
        lib.articore_runtime_get_last_disable_report.argtypes = [c_void_p, POINTER(CDisableReport)]
        lib.articore_runtime_get_last_disable_report.restype = c_int32
        lib.articore_runtime_configure_motor_identities.argtypes = [c_void_p, POINTER(CMotorIdentity), c_uint32]
        lib.articore_runtime_configure_motor_identities.restype = c_int32
        lib.articore_runtime_get_last_connect_report.argtypes = [c_void_p, POINTER(CConnectReport)]
        lib.articore_runtime_get_last_connect_report.restype = c_int32
        lib.articore_runtime_configure_joints.argtypes = [c_void_p, POINTER(CJointControlConfig), c_uint32]
        lib.articore_runtime_configure_joints.restype = c_int32
        lib.articore_runtime_configure_joint_safety_limits.argtypes = [c_void_p, POINTER(CJointSafetyLimits), c_uint32]
        lib.articore_runtime_configure_joint_safety_limits.restype = c_int32
        lib.articore_runtime_configure_gripper_products.argtypes = [c_void_p, POINTER(CGripperProductBinding), c_uint32]
        lib.articore_runtime_configure_gripper_products.restype = c_int32
        lib.articore_runtime_configure_gravity_products.argtypes = [c_void_p, POINTER(CGravityProductBinding), c_uint32]
        lib.articore_runtime_configure_gravity_products.restype = c_int32
        lib.articore_runtime_start_gravity_compensation.argtypes = [c_void_p, POINTER(CGravityCompensationConfig)]
        lib.articore_runtime_start_gravity_compensation.restype = c_int32
        lib.articore_runtime_stop_gravity_compensation.argtypes = [c_void_p]
        lib.articore_runtime_stop_gravity_compensation.restype = c_int32
        lib.articore_runtime_get_gravity_compensation_status.argtypes = [c_void_p, POINTER(CGravityCompensationStatus)]
        lib.articore_runtime_get_gravity_compensation_status.restype = c_int32
        lib.articore_runtime_set_gripper_commands.argtypes = [c_void_p, POINTER(CGripperCommand), c_uint32]
        lib.articore_runtime_set_gripper_commands.restype = c_int32
        for name in ("set_joint_pv", "set_joint_mit"):
            function = getattr(lib, f"articore_runtime_{name}")
            function.argtypes = [c_void_p, POINTER(CJointTarget), c_uint32, c_float]
            function.restype = c_int32
        lib.articore_runtime_submit_pos_vel_ex.argtypes = [c_void_p, POINTER(CPosVelCommand), c_uint32, c_int32]
        lib.articore_runtime_submit_pos_vel_ex.restype = c_int32
        lib.articore_runtime_submit_mit_ex.argtypes = [c_void_p, POINTER(CMitCommand), c_uint32, c_int32]
        lib.articore_runtime_submit_mit_ex.restype = c_int32
        lib.articore_runtime_report_feedback_failure.argtypes = [c_void_p, c_uint8, c_char_p]
        lib.articore_runtime_report_feedback_failure.restype = c_int32
        lib.articore_runtime_declare_motor_presence.argtypes = [c_void_p, c_char_p, c_int32]
        lib.articore_runtime_declare_motor_presence.restype = c_int32
        lib.articore_runtime_motor_presence.argtypes = [c_void_p, c_char_p, POINTER(c_int32)]
        lib.articore_runtime_motor_presence.restype = c_int32
        lib.articore_runtime_active_capabilities.argtypes = [c_void_p]
        lib.articore_runtime_active_capabilities.restype = c_uint64


_runtime_abi: RuntimeAbi | None = None


def get_runtime_abi() -> RuntimeAbi:
    global _runtime_abi
    if _runtime_abi is None:
        _runtime_abi = RuntimeAbi()
    return _runtime_abi

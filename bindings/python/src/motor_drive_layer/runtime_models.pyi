from enum import IntEnum, IntFlag
from .core import Motor

class SafetyState(IntEnum):
    DISCONNECTED: int
    READY: int
    ENABLED: int
    RUNNING: int
    SAFE_HOLD: int
    FAULT: int
class RuntimeControlMode(IntEnum):
    PV: int
    MIT: int
class GravityCompensationPhase(IntEnum):
    INACTIVE: int
    ENTERING: int
    ACTIVE: int
    EXITING: int
class CommandLifetime(IntEnum):
    STREAMING: int
    HOLD_UNTIL_REPLACED: int
class GripperControlState(IntEnum):
    DISABLED: int
    IDLE: int
    MOVING: int
    CONTACT: int
    HOLDING: int
    OVERLOAD_RETREAT: int
    FAULT: int
class GripperFaultAction(IntEnum):
    HOLD: int
    DISABLE: int
class ActiveCapability(IntFlag):
    ARM_SIDE_0: int
    ARM_SIDE_1: int
    GRIPPER_SIDE_0: int
    GRIPPER_SIDE_1: int
class ConnectErrorCode(IntEnum):
    OK: int
    CONFIGURATION: int
    TRANSPORT: int
    FEEDBACK_TIMEOUT: int
    FEEDBACK_INCOMPLETE: int
    FEEDBACK_INVALID: int

class RuntimeConfig:
    control_hz: int
    command_timeout_ms: int
    enable_grace_ms: int
    safe_hold_hz: int
    feedback_check_hz: int
    feedback_failure_threshold: int
    feedback_max_age_ms: int
    safe_hold_failure_threshold: int
    disable_feedback_timeout_ms: int
    safe_pv_velocity_limit: float
    gripper_control_hz: int
    gripper_fault_action: GripperFaultAction
    def __init__(self, control_hz: int = ..., command_timeout_ms: int = ..., enable_grace_ms: int = ..., safe_hold_hz: int = ..., feedback_check_hz: int = ..., feedback_failure_threshold: int = ..., feedback_max_age_ms: int = ..., safe_hold_failure_threshold: int = ..., disable_feedback_timeout_ms: int = ..., safe_pv_velocity_limit: float = ..., gripper_control_hz: int = ..., gripper_fault_action: GripperFaultAction = ...) -> None: ...
class RuntimeMotor:
    motor: Motor
    side: int
    name: str
    is_gripper: bool
    safe_kp: float
    safe_kd: float
    def __init__(self, motor: Motor, side: int, name: str, is_gripper: bool = ..., safe_kp: float = ..., safe_kd: float = ...) -> None: ...
class JointControlConfig:
    motor: Motor
    lower_position: float
    upper_position: float
    velocity_limit: float
    torque_limit: float
    mit_kp: float
    mit_kd: float
    mit_feedforward_torque: float
    def __init__(self, motor: Motor, lower_position: float, upper_position: float, velocity_limit: float, torque_limit: float, mit_kp: float, mit_kd: float, mit_feedforward_torque: float = ...) -> None: ...
class JointSafetyLimits:
    motor: Motor
    hard_lower_position: float
    hard_upper_position: float
    soft_lower_position: float
    soft_upper_position: float
    soft_limit_braking_zone: float
    braking_acceleration: float
    def __init__(self, motor: Motor, hard_lower_position: float, hard_upper_position: float, soft_lower_position: float, soft_upper_position: float, soft_limit_braking_zone: float, braking_acceleration: float) -> None: ...
class MitTorqueLimitJointStats:
    motor: Motor
    requested_resultant_torque: float
    applied_scale: float
    applied_resultant_torque: float
    limited: bool
    def __init__(self, motor: Motor, requested_resultant_torque: float, applied_scale: float, applied_resultant_torque: float, limited: bool) -> None: ...
class MitTorqueLimitStats:
    torque_limit_activation_count: int
    torque_limited_joint_mask: int
    joints: tuple[MitTorqueLimitJointStats, ...]
    def __init__(self, torque_limit_activation_count: int, torque_limited_joint_mask: int, joints: tuple[MitTorqueLimitJointStats, ...]) -> None: ...
class GripperProductBinding:
    motor: Motor
    profile_id: str
    def __init__(self, motor: Motor, profile_id: str) -> None: ...
class GravityProductBinding:
    runtime_side: int
    robot_side: int
    product_id: str
    def __init__(self, runtime_side: int, robot_side: int, product_id: str = ...) -> None: ...
class GravityCompensationStatus:
    phase: GravityCompensationPhase
    active: bool
    transition_progress: float
    control_cycles: int
    joints: tuple[Motor, ...]
    gravity_feedforward_torque: tuple[float, ...]
class GripperCommand:
    motor: Motor
    opening: float
    speed: float
    force_level: int
    def __init__(self, motor: Motor, opening: float, speed: float = ..., force_level: int = ...) -> None: ...
class JointPositionTarget:
    motor: Motor
    position: float
    def __init__(self, motor: Motor, position: float) -> None: ...
class RuntimeMitCommand:
    motor: Motor
    position: float
    velocity: float
    kp: float
    kd: float
    feedforward_torque: float
    def __init__(self, motor: Motor, position: float, velocity: float, kp: float, kd: float, feedforward_torque: float) -> None: ...
class RuntimePvCommand:
    motor: Motor
    position: float
    velocity_limit: float
    def __init__(self, motor: Motor, position: float, velocity_limit: float) -> None: ...
class EnableMotorResult:
    side: int
    can_id: int
    status_code: int
    has_feedback: bool
    feedback_fresh: bool
    enabled: bool
    name: str
class ConnectChannelResult:
    side: int
    active: bool
    request_code: int
    expected_count: int
    received_count: int
    missing_motor_ids: tuple[int, ...]
    error: str | None
class ConnectMotorResult:
    side: int
    configured_can_id: int
    reported_can_id: int
    has_feedback: bool
    feedback_fresh: bool
    feedback_valid: bool
    update_count: int
    feedback_age_ns: int | None
    name: str
    error: str | None
class ConnectReport:
    success: bool
    error_code: ConnectErrorCode
    expected_count: int
    received_count: int
    missing_count: int
    failure_count: int
    channels: tuple[ConnectChannelResult, ...]
    motors: tuple[ConnectMotorResult, ...]
    error: str | None
class EnableReport:
    success: bool
    disable_confirmed: bool
    expected_count: int
    enabled_count: int
    missing_count: int
    failure_count: int
    missing_motors: tuple[tuple[int, int], ...]
    motors: tuple[EnableMotorResult, ...]
    error: str | None
class DisableMotorResult:
    side: int
    can_id: int
    status_code: int
    has_feedback: bool
    feedback_fresh: bool
    disabled: bool
    disable_sent: bool
    retry_sent: bool
    name: str
class DisableReport:
    success: bool
    barrier_confirmed: bool
    expected_count: int
    disabled_count: int
    missing_count: int
    failure_count: int
    retry_count: int
    missing_motors: tuple[tuple[int, int], ...]
    motors: tuple[DisableMotorResult, ...]
    error: str | None
class RuntimeTransportHealth:
    connected: bool
    healthy: bool
    consecutive_send_failures: int
    consecutive_feedback_failures: int
    last_feedback_age_ns: int | None
    tx_frames: int
    rx_frames: int
    send_errors: int
    receive_errors: int
    last_tx_age_ns: int | None
    last_rx_age_ns: int | None
    last_error: str | None
class GripperHealth:
    available: bool
    side: int
    control_state: GripperControlState
    opening: float
    motor_position: float
    torque: float
    contact_detected: bool
    stalled: bool
    overload: bool
    hold_target: float | None
    feedback_age_ns: int | None
    name: str
    fault_reason: str | None
class SafetyHealth:
    state: SafetyState
    safe_holding: bool
    disable_confirmed: bool
    last_successful_command_age_ns: int | None
    last_fresh_feedback_age_ns: int | None
    consecutive_send_failures: int
    consecutive_feedback_failures: int
    left_transport: RuntimeTransportHealth
    right_transport: RuntimeTransportHealth
    grippers: tuple[GripperHealth, ...]
    motor_faults: tuple[str, ...]
    unconfirmed_disable: tuple[str, ...]
    fault_reason: str | None

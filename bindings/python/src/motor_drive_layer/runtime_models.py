from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum, IntFlag
from math import isfinite
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .core import Motor


class SafetyState(IntEnum):
    DISCONNECTED = 0
    READY = 1
    ENABLED = 2
    RUNNING = 3
    SAFE_HOLD = 4
    FAULT = 5


class RuntimeControlMode(IntEnum):
    PV = 1
    MIT = 2


class GravityCompensationPhase(IntEnum):
    INACTIVE = 0
    ENTERING = 1
    ACTIVE = 2
    EXITING = 3


class CommandLifetime(IntEnum):
    STREAMING = 1
    HOLD_UNTIL_REPLACED = 2


class GripperControlState(IntEnum):
    DISABLED = 0
    IDLE = 1
    MOVING = 2
    CONTACT = 3
    HOLDING = 4
    OVERLOAD_RETREAT = 5
    FAULT = 6


class GripperFaultAction(IntEnum):
    HOLD = 1
    DISABLE = 2


class ActiveCapability(IntFlag):
    ARM_SIDE_0 = 1 << 0
    ARM_SIDE_1 = 1 << 1
    GRIPPER_SIDE_0 = 1 << 2
    GRIPPER_SIDE_1 = 1 << 3


class ConnectErrorCode(IntEnum):
    OK = 0
    CONFIGURATION = 1
    TRANSPORT = 2
    FEEDBACK_TIMEOUT = 3
    FEEDBACK_INCOMPLETE = 4
    FEEDBACK_INVALID = 5


@dataclass(frozen=True)
class RuntimeConfig:
    control_hz: int = 400
    command_timeout_ms: int = 250
    enable_grace_ms: int = 2000
    safe_hold_hz: int = 100
    feedback_check_hz: int = 100
    feedback_failure_threshold: int = 3
    feedback_max_age_ms: int = 50
    safe_hold_failure_threshold: int = 3
    disable_feedback_timeout_ms: int = 50
    safe_pv_velocity_limit: float = 0.5
    gripper_control_hz: int = 400
    gripper_fault_action: GripperFaultAction = GripperFaultAction.HOLD

    def __post_init__(self) -> None:
        positive_u32 = (
            "control_hz", "command_timeout_ms", "enable_grace_ms",
            "safe_hold_hz", "feedback_check_hz", "feedback_failure_threshold",
            "feedback_max_age_ms", "safe_hold_failure_threshold",
            "disable_feedback_timeout_ms", "gripper_control_hz",
        )
        for field in positive_u32:
            value = getattr(self, field)
            if not isinstance(value, int) or not 1 <= value <= 0xFFFFFFFF:
                raise ValueError(f"{field} must be an integer in 1..=4294967295")
        if not isfinite(self.safe_pv_velocity_limit) or self.safe_pv_velocity_limit <= 0:
            raise ValueError("safe_pv_velocity_limit must be finite and positive")
        GripperFaultAction(self.gripper_fault_action)


@dataclass(frozen=True)
class RuntimeMotor:
    motor: Motor
    side: int
    name: str
    is_gripper: bool = False
    safe_kp: float = 0.0
    safe_kd: float = 0.0

    def __post_init__(self) -> None:
        if self.side not in (0, 1):
            raise ValueError("side must be 0 or 1")
        if not self.name:
            raise ValueError("name must not be empty")
        if not isfinite(self.safe_kp) or self.safe_kp < 0:
            raise ValueError("safe_kp must be finite and non-negative")
        if not isfinite(self.safe_kd) or self.safe_kd < 0:
            raise ValueError("safe_kd must be finite and non-negative")


@dataclass(frozen=True)
class JointControlConfig:
    motor: Motor
    lower_position: float
    upper_position: float
    velocity_limit: float
    torque_limit: float
    mit_kp: float
    mit_kd: float
    mit_feedforward_torque: float = 0.0


@dataclass(frozen=True)
class JointSafetyLimits:
    motor: Motor
    hard_lower_position: float
    hard_upper_position: float
    soft_lower_position: float
    soft_upper_position: float
    soft_limit_braking_zone: float
    braking_acceleration: float


@dataclass(frozen=True)
class MitTorqueLimitJointStats:
    motor: Motor
    requested_resultant_torque: float
    applied_scale: float
    applied_resultant_torque: float
    limited: bool


@dataclass(frozen=True)
class MitTorqueLimitStats:
    torque_limit_activation_count: int
    torque_limited_joint_mask: int
    joints: tuple[MitTorqueLimitJointStats, ...]


@dataclass(frozen=True)
class GripperProductBinding:
    motor: Motor
    profile_id: str


@dataclass(frozen=True)
class GravityProductBinding:
    runtime_side: int
    robot_side: int
    product_id: str = "yunyi_v1_0"

    def __post_init__(self) -> None:
        if self.runtime_side not in (0, 1):
            raise ValueError("runtime_side must be 0 or 1")
        if self.robot_side not in (0, 1):
            raise ValueError("robot_side must be 0 or 1")
        if not self.product_id:
            raise ValueError("product_id must not be empty")


@dataclass(frozen=True)
class GravityCompensationStatus:
    phase: GravityCompensationPhase
    active: bool
    transition_progress: float
    control_cycles: int
    joints: tuple[Motor, ...]
    gravity_feedforward_torque: tuple[float, ...]


@dataclass(frozen=True)
class GripperCommand:
    motor: Motor
    opening: float
    speed: float = 1000.0
    force_level: int = 5

    def __post_init__(self) -> None:
        if not isfinite(self.opening) or not 0 <= self.opening <= 1000:
            raise ValueError("opening must be finite and in 0..1000")
        if not isfinite(self.speed) or not 0 < self.speed <= 1000:
            raise ValueError("speed must be finite and in (0, 1000]")
        if not isinstance(self.force_level, int) or not 1 <= self.force_level <= 10:
            raise ValueError("force_level must be an integer in 1..10")


@dataclass(frozen=True)
class JointPositionTarget:
    motor: Motor
    position: float


@dataclass(frozen=True)
class RuntimeMitCommand:
    motor: Motor
    position: float
    velocity: float
    kp: float
    kd: float
    feedforward_torque: float


@dataclass(frozen=True)
class RuntimePvCommand:
    motor: Motor
    position: float
    velocity_limit: float


@dataclass(frozen=True)
class EnableMotorResult:
    side: int
    can_id: int
    status_code: int
    has_feedback: bool
    feedback_fresh: bool
    enabled: bool
    name: str


@dataclass(frozen=True)
class ConnectChannelResult:
    side: int
    active: bool
    request_code: int
    expected_count: int
    received_count: int
    missing_motor_ids: tuple[int, ...]
    error: str | None


@dataclass(frozen=True)
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


@dataclass(frozen=True)
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


@dataclass(frozen=True)
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


@dataclass(frozen=True)
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


@dataclass(frozen=True)
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


@dataclass(frozen=True)
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


@dataclass(frozen=True)
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


@dataclass(frozen=True)
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

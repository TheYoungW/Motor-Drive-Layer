from enum import IntEnum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .core import Motor

class Mode(IntEnum):
    MIT = 1
    POS_VEL = 2
    VEL = 3
    FORCE_POS = 4

class PresencePolicy(IntEnum):
    REQUIRED = 1
    OPTIONAL = 2
    DISABLED = 3
    @classmethod
    def resolve(
        cls, override: bool | None, model_default: PresencePolicy
    ) -> PresencePolicy: ...

class PresenceState(IntEnum):
    NOT_INSTALLED = 1
    PRESENT = 2
    FAULTED = 3

class MotorCandidate:
    role: str
    motor_id: int
    feedback_id: int
    model: str
    policy: PresencePolicy
    def __init__(
        self,
        role: str,
        motor_id: int,
        feedback_id: int,
        model: str,
        policy: PresencePolicy = PresencePolicy.REQUIRED,
    ) -> None: ...

class MotorDiscoveryResult:
    role: str
    motor_id: int
    feedback_id: int
    policy: PresencePolicy
    state: PresenceState
    motor: Motor | None
    reason: str | None
    def __init__(
        self,
        role: str,
        motor_id: int,
        feedback_id: int,
        policy: PresencePolicy,
        state: PresenceState,
        motor: Motor | None,
        reason: str | None,
    ) -> None: ...

class MotorState:
    can_id: int
    arbitration_id: int
    status_code: int
    pos: float
    vel: float
    torq: float
    t_mos: float
    t_rotor: float
    def __init__(
        self,
        can_id: int,
        arbitration_id: int,
        status_code: int,
        pos: float,
        vel: float,
        torq: float,
        t_mos: float,
        t_rotor: float,
    ) -> None: ...

class FeedbackStats:
    has_feedback: bool
    update_count: int
    age_ns: int
    def __init__(self, has_feedback: bool, update_count: int, age_ns: int) -> None: ...

class TransportCapabilities:
    transport: str
    max_payload_bytes: int
    channel_count: int
    can_fd: bool
    parallel_batches: bool
    hardware_rx_timestamps: bool
    reconnect: bool
    process_session_reuse: bool
    def __init__(
        self,
        transport: str,
        max_payload_bytes: int,
        channel_count: int,
        can_fd: bool,
        parallel_batches: bool,
        hardware_rx_timestamps: bool,
        reconnect: bool,
        process_session_reuse: bool,
    ) -> None: ...

class TransportHealth:
    connected: bool
    healthy: bool
    tx_frames: int
    rx_frames: int
    send_errors: int
    receive_errors: int
    last_tx_age_ns: int | None
    last_rx_age_ns: int | None
    last_error: str | None
    def __init__(
        self,
        connected: bool,
        healthy: bool,
        tx_frames: int,
        rx_frames: int,
        send_errors: int,
        receive_errors: int,
        last_tx_age_ns: int | None,
        last_rx_age_ns: int | None,
        last_error: str | None,
    ) -> None: ...

class MitCommand:
    motor: Motor
    pos: float
    vel: float
    kp: float
    kd: float
    tau: float
    def __init__(
        self, motor: Motor, pos: float, vel: float, kp: float, kd: float, tau: float
    ) -> None: ...

class PosVelCommand:
    motor: Motor
    pos: float
    vlim: float
    def __init__(self, motor: Motor, pos: float, vlim: float) -> None: ...

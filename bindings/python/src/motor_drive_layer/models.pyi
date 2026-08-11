from enum import IntEnum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .core import Motor

class Mode(IntEnum):
    MIT = 1
    POS_VEL = 2
    VEL = 3
    FORCE_POS = 4

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

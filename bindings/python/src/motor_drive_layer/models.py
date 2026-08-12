from __future__ import annotations

from dataclasses import dataclass
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
    ) -> PresencePolicy:
        """Resolve True/False/None against a model's default policy."""
        if override is True:
            return cls.REQUIRED
        if override is False:
            return cls.DISABLED
        if override is None:
            return cls(model_default)
        raise TypeError("presence override must be True, False, or None")


class PresenceState(IntEnum):
    NOT_INSTALLED = 1
    PRESENT = 2
    FAULTED = 3


@dataclass(frozen=True)
class MotorCandidate:
    role: str
    motor_id: int
    feedback_id: int
    model: str
    policy: PresencePolicy = PresencePolicy.REQUIRED


@dataclass(frozen=True)
class MotorDiscoveryResult:
    role: str
    motor_id: int
    feedback_id: int
    policy: PresencePolicy
    state: PresenceState
    motor: Motor | None
    reason: str | None


@dataclass(frozen=True)
class MotorState:
    can_id: int
    arbitration_id: int
    status_code: int
    pos: float
    vel: float
    torq: float
    t_mos: float
    t_rotor: float


@dataclass(frozen=True)
class FeedbackStats:
    has_feedback: bool
    update_count: int
    age_ns: int


@dataclass(frozen=True)
class TransportCapabilities:
    transport: str
    max_payload_bytes: int
    channel_count: int
    can_fd: bool
    parallel_batches: bool
    hardware_rx_timestamps: bool
    reconnect: bool
    process_session_reuse: bool


@dataclass(frozen=True)
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


@dataclass(frozen=True)
class MitCommand:
    motor: Motor
    pos: float
    vel: float
    kp: float
    kd: float
    tau: float


@dataclass(frozen=True)
class PosVelCommand:
    motor: Motor
    pos: float
    vlim: float

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

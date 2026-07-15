from enum import IntEnum

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

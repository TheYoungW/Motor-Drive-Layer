from __future__ import annotations

from dataclasses import dataclass

from .abi import damiao_register_info


@dataclass(frozen=True)
class RegisterSpec:
    rid: int
    variable: str
    description: str
    data_type: str
    access: str
    range_str: str


# Python owns only user-facing documentation. Access and data type come from
# the canonical C++ register_info() table through the C ABI.
_DAMIAO_RW_REGISTER_DOCS: tuple[tuple[int, str, str, str], ...] = (
    (0, "UV_Value", "Under-voltage protection value", "(10.0, 3.4E38]"),
    (1, "KT_Value", "Torque coefficient", "[0.0, 3.4E38]"),
    (2, "OT_Value", "Over-temperature protection value", "[80.0, 200)"),
    (3, "OC_Value", "Over-current protection value", "(0.0, 1.0)"),
    (4, "ACC", "Acceleration", "(0.0, 3.4E38)"),
    (5, "DEC", "Deceleration", "[-3.4E38, 0.0)"),
    (6, "MAX_SPD", "Maximum speed", "(0.0, 3.4E38]"),
    (7, "MST_ID", "Feedback ID", "[0, 0x7FF]"),
    (8, "ESC_ID", "Receive ID", "[0, 0x7FF]"),
    (9, "TIMEOUT", "Timeout alarm time", "[0, 2^32-1]"),
    (10, "CTRL_MODE", "Control mode", "[1, 4]"),
    (21, "PMAX", "Position mapping range", "(0.0, 3.4E38]"),
    (22, "VMAX", "Speed mapping range", "(0.0, 3.4E38]"),
    (23, "TMAX", "Torque mapping range", "(0.0, 3.4E38]"),
    (24, "I_BW", "Current loop control bandwidth", "[100.0, 10000.0]"),
    (25, "KP_ASR", "Speed loop Kp", "[0.0, 3.4E38]"),
    (26, "KI_ASR", "Speed loop Ki", "[0.0, 3.4E38]"),
    (27, "KP_APR", "Position loop Kp", "[0.0, 3.4E38]"),
    (28, "KI_APR", "Position loop Ki", "[0.0, 3.4E38]"),
    (29, "OV_Value", "Over-voltage protection value", "TBD"),
    (30, "GREF", "Gear torque efficiency", "(0.0, 1.0]"),
    (31, "Deta", "Speed loop damping coefficient", "[1.0, 30.0]"),
    (32, "V_BW", "Speed loop filter bandwidth", "(0.0, 500.0)"),
    (33, "IQ_c1", "Current loop enhancement coefficient", "[100.0, 10000.0]"),
    (34, "VL_c1", "Speed loop enhancement coefficient", "(0.0, 10000.0]"),
    (35, "can_br", "CAN baud rate code", "[0, 4]"),
)


def _build_rw_registers() -> dict[int, RegisterSpec]:
    result: dict[int, RegisterSpec] = {}
    for rid, variable, description, range_str in _DAMIAO_RW_REGISTER_DOCS:
        metadata = damiao_register_info(rid)
        if metadata is None:
            raise RuntimeError(f"C++ register table does not define Damiao register {rid}")
        access, data_type = metadata
        if access != "RW":
            raise RuntimeError(
                f"Damiao register {rid} is documented as RW but C++ reports {access}"
            )
        result[rid] = RegisterSpec(rid, variable, description, data_type, access, range_str)
    return result


DAMIAO_RW_REGISTERS: dict[int, RegisterSpec] = _build_rw_registers()

DAMIAO_HIGH_IMPACT_RIDS: tuple[int, ...] = (21, 22, 23, 25, 26, 27, 28, 4, 5, 6, 9)
DAMIAO_PROTECTION_RIDS: tuple[int, ...] = (0, 2, 3, 29)

RID_CTRL_MODE = 10
RID_MST_ID = 7
RID_ESC_ID = 8
RID_TIMEOUT = 9
RID_PMAX = 21
RID_VMAX = 22
RID_TMAX = 23
RID_KP_ASR = 25
RID_KI_ASR = 26
RID_KP_APR = 27
RID_KI_APR = 28

MODE_MIT = 1
MODE_POS_VEL = 2
MODE_VEL = 3
MODE_FORCE_POS = 4


def get_damiao_register_spec(rid: int) -> RegisterSpec | None:
    return DAMIAO_RW_REGISTERS.get(rid)

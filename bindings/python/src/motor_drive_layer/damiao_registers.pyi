from typing import Final

class RegisterSpec:
    rid: int
    variable: str
    description: str
    data_type: str
    access: str
    range_str: str
    def __init__(
        self,
        rid: int,
        variable: str,
        description: str,
        data_type: str,
        access: str,
        range_str: str,
    ) -> None: ...

DAMIAO_RW_REGISTERS: dict[int, RegisterSpec]
DAMIAO_HIGH_IMPACT_RIDS: tuple[int, ...]
DAMIAO_PROTECTION_RIDS: tuple[int, ...]
RID_CTRL_MODE: Final[int]
RID_MST_ID: Final[int]
RID_ESC_ID: Final[int]
RID_TIMEOUT: Final[int]
RID_PMAX: Final[int]
RID_VMAX: Final[int]
RID_TMAX: Final[int]
RID_KP_ASR: Final[int]
RID_KI_ASR: Final[int]
RID_KP_APR: Final[int]
RID_KI_APR: Final[int]
MODE_MIT: Final[int]
MODE_POS_VEL: Final[int]
MODE_VEL: Final[int]
MODE_FORCE_POS: Final[int]

def get_damiao_register_spec(rid: int) -> RegisterSpec | None: ...

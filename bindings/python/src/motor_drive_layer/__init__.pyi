from .abi import abi_capabilities as abi_capabilities, abi_version as abi_version
from .core import Controller as Controller, Motor as Motor
from .damiao_registers import (
    DAMIAO_HIGH_IMPACT_RIDS as DAMIAO_HIGH_IMPACT_RIDS,
    DAMIAO_PROTECTION_RIDS as DAMIAO_PROTECTION_RIDS,
    DAMIAO_RW_REGISTERS as DAMIAO_RW_REGISTERS,
    MODE_FORCE_POS as MODE_FORCE_POS,
    MODE_MIT as MODE_MIT,
    MODE_POS_VEL as MODE_POS_VEL,
    MODE_VEL as MODE_VEL,
    RID_CTRL_MODE as RID_CTRL_MODE,
    RID_ESC_ID as RID_ESC_ID,
    RID_KI_APR as RID_KI_APR,
    RID_KI_ASR as RID_KI_ASR,
    RID_KP_APR as RID_KP_APR,
    RID_KP_ASR as RID_KP_ASR,
    RID_MST_ID as RID_MST_ID,
    RID_PMAX as RID_PMAX,
    RID_TMAX as RID_TMAX,
    RID_TIMEOUT as RID_TIMEOUT,
    RID_VMAX as RID_VMAX,
    RegisterSpec as RegisterSpec,
    get_damiao_register_spec as get_damiao_register_spec,
)
from .dm_device_runtime import ensure_dm_device_runtime as ensure_dm_device_runtime
from .errors import (
    AbiLoadError as AbiLoadError,
    CallError as CallError,
    MotorBridgeError as MotorBridgeError,
)
from .models import FeedbackStats as FeedbackStats, Mode as Mode, MotorState as MotorState

__version__: str
def get_version() -> str: ...

from importlib import import_module

from .abi import abi_capabilities, abi_version
from .core import Controller, Motor
from .dm_device_runtime import ensure_dm_device_runtime
from .errors import AbiLoadError, CallError, MotorBridgeError
from .models import FeedbackStats, Mode, MotorState
from ._version import VERSION


_REGISTER_EXPORTS = {
    "RegisterSpec",
    "DAMIAO_RW_REGISTERS",
    "DAMIAO_HIGH_IMPACT_RIDS",
    "DAMIAO_PROTECTION_RIDS",
    "get_damiao_register_spec",
    "RID_CTRL_MODE",
    "RID_MST_ID",
    "RID_ESC_ID",
    "RID_TIMEOUT",
    "RID_PMAX",
    "RID_VMAX",
    "RID_TMAX",
    "RID_KP_ASR",
    "RID_KI_ASR",
    "RID_KP_APR",
    "RID_KI_APR",
    "MODE_MIT",
    "MODE_POS_VEL",
    "MODE_VEL",
    "MODE_FORCE_POS",
}


def __getattr__(name: str):
    if name in _REGISTER_EXPORTS:
        module = import_module(".damiao_registers", __name__)
        value = getattr(module, name)
        globals()[name] = value
        return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__() -> list[str]:
    return sorted(set(globals()) | _REGISTER_EXPORTS)


def get_version() -> str:
    return VERSION


__version__ = get_version()

__all__ = [
    "__version__",
    "get_version",
    "abi_version",
    "abi_capabilities",
    "ensure_dm_device_runtime",
    "Controller",
    "Motor",
    "Mode",
    "MotorState",
    "FeedbackStats",
    "RegisterSpec",
    "DAMIAO_RW_REGISTERS",
    "DAMIAO_HIGH_IMPACT_RIDS",
    "DAMIAO_PROTECTION_RIDS",
    "get_damiao_register_spec",
    "RID_CTRL_MODE",
    "RID_MST_ID",
    "RID_ESC_ID",
    "RID_TIMEOUT",
    "RID_PMAX",
    "RID_VMAX",
    "RID_TMAX",
    "RID_KP_ASR",
    "RID_KI_ASR",
    "RID_KP_APR",
    "RID_KI_APR",
    "MODE_MIT",
    "MODE_POS_VEL",
    "MODE_VEL",
    "MODE_FORCE_POS",
    "MotorBridgeError",
    "AbiLoadError",
    "CallError",
]

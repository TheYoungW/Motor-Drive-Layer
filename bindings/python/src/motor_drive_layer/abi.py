import ctypes
import ctypes.util
import json
import os
import sys
from ctypes import POINTER, Structure, c_char_p, c_float, c_int32, c_uint8, c_uint16, c_uint32, c_uint64, c_void_p
from pathlib import Path

from .errors import AbiLoadError, CallError


class CState(Structure):
    _fields_ = [
        ("has_value", c_int32),
        ("can_id", c_uint8),
        ("arbitration_id", c_uint32),
        ("status_code", c_uint8),
        ("pos", c_float),
        ("vel", c_float),
        ("torq", c_float),
        ("t_mos", c_float),
        ("t_rotor", c_float),
    ]


class CFeedbackStats(Structure):
    _fields_ = [
        ("has_feedback", c_int32),
        ("update_count", c_uint64),
        ("age_ns", c_uint64),
    ]


class CFeedbackReport(Structure):
    _fields_ = [
        ("struct_size", c_uint32),
        ("timeout_ms", c_uint32),
        ("expected_count", c_uint32),
        ("received_count", c_uint32),
        ("missing_count", c_uint32),
    ]


MOTOR_OK = 0
MOTOR_ERROR_INVALID_ARGUMENT = 1
MOTOR_ERROR_TRANSPORT = 2
MOTOR_ERROR_FEEDBACK_TIMEOUT = 3
MOTOR_ERROR_FEEDBACK_INCOMPLETE = 4
MOTOR_ERROR_MOTOR_FAULT = 5


class CTransportCapabilities(Structure):
    _fields_ = [
        ("transport", ctypes.c_char * 32),
        ("max_payload_bytes", c_uint32),
        ("channel_count", c_uint32),
        ("can_fd", c_int32),
        ("parallel_batches", c_int32),
        ("hardware_rx_timestamps", c_int32),
        ("reconnect", c_int32),
        ("process_session_reuse", c_int32),
    ]


class CTransportHealth(Structure):
    _fields_ = [
        ("connected", c_int32),
        ("healthy", c_int32),
        ("tx_frames", c_uint64),
        ("rx_frames", c_uint64),
        ("send_errors", c_uint64),
        ("receive_errors", c_uint64),
        ("last_tx_age_ns", c_uint64),
        ("last_rx_age_ns", c_uint64),
        ("last_error", ctypes.c_char * 256),
    ]


class CDiscoveryCandidate(Structure):
    _fields_ = [
        ("role", c_char_p),
        ("motor_id", c_uint16),
        ("feedback_id", c_uint16),
        ("model", c_char_p),
        ("policy", c_int32),
    ]


class CDiscoveryResult(Structure):
    _fields_ = [
        ("role", ctypes.c_char * 64),
        ("motor_id", c_uint16),
        ("feedback_id", c_uint16),
        ("policy", c_int32),
        ("state", c_int32),
        ("motor", c_void_p),
        ("reason", ctypes.c_char * 256),
    ]


class CRegisterInfo(Structure):
    _fields_ = [
        ("has_value", c_int32),
        ("rid", c_uint8),
        ("access", c_uint8),
        ("data_type", c_uint8),
    ]


class CMitBatchCommand(Structure):
    _fields_ = [
        ("motor", c_void_p),
        ("target_position", c_float),
        ("target_velocity", c_float),
        ("stiffness", c_float),
        ("damping", c_float),
        ("feedforward_torque", c_float),
    ]


class CPosVelBatchCommand(Structure):
    _fields_ = [
        ("motor", c_void_p),
        ("target_position", c_float),
        ("velocity_limit", c_float),
    ]


def _candidate_lib_paths() -> list[Path]:
    candidates: list[Path] = []
    env = os.getenv("MOTOR_DRIVE_LAYER_LIB")
    if env:
        candidates.append(Path(env).expanduser())

    here = Path(__file__).resolve()
    pkg_lib = here.parent / "lib"
    candidates.extend(
        [
            pkg_lib / "libmotor_abi.so",
            pkg_lib / "libmotor_abi.dylib",
            pkg_lib / "motor_abi.dll",
        ]
    )

    # The source tree has enough parents to locate cpp_damiao/build.  An
    # installed package may live directly under a shallow --target directory,
    # so this fallback must not assume a fixed minimum path depth.
    if len(here.parents) > 4:
        repo_root = here.parents[4]
        candidates.extend(
            [
                repo_root / "build" / "cpp_damiao" / "libmotor_abi.so",
                repo_root / "build" / "cpp_damiao" / "libmotor_abi.dylib",
                repo_root / "build" / "cpp_damiao" / "motor_abi.dll",
                repo_root
                / "build"
                / "cpp_damiao"
                / "Release"
                / "libmotor_abi.so",
                repo_root
                / "build"
                / "cpp_damiao"
                / "Release"
                / "libmotor_abi.dylib",
                repo_root / "build" / "cpp_damiao" / "Release" / "motor_abi.dll",
                repo_root / "cpp_damiao" / "build" / "libmotor_abi.so",
                repo_root / "cpp_damiao" / "build" / "libmotor_abi.dylib",
                repo_root / "cpp_damiao" / "build" / "motor_abi.dll",
                repo_root / "cpp_damiao" / "build" / "Release" / "libmotor_abi.so",
                repo_root / "cpp_damiao" / "build" / "Release" / "libmotor_abi.dylib",
                repo_root / "cpp_damiao" / "build" / "Release" / "motor_abi.dll",
            ]
        )

    cwd = Path.cwd()
    candidates.extend(
        [
            cwd / "build" / "cpp_damiao" / "libmotor_abi.so",
            cwd / "build" / "cpp_damiao" / "libmotor_abi.dylib",
            cwd / "build" / "cpp_damiao" / "motor_abi.dll",
            cwd / "build" / "cpp_damiao" / "Release" / "libmotor_abi.so",
            cwd / "build" / "cpp_damiao" / "Release" / "libmotor_abi.dylib",
            cwd / "build" / "cpp_damiao" / "Release" / "motor_abi.dll",
            cwd / "cpp_damiao" / "build" / "libmotor_abi.so",
            cwd / "cpp_damiao" / "build" / "libmotor_abi.dylib",
            cwd / "cpp_damiao" / "build" / "motor_abi.dll",
            cwd / "cpp_damiao" / "build" / "Release" / "libmotor_abi.so",
            cwd / "cpp_damiao" / "build" / "Release" / "libmotor_abi.dylib",
            cwd / "cpp_damiao" / "build" / "Release" / "motor_abi.dll",
        ]
    )
    return candidates


def _articore_runtime_lib_name() -> str:
    if sys.platform.startswith("win"):
        return "articore_runtime.dll"
    if sys.platform == "darwin":
        return "libarticore_runtime.dylib"
    return "libarticore_runtime.so"


def _candidate_articore_runtime_paths() -> list[Path]:
    candidates: list[Path] = []
    env = os.getenv("ARTICORE_RUNTIME_LIB")
    if env:
        candidates.append(Path(env).expanduser())

    here = Path(__file__).resolve()
    lib_name = _articore_runtime_lib_name()
    candidates.append(here.parent / "lib" / lib_name)
    if len(here.parents) > 4:
        repo_root = here.parents[4]
        candidates.extend(
            [
                repo_root / "build" / "articore_runtime" / lib_name,
                repo_root
                / "build"
                / "articore_runtime"
                / "Release"
                / lib_name,
                repo_root
                / "cpp_damiao"
                / "build"
                / "articore_runtime"
                / lib_name,
                repo_root
                / "cpp_damiao"
                / "build"
                / "articore_runtime"
                / "Release"
                / lib_name,
            ]
        )
    cwd = Path.cwd()
    candidates.extend(
        [
            cwd / "build" / "articore_runtime" / lib_name,
            cwd / "build" / "articore_runtime" / "Release" / lib_name,
            cwd / "cpp_damiao" / "build" / "articore_runtime" / lib_name,
            cwd
            / "cpp_damiao"
            / "build"
            / "articore_runtime"
            / "Release"
            / lib_name,
        ]
    )
    return candidates


def articore_runtime_library_path() -> str:
    """Return the packaged Articore native runtime shared-library path."""
    tried: list[str] = []
    for path in _candidate_articore_runtime_paths():
        tried.append(str(path))
        if path.exists():
            return str(path)
    found = ctypes.util.find_library("articore_runtime")
    if found:
        return found
    raise AbiLoadError(
        "Failed to locate the Articore native runtime shared library. Tried:\n"
        + "\n".join(f"- {path}" for path in tried)
        + "\nHint: install a motor-drive-layer wheel that includes "
        "libarticore_runtime."
    )


def _articore_runtime_metadata() -> tuple[int, int]:
    library = ctypes.CDLL(articore_runtime_library_path())
    library.articore_runtime_abi_version.restype = c_uint32
    library.articore_runtime_capabilities.restype = c_uint64
    return (
        int(library.articore_runtime_abi_version()),
        int(library.articore_runtime_capabilities()),
    )


def articore_runtime_abi_version() -> str:
    """Return the separately versioned Articore runtime ABI version."""
    version, _ = _articore_runtime_metadata()
    return f"{version >> 16}.{version & 0xFFFF}"


def articore_runtime_capabilities() -> dict[str, bool]:
    """Return product-runtime capabilities independently of motor features."""
    _, bits = _articore_runtime_metadata()
    return {
        "command_watchdog": bool(bits & (1 << 0)),
        "safe_hold": bool(bits & (1 << 1)),
        "gripper_protection": bool(bits & (1 << 2)),
        "single_channel": bool(bits & (1 << 3)),
        "dual_channel": bool(bits & (1 << 4)),
        "transport_health": bool(bits & (1 << 5)),
        "current_position_hold": bool(bits & (1 << 6)),
        "motor_presence": bool(bits & (1 << 7)),
        "realtime_joint_mailbox": bool(bits & (1 << 8)),
        "joint_trajectory": bool(bits & (1 << 9)),
        "atomic_enable": bool(bits & (1 << 10)),
    }


def _platform_dm_device_lib_name() -> str:
    if sys.platform.startswith("win"):
        return "dm_device.dll"
    if sys.platform == "darwin":
        return "libdm_device.dylib"
    return "libdm_device.so"


def _configure_packaged_dm_device_runtime() -> None:
    if os.getenv("MOTOR_DM_DEVICE_LIB"):
        return

    pkg_dm_lib = Path(__file__).resolve().parent / "lib" / "dm_device" / _platform_dm_device_lib_name()
    if pkg_dm_lib.exists():
        os.environ["MOTOR_DM_DEVICE_LIB"] = str(pkg_dm_lib)


def _load_library() -> ctypes.CDLL:
    _configure_packaged_dm_device_runtime()
    tried: list[str] = []
    for p in _candidate_lib_paths():
        tried.append(str(p))
        if p.exists():
            return ctypes.CDLL(str(p))

    found = ctypes.util.find_library("motor_abi")
    if found:
        return ctypes.CDLL(found)

    raise AbiLoadError(
        "Failed to load motor_abi shared library. Tried:\n"
        + "\n".join(f"- {x}" for x in tried)
        + "\nHint: build native libraries first: "
        "cmake -S . -B build && cmake --build build"
    )


class Abi:
    def __init__(self) -> None:
        self.lib = _load_library()
        self._bind()

    def _bind(self) -> None:
        lib = self.lib

        lib.motor_last_error_message.restype = c_char_p
        lib.motor_abi_version.restype = c_char_p
        lib.motor_abi_capabilities_json.restype = c_char_p
        lib.motor_damiao_register_info.argtypes = [c_uint8, POINTER(CRegisterInfo)]
        lib.motor_damiao_register_info.restype = c_int32

        lib.motor_controller_new_socketcan.argtypes = [c_char_p]
        lib.motor_controller_new_socketcan.restype = c_void_p
        lib.motor_controller_new_socketcanfd.argtypes = [c_char_p]
        lib.motor_controller_new_socketcanfd.restype = c_void_p
        lib.motor_controller_new_dm_serial.argtypes = [c_char_p, c_uint32]
        lib.motor_controller_new_dm_serial.restype = c_void_p
        lib.motor_controller_new_dm_device.argtypes = [c_char_p, c_char_p]
        lib.motor_controller_new_dm_device.restype = c_void_p
        # Additive in motor-drive-layer 0.5.7.  Keep the Python package able to
        # load an older native library and fall back to its fixed 1M/5M entry.
        self.has_dm_device_ex = hasattr(lib, "motor_controller_new_dm_device_ex")
        if self.has_dm_device_ex:
            lib.motor_controller_new_dm_device_ex.argtypes = [
                c_char_p,
                c_char_p,
                c_uint32,
                c_uint32,
            ]
            lib.motor_controller_new_dm_device_ex.restype = c_void_p
        self.has_controller_group = all(
            hasattr(lib, name)
            for name in (
                "motor_controller_group_new",
                "motor_controller_group_free",
                "motor_controller_group_send_mit",
                "motor_controller_group_send_pos_vel",
            )
        )
        if self.has_controller_group:
            lib.motor_controller_group_new.argtypes = [POINTER(c_void_p), c_uint32]
            lib.motor_controller_group_new.restype = c_void_p
            lib.motor_controller_group_free.argtypes = [c_void_p]
            lib.motor_controller_group_send_mit.argtypes = [
                c_void_p,
                POINTER(CMitBatchCommand),
                c_uint32,
            ]
            lib.motor_controller_group_send_mit.restype = c_int32
            lib.motor_controller_group_send_pos_vel.argtypes = [
                c_void_p,
                POINTER(CPosVelBatchCommand),
                c_uint32,
            ]
            lib.motor_controller_group_send_pos_vel.restype = c_int32
        lib.motor_controller_free.argtypes = [c_void_p]
        lib.motor_controller_poll_feedback_once.argtypes = [c_void_p]
        lib.motor_controller_poll_feedback_once.restype = c_int32
        lib.motor_controller_request_feedback_all.argtypes = [c_void_p, c_uint32]
        lib.motor_controller_request_feedback_all.restype = c_int32
        self.has_structured_feedback_report = hasattr(
            lib, "motor_controller_request_feedback_all_ex"
        )
        if self.has_structured_feedback_report:
            lib.motor_controller_request_feedback_all_ex.argtypes = [
                c_void_p,
                c_uint32,
                POINTER(CFeedbackReport),
                POINTER(c_uint32),
                c_uint32,
            ]
            lib.motor_controller_request_feedback_all_ex.restype = c_int32
        lib.motor_controller_enable_all.argtypes = [c_void_p]
        lib.motor_controller_enable_all.restype = c_int32
        lib.motor_controller_disable_all.argtypes = [c_void_p]
        lib.motor_controller_disable_all.restype = c_int32
        lib.motor_controller_shutdown.argtypes = [c_void_p]
        lib.motor_controller_shutdown.restype = c_int32
        lib.motor_controller_close_bus.argtypes = [c_void_p]
        lib.motor_controller_close_bus.restype = c_int32
        lib.motor_controller_set_tx_gap_us.argtypes = [c_void_p, c_uint32]
        lib.motor_controller_set_tx_gap_us.restype = c_int32
        self.has_transport_capabilities = hasattr(
            lib, "motor_controller_get_transport_capabilities"
        )
        if self.has_transport_capabilities:
            lib.motor_controller_get_transport_capabilities.argtypes = [
                c_void_p,
                POINTER(CTransportCapabilities),
            ]
            lib.motor_controller_get_transport_capabilities.restype = c_int32
        self.has_transport_health = hasattr(lib, "motor_controller_get_transport_health")
        if self.has_transport_health:
            lib.motor_controller_get_transport_health.argtypes = [
                c_void_p,
                POINTER(CTransportHealth),
            ]
            lib.motor_controller_get_transport_health.restype = c_int32

        lib.motor_controller_add_damiao_motor.argtypes = [c_void_p, c_uint16, c_uint16, c_char_p]
        lib.motor_controller_add_damiao_motor.restype = c_void_p
        self.has_motor_presence_discovery = hasattr(
            lib, "motor_controller_discover_damiao_motors"
        )
        if self.has_motor_presence_discovery:
            lib.motor_controller_discover_damiao_motors.argtypes = [
                c_void_p,
                POINTER(CDiscoveryCandidate),
                c_uint32,
                c_uint32,
                c_uint32,
                POINTER(CDiscoveryResult),
                c_uint32,
            ]
            lib.motor_controller_discover_damiao_motors.restype = c_int32

        lib.motor_handle_free.argtypes = [c_void_p]
        lib.motor_handle_enable.argtypes = [c_void_p]
        lib.motor_handle_enable.restype = c_int32
        lib.motor_handle_disable.argtypes = [c_void_p]
        lib.motor_handle_disable.restype = c_int32
        lib.motor_handle_clear_error.argtypes = [c_void_p]
        lib.motor_handle_clear_error.restype = c_int32
        lib.motor_handle_set_zero_position.argtypes = [c_void_p]
        lib.motor_handle_set_zero_position.restype = c_int32
        lib.motor_handle_ensure_mode.argtypes = [c_void_p, c_uint32, c_uint32]
        lib.motor_handle_ensure_mode.restype = c_int32

        lib.motor_handle_send_mit.argtypes = [c_void_p, c_float, c_float, c_float, c_float, c_float]
        lib.motor_handle_send_mit.restype = c_int32
        lib.motor_handle_send_pos_vel.argtypes = [c_void_p, c_float, c_float]
        lib.motor_handle_send_pos_vel.restype = c_int32
        lib.motor_handle_send_vel.argtypes = [c_void_p, c_float]
        lib.motor_handle_send_vel.restype = c_int32
        lib.motor_handle_send_force_pos.argtypes = [c_void_p, c_float, c_float, c_float]
        lib.motor_handle_send_force_pos.restype = c_int32

        lib.motor_handle_store_parameters.argtypes = [c_void_p]
        lib.motor_handle_store_parameters.restype = c_int32
        lib.motor_handle_request_feedback.argtypes = [c_void_p]
        lib.motor_handle_request_feedback.restype = c_int32
        lib.motor_handle_request_fresh_state.argtypes = [c_void_p, c_uint32, POINTER(CState)]
        lib.motor_handle_request_fresh_state.restype = c_int32
        lib.motor_handle_set_can_timeout_ms.argtypes = [c_void_p, c_uint32]
        lib.motor_handle_set_can_timeout_ms.restype = c_int32

        lib.motor_handle_write_register_f32.argtypes = [c_void_p, c_uint8, c_float]
        lib.motor_handle_write_register_f32.restype = c_int32
        lib.motor_handle_write_register_u32.argtypes = [c_void_p, c_uint8, c_uint32]
        lib.motor_handle_write_register_u32.restype = c_int32
        lib.motor_handle_get_register_f32.argtypes = [c_void_p, c_uint8, c_uint32, POINTER(c_float)]
        lib.motor_handle_get_register_f32.restype = c_int32
        lib.motor_handle_get_register_u32.argtypes = [c_void_p, c_uint8, c_uint32, POINTER(c_uint32)]
        lib.motor_handle_get_register_u32.restype = c_int32

        lib.motor_handle_get_state.argtypes = [c_void_p, POINTER(CState)]
        lib.motor_handle_get_state.restype = c_int32
        lib.motor_handle_get_feedback_stats.argtypes = [c_void_p, POINTER(CFeedbackStats)]
        lib.motor_handle_get_feedback_stats.restype = c_int32

        lib.motor_handle_damiao_get_param_f32.argtypes = [c_void_p, c_uint16, c_uint32, POINTER(c_float)]
        lib.motor_handle_damiao_get_param_f32.restype = c_int32
        lib.motor_handle_damiao_get_param_u32.argtypes = [c_void_p, c_uint16, c_uint32, POINTER(c_uint32)]
        lib.motor_handle_damiao_get_param_u32.restype = c_int32
        lib.motor_handle_damiao_write_param_f32.argtypes = [c_void_p, c_uint16, c_float]
        lib.motor_handle_damiao_write_param_f32.restype = c_int32
        lib.motor_handle_damiao_write_param_u32.argtypes = [c_void_p, c_uint16, c_uint32]
        lib.motor_handle_damiao_write_param_u32.restype = c_int32


_abi_singleton: Abi | None = None


def get_abi() -> Abi:
    global _abi_singleton
    if _abi_singleton is None:
        _abi_singleton = Abi()
    return _abi_singleton


def abi_version() -> str:
    msg = get_abi().lib.motor_abi_version()
    return msg.decode() if msg else ""


def abi_capabilities() -> dict:
    msg = get_abi().lib.motor_abi_capabilities_json()
    return json.loads(msg.decode() if msg else "{}")


def damiao_register_info(rid: int) -> tuple[str, str] | None:
    """Return (access, data_type) from the native canonical register table."""
    value = int(rid)
    if value < 0 or value > 0xFF:
        return None
    info = CRegisterInfo()
    rc = get_abi().lib.motor_damiao_register_info(value, ctypes.byref(info))
    if rc != 0:
        msg = get_abi().lib.motor_last_error_message()
        raise CallError(msg.decode() if msg else "motor_damiao_register_info failed")
    if not info.has_value:
        return None
    access = {0: "RO", 1: "RW"}.get(int(info.access))
    data_type = {0: "f32", 1: "u32"}.get(int(info.data_type))
    if access is None or data_type is None:
        raise CallError("motor_damiao_register_info returned invalid metadata")
    return access, data_type

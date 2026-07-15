from __future__ import annotations

import ctypes
from ctypes import c_float, c_uint32

from .abi import CFeedbackStats, CState, get_abi
from .dm_device_runtime import ensure_dm_device_runtime
from .errors import CallError
from .models import FeedbackStats, Mode, MotorState


def _err_text() -> str:
    msg = get_abi().lib.motor_last_error_message()
    return msg.decode() if msg else "unknown error"


def _ok(rc: int, what: str) -> None:
    if rc != 0:
        raise CallError(f"{what} failed: {_err_text()}")


def _timeout_u32(timeout_ms: int) -> int:
    value = int(timeout_ms)
    if value < 0 or value > 0xFFFFFFFF:
        raise ValueError("timeout_ms must be in 0..=4294967295")
    return value


def _state_from_c(state: CState) -> MotorState | None:
    if not state.has_value:
        return None
    return MotorState(
        can_id=int(state.can_id),
        arbitration_id=int(state.arbitration_id),
        status_code=int(state.status_code),
        pos=float(state.pos),
        vel=float(state.vel),
        torq=float(state.torq),
        t_mos=float(state.t_mos),
        t_rotor=float(state.t_rotor),
    )


class Controller:
    """Own one native bus and the motor handles added to that bus."""

    def __init__(self, channel: str = "can0") -> None:
        self._abi = get_abi()
        self._ptr = self._abi.lib.motor_controller_new_socketcan(channel.encode())
        if not self._ptr:
            raise CallError(f"new_socketcan failed: {_err_text()}")

    @classmethod
    def from_socketcanfd(cls, channel: str = "can0") -> "Controller":
        self = cls.__new__(cls)
        self._abi = get_abi()
        self._ptr = self._abi.lib.motor_controller_new_socketcanfd(channel.encode())
        if not self._ptr:
            raise CallError(f"new_socketcanfd failed: {_err_text()}")
        return self

    @classmethod
    def from_dm_serial(cls, serial_port: str = "/dev/ttyACM0", baud: int = 1_000_000) -> "Controller":
        self = cls.__new__(cls)
        self._abi = get_abi()
        self._ptr = self._abi.lib.motor_controller_new_dm_serial(serial_port.encode(), int(baud))
        if not self._ptr:
            raise CallError(f"new_dm_serial failed: {_err_text()}")
        return self

    @classmethod
    def from_dm_device(
        cls,
        dm_device_type: str = "usb2canfd-dual",
        dm_channel: str = "0",
    ) -> "Controller":
        self = cls.__new__(cls)
        ensure_dm_device_runtime(quiet=True)
        self._abi = get_abi()
        self._ptr = self._abi.lib.motor_controller_new_dm_device(
            dm_device_type.encode(),
            dm_channel.encode(),
        )
        if not self._ptr:
            raise CallError(f"new_dm_device failed: {_err_text()}")
        return self

    def close(self) -> None:
        if self._ptr:
            self._abi.lib.motor_controller_free(self._ptr)
            self._ptr = None

    @property
    def closed(self) -> bool:
        return not bool(self._ptr)

    def _require_open(self) -> int:
        if not self._ptr:
            raise CallError("controller is closed")
        return self._ptr

    def shutdown(self) -> None:
        _ok(self._abi.lib.motor_controller_shutdown(self._require_open()), "controller_shutdown")

    def close_bus(self) -> None:
        _ok(self._abi.lib.motor_controller_close_bus(self._require_open()), "controller_close_bus")

    def enable_all(self) -> None:
        _ok(self._abi.lib.motor_controller_enable_all(self._require_open()), "enable_all")

    def disable_all(self) -> None:
        _ok(self._abi.lib.motor_controller_disable_all(self._require_open()), "disable_all")

    def poll_feedback_once(self) -> None:
        _ok(
            self._abi.lib.motor_controller_poll_feedback_once(self._require_open()),
            "poll_feedback_once",
        )

    def request_feedback_all(self, timeout_ms: int = 50) -> None:
        """Request one fresh feedback frame from every motor or raise on timeout."""
        _ok(
            self._abi.lib.motor_controller_request_feedback_all(
                self._require_open(), _timeout_u32(timeout_ms)
            ),
            "request_feedback_all",
        )

    def set_tx_gap_us(self, gap_us: int) -> None:
        value = int(gap_us)
        if value < 0 or value > 0xFFFFFFFF:
            raise ValueError("gap_us must be in 0..=4294967295")
        _ok(
            self._abi.lib.motor_controller_set_tx_gap_us(self._require_open(), value),
            "set_tx_gap_us",
        )

    def add_damiao_motor(self, motor_id: int, feedback_id: int, model: str) -> "Motor":
        m = self._abi.lib.motor_controller_add_damiao_motor(
            self._require_open(), motor_id, feedback_id, model.encode()
        )
        if not m:
            raise CallError(f"add_damiao_motor failed: {_err_text()}")
        return Motor(m, self)

    def __enter__(self) -> "Controller":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        try:
            self.shutdown()
        finally:
            self.close()


class Motor:
    """A native motor handle whose parent Controller must remain open."""

    def __init__(self, ptr: int, controller: Controller | None = None) -> None:
        self._abi = get_abi()
        self._ptr = ptr
        self._controller = controller

    def close(self) -> None:
        if self._ptr:
            # Freeing the ABI wrapper remains valid after the parent controller
            # closes; operational methods are rejected by _require_open().
            self._abi.lib.motor_handle_free(self._ptr)
            self._ptr = None

    @property
    def closed(self) -> bool:
        return not bool(self._ptr)

    def _require_open(self) -> int:
        if not self._ptr:
            raise CallError("motor handle is closed")
        controller = getattr(self, "_controller", None)
        if controller is not None and controller.closed:
            raise CallError("motor controller is closed")
        return self._ptr

    def __enter__(self) -> "Motor":
        self._require_open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def enable(self) -> None:
        _ok(self._abi.lib.motor_handle_enable(self._require_open()), "enable")

    def disable(self) -> None:
        _ok(self._abi.lib.motor_handle_disable(self._require_open()), "disable")

    def clear_error(self) -> None:
        _ok(self._abi.lib.motor_handle_clear_error(self._require_open()), "clear_error")

    def set_zero_position(self) -> None:
        _ok(self._abi.lib.motor_handle_set_zero_position(self._require_open()), "set_zero_position")

    def ensure_mode(self, mode: Mode | int, timeout_ms: int = 1000) -> None:
        _ok(self._abi.lib.motor_handle_ensure_mode(self._require_open(), int(mode), timeout_ms), "ensure_mode")

    def send_mit(self, pos: float, vel: float, kp: float, kd: float, tau: float) -> None:
        _ok(self._abi.lib.motor_handle_send_mit(self._require_open(), pos, vel, kp, kd, tau), "send_mit")

    def send_pos_vel(self, pos: float, vlim: float) -> None:
        _ok(self._abi.lib.motor_handle_send_pos_vel(self._require_open(), pos, vlim), "send_pos_vel")

    def send_vel(self, vel: float) -> None:
        _ok(self._abi.lib.motor_handle_send_vel(self._require_open(), vel), "send_vel")

    def send_force_pos(self, pos: float, vlim: float, ratio: float) -> None:
        _ok(
            self._abi.lib.motor_handle_send_force_pos(self._require_open(), pos, vlim, ratio),
            "send_force_pos",
        )

    def request_feedback(self) -> None:
        _ok(self._abi.lib.motor_handle_request_feedback(self._require_open()), "request_feedback")

    def request_fresh_state(self, timeout_ms: int = 50) -> MotorState:
        """Request feedback and wait for a newer state than the cached sample."""
        state = CState()
        _ok(
            self._abi.lib.motor_handle_request_fresh_state(
                self._require_open(), _timeout_u32(timeout_ms), ctypes.byref(state)
            ),
            "request_fresh_state",
        )
        result = _state_from_c(state)
        if result is None:
            raise CallError("request_fresh_state returned no state")
        return result

    def set_can_timeout_ms(self, timeout_ms: int) -> None:
        _ok(self._abi.lib.motor_handle_set_can_timeout_ms(self._require_open(), timeout_ms), "set_can_timeout_ms")

    def store_parameters(self) -> None:
        _ok(self._abi.lib.motor_handle_store_parameters(self._require_open()), "store_parameters")

    def write_register_f32(self, rid: int, value: float) -> None:
        _ok(self._abi.lib.motor_handle_write_register_f32(self._require_open(), rid, value), "write_register_f32")

    def write_register_u32(self, rid: int, value: int) -> None:
        _ok(self._abi.lib.motor_handle_write_register_u32(self._require_open(), rid, value), "write_register_u32")

    def get_register_f32(self, rid: int, timeout_ms: int = 1000) -> float:
        out = c_float(0.0)
        _ok(
            self._abi.lib.motor_handle_get_register_f32(
                self._require_open(), rid, timeout_ms, ctypes.byref(out)
            ),
            "get_register_f32",
        )
        return float(out.value)

    def get_register_u32(self, rid: int, timeout_ms: int = 1000) -> int:
        out = c_uint32(0)
        _ok(
            self._abi.lib.motor_handle_get_register_u32(
                self._require_open(), rid, timeout_ms, ctypes.byref(out)
            ),
            "get_register_u32",
        )
        return int(out.value)

    def damiao_get_param_f32(self, param_id: int, timeout_ms: int = 1000) -> float:
        out = c_float(0.0)
        _ok(
            self._abi.lib.motor_handle_damiao_get_param_f32(
                self._require_open(), param_id, timeout_ms, ctypes.byref(out)
            ),
            "damiao_get_param_f32",
        )
        return float(out.value)

    def damiao_get_param_u32(self, param_id: int, timeout_ms: int = 1000) -> int:
        out = c_uint32(0)
        _ok(
            self._abi.lib.motor_handle_damiao_get_param_u32(
                self._require_open(), param_id, timeout_ms, ctypes.byref(out)
            ),
            "damiao_get_param_u32",
        )
        return int(out.value)

    def damiao_write_param_f32(self, param_id: int, value: float) -> None:
        _ok(self._abi.lib.motor_handle_damiao_write_param_f32(self._require_open(), param_id, value), "damiao_write_param_f32")

    def damiao_write_param_u32(self, param_id: int, value: int) -> None:
        _ok(self._abi.lib.motor_handle_damiao_write_param_u32(self._require_open(), param_id, value), "damiao_write_param_u32")

    def get_state(self) -> MotorState | None:
        st = CState()
        _ok(self._abi.lib.motor_handle_get_state(self._require_open(), ctypes.byref(st)), "get_state")
        return _state_from_c(st)

    def get_feedback_stats(self) -> FeedbackStats:
        stats = CFeedbackStats()
        _ok(
            self._abi.lib.motor_handle_get_feedback_stats(
                self._require_open(), ctypes.byref(stats)
            ),
            "get_feedback_stats",
        )
        return FeedbackStats(
            has_feedback=bool(stats.has_feedback),
            update_count=int(stats.update_count),
            age_ns=int(stats.age_ns),
        )

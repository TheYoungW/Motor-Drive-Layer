from __future__ import annotations

import ctypes
from collections.abc import Sequence
from ctypes import c_float, c_uint32
from numbers import Real
from threading import Lock

from .abi import (
    CFeedbackStats,
    CMitBatchCommand,
    CPosVelBatchCommand,
    CState,
    CTransportCapabilities,
    CTransportHealth,
    get_abi,
)
from .dm_device_runtime import ensure_dm_device_runtime
from .errors import CallError
from .models import (
    FeedbackStats,
    MitCommand,
    Mode,
    MotorState,
    PosVelCommand,
    TransportCapabilities,
    TransportHealth,
)


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
        dm_channel: str | int = "0",
        bitrate: int = 1_000_000,
        data_bitrate: int = 5_000_000,
        *,
        device: str | None = None,
        channel: str | int | None = None,
    ) -> "Controller":
        """Open a DaMiao device using its original DM_Device firmware/runtime.

        ``device``/``channel`` are the preferred keyword names.  The original
        ``dm_device_type``/``dm_channel`` names and positional form remain
        supported for compatibility.
        """
        if device is not None:
            if dm_device_type != "usb2canfd-dual" and dm_device_type != device:
                raise ValueError("device and dm_device_type specify different values")
            dm_device_type = device
        if channel is not None:
            if str(dm_channel) != "0" and str(dm_channel) != str(channel):
                raise ValueError("channel and dm_channel specify different values")
            dm_channel = channel
        bitrate_value = int(bitrate)
        data_bitrate_value = int(data_bitrate)
        if not 1 <= bitrate_value <= 0xFFFFFFFF:
            raise ValueError("bitrate must be in 1..=4294967295")
        if not 1 <= data_bitrate_value <= 0xFFFFFFFF:
            raise ValueError("data_bitrate must be in 1..=4294967295")

        self = cls.__new__(cls)
        try:
            ensure_dm_device_runtime(quiet=True)
        except RuntimeError as exc:
            raise CallError(f"DM_Device runtime setup failed: {exc}") from exc
        self._abi = get_abi()
        if self._abi.has_dm_device_ex:
            self._ptr = self._abi.lib.motor_controller_new_dm_device_ex(
                dm_device_type.encode(),
                str(dm_channel).encode(),
                bitrate_value,
                data_bitrate_value,
            )
        else:
            if (bitrate_value, data_bitrate_value) != (1_000_000, 5_000_000):
                raise CallError(
                    "the loaded motor_abi does not support configurable DM_Device bitrates; "
                    "upgrade motor-drive-layer or use 1,000,000/5,000,000"
                )
            self._ptr = self._abi.lib.motor_controller_new_dm_device(
                dm_device_type.encode(),
                str(dm_channel).encode(),
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

    def transport_capabilities(self) -> TransportCapabilities:
        if not self._abi.has_transport_capabilities:
            raise CallError(
                "the loaded motor_abi does not support transport capabilities; "
                "upgrade motor-drive-layer"
            )
        native = CTransportCapabilities()
        _ok(
            self._abi.lib.motor_controller_get_transport_capabilities(
                self._require_open(), ctypes.byref(native)
            ),
            "controller_get_transport_capabilities",
        )
        return TransportCapabilities(
            transport=bytes(native.transport).split(b"\0", 1)[0].decode(),
            max_payload_bytes=int(native.max_payload_bytes),
            channel_count=int(native.channel_count),
            can_fd=bool(native.can_fd),
            parallel_batches=bool(native.parallel_batches),
            hardware_rx_timestamps=bool(native.hardware_rx_timestamps),
            reconnect=bool(native.reconnect),
            process_session_reuse=bool(native.process_session_reuse),
        )

    def transport_health(self) -> TransportHealth:
        if not self._abi.has_transport_health:
            raise CallError(
                "the loaded motor_abi does not support transport health; "
                "upgrade motor-drive-layer"
            )
        native = CTransportHealth()
        _ok(
            self._abi.lib.motor_controller_get_transport_health(
                self._require_open(), ctypes.byref(native)
            ),
            "controller_get_transport_health",
        )
        missing_age = (1 << 64) - 1
        last_error = bytes(native.last_error).split(b"\0", 1)[0]
        return TransportHealth(
            connected=bool(native.connected),
            healthy=bool(native.healthy),
            tx_frames=int(native.tx_frames),
            rx_frames=int(native.rx_frames),
            send_errors=int(native.send_errors),
            receive_errors=int(native.receive_errors),
            last_tx_age_ns=(
                None if native.last_tx_age_ns == missing_age else int(native.last_tx_age_ns)
            ),
            last_rx_age_ns=(
                None if native.last_rx_age_ns == missing_age else int(native.last_rx_age_ns)
            ),
            last_error=last_error.decode(errors="replace") if last_error else None,
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


class ControllerGroup:
    """Persistent native workers for synchronized multi-controller batches."""

    def __init__(self, controllers: Sequence[Controller]) -> None:
        values = tuple(controllers)
        if not values:
            raise ValueError("controllers must not be empty")
        if len({id(controller) for controller in values}) != len(values):
            raise ValueError("controllers must not contain duplicates")
        self._abi = get_abi()
        self._call_lock = Lock()
        self._ptr = None
        if not self._abi.has_controller_group:
            raise CallError(
                "the loaded motor_abi does not support ControllerGroup; "
                "upgrade motor-drive-layer"
            )
        self._controllers = values
        pointers = (ctypes.c_void_p * len(values))(
            *(controller._require_open() for controller in values)
        )
        self._ptr = self._abi.lib.motor_controller_group_new(pointers, len(values))
        if not self._ptr:
            raise CallError(f"controller_group_new failed: {_err_text()}")

    @property
    def closed(self) -> bool:
        return not bool(self._ptr)

    def _require_open(self) -> int:
        if not self._ptr:
            raise CallError("controller group is closed")
        for index, controller in enumerate(self._controllers):
            if controller.closed:
                raise CallError(f"controller group member {index} is closed")
        return self._ptr

    def _validate_motor(self, motor: Motor) -> int:
        ptr = motor._require_open()
        if getattr(motor, "_controller", None) not in self._controllers:
            raise ValueError("command motor does not belong to this ControllerGroup")
        return ptr

    def send_mit(self, commands: Sequence[MitCommand]) -> None:
        values = tuple(commands)
        native = (CMitBatchCommand * len(values))(
            *(
                CMitBatchCommand(
                    self._validate_motor(command.motor),
                    command.pos,
                    command.vel,
                    command.kp,
                    command.kd,
                    command.tau,
                )
                for command in values
            )
        )
        with self._call_lock:
            _ok(
                self._abi.lib.motor_controller_group_send_mit(
                    self._require_open(), native if values else None, len(values)
                ),
                "controller_group_send_mit",
            )

    def send_pos_vel(self, commands: Sequence[PosVelCommand]) -> None:
        values = tuple(commands)
        native = (CPosVelBatchCommand * len(values))(
            *(
                CPosVelBatchCommand(
                    self._validate_motor(command.motor),
                    command.pos,
                    command.vlim,
                )
                for command in values
            )
        )
        with self._call_lock:
            _ok(
                self._abi.lib.motor_controller_group_send_pos_vel(
                    self._require_open(), native if values else None, len(values)
                ),
                "controller_group_send_pos_vel",
            )

    def prepare_mit(self, motors: Sequence[Motor]) -> "PreparedMitBatch":
        return PreparedMitBatch(self, motors)

    def prepare_pos_vel(self, motors: Sequence[Motor]) -> "PreparedPosVelBatch":
        return PreparedPosVelBatch(self, motors)

    def close(self) -> None:
        with self._call_lock:
            if self._ptr:
                self._abi.lib.motor_controller_group_free(self._ptr)
                self._ptr = None

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass

    def __enter__(self) -> "ControllerGroup":
        self._require_open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


def _check_vector(name: str, values: float | Sequence[float], count: int) -> None:
    if isinstance(values, Real):
        return
    if len(values) != count:
        raise ValueError(f"{name} must contain exactly {count} values")


def _vector_value(values: float | Sequence[float], index: int) -> float:
    if isinstance(values, Real):
        return float(values)
    return float(values[index])


class PreparedPosVelBatch:
    """Reusable fixed-motor POS_VEL batch with one preallocated ctypes array."""

    def __init__(self, group: ControllerGroup, motors: Sequence[Motor]) -> None:
        self._group = group
        self._motors = tuple(motors)
        if not self._motors:
            raise ValueError("motors must not be empty")
        group._require_open()
        pointers = tuple(group._validate_motor(motor) for motor in self._motors)
        if len(set(pointers)) != len(pointers):
            raise ValueError("motors must not contain duplicates")
        self._native = (CPosVelBatchCommand * len(pointers))(
            *(CPosVelBatchCommand(pointer, 0.0, 0.0) for pointer in pointers)
        )
        self._pointers = pointers
        self._lock = Lock()

    @property
    def motor_count(self) -> int:
        return len(self._motors)

    def send(
        self,
        positions: Sequence[float],
        velocity_limits: float | Sequence[float],
    ) -> None:
        count = self.motor_count
        _check_vector("positions", positions, count)
        _check_vector("velocity_limits", velocity_limits, count)
        with self._lock, self._group._call_lock:
            self._group._require_open()
            for motor, pointer in zip(self._motors, self._pointers):
                if self._group._validate_motor(motor) != pointer:
                    raise CallError("prepared batch motor handle changed")
            for index in range(count):
                self._native[index].target_position = float(positions[index])
                self._native[index].velocity_limit = _vector_value(
                    velocity_limits, index
                )
            _ok(
                self._group._abi.lib.motor_controller_group_send_pos_vel(
                    self._group._require_open(), self._native, count
                ),
                "controller_group_send_pos_vel",
            )


class PreparedMitBatch:
    """Reusable fixed-motor MIT batch with one preallocated ctypes array."""

    def __init__(self, group: ControllerGroup, motors: Sequence[Motor]) -> None:
        self._group = group
        self._motors = tuple(motors)
        if not self._motors:
            raise ValueError("motors must not be empty")
        group._require_open()
        pointers = tuple(group._validate_motor(motor) for motor in self._motors)
        if len(set(pointers)) != len(pointers):
            raise ValueError("motors must not contain duplicates")
        self._native = (CMitBatchCommand * len(pointers))(
            *(CMitBatchCommand(pointer, 0.0, 0.0, 0.0, 0.0, 0.0) for pointer in pointers)
        )
        self._pointers = pointers
        self._lock = Lock()

    @property
    def motor_count(self) -> int:
        return len(self._motors)

    def send(
        self,
        positions: Sequence[float],
        velocities: float | Sequence[float],
        stiffness: float | Sequence[float],
        damping: float | Sequence[float],
        feedforward_torques: float | Sequence[float],
    ) -> None:
        count = self.motor_count
        _check_vector("positions", positions, count)
        _check_vector("velocities", velocities, count)
        _check_vector("stiffness", stiffness, count)
        _check_vector("damping", damping, count)
        _check_vector("feedforward_torques", feedforward_torques, count)
        with self._lock, self._group._call_lock:
            self._group._require_open()
            for motor, pointer in zip(self._motors, self._pointers):
                if self._group._validate_motor(motor) != pointer:
                    raise CallError("prepared batch motor handle changed")
            for index in range(count):
                command = self._native[index]
                command.target_position = float(positions[index])
                command.target_velocity = _vector_value(velocities, index)
                command.stiffness = _vector_value(stiffness, index)
                command.damping = _vector_value(damping, index)
                command.feedforward_torque = _vector_value(
                    feedforward_torques, index
                )
            _ok(
                self._group._abi.lib.motor_controller_group_send_mit(
                    self._group._require_open(), self._native, count
                ),
                "controller_group_send_mit",
            )

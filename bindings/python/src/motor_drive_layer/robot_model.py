from __future__ import annotations

import ctypes
from dataclasses import dataclass
from enum import IntEnum
from math import isfinite
from pathlib import Path
from typing import Sequence

from .abi import articore_runtime_library_path
from .errors import RuntimeCallError


class RobotSide(IntEnum):
    LEFT = 0
    RIGHT = 1


class JacobianReference(IntEnum):
    LOCAL = 0
    WORLD = 1
    LOCAL_WORLD_ALIGNED = 2


@dataclass(frozen=True)
class RobotModelInfo:
    product_id: str
    side: RobotSide
    dof: int
    joint_names: tuple[str, ...]
    end_effector_frame: str
    lower_limits: tuple[float, ...]
    upper_limits: tuple[float, ...]


@dataclass(frozen=True)
class RobotPose:
    position: tuple[float, float, float]
    rotation: tuple[tuple[float, float, float], ...]
    homogeneous: tuple[tuple[float, float, float, float], ...]


@dataclass(frozen=True)
class IkOptions:
    max_iterations: int = 1000
    max_retries: int = 8
    tolerance: float = 1e-4
    step_size: float = 0.5
    damping: float = 1e-6
    random_seed: int = 0


@dataclass(frozen=True)
class IkResult:
    q: tuple[float, ...]
    success: bool
    error: float
    iterations: int


_MAX_DOF = 16
_Double = ctypes.c_double
_DoubleP = ctypes.POINTER(_Double)


class _CInfo(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32), ("dof", ctypes.c_uint32),
        ("side", ctypes.c_uint32), ("product_id", ctypes.c_char * 64),
        ("end_effector_frame", ctypes.c_char * 64),
        ("joint_names", (ctypes.c_char * 64) * _MAX_DOF),
        ("lower_limits", _Double * _MAX_DOF),
        ("upper_limits", _Double * _MAX_DOF),
    ]


class _CPose(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32), ("position", _Double * 3),
        ("rotation", _Double * 9), ("homogeneous", _Double * 16),
    ]


class _COptions(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("max_iterations", ctypes.c_uint32), ("max_retries", ctypes.c_uint32),
        ("tolerance", _Double), ("step_size", _Double), ("damping", _Double),
        ("random_seed", ctypes.c_uint64),
    ]


class _CResult(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32), ("success", ctypes.c_int32),
        ("iterations", ctypes.c_uint32), ("dof", ctypes.c_uint32),
        ("error_norm", _Double), ("q", _Double * _MAX_DOF),
    ]


def _decode(value: bytes) -> str:
    return value.split(b"\0", 1)[0].decode("utf-8")


class NativeRobotModel:
    """Private Runtime model through a stable Pinocchio-independent ABI."""

    def __init__(self, product_id: str = "yunyi_v1_0", side: RobotSide | int = RobotSide.LEFT):
        runtime_path = Path(articore_runtime_library_path())
        robotics = runtime_path.parent / "robotics"
        # Load the exact C++ numerical runtime globally before the ABI library.
        # This avoids accidental symbol interposition from unrelated system or
        # ROS Pinocchio/Boost copies already present in a Python process.
        dependencies = [robotics / "libstdc++.so.6"]
        dependencies.extend(sorted(robotics.glob("libboost_serialization.so.*")))
        dependencies.append(robotics / "libpinocchio_default.so")
        self._native_dependencies = tuple(
            ctypes.CDLL(str(path), mode=ctypes.RTLD_GLOBAL)
            for path in dependencies
            if path.is_file()
        )
        self._lib = ctypes.CDLL(str(runtime_path))
        self._bind()
        self._ptr = self._lib.articore_robot_model_create(
            product_id.encode("utf-8"), int(RobotSide(side))
        )
        if not self._ptr:
            raise RuntimeCallError(f"create robot model failed: {self._last_error()}")
        self._info = self._read_info()

    def _bind(self) -> None:
        lib = self._lib
        lib.articore_runtime_last_error.restype = ctypes.c_char_p
        lib.articore_robot_model_create.argtypes = [ctypes.c_char_p, ctypes.c_uint32]
        lib.articore_robot_model_create.restype = ctypes.c_void_p
        lib.articore_robot_model_free.argtypes = [ctypes.c_void_p]
        lib.articore_robot_model_get_info.argtypes = [ctypes.c_void_p, ctypes.POINTER(_CInfo)]
        lib.articore_robot_model_get_info.restype = ctypes.c_int32
        lib.articore_robot_model_fk.argtypes = [ctypes.c_void_p, _DoubleP, ctypes.c_uint32, ctypes.POINTER(_CPose)]
        lib.articore_robot_model_fk.restype = ctypes.c_int32
        for name in ("gravity", "mass_matrix"):
            fn = getattr(lib, f"articore_robot_model_{name}")
            fn.argtypes = [ctypes.c_void_p, _DoubleP, ctypes.c_uint32, _DoubleP, ctypes.c_uint32]
            fn.restype = ctypes.c_int32
        lib.articore_robot_model_jacobian.argtypes = [ctypes.c_void_p, _DoubleP, ctypes.c_uint32, ctypes.c_uint32, _DoubleP, ctypes.c_uint32]
        lib.articore_robot_model_jacobian.restype = ctypes.c_int32
        for name in ("coriolis_matrix", "nonlinear_effects"):
            fn = getattr(lib, f"articore_robot_model_{name}")
            fn.argtypes = [ctypes.c_void_p, _DoubleP, ctypes.c_uint32, _DoubleP, ctypes.c_uint32, _DoubleP, ctypes.c_uint32]
            fn.restype = ctypes.c_int32
        for name in ("rnea", "aba"):
            fn = getattr(lib, f"articore_robot_model_{name}")
            fn.argtypes = [ctypes.c_void_p, _DoubleP, ctypes.c_uint32, _DoubleP, ctypes.c_uint32, _DoubleP, ctypes.c_uint32, _DoubleP, ctypes.c_uint32]
            fn.restype = ctypes.c_int32
        lib.articore_robot_model_ik.argtypes = [ctypes.c_void_p, ctypes.POINTER(_CPose), _DoubleP, ctypes.c_uint32, ctypes.POINTER(_COptions), ctypes.POINTER(_CResult)]
        lib.articore_robot_model_ik.restype = ctypes.c_int32

    def _last_error(self) -> str:
        value = self._lib.articore_runtime_last_error()
        return value.decode("utf-8", errors="replace") if value else "unknown error"

    def _check(self, status: int, operation: str) -> None:
        if status != 0:
            raise RuntimeCallError(f"{operation} failed: {self._last_error()}")

    def _read_info(self) -> RobotModelInfo:
        raw = _CInfo(); raw.struct_size = ctypes.sizeof(raw)
        self._check(self._lib.articore_robot_model_get_info(self._ptr, ctypes.byref(raw)), "get model info")
        n = int(raw.dof)
        return RobotModelInfo(
            _decode(bytes(raw.product_id)), RobotSide(raw.side), n,
            tuple(_decode(bytes(raw.joint_names[i])) for i in range(n)),
            _decode(bytes(raw.end_effector_frame)),
            tuple(float(raw.lower_limits[i]) for i in range(n)),
            tuple(float(raw.upper_limits[i]) for i in range(n)),
        )

    @property
    def info(self) -> RobotModelInfo:
        return self._info

    def _vector(self, values: Sequence[float], name: str) -> ctypes.Array[_Double]:
        if len(values) != self.info.dof:
            raise ValueError(f"{name} must contain exactly {self.info.dof} values")
        converted = tuple(float(value) for value in values)
        if not all(isfinite(value) for value in converted):
            raise ValueError(f"{name} must contain only finite values")
        return (_Double * self.info.dof)(*converted)

    @staticmethod
    def _rows(values: ctypes.Array[_Double], rows: int, columns: int) -> tuple[tuple[float, ...], ...]:
        return tuple(tuple(float(values[r * columns + c]) for c in range(columns)) for r in range(rows))

    def fk(self, q: Sequence[float]) -> RobotPose:
        vector = self._vector(q, "q"); raw = _CPose(); raw.struct_size = ctypes.sizeof(raw)
        self._check(self._lib.articore_robot_model_fk(self._ptr, vector, self.info.dof, ctypes.byref(raw)), "FK")
        return RobotPose(tuple(float(x) for x in raw.position), self._rows(raw.rotation, 3, 3), self._rows(raw.homogeneous, 4, 4))

    def jacobian(self, q: Sequence[float], reference: JacobianReference | int = JacobianReference.LOCAL) -> tuple[tuple[float, ...], ...]:
        vector = self._vector(q, "q"); output = (_Double * (6 * self.info.dof))()
        self._check(self._lib.articore_robot_model_jacobian(self._ptr, vector, self.info.dof, int(JacobianReference(reference)), output, len(output)), "Jacobian")
        return self._rows(output, 6, self.info.dof)

    def _unary(self, operation: str, q: Sequence[float], size: int) -> tuple[float, ...]:
        vector = self._vector(q, "q"); output = (_Double * size)()
        self._check(getattr(self._lib, f"articore_robot_model_{operation}")(self._ptr, vector, self.info.dof, output, size), operation)
        return tuple(float(x) for x in output)

    def gravity(self, q: Sequence[float]) -> tuple[float, ...]: return self._unary("gravity", q, self.info.dof)
    def mass_matrix(self, q: Sequence[float]) -> tuple[tuple[float, ...], ...]:
        values = self._unary("mass_matrix", q, self.info.dof ** 2)
        return tuple(tuple(values[r * self.info.dof:(r + 1) * self.info.dof]) for r in range(self.info.dof))

    def _binary(self, operation: str, q: Sequence[float], dq: Sequence[float], size: int) -> tuple[float, ...]:
        qv = self._vector(q, "q"); dqv = self._vector(dq, "dq"); output = (_Double * size)()
        self._check(getattr(self._lib, f"articore_robot_model_{operation}")(self._ptr, qv, self.info.dof, dqv, self.info.dof, output, size), operation)
        return tuple(float(x) for x in output)

    def coriolis_matrix(self, q: Sequence[float], dq: Sequence[float]) -> tuple[tuple[float, ...], ...]:
        values = self._binary("coriolis_matrix", q, dq, self.info.dof ** 2)
        return tuple(tuple(values[r * self.info.dof:(r + 1) * self.info.dof]) for r in range(self.info.dof))
    def nonlinear_effects(self, q: Sequence[float], dq: Sequence[float]) -> tuple[float, ...]: return self._binary("nonlinear_effects", q, dq, self.info.dof)

    def _ternary(self, operation: str, q: Sequence[float], dq: Sequence[float], third: Sequence[float], third_name: str) -> tuple[float, ...]:
        qv=self._vector(q,"q"); dqv=self._vector(dq,"dq"); tv=self._vector(third,third_name); output=(_Double*self.info.dof)()
        self._check(getattr(self._lib,f"articore_robot_model_{operation}")(self._ptr,qv,self.info.dof,dqv,self.info.dof,tv,self.info.dof,output,self.info.dof),operation)
        return tuple(float(x) for x in output)

    def rnea(self, q: Sequence[float], dq: Sequence[float], ddq: Sequence[float]) -> tuple[float, ...]: return self._ternary("rnea",q,dq,ddq,"ddq")
    def aba(self, q: Sequence[float], dq: Sequence[float], torque: Sequence[float]) -> tuple[float, ...]: return self._ternary("aba",q,dq,torque,"torque")

    def ik(self, target: RobotPose, initial_q: Sequence[float], options: IkOptions = IkOptions()) -> IkResult:
        raw_pose=_CPose(); raw_pose.struct_size=ctypes.sizeof(raw_pose)
        raw_pose.position[:]=target.position; raw_pose.rotation[:]=sum(target.rotation,())
        q=self._vector(initial_q,"initial_q")
        raw_options=_COptions(ctypes.sizeof(_COptions),options.max_iterations,options.max_retries,options.tolerance,options.step_size,options.damping,options.random_seed)
        raw_result=_CResult(); raw_result.struct_size=ctypes.sizeof(raw_result)
        self._check(self._lib.articore_robot_model_ik(self._ptr,ctypes.byref(raw_pose),q,self.info.dof,ctypes.byref(raw_options),ctypes.byref(raw_result)),"IK")
        return IkResult(tuple(float(raw_result.q[i]) for i in range(raw_result.dof)),bool(raw_result.success),float(raw_result.error_norm),int(raw_result.iterations))

    def close(self) -> None:
        if getattr(self, "_ptr", None):
            self._lib.articore_robot_model_free(self._ptr); self._ptr = None

    def __enter__(self) -> NativeRobotModel: return self
    def __exit__(self, *_: object) -> None: self.close()
    def __del__(self) -> None:
        try: self.close()
        except Exception: pass

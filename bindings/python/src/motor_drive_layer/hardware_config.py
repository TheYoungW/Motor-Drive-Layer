from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping


class ConfigError(ValueError):
    pass


@dataclass(frozen=True)
class JointConfig:
    name: str
    motor_id: int
    feedback_id: int
    model: str


@dataclass(frozen=True)
class ControllerConfig:
    name: str
    port: str
    baud: int
    tx_gap_us: int
    joints: tuple[JointConfig, ...]


@dataclass(frozen=True)
class BenchmarkConfig:
    frequency_hz: int
    duration_s: float
    feedback_settle_ms: int


@dataclass(frozen=True)
class HardwareConfig:
    schema_version: int
    benchmark: BenchmarkConfig
    controllers: tuple[ControllerConfig, ...]


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ConfigError(f"{path} must be a mapping")
    return value


def _required(mapping: Mapping[str, Any], key: str, path: str) -> Any:
    if key not in mapping:
        raise ConfigError(f"{path}.{key} is required")
    return mapping[key]


def _string(value: Any, path: str, *, coerce_number: bool = False) -> str:
    if coerce_number and isinstance(value, (int, float)) and not isinstance(value, bool):
        value = str(value)
    if not isinstance(value, str) or not value.strip():
        raise ConfigError(f"{path} must be a non-empty string")
    return value.strip()


def _int(value: Any, path: str) -> int:
    if isinstance(value, bool):
        raise ConfigError(f"{path} must be an integer")
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        try:
            return int(value, 0)
        except ValueError as error:
            raise ConfigError(f"{path} must be an integer or hexadecimal string") from error
    raise ConfigError(f"{path} must be an integer or hexadecimal string")


def _positive_int(value: Any, path: str) -> int:
    parsed = _int(value, path)
    if parsed <= 0:
        raise ConfigError(f"{path} must be greater than zero")
    return parsed


def _non_negative_int(value: Any, path: str) -> int:
    parsed = _int(value, path)
    if parsed < 0:
        raise ConfigError(f"{path} must be zero or greater")
    return parsed


def _positive_float(value: Any, path: str) -> float:
    if isinstance(value, bool):
        raise ConfigError(f"{path} must be a number")
    try:
        parsed = float(value)
    except (TypeError, ValueError) as error:
        raise ConfigError(f"{path} must be a number") from error
    if not math.isfinite(parsed) or parsed <= 0:
        raise ConfigError(f"{path} must be a finite number greater than zero")
    return parsed


def _can_id(value: Any, path: str) -> int:
    parsed = _int(value, path)
    if parsed < 0 or parsed > 0x7FF:
        raise ConfigError(f"{path} must be a standard CAN ID in 0x000..0x7FF")
    return parsed


def load_hardware_config(path: str | Path) -> HardwareConfig:
    try:
        import yaml
    except ImportError as error:
        raise ConfigError(
            'YAML support is optional; install it with pip install "motor-drive-layer[hardware]"'
        ) from error

    config_path = Path(path)
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except OSError as error:
        raise ConfigError(f"cannot read config {config_path}: {error}") from error
    except yaml.YAMLError as error:
        raise ConfigError(f"invalid YAML in {config_path}: {error}") from error

    root = _mapping(raw, "config")
    schema_version = _int(_required(root, "schema_version", "config"), "schema_version")
    if schema_version != 1:
        raise ConfigError(f"schema_version must be 1, got {schema_version}")

    benchmark_raw = _mapping(_required(root, "benchmark", "config"), "benchmark")
    benchmark = BenchmarkConfig(
        frequency_hz=_positive_int(
            _required(benchmark_raw, "frequency_hz", "benchmark"), "benchmark.frequency_hz"
        ),
        duration_s=_positive_float(
            _required(benchmark_raw, "duration_s", "benchmark"), "benchmark.duration_s"
        ),
        feedback_settle_ms=_non_negative_int(
            benchmark_raw.get("feedback_settle_ms", 300), "benchmark.feedback_settle_ms"
        ),
    )

    controllers_raw = _required(root, "controllers", "config")
    if not isinstance(controllers_raw, list) or not controllers_raw:
        raise ConfigError("controllers must be a non-empty list")

    controllers: list[ControllerConfig] = []
    controller_names: set[str] = set()
    controller_ports: set[str] = set()
    for controller_index, item in enumerate(controllers_raw):
        path_prefix = f"controllers[{controller_index}]"
        controller_raw = _mapping(item, path_prefix)
        name = _string(_required(controller_raw, "name", path_prefix), f"{path_prefix}.name")
        port = _string(_required(controller_raw, "port", path_prefix), f"{path_prefix}.port")
        if name in controller_names:
            raise ConfigError(f"duplicate controller name: {name}")
        if port in controller_ports:
            raise ConfigError(f"duplicate controller port: {port}")
        controller_names.add(name)
        controller_ports.add(port)

        joints_raw = _required(controller_raw, "joints", path_prefix)
        if not isinstance(joints_raw, list) or not joints_raw:
            raise ConfigError(f"{path_prefix}.joints must be a non-empty list")
        joints: list[JointConfig] = []
        joint_names: set[str] = set()
        motor_ids: set[int] = set()
        feedback_ids: set[int] = set()
        for joint_index, joint_item in enumerate(joints_raw):
            joint_path = f"{path_prefix}.joints[{joint_index}]"
            joint_raw = _mapping(joint_item, joint_path)
            joint_name = _string(_required(joint_raw, "name", joint_path), f"{joint_path}.name")
            motor_id = _can_id(
                _required(joint_raw, "motor_id", joint_path), f"{joint_path}.motor_id"
            )
            feedback_id = _can_id(
                _required(joint_raw, "feedback_id", joint_path),
                f"{joint_path}.feedback_id",
            )
            model = _string(
                _required(joint_raw, "model", joint_path),
                f"{joint_path}.model",
                coerce_number=True,
            )
            if joint_name in joint_names:
                raise ConfigError(f"duplicate joint name in {name}: {joint_name}")
            if motor_id in motor_ids:
                raise ConfigError(f"duplicate motor_id in {name}: 0x{motor_id:X}")
            if feedback_id in feedback_ids:
                raise ConfigError(f"duplicate feedback_id in {name}: 0x{feedback_id:X}")
            joint_names.add(joint_name)
            motor_ids.add(motor_id)
            feedback_ids.add(feedback_id)
            joints.append(JointConfig(joint_name, motor_id, feedback_id, model))

        baud = _positive_int(
            _required(controller_raw, "baud", path_prefix), f"{path_prefix}.baud"
        )
        if baud > 0xFFFFFFFF:
            raise ConfigError(f"{path_prefix}.baud exceeds uint32 range")
        tx_gap_us = _non_negative_int(
            controller_raw.get("tx_gap_us", 120), f"{path_prefix}.tx_gap_us"
        )
        if tx_gap_us > 0xFFFFFFFF:
            raise ConfigError(f"{path_prefix}.tx_gap_us exceeds uint32 range")
        controllers.append(
            ControllerConfig(name, port, baud, tx_gap_us, tuple(joints))
        )

    return HardwareConfig(schema_version, benchmark, tuple(controllers))

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class JointConfig:
    name: str
    motor_id: int
    feedback_id: int
    model: str
    vendor: str


def _parse_scalar(raw: str) -> str:
    value = raw.strip()
    if value.startswith('"') and value.endswith('"'):
        return value[1:-1]
    if value.startswith("'") and value.endswith("'"):
        return value[1:-1]
    return value


def _parse_int(raw: str) -> int:
    return int(_parse_scalar(raw), 0)


def load_rebot_dm_config(path: str | Path) -> tuple[str, list[JointConfig]]:
    cfg_path = Path(path)
    channel = "/dev/ttyACM0"
    joints: list[JointConfig] = []
    current: dict[str, str] | None = None
    in_joints = False

    for line in cfg_path.read_text(encoding="utf-8").splitlines():
      stripped = line.strip()
      if not stripped or stripped.startswith("#"):
          continue
      if stripped.startswith("channel:"):
          channel = _parse_scalar(stripped.split(":", 1)[1])
          continue
      if stripped == "joints:":
          in_joints = True
          continue
      if not in_joints:
          continue
      if stripped.startswith("- name:"):
          if current is not None:
              joints.append(_joint_from_dict(current))
          current = {"name": _parse_scalar(stripped.split(":", 1)[1])}
          continue
      if current is not None and ":" in stripped and not stripped.endswith(":"):
          key, value = stripped.split(":", 1)
          current[key.strip()] = _parse_scalar(value)

    if current is not None:
        joints.append(_joint_from_dict(current))

    return channel, joints


def _joint_from_dict(raw: dict[str, str]) -> JointConfig:
    return JointConfig(
        name=raw["name"],
        motor_id=_parse_int(raw["motor_id"]),
        feedback_id=_parse_int(raw["feedback_id"]),
        model=raw.get("model", "4340P"),
        vendor=raw.get("vendor", "damiao").lower(),
    )

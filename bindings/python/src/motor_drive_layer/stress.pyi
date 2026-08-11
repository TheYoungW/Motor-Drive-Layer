from dataclasses import dataclass
from typing import Sequence

@dataclass(frozen=True)
class MotorSpec:
    channel: int
    motor_id: int
    feedback_id: int
    model: str

def parse_motor_spec(value: str) -> MotorSpec: ...
def run_dm_device_feedback_stress(
    motors: Sequence[MotorSpec],
    *,
    device: str = ...,
    bitrate: int = ...,
    data_bitrate: int = ...,
    iterations: int = ...,
    reconnect_cycles: int = ...,
    timeout_ms: int = ...,
) -> dict[str, object]: ...
def main(argv: Sequence[str] | None = ...) -> int: ...

from __future__ import annotations

from pathlib import Path

import pytest

from motorbridge.hardware_config import ConfigError, load_hardware_config


VALID_YAML = """
schema_version: 1
benchmark:
  frequency_hz: 500
  duration_s: 10
  feedback_settle_ms: 300
controllers:
  - name: arm_left
    port: /dev/ttyACM0
    baud: 1000000
    tx_gap_us: 120
    joints:
      - name: joint1
        motor_id: 0x01
        feedback_id: 0x201
        model: 4340P
      - name: joint2
        motor_id: "0x02"
        feedback_id: "0x202"
        model: "4310"
"""


def write_config(tmp_path: Path, content: str) -> Path:
    path = tmp_path / "hardware.yaml"
    path.write_text(content, encoding="utf-8")
    return path


def test_loads_editable_python_hardware_configuration(tmp_path: Path) -> None:
    config = load_hardware_config(write_config(tmp_path, VALID_YAML))

    assert config.benchmark.frequency_hz == 500
    assert config.controllers[0].port == "/dev/ttyACM0"
    assert config.controllers[0].baud == 1_000_000
    assert config.controllers[0].tx_gap_us == 120
    assert config.controllers[0].joints[0].feedback_id == 0x201
    assert config.controllers[0].joints[1].feedback_id == 0x202


def test_feedback_ids_are_user_editable_not_forced_to_example_values(tmp_path: Path) -> None:
    config = load_hardware_config(
        write_config(tmp_path, VALID_YAML.replace("0x201", "0x11").replace("0x202", "0x12"))
    )

    assert [joint.feedback_id for joint in config.controllers[0].joints] == [0x11, 0x12]


@pytest.mark.parametrize(
    ("content", "message"),
    [
        (VALID_YAML.replace("schema_version: 1", "schema_version: 2"), "schema_version"),
        (VALID_YAML.replace("port: /dev/ttyACM0", "port: ''"), "port"),
        (VALID_YAML.replace("motor_id: \"0x02\"", "motor_id: 0x01"), "motor_id"),
        (VALID_YAML.replace("feedback_id: \"0x202\"", "feedback_id: 0x201"), "feedback_id"),
        (VALID_YAML.replace("frequency_hz: 500", "frequency_hz: 0"), "frequency_hz"),
        (VALID_YAML.replace("duration_s: 10", "duration_s: 0"), "duration_s"),
        (VALID_YAML.replace("duration_s: 10", "duration_s: .nan"), "duration_s"),
        (VALID_YAML.replace("duration_s: 10", "duration_s: .inf"), "duration_s"),
        (VALID_YAML.replace("baud: 1000000", "baud: 4294967296"), "baud"),
        (VALID_YAML.replace("tx_gap_us: 120", "tx_gap_us: -1"), "tx_gap_us"),
    ],
)
def test_rejects_invalid_configuration(tmp_path: Path, content: str, message: str) -> None:
    with pytest.raises(ConfigError, match=message):
        load_hardware_config(write_config(tmp_path, content))


def test_rejects_duplicate_controller_ports(tmp_path: Path) -> None:
    second = VALID_YAML.split("controllers:\n", 1)[1].replace("arm_left", "arm_right")
    content = VALID_YAML + second

    with pytest.raises(ConfigError, match="port"):
        load_hardware_config(write_config(tmp_path, content))


def test_repository_dual_arm_example_is_valid_and_user_editable() -> None:
    example = (
        Path(__file__).resolve().parents[1]
        / "src"
        / "motorbridge"
        / "configs"
        / "damiao_dual_arm.yaml"
    )

    config = load_hardware_config(example)

    assert len(config.controllers) == 2
    assert all(len(controller.joints) == 7 for controller in config.controllers)
    assert config.controllers[0].joints[0].feedback_id == 0x201
    assert config.controllers[0].joints[-1].feedback_id == 0x207

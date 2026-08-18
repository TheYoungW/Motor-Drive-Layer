from motor_drive_layer import (
    ArticoreRuntime,
    Controller,
    GravityCompensationPhase,
    GravityProductBinding,
    Mode,
    MotorState,
    get_version,
)


def test_import_symbols() -> None:
    assert Controller is not None
    assert Mode.MIT.value == 1
    assert MotorState is not None
    assert ArticoreRuntime is not None
    assert GravityCompensationPhase.ACTIVE.value == 2
    assert GravityProductBinding(0, 0).product_id == "yunyi_v1_0"
    assert get_version() == "0.10.11"

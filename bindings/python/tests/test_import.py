from motor_drive_layer import Controller, Mode, MotorState, get_version


def test_import_symbols() -> None:
    assert Controller is not None
    assert Mode.MIT.value == 1
    assert MotorState is not None
    assert get_version() == "0.5.2"

from __future__ import annotations

from motor_drive_layer import DAMIAO_RW_REGISTERS
from motor_drive_layer.abi import damiao_register_info


def test_native_register_metadata_is_the_python_source_of_truth() -> None:
    assert damiao_register_info(11) == ("RO", "f32")
    assert damiao_register_info(9) == ("RW", "u32")
    assert damiao_register_info(37) is None
    assert damiao_register_info(256) is None

    for rid, spec in DAMIAO_RW_REGISTERS.items():
        assert damiao_register_info(rid) == (spec.access, spec.data_type)

from __future__ import annotations

from threading import Lock

import pytest

from motor_drive_layer import (
    ActiveCapability,
    ArticoreRuntime,
    CallError,
    ConnectErrorCode,
    Controller,
    ControllerGroup,
    GripperProductBinding,
    JointControlConfig,
    JointSafetyLimits,
    Motor,
    PresenceState,
    RuntimeCallError,
    RuntimeConfig,
    RuntimeMotor,
    RuntimeTransactionError,
    SafetyState,
)
from motor_drive_layer.abi import get_abi


def _fake_controller(pointer: int) -> Controller:
    controller = Controller.__new__(Controller)
    controller._abi = get_abi()
    controller._runtime_lease_lock = Lock()
    controller._runtime_lease_count = 0
    controller._feedback_motor_count = 0
    controller._ptr = pointer
    return controller


def _fake_group(controllers: tuple[Controller, ...], pointer: int) -> ControllerGroup:
    group = ControllerGroup.__new__(ControllerGroup)
    group._abi = get_abi()
    group._call_lock = Lock()
    group._runtime_lease_lock = Lock()
    group._runtime_lease_count = 0
    group._controllers = controllers
    group._ptr = pointer
    return group


def _abandon_fake_handles(
    group: ControllerGroup, controllers: tuple[Controller, ...], motors: tuple[Motor, ...]
) -> None:
    group._ptr = None
    for motor in motors:
        motor._ptr = None
    for controller in controllers:
        controller._ptr = None


def test_runtime_binding_owns_handles_and_converts_health() -> None:
    left = _fake_controller(0x101)
    group = _fake_group((left,), 0x201)
    joint = Motor(0x301, left)
    runtime = ArticoreRuntime(
        RuntimeConfig(), group, left, None,
        [RuntimeMotor(joint, side=0, name="l-joint1", safe_kp=2.0, safe_kd=0.5)],
    )
    try:
        with pytest.raises(CallError, match="active ArticoreRuntime"):
            joint.close()
        with pytest.raises(CallError, match="active ArticoreRuntime"):
            group.close()
        with pytest.raises(CallError, match="active ArticoreRuntime"):
            left.close()
        with pytest.raises(CallError, match="active ArticoreRuntime"):
            joint.send_mit(0.0, 0.0, 1.0, 0.1, 0.0)
        with pytest.raises(CallError, match="active ArticoreRuntime"):
            left.request_feedback_all()

        runtime.configure_joints([
            JointControlConfig(joint, -3.0, 3.0, 2.0, 20.0, 4.0, 0.5)
        ])
        runtime.configure_joint_safety_limits([
            JointSafetyLimits(joint, -3.1, 3.1, -3.0, 3.0, 0.1, 4.0)
        ])
        runtime.configure_gripper_products([])
        runtime.declare_motor_presence("l-tool", PresenceState.NOT_INSTALLED)
        assert runtime.motor_presence("l-tool") is PresenceState.NOT_INSTALLED
        # These ownership tests intentionally use sentinel native pointers,
        # not live Controller/Motor handles. Runtime ABI 2.4 connect performs
        # real feedback I/O, which is covered with the native fake driver and
        # hardware tests rather than dereferencing these sentinels.
        assert runtime.health.state is SafetyState.DISCONNECTED
        assert runtime.control_hz == 400
        assert runtime.active_capabilities == ActiveCapability.ARM_SIDE_0
        assert runtime.last_enable_report().expected_count == 0
        assert runtime.last_disable_report().expected_count == 0
        with pytest.raises(RuntimeTransactionError, match="control mode must be PV or MIT"):
            runtime.enable(99)  # type: ignore[arg-type]
    finally:
        runtime.close()
        assert runtime.closed
        assert joint._runtime_lease_count == 0
        assert group._runtime_lease_count == 0
        assert left._runtime_lease_count == 0
        _abandon_fake_handles(group, (left,), (joint,))


def test_runtime_binding_requires_native_gripper_profile_before_connect() -> None:
    left = _fake_controller(0x111)
    group = _fake_group((left,), 0x211)
    gripper = Motor(0x311, left)
    runtime = ArticoreRuntime(
        RuntimeConfig(), group, left, None,
        [RuntimeMotor(gripper, side=0, name="l-gripper", is_gripper=True)],
    )
    try:
        with pytest.raises(RuntimeTransactionError, match="profile is required") as failure:
            runtime.connect()
        assert failure.value.report.error_code is ConnectErrorCode.CONFIGURATION
        assert failure.value.report.failure_count == 1
        assert failure.value.report.motors[0].name == "l-gripper"
        with pytest.raises(RuntimeCallError, match="unknown built-in"):
            runtime.configure_gripper_products([
                GripperProductBinding(gripper, "unknown_product")
            ])
        runtime.configure_gripper_products([
            GripperProductBinding(gripper, "yunyi_gripper_v1")
        ])
        health = runtime.health
        assert health.state is SafetyState.DISCONNECTED
        assert len(health.grippers) == 1
        assert health.grippers[0].name == "l-gripper"
        assert runtime.active_capabilities == ActiveCapability.GRIPPER_SIDE_0
    finally:
        runtime.close()
        _abandon_fake_handles(group, (left,), (gripper,))

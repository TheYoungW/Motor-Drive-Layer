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
    GravityCompensationPhase,
    JointControlConfig,
    JointSafetyLimits,
    Motor,
    PresenceState,
    RuntimeCallError,
    RuntimeConfig,
    RuntimeMotor,
    RuntimeTransactionError,
    SafetyState,
    TransportCapabilities,
)
from motor_drive_layer.abi import get_abi


def _fake_controller(
    pointer: int,
    *,
    transport: str = "fake-test",
    can_fd: bool = False,
    can_fd_brs: bool = False,
) -> Controller:
    controller = Controller.__new__(Controller)
    controller._abi = get_abi()
    controller._runtime_lease_lock = Lock()
    controller._runtime_lease_count = 0
    controller._feedback_motor_count = 0
    controller._ptr = pointer
    controller.transport_capabilities = lambda: TransportCapabilities(
        transport=transport,
        max_payload_bytes=8,
        channel_count=1,
        can_fd=can_fd,
        parallel_batches=True,
        hardware_rx_timestamps=False,
        reconnect=True,
        process_session_reuse=False,
        can_fd_brs=can_fd_brs,
    )
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


def test_runtime_config_uses_50_ms_feedback_freshness_window() -> None:
    config = RuntimeConfig()

    assert config.feedback_check_hz == 100
    assert config.feedback_max_age_ms == 50
    assert config.feedback_failure_threshold == 3


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
        torque_stats = runtime.mit_torque_limit_stats
        assert torque_stats.torque_limit_activation_count == 0
        assert torque_stats.torque_limited_joint_mask == 0
        assert torque_stats.joints == ()
        gravity_status = runtime.gravity_compensation_status
        assert gravity_status.phase is GravityCompensationPhase.INACTIVE
        assert gravity_status.active is False
        assert gravity_status.joints == ()
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


def test_failed_close_keeps_runtime_and_native_resource_leases(monkeypatch) -> None:
    left = _fake_controller(0x141)
    group = _fake_group((left,), 0x241)
    joint = Motor(0x341, left)
    runtime = ArticoreRuntime(
        RuntimeConfig(), group, left, None,
        [RuntimeMotor(joint, side=0, name="l-joint1")],
    )
    native_pointer = runtime._ptr
    with monkeypatch.context() as patch:
        patch.setattr(
            runtime._runtime_abi.lib,
            "articore_runtime_close",
            lambda _runtime: -1,
        )
        with pytest.raises(RuntimeTransactionError, match="close failed"):
            runtime.close()

    assert not runtime.closed
    assert runtime._ptr == native_pointer
    assert joint._runtime_lease_count == 1
    assert group._runtime_lease_count == 1
    assert left._runtime_lease_count == 1
    with pytest.raises(CallError, match="active ArticoreRuntime"):
        group.close()
    with pytest.raises(CallError, match="active ArticoreRuntime"):
        left.close()

    runtime.close()
    assert runtime.closed
    assert joint._runtime_lease_count == 0
    assert group._runtime_lease_count == 0
    assert left._runtime_lease_count == 0
    _abandon_fake_handles(group, (left,), (joint,))


def test_dual_runtime_500_hz_requires_socketcanfd_brs_on_both_sides() -> None:
    left = _fake_controller(
        0x121, transport="socketcanfd", can_fd=True, can_fd_brs=True
    )
    right = _fake_controller(
        0x122, transport="socketcanfd", can_fd=True, can_fd_brs=True
    )
    group = _fake_group((left, right), 0x221)
    motors = (Motor(0x321, left), Motor(0x322, right))
    runtime = ArticoreRuntime(
        RuntimeConfig(control_hz=500), group, left, right,
        [
            RuntimeMotor(motors[0], side=0, name="l-joint1"),
            RuntimeMotor(motors[1], side=1, name="r-joint1"),
        ],
    )
    try:
        assert runtime.control_hz == 500
    finally:
        runtime.close()
        _abandon_fake_handles(group, (left, right), motors)

    left = _fake_controller(
        0x131, transport="dm-device", can_fd=True, can_fd_brs=True
    )
    right = _fake_controller(
        0x132, transport="dm-device", can_fd=True, can_fd_brs=True
    )
    group = _fake_group((left, right), 0x231)
    motors = (Motor(0x331, left), Motor(0x332, right))
    runtime = ArticoreRuntime(
        RuntimeConfig(control_hz=500), group, left, right,
        [
            RuntimeMotor(motors[0], side=0, name="l-joint1"),
            RuntimeMotor(motors[1], side=1, name="r-joint1"),
        ],
    )
    try:
        assert runtime.control_hz == 400
    finally:
        runtime.close()
        _abandon_fake_handles(group, (left, right), motors)


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

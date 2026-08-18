from __future__ import annotations

import math

import pytest

import motor_drive_layer as mdl


@pytest.mark.parametrize(
    ("side", "prefix", "zero_position"),
    [
        (mdl.RobotSide.LEFT, "l", (0.001704439680308022, 0.2318916953069385, 0.18147106361002283)),
        (mdl.RobotSide.RIGHT, "r", (0.0017044146523629705, -0.23188869792476557, 0.18147150461594638)),
    ],
)
def test_native_robot_model_surface(side, prefix, zero_position):
    with mdl.NativeRobotModel(side=side) as model:
        assert model.info.dof == 7
        assert model.info.joint_names == tuple(f"{prefix}-joint{i}" for i in range(1, 8))
        pose = model.fk([0.0] * 7)
        assert pose.position == pytest.approx(zero_position, abs=1e-14)
        assert len(model.jacobian([0.0] * 7)) == 6
        assert len(model.mass_matrix([0.0] * 7)) == 7
        gravity = model.gravity([0.0] * 7)
        assert model.rnea([0.0] * 7, [0.0] * 7, [0.0] * 7) == pytest.approx(gravity)
        assert max(abs(value) for value in model.aba([0.0] * 7, [0.0] * 7, gravity)) < 1e-10
        result = model.ik(pose, [0.0] * 7)
        assert result.success
        assert result.error == 0.0
        assert all(math.isfinite(value) for value in result.q)


def test_native_robot_model_validates_shape():
    with mdl.NativeRobotModel() as model:
        with pytest.raises(ValueError, match="exactly 7"):
            model.fk([0.0] * 6)


def test_native_robot_model_rejects_invalid_ik_rotation():
    with mdl.NativeRobotModel() as model:
        pose = mdl.RobotPose(
            position=(0.0, 0.0, 0.0),
            rotation=((1.0, 0.0, 0.0), (0.0, 2.0, 0.0), (0.0, 0.0, 1.0)),
            homogeneous=((1.0, 0.0, 0.0, 0.0),) * 4,
        )
        with pytest.raises(mdl.RuntimeCallError, match="proper rotation"):
            model.ik(pose, [0.0] * 7)

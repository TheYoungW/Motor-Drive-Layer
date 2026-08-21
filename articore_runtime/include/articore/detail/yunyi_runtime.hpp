#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <vector>

#include "articore/runtime_abi.h"
#include "motor_abi.h"
#include "articore/detail/robot_model.hpp"
#include "articore/detail/runtime.hpp"

namespace articore {

inline constexpr float kYunyiOrdinaryMaximumVelocity = 5.0f;
inline constexpr float kYunyiDefaultSpeedPercent = 70.0f;

// Complete native ownership for the only supported robot product. Nothing in
// this structure crosses the public ABI or needs to be assembled by Python.
struct YunyiRuntimeResources {
  struct Joint {
    MotorHandle* motor = nullptr;
    float direction = 1.0f;
    float lower = 0.0f;
    float upper = 0.0f;
    float velocity_limit = 0.0f;
    float acceleration_limit = 0.0f;
    float torque_limit = 0.0f;
    float velocity_command_scale = 1.0f;
    float velocity_feedback_scale = 1.0f;
    float torque_command_scale = 1.0f;
    float torque_feedback_scale = 1.0f;
    float kp = 0.0f;
    float kd = 0.0f;
  };

  YunyiRuntimeResources() = default;
  ~YunyiRuntimeResources();

  YunyiRuntimeResources(const YunyiRuntimeResources&) = delete;
  YunyiRuntimeResources& operator=(const YunyiRuntimeResources&) = delete;

  MotorController* controllers[2]{};
  MotorControllerGroup* group = nullptr;
  std::vector<MotorHandle*> motors;
  std::array<MotorHandle*, ARTICORE_PRODUCT_DUAL_ARM_DOF> arm_motors{};
  std::array<Joint, ARTICORE_PRODUCT_DUAL_ARM_DOF> joints{};
  std::array<MotorHandle*, 2> grippers{};
  std::array<std::unique_ptr<RobotModel>, 2> pose_models;
  std::array<std::mutex, 2> pose_mutexes;
  bool with_grippers = true;
  float default_mit_reference_velocity = kYunyiOrdinaryMaximumVelocity;
  float default_pv_reference_velocity = kYunyiOrdinaryMaximumVelocity;
};

struct YunyiRuntimeBundle {
  std::unique_ptr<SafetyRuntime> runtime;
  std::unique_ptr<YunyiRuntimeResources> resources;
  ArticoreControlMode mode = ARTICORE_MODE_PV;
};

// Builds the fixed Yunyi dual-arm product: two SocketCAN-FD+BRS channels,
// fourteen arm joints, optional paired grippers, all calibration, safety and
// model bindings, and complete native ownership.
YunyiRuntimeBundle create_yunyi_runtime(
    ArticoreControlMode mode, bool with_grippers);

}  // namespace articore

#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "articore/runtime_abi.h"
#include "articore/detail/robot_model.hpp"
#include "articore/detail/runtime.hpp"
#include "damiao/runtime.hpp"

namespace articore {

inline constexpr float kYunyiOrdinaryMaximumVelocity = 5.0f;
inline constexpr float kYunyiDefaultSpeedPercent = 70.0f;

// Complete native ownership for the only supported robot product. Nothing in
// this structure crosses the public ABI or needs to be assembled by Python.
struct YunyiRuntimeResources {
  struct Joint {
    damiao::MotorHandle* motor = nullptr;
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

  YunyiRuntimeResources(const YunyiRuntimeResources&) = delete;
  YunyiRuntimeResources& operator=(const YunyiRuntimeResources&) = delete;

  std::array<std::unique_ptr<damiao::Controller>, 2> controllers;
  std::unique_ptr<damiao::ControllerGroup> group;
  std::unordered_map<damiao::MotorHandle*, std::shared_ptr<damiao::MotorHandle>>
      motor_owners;
  std::array<damiao::MotorHandle*, ARTICORE_PRODUCT_DUAL_ARM_DOF> arm_motors{};
  std::array<Joint, ARTICORE_PRODUCT_DUAL_ARM_DOF> joints{};
  std::array<damiao::MotorHandle*, 2> grippers{};
  std::array<std::unique_ptr<RobotModel>, 2> pose_models;
  std::array<std::mutex, 2> pose_mutexes;
  bool with_grippers = true;
  float default_mit_reference_velocity = kYunyiOrdinaryMaximumVelocity;
  float default_pv_reference_velocity = kYunyiOrdinaryMaximumVelocity;
};

// Reads the Motor core cache directly. These helpers keep all conversion from
// native C++ state to the public product ABI in the product layer; there is no
// intermediate Motor C ABI or second shared library.
bool read_yunyi_motor_state(
    damiao::MotorHandle* motor, ArticoreMotorState& state,
    ArticoreFeedbackStats& stats) noexcept;

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

#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "articore/runtime_abi.h"
#include "articore/detail/robot_model.hpp"
#include "articore/detail/runtime.hpp"
#include "articore/detail/yunyi_product.hpp"
#include "damiao/runtime.hpp"

namespace articore {

// Ordinary PV uses the reference-slew range selected by real-hardware tracking
// and settling tests. The Damiao POS_VEL V field is a separate drive ceiling;
// never scale it down with the public percentage.
inline constexpr float kYunyiOrdinaryPvMaximumVelocity = 2.0f;
inline constexpr float kYunyiPvDriveVelocityLimit = 3.0f;
// The public 0..100 scale maps linearly onto 0..2 rad/s. Product calls keep
// 50 percent as the ordinary default, or 1 rad/s.
inline constexpr float kYunyiDefaultPvSpeedPercent = 50.0f;
inline constexpr float kYunyiOrdinaryMitMaximumVelocity = 5.0f;
inline constexpr float yunyi_effective_pv_speed_percent(
    float command_speed_percent, float maximum_speed_percent) {
  return command_speed_percent < maximum_speed_percent
      ? command_speed_percent : maximum_speed_percent;
}
static_assert(kYunyiOrdinaryPvMaximumVelocity == 2.0f,
              "ordinary PV 100 percent must map to 2 rad/s");
static_assert(kYunyiPvDriveVelocityLimit == 3.0f,
              "ordinary PV must retain the independent 3 rad/s drive ceiling");
inline constexpr float kYunyiDefaultPvReferenceVelocity =
    kYunyiOrdinaryPvMaximumVelocity *
    kYunyiDefaultPvSpeedPercent / 100.0f;
static_assert(kYunyiDefaultPvReferenceVelocity > 0.99999f &&
                  kYunyiDefaultPvReferenceVelocity < 1.00001f,
              "ordinary PV must default to a 1 rad/s reference velocity");
static_assert(kYunyiOrdinaryMitMaximumVelocity == 5.0f,
              "ordinary MIT 100 percent must map to 5 rad/s");
static_assert(yunyi_effective_pv_speed_percent(30.0f, 50.0f) == 30.0f &&
                  yunyi_effective_pv_speed_percent(80.0f, 50.0f) == 50.0f,
              "PV command speed must be bounded by the configured maximum");

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
  // Active flange(link7)-to-TCP transforms [x,y,z,roll,pitch,yaw]. They are
  // native session configuration shared by pose reporting and every IK path.
  std::array<std::array<float, ARTICORE_PRODUCT_POSE_DOF>, 2> tcp_offsets{};
  bool with_grippers = true;
  float default_mit_reference_velocity =
      kYunyiOrdinaryMitMaximumVelocity;
  float default_pv_reference_velocity =
      kYunyiOrdinaryPvMaximumVelocity;
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

std::array<float, ARTICORE_PRODUCT_POSE_DOF> default_yunyi_tcp_offset(
    bool with_grippers);

}  // namespace articore

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

// Complete trajectories keep their existing internal PV limits. Ordinary
// product PV owns a separate per-joint online profile below; do not reuse one
// set of limits for the other execution path.
// Historical name retained because this scalar is already used by the
// complete-trajectory planner. Ordinary product PV does not use it.
inline constexpr float kYunyiOrdinaryPvMaximumVelocity = 2.0f;
inline constexpr float kYunyiPvDriveVelocityLimit = 3.0f;
inline constexpr float kYunyiTrajectoryPvAccelerationLimit = 6.0f;
inline constexpr float kYunyiMitFastFollowMaximumVelocity = 5.0f;
inline constexpr std::array<float, ARTICORE_PRODUCT_ARM_DOF>
    kYunyiMitDirectKp = {15, 15, 12, 12, 8, 7, 6};
inline constexpr std::array<float, ARTICORE_PRODUCT_ARM_DOF>
    kYunyiMitDirectKd = {.8f, .8f, .7f, .7f, .5f, .5f, .4f};
inline constexpr std::array<float, ARTICORE_PRODUCT_ARM_DOF>
    kYunyiMitFastFollowKp = {190, 190, 100, 100, 70, 60, 50};
inline constexpr std::array<float, ARTICORE_PRODUCT_ARM_DOF>
    kYunyiMitFastFollowKd = {
        4.55f, 4.5f, 2.5f, 2.5f, .7f, .6f, .5f};
inline constexpr float yunyi_effective_pv_reference_velocity(
    float command_speed_percent) {
  return kYunyiOrdinaryPvMaximumVelocity * command_speed_percent / 100.0f;
}
static_assert(kYunyiOrdinaryPvMaximumVelocity == 2.0f,
              "trajectory PV reference policy must remain 2 rad/s");
static_assert(kYunyiPvDriveVelocityLimit == 3.0f,
              "PV must retain the independent 3 rad/s Motor V ceiling");
static_assert(kYunyiTrajectoryPvAccelerationLimit ==
                  kNativeRealtimePvTrajectoryAccelerationLimit,
              "trajectory acceleration base must remain Runtime-owned");
static_assert(kYunyiMitFastFollowMaximumVelocity == 5.0f,
              "fast-follow MIT must use the fixed 5 rad/s step limit");
static_assert(yunyi_effective_pv_reference_velocity(50.0f) == 1.0f &&
                  yunyi_effective_pv_reference_velocity(100.0f) == 2.0f,
              "trajectory PV reference scaling must remain unchanged");

inline constexpr float kRadiansPerDegree =
    3.14159265358979323846f / 180.0f;

// Ordinary public PV hard limits, in logical J1..J7 order. These limits apply
// independently to both arms and never alter complete trajectory planning.
inline constexpr std::array<float, ARTICORE_PRODUCT_ARM_DOF>
    kYunyiOrdinaryPvJointMaximumVelocities = {
        180.0f * kRadiansPerDegree,
        180.0f * kRadiansPerDegree,
        180.0f * kRadiansPerDegree,
        225.0f * kRadiansPerDegree,
        225.0f * kRadiansPerDegree,
        225.0f * kRadiansPerDegree,
        225.0f * kRadiansPerDegree,
    };
inline constexpr std::array<float, ARTICORE_PRODUCT_ARM_DOF>
    kYunyiOrdinaryPvJointMaximumAccelerations = {
        450.0f * kRadiansPerDegree,
        450.0f * kRadiansPerDegree,
        900.0f * kRadiansPerDegree,
        900.0f * kRadiansPerDegree,
        900.0f * kRadiansPerDegree,
        900.0f * kRadiansPerDegree,
        900.0f * kRadiansPerDegree,
    };

// A scalar user override applies to every arm joint, so its accepted range is
// bounded by the most restrictive J1..J7 product limit. A zero configured
// value means that no user override is installed and the per-joint defaults
// above remain authoritative.
inline constexpr float kYunyiPvMotionLimitResolution = 0.01f;
inline constexpr float kYunyiOrdinaryPvDefaultLimitSelection = 0.0f;
inline constexpr float kYunyiOrdinaryPvMaximumConfigurableVelocity =
    kYunyiOrdinaryPvJointMaximumVelocities[0];
inline constexpr float kYunyiOrdinaryPvMaximumConfigurableAcceleration =
    kYunyiOrdinaryPvJointMaximumAccelerations[0];

inline constexpr float yunyi_ordinary_pv_velocity_limit(
    uint32_t joint_index, float speed_percent,
    float configured_maximum_velocity =
        kYunyiOrdinaryPvDefaultLimitSelection) {
  const float default_limit = kYunyiOrdinaryPvJointMaximumVelocities[
      joint_index % ARTICORE_PRODUCT_ARM_DOF];
  const float base_limit = configured_maximum_velocity > 0.0f
      ? configured_maximum_velocity : default_limit;
  return base_limit * speed_percent / 100.0f;
}

inline constexpr float yunyi_ordinary_pv_acceleration_limit(
    uint32_t joint_index, float speed_percent,
    float configured_maximum_acceleration =
        kYunyiOrdinaryPvDefaultLimitSelection) {
  const float time_scale = speed_percent / 100.0f;
  const float default_limit = kYunyiOrdinaryPvJointMaximumAccelerations[
      joint_index % ARTICORE_PRODUCT_ARM_DOF];
  const float base_limit = configured_maximum_acceleration > 0.0f
      ? configured_maximum_acceleration : default_limit;
  return base_limit * time_scale * time_scale;
}

static_assert(kYunyiOrdinaryPvJointMaximumVelocities[0] > 3.14159f &&
                  kYunyiOrdinaryPvJointMaximumVelocities[0] < 3.14160f &&
                  kYunyiOrdinaryPvJointMaximumVelocities[3] > 3.9269f &&
                  kYunyiOrdinaryPvJointMaximumVelocities[3] < 3.9271f,
              "ordinary PV joint velocity limits must be rad/s conversions");
static_assert(kYunyiOrdinaryPvJointMaximumAccelerations[0] > 7.85398f &&
                  kYunyiOrdinaryPvJointMaximumAccelerations[0] < 7.85399f &&
                  kYunyiOrdinaryPvJointMaximumAccelerations[2] > 15.70796f &&
                  kYunyiOrdinaryPvJointMaximumAccelerations[2] < 15.70797f,
              "ordinary PV joint acceleration limits must be rad/s^2 conversions");

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
  float mit_fast_follow_reference_velocity =
      kYunyiMitFastFollowMaximumVelocity;
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

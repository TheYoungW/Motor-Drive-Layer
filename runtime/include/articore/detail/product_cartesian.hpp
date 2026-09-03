#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "articore/detail/runtime_bridge.hpp"
#include "articore/detail/runtime.hpp"

namespace articore {

// Linear/Circular trajectory points use a 1 rad/s reference ceiling and
// 6 rad/s^2 acceleration ceiling at the shared 100-percent speed setting.
// Product inverse kinematics is a pure computation. Callers explicitly submit
// solved joint endpoints through the ordinary PV or MIT surface when desired.
// The Damiao POS_VEL V field remains a separate PV drive-level limit.
inline constexpr float kYunyiCartesianMaximumVelocity = 1.0f;
inline constexpr float kYunyiCartesianTrajectoryPvDriveVelocityLimit = 3.0f;
inline constexpr float kYunyiCartesianWristAccelerationLimit = 6.0f;
// Linear preserves its Cartesian geometry with at most 2 mm translation or
// 0.1 rad orientation between consecutive IK targets. The resulting joint
// plan uses adaptive trajectory-PV knots, linearly resampled at the Runtime's
// 500 Hz command cadence.
inline constexpr double kYunyiLinearTranslationSampleDistance = 0.002;
inline constexpr double kYunyiLinearOrientationSampleDistance = 0.1;
// Circular uses the same geometric density and adaptive trajectory-PV knot
// policy as Linear. Its positional path is the unique directed arc through
// start/via/end; orientation follows shortest-path SLERP through the via
// orientation.
inline constexpr double kYunyiCircularTranslationSampleDistance = 0.002;
inline constexpr double kYunyiCircularOrientationSampleDistance = 0.1;
inline constexpr double kYunyiCartesianMinimumKnotIntervalSeconds = 0.004;
inline constexpr double kYunyiCartesianMaximumKnotIntervalSeconds = 0.050;
inline constexpr double kYunyiCartesianMaximumKnotJointStep = 0.020;
inline constexpr double kYunyiCartesianKnotLinearizationTolerance = 0.001;
// Cartesian geometry samples are close, so a large per-sample joint move still
// indicates a branch jump. One required reversal is allowed, but repeated small
// reversals inside a short excursion are rejected as visible IK chatter.
inline constexpr double kYunyiCartesianMaximumIkJointStep = 0.35;
inline constexpr double kYunyiCartesianIkPositionTolerance = 0.0005;
inline constexpr double kYunyiCartesianIkOrientationTolerance = 0.035;
inline constexpr double kYunyiCartesianIkOrientationWeight = 0.20;
// Below 1 mrad a direction sign is treated as numerical/local-extremum noise;
// it must not create a new joint-motion trend for chatter detection.
inline constexpr double kYunyiCartesianIkDirectionDeadband = 0.001;
inline constexpr double kYunyiCartesianIkChatterExcursion = 0.010;
// Keep the numerical solve bounded so validation, locking and atomic command
// installation remain responsive.
inline constexpr std::chrono::microseconds kYunyiProductIkBudget{8000};

inline float product_cartesian_reference_velocity_limit(
    float joint_hard_velocity_limit, float speed_scale) {
  return std::min(
      joint_hard_velocity_limit, kYunyiCartesianMaximumVelocity) * speed_scale;
}

inline float product_cartesian_trajectory_pv_drive_velocity_limit(
    float joint_hard_velocity_limit) {
  return std::min(
      joint_hard_velocity_limit,
      kYunyiCartesianTrajectoryPvDriveVelocityLimit);
}

inline float product_cartesian_acceleration_limit(
    uint32_t product_joint_index, float joint_hard_acceleration_limit) {
  const uint32_t arm_joint_index =
      product_joint_index % ARTICORE_PRODUCT_ARM_DOF;
  return arm_joint_index >= 4
      ? std::min(joint_hard_acceleration_limit,
                 kYunyiCartesianWristAccelerationLimit)
      : joint_hard_acceleration_limit;
}

struct YunyiRuntimeResources;

struct NativeCartesianTrajectoryPlan {
  NativeTrajectoryRequest trajectory;
  double minimum_duration_s = 0.0;
};

struct CartesianIkContinuityState {
  std::array<int8_t, ARTICORE_PRODUCT_ARM_DOF> directions{};
  std::array<double, ARTICORE_PRODUCT_ARM_DOF> excursion_since_reversal{};
  std::array<bool, ARTICORE_PRODUCT_ARM_DOF> has_reversed{};
};

void require_cartesian_ik_continuity(
    uint32_t side,
    const std::array<double, ARTICORE_PRODUCT_ARM_DOF>& previous,
    const std::array<double, ARTICORE_PRODUCT_ARM_DOF>& current,
    CartesianIkContinuityState& state,
    const char* context,
    std::size_t sample_index);

inline void require_unchanged_planned_reference(
    const NativeTrajectorySample& before,
    const NativeTrajectorySample& after,
    const char* motion_name) {
  bool same = before.active == after.active &&
      before.motion_id == after.motion_id &&
      before.operation == after.operation &&
      before.positions.size() == after.positions.size();
  if (same) {
    for (std::size_t index = 0; index < before.positions.size(); ++index) {
      if (std::abs(before.positions[index] - after.positions[index]) > 1e-4f) {
        same = false;
        break;
      }
    }
  }
  if (!same) {
    throw std::invalid_argument(
        std::string(motion_name) +
        " queue tail changed while planning; no motion was queued");
  }
}

// Continuous Cartesian path samples solve only from the current/previous
// planned configuration. Random retries are disabled by the seed-only model
// entry point; these options provide only its numerical solver settings.
inline ArticoreIkOptions product_path_ik_options() {
  ArticoreIkOptions options{};
  options.struct_size = sizeof(options);
  options.max_iterations = 1000;
  options.random_seed = 0;
  return options;
}

// A discrete IK endpoint has no continuous path branch to preserve, so it
// retains the deterministic global search and selects the solution nearest to
// the current/planned seed.
inline ArticoreIkOptions product_endpoint_ik_options() {
  auto options = product_path_ik_options();
  options.max_retries = 1000;
  options.random_seed = 0;
  return options;
}

std::vector<NativeTrajectoryJoint> product_cartesian_joints(
    const YunyiRuntimeResources& product);

std::vector<std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>>
solve_cartesian_trajectory_approach_targets(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* target_pose,
    const char* motion_name);

std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>
solve_dual_point_to_point_targets_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    const NativeTrajectorySample& reference,
    const float* left_target_pose,
    const float* right_target_pose,
    std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::time_point::max());

NativeCartesianTrajectoryPlan build_linear_trajectory_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* end_pose,
    float pv_reference_acceleration =
        kNativeTrajectoryPvAccelerationLimit,
    float pv_reference_velocity = kYunyiCartesianMaximumVelocity);

NativeCartesianTrajectoryPlan build_linear_path_trajectory_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* poses,
    uint32_t pose_count,
    float pv_reference_acceleration =
        kNativeTrajectoryPvAccelerationLimit,
    float pv_reference_velocity = kYunyiCartesianMaximumVelocity);

NativeCartesianTrajectoryPlan build_linear_trajectory_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* start_pose,
    const float* end_pose,
    float pv_reference_acceleration =
        kNativeTrajectoryPvAccelerationLimit,
    float pv_reference_velocity = kYunyiCartesianMaximumVelocity);

void validate_cartesian_start_pose(
    bool with_grippers,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const ArticoreRobotPose& start_pose,
    const char* motion_name);

void validate_cartesian_start_pose(
    const std::array<float, ARTICORE_PRODUCT_POSE_DOF>& tcp_offset,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const ArticoreRobotPose& start_pose,
    const char* motion_name);

void validate_cartesian_start_pose(
    bool with_grippers,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* start_pose,
    const char* motion_name);

NativeCartesianTrajectoryPlan build_circular_trajectory_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* via_pose,
    const float* end_pose,
    float pv_reference_acceleration =
        kNativeTrajectoryPvAccelerationLimit,
    float pv_reference_velocity = kYunyiCartesianMaximumVelocity);

NativeCartesianTrajectoryPlan build_circular_trajectory_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* start_pose,
    const float* via_pose,
    const float* end_pose,
    float pv_reference_acceleration =
        kNativeTrajectoryPvAccelerationLimit,
    float pv_reference_velocity = kYunyiCartesianMaximumVelocity);

}  // namespace articore

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "articore/runtime_abi.h"
#include "articore/detail/runtime.hpp"

namespace articore {

// PV Joint/Linear/Circular trajectory points use an internal speed-50 policy,
// i.e. a 1 rad/s reference ceiling. Product inverse kinematics is a pure
// computation; the compatibility set_pose entry resolves IK and then submits
// the caller's endpoint through the current ordinary PV or MIT mode. The
// Damiao POS_VEL V field remains a separate PV drive-level limit.
inline constexpr float kYunyiCartesianMaximumVelocity = 1.0f;
inline constexpr float kYunyiCartesianPvDriveVelocityLimit = 3.0f;
inline constexpr float kYunyiCartesianWristAccelerationLimit = 6.0f;
inline constexpr double kYunyiCartesianDefaultBlendRadius = 0.010;
// Linear preserves its Cartesian geometry with at most 2 mm translation or
// 0.1 rad orientation between consecutive IK targets. The resulting joint
// references are sent through internal real-time PV at 100 Hz, once per point.
inline constexpr double kYunyiLinearTranslationSampleDistance = 0.002;
inline constexpr double kYunyiLinearOrientationSampleDistance = 0.1;
inline constexpr uint32_t kYunyiLinearReferenceHz = 100;
// Circular uses the same geometric density and real-time PV command clock as
// Linear. Its positional path is the unique directed arc through start/via/end;
// orientation follows shortest-path SLERP through the via orientation.
inline constexpr double kYunyiCircularTranslationSampleDistance = 0.002;
inline constexpr double kYunyiCircularOrientationSampleDistance = 0.1;
inline constexpr uint32_t kYunyiCircularReferenceHz = 100;
// A 100 Hz caller has a 10 ms period. Keep two milliseconds outside the
// numerical solve for ABI validation, locking and atomic command install.
inline constexpr std::chrono::microseconds kYunyiProductIkBudget{8000};

inline float product_cartesian_reference_velocity_limit(
    float joint_hard_velocity_limit, float speed_scale) {
  return std::min(
      joint_hard_velocity_limit, kYunyiCartesianMaximumVelocity) * speed_scale;
}

inline float product_pv_drive_velocity_limit(
    float joint_hard_velocity_limit) {
  return std::min(
      joint_hard_velocity_limit, kYunyiCartesianPvDriveVelocityLimit);
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

struct NativeCartesianPlan {
  NativeTrajectoryRequest trajectory;
  double minimum_duration_s = 0.0;
};

enum class CartesianIkSearch {
  LocalPath,
  GlobalEndpoint,
};

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

// Product Cartesian planning uses a small local search for sequential path
// samples and a deterministic, fixed-seed global search for user endpoints.
inline ArticoreIkOptions product_cartesian_ik_options(
    CartesianIkSearch search) {
  ArticoreIkOptions options{};
  options.struct_size = sizeof(options);
  options.max_iterations = 1000;
  options.max_retries = search == CartesianIkSearch::GlobalEndpoint
      ? 1000U
      : 8U;
  options.random_seed = 0;
  return options;
}

std::vector<NativeTrajectoryJoint> product_cartesian_joints(
    const YunyiRuntimeResources& product);

std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>
solve_point_to_point_target_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* target_pose,
    std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::time_point::max());

std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>
solve_dual_point_to_point_targets_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    const NativeTrajectorySample& reference,
    const float* left_target_pose,
    const float* right_target_pose,
    std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::time_point::max());

NativeCartesianPlan build_linear_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* end_pose,
    double duration_s,
    float pv_reference_acceleration =
        kNativeRealtimePvTrajectoryAccelerationLimit,
    uint32_t point_count = 0);

NativeCartesianPlan build_linear_path_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* poses,
    uint32_t pose_count,
    double segment_duration_s,
    float pv_reference_acceleration =
        kNativeRealtimePvTrajectoryAccelerationLimit);

NativeCartesianPlan build_linear_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* start_pose,
    const float* end_pose,
    double duration_s,
    float pv_reference_acceleration =
        kNativeRealtimePvTrajectoryAccelerationLimit,
    uint32_t point_count = 0);

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

NativeCartesianPlan build_circular_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* via_pose,
    const float* end_pose,
    double duration_s,
    float pv_reference_acceleration =
        kNativeRealtimePvTrajectoryAccelerationLimit);

NativeCartesianPlan build_circular_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* start_pose,
    const float* via_pose,
    const float* end_pose,
    double duration_s,
    float pv_reference_acceleration =
        kNativeRealtimePvTrajectoryAccelerationLimit);

}  // namespace articore

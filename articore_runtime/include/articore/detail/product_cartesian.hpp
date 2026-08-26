#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>

#include "articore/runtime_abi.h"
#include "articore/detail/runtime.hpp"

namespace articore {

// Linear/circular path timing retains its Cartesian reference ceiling. PTP
// resolves endpoint IK and then uses ordinary PV reference stepping. The
// Damiao POS_VEL V field remains a separate drive-level limit.
inline constexpr float kYunyiCartesianMaximumVelocity = 3.0f;
inline constexpr float kYunyiCartesianPvDriveVelocityLimit = 3.0f;
inline constexpr float kYunyiCartesianWristAccelerationLimit = 6.0f;
inline constexpr float kYunyiCircularSpeedScale = 0.40f;
// A 100 Hz caller has a 10 ms period. Keep two milliseconds outside the
// numerical solve for ABI validation, locking and atomic command install.
inline constexpr std::chrono::microseconds kYunyiMovePoseIkBudget{8000};

inline float product_circular_speed_percent(float public_speed_percent) {
  return public_speed_percent * kYunyiCircularSpeedScale;
}

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
};

enum class CartesianIkSearch {
  LocalPath,
  GlobalEndpoint,
};

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
    float speed_percent);

NativeCartesianPlan build_linear_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* start_pose,
    const float* end_pose,
    float speed_percent);

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
    float speed_percent);

NativeCartesianPlan build_circular_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* start_pose,
    const float* via_pose,
    const float* end_pose,
    float speed_percent);

}  // namespace articore

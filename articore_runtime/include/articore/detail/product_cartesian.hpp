#pragma once

#include <cstdint>

#include "articore/runtime_abi.h"
#include "articore/detail/runtime.hpp"

namespace articore {

struct YunyiRuntimeResources;

struct NativeCartesianPlan {
  NativeTrajectoryRequest trajectory;
  uint64_t replace_trajectory_id = 0;
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

NativeCartesianPlan build_cartesian_plan(
    SafetyRuntime& safety,
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const float* target_pose,
    float speed_percent,
    ArticoreCartesianInterpolation interpolation);

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

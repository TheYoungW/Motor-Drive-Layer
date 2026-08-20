#pragma once

#include <cstdint>

#include "articore/runtime_abi.h"
#include "runtime.hpp"
#include "yunyi_runtime.hpp"

namespace articore {

struct NativeCartesianPlan {
  NativeTrajectoryRequest trajectory;
  uint64_t replace_trajectory_id = 0;
};

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

NativeCartesianPlan build_circular_plan(
    SafetyRuntime& safety,
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const float* start_pose,
    const float* via_pose,
    const float* end_pose,
    float speed_percent);

NativeCartesianPlan build_circular_plan_from_reference(
    YunyiRuntimeResources& product,
    ArticoreControlMode mode,
    uint32_t side,
    const NativeTrajectorySample& reference,
    const float* via_pose,
    const float* end_pose,
    float speed_percent);

}  // namespace articore

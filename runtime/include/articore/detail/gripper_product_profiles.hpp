#pragma once

#include <array>
#include <cstdint>

#include "articore/detail/runtime_types.hpp"

namespace articore {

struct GripperForceCalibration {
  float contact_torque;
  float overload_torque;
  float moving_kp;
  float moving_kd;
  float hold_kp;
  float hold_kd;
};

struct GripperProductProfile {
  const char* id;
  float open_position;
  float closed_position;
  float max_speed;
  uint32_t motion_window_ms;
  float stall_movement;
  float min_position_error;
  uint32_t contact_hold_ms;
  uint32_t overload_hold_ms;
  float hold_offset;
  float retreat_distance;
  uint32_t retreat_retry_ms;
  uint32_t max_step_interval_ms;
  int32_t fault_action;
  std::array<GripperForceCalibration, 10> force_levels;
};

const GripperProductProfile* find_builtin_gripper_product_profile(
    const char* profile_id);

}  // namespace articore

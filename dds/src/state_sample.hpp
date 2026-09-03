#pragma once

#include <algorithm>

#include "articore/runtime.hpp"
#include "articore_protocol.h"

namespace articore::dds::detail {

inline void copy_runtime_state_payload(
    const RuntimeState& source, articore_wire_RobotState& destination) {
  std::copy(source.positions.begin(), source.positions.end(),
            destination.positions);
  std::copy(source.velocities.begin(), source.velocities.end(),
            destination.velocities);
  std::copy(source.torques.begin(), source.torques.end(), destination.torques);
  std::copy(source.mos_temperatures.begin(), source.mos_temperatures.end(),
            destination.mos_temperatures);
  std::copy(source.rotor_temperatures.begin(), source.rotor_temperatures.end(),
            destination.rotor_temperatures);
  destination.enabled_mask = source.enabled_mask;
  destination.enabled_valid_mask = source.enabled_valid_mask;
  destination.temperature_valid_mask = source.temperature_valid_mask;
  std::copy(source.gripper_openings.begin(), source.gripper_openings.end(),
            destination.gripper_openings);
  std::copy(source.gripper_available.begin(), source.gripper_available.end(),
            destination.gripper_available);
  std::copy(source.gripper_feedback_valid.begin(),
            source.gripper_feedback_valid.end(),
            destination.gripper_feedback_valid);
  destination.motion_arrived = source.motion_arrived;
}

}  // namespace articore::dds::detail

#include <type_traits>
#include <vector>

#include "articore/runtime.hpp"

int main() {
  static_assert(!std::is_copy_constructible_v<articore::Runtime>);
  static_assert(!std::is_copy_assignable_v<articore::Runtime>);
  static_assert(std::is_move_constructible_v<articore::Runtime>);

  ArticoreRuntimeConfig config{};
  config.control_hz = 400;
  config.command_timeout_ms = 250;
  config.enable_grace_ms = 2000;
  config.safe_hold_hz = 100;
  config.feedback_check_hz = 100;
  config.feedback_failure_threshold = 3;
  config.feedback_max_age_ms = 20;
  config.safe_hold_failure_threshold = 3;
  config.disable_feedback_timeout_ms = 50;
  config.safe_pv_velocity_limit = 0.5f;
  config.gripper_control_hz = 400;
  config.gripper_fault_action = ARTICORE_GRIPPER_FAULT_HOLD;

  try {
    articore::Runtime runtime(
        config, reinterpret_cast<MotorControllerGroup*>(1),
        reinterpret_cast<MotorController*>(2), nullptr, {});
  } catch (const articore::RuntimeError&) {
    return 0;
  }
  return 1;
}

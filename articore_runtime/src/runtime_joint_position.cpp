#include "runtime.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace articore {

void SafetyRuntime::validate_motor_set(
    const ArticoreJointMitTarget* targets, uint32_t count) const {
  const auto expected = static_cast<uint32_t>(std::count_if(
      motors_.begin(), motors_.end(), [](const MotorRecord& motor) {
        return motor.descriptor.is_gripper == 0;
      }));
  if (count != expected) {
    throw std::invalid_argument(
        "ordinary MIT position command must contain every active arm joint");
  }
  std::set<void*> unique;
  for (uint32_t i = 0; i < count; ++i) {
    const auto& target = targets[i];
    if (target.struct_size < sizeof(ArticoreJointMitTarget) ||
        !target.motor || !finite(target.target_position)) {
      throw std::invalid_argument(
          "ordinary MIT position command contains invalid values");
    }
    const auto found = std::find_if(
        motors_.begin(), motors_.end(), [&](const MotorRecord& motor) {
          return !motor.descriptor.is_gripper &&
                 motor.descriptor.motor == target.motor;
        });
    if (found == motors_.end()) {
      throw std::invalid_argument(
          "ordinary MIT position command contains an unexpected motor");
    }
    if (!unique.insert(target.motor).second) {
      throw std::invalid_argument(
          "ordinary MIT position command contains duplicate motors");
    }
  }
}

void SafetyRuntime::validate_motor_set(
    const ArticoreJointPvTarget* targets, uint32_t count) const {
  const auto expected = static_cast<uint32_t>(std::count_if(
      motors_.begin(), motors_.end(), [](const MotorRecord& motor) {
        return motor.descriptor.is_gripper == 0;
      }));
  if (count != expected) {
    throw std::invalid_argument(
        "ordinary PV position command must contain every active arm joint");
  }
  std::set<void*> unique;
  for (uint32_t i = 0; i < count; ++i) {
    const auto& target = targets[i];
    if (target.struct_size < sizeof(ArticoreJointPvTarget) ||
        !target.motor || !finite(target.target_position)) {
      throw std::invalid_argument(
          "ordinary PV position command contains invalid values");
    }
    const auto found = std::find_if(
        motors_.begin(), motors_.end(), [&](const MotorRecord& motor) {
          return !motor.descriptor.is_gripper &&
                 motor.descriptor.motor == target.motor;
        });
    if (found == motors_.end()) {
      throw std::invalid_argument(
          "ordinary PV position command contains an unexpected motor");
    }
    if (!unique.insert(target.motor).second) {
      throw std::invalid_argument(
          "ordinary PV position command contains duplicate motors");
    }
  }
}

void SafetyRuntime::set_joint_mit(
    const ArticoreJointMitTarget* targets, uint32_t count,
    float max_reference_velocity) {
  if (!targets || count == 0) {
    throw std::invalid_argument("ordinary MIT position target is empty");
  }
  if (!finite(max_reference_velocity) || max_reference_velocity <= 0.0f) {
    throw std::invalid_argument(
        "max_reference_velocity must be finite and greater than zero");
  }
  validate_motor_set(targets, count);

  for (uint32_t i = 0; i < count; ++i) {
    const auto& limits = joint_config(targets[i].motor);
    if (max_reference_velocity > limits.velocity_limit) {
      const auto motor = std::find_if(
          motors_.begin(), motors_.end(), [&](const MotorRecord& record) {
            return record.descriptor.motor == targets[i].motor;
          });
      throw std::invalid_argument(
          std::string(motor == motors_.end()
                          ? "joint"
                          : motor->descriptor.name) +
          ": shared MIT reference velocity exceeds joint safety limit");
    }
    validate_position_velocity_torque(
        targets[i].motor, targets[i].target_position, 0.0f, 0.0f);
  }

  const auto now = Clock::now();
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  require_state_for_command();
  if (mode_ != ARTICORE_MODE_MIT) {
    throw std::runtime_error(
        "ordinary MIT position command requires MIT runtime mode");
  }
  if (active_trajectory_) {
    throw std::runtime_error(
        "joint trajectory is active; ordinary MIT position command rejected");
  }
  for (uint8_t side = 0; side < 2; ++side) {
    if (!active_sides_[side]) continue;
    if (!sides_[side].connected || !sides_[side].transport_healthy) {
      throw std::runtime_error(
          std::string(side == 0 ? "CH0" : "CH1") +
          " transport is not healthy for ordinary MIT position control");
    }
  }

  const bool continuing = arm_mailbox_.joint_position &&
                          mode_ == ARTICORE_MODE_MIT;
  const auto maximum_age_ns =
      static_cast<uint64_t>(config_.feedback_max_age_ms) * 1'000'000ULL;
  std::vector<ArticoreMitCommand> next_commands;
  std::vector<float> next_final_positions;
  next_commands.reserve(count);
  next_final_positions.reserve(count);

  for (uint32_t i = 0; i < count; ++i) {
    const auto& target = targets[i];
    ArticoreFeedbackStats stats{};
    ArticoreMotorState state{};
    if (api_.motor_get_feedback_stats(target.motor, &stats) != 0 ||
        !stats.has_feedback || stats.age_ns > maximum_age_ns ||
        api_.motor_get_state(target.motor, &state) != 0 ||
        !state.has_value || !finite(state.pos) || state.status_code != 1) {
      const auto motor = std::find_if(
          motors_.begin(), motors_.end(), [&](const MotorRecord& record) {
            return record.descriptor.motor == target.motor;
          });
      const std::string name = motor == motors_.end()
          ? "joint" : std::string(motor->descriptor.name);
      throw std::runtime_error(
          "CH" + std::to_string(motor == motors_.end()
                                     ? 0 : motor->descriptor.side) +
          "/" + name + ": complete fresh enabled feedback is required for "
          "ordinary MIT position control");
    }

    float current_position = state.pos;
    if (continuing) {
      const auto previous = std::find_if(
          arm_mailbox_.mit.begin(), arm_mailbox_.mit.end(),
          [&](const ArticoreMitCommand& command) {
            return command.motor == target.motor;
          });
      if (previous == arm_mailbox_.mit.end()) {
        throw std::runtime_error(
            "ordinary MIT position state does not match the active arm layout");
      }
      current_position = previous->target_position;
    } else {
      const auto& limits = joint_config(target.motor);
      if (current_position < limits.hard_lower_position ||
          current_position > limits.hard_upper_position) {
        const auto motor = std::find_if(
            motors_.begin(), motors_.end(), [&](const MotorRecord& record) {
              return record.descriptor.motor == target.motor;
            });
        throw std::runtime_error(
            std::string(motor == motors_.end()
                            ? "joint"
                            : motor->descriptor.name) +
            ": fresh position is outside the configured hard limits");
      }
    }

    const auto& config = joint_config(target.motor);
    next_commands.push_back(ArticoreMitCommand{
        target.motor, current_position, 0.0f,
        config.mit_kp, config.mit_kd, 0.0f});
    next_final_positions.push_back(target.target_position);
  }

  ArmMailbox next;
  next.valid = true;
  next.user_command = true;
  next.trajectory_endpoint_hold = false;
  next.lifetime = ARTICORE_COMMAND_HOLD_UNTIL_REPLACED;
  next.generation = arm_mailbox_.generation + 1;
  next.submitted_at = now;
  next.joint_position = true;
  next.max_reference_velocity = max_reference_velocity;
  next.mit = std::move(next_commands);
  next.final_positions = std::move(next_final_positions);
  arm_mailbox_ = std::move(next);
  wakeup_.notify_all();
}

void SafetyRuntime::set_joint_pv(
    const ArticoreJointPvTarget* targets, uint32_t count,
    float max_reference_velocity) {
  if (!targets || count == 0) {
    throw std::invalid_argument("ordinary PV position target is empty");
  }
  if (!finite(max_reference_velocity) || max_reference_velocity <= 0.0f) {
    throw std::invalid_argument(
        "max_reference_velocity must be finite and greater than zero");
  }
  validate_motor_set(targets, count);

  for (uint32_t i = 0; i < count; ++i) {
    const auto& limits = joint_config(targets[i].motor);
    if (max_reference_velocity > limits.velocity_limit) {
      const auto motor = std::find_if(
          motors_.begin(), motors_.end(), [&](const MotorRecord& record) {
            return record.descriptor.motor == targets[i].motor;
          });
      throw std::invalid_argument(
          std::string(motor == motors_.end()
                          ? "joint"
                          : motor->descriptor.name) +
          ": shared PV reference velocity exceeds joint safety limit");
    }
    validate_position_velocity_torque(
        targets[i].motor, targets[i].target_position, 0.0f, 0.0f);
  }

  const auto now = Clock::now();
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  require_state_for_command();
  if (mode_ != ARTICORE_MODE_PV) {
    throw std::runtime_error(
        "ordinary PV position command requires PV runtime mode");
  }
  if (active_trajectory_) {
    throw std::runtime_error(
        "joint trajectory is active; ordinary PV position command rejected");
  }
  for (uint8_t side = 0; side < 2; ++side) {
    if (!active_sides_[side]) continue;
    if (!sides_[side].connected || !sides_[side].transport_healthy) {
      throw std::runtime_error(
          std::string(side == 0 ? "CH0" : "CH1") +
          " transport is not healthy for ordinary PV position control");
    }
  }

  const bool continuing = arm_mailbox_.joint_position &&
                          mode_ == ARTICORE_MODE_PV;
  const auto maximum_age_ns =
      static_cast<uint64_t>(config_.feedback_max_age_ms) * 1'000'000ULL;
  std::vector<ArticorePosVelCommand> next_commands;
  std::vector<float> next_final_positions;
  next_commands.reserve(count);
  next_final_positions.reserve(count);

  for (uint32_t i = 0; i < count; ++i) {
    const auto& target = targets[i];
    ArticoreFeedbackStats stats{};
    ArticoreMotorState state{};
    if (api_.motor_get_feedback_stats(target.motor, &stats) != 0 ||
        !stats.has_feedback || stats.age_ns > maximum_age_ns ||
        api_.motor_get_state(target.motor, &state) != 0 ||
        !state.has_value || !finite(state.pos) || state.status_code != 1) {
      const auto motor = std::find_if(
          motors_.begin(), motors_.end(), [&](const MotorRecord& record) {
            return record.descriptor.motor == target.motor;
          });
      const std::string name = motor == motors_.end()
          ? "joint" : std::string(motor->descriptor.name);
      throw std::runtime_error(
          "CH" + std::to_string(motor == motors_.end()
                                     ? 0 : motor->descriptor.side) +
          "/" + name + ": complete fresh enabled feedback is required for "
          "ordinary PV position control");
    }

    float current_position = state.pos;
    if (continuing) {
      const auto previous = std::find_if(
          arm_mailbox_.pv.begin(), arm_mailbox_.pv.end(),
          [&](const ArticorePosVelCommand& command) {
            return command.motor == target.motor;
          });
      if (previous == arm_mailbox_.pv.end()) {
        throw std::runtime_error(
            "ordinary PV position state does not match the active arm layout");
      }
      current_position = previous->target_position;
    } else {
      const auto& limits = joint_config(target.motor);
      if (current_position < limits.hard_lower_position ||
          current_position > limits.hard_upper_position) {
        const auto motor = std::find_if(
            motors_.begin(), motors_.end(), [&](const MotorRecord& record) {
              return record.descriptor.motor == target.motor;
            });
        throw std::runtime_error(
            std::string(motor == motors_.end()
                            ? "joint"
                            : motor->descriptor.name) +
            ": fresh position is outside the configured hard limits");
      }
    }

    next_commands.push_back(ArticorePosVelCommand{
        target.motor, current_position, max_reference_velocity});
    next_final_positions.push_back(target.target_position);
  }

  ArmMailbox next;
  next.valid = true;
  next.user_command = true;
  next.trajectory_endpoint_hold = false;
  next.lifetime = ARTICORE_COMMAND_HOLD_UNTIL_REPLACED;
  next.generation = arm_mailbox_.generation + 1;
  next.submitted_at = now;
  next.joint_position = true;
  next.max_reference_velocity = max_reference_velocity;
  next.pv = std::move(next_commands);
  next.final_positions = std::move(next_final_positions);
  arm_mailbox_ = std::move(next);
  wakeup_.notify_all();
}

}  // namespace articore

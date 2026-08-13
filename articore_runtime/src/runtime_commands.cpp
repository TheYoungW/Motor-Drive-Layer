#include "runtime.hpp"
#include "runtime_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace articore {

using detail::age_ns;
using detail::copy_text;

void SafetyRuntime::configure_joints(
    const ArticoreJointControlConfig* configs, uint32_t count) {
  if (!configs || count == 0) {
    throw std::invalid_argument("joint control configuration is empty");
  }
  const auto expected = static_cast<uint32_t>(std::count_if(
      motors_.begin(), motors_.end(), [](const MotorRecord& motor) {
        return motor.descriptor.is_gripper == 0;
      }));
  if (count != expected) {
    throw std::invalid_argument(
        "joint control configuration must cover every active arm motor");
  }
  std::unordered_map<void*, JointControlConfig> configured;
  for (uint32_t i = 0; i < count; ++i) {
    const auto& value = configs[i];
    const auto found = std::find_if(
        motors_.begin(), motors_.end(), [&](const MotorRecord& motor) {
          return !motor.descriptor.is_gripper &&
                 motor.descriptor.motor == value.motor;
        });
    if (found == motors_.end() ||
        !finite(value.lower_position) || !finite(value.upper_position) ||
        value.lower_position > value.upper_position ||
        !finite(value.velocity_limit) || value.velocity_limit <= 0.0f ||
        config_.safe_pv_velocity_limit > value.velocity_limit ||
        !finite(value.torque_limit) || value.torque_limit <= 0.0f ||
        !finite(value.mit_kp) || value.mit_kp < 0.0f ||
        !finite(value.mit_kd) || value.mit_kd < 0.0f ||
        !finite(value.mit_feedforward_torque) ||
        std::abs(value.mit_feedforward_torque) > value.torque_limit) {
      throw std::invalid_argument("invalid joint control configuration");
    }
    if (!configured.emplace(
            value.motor,
            JointControlConfig{value.lower_position, value.upper_position,
                               value.lower_position, value.upper_position,
                               0.0f, 0.0f,
                               value.velocity_limit, value.torque_limit,
                               value.mit_kp, value.mit_kd,
                               value.mit_feedforward_torque}).second) {
      throw std::invalid_argument("duplicate joint control configuration");
    }
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_ != ARTICORE_DISCONNECTED) {
    throw std::runtime_error("joint configuration is fixed after connect");
  }
  joint_configs_ = std::move(configured);
}

void SafetyRuntime::configure_joint_safety_limits(
    const ArticoreJointSafetyLimits* limits, uint32_t count) {
  if (!limits || count == 0) {
    throw std::invalid_argument("joint safety limits are empty");
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_ != ARTICORE_DISCONNECTED) {
    throw std::runtime_error("joint safety limits are fixed after connect");
  }
  if (joint_configs_.empty() || count != joint_configs_.size()) {
    throw std::invalid_argument(
        "joint safety limits must cover every configured arm motor");
  }
  std::set<void*> unique;
  auto updated = joint_configs_;
  for (uint32_t i = 0; i < count; ++i) {
    const auto& value = limits[i];
    const auto configured = updated.find(value.motor);
    if (value.struct_size < sizeof(ArticoreJointSafetyLimits) ||
        configured == updated.end() || !unique.insert(value.motor).second ||
        !finite(value.hard_lower_position) ||
        !finite(value.hard_upper_position) ||
        !finite(value.soft_lower_position) ||
        !finite(value.soft_upper_position) ||
        !finite(value.soft_limit_braking_zone) ||
        !finite(value.braking_acceleration) ||
        value.hard_lower_position >= value.hard_upper_position ||
        value.soft_lower_position < value.hard_lower_position ||
        value.soft_upper_position > value.hard_upper_position ||
        value.soft_lower_position >= value.soft_upper_position ||
        value.soft_limit_braking_zone <= 0.0f ||
        value.soft_limit_braking_zone >
            value.soft_upper_position - value.soft_lower_position ||
        value.braking_acceleration <= 0.0f) {
      throw std::invalid_argument("invalid layered joint safety limits");
    }
    auto& output = configured->second;
    output.hard_lower_position = value.hard_lower_position;
    output.hard_upper_position = value.hard_upper_position;
    output.soft_lower_position = value.soft_lower_position;
    output.soft_upper_position = value.soft_upper_position;
    output.soft_limit_braking_zone = value.soft_limit_braking_zone;
    output.braking_acceleration = value.braking_acceleration;
    output.layered_limits_configured = true;
  }
  joint_configs_ = std::move(updated);
}

const SafetyRuntime::JointControlConfig& SafetyRuntime::joint_config(
    void* motor) const {
  const auto found = joint_configs_.find(motor);
  if (found == joint_configs_.end()) {
    throw std::runtime_error(
        "joint control configuration is required for native joint control");
  }
  return found->second;
}

void SafetyRuntime::validate_position_velocity_torque(
    void* motor, float position, float velocity, float torque) const {
  if (!finite(position) || !finite(velocity) || !finite(torque)) {
    throw std::invalid_argument("joint command contains non-finite values");
  }
  const auto configured = joint_configs_.find(motor);
  if (configured != joint_configs_.end()) {
    const auto& limits = configured->second;
    if (position < limits.hard_lower_position ||
        position > limits.hard_upper_position) {
      throw std::invalid_argument("joint command exceeds hard position limits");
    }
    if (position < limits.soft_lower_position ||
        position > limits.soft_upper_position) {
      throw std::invalid_argument("joint command exceeds position limits");
    }
    if (std::abs(velocity) > limits.velocity_limit) {
      throw std::invalid_argument("joint command exceeds velocity limit");
    }
    if (std::abs(torque) > limits.torque_limit) {
      throw std::invalid_argument("joint command exceeds torque limit");
    }
    return;
  }
  const auto descriptor = std::find_if(
      motors_.begin(), motors_.end(), [&](const MotorRecord& record) {
        return record.descriptor.motor == motor;
      });
  if (descriptor != motors_.end() &&
      (position < descriptor->descriptor.lower_position ||
       position > descriptor->descriptor.upper_position)) {
    throw std::invalid_argument("joint command exceeds position limits");
  }
}

void SafetyRuntime::initialize_arm_mailbox_from_feedback(
    ArticoreControlMode mode, bool require_enabled) {
  ArmMailbox initialized;
  initialized.valid = true;
  initialized.user_command = false;
  initialized.generation = arm_mailbox_.generation + 1;
  initialized.submitted_at = Clock::now();
  for (const auto& motor : motors_) {
    if (motor.descriptor.is_gripper) continue;
    ArticoreFeedbackStats stats{};
    ArticoreMotorState state{};
    const std::string name(motor.descriptor.name);
    if (api_.motor_get_feedback_stats(motor.descriptor.motor, &stats) != 0 ||
        !stats.has_feedback ||
        stats.age_ns > static_cast<uint64_t>(config_.feedback_max_age_ms) *
                           1'000'000ULL ||
        api_.motor_get_state(motor.descriptor.motor, &state) != 0 ||
        !state.has_value || !finite(state.pos) || !finite(state.vel) ||
        !finite(state.torq) || state.status_code > 1 ||
        (require_enabled && state.status_code != 1)) {
      throw std::runtime_error(
          name + (require_enabled
              ? ": fresh enabled feedback is required before enable"
              : ": fresh fault-free feedback is required before enable"));
    }
    if (mode == ARTICORE_MODE_PV) {
      initialized.pv.push_back(ArticorePosVelCommand{
          motor.descriptor.motor, state.pos, config_.safe_pv_velocity_limit});
    } else {
      const auto configured = joint_configs_.find(motor.descriptor.motor);
      const auto kp = configured == joint_configs_.end()
          ? motor.descriptor.safe_kp : configured->second.mit_kp;
      const auto kd = configured == joint_configs_.end()
          ? motor.descriptor.safe_kd : configured->second.mit_kd;
      const auto tau = configured == joint_configs_.end()
          ? 0.0f : configured->second.mit_feedforward_torque;
      initialized.mit.push_back(ArticoreMitCommand{
          motor.descriptor.motor, state.pos, 0.0f, kp, kd, tau});
    }
  }
  if (initialized.pv.empty() && initialized.mit.empty()) {
    throw std::runtime_error("runtime requires at least one active arm motor");
  }
  arm_mailbox_ = std::move(initialized);
}

void SafetyRuntime::require_state_for_command() const {
  if (fault_latched_ || hardware_transition_ || enable_transaction_ ||
      (state_ != ARTICORE_ENABLED && state_ != ARTICORE_RUNNING)) {
    throw std::runtime_error("Articore runtime is not accepting motion commands");
  }
}

void SafetyRuntime::validate_motor_set(const ArticorePosVelCommand* commands,
                                       uint32_t count,
                                       bool grippers_only) const {
  const auto expected = static_cast<uint32_t>(std::count_if(
      motors_.begin(), motors_.end(), [&](const MotorRecord& motor) {
        return (motor.descriptor.is_gripper != 0) == grippers_only;
      }));
  if (count != expected) {
    throw std::invalid_argument("command must contain the complete fixed motor layout");
  }
  for (uint32_t i = 0; i < count; ++i) {
    const auto found = std::find_if(
        motors_.begin(), motors_.end(), [&](const MotorRecord& motor) {
          return motor.descriptor.motor == commands[i].motor &&
                 (motor.descriptor.is_gripper != 0) == grippers_only;
        });
    if (found == motors_.end()) {
      throw std::invalid_argument("command contains an unexpected motor");
    }
    for (uint32_t previous = 0; previous < i; ++previous) {
      if (commands[previous].motor == commands[i].motor) {
        throw std::invalid_argument("command contains duplicate motors");
      }
    }
  }
}

void SafetyRuntime::validate_motor_set(const ArticoreMitCommand* commands,
                                       uint32_t count,
                                       bool grippers_only) const {
  const auto expected = static_cast<uint32_t>(std::count_if(
      motors_.begin(), motors_.end(), [&](const MotorRecord& motor) {
        return (motor.descriptor.is_gripper != 0) == grippers_only;
      }));
  if (count != expected && !grippers_only) {
    throw std::invalid_argument("command must contain the complete fixed motor layout");
  }
  if (grippers_only && (count == 0 || count > expected)) {
    throw std::invalid_argument("gripper command contains an invalid motor count");
  }
  for (uint32_t i = 0; i < count; ++i) {
    const auto found = std::find_if(
        motors_.begin(), motors_.end(), [&](const MotorRecord& motor) {
          return motor.descriptor.motor == commands[i].motor &&
                 (motor.descriptor.is_gripper != 0) == grippers_only;
        });
    if (found == motors_.end()) {
      throw std::invalid_argument("command contains an unexpected motor");
    }
    for (uint32_t previous = 0; previous < i; ++previous) {
      if (commands[previous].motor == commands[i].motor) {
        throw std::invalid_argument("command contains duplicate motors");
      }
    }
  }
}

bool SafetyRuntime::enter_safe_hold_from_feedback(const std::string& reason,
                                                  std::string& error) {
  ArticoreControlMode mode;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (state_ != ARTICORE_RUNNING || !has_successful_command_) {
      error = "current-position hold requires a running arm command";
      return false;
    }
    mode = mode_;
  }

  std::vector<ArticorePosVelCommand> pv;
  std::vector<ArticoreMitCommand> mit;
  uint64_t maximum_age_ns = 0;
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  for (const auto& motor : motors_) {
    if (motor.descriptor.is_gripper) continue;

    const std::string name(motor.descriptor.name);
    ArticoreFeedbackStats stats{};
    if (api_.motor_get_feedback_stats(motor.descriptor.motor, &stats) != 0 ||
        !stats.has_feedback) {
      error = name + ": current-position feedback is unavailable";
      return false;
    }
    if (stats.age_ns >
        static_cast<uint64_t>(config_.feedback_max_age_ms) * 1'000'000ULL) {
      error = name + ": current-position feedback exceeds maximum age";
      return false;
    }

    ArticoreMotorState state{};
    if (api_.motor_get_state(motor.descriptor.motor, &state) != 0 ||
        !state.has_value) {
      error = name + ": current-position motor state is unavailable";
      return false;
    }
    if (!finite(state.pos)) {
      error = name + ": current-position feedback is not finite";
      return false;
    }
    if (state.status_code == 0) {
      error = name + ": motor is unexpectedly disabled";
      return false;
    }
    if (state.status_code > 1) {
      error = name + ": motor fault status " +
              std::to_string(state.status_code);
      return false;
    }

    maximum_age_ns = std::max(maximum_age_ns, stats.age_ns);
    if (mode == ARTICORE_MODE_PV) {
      pv.push_back(ArticorePosVelCommand{
          motor.descriptor.motor, state.pos, config_.safe_pv_velocity_limit});
    } else {
      mit.push_back(ArticoreMitCommand{
          motor.descriptor.motor, state.pos, 0.0f,
          motor.descriptor.safe_kp, motor.descriptor.safe_kd, 0.0f});
    }
  }
  if (pv.empty() && mit.empty()) {
    error = "current-position hold requires at least one arm motor";
    return false;
  }

  const auto now = Clock::now();
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  if (state_ != ARTICORE_RUNNING || !has_successful_command_ || mode_ != mode) {
    error = "runtime state changed while capturing current positions";
    return false;
  }
  safe_pv_ = std::move(pv);
  safe_mit_ = std::move(mit);
  fault_hold_active_ = false;
  arm_mailbox_ = ArmMailbox{};
  last_fresh_feedback_ = now - std::chrono::nanoseconds(maximum_age_ns);
  state_ = ARTICORE_SAFE_HOLD;
  fault_reason_ = reason;
  next_safe_hold_ = now;
  consecutive_hold_failures_ = 0;
  return true;
}

void SafetyRuntime::submit_pos_vel(const ArticorePosVelCommand* commands,
                                   uint32_t count) {
  submit_pos_vel_ex(commands, count, ARTICORE_COMMAND_STREAMING);
}

void SafetyRuntime::submit_pos_vel_ex(const ArticorePosVelCommand* commands,
                                      uint32_t count,
                                      ArticoreCommandLifetime lifetime) {
  if (lifetime != ARTICORE_COMMAND_STREAMING &&
      lifetime != ARTICORE_COMMAND_HOLD_UNTIL_REPLACED) {
    throw std::invalid_argument("invalid PV command lifetime");
  }
  if (!commands || count == 0) throw std::invalid_argument("PV command is empty");
  for (uint32_t i = 0; i < count; ++i) {
    if (!commands[i].motor || !finite(commands[i].target_position) ||
        !finite(commands[i].velocity_limit) || commands[i].velocity_limit <= 0.0f) {
      throw std::invalid_argument("PV command contains invalid values");
    }
    validate_position_velocity_torque(
        commands[i].motor, commands[i].target_position,
        commands[i].velocity_limit, 0.0f);
  }
  validate_motor_set(commands, count, false);

  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    require_state_for_command();
  }
  {
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    require_state_for_command();
    if (mode_ != ARTICORE_MODE_PV) {
      throw std::runtime_error("cannot submit PV while runtime mode is MIT");
    }
    arm_mailbox_.valid = true;
    arm_mailbox_.user_command = true;
    arm_mailbox_.lifetime = lifetime;
    ++arm_mailbox_.generation;
    arm_mailbox_.submitted_at = Clock::now();
    arm_mailbox_.joint_position = false;
    arm_mailbox_.max_reference_velocity = 0.0f;
    arm_mailbox_.final_positions.clear();
    arm_mailbox_.pv.assign(commands, commands + count);
    arm_mailbox_.mit.clear();
  }
  wakeup_.notify_all();
}

void SafetyRuntime::submit_mit(const ArticoreMitCommand* commands, uint32_t count) {
  submit_mit_ex(commands, count, ARTICORE_COMMAND_STREAMING);
}

void SafetyRuntime::submit_mit_ex(const ArticoreMitCommand* commands,
                                  uint32_t count,
                                  ArticoreCommandLifetime lifetime) {
  if (lifetime != ARTICORE_COMMAND_STREAMING &&
      lifetime != ARTICORE_COMMAND_HOLD_UNTIL_REPLACED) {
    throw std::invalid_argument("invalid MIT command lifetime");
  }
  if (!commands || count == 0) throw std::invalid_argument("MIT command is empty");
  for (uint32_t i = 0; i < count; ++i) {
    const auto& command = commands[i];
    if (!command.motor || !finite(command.target_position) ||
        !finite(command.target_velocity) || !finite(command.stiffness) ||
        !finite(command.damping) || !finite(command.feedforward_torque) ||
        command.stiffness < 0.0f || command.damping < 0.0f) {
      throw std::invalid_argument("MIT command contains invalid values");
    }
    if (lifetime == ARTICORE_COMMAND_HOLD_UNTIL_REPLACED &&
        (command.target_velocity != 0.0f ||
         command.feedforward_torque != 0.0f)) {
      throw std::invalid_argument(
          "persistent MIT command requires zero target velocity and "
          "feedforward torque");
    }
    validate_position_velocity_torque(
        command.motor, command.target_position, command.target_velocity,
        command.feedforward_torque);
  }
  validate_motor_set(commands, count, false);

  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    require_state_for_command();
  }
  {
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    require_state_for_command();
    if (mode_ != ARTICORE_MODE_MIT) {
      throw std::runtime_error("cannot submit MIT while runtime mode is PV");
    }
    arm_mailbox_.valid = true;
    arm_mailbox_.user_command = true;
    arm_mailbox_.lifetime = lifetime;
    ++arm_mailbox_.generation;
    arm_mailbox_.submitted_at = Clock::now();
    arm_mailbox_.joint_position = false;
    arm_mailbox_.max_reference_velocity = 0.0f;
    arm_mailbox_.final_positions.clear();
    arm_mailbox_.mit.assign(commands, commands + count);
    arm_mailbox_.pv.clear();
  }
  wakeup_.notify_all();
}

bool SafetyRuntime::run_arm_control_cycle(Clock::time_point now,
                                          std::string& error) {
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  ArticoreControlMode mode;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (hardware_transition_ ||
        (state_ != ARTICORE_ENABLED && state_ != ARTICORE_RUNNING)) {
      return true;
    }
    mode = mode_;
  }

  if (!arm_mailbox_.valid) return true;
  const bool mailbox_user_command = arm_mailbox_.user_command;
  const uint64_t mailbox_generation = arm_mailbox_.generation;
  if (arm_mailbox_.joint_position) {
    const auto command_size = mode == ARTICORE_MODE_PV
        ? arm_mailbox_.pv.size() : arm_mailbox_.mit.size();
    if (command_size != arm_mailbox_.final_positions.size() ||
        !finite(arm_mailbox_.max_reference_velocity) ||
        arm_mailbox_.max_reference_velocity <= 0.0f) {
      throw std::runtime_error(
          "ordinary joint position state is internally inconsistent");
    }
    const float max_delta = arm_mailbox_.max_reference_velocity /
                            static_cast<float>(config_.control_hz);
    if (mode == ARTICORE_MODE_PV) {
      for (std::size_t i = 0; i < arm_mailbox_.pv.size(); ++i) {
        auto& command = arm_mailbox_.pv[i];
        const float error_to_target =
            arm_mailbox_.final_positions[i] - command.target_position;
        command.target_position += std::clamp(
            error_to_target, -max_delta, max_delta);
        command.velocity_limit = arm_mailbox_.max_reference_velocity;
      }
    } else {
      for (std::size_t i = 0; i < arm_mailbox_.mit.size(); ++i) {
        auto& command = arm_mailbox_.mit[i];
        const float error_to_target =
            arm_mailbox_.final_positions[i] - command.target_position;
        command.target_position += std::clamp(
            error_to_target, -max_delta, max_delta);
        command.target_velocity = 0.0f;
        command.feedforward_torque = 0.0f;
      }
    }
  }

  const auto* pv_data = arm_mailbox_.pv.data();
  const auto* mit_data = arm_mailbox_.mit.data();
  const uint32_t command_count = mode == ARTICORE_MODE_PV
      ? static_cast<uint32_t>(arm_mailbox_.pv.size())
      : static_cast<uint32_t>(arm_mailbox_.mit.size());

  const int32_t result = mode == ARTICORE_MODE_PV
      ? api_.group_send_pos_vel(controller_group_, pv_data, command_count)
      : api_.group_send_mit(controller_group_, mit_data, command_count);
  if (result != 0) {
    error = motor_error(mode == ARTICORE_MODE_PV
                            ? "ControllerGroup PV send failed"
                            : "ControllerGroup MIT send failed");
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    ++consecutive_send_failures_;
    for (uint8_t side = 0; side < 2; ++side) {
      if (active_sides_[side]) set_side_error_locked(side, error, true);
    }
    return false;
  }

  std::lock_guard<std::mutex> state_lock(state_mutex_);
  consecutive_send_failures_ = 0;
  if (mode == ARTICORE_MODE_PV) {
    last_sent_pv_.assign(pv_data, pv_data + command_count);
    last_sent_mit_.clear();
  } else {
    last_sent_mit_.assign(mit_data, mit_data + command_count);
    last_sent_pv_.clear();
  }
  for (uint8_t side = 0; side < 2; ++side) {
    if (!active_sides_[side]) continue;
    sides_[side].send_failures = 0;
    sides_[side].healthy = true;
  }
  if (mailbox_user_command) {
    has_successful_command_ = true;
    state_ = ARTICORE_RUNNING;
    if (mailbox_generation > arm_mailbox_.sent_generation) {
      arm_mailbox_.sent_generation = mailbox_generation;
      last_successful_command_ = now;
    }
  }
  return true;
}

}  // namespace articore

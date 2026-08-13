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

const SafetyRuntime::JointControlConfig& SafetyRuntime::joint_config(
    void* motor) const {
  const auto found = joint_configs_.find(motor);
  if (found == joint_configs_.end()) {
    throw std::runtime_error(
        "joint control configuration is required for trajectories");
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
    if (position < limits.lower_position || position > limits.upper_position) {
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
  cancel_active_trajectory_locked(
      ARTICORE_TRAJECTORY_FAILED, reason);
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
    if (active_trajectory_) {
      throw std::runtime_error(
          "joint trajectory is active; direct PV command rejected");
    }
    arm_mailbox_.valid = true;
    arm_mailbox_.user_command = true;
    arm_mailbox_.trajectory_endpoint_hold = false;
    arm_mailbox_.lifetime = lifetime;
    ++arm_mailbox_.generation;
    arm_mailbox_.submitted_at = Clock::now();
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
    if (active_trajectory_) {
      throw std::runtime_error(
          "joint trajectory is active; direct MIT command rejected");
    }
    arm_mailbox_.valid = true;
    arm_mailbox_.user_command = true;
    arm_mailbox_.trajectory_endpoint_hold = false;
    arm_mailbox_.lifetime = lifetime;
    ++arm_mailbox_.generation;
    arm_mailbox_.submitted_at = Clock::now();
    arm_mailbox_.mit.assign(commands, commands + count);
    arm_mailbox_.pv.clear();
  }
  wakeup_.notify_all();
}

bool SafetyRuntime::run_arm_control_cycle(Clock::time_point now,
                                          std::string& error) {
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  ArticoreControlMode mode;
  std::optional<TrajectoryRecord> trajectory;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (hardware_transition_ ||
        (state_ != ARTICORE_ENABLED && state_ != ARTICORE_RUNNING)) {
      return true;
    }
    mode = mode_;
    trajectory = active_trajectory_;
  }

  std::vector<ArticorePosVelCommand> pv;
  std::vector<ArticoreMitCommand> mit;
  const ArticorePosVelCommand* pv_data = nullptr;
  const ArticoreMitCommand* mit_data = nullptr;
  uint32_t command_count = 0;
  bool mailbox_user_command = false;
  uint64_t mailbox_generation = 0;
  bool trajectory_complete = false;
  uint64_t trajectory_id = 0;
  if (trajectory) {
    trajectory_id = trajectory->id;
    const auto elapsed = std::max(
        std::chrono::nanoseconds::zero(),
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            now - trajectory->start_time));
    const auto complete = elapsed >= trajectory->duration;
    const double u = complete ? 1.0 : std::clamp(
        static_cast<double>(elapsed.count()) / trajectory->duration.count(),
        0.0, 1.0);
    double position_scale = u;
    double velocity_scale_per_second = complete
        ? 0.0 : 1.0 / std::chrono::duration<double>(trajectory->duration).count();
    if (trajectory->profile == ARTICORE_TRAJECTORY_MIN_JERK) {
      const auto u2 = u * u;
      const auto u3 = u2 * u;
      const auto u4 = u3 * u;
      const auto u5 = u4 * u;
      position_scale = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
      velocity_scale_per_second = complete ? 0.0 :
          (30.0 * u2 - 60.0 * u3 + 30.0 * u4) /
              std::chrono::duration<double>(trajectory->duration).count();
    }
    for (const auto& joint : trajectory->joints) {
      const auto delta = static_cast<double>(joint.goal_position) -
                         joint.start_position;
      const auto position = complete ? joint.goal_position : static_cast<float>(
          joint.start_position + position_scale * delta);
      const auto velocity = complete ? 0.0f : static_cast<float>(
          velocity_scale_per_second * delta);
      const auto& config = joint_config(joint.motor);
      validate_position_velocity_torque(
          joint.motor, position, velocity, config.mit_feedforward_torque);
      if (mode == ARTICORE_MODE_PV) {
        pv.push_back(ArticorePosVelCommand{
            joint.motor, position, joint.velocity_limit});
      } else {
        mit.push_back(ArticoreMitCommand{
            joint.motor, position, velocity, config.mit_kp, config.mit_kd,
            config.mit_feedforward_torque});
      }
    }
    trajectory_complete = complete;
    if (mode == ARTICORE_MODE_PV) {
      pv_data = pv.data();
      command_count = static_cast<uint32_t>(pv.size());
    } else {
      mit_data = mit.data();
      command_count = static_cast<uint32_t>(mit.size());
    }
  } else {
    if (!arm_mailbox_.valid) return true;
    mailbox_user_command = arm_mailbox_.user_command;
    mailbox_generation = arm_mailbox_.generation;
    if (mode == ARTICORE_MODE_PV) {
      pv_data = arm_mailbox_.pv.data();
      command_count = static_cast<uint32_t>(arm_mailbox_.pv.size());
    } else {
      mit_data = arm_mailbox_.mit.data();
      command_count = static_cast<uint32_t>(arm_mailbox_.mit.size());
    }
  }

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
    if (trajectory_id != 0) {
      finish_trajectory_locked(
          trajectory_id, ARTICORE_TRAJECTORY_FAILED, error, now);
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
  if (trajectory_id != 0) {
    has_successful_command_ = true;
    state_ = ARTICORE_RUNNING;
    last_successful_command_ = now;
    if (trajectory_complete && active_trajectory_ &&
        active_trajectory_->id == trajectory_id) {
      arm_mailbox_.valid = true;
      arm_mailbox_.user_command = false;
      arm_mailbox_.trajectory_endpoint_hold = true;
      arm_mailbox_.lifetime = ARTICORE_COMMAND_HOLD_UNTIL_REPLACED;
      ++arm_mailbox_.generation;
      arm_mailbox_.sent_generation = arm_mailbox_.generation;
      arm_mailbox_.submitted_at = now;
      arm_mailbox_.pv = std::move(pv);
      arm_mailbox_.mit = std::move(mit);
      finish_trajectory_locked(
          trajectory_id, ARTICORE_TRAJECTORY_COMPLETED, "", now);
    }
  } else if (mailbox_user_command) {
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

#include "articore/detail/runtime.hpp"
#include "articore/detail/runtime_utils.hpp"

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

uint64_t SafetyRuntime::next_arm_generation() noexcept {
  return arm_generation_.fetch_add(1, std::memory_order_relaxed) + 1;
}

void SafetyRuntime::consume_pending_arm_mailbox() {
  std::lock_guard<std::mutex> pending_lock(pending_arm_mutex_);
  if (!pending_arm_mailbox_.valid) return;
  arm_mailbox_ = std::move(pending_arm_mailbox_);
  pending_arm_mailbox_ = ArmMailbox{};
}

void SafetyRuntime::clear_pending_arm_mailbox() {
  std::lock_guard<std::mutex> pending_lock(pending_arm_mutex_);
  pending_arm_mailbox_ = ArmMailbox{};
}

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
    if (value.struct_size != sizeof(ArticoreJointSafetyLimits) ||
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
  initialized.lifetime = ARTICORE_COMMAND_HOLD_UNTIL_REPLACED;
  initialized.generation = next_arm_generation();
  initialized.submitted_at = Clock::now();
  initialized.joint_position = true;
  initialized.max_reference_velocity = 0.0f;
  initialized.max_reference_acceleration =
      mode == ARTICORE_MODE_PV
      ? kNativeOrdinaryPvDefaultAcceleration : 0.0f;
  initialized.pv_velocity_limit = config_.safe_pv_velocity_limit;
  for (const auto& motor : motors_) {
    if (motor.descriptor.is_gripper) continue;
    ArticoreFeedbackStats stats{};
    ArticoreMotorState state{};
    const std::string name(motor.descriptor.name);
    if (backend_->get_feedback_stats(motor.descriptor.motor, &stats) != 0 ||
        !stats.has_feedback ||
        stats.age_ns > static_cast<uint64_t>(config_.feedback_max_age_ms) *
                           1'000'000ULL ||
        backend_->get_state(motor.descriptor.motor, &state) != 0 ||
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
      initialized.pv_hold_confirmation_cycles.push_back(0);
      initialized.pv_stationary_hold.push_back(0);
      initialized.pv_reference_velocities.push_back(0.0f);
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
    initialized.final_positions.push_back(state.pos);
  }
  if (initialized.pv.empty() && initialized.mit.empty()) {
    throw std::runtime_error("runtime requires at least one active arm motor");
  }
  arm_mailbox_ = std::move(initialized);
}

void SafetyRuntime::require_state_for_command(bool allow_gravity,
                                              bool allow_trajectory,
                                              uint64_t planning_token) const {
  if (fault_latched_ || hardware_transition_ || enable_transaction_ ||
      enable_grace_transition_ ||
      (state_ != ARTICORE_ENABLED && state_ != ARTICORE_RUNNING &&
       state_ != ARTICORE_DEGRADED &&
       state_ != ARTICORE_PARTIALLY_ENABLED)) {
    throw std::runtime_error("Articore runtime is not accepting motion commands");
  }
  if (active_command_planning_token_ != 0 &&
      active_command_planning_token_ != planning_token) {
    throw std::runtime_error(
        "motion commands are owned by an active native planning transaction");
  }
  if (!allow_gravity &&
      gravity_control_.phase != ARTICORE_GRAVITY_INACTIVE) {
    throw std::runtime_error(
        "arm commands are owned by active gravity compensation");
  }
  if (!allow_trajectory &&
      trajectory_control_.state == ARTICORE_MOTION_RUNNING) {
    throw std::runtime_error(
        "arm commands are owned by an active native trajectory");
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
    if (backend_->get_feedback_stats(motor.descriptor.motor, &stats) != 0 ||
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
    if (backend_->get_state(motor.descriptor.motor, &state) != 0 ||
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
  clear_pending_arm_mailbox();
  arm_mailbox_ = ArmMailbox{};
  last_fresh_feedback_ = now - std::chrono::nanoseconds(maximum_age_ns);
  state_ = ARTICORE_SAFE_HOLD;
  fault_reason_.clear();
  safety_reason_ = reason;
  next_safe_hold_ = now;
  consecutive_hold_failures_ = 0;
  return true;
}

void SafetyRuntime::enter_degraded(const std::string& reason) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_ != ARTICORE_RUNNING) return;
  state_ = ARTICORE_DEGRADED;
  safety_reason_ = reason;
  // This is deliberately not a motor fault. Existing targets continue at a
  // conservative scale while the Runtime decides whether feedback recovers or
  // a protective stop is required.
  fault_reason_.clear();
}

bool SafetyRuntime::enter_safe_stop(const std::string& reason,
                                    std::string& error) {
  if (!prepare_protective_hold(error)) return false;
  const auto now = Clock::now();
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_ == ARTICORE_DISCONNECTED || state_ == ARTICORE_READY ||
      state_ == ARTICORE_FAULT) {
    error = "runtime state changed before safe stop";
    return false;
  }
  state_ = ARTICORE_SAFE_STOP;
  terminate_trajectory_locked(
      ARTICORE_MOTION_FAULT,
      "trajectory terminated by safe stop: " + reason);
  fault_latched_ = false;
  fault_reason_.clear();
  safety_reason_ = reason;
  clear_pending_arm_mailbox();
  arm_mailbox_ = ArmMailbox{};
  gravity_control_.phase = ARTICORE_GRAVITY_INACTIVE;
  gravity_control_.hold_positions.clear();
  gravity_control_.status.active = 0;
  gravity_control_.status.phase = ARTICORE_GRAVITY_INACTIVE;
  reset_bimanual_follow_locked();
  next_safe_hold_ = now;
  consecutive_hold_failures_ = 0;
  fault_hold_active_ = !safe_pv_.empty() || !safe_mit_.empty() ||
                       !safe_grippers_.empty();
  return fault_hold_active_;
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

  ArmMailbox pending;
  pending.valid = true;
  pending.user_command = true;
  pending.lifetime = lifetime;
  pending.generation = next_arm_generation();
  pending.submitted_at = Clock::now();
  pending.joint_position = false;
  pending.pv.assign(commands, commands + count);
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    require_state_for_command();
    if (bimanual_follow_.active) {
      throw std::runtime_error(
          "raw PV cannot replace active bimanual ordinary control");
    }
    if (mode_ != ARTICORE_MODE_PV) {
      throw std::runtime_error("cannot submit PV while runtime mode is MIT");
    }
    std::lock_guard<std::mutex> pending_lock(pending_arm_mutex_);
    pending_arm_mailbox_ = std::move(pending);
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

  ArmMailbox pending;
  pending.valid = true;
  pending.user_command = true;
  pending.lifetime = lifetime;
  pending.generation = next_arm_generation();
  pending.submitted_at = Clock::now();
  pending.joint_position = false;
  pending.mit.assign(commands, commands + count);
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    require_state_for_command();
    if (bimanual_follow_.active) {
      throw std::runtime_error(
          "raw MIT cannot replace active bimanual ordinary control");
    }
    if (mode_ != ARTICORE_MODE_MIT) {
      throw std::runtime_error("cannot submit MIT while runtime mode is PV");
    }
    std::lock_guard<std::mutex> pending_lock(pending_arm_mutex_);
    pending_arm_mailbox_ = std::move(pending);
  }
  wakeup_.notify_all();
}

bool SafetyRuntime::prepare_mit_torque_limited_commands(
    const std::vector<ArticoreMitCommand>& requested,
    std::vector<ArticoreMitCommand>& applied,
    ArticoreMitTorqueLimitStats& cycle_stats,
    std::string& error,
    float torque_limit_scale) const {
  cycle_stats = {};
  cycle_stats.struct_size = sizeof(cycle_stats);
  std::fill(std::begin(cycle_stats.applied_scale),
            std::end(cycle_stats.applied_scale), 1.0f);
  if (requested.size() > ARTICORE_PRODUCT_DUAL_ARM_DOF) {
    error = "MIT torque limiter received too many arm joints";
    return false;
  }

  applied.assign(requested.begin(), requested.end());
  cycle_stats.joint_count = static_cast<uint32_t>(requested.size());
  // The callback backend exists only for deterministic unit tests. Product
  // Yunyi construction always installs native joint limits before connect.
  if (joint_configs_.empty()) return true;

  std::array<ArticoreMotorState, ARTICORE_PRODUCT_DUAL_ARM_DOF> feedback{};
  for (std::size_t index = 0; index < requested.size(); ++index) {
    const auto& command = requested[index];
    const auto configured = joint_configs_.find(command.motor);
    const auto role = motor_roles_.find(command.motor);
    const std::string name = role == motor_roles_.end()
        ? std::string("unknown arm motor") : role->second;
    if (configured == joint_configs_.end()) {
      error = name + ": MIT torque limit configuration is unavailable";
      return false;
    }

    auto& state = feedback[index];
    if (backend_->get_state(command.motor, &state) != 0 ||
        !state.has_value || !finite(state.pos) || !finite(state.vel)) {
      error = name +
          ": MIT torque limit requires finite position/velocity feedback";
      return false;
    }
    if (state.status_code != 1) {
      error = name +
          ": MIT torque limit requires enabled feedback; actual_status=" +
          std::to_string(state.status_code);
      return false;
    }
  }

  for (std::size_t index = 0; index < requested.size(); ++index) {
    const auto& command = requested[index];
    auto& output = applied[index];
    const auto& state = feedback[index];
    const auto& limits = joint_configs_.at(command.motor);
    const double resultant =
        static_cast<double>(command.stiffness) *
            (static_cast<double>(command.target_position) - state.pos) +
        static_cast<double>(command.damping) *
            (static_cast<double>(command.target_velocity) - state.vel) +
        command.feedforward_torque;
    if (!std::isfinite(resultant)) {
      error = motor_roles_.at(command.motor) +
          ": MIT resultant torque is not finite";
      return false;
    }
    const float requested_torque = static_cast<float>(resultant);
    if (!finite(requested_torque)) {
      error = motor_roles_.at(command.motor) +
          ": MIT resultant torque exceeds the native command range";
      return false;
    }
    const float torque_limit = limits.torque_limit * torque_limit_scale;
    float scale = 1.0f;
    if (std::abs(requested_torque) > torque_limit) {
      scale = torque_limit / std::abs(requested_torque);
      output.stiffness *= scale;
      output.damping *= scale;
      output.feedforward_torque *= scale;
      cycle_stats.torque_limited_joint_mask |= uint64_t{1} << index;
    }
    cycle_stats.requested_resultant_torque[index] = requested_torque;
    cycle_stats.applied_scale[index] = scale;
    cycle_stats.applied_resultant_torque[index] = requested_torque * scale;
  }
  return true;
}

bool SafetyRuntime::run_arm_control_cycle(Clock::time_point now,
                                          bool include_grippers,
                                          std::string& error) {
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  ArticoreControlMode mode;
  bool gravity_active = false;
  bool degraded = false;
  bool adaptive_cartesian_tracking = false;
  bool bimanual_active = false;
  uint32_t bimanual_leader_side = ARTICORE_ROBOT_LEFT;
  std::vector<float> bimanual_start_positions;
  std::array<float, ARTICORE_PRODUCT_ARM_DOF> bimanual_follower_reference{};
  std::array<float, ARTICORE_PRODUCT_ARM_DOF>
      bimanual_follower_reference_velocity{};
  std::set<void*> intentionally_disabled;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (hardware_transition_ ||
        (state_ != ARTICORE_ENABLED && state_ != ARTICORE_RUNNING &&
         state_ != ARTICORE_DEGRADED &&
         state_ != ARTICORE_PARTIALLY_ENABLED)) {
      return true;
    }
    mode = mode_;
    degraded = state_ == ARTICORE_DEGRADED;
    gravity_active =
        gravity_control_.phase != ARTICORE_GRAVITY_INACTIVE;
    adaptive_cartesian_tracking =
        trajectory_control_.state == ARTICORE_MOTION_RUNNING &&
        native_cartesian_operation(trajectory_control_.operation) &&
        trajectory_control_.approach_complete;
    bimanual_active = bimanual_follow_.active;
    bimanual_leader_side = bimanual_follow_.leader_side;
    bimanual_start_positions = bimanual_follow_.start_positions;
    bimanual_follower_reference = bimanual_follow_.follower_reference;
    bimanual_follower_reference_velocity =
        bimanual_follow_.follower_reference_velocity;
    intentionally_disabled = intentionally_disabled_motors_;
  }

  if (gravity_active) {
    return run_gravity_control_cycle(now, include_grippers, error);
  }

  bool trajectory_completing = false;
  const bool trajectory_active =
      prepare_trajectory_cycle(now, trajectory_completing, error);
  if (!error.empty()) return false;
  if (!trajectory_active) {
    // Consume only the newest accepted raw command. Physical sends may take
    // most of a 2 ms cycle, but publishers can replace this pending slot while
    // the previous generation is in flight.
    consume_pending_arm_mailbox();
  }

  if (!arm_mailbox_.valid) return true;
  const bool mailbox_user_command = arm_mailbox_.user_command;
  const uint64_t mailbox_generation = arm_mailbox_.generation;
  if (arm_mailbox_.joint_position) {
    const auto command_size = mode == ARTICORE_MODE_PV
        ? arm_mailbox_.pv.size() : arm_mailbox_.mit.size();
    if (command_size != arm_mailbox_.final_positions.size() ||
        !finite(arm_mailbox_.max_reference_velocity) ||
        arm_mailbox_.max_reference_velocity < 0.0f ||
        (mode == ARTICORE_MODE_PV &&
         (!finite(arm_mailbox_.max_reference_acceleration) ||
          arm_mailbox_.max_reference_acceleration <= 0.0f ||
          arm_mailbox_.pv_reference_velocities.size() !=
              arm_mailbox_.pv.size() ||
          !finite(arm_mailbox_.pv_velocity_limit) ||
          arm_mailbox_.pv_velocity_limit < 0.0f))) {
      throw std::runtime_error(
          "ordinary joint position state is internally inconsistent");
    }
    const float command_scale = degraded ? 0.25f : 1.0f;
    if (mode == ARTICORE_MODE_PV) {
      const float period_s = 1.0f / static_cast<float>(control_hz_);
      const float maximum_velocity =
          arm_mailbox_.max_reference_velocity * command_scale;
      if (arm_mailbox_.pv_hold_confirmation_cycles.size() !=
              arm_mailbox_.pv.size() ||
          arm_mailbox_.pv_stationary_hold.size() != arm_mailbox_.pv.size()) {
        arm_mailbox_.pv_hold_confirmation_cycles.assign(
            arm_mailbox_.pv.size(), 0);
        arm_mailbox_.pv_stationary_hold.assign(arm_mailbox_.pv.size(), 0);
      }
      for (std::size_t i = 0; i < arm_mailbox_.pv.size(); ++i) {
        auto& command = arm_mailbox_.pv[i];
        const auto reference = advance_acceleration_limited_pv_reference(
            command.target_position,
            arm_mailbox_.pv_reference_velocities[i],
            arm_mailbox_.final_positions[i], maximum_velocity,
            arm_mailbox_.max_reference_acceleration, period_s);
        command.target_position = reference.position;
        arm_mailbox_.pv_reference_velocities[i] = reference.velocity;
        const float final_position = arm_mailbox_.final_positions[i];
        const bool reference_reached =
            std::abs(command.target_position - final_position) <= 1.0e-6f;
        ArticoreFeedbackStats stats{};
        ArticoreMotorState actual{};
        const bool fresh_feedback =
            backend_->get_feedback_stats(command.motor, &stats) == 0 &&
            stats.has_feedback && stats.age_ns <= feedback_max_age_ns() &&
            backend_->get_state(command.motor, &actual) == 0 &&
            actual.has_value && actual.status_code == 1 && finite(actual.pos);
        if (!reference_reached) {
          arm_mailbox_.pv_hold_confirmation_cycles[i] = 0;
          arm_mailbox_.pv_stationary_hold[i] = 0;
        } else if (fresh_feedback) {
          const float position_error = std::abs(actual.pos - final_position);
          if (arm_mailbox_.pv_stationary_hold[i] != 0) {
            if (position_error > kNativeOrdinaryPvHoldReleaseTolerance) {
              arm_mailbox_.pv_stationary_hold[i] = 0;
              arm_mailbox_.pv_hold_confirmation_cycles[i] = 0;
            }
          } else if (position_error <=
                     kNativeOrdinaryPvHoldPositionTolerance) {
            auto& confirmations =
                arm_mailbox_.pv_hold_confirmation_cycles[i];
            if (confirmations < kNativeOrdinaryPvHoldConfirmationCycles) {
              ++confirmations;
            }
            if (confirmations >= kNativeOrdinaryPvHoldConfirmationCycles) {
              arm_mailbox_.pv_stationary_hold[i] = 1;
            }
          } else {
            arm_mailbox_.pv_hold_confirmation_cycles[i] = 0;
          }
        }
        command.velocity_limit = arm_mailbox_.pv_stationary_hold[i] != 0
            ? kNativePvFinalHoldVelocityLimit
            : std::max(config_.safe_pv_velocity_limit,
                       arm_mailbox_.pv_velocity_limit);
      }
    } else {
      const float max_delta =
          arm_mailbox_.max_reference_velocity * command_scale /
          static_cast<float>(control_hz_);
      for (std::size_t i = 0; i < arm_mailbox_.mit.size(); ++i) {
        auto& command = arm_mailbox_.mit[i];
        command.target_position = advance_pv_position_reference(
            command.target_position, arm_mailbox_.final_positions[i],
            max_delta);
        command.target_velocity = 0.0f;
        command.feedforward_torque = 0.0f;
      }
    }
  }

  std::array<float, ARTICORE_PRODUCT_ARM_DOF> bimanual_leader_positions{};
  std::array<float, ARTICORE_PRODUCT_ARM_DOF> bimanual_follower_positions{};
  float bimanual_tracking_error = 0.0f;
  if (bimanual_active) {
    if (!arm_mailbox_.joint_position ||
        bimanual_start_positions.size() != 2 * ARTICORE_PRODUCT_ARM_DOF) {
      error = "bimanual follow requires an ordinary PV/MIT position command";
      return false;
    }
    const uint32_t follower_side = 1U - bimanual_leader_side;
    const float command_scale = degraded ? 0.25f : 1.0f;
    const float max_delta = arm_mailbox_.max_reference_velocity * command_scale /
                            static_cast<float>(control_hz_);
    const float maximum_velocity =
        arm_mailbox_.max_reference_velocity * command_scale;
    const float period_s = 1.0f / static_cast<float>(control_hz_);
    for (std::size_t joint = 0; joint < ARTICORE_PRODUCT_ARM_DOF; ++joint) {
      void* leader_motor =
          gravity_arms_[bimanual_leader_side].joints[joint];
      void* follower_motor = gravity_arms_[follower_side].joints[joint];
      ArticoreFeedbackStats leader_stats{};
      ArticoreFeedbackStats follower_stats{};
      ArticoreMotorState leader{};
      ArticoreMotorState follower{};
      if (backend_->get_feedback_stats(leader_motor, &leader_stats) != 0 ||
          !leader_stats.has_feedback ||
          leader_stats.age_ns > feedback_max_age_ns() ||
          backend_->get_feedback_stats(follower_motor, &follower_stats) != 0 ||
          !follower_stats.has_feedback ||
          follower_stats.age_ns > feedback_max_age_ns() ||
          backend_->get_state(leader_motor, &leader) != 0 ||
          backend_->get_state(follower_motor, &follower) != 0 ||
          !leader.has_value || !follower.has_value ||
          leader.status_code != 1 || follower.status_code != 1 ||
          !finite(leader.pos) || !finite(leader.vel) ||
          !finite(follower.pos) || !finite(follower.vel)) {
        error = motor_roles_.at(leader_motor) + " / " +
            motor_roles_.at(follower_motor) +
            ": bimanual follow requires fresh enabled feedback";
        return false;
      }

      const float leader_direction =
          gravity_arms_[bimanual_leader_side].position_directions[joint];
      const float follower_direction =
          gravity_arms_[follower_side].position_directions[joint];
      float leader_reference = 0.0f;
      if (mode == ARTICORE_MODE_PV) {
        const auto command = std::find_if(
            arm_mailbox_.pv.begin(), arm_mailbox_.pv.end(),
            [&](const ArticorePosVelCommand& value) {
              return value.motor == leader_motor;
            });
        if (command == arm_mailbox_.pv.end()) {
          error = "PV bimanual follow lost the leader product layout";
          return false;
        }
        leader_reference = command->target_position;
      } else {
        const auto command = std::find_if(
            arm_mailbox_.mit.begin(), arm_mailbox_.mit.end(),
            [&](const ArticoreMitCommand& value) {
              return value.motor == leader_motor;
            });
        if (command == arm_mailbox_.mit.end()) {
          error = "MIT bimanual follow lost the leader product layout";
          return false;
        }
        leader_reference = command->target_position;
      }
      const float desired =
          bimanual_start_positions[
              follower_side * ARTICORE_PRODUCT_ARM_DOF + joint] +
          follower_direction * leader_direction *
              (leader_reference -
               bimanual_start_positions[
                   bimanual_leader_side * ARTICORE_PRODUCT_ARM_DOF + joint]);
      const auto& limits = joint_config(follower_motor);
      if (!finite(desired) || desired < limits.soft_lower_position ||
          desired > limits.soft_upper_position) {
        const float logical_target = follower_direction * desired;
        const float logical_lower = follower_direction > 0.0f
            ? limits.soft_lower_position : -limits.soft_upper_position;
        const float logical_upper = follower_direction > 0.0f
            ? limits.soft_upper_position : -limits.soft_lower_position;
        error = motor_roles_.at(follower_motor) +
            ": bimanual target=" + std::to_string(logical_target) +
            " rad exceeds product limits=[" +
            std::to_string(logical_lower) + ", " +
            std::to_string(logical_upper) + "] rad";
        return false;
      }

      if (mode == ARTICORE_MODE_PV) {
        const auto reference = advance_acceleration_limited_pv_reference(
            bimanual_follower_reference[joint],
            bimanual_follower_reference_velocity[joint], desired,
            maximum_velocity, arm_mailbox_.max_reference_acceleration,
            period_s);
        bimanual_follower_reference[joint] = reference.position;
        bimanual_follower_reference_velocity[joint] = reference.velocity;
        const auto command = std::find_if(
            arm_mailbox_.pv.begin(), arm_mailbox_.pv.end(),
            [&](const ArticorePosVelCommand& value) {
              return value.motor == follower_motor;
            });
        if (command == arm_mailbox_.pv.end()) {
          error = "PV bimanual follow lost the follower product layout";
          return false;
        }
        const auto index = static_cast<std::size_t>(
            std::distance(arm_mailbox_.pv.begin(), command));
        command->target_position = bimanual_follower_reference[joint];
        arm_mailbox_.pv_reference_velocities[index] =
            bimanual_follower_reference_velocity[joint];
        arm_mailbox_.final_positions[index] = desired;
        arm_mailbox_.pv_hold_confirmation_cycles[index] = 0;
        arm_mailbox_.pv_stationary_hold[index] = 0;
        command->velocity_limit = std::max(
            config_.safe_pv_velocity_limit, arm_mailbox_.pv_velocity_limit);
      } else {
        bimanual_follower_reference[joint] = advance_pv_position_reference(
            bimanual_follower_reference[joint], desired, max_delta);
        const auto command = std::find_if(
            arm_mailbox_.mit.begin(), arm_mailbox_.mit.end(),
            [&](const ArticoreMitCommand& value) {
              return value.motor == follower_motor;
            });
        if (command == arm_mailbox_.mit.end()) {
          error = "MIT bimanual follow lost the follower product layout";
          return false;
        }
        const auto index = static_cast<std::size_t>(
            std::distance(arm_mailbox_.mit.begin(), command));
        command->target_position = bimanual_follower_reference[joint];
        command->target_velocity = 0.0f;
        command->feedforward_torque = 0.0f;
        arm_mailbox_.final_positions[index] = desired;
      }
      bimanual_leader_positions[joint] = leader_direction * leader.pos;
      bimanual_follower_positions[joint] = follower_direction * follower.pos;
      bimanual_tracking_error = std::max(
          bimanual_tracking_error,
          std::fabs(follower_direction * bimanual_follower_reference[joint] -
                    bimanual_follower_positions[joint]));
    }
  }

  const auto* pv_data = arm_mailbox_.pv.data();
  if (degraded && mode == ARTICORE_MODE_PV) {
    degraded_pv_commands_ = arm_mailbox_.pv;
    for (auto& command : degraded_pv_commands_) {
      command.velocity_limit *= 0.25f;
    }
    pv_data = degraded_pv_commands_.data();
  }
  ArticoreMitTorqueLimitStats torque_limit_cycle{};
  const auto* mit_data = arm_mailbox_.mit.data();
  if (mode == ARTICORE_MODE_MIT) {
    const auto* requested = &arm_mailbox_.mit;
    if (degraded) {
      degraded_mit_commands_ = arm_mailbox_.mit;
      for (auto& command : degraded_mit_commands_) {
        command.target_velocity *= 0.25f;
      }
      requested = &degraded_mit_commands_;
    }
    filtered_mit_commands_.clear();
    filtered_mit_commands_.reserve(requested->size());
    for (const auto& command : *requested) {
      if (!intentionally_disabled.count(command.motor)) {
        filtered_mit_commands_.push_back(command);
      }
    }
    if (!prepare_mit_torque_limited_commands(
            filtered_mit_commands_, mit_torque_limited_commands_,
            torque_limit_cycle, error, degraded ? 0.25f : 1.0f)) {
      return false;
    }
    mit_data = mit_torque_limited_commands_.data();
  }
  uint32_t command_count = 0;
  if (mode == ARTICORE_MODE_PV) {
    filtered_pv_commands_.clear();
    const auto& source = degraded ? degraded_pv_commands_ : arm_mailbox_.pv;
    filtered_pv_commands_.reserve(source.size());
    for (const auto& command : source) {
      if (!intentionally_disabled.count(command.motor)) {
        filtered_pv_commands_.push_back(command);
      }
    }
    pv_data = filtered_pv_commands_.data();
    command_count = static_cast<uint32_t>(filtered_pv_commands_.size());
  } else {
    mit_data = mit_torque_limited_commands_.data();
    command_count = static_cast<uint32_t>(mit_torque_limited_commands_.size());
  }

  std::vector<ArticoreMitCommand> gripper_commands;
  std::vector<ArticoreMitCommand> combined_mit;
  const ArticoreMitCommand* mit_send_data = mit_data;
  uint32_t mit_send_count = command_count;
  if (include_grippers && mode == ARTICORE_MODE_MIT) {
    if (!prepare_gripper_commands_locked(now, gripper_commands, error)) {
      return false;
    }
    if (!gripper_commands.empty()) {
      combined_mit.reserve(gripper_commands.size() + command_count);
      // Put grippers first so they no longer sit behind a completed 14-axis
      // transaction and a second ControllerGroup dispatch barrier. The group
      // still preserves one atomic cross-channel generation.
      combined_mit.insert(combined_mit.end(), gripper_commands.begin(),
                          gripper_commands.end());
      combined_mit.insert(combined_mit.end(),
                          mit_torque_limited_commands_.begin(),
                          mit_torque_limited_commands_.end());
      mit_send_data = combined_mit.data();
      mit_send_count = static_cast<uint32_t>(combined_mit.size());
    }
  }

  const int32_t result = mode == ARTICORE_MODE_PV
      ? (command_count == 0 ? 0 : backend_->send_pos_vel(
            controller_group_, pv_data, command_count))
      : (mit_send_count == 0 ? 0 : backend_->send_mit(
            controller_group_, mit_send_data, mit_send_count));
  if (result != 0) {
    error = motor_error(
        mode == ARTICORE_MODE_PV
            ? "ControllerGroup PV send failed"
            : (gripper_commands.empty()
                   ? "ControllerGroup MIT send failed"
                   : "combined arm/gripper ControllerGroup MIT send failed"));
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    ++consecutive_send_failures_;
    for (uint8_t side = 0; side < 2; ++side) {
      if (active_sides_[side]) set_side_error_locked(side, error, true);
    }
    return false;
  }

  float cartesian_tracking_error = 0.0f;
  void* cartesian_tracking_worst_motor = nullptr;
  bool cartesian_tracking_feedback_valid =
      adaptive_cartesian_tracking && mode == ARTICORE_MODE_PV &&
      command_count > 0;
  if (cartesian_tracking_feedback_valid) {
    for (uint32_t index = 0; index < command_count; ++index) {
      ArticoreFeedbackStats stats{};
      ArticoreMotorState actual{};
      if (backend_->get_feedback_stats(pv_data[index].motor, &stats) != 0 ||
          !stats.has_feedback || stats.age_ns > feedback_max_age_ns() ||
          backend_->get_state(pv_data[index].motor, &actual) != 0 ||
          !actual.has_value || actual.status_code != 1 ||
          !finite(actual.pos)) {
        cartesian_tracking_feedback_valid = false;
        break;
      }
      const float position_error =
          std::abs(actual.pos - pv_data[index].target_position);
      if (position_error > cartesian_tracking_error) {
        cartesian_tracking_error = position_error;
        cartesian_tracking_worst_motor = pv_data[index].motor;
      }
    }
  }

  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    consecutive_send_failures_ = 0;
    if (mode == ARTICORE_MODE_PV) {
      last_sent_pv_.assign(pv_data, pv_data + command_count);
      last_sent_mit_.clear();
      if (adaptive_cartesian_tracking &&
          trajectory_control_.state == ARTICORE_MOTION_RUNNING &&
          native_cartesian_operation(trajectory_control_.operation) &&
          trajectory_control_.approach_complete) {
        trajectory_control_.tracking_position_error =
            cartesian_tracking_error;
        trajectory_control_.tracking_feedback_valid =
            cartesian_tracking_feedback_valid;
        trajectory_control_.tracking_worst_role.clear();
        if (cartesian_tracking_feedback_valid &&
            cartesian_tracking_worst_motor) {
          const auto found = std::find_if(
              trajectory_control_.joints.begin(),
              trajectory_control_.joints.end(),
              [&](const NativeTrajectoryJoint& joint) {
                return joint.motor == cartesian_tracking_worst_motor;
              });
          if (found != trajectory_control_.joints.end()) {
            trajectory_control_.tracking_worst_role = found->role;
          }
        }
      }
    } else {
      // Safe arm hold must remain independent from product gripper policy.
      last_sent_mit_.assign(mit_data, mit_data + command_count);
      last_sent_pv_.clear();
      const auto activation_count =
          mit_torque_limit_stats_.torque_limit_activation_count;
      mit_torque_limit_stats_ = torque_limit_cycle;
      mit_torque_limit_stats_.torque_limit_activation_count =
          activation_count +
          (torque_limit_cycle.torque_limited_joint_mask != 0 ? 1 : 0);
    }
    for (uint8_t side = 0; side < 2; ++side) {
      if (!active_sides_[side]) continue;
      sides_[side].send_failures = 0;
      sides_[side].healthy = true;
    }
    if (bimanual_active && bimanual_follow_.active) {
      const uint32_t follower_side = 1U - bimanual_leader_side;
      bimanual_follow_.follower_reference = bimanual_follower_reference;
      bimanual_follow_.follower_reference_velocity =
          bimanual_follower_reference_velocity;
      ++bimanual_follow_.status.control_cycles;
      bimanual_follow_.status.phase = ARTICORE_BIMANUAL_FOLLOW_ACTIVE;
      bimanual_follow_.status.active = 1;
      bimanual_follow_.status.transition_progress = 1.0f;
      bimanual_follow_.status.leader_side = bimanual_leader_side;
      bimanual_follow_.status.follower_side = follower_side;
      bimanual_follow_.status.max_tracking_error = bimanual_tracking_error;
      for (std::size_t joint = 0; joint < ARTICORE_PRODUCT_ARM_DOF; ++joint) {
        bimanual_follow_.status.leader_positions[joint] =
            bimanual_leader_positions[joint];
        bimanual_follow_.status.follower_target_positions[joint] =
            gravity_arms_[follower_side].position_directions[joint] *
            bimanual_follower_reference[joint];
      }
    }
    if (mailbox_user_command) {
      has_successful_command_ = true;
      first_command_accepted_ = false;
      if (state_ == ARTICORE_ENABLED) state_ = ARTICORE_RUNNING;
      if (mailbox_generation > arm_mailbox_.sent_generation) {
        arm_mailbox_.sent_generation = mailbox_generation;
        last_successful_command_ = now;
      }
    }
  }
  commit_gripper_commands_sent(gripper_commands, now);
  record_control_trace(
      now, mode, pv_data,
      mode == ARTICORE_MODE_PV ? command_count : 0U,
      mit_data, mode == ARTICORE_MODE_MIT ? command_count : 0U);
  if (trajectory_completing) update_trajectory_completion(now);
  return true;
}

void SafetyRuntime::record_control_trace(
    Clock::time_point now, ArticoreControlMode mode,
    const ArticorePosVelCommand* pv_commands, uint32_t pv_count,
    const ArticoreMitCommand* mit_commands, uint32_t mit_count) {
  if (control_trace_path_.empty() ||
      control_trace_.size() >= control_trace_.capacity()) {
    return;
  }

  ControlTraceSample sample;
  sample.sequence = ++control_trace_sequence_;
  sample.timestamp_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          now.time_since_epoch()).count());
  sample.planned_positions.fill(std::numeric_limits<float>::quiet_NaN());
  sample.planned_velocities.fill(std::numeric_limits<float>::quiet_NaN());
  sample.command_positions.fill(std::numeric_limits<float>::quiet_NaN());
  sample.pv_velocity_limits.fill(std::numeric_limits<float>::quiet_NaN());
  sample.actual_positions.fill(std::numeric_limits<float>::quiet_NaN());
  sample.actual_velocities.fill(std::numeric_limits<float>::quiet_NaN());

  std::vector<NativeTrajectoryJoint> joints;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    sample.runtime_state = state_;
    sample.motion_state = trajectory_control_.state;
    sample.motion_id = trajectory_control_.id;
    sample.progress = trajectory_control_.duration_s > 0.0
        ? static_cast<float>(std::clamp(
              trajectory_control_.elapsed_s / trajectory_control_.duration_s,
              0.0, 1.0))
        : 0.0f;
    sample.tracking_time_scale = trajectory_control_.tracking_time_scale;
    sample.tracking_position_error =
        trajectory_control_.tracking_position_error;
    joints = trajectory_control_.joints;
    const auto planned = trajectory_sample_locked(now);
    const auto planned_count = std::min<std::size_t>(
        ARTICORE_PRODUCT_DUAL_ARM_DOF,
        std::min(planned.positions.size(), planned.velocities.size()));
    for (std::size_t index = 0; index < planned_count; ++index) {
      sample.planned_positions[index] = planned.positions[index];
      sample.planned_velocities[index] = planned.velocities[index];
      sample.planned_valid_mask |= uint32_t{1} << index;
    }
  }

  const auto joint_count = std::min<std::size_t>(
      ARTICORE_PRODUCT_DUAL_ARM_DOF, joints.size());
  for (std::size_t index = 0; index < joint_count; ++index) {
    const auto& joint = joints[index];
    if (mode == ARTICORE_MODE_PV && pv_commands && pv_count > 0) {
      const auto command = std::find_if(
          pv_commands, pv_commands + pv_count,
          [&](const ArticorePosVelCommand& value) {
            return value.motor == joint.motor;
          });
      if (command != pv_commands + pv_count) {
        sample.command_positions[index] =
            joint.direction * command->target_position;
        sample.pv_velocity_limits[index] = command->velocity_limit;
        sample.command_valid_mask |= uint32_t{1} << index;
      }
    } else if (mode == ARTICORE_MODE_MIT && mit_commands && mit_count > 0) {
      const auto command = std::find_if(
          mit_commands, mit_commands + mit_count,
          [&](const ArticoreMitCommand& value) {
            return value.motor == joint.motor;
          });
      if (command != mit_commands + mit_count) {
        sample.command_positions[index] =
            joint.direction * command->target_position;
        sample.command_valid_mask |= uint32_t{1} << index;
      }
    }
    ArticoreMotorState actual{};
    ArticoreFeedbackStats stats{};
    if (backend_->get_state(joint.motor, &actual) == 0 && actual.has_value &&
        backend_->get_feedback_stats(joint.motor, &stats) == 0 &&
        stats.has_feedback && finite(actual.pos) && finite(actual.vel)) {
      sample.actual_positions[index] = joint.direction * actual.pos;
      sample.actual_velocities[index] =
          joint.direction * actual.vel * joint.velocity_feedback_scale;
      sample.actual_valid_mask |= uint32_t{1} << index;
    }
  }
  control_trace_.push_back(std::move(sample));
}

}  // namespace articore

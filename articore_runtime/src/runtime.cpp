#include "runtime.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace articore {
namespace {

template <std::size_t N>
void copy_text(char (&destination)[N], const std::string& source) {
  std::memset(destination, 0, N);
  std::memcpy(destination, source.data(), std::min(source.size(), N - 1));
}

uint64_t age_ns(std::chrono::steady_clock::time_point value,
                bool available,
                std::chrono::steady_clock::time_point now) {
  if (!available) return std::numeric_limits<uint64_t>::max();
  const auto age = std::chrono::duration_cast<std::chrono::nanoseconds>(now - value);
  return static_cast<uint64_t>(std::max<int64_t>(0, age.count()));
}

}  // namespace

bool SafetyRuntime::finite(float value) {
  return std::isfinite(static_cast<double>(value));
}

float SafetyRuntime::clamp_opening(float opening) {
  return std::clamp(opening, 0.0f, 1000.0f);
}

float SafetyRuntime::opening_to_position(const MotorRecord& motor,
                                         float opening) {
  const auto ratio = clamp_opening(opening) / 1000.0f;
  return motor.descriptor.closed_position +
         (motor.descriptor.open_position - motor.descriptor.closed_position) * ratio;
}

float SafetyRuntime::position_to_opening(const MotorRecord& motor,
                                         float position) {
  const auto range = motor.descriptor.open_position -
                     motor.descriptor.closed_position;
  if (range == 0.0f) return 0.0f;
  return clamp_opening(
      1000.0f * (position - motor.descriptor.closed_position) / range);
}

SafetyRuntime::SafetyRuntime(ArticoreRuntimeConfig config,
                             ArticoreMotorApi api,
                             void* controller_group,
                             void* left_controller,
                             void* right_controller,
                             std::vector<ArticoreMotorDescriptor> motors)
    : config_(config), api_(api), controller_group_(controller_group) {
  controllers_[0] = left_controller;
  controllers_[1] = right_controller;
  if (!controller_group_) {
    throw std::invalid_argument("Articore runtime requires a controller group");
  }
  if (!api_.group_send_pos_vel || !api_.group_send_mit ||
      !api_.controller_disable_all || !api_.controller_request_feedback_all_ex ||
      !api_.motor_get_state || !api_.motor_get_feedback_stats ||
      !api_.last_error_message || !api_.motor_disable) {
    throw std::invalid_argument("Articore runtime motor API is incomplete");
  }
  if (config_.control_hz == 0 || config_.command_timeout_ms == 0 ||
      config_.enable_grace_ms == 0 || config_.safe_hold_hz == 0 ||
      config_.feedback_check_hz == 0 || config_.feedback_failure_threshold == 0 ||
      config_.feedback_max_age_ms == 0 ||
      config_.safe_hold_failure_threshold == 0 ||
      config_.disable_feedback_timeout_ms == 0 ||
      config_.gripper_control_hz == 0 ||
      (config_.gripper_fault_action != ARTICORE_GRIPPER_FAULT_HOLD &&
       config_.gripper_fault_action != ARTICORE_GRIPPER_FAULT_DISABLE) ||
      !finite(config_.safe_pv_velocity_limit) ||
      config_.safe_pv_velocity_limit <= 0.0f) {
    throw std::invalid_argument("Articore runtime configuration contains invalid values");
  }
  if (motors.empty() || motors.size() > 32) {
    throw std::invalid_argument("Articore runtime requires 1..32 motors");
  }
  std::set<void*> unique;
  motors_.reserve(motors.size());
  for (const auto& motor : motors) {
    if (!motor.motor || motor.side > 1 || motor.name[0] == '\0') {
      throw std::invalid_argument("invalid Articore motor descriptor");
    }
    if (!unique.insert(motor.motor).second) {
      throw std::invalid_argument("duplicate Articore motor handle");
    }
    if (!finite(motor.safe_kp) || !finite(motor.safe_kd) ||
        motor.safe_kp < 0.0f || motor.safe_kd < 0.0f ||
        !finite(motor.overload_torque) || motor.overload_torque < 0.0f ||
        !finite(motor.retreat_distance) || motor.retreat_distance < 0.0f ||
        !finite(motor.contact_torque) || motor.contact_torque < 0.0f ||
        !finite(motor.stall_movement) || motor.stall_movement < 0.0f ||
        !finite(motor.min_position_error) || motor.min_position_error < 0.0f ||
        !finite(motor.hold_offset) || motor.hold_offset < 0.0f ||
        !finite(motor.open_position) || !finite(motor.closed_position) ||
        !finite(motor.normal_kp) || motor.normal_kp < 0.0f ||
        !finite(motor.normal_kd) || motor.normal_kd < 0.0f ||
        !finite(motor.close_speed) || motor.close_speed < 0.0f ||
        !finite(motor.closing_direction) || std::isnan(motor.lower_position) ||
        std::isnan(motor.upper_position) ||
        motor.lower_position > motor.upper_position) {
      throw std::invalid_argument("invalid safe-hold motor descriptor");
    }
    active_sides_[motor.side] = true;
    const std::string role(motor.name);
    if (!presence_.emplace(role, ARTICORE_PRESENT).second) {
      throw std::invalid_argument("duplicate Articore motor role: " + role);
    }
    motor_roles_.emplace(motor.motor, role);
    MotorRecord record;
    record.descriptor = motor;
    if (motor.is_gripper) {
      if (motor.open_position == motor.closed_position ||
          motor.normal_kp <= 0.0f || motor.close_speed <= 0.0f) {
        throw std::invalid_argument("invalid active gripper descriptor");
      }
      record.gripper_state = ARTICORE_GRIPPER_DISABLED;
    }
    motors_.push_back(std::move(record));
  }
  if ((!active_sides_[0] && !active_sides_[1]) ||
      (active_sides_[0] && !controllers_[0]) ||
      (active_sides_[1] && !controllers_[1])) {
    throw std::invalid_argument(
        "Articore runtime requires a controller for every active side");
  }
  const auto arm_count = static_cast<std::size_t>(std::count_if(
      motors_.begin(), motors_.end(), [](const MotorRecord& motor) {
        return motor.descriptor.is_gripper == 0;
      }));
  const auto gripper_count = motors_.size() - arm_count;
  safe_pv_.reserve(arm_count);
  safe_mit_.reserve(arm_count);
  safe_grippers_.reserve(gripper_count);
  worker_ = std::thread([this] { worker_loop(); });
}

SafetyRuntime::~SafetyRuntime() {
  close();
}

void SafetyRuntime::connect() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_ != ARTICORE_DISCONNECTED) return;
  state_ = ARTICORE_READY;
  fault_latched_ = false;
  disable_confirmed_ = true;
  fault_reason_.clear();
  motor_faults_.clear();
  unconfirmed_disable_.clear();
  for (uint8_t side = 0; side < 2; ++side) {
    sides_[side] = SideHealth{};
    sides_[side].connected = active_sides_[side];
    sides_[side].healthy = active_sides_[side];
  }
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
    ArticoreControlMode mode) {
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
        !finite(state.torq) || state.status_code == 0 ||
        state.status_code > 1) {
      throw std::runtime_error(
          name + ": fresh enabled feedback is required before enable");
    }
    validate_position_velocity_torque(
        motor.descriptor.motor, state.pos, 0.0f, 0.0f);
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

void SafetyRuntime::trim_trajectory_history_locked() {
  constexpr std::size_t kMaximumTrajectoryHistory = 64;
  while (trajectory_history_order_.size() > kMaximumTrajectoryHistory) {
    trajectory_history_.erase(trajectory_history_order_.front());
    trajectory_history_order_.pop_front();
  }
}

void SafetyRuntime::finish_trajectory_locked(
    uint64_t id, ArticoreTrajectoryStatus status, const std::string& error,
    Clock::time_point now) {
  if (!active_trajectory_ || active_trajectory_->id != id) return;
  active_trajectory_->status = status;
  active_trajectory_->error = error;
  active_trajectory_->finished_at = now;
  trajectory_history_[id] = *active_trajectory_;
  trajectory_history_order_.push_back(id);
  active_trajectory_.reset();
  trim_trajectory_history_locked();
  trajectory_cv_.notify_all();
}

void SafetyRuntime::cancel_active_trajectory_locked(
    ArticoreTrajectoryStatus status, const std::string& error) {
  if (!active_trajectory_) return;
  finish_trajectory_locked(active_trajectory_->id, status, error, Clock::now());
}

uint64_t SafetyRuntime::start_joint_trajectory(
    const ArticoreJointTrajectoryTarget* targets, uint32_t count,
    ArticoreTrajectoryProfile profile) {
  if (!targets || count == 0) {
    throw std::invalid_argument("joint trajectory is empty");
  }
  if (profile != ARTICORE_TRAJECTORY_MIN_JERK &&
      profile != ARTICORE_TRAJECTORY_LINEAR) {
    throw std::invalid_argument("unsupported trajectory profile");
  }
  const auto expected = static_cast<uint32_t>(std::count_if(
      motors_.begin(), motors_.end(), [](const MotorRecord& motor) {
        return motor.descriptor.is_gripper == 0;
      }));
  if (count != expected) {
    throw std::invalid_argument(
        "trajectory must contain the complete fixed arm layout");
  }

  std::lock_guard<std::mutex> command_lock(command_mutex_);
  uint64_t trajectory_id = 0;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    require_state_for_command();
  }

  TrajectoryRecord trajectory;
  trajectory.profile = profile;
  trajectory.start_time = Clock::now();
  trajectory.joints.reserve(count);
  double duration_seconds = 0.0;
  std::set<void*> unique;
  for (uint32_t i = 0; i < count; ++i) {
    const auto& target = targets[i];
    if (!target.motor || !finite(target.target_position) ||
        !finite(target.velocity_limit) || target.velocity_limit <= 0.0f ||
        !unique.insert(target.motor).second) {
      throw std::invalid_argument("trajectory contains invalid targets");
    }
    const auto motor = std::find_if(
        motors_.begin(), motors_.end(), [&](const MotorRecord& record) {
          return !record.descriptor.is_gripper &&
                 record.descriptor.motor == target.motor;
        });
    if (motor == motors_.end()) {
      throw std::invalid_argument("trajectory contains an unexpected motor");
    }
    const auto& limits = joint_config(target.motor);
    if (target.velocity_limit > limits.velocity_limit) {
      throw std::invalid_argument("trajectory velocity exceeds joint limit");
    }
    validate_position_velocity_torque(
        target.motor, target.target_position, target.velocity_limit, 0.0f);

    ArticoreFeedbackStats stats{};
    ArticoreMotorState state{};
    if (api_.motor_get_feedback_stats(target.motor, &stats) != 0 ||
        !stats.has_feedback ||
        stats.age_ns > static_cast<uint64_t>(config_.feedback_max_age_ms) *
                           1'000'000ULL ||
        api_.motor_get_state(target.motor, &state) != 0 || !state.has_value ||
        !finite(state.pos) || !finite(state.vel) || !finite(state.torq) ||
        state.status_code != 1) {
      throw std::runtime_error(
          std::string(motor->descriptor.name) +
          ": complete fresh enabled feedback is required for trajectory");
    }
    validate_position_velocity_torque(
        target.motor, state.pos, state.vel, state.torq);
    const auto distance = std::abs(
        static_cast<double>(target.target_position) - state.pos);
    const auto factor = profile == ARTICORE_TRAJECTORY_MIN_JERK ? 1.875 : 1.0;
    duration_seconds = std::max(
        duration_seconds, factor * distance / target.velocity_limit);
    trajectory.joints.push_back(TrajectoryJoint{
        target.motor, state.pos, target.target_position, target.velocity_limit});
  }

  const auto minimum_duration =
      std::chrono::duration<double>(1.0 / config_.control_hz);
  trajectory.duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::max(std::chrono::duration<double>(duration_seconds), minimum_duration));
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    require_state_for_command();
    cancel_active_trajectory_locked(
        ARTICORE_TRAJECTORY_PREEMPTED, "preempted by a newer trajectory");
    trajectory.id = next_trajectory_id_++;
    if (trajectory.id == 0) trajectory.id = next_trajectory_id_++;
    trajectory_id = trajectory.id;
    active_trajectory_ = std::move(trajectory);
  }
  wakeup_.notify_all();
  return trajectory_id;
}

ArticoreTrajectoryInfo SafetyRuntime::trajectory_info(
    uint64_t trajectory_id) const {
  if (trajectory_id == 0) {
    throw std::invalid_argument("trajectory id must be nonzero");
  }
  const auto now = Clock::now();
  std::lock_guard<std::mutex> lock(state_mutex_);
  const TrajectoryRecord* record = nullptr;
  if (active_trajectory_ && active_trajectory_->id == trajectory_id) {
    record = &*active_trajectory_;
  } else {
    const auto found = trajectory_history_.find(trajectory_id);
    if (found != trajectory_history_.end()) record = &found->second;
  }
  if (!record) throw std::invalid_argument("unknown trajectory id");
  ArticoreTrajectoryInfo info{};
  info.struct_size = sizeof(info);
  info.trajectory_id = record->id;
  info.status = record->status;
  info.profile = record->profile;
  info.duration_ns = static_cast<uint64_t>(record->duration.count());
  const auto end = record->status == ARTICORE_TRAJECTORY_RUNNING
      ? now : record->finished_at;
  info.elapsed_ns = age_ns(record->start_time, true, end);
  if (info.duration_ns > 0) {
    info.elapsed_ns = std::min(info.elapsed_ns, info.duration_ns);
  }
  copy_text(info.error, record->error);
  return info;
}

ArticoreTrajectoryInfo SafetyRuntime::wait_trajectory(
    uint64_t trajectory_id, std::chrono::milliseconds timeout) {
  if (trajectory_id == 0) {
    throw std::invalid_argument("trajectory id must be nonzero");
  }
  std::unique_lock<std::mutex> lock(state_mutex_);
  const auto known = [&] {
    return (active_trajectory_ && active_trajectory_->id == trajectory_id) ||
           trajectory_history_.find(trajectory_id) != trajectory_history_.end();
  };
  if (!known()) throw std::invalid_argument("unknown trajectory id");
  const auto terminal = [&] {
    return trajectory_history_.find(trajectory_id) != trajectory_history_.end();
  };
  if (!terminal() && !trajectory_cv_.wait_for(lock, timeout, terminal)) {
    throw std::runtime_error("trajectory wait timed out");
  }
  lock.unlock();
  return trajectory_info(trajectory_id);
}

void SafetyRuntime::declare_motor_presence(const std::string& role,
                                           ArticorePresenceState state) {
  if (role.empty()) throw std::invalid_argument("motor role is empty");
  if (state != ARTICORE_NOT_INSTALLED && state != ARTICORE_PRESENT) {
    throw std::invalid_argument(
        "only NotInstalled or Present may be declared before connect");
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_ != ARTICORE_DISCONNECTED) {
    throw std::runtime_error(
        "motor presence is fixed after the runtime connects");
  }
  const auto found = presence_.find(role);
  if (found != presence_.end() && found->second == ARTICORE_PRESENT &&
      state != ARTICORE_PRESENT) {
    throw std::invalid_argument(
        "an active motor descriptor cannot be declared NotInstalled: " + role);
  }
  if (found == presence_.end() && state == ARTICORE_PRESENT) {
    throw std::invalid_argument(
        "Present motor role requires an active motor descriptor: " + role);
  }
  presence_[role] = state;
}

ArticorePresenceState SafetyRuntime::motor_presence(
    const std::string& role) const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto found = presence_.find(role);
  if (found == presence_.end()) {
    throw std::invalid_argument("unknown motor role: " + role);
  }
  return found->second;
}

uint64_t SafetyRuntime::active_capabilities() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  uint64_t capabilities = 0;
  for (const auto& motor : motors_) {
    const auto found = motor_roles_.find(motor.descriptor.motor);
    if (found == motor_roles_.end()) continue;
    const auto presence = presence_.find(found->second);
    if (presence == presence_.end() ||
        presence->second == ARTICORE_NOT_INSTALLED) {
      continue;
    }
    if (motor.descriptor.is_gripper) {
      capabilities |= motor.descriptor.side == 0
          ? ARTICORE_ACTIVE_GRIPPER_SIDE_0
          : ARTICORE_ACTIVE_GRIPPER_SIDE_1;
    } else {
      capabilities |= motor.descriptor.side == 0
          ? ARTICORE_ACTIVE_ARM_SIDE_0
          : ARTICORE_ACTIVE_ARM_SIDE_1;
    }
  }
  return capabilities;
}

void SafetyRuntime::mark_motor_faulted(void* motor) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto role = motor_roles_.find(motor);
  if (role != motor_roles_.end()) presence_[role->second] = ARTICORE_FAULTED;
}

void SafetyRuntime::enable(ArticoreControlMode mode) {
  if (mode != ARTICORE_MODE_PV && mode != ARTICORE_MODE_MIT) {
    throw std::invalid_argument("control mode must be PV or MIT");
  }
  {
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (state_ != ARTICORE_READY || fault_latched_ || hardware_transition_) {
        throw std::runtime_error("Articore runtime can only enable from READY");
      }
    }
    initialize_arm_mailbox_from_feedback(mode);
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ != ARTICORE_READY || fault_latched_ || hardware_transition_) {
      throw std::runtime_error("runtime state changed while initializing enable target");
    }
    mode_ = mode;
    state_ = ARTICORE_ENABLED;
    disable_confirmed_ = false;
    has_successful_command_ = false;
    enabled_at_ = Clock::now();
    last_successful_command_ = {};
    consecutive_send_failures_ = 0;
    consecutive_feedback_failures_ = 0;
    consecutive_hold_failures_ = 0;
    safe_pv_.clear();
    safe_mit_.clear();
    safe_grippers_.clear();
    cancel_active_trajectory_locked(ARTICORE_TRAJECTORY_CANCELED,
                                    "runtime re-enabled");
    for (auto& motor : motors_) {
      motor.retreat_active = false;
      motor.protective_target_active = false;
      motor.contact_started_at = {};
      motor.overload_started_at = {};
      motor.last_retreat_at = {};
      motor.motion_samples.clear();
      motor.gripper_state = motor.descriptor.is_gripper
          ? ARTICORE_GRIPPER_IDLE
          : ARTICORE_GRIPPER_DISABLED;
      motor.has_gripper_target = false;
      motor.contact_detected = false;
      motor.stalled = false;
      motor.overload = false;
      motor.gripper_fault_reason.clear();
      motor.last_gripper_update = {};
    }
    next_feedback_check_ = enabled_at_;
    next_gripper_control_ = enabled_at_;
    next_control_tick_ = enabled_at_;
  }
  seed_gripper_targets_from_feedback();
  wakeup_.notify_all();
}

void SafetyRuntime::require_state_for_command() const {
  if (fault_latched_ || hardware_transition_ ||
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
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    require_state_for_command();
    if (mode_ != ARTICORE_MODE_PV) {
      throw std::runtime_error("cannot submit PV while runtime mode is MIT");
    }
    cancel_active_trajectory_locked(
        ARTICORE_TRAJECTORY_PREEMPTED, "preempted by direct PV command");
    arm_mailbox_.valid = true;
    arm_mailbox_.user_command = true;
    ++arm_mailbox_.generation;
    arm_mailbox_.submitted_at = Clock::now();
    arm_mailbox_.pv.assign(commands, commands + count);
    arm_mailbox_.mit.clear();
  }
  wakeup_.notify_all();
}

void SafetyRuntime::submit_mit(const ArticoreMitCommand* commands, uint32_t count) {
  if (!commands || count == 0) throw std::invalid_argument("MIT command is empty");
  for (uint32_t i = 0; i < count; ++i) {
    const auto& command = commands[i];
    if (!command.motor || !finite(command.target_position) ||
        !finite(command.target_velocity) || !finite(command.stiffness) ||
        !finite(command.damping) || !finite(command.feedforward_torque) ||
        command.stiffness < 0.0f || command.damping < 0.0f) {
      throw std::invalid_argument("MIT command contains invalid values");
    }
    validate_position_velocity_torque(
        command.motor, command.target_position, command.target_velocity,
        command.feedforward_torque);
  }
  validate_motor_set(commands, count, false);

  {
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    require_state_for_command();
    if (mode_ != ARTICORE_MODE_MIT) {
      throw std::runtime_error("cannot submit MIT while runtime mode is PV");
    }
    cancel_active_trajectory_locked(
        ARTICORE_TRAJECTORY_PREEMPTED, "preempted by direct MIT command");
    arm_mailbox_.valid = true;
    arm_mailbox_.user_command = true;
    ++arm_mailbox_.generation;
    arm_mailbox_.submitted_at = Clock::now();
    arm_mailbox_.mit.assign(commands, commands + count);
    arm_mailbox_.pv.clear();
  }
  wakeup_.notify_all();
}

void SafetyRuntime::submit_gripper_mit(const ArticoreMitCommand* commands,
                                       uint32_t count) {
  if (!commands || count == 0) throw std::invalid_argument("gripper command is empty");
  for (uint32_t i = 0; i < count; ++i) {
    const auto& command = commands[i];
    if (!command.motor || !finite(command.target_position) ||
        !finite(command.target_velocity) || !finite(command.stiffness) ||
        !finite(command.damping) || command.stiffness < 0.0f ||
        command.damping < 0.0f || !finite(command.feedforward_torque)) {
      throw std::invalid_argument("invalid gripper MIT command");
    }
  }
  validate_motor_set(commands, count, true);
  std::string error;
  bool send_failed = false;
  {
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      require_state_for_command();
    }
    if (api_.group_send_mit(controller_group_, commands, count) != 0) {
      error = motor_error("gripper group send failed");
      send_failed = true;
    } else {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      for (uint32_t i = 0; i < count; ++i) {
        auto safe = commands[i];
        safe.target_velocity = 0.0f;
        safe.feedforward_torque = 0.0f;
        const auto descriptor = std::find_if(
            motors_.begin(), motors_.end(),
            [&](const MotorRecord& motor) { return motor.descriptor.motor == safe.motor; });
        safe.stiffness = descriptor->descriptor.safe_kp;
        safe.damping = descriptor->descriptor.safe_kd;
        descriptor->retreat_active = false;
        descriptor->protective_target_active = false;
        descriptor->contact_started_at = {};
        descriptor->overload_started_at = {};
        descriptor->last_retreat_at = {};
        descriptor->motion_samples.clear();
        auto existing = std::find_if(
            safe_grippers_.begin(), safe_grippers_.end(),
            [&](const ArticoreMitCommand& value) { return value.motor == safe.motor; });
        if (existing == safe_grippers_.end()) safe_grippers_.push_back(safe);
        else *existing = safe;
      }
    }
  }
  if (send_failed) {
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      ++consecutive_send_failures_;
    }
    std::string hold_error;
    if (!enter_safe_hold_from_feedback("gripper send failed: " + error,
                                       hold_error)) {
      enter_fault("gripper send failed: " + error +
                  "; current-position hold unavailable: " + hold_error);
    }
  }
  wakeup_.notify_all();
  if (!error.empty()) throw std::runtime_error(error);
}

void SafetyRuntime::set_gripper_openings(const ArticoreGripperTarget* targets,
                                         uint32_t count) {
  if (!targets || count == 0) {
    throw std::invalid_argument("gripper opening target is empty");
  }
  const auto expected = static_cast<uint32_t>(std::count_if(
      motors_.begin(), motors_.end(), [](const MotorRecord& motor) {
        return motor.descriptor.is_gripper != 0;
      }));
  if (count != expected) {
    throw std::invalid_argument(
        "gripper target must contain every active product gripper");
  }
  for (uint32_t i = 0; i < count; ++i) {
    if (!targets[i].motor || !finite(targets[i].opening)) {
      throw std::invalid_argument("gripper opening must be finite");
    }
    for (uint32_t previous = 0; previous < i; ++previous) {
      if (targets[previous].motor == targets[i].motor) {
        throw std::invalid_argument("gripper target contains duplicate motors");
      }
    }
  }

  std::lock_guard<std::mutex> command_lock(command_mutex_);
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  require_state_for_command();
  for (uint32_t i = 0; i < count; ++i) {
    const auto found = std::find_if(
        motors_.begin(), motors_.end(), [&](const MotorRecord& motor) {
          return motor.descriptor.is_gripper &&
                 motor.descriptor.motor == targets[i].motor;
        });
    if (found == motors_.end()) {
      throw std::invalid_argument("gripper target contains an unexpected motor");
    }
    const auto opening = clamp_opening(targets[i].opening);
    const auto position = opening_to_position(*found, opening);
    const bool opens = found->has_gripper_target &&
        (position - found->requested_position) *
            found->descriptor.closing_direction < 0.0f;
    found->requested_opening = opening;
    found->requested_position = position;
    found->has_gripper_target = true;
    found->gripper_state = ARTICORE_GRIPPER_MOVING;
    found->gripper_fault_reason.clear();
    if (opens) {
      found->command_position = position;
    }
    found->contact_detected = false;
    found->stalled = false;
    found->overload = false;
    found->protective_target_active = false;
    found->retreat_active = false;
    found->contact_started_at = {};
    found->overload_started_at = {};
    found->motion_samples.clear();
  }
  next_gripper_control_ = Clock::now();
  wakeup_.notify_all();
}

void SafetyRuntime::report_feedback_failure(uint8_t side,
                                            const std::string& reason) {
  if (side > 1 || !active_sides_[side]) {
    throw std::invalid_argument("feedback side is not active");
  }
  bool should_hold = false;
  bool should_fault = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ++consecutive_feedback_failures_;
    ++sides_[side].feedback_failures;
    sides_[side].healthy = false;
    sides_[side].last_error = reason;
    if (state_ == ARTICORE_SAFE_HOLD) {
      should_fault = true;
    } else if (state_ == ARTICORE_RUNNING &&
               consecutive_feedback_failures_ >= config_.feedback_failure_threshold) {
      should_hold = true;
    }
  }
  if (should_fault) enter_fault("feedback failed during safe hold: " + reason);
  if (should_hold) {
    std::string hold_error;
    if (!enter_safe_hold_from_feedback(
            "consecutive feedback failures: " + reason, hold_error)) {
      enter_fault("consecutive feedback failures: " + reason +
                  "; current-position hold unavailable: " + hold_error);
    } else {
      wakeup_.notify_all();
    }
  }
}

std::string SafetyRuntime::motor_error(const std::string& fallback) const {
  const char* message = api_.last_error_message();
  return message && message[0] ? std::string(message) : fallback;
}

void SafetyRuntime::set_side_error_locked(uint8_t side,
                                          const std::string& error,
                                          bool send_failure) {
  auto& health = sides_[side];
  health.healthy = false;
  health.last_error = error;
  if (send_failure) ++health.send_failures;
  else ++health.feedback_failures;
}

void SafetyRuntime::seed_gripper_targets_from_feedback() {
  std::vector<ArticoreMitCommand> seeded;
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  for (auto& motor : motors_) {
    if (!motor.descriptor.is_gripper) continue;
    ArticoreMotorState state{};
    if (api_.motor_get_state(motor.descriptor.motor, &state) == 0 && state.has_value) {
      motor.last_position = state.pos;
      motor.has_position = true;
      motor.command_position = state.pos;
      motor.requested_position = state.pos;
      motor.requested_opening = position_to_opening(motor, state.pos);
      motor.last_torque = state.torq;
      seeded.push_back(ArticoreMitCommand{motor.descriptor.motor, state.pos, 0.0f,
                                          motor.descriptor.safe_kp,
                                          motor.descriptor.safe_kd, 0.0f});
    }
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  safe_grippers_ = std::move(seeded);
}

bool SafetyRuntime::run_gripper_control_once(std::string& error) {
  std::vector<ArticoreMitCommand> commands;
  commands.reserve(motors_.size());
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  const auto now = Clock::now();
  for (auto& motor : motors_) {
    if (!motor.descriptor.is_gripper || !motor.has_gripper_target) continue;
    ArticoreFeedbackStats stats{};
    ArticoreMotorState state{};
    if (api_.motor_get_feedback_stats(motor.descriptor.motor, &stats) != 0 ||
        !stats.has_feedback ||
        api_.motor_get_state(motor.descriptor.motor, &state) != 0 ||
        !state.has_value) {
      motor.gripper_fault_reason = motor_error("gripper feedback unavailable");
      mark_motor_faulted(motor.descriptor.motor);
      error = std::string(motor.descriptor.name) + ": " +
              motor.gripper_fault_reason;
      return false;
    }
    motor.feedback_age_ns = stats.age_ns;
    if (stats.age_ns >
        static_cast<uint64_t>(config_.feedback_max_age_ms) * 1'000'000ULL) {
      motor.gripper_fault_reason = "gripper feedback exceeds maximum age";
      mark_motor_faulted(motor.descriptor.motor);
      error = std::string(motor.descriptor.name) + ": " +
              motor.gripper_fault_reason;
      return false;
    }
    if (state.status_code > 1 || state.status_code == 0) {
      motor.gripper_state = ARTICORE_GRIPPER_FAULT;
      motor.gripper_fault_reason = state.status_code == 0
          ? "gripper unexpectedly disabled"
          : "gripper motor fault status " + std::to_string(state.status_code);
      mark_motor_faulted(motor.descriptor.motor);
      error = std::string(motor.descriptor.name) + ": " +
              motor.gripper_fault_reason;
      return false;
    }

    const auto& descriptor = motor.descriptor;
    motor.last_position = state.pos;
    motor.has_position = true;
    motor.last_torque = state.torq;
    const auto torque = std::abs(state.torq);
    const auto max_dt = std::chrono::milliseconds(descriptor.max_step_interval_ms);
    const auto dt = motor.last_gripper_update == Clock::time_point{}
        ? std::chrono::duration<float>::zero()
        : std::min(std::chrono::duration<float>(now - motor.last_gripper_update),
                   std::chrono::duration<float>(max_dt));
    motor.last_gripper_update = now;
    motor.motion_samples.emplace_back(now, state.pos);
    const auto motion_window = std::chrono::milliseconds(descriptor.motion_window_ms);
    while (motor.motion_samples.size() > 1 &&
           motor.motion_samples[1].first <= now - motion_window) {
      motor.motion_samples.pop_front();
    }

    const auto update_overload = [&]() {
      motor.overload = torque >= descriptor.overload_torque;
      if (!motor.overload) {
        motor.overload_started_at = {};
        return false;
      }
      if (motor.overload_started_at == Clock::time_point{}) {
        motor.overload_started_at = now;
      }
      const bool sustained = now - motor.overload_started_at >=
          std::chrono::milliseconds(descriptor.overload_hold_ms);
      const bool retry_due = motor.last_retreat_at == Clock::time_point{} ||
          now - motor.last_retreat_at >=
              std::chrono::milliseconds(descriptor.retreat_retry_ms);
      if (!sustained || !retry_due) return false;
      const auto base = motor.retreat_active ? motor.retreat_target : state.pos;
      motor.retreat_target = std::clamp(
          base - descriptor.closing_direction * descriptor.retreat_distance,
          descriptor.lower_position, descriptor.upper_position);
      motor.command_position = motor.retreat_target;
      motor.retreat_active = true;
      motor.protective_target_active = true;
      motor.protective_target = motor.retreat_target;
      motor.gripper_state = ARTICORE_GRIPPER_OVERLOAD_RETREAT;
      motor.last_retreat_at = now;
      motor.contact_detected = true;
      motor.stalled = true;
      return true;
    };

    if (motor.gripper_state == ARTICORE_GRIPPER_CONTACT) {
      motor.gripper_state = ARTICORE_GRIPPER_HOLDING;
    }
    if (motor.gripper_state == ARTICORE_GRIPPER_OVERLOAD_RETREAT) {
      const bool retreat_complete =
          std::abs(state.pos - motor.retreat_target) <= descriptor.min_position_error ||
          now - motor.last_retreat_at >=
              std::chrono::milliseconds(descriptor.retreat_retry_ms);
      if (retreat_complete) {
        motor.gripper_state = ARTICORE_GRIPPER_HOLDING;
      }
    }

    if (motor.gripper_state == ARTICORE_GRIPPER_MOVING) {
      const bool closing =
          (motor.requested_position - state.pos) *
              descriptor.closing_direction > 1e-6f;
      if (closing) {
        const auto step = descriptor.close_speed * dt.count();
        const auto delta = motor.requested_position - motor.command_position;
        motor.command_position += std::copysign(
            std::min(std::abs(delta), step), delta);
      } else {
        motor.command_position = motor.requested_position;
        motor.contact_started_at = {};
        motor.overload_started_at = {};
        motor.motion_samples.clear();
      }

      motor.overload = torque >= descriptor.overload_torque;
      bool contact = false;
      if (torque >= descriptor.contact_torque &&
          motor.motion_samples.size() >= 2 &&
          motor.motion_samples.back().first - motor.motion_samples.front().first >=
              motion_window &&
          std::abs(motor.command_position - state.pos) >=
              descriptor.min_position_error) {
        const auto bounds = std::minmax_element(
            motor.motion_samples.begin(), motor.motion_samples.end(),
            [](const auto& left, const auto& right) {
              return left.second < right.second;
            });
        contact = bounds.second->second - bounds.first->second <=
                  descriptor.stall_movement;
      }
      if (contact) {
        if (motor.contact_started_at == Clock::time_point{}) {
          motor.contact_started_at = now;
        }
        if (now - motor.contact_started_at >=
            std::chrono::milliseconds(descriptor.contact_hold_ms)) {
          motor.protective_target = std::clamp(
              state.pos + descriptor.closing_direction * descriptor.hold_offset,
              descriptor.lower_position, descriptor.upper_position);
          motor.command_position = motor.protective_target;
          motor.protective_target_active = true;
          motor.contact_detected = true;
          motor.stalled = true;
          motor.gripper_state = ARTICORE_GRIPPER_CONTACT;
        }
      } else {
        motor.contact_started_at = {};
      }
      if (motor.gripper_state == ARTICORE_GRIPPER_MOVING &&
          std::abs(motor.requested_position - state.pos) <=
              descriptor.min_position_error) {
        motor.gripper_state = ARTICORE_GRIPPER_IDLE;
      }
    } else if (motor.gripper_state == ARTICORE_GRIPPER_HOLDING) {
      update_overload();
    } else if (motor.gripper_state == ARTICORE_GRIPPER_IDLE) {
      motor.command_position = motor.requested_position;
    }

    const bool low_gain =
        motor.gripper_state == ARTICORE_GRIPPER_CONTACT ||
        motor.gripper_state == ARTICORE_GRIPPER_HOLDING ||
        motor.gripper_state == ARTICORE_GRIPPER_OVERLOAD_RETREAT;
    commands.push_back(ArticoreMitCommand{
        descriptor.motor, motor.command_position, 0.0f,
        low_gain ? descriptor.safe_kp : descriptor.normal_kp,
        low_gain ? descriptor.safe_kd : descriptor.normal_kd, 0.0f});
  }
  if (commands.empty()) return true;
  if (api_.group_send_mit(controller_group_, commands.data(),
                          static_cast<uint32_t>(commands.size())) != 0) {
    error = motor_error("gripper control batch failed");
    return false;
  }
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    safe_grippers_ = commands;
  }
  return true;
}

bool SafetyRuntime::send_safe_hold_once(std::string& error) {
  std::vector<ArticorePosVelCommand> pv;
  std::vector<ArticoreMitCommand> mit;
  ArticoreControlMode mode;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ != ARTICORE_SAFE_HOLD) return true;
    mode = mode_;
    pv = safe_pv_;
    mit = safe_mit_;
  }
  int32_t rc = 0;
  {
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    if (mode == ARTICORE_MODE_PV) {
      if (pv.empty()) {
        error = "no PV safe-hold target";
        return false;
      }
      rc = api_.group_send_pos_vel(controller_group_, pv.data(),
                                    static_cast<uint32_t>(pv.size()));
    } else {
      if (mit.empty()) {
        error = "no MIT safe-hold target";
        return false;
      }
      rc = api_.group_send_mit(controller_group_, mit.data(),
                              static_cast<uint32_t>(mit.size()));
    }
  }
  if (rc != 0) {
    error = motor_error("safe-hold arm send failed");
    return false;
  }
  return send_gripper_hold_once(error);
}

bool SafetyRuntime::send_gripper_hold_once(std::string& error) {
  std::vector<ArticoreMitCommand> commands;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    commands = safe_grippers_;
  }
  if (commands.empty()) return true;

  std::lock_guard<std::mutex> command_lock(command_mutex_);
  const auto now = Clock::now();
  for (auto& command : commands) {
    const auto found = std::find_if(
        motors_.begin(), motors_.end(), [&](const MotorRecord& motor) {
          return motor.descriptor.is_gripper &&
                 motor.descriptor.motor == command.motor;
        });
    if (found == motors_.end()) continue;
    ArticoreFeedbackStats stats{};
    ArticoreMotorState state{};
    if (api_.motor_get_feedback_stats(command.motor, &stats) != 0 ||
        !stats.has_feedback ||
        api_.motor_get_state(command.motor, &state) != 0 || !state.has_value) {
      found->gripper_state = ARTICORE_GRIPPER_FAULT;
      found->gripper_fault_reason =
          motor_error("safe-hold gripper feedback unavailable");
      mark_motor_faulted(command.motor);
      error = std::string(found->descriptor.name) + ": " +
              found->gripper_fault_reason;
      return false;
    }
    found->feedback_age_ns = stats.age_ns;
    found->last_position = state.pos;
    found->last_torque = state.torq;
    found->has_position = true;
    if (stats.age_ns >
        static_cast<uint64_t>(config_.feedback_max_age_ms) * 1'000'000ULL ||
        state.status_code == 0 || state.status_code > 1) {
      found->gripper_state = ARTICORE_GRIPPER_FAULT;
      found->gripper_fault_reason = stats.age_ns >
              static_cast<uint64_t>(config_.feedback_max_age_ms) * 1'000'000ULL
          ? "safe-hold gripper feedback exceeds maximum age"
          : (state.status_code == 0 ? "safe-hold gripper unexpectedly disabled"
                                    : "safe-hold gripper motor fault");
      mark_motor_faulted(command.motor);
      error = std::string(found->descriptor.name) + ": " +
              found->gripper_fault_reason;
      return false;
    }

    const auto& descriptor = found->descriptor;
    const auto torque = std::abs(state.torq);
    found->overload = torque >= descriptor.overload_torque;
    if (found->overload) {
      if (found->overload_started_at == Clock::time_point{}) {
        found->overload_started_at = now;
      }
      const bool sustained = now - found->overload_started_at >=
          std::chrono::milliseconds(descriptor.overload_hold_ms);
      const bool retry_due = found->last_retreat_at == Clock::time_point{} ||
          now - found->last_retreat_at >=
              std::chrono::milliseconds(descriptor.retreat_retry_ms);
      if (sustained && retry_due) {
        found->retreat_target = std::clamp(
            state.pos - descriptor.closing_direction * descriptor.retreat_distance,
            descriptor.lower_position, descriptor.upper_position);
        found->retreat_active = true;
        found->protective_target_active = true;
        found->protective_target = found->retreat_target;
        found->last_retreat_at = now;
        found->gripper_state = ARTICORE_GRIPPER_OVERLOAD_RETREAT;
        found->contact_detected = true;
        found->stalled = true;
      }
    } else {
      found->overload_started_at = {};
      if (found->gripper_state == ARTICORE_GRIPPER_OVERLOAD_RETREAT) {
        found->gripper_state = ARTICORE_GRIPPER_HOLDING;
      }
    }
    if (found->retreat_active) command.target_position = found->retreat_target;
    else if (found->protective_target_active) {
      command.target_position = found->protective_target;
    }
    command.target_velocity = 0.0f;
    command.stiffness = descriptor.safe_kp;
    command.damping = descriptor.safe_kd;
    command.feedforward_torque = 0.0f;
  }
  if (api_.group_send_mit(controller_group_, commands.data(),
                          static_cast<uint32_t>(commands.size())) != 0) {
    error = motor_error("safe-hold gripper send failed");
    return false;
  }
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    safe_grippers_ = commands;
  }
  return true;
}

bool SafetyRuntime::refresh_feedback_health(bool recovery_check,
                                            bool allow_held_grippers,
                                            std::string& error) {
  const bool transports_ok = refresh_transport_health(error);
  uint64_t maximum_age[2] = {0, 0};
  std::vector<std::string> motor_faults;
  std::vector<std::string> unconfirmed;
  std::vector<void*> faulted_presence;
  const auto mark_unconfirmed = [&](const std::string& name) {
    if (std::find(unconfirmed.begin(), unconfirmed.end(), name) ==
        unconfirmed.end()) {
      unconfirmed.push_back(name);
    }
  };
  bool side_ok[2] = {active_sides_[0], active_sides_[1]};
  std::string side_error[2];
  for (const auto& motor : motors_) {
    const std::string name(motor.descriptor.name);
    ArticoreFeedbackStats stats{};
    if (api_.motor_get_feedback_stats(motor.descriptor.motor, &stats) != 0 ||
        !stats.has_feedback) {
      faulted_presence.push_back(motor.descriptor.motor);
      side_ok[motor.descriptor.side] = false;
      side_error[motor.descriptor.side] = motor_error("feedback statistics unavailable");
      if (recovery_check) mark_unconfirmed(name);
      continue;
    }
    maximum_age[motor.descriptor.side] =
        std::max(maximum_age[motor.descriptor.side], stats.age_ns);
    if (stats.age_ns > static_cast<uint64_t>(config_.feedback_max_age_ms) * 1'000'000ULL) {
      faulted_presence.push_back(motor.descriptor.motor);
      side_ok[motor.descriptor.side] = false;
      side_error[motor.descriptor.side] = "feedback exceeds maximum age";
      if (recovery_check) mark_unconfirmed(name);
    }
    ArticoreMotorState state{};
    if (api_.motor_get_state(motor.descriptor.motor, &state) != 0 || !state.has_value) {
      faulted_presence.push_back(motor.descriptor.motor);
      side_ok[motor.descriptor.side] = false;
      side_error[motor.descriptor.side] = motor_error("motor state unavailable");
      if (recovery_check) mark_unconfirmed(name);
      continue;
    }
    if (!finite(state.pos) || !finite(state.vel) || !finite(state.torq)) {
      faulted_presence.push_back(motor.descriptor.motor);
      side_ok[motor.descriptor.side] = false;
      side_error[motor.descriptor.side] = "motor feedback is not finite";
      if (recovery_check) mark_unconfirmed(name);
      continue;
    }
    if (!motor.descriptor.is_gripper) {
      try {
        validate_position_velocity_torque(
            motor.descriptor.motor, state.pos, state.vel, state.torq);
      } catch (const std::exception& limit_error) {
        faulted_presence.push_back(motor.descriptor.motor);
        side_ok[motor.descriptor.side] = false;
        side_error[motor.descriptor.side] = limit_error.what();
        motor_faults.push_back(name);
        continue;
      }
    }
    if (state.status_code > 1) {
      motor_faults.push_back(name);
      faulted_presence.push_back(motor.descriptor.motor);
    }
    if (recovery_check && state.status_code != 0 &&
        !(allow_held_grippers && motor.descriptor.is_gripper)) {
      mark_unconfirmed(name);
    }
    if (!recovery_check && state.status_code == 0) {
      error = "motor unexpectedly disabled: " + name;
      motor_faults.push_back(name);
      faulted_presence.push_back(motor.descriptor.motor);
    }
  }

  const auto now = Clock::now();
  const bool has_motor_faults = !motor_faults.empty();
  const bool has_unconfirmed = !unconfirmed.empty();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    motor_faults_ = std::move(motor_faults);
    for (auto* motor : faulted_presence) {
      const auto role = motor_roles_.find(motor);
      if (role != motor_roles_.end()) presence_[role->second] = ARTICORE_FAULTED;
    }
    if (recovery_check) unconfirmed_disable_ = std::move(unconfirmed);
    for (uint8_t side = 0; side < 2; ++side) {
      if (!active_sides_[side]) continue;
      sides_[side].last_feedback_age_ns = maximum_age[side];
      sides_[side].healthy = side_ok[side] && sides_[side].connected &&
                             sides_[side].transport_healthy;
      if (!side_ok[side]) {
        sides_[side].last_error = side_error[side];
        ++sides_[side].feedback_failures;
      } else {
        sides_[side].feedback_failures = 0;
      }
    }
    const bool active_feedback_ok =
        (!active_sides_[0] || side_ok[0]) &&
        (!active_sides_[1] || side_ok[1]);
    if (transports_ok && active_feedback_ok && !has_motor_faults &&
        (!recovery_check || !has_unconfirmed)) {
      uint64_t maximum_active_age = 0;
      for (uint8_t side = 0; side < 2; ++side) {
        if (active_sides_[side]) {
          maximum_active_age = std::max(maximum_active_age, maximum_age[side]);
        }
      }
      last_fresh_feedback_ =
          now - std::chrono::nanoseconds(maximum_active_age);
      consecutive_feedback_failures_ = 0;
      return true;
    }
  }
  if (error.empty()) {
    if (active_sides_[0] && !side_ok[0]) {
      error = "CH0 feedback unhealthy: " + side_error[0];
    } else if (active_sides_[1] && !side_ok[1]) {
      error = "CH1 feedback unhealthy: " + side_error[1];
    }
    else if (has_motor_faults) error = "motor fault status reported";
    else error = "not all motors are confirmed disabled";
  }
  return false;
}

bool SafetyRuntime::refresh_transport_health(std::string& error) {
  if (!api_.controller_get_transport_health) return true;
  bool healthy = true;
  for (uint8_t side = 0; side < 2; ++side) {
    if (!active_sides_[side]) continue;
    ArticoreDriverTransportHealth native{};
    const auto rc = api_.controller_get_transport_health(controllers_[side], &native);
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto& output = sides_[side];
    if (rc != 0) {
      healthy = false;
      output.transport_healthy = false;
      output.healthy = false;
      output.last_error = motor_error("transport health query failed");
    } else {
      output.connected = native.connected != 0;
      output.transport_healthy = native.healthy != 0;
      output.tx_frames = native.tx_frames;
      output.rx_frames = native.rx_frames;
      output.send_errors = native.send_errors;
      output.receive_errors = native.receive_errors;
      output.last_tx_age_ns = native.last_tx_age_ns;
      output.last_rx_age_ns = native.last_rx_age_ns;
      const std::string detail(native.last_error);
      if (!detail.empty()) output.last_error = detail;
      if (!output.connected || !output.transport_healthy) healthy = false;
      if (!output.connected) {
        for (const auto& motor : motors_) {
          if (motor.descriptor.side != side) continue;
          const auto role = motor_roles_.find(motor.descriptor.motor);
          if (role != motor_roles_.end()) {
            presence_[role->second] = ARTICORE_FAULTED;
          }
        }
      }
    }
    if (!healthy && error.empty()) {
      error = std::string(side == 0 ? "CH0" : "CH1") +
              (output.connected ? " transport unhealthy" : " transport disconnected");
      if (!output.last_error.empty()) error += ": " + output.last_error;
    }
  }
  return healthy;
}

bool SafetyRuntime::disable_hardware(bool request_feedback,
                                     bool preserve_grippers,
                                     std::string& error) {
  bool ok = true;
  if (preserve_grippers) {
    std::string hold_error;
    if (!send_gripper_hold_once(hold_error)) {
      ok = false;
      error = "gripper hold failed: " + hold_error;
      preserve_grippers = false;
    }
  }

  {
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    for (auto& motor : motors_) {
      if (preserve_grippers && motor.descriptor.is_gripper) continue;
      if (api_.motor_disable(motor.descriptor.motor) != 0) {
        ok = false;
        const auto detail = motor_error("motor disable failed");
        if (!error.empty()) error += "; ";
        error += std::string(motor.descriptor.name) + ": " + detail;
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        set_side_error_locked(motor.descriptor.side, detail, true);
      } else if (motor.descriptor.is_gripper) {
        motor.gripper_state = ARTICORE_GRIPPER_DISABLED;
      }
    }
    if (request_feedback) {
      for (uint8_t side = 0; side < 2; ++side) {
        if (!active_sides_[side]) continue;
        const auto motor_count = static_cast<uint32_t>(std::count_if(
            motors_.begin(), motors_.end(), [&](const MotorRecord& motor) {
              return motor.descriptor.side == side;
            }));
        std::vector<uint32_t> missing_motor_ids(
            std::max<uint32_t>(motor_count, 1));
        ArticoreFeedbackReport report{};
        report.struct_size = sizeof(report);
        const auto feedback_code = api_.controller_request_feedback_all_ex(
            controllers_[side], config_.disable_feedback_timeout_ms,
            &report, missing_motor_ids.data(),
            static_cast<uint32_t>(missing_motor_ids.size()));
        if (feedback_code != 0) {
          ok = false;
          std::string detail = "disable feedback confirmation failed: code=" +
              std::to_string(feedback_code);
          detail += ", expected=" + std::to_string(report.expected_count) +
                    ", received=" + std::to_string(report.received_count) +
                    ", missing=" + std::to_string(report.missing_count);
          if (report.missing_count > 0) {
            detail += ", missing motor IDs:";
            const auto copied = std::min<std::size_t>(
                report.missing_count, missing_motor_ids.size());
            for (std::size_t i = 0; i < copied; ++i) {
              detail += " " + std::to_string(missing_motor_ids[i]);
            }
          }
          const auto native_error = motor_error("");
          if (!native_error.empty()) detail += "; " + native_error;
          if (!error.empty()) error += "; ";
          error += (side == 0 ? "CH0: " : "CH1: ") + detail;
        }
      }
      std::string feedback_error;
      if (!refresh_feedback_health(true, preserve_grippers, feedback_error)) {
        ok = false;
        if (!error.empty()) error += "; ";
        error += feedback_error;
      }
    }
  }
  return ok;
}

void SafetyRuntime::enter_fault(const std::string& reason) {
  bool preserve_grippers = false;
  {
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ == ARTICORE_DISCONNECTED) return;
    preserve_grippers =
        config_.gripper_fault_action == ARTICORE_GRIPPER_FAULT_HOLD &&
        !safe_grippers_.empty();
    state_ = ARTICORE_FAULT;
    fault_latched_ = true;
    hardware_transition_ = true;
    disable_confirmed_ = false;
    fault_reason_ = reason;
    arm_mailbox_ = ArmMailbox{};
    cancel_active_trajectory_locked(
        ARTICORE_TRAJECTORY_FAILED, reason);
    next_safe_hold_ = Clock::now();
    for (auto& motor : motors_) {
      if (!motor.descriptor.is_gripper) continue;
      if (preserve_grippers && motor.has_position) {
        if (motor.gripper_state != ARTICORE_GRIPPER_OVERLOAD_RETREAT) {
          motor.gripper_state = ARTICORE_GRIPPER_HOLDING;
        }
      } else {
        motor.gripper_state = ARTICORE_GRIPPER_FAULT;
        motor.gripper_fault_reason = reason;
      }
    }
  }
  std::string disable_error;
  const bool policy_applied =
      disable_hardware(true, preserve_grippers, disable_error);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    disable_confirmed_ = policy_applied && !preserve_grippers;
    hardware_transition_ = false;
    if (!disable_error.empty()) fault_reason_ += "; disable: " + disable_error;
  }
  wakeup_.notify_all();
}

void SafetyRuntime::disable() {
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ == ARTICORE_DISCONNECTED) {
      throw std::runtime_error("cannot disable a disconnected runtime");
    }
    if (state_ == ARTICORE_FAULT) {
      throw std::runtime_error("FAULT is latched; use recover after physical disable");
    }
    hardware_transition_ = true;
  }
  std::string error;
  const bool confirmed = disable_hardware(true, false, error);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    disable_confirmed_ = confirmed;
    safe_pv_.clear();
    safe_mit_.clear();
    safe_grippers_.clear();
    arm_mailbox_ = ArmMailbox{};
    cancel_active_trajectory_locked(
        confirmed ? ARTICORE_TRAJECTORY_CANCELED
                  : ARTICORE_TRAJECTORY_FAILED,
        confirmed ? "runtime disabled" : "disable confirmation failed");
    has_successful_command_ = false;
    hardware_transition_ = false;
    if (confirmed) {
      state_ = ARTICORE_READY;
      fault_reason_.clear();
    } else {
      state_ = ARTICORE_FAULT;
      fault_latched_ = true;
      fault_reason_ = "disable confirmation failed: " + error;
    }
  }
  if (!confirmed) throw std::runtime_error(error);
}

void SafetyRuntime::estop(const std::string& reason) {
  enter_fault(reason.empty() ? "emergency stop" : reason);
}

void SafetyRuntime::recover() {
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ != ARTICORE_FAULT || !fault_latched_) {
      throw std::runtime_error("recover is only valid from latched FAULT");
    }
    if ((active_sides_[0] && !sides_[0].connected) ||
        (active_sides_[1] && !sides_[1].connected)) {
      throw std::runtime_error(
          "all active transports must be connected before recover");
    }
    hardware_transition_ = true;
  }
  std::string error;
  if (!disable_hardware(true, false, error)) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    hardware_transition_ = false;
    throw std::runtime_error(error);
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_ = ARTICORE_READY;
  fault_latched_ = false;
  disable_confirmed_ = true;
  fault_reason_.clear();
  consecutive_send_failures_ = 0;
  consecutive_feedback_failures_ = 0;
  motor_faults_.clear();
  unconfirmed_disable_.clear();
  safe_pv_.clear();
  safe_mit_.clear();
  safe_grippers_.clear();
  arm_mailbox_ = ArmMailbox{};
  cancel_active_trajectory_locked(
      ARTICORE_TRAJECTORY_CANCELED, "runtime recovered");
  has_successful_command_ = false;
  hardware_transition_ = false;
  for (auto& entry : presence_) {
    if (entry.second == ARTICORE_FAULTED) entry.second = ARTICORE_PRESENT;
  }
}

ArticoreSafetyHealth SafetyRuntime::health() const {
  ArticoreSafetyHealth result{};
  const auto now = Clock::now();
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  std::lock_guard<std::mutex> lock(state_mutex_);
  result.state = state_;
  result.safe_holding = state_ == ARTICORE_SAFE_HOLD;
  result.disable_confirmed = disable_confirmed_ ? 1 : 0;
  result.last_successful_command_age_ns = age_ns(
      last_successful_command_, has_successful_command_, now);
  result.last_fresh_feedback_age_ns = age_ns(
      last_fresh_feedback_, last_fresh_feedback_ != Clock::time_point{}, now);
  result.consecutive_send_failures = consecutive_send_failures_;
  result.consecutive_feedback_failures = consecutive_feedback_failures_;
  for (uint8_t side = 0; side < 2; ++side) {
    auto& output = side == 0 ? result.left_transport : result.right_transport;
    output.connected = sides_[side].connected ? 1 : 0;
    output.healthy = sides_[side].healthy ? 1 : 0;
    output.consecutive_send_failures = sides_[side].send_failures;
    output.consecutive_feedback_failures = sides_[side].feedback_failures;
    output.last_feedback_age_ns = sides_[side].last_feedback_age_ns;
    output.tx_frames = sides_[side].tx_frames;
    output.rx_frames = sides_[side].rx_frames;
    output.send_errors = sides_[side].send_errors;
    output.receive_errors = sides_[side].receive_errors;
    output.last_tx_age_ns = sides_[side].last_tx_age_ns;
    output.last_rx_age_ns = sides_[side].last_rx_age_ns;
    copy_text(output.last_error, sides_[side].last_error);
  }
  for (const auto& motor : motors_) {
    if (!motor.descriptor.is_gripper || result.gripper_count >= 2) continue;
    auto& output = result.grippers[result.gripper_count++];
    output.available = 1;
    output.side = motor.descriptor.side;
    output.control_state = motor.gripper_state;
    output.opening = motor.has_position
        ? position_to_opening(motor, motor.last_position)
        : motor.requested_opening;
    output.motor_position = motor.last_position;
    output.torque = motor.last_torque;
    output.contact_detected = motor.contact_detected ? 1 : 0;
    output.stalled = motor.stalled ? 1 : 0;
    output.overload = motor.overload ? 1 : 0;
    output.has_hold_target = motor.protective_target_active ||
                             motor.retreat_active || motor.has_gripper_target;
    output.hold_target = motor.retreat_active
        ? motor.retreat_target
        : (motor.protective_target_active ? motor.protective_target
                                          : motor.command_position);
    output.feedback_age_ns = motor.feedback_age_ns;
    copy_text(output.name, motor.descriptor.name);
    copy_text(output.fault_reason, motor.gripper_fault_reason);
  }
  result.motor_fault_count = static_cast<uint32_t>(
      std::min<std::size_t>(motor_faults_.size(), 32));
  for (uint32_t i = 0; i < result.motor_fault_count; ++i) {
    copy_text(result.motor_faults[i], motor_faults_[i]);
  }
  result.unconfirmed_disable_count = static_cast<uint32_t>(
      std::min<std::size_t>(unconfirmed_disable_.size(), 32));
  for (uint32_t i = 0; i < result.unconfirmed_disable_count; ++i) {
    copy_text(result.unconfirmed_disable[i], unconfirmed_disable_[i]);
  }
  copy_text(result.fault_reason, fault_reason_);
  return result;
}

bool SafetyRuntime::run_arm_control_cycle(Clock::time_point now,
                                          std::string& error) {
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  ArticoreControlMode mode;
  ArmMailbox mailbox;
  std::optional<TrajectoryRecord> trajectory;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (hardware_transition_ ||
        (state_ != ARTICORE_ENABLED && state_ != ARTICORE_RUNNING)) {
      return true;
    }
    mode = mode_;
    mailbox = arm_mailbox_;
    trajectory = active_trajectory_;
  }

  std::vector<ArticorePosVelCommand> pv;
  std::vector<ArticoreMitCommand> mit;
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
  } else {
    if (!mailbox.valid) return true;
    pv = mailbox.pv;
    mit = mailbox.mit;
  }

  const int32_t result = mode == ARTICORE_MODE_PV
      ? api_.group_send_pos_vel(controller_group_, pv.data(),
                                static_cast<uint32_t>(pv.size()))
      : api_.group_send_mit(controller_group_, mit.data(),
                           static_cast<uint32_t>(mit.size()));
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
      ++arm_mailbox_.generation;
      arm_mailbox_.sent_generation = arm_mailbox_.generation;
      arm_mailbox_.submitted_at = now;
      arm_mailbox_.pv = std::move(pv);
      arm_mailbox_.mit = std::move(mit);
      finish_trajectory_locked(
          trajectory_id, ARTICORE_TRAJECTORY_COMPLETED, "", now);
    }
  } else if (mailbox.user_command) {
    has_successful_command_ = true;
    state_ = ARTICORE_RUNNING;
    if (mailbox.generation > arm_mailbox_.sent_generation) {
      arm_mailbox_.sent_generation = mailbox.generation;
      last_successful_command_ = now;
    }
  }
  return true;
}

void SafetyRuntime::worker_loop() {
  const auto control_period = std::chrono::nanoseconds(
      1'000'000'000ULL / config_.control_hz);
  const auto idle_poll_period = std::chrono::milliseconds(2);
  const auto feedback_period = std::chrono::nanoseconds(
      1'000'000'000ULL / config_.feedback_check_hz);
  const auto hold_period = std::chrono::nanoseconds(
      1'000'000'000ULL / config_.safe_hold_hz);
  const auto gripper_period = std::chrono::nanoseconds(
      1'000'000'000ULL / config_.gripper_control_hz);
  const auto advance_deadline = [](Clock::time_point& deadline,
                                   std::chrono::nanoseconds period,
                                   Clock::time_point now) {
    if (deadline == Clock::time_point{}) deadline = now;
    if (deadline > now) return;
    const auto overdue = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now - deadline);
    const auto periods = overdue.count() / period.count() + 1;
    deadline += period * periods;
  };
  for (;;) {
    {
      std::unique_lock<std::mutex> lock(state_mutex_);
      const auto now = Clock::now();
      const auto deadline =
          !hardware_transition_ &&
                  (state_ == ARTICORE_ENABLED || state_ == ARTICORE_RUNNING) &&
                  next_control_tick_ != Clock::time_point{}
              ? next_control_tick_
              : now + idle_poll_period;
      wakeup_.wait_until(lock, deadline, [&] { return stopping_; });
      if (stopping_) return;
      if (hardware_transition_) continue;
    }

    const auto now = Clock::now();
    bool grace_fault = false;
    bool command_timeout = false;
    bool run_feedback_check = false;
    bool run_arm_control = false;
    bool run_hold = false;
    bool run_gripper_control = false;
    bool fault_gripper_hold = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (state_ == ARTICORE_ENABLED &&
          now - enabled_at_ >= std::chrono::milliseconds(config_.enable_grace_ms) &&
          !has_successful_command_) {
        grace_fault = true;
      } else if (state_ == ARTICORE_RUNNING && has_successful_command_ &&
                 now - last_successful_command_ >=
                     std::chrono::milliseconds(config_.command_timeout_ms)) {
        command_timeout = true;
      }
      if ((state_ == ARTICORE_ENABLED || state_ == ARTICORE_RUNNING) &&
          now >= next_control_tick_) {
        run_arm_control = true;
        advance_deadline(next_control_tick_, control_period, now);
      }
      if ((state_ == ARTICORE_RUNNING || state_ == ARTICORE_SAFE_HOLD) &&
          now >= next_feedback_check_) {
        run_feedback_check = true;
        advance_deadline(next_feedback_check_, feedback_period, now);
      }
      if (state_ == ARTICORE_SAFE_HOLD && now >= next_safe_hold_) {
        run_hold = true;
        advance_deadline(next_safe_hold_, hold_period, now);
      }
      if ((state_ == ARTICORE_ENABLED || state_ == ARTICORE_RUNNING) &&
          now >= next_gripper_control_ &&
          std::any_of(motors_.begin(), motors_.end(),
                      [](const MotorRecord& motor) {
                        return motor.descriptor.is_gripper &&
                               motor.has_gripper_target;
                      })) {
        run_gripper_control = true;
        advance_deadline(next_gripper_control_, gripper_period, now);
      }
      if (state_ == ARTICORE_FAULT &&
          config_.gripper_fault_action == ARTICORE_GRIPPER_FAULT_HOLD &&
          !safe_grippers_.empty() && now >= next_safe_hold_) {
        run_hold = true;
        fault_gripper_hold = true;
        advance_deadline(next_safe_hold_, hold_period, now);
      }
    }

    if (grace_fault) {
      enter_fault("enable grace expired before the first successful command");
      continue;
    }
    if (command_timeout) {
      std::string hold_error;
      if (!enter_safe_hold_from_feedback("command watchdog timed out",
                                         hold_error)) {
        enter_fault("command watchdog timed out; current-position hold "
                    "unavailable: " + hold_error);
      }
      continue;
    }
    if (run_arm_control) {
      std::string error;
      if (!run_arm_control_cycle(now, error)) {
        std::string hold_error;
        if (!enter_safe_hold_from_feedback(
                "batch send failed: " + error, hold_error)) {
          enter_fault("batch send failed: " + error +
                      "; current-position hold unavailable: " + hold_error);
        }
        continue;
      }
    }
    if (run_feedback_check) {
      std::string error;
      bool healthy = false;
      {
        std::lock_guard<std::mutex> command_lock(command_mutex_);
        healthy = refresh_feedback_health(false, false, error);
      }
      if (!healthy) {
        const bool severe =
            error.find("maximum age") != std::string::npos ||
            error.find("motor fault status") != std::string::npos ||
            error.find("unexpectedly disabled") != std::string::npos ||
            error.find("transport disconnected") != std::string::npos;
        bool fault = severe;
        bool enter_hold = false;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          ++consecutive_feedback_failures_;
          const bool trajectory_failed = active_trajectory_.has_value();
          if (trajectory_failed) {
            cancel_active_trajectory_locked(
                ARTICORE_TRAJECTORY_FAILED,
                "trajectory feedback failure: " + error);
            arm_mailbox_ = ArmMailbox{};
          }
          if (state_ == ARTICORE_SAFE_HOLD || severe) {
            fault = true;
          } else if (state_ == ARTICORE_RUNNING &&
                     (trajectory_failed ||
                      consecutive_feedback_failures_ >=
                          config_.feedback_failure_threshold)) {
            enter_hold = true;
          }
        }
        if (enter_hold) {
          std::string hold_error;
          if (!enter_safe_hold_from_feedback(
                  "consecutive feedback failures: " + error, hold_error)) {
            error += "; current-position hold unavailable: " + hold_error;
            fault = true;
          }
        }
        if (fault) enter_fault(error);
        continue;
      }
    }
    if (run_gripper_control) {
      std::string error;
      if (!run_gripper_control_once(error)) {
        const bool send_failure =
            error.find("batch failed") != std::string::npos;
        if (send_failure) {
          {
            std::lock_guard<std::mutex> lock(state_mutex_);
            ++consecutive_send_failures_;
          }
          std::string hold_error;
          if (!enter_safe_hold_from_feedback(
                  "gripper control send failed: " + error, hold_error)) {
            enter_fault("gripper control send failed: " + error +
                        "; current-position hold unavailable: " + hold_error);
          }
        } else {
          enter_fault("gripper control fault: " + error);
        }
        continue;
      }
    }
    if (run_hold) {
      std::string error;
      const bool held = fault_gripper_hold
          ? send_gripper_hold_once(error)
          : send_safe_hold_once(error);
      if (!held) {
        if (fault_gripper_hold) {
          std::string disable_error;
          const bool disabled = disable_hardware(true, false, disable_error);
          std::lock_guard<std::mutex> lock(state_mutex_);
          disable_confirmed_ = disabled;
          fault_reason_ += "; gripper fault hold failed: " + error;
          if (!disable_error.empty()) {
            fault_reason_ += "; disable: " + disable_error;
          }
          safe_grippers_.clear();
          for (auto& motor : motors_) {
            if (motor.descriptor.is_gripper) {
              motor.gripper_state = ARTICORE_GRIPPER_FAULT;
              motor.gripper_fault_reason = error;
            }
          }
          continue;
        }
        bool fault = false;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          ++consecutive_hold_failures_;
          ++consecutive_send_failures_;
          fault = consecutive_hold_failures_ >= config_.safe_hold_failure_threshold;
        }
        if (fault) enter_fault("safe hold failed: " + error);
      } else {
        std::lock_guard<std::mutex> lock(state_mutex_);
        consecutive_hold_failures_ = 0;
      }
    }
  }
}

void SafetyRuntime::close() {
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (stopping_) return;
    stopping_ = true;
    arm_mailbox_ = ArmMailbox{};
    cancel_active_trajectory_locked(
        ARTICORE_TRAJECTORY_CANCELED, "runtime closed");
  }
  wakeup_.notify_all();
  if (worker_.joinable()) worker_.join();
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_ = ARTICORE_DISCONNECTED;
  for (auto& side : sides_) {
    side.connected = false;
    side.healthy = false;
  }
}

}  // namespace articore

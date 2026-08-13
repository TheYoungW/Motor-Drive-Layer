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

bool SafetyRuntime::finite(float value) {
  return std::isfinite(static_cast<double>(value));
}

SafetyRuntime::SafetyRuntime(ArticoreRuntimeConfig config,
                             ArticoreMotorApi api,
                             void* controller_group,
                             void* left_controller,
                             void* right_controller,
                             std::vector<ArticoreMotorDescriptor> motors,
                             ArticoreControllerCallFn controller_enable_all,
                             ArticoreControllerCallFn motor_enable)
    : config_(config), api_(api), controller_group_(controller_group),
      controller_enable_all_(controller_enable_all), motor_enable_(motor_enable) {
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
      record.requested_speed = 1000.0f;
      record.command_speed = motor.close_speed;
      record.force_profiles.emplace(
          ARTICORE_GRIPPER_FORCE_DEFAULT,
          MotorRecord::GripperForceProfile{
              motor.contact_torque, motor.overload_torque,
              motor.normal_kp, motor.normal_kd, motor.safe_kp, motor.safe_kd});
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
  last_enable_report_.struct_size = sizeof(last_enable_report_);
  last_disable_report_.struct_size = sizeof(last_disable_report_);
  worker_ = std::thread([this] { worker_loop(); });
}

SafetyRuntime::~SafetyRuntime() {
  try {
    close();
  } catch (...) {
    // Destructors cannot propagate a failed physical-disable confirmation.
    // The checked C ABI close entry point reports it; legacy free remains
    // best-effort but still has to stop and join the native worker safely.
    stop_worker();
  }
}

void SafetyRuntime::connect() {
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
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

void SafetyRuntime::stop_worker() {
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (stopping_) return;
    stopping_ = true;
    arm_mailbox_ = ArmMailbox{};
    fault_hold_active_ = false;
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

void SafetyRuntime::close() {
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  bool needs_disable = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (stopping_) return;
    if (state_ == ARTICORE_DISCONNECTED) {
      needs_disable = false;
    } else {
      needs_disable = !disable_confirmed_ || state_ == ARTICORE_ENABLED ||
                      state_ == ARTICORE_RUNNING ||
                      state_ == ARTICORE_SAFE_HOLD;
    }
  }
  if (needs_disable) disable();
  stop_worker();
}

}  // namespace articore

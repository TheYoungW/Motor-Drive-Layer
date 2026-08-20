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

void SafetyRuntime::report_feedback_failure(uint8_t side,
                                            const std::string& reason) {
  if (side > 1 || !active_sides_[side]) {
    throw std::invalid_argument("feedback side is not active");
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  ++consecutive_feedback_failures_;
  ++sides_[side].feedback_failures;
  sides_[side].healthy = false;
  sides_[side].last_error = reason;
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

bool SafetyRuntime::prepare_protective_hold(std::string& error) {
  std::vector<ArticorePosVelCommand> pv;
  std::vector<ArticoreMitCommand> mit;
  std::vector<ArticoreMitCommand> grippers;
  std::vector<void*> faulted;
  ArticoreControlMode mode;
  bool side_connected[2]{};
  {
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      mode = mode_;
      side_connected[0] = sides_[0].connected;
      side_connected[1] = sides_[1].connected;
    }

    const auto fallback_position = [&](void* handle, float& position) {
      if (mode == ARTICORE_MODE_PV) {
        const auto found = std::find_if(
            last_sent_pv_.begin(), last_sent_pv_.end(),
            [&](const ArticorePosVelCommand& value) {
              return value.motor == handle;
            });
        if (found == last_sent_pv_.end()) return false;
        position = found->target_position;
        return true;
      }
      const auto found = std::find_if(
          last_sent_mit_.begin(), last_sent_mit_.end(),
          [&](const ArticoreMitCommand& value) {
            return value.motor == handle;
          });
      if (found == last_sent_mit_.end()) return false;
      position = found->target_position;
      return true;
    };

    for (const auto& motor : motors_) {
      if (motor.descriptor.is_gripper) continue;
      if (!side_connected[motor.descriptor.side]) {
        faulted.push_back(motor.descriptor.motor);
        continue;
      }
      ArticoreFeedbackStats stats{};
      ArticoreMotorState state{};
      const bool has_stats =
          api_.motor_get_feedback_stats(motor.descriptor.motor, &stats) == 0 &&
          stats.has_feedback;
      const bool has_state =
          api_.motor_get_state(motor.descriptor.motor, &state) == 0 &&
          state.has_value;
      if (has_state && state.status_code != 1) {
        faulted.push_back(motor.descriptor.motor);
        if (!error.empty()) error += "; ";
        error += std::string(motor.descriptor.name) +
                 ": excluded from protective hold because status=" +
                 std::to_string(state.status_code);
        continue;
      }

      float position = 0.0f;
      const bool fresh_position =
          has_stats && has_state && finite(state.pos) &&
          stats.age_ns <=
              static_cast<uint64_t>(config_.feedback_max_age_ms) *
                  1'000'000ULL;
      if (fresh_position) {
        position = state.pos;
      } else if (!fallback_position(motor.descriptor.motor, position)) {
        if (!error.empty()) error += "; ";
        error += std::string(motor.descriptor.name) +
                 ": no feedback or last successful output for protective hold";
        continue;
      }

      if (mode == ARTICORE_MODE_PV) {
        pv.push_back(ArticorePosVelCommand{
            motor.descriptor.motor, position, config_.safe_pv_velocity_limit});
      } else {
        mit.push_back(ArticoreMitCommand{
            motor.descriptor.motor, position, 0.0f,
            motor.descriptor.safe_kp, motor.descriptor.safe_kd, 0.0f});
      }
    }

    for (const auto& command : safe_grippers_) {
      const auto motor = std::find_if(
          motors_.begin(), motors_.end(), [&](const MotorRecord& value) {
            return value.descriptor.is_gripper &&
                   value.descriptor.motor == command.motor;
          });
      if (motor == motors_.end() || !side_connected[motor->descriptor.side]) {
        if (motor != motors_.end()) faulted.push_back(command.motor);
        continue;
      }
      ArticoreMotorState state{};
      const bool has_state =
          api_.motor_get_state(command.motor, &state) == 0 && state.has_value;
      if (has_state && state.status_code != 1) {
        faulted.push_back(command.motor);
        continue;
      }
      // Missing/stale feedback is a communication-quality condition, not a
      // declaration that the physical gripper itself is faulted. The last
      // successfully transmitted target remains the conservative fallback.
      auto safe = command;
      const auto& profile = active_gripper_profile(*motor);
      const bool protection_enabled =
          motor->gripper_mode == ARTICORE_GRIPPER_MODE_PROTECTED &&
          static_cast<int32_t>(motor->force_level) !=
              ARTICORE_GRIPPER_STRENGTH_MIN;
      safe.target_velocity = 0.0f;
      safe.stiffness =
          protection_enabled ? profile.hold_kp : profile.moving_kp;
      safe.damping =
          protection_enabled ? profile.hold_kd : profile.moving_kd;
      safe.feedforward_torque = 0.0f;
      grippers.push_back(safe);
    }
  }

  std::lock_guard<std::mutex> state_lock(state_mutex_);
  safe_pv_ = std::move(pv);
  safe_mit_ = std::move(mit);
  safe_grippers_ = std::move(grippers);
  for (auto* handle : faulted) {
    const auto role = motor_roles_.find(handle);
    if (role != motor_roles_.end()) presence_[role->second] = ARTICORE_FAULTED;
  }
  fault_hold_active_ = !safe_pv_.empty() || !safe_mit_.empty() ||
                       !safe_grippers_.empty();
  return mode == ARTICORE_MODE_PV ? !safe_pv_.empty() : !safe_mit_.empty();
}

bool SafetyRuntime::send_safe_hold_once(std::string& error) {
  std::vector<ArticorePosVelCommand> pv;
  std::vector<ArticoreMitCommand> mit;
  std::set<void*> intentionally_disabled;
  ArticoreControlMode mode;
  ArticoreSafetyState hold_state;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ != ARTICORE_SAFE_HOLD && state_ != ARTICORE_SAFE_STOP &&
        state_ != ARTICORE_FAULT) return true;
    hold_state = state_;
    mode = mode_;
    pv = safe_pv_;
    mit = safe_mit_;
    intentionally_disabled = intentionally_disabled_motors_;
  }
  const bool had_pv_hold = !pv.empty();
  const bool had_mit_hold = !mit.empty();
  pv.erase(std::remove_if(pv.begin(), pv.end(), [&](const auto& command) {
             return intentionally_disabled.count(command.motor) != 0;
           }), pv.end());
  mit.erase(std::remove_if(mit.begin(), mit.end(), [&](const auto& command) {
              return intentionally_disabled.count(command.motor) != 0;
            }), mit.end());
  int32_t rc = 0;
  {
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      if (hardware_transition_ || state_ != hold_state) return true;
    }
    if (hold_state == ARTICORE_FAULT) {
      for (uint8_t side = 0; side < 2; ++side) {
        if (mode == ARTICORE_MODE_PV) {
          std::vector<ArticorePosVelCommand> side_commands;
          for (const auto& command : pv) {
            const auto record = std::find_if(
                motors_.begin(), motors_.end(), [&](const MotorRecord& value) {
                  return value.descriptor.motor == command.motor;
                });
            if (record != motors_.end() && record->descriptor.side == side) {
              side_commands.push_back(command);
            }
          }
          if (!side_commands.empty() &&
              api_.group_send_pos_vel(controller_group_, side_commands.data(),
                  static_cast<uint32_t>(side_commands.size())) != 0) {
            rc = -1;
            if (!error.empty()) error += "; ";
            error += "CH" + std::to_string(side) +
                     " protective hold send failed";
          }
        } else {
          std::vector<ArticoreMitCommand> side_commands;
          for (const auto& command : mit) {
            const auto record = std::find_if(
                motors_.begin(), motors_.end(), [&](const MotorRecord& value) {
                  return value.descriptor.motor == command.motor;
                });
            if (record != motors_.end() && record->descriptor.side == side) {
              side_commands.push_back(command);
            }
          }
          if (!side_commands.empty() &&
              api_.group_send_mit(controller_group_, side_commands.data(),
                  static_cast<uint32_t>(side_commands.size())) != 0) {
            rc = -1;
            if (!error.empty()) error += "; ";
            error += "CH" + std::to_string(side) +
                     " protective hold send failed";
          }
        }
      }
    } else if (mode == ARTICORE_MODE_PV) {
      if (pv.empty()) {
        if (had_pv_hold) {
          rc = 0;
        } else {
          error = "no PV safe-hold target";
          return false;
        }
      } else {
        rc = api_.group_send_pos_vel(controller_group_, pv.data(),
                                      static_cast<uint32_t>(pv.size()));
      }
    } else {
      if (mit.empty()) {
        if (had_mit_hold) {
          rc = 0;
        } else {
          error = "no MIT safe-hold target";
          return false;
        }
      } else {
        rc = api_.group_send_mit(controller_group_, mit.data(),
                                static_cast<uint32_t>(mit.size()));
      }
    }
  }
  if (rc != 0 && error.empty()) {
    error = motor_error("safe-hold arm send failed");
  }
  std::string gripper_error;
  const bool gripper_sent = send_gripper_hold_once(gripper_error);
  if (!gripper_error.empty()) {
    if (!error.empty()) error += "; ";
    error += gripper_error;
  }
  return rc == 0 && gripper_sent;
}

bool SafetyRuntime::refresh_feedback_health(bool recovery_check,
                                            bool allow_held_grippers,
                                            std::string& error,
                                            bool* diagnostic_only) {
  if (diagnostic_only) *diagnostic_only = false;
  const bool transports_ok = refresh_transport_health(error);
  bool actionable_failure = !transports_ok;
  uint64_t maximum_age[2] = {0, 0};
  std::vector<std::string> motor_faults;
  std::vector<std::string> unconfirmed;
  std::vector<void*> faulted_presence;
  std::set<void*> intentionally_disabled;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    intentionally_disabled = intentionally_disabled_motors_;
  }
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
    ArticoreMotorState state{};
    const bool has_state =
        api_.motor_get_state(motor.descriptor.motor, &state) == 0 &&
        state.has_value;
    const auto identity = [&]() {
      std::ostringstream detail;
      detail << "CH" << static_cast<unsigned>(motor.descriptor.side) << "/"
             << name << " (CAN ID ";
      if (has_state) detail << static_cast<unsigned>(state.can_id);
      else detail << "unavailable";
      detail << ")";
      return detail.str();
    };
    if (api_.motor_get_feedback_stats(motor.descriptor.motor, &stats) != 0 ||
        !stats.has_feedback) {
      side_ok[motor.descriptor.side] = false;
      side_error[motor.descriptor.side] =
          identity() + ": feedback statistics unavailable; actual=missing, "
                       "threshold=valid fresh feedback required";
      if (recovery_check) mark_unconfirmed(name);
      continue;
    }
    maximum_age[motor.descriptor.side] =
        std::max(maximum_age[motor.descriptor.side], stats.age_ns);
    if (stats.age_ns > static_cast<uint64_t>(config_.feedback_max_age_ms) * 1'000'000ULL) {
      side_ok[motor.descriptor.side] = false;
      side_error[motor.descriptor.side] =
          identity() + ": feedback exceeds maximum age; actual_age_ns=" +
          std::to_string(stats.age_ns) + ", threshold_age_ns<=" +
          std::to_string(static_cast<uint64_t>(config_.feedback_max_age_ms) *
                         1'000'000ULL);
      if (recovery_check) mark_unconfirmed(name);
    }
    if (!has_state) {
      side_ok[motor.descriptor.side] = false;
      side_error[motor.descriptor.side] =
          identity() + ": motor state unavailable; actual=missing, "
                       "threshold=valid motor state required";
      if (recovery_check) mark_unconfirmed(name);
      continue;
    }
    if (!finite(state.pos) || !finite(state.vel) || !finite(state.torq)) {
      actionable_failure = true;
      side_ok[motor.descriptor.side] = false;
      std::ostringstream detail;
      detail << identity() << ": motor feedback is not finite; actual={position="
             << state.pos << ", velocity=" << state.vel << ", torque="
             << state.torq << "}, threshold=all feedback values finite";
      side_error[motor.descriptor.side] = detail.str();
      if (recovery_check) mark_unconfirmed(name);
      continue;
    }
    if (state.status_code > 1) {
      actionable_failure = true;
      motor_faults.push_back(name);
      faulted_presence.push_back(motor.descriptor.motor);
      side_ok[motor.descriptor.side] = false;
      side_error[motor.descriptor.side] =
          identity() + ": motor fault status reported; actual_status=" +
          std::to_string(state.status_code) + ", threshold_status<=1";
    }
    if (state.status_code <= 1 && recovery_check && state.status_code != 0 &&
        !(allow_held_grippers && motor.descriptor.is_gripper)) {
      mark_unconfirmed(name);
      side_ok[motor.descriptor.side] = false;
      side_error[motor.descriptor.side] =
          identity() + ": motor is not disabled; actual_status=" +
          std::to_string(state.status_code) + ", threshold_status=0";
    }
    if (!recovery_check && state.status_code == 0 &&
        !intentionally_disabled.count(motor.descriptor.motor)) {
      actionable_failure = true;
      error = identity() +
          ": motor unexpectedly disabled; actual_status=0, threshold_status=1";
      motor_faults.push_back(name);
      faulted_presence.push_back(motor.descriptor.motor);
      side_ok[motor.descriptor.side] = false;
      side_error[motor.descriptor.side] = error;
    }
  }

  const auto now = Clock::now();
  const bool has_motor_faults = !motor_faults.empty();
  const bool has_unconfirmed = !unconfirmed.empty();
  std::string unconfirmed_detail;
  for (const auto& name : unconfirmed) {
    if (!unconfirmed_detail.empty()) unconfirmed_detail += ", ";
    unconfirmed_detail += name;
  }
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
  if (diagnostic_only) {
    *diagnostic_only = !recovery_check && !actionable_failure;
  }
  if (error.empty()) {
    if (active_sides_[0] && !side_ok[0]) {
      error = "CH0 feedback unhealthy: " + side_error[0];
    } else if (active_sides_[1] && !side_ok[1]) {
      error = "CH1 feedback unhealthy: " + side_error[1];
    }
    else if (has_motor_faults) error = "motor fault status reported";
    else {
      error = "not all motors are confirmed disabled";
      if (!unconfirmed_detail.empty()) error += ": " + unconfirmed_detail;
    }
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

void SafetyRuntime::enter_fault(const std::string& reason, bool torque_off,
                                bool allow_protective_hold) {
  std::string hold_error;
  const bool arm_hold_available =
      allow_protective_hold && prepare_protective_hold(hold_error);
  {
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ == ARTICORE_DISCONNECTED) return;
    state_ = ARTICORE_FAULT;
    terminate_trajectory_locked(
        ARTICORE_TRAJECTORY_FAULT,
        "trajectory terminated by Runtime fault: " + reason);
    fault_latched_ = true;
    hardware_transition_ = torque_off;
    disable_confirmed_ = false;
    fault_reason_ = emergency_stop_latched_
        ? "emergency stop requested"
        : reason;
    safety_reason_.clear();
    if (allow_protective_hold && !hold_error.empty()) {
      fault_reason_ += "; protective hold: " + hold_error;
    }
    clear_pending_arm_mailbox();
    arm_mailbox_ = ArmMailbox{};
    gravity_control_.phase = ARTICORE_GRAVITY_INACTIVE;
    gravity_control_.hold_positions.clear();
    gravity_control_.status.active = 0;
    gravity_control_.status.phase = ARTICORE_GRAVITY_INACTIVE;
    next_safe_hold_ = Clock::now();
    if (!allow_protective_hold) {
      safe_pv_.clear();
      safe_mit_.clear();
      safe_grippers_.clear();
    }
    fault_hold_active_ = allow_protective_hold &&
        (arm_hold_available || !safe_grippers_.empty());
    for (auto& motor : motors_) {
      if (!motor.descriptor.is_gripper) continue;
      if (motor.gripper_fault_reason.empty() && motor.has_position &&
          std::any_of(safe_grippers_.begin(), safe_grippers_.end(),
                      [&](const ArticoreMitCommand& command) {
                        return command.motor == motor.descriptor.motor;
                      })) {
        if (motor.gripper_state != ARTICORE_GRIPPER_OVERLOAD_RETREAT) {
          motor.gripper_state = ARTICORE_GRIPPER_HOLDING;
        }
      } else {
        motor.gripper_state = ARTICORE_GRIPPER_FAULT;
        if (motor.gripper_fault_reason.empty()) {
          motor.gripper_fault_reason = reason;
        }
      }
    }
  }
  if (torque_off) {
    const bool preserve_grippers = allow_protective_hold &&
        config_.gripper_fault_action == ARTICORE_GRIPPER_FAULT_HOLD &&
        !safe_grippers_.empty();
    std::string disable_error;
    const bool disabled = disable_hardware(true, preserve_grippers, disable_error);
    std::lock_guard<std::mutex> lock(state_mutex_);
    disable_confirmed_ = disabled && !preserve_grippers;
    hardware_transition_ = false;
    safe_pv_.clear();
    safe_mit_.clear();
    if (!preserve_grippers) safe_grippers_.clear();
    last_sent_pv_.clear();
    last_sent_mit_.clear();
    fault_hold_active_ = preserve_grippers && !safe_grippers_.empty();
    if (!disable_error.empty()) fault_reason_ += "; disable: " + disable_error;
  }
  wakeup_.notify_all();
}

ArticoreSafetyHealth SafetyRuntime::health() const {
  ArticoreSafetyHealth result{};
  const auto now = Clock::now();
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  std::lock_guard<std::mutex> lock(state_mutex_);
  result.state = state_;
  result.safe_holding = state_ == ARTICORE_SAFE_HOLD ||
      state_ == ARTICORE_SAFE_STOP ||
      (state_ == ARTICORE_FAULT && fault_hold_active_);
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
    ArticoreFeedbackStats live_stats{};
    output.feedback_age_ns =
        state_ != ARTICORE_DISCONNECTED &&
                api_.motor_get_feedback_stats(motor.descriptor.motor,
                                              &live_stats) == 0 &&
                live_stats.has_feedback
            ? live_stats.age_ns
            : motor.feedback_age_ns;
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

ArticoreSafetyHealthV2 SafetyRuntime::health_v2() const {
  ArticoreSafetyHealthV2 result{};
  result.struct_size = sizeof(result);
  result.health = health();
  std::lock_guard<std::mutex> lock(state_mutex_);
  result.last_operation = last_operation_;
  result.last_operation_code = last_operation_code_;
  result.operation_failed_motor_count = static_cast<uint32_t>(
      std::min<std::size_t>(operation_failed_motors_.size(), 32));
  for (uint32_t i = 0; i < result.operation_failed_motor_count; ++i) {
    copy_text(result.operation_failed_motors[i], operation_failed_motors_[i]);
  }
  copy_text(result.last_operation_error, last_operation_error_);
  result.degraded = result.health.state == ARTICORE_DEGRADED;
  result.safe_stopped = result.health.state == ARTICORE_SAFE_STOP;
  result.requires_resynchronization =
      result.health.state == ARTICORE_DEGRADED ||
      result.health.state == ARTICORE_SAFE_STOP;
  result.command_scale = result.health.state == ARTICORE_DEGRADED
      ? 0.25f
      : (result.health.state == ARTICORE_SAFE_STOP ? 0.0f : 1.0f);
  copy_text(result.safety_reason, safety_reason_);
  return result;
}

}  // namespace articore

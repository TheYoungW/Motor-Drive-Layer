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
  const char* message = backend_->last_error_message();
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
          backend_->get_feedback_stats(motor.descriptor.motor, &stats) == 0 &&
          stats.has_feedback;
      const bool has_state =
          backend_->get_state(motor.descriptor.motor, &state) == 0 &&
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
          backend_->get_state(command.motor, &state) == 0 && state.has_value;
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
      const auto gain_scale = gripper_gain_scale(*motor);
      safe.stiffness =
          (protection_enabled ? profile.hold_kp : profile.moving_kp) *
          gain_scale;
      safe.damping =
          (protection_enabled ? profile.hold_kd : profile.moving_kd) *
          gain_scale;
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
              backend_->send_pos_vel(controller_group_, side_commands.data(),
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
              backend_->send_mit(controller_group_, side_commands.data(),
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
        rc = backend_->send_pos_vel(controller_group_, pv.data(),
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
        rc = backend_->send_mit(controller_group_, mit.data(),
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

std::vector<ArticoreMotorFeedbackHealth>
SafetyRuntime::collect_motor_feedback_health() const {
  std::vector<ArticoreMotorFeedbackHealth> result;
  result.reserve(motors_.size());
  const uint64_t maximum_age_ns = feedback_max_age_ns();
  for (const auto& motor : motors_) {
    ArticoreMotorFeedbackHealth item{};
    item.side = motor.descriptor.side;
    item.is_gripper = motor.descriptor.is_gripper ? 1U : 0U;
    item.feedback_age_ns = std::numeric_limits<uint64_t>::max();
    copy_text(item.role, motor.descriptor.name);

    ArticoreFeedbackStats stats{};
    ArticoreMotorState state{};
    const bool has_feedback =
        backend_->get_feedback_stats(motor.descriptor.motor, &stats) == 0 &&
        stats.has_feedback;
    const bool has_state =
        backend_->get_state(motor.descriptor.motor, &state) == 0 &&
        state.has_value;
    if (motor.motor_identity_configured) {
      item.can_id = motor.configured_can_id;
      item.can_id_valid = 1;
    } else if (has_state) {
      item.can_id = state.can_id;
      item.can_id_valid = 1;
    }
    item.has_feedback = has_feedback ? 1U : 0U;
    item.has_state = has_state ? 1U : 0U;
    if (has_feedback) {
      item.feedback_age_ns = stats.age_ns;
      item.update_count = stats.update_count;
      item.fresh = stats.age_ns <= maximum_age_ns ? 1U : 0U;
      if (!item.fresh) item.issues |= ARTICORE_FEEDBACK_ISSUE_STALE;
    } else {
      item.issues |= ARTICORE_FEEDBACK_ISSUE_MISSING;
    }
    if (has_state) {
      item.status_code = state.status_code;
      item.position = state.pos;
      item.velocity = state.vel;
      item.torque = state.torq;
      item.values_finite =
          finite(state.pos) && finite(state.vel) && finite(state.torq) ? 1U : 0U;
      if (!item.values_finite) {
        item.issues |= ARTICORE_FEEDBACK_ISSUE_NONFINITE;
      }
      if (state.status_code > 1) {
        item.issues |= ARTICORE_FEEDBACK_ISSUE_MOTOR_FAULT;
      }
    } else {
      item.issues |= ARTICORE_FEEDBACK_ISSUE_STATE_UNAVAILABLE;
    }
    result.push_back(item);
  }
  return result;
}

ArticoreFeedbackIssueScope SafetyRuntime::classify_feedback_issue_scope(
    const std::vector<ArticoreMotorFeedbackHealth>& motors) const {
  uint32_t installed[2]{};
  uint32_t unavailable[2]{};
  uint32_t issue_count = 0;
  constexpr uint32_t availability_issues =
      ARTICORE_FEEDBACK_ISSUE_MISSING |
      ARTICORE_FEEDBACK_ISSUE_STALE |
      ARTICORE_FEEDBACK_ISSUE_STATE_UNAVAILABLE;
  for (const auto& motor : motors) {
    if (motor.side > 1) continue;
    ++installed[motor.side];
    if (motor.issues != ARTICORE_FEEDBACK_ISSUE_NONE) ++issue_count;
    if ((motor.issues & availability_issues) != 0) ++unavailable[motor.side];
  }
  bool channel[2]{};
  for (uint8_t side = 0; side < 2; ++side) {
    if (!active_sides_[side]) continue;
    const bool transport_failed =
        !sides_[side].connected || !sides_[side].transport_healthy;
    const bool all_feedback_unavailable =
        installed[side] > 0 && unavailable[side] == installed[side];
    const bool receive_stalled =
        sides_[side].last_rx_age_ns > feedback_max_age_ns();
    channel[side] = transport_failed ||
        (all_feedback_unavailable && receive_stalled);
  }
  if (channel[0] && channel[1]) return ARTICORE_FEEDBACK_SCOPE_BOTH_CHANNELS;
  if (channel[0]) return ARTICORE_FEEDBACK_SCOPE_LEFT_CHANNEL;
  if (channel[1]) return ARTICORE_FEEDBACK_SCOPE_RIGHT_CHANNEL;
  if (issue_count == 1) return ARTICORE_FEEDBACK_SCOPE_SINGLE_MOTOR;
  if (issue_count > 1) return ARTICORE_FEEDBACK_SCOPE_MULTIPLE_MOTORS;
  return ARTICORE_FEEDBACK_SCOPE_NONE;
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
  std::vector<std::string> side_errors[2];
  auto feedback = collect_motor_feedback_health();
  for (std::size_t index = 0; index < feedback.size(); ++index) {
    auto& item = feedback[index];
    const auto& motor = motors_[index];
    const std::string name(item.role);
    maximum_age[item.side] =
        std::max(maximum_age[item.side], item.feedback_age_ns);

    if (item.has_state && item.status_code <= 1 && recovery_check &&
        item.status_code != 0 &&
        !(allow_held_grippers && item.is_gripper)) {
      item.issues |= ARTICORE_FEEDBACK_ISSUE_UNEXPECTED_POWER_STATE;
      mark_unconfirmed(name);
    }
    if (item.has_state && !recovery_check && item.status_code == 0 &&
        !intentionally_disabled.count(motor.descriptor.motor)) {
      item.issues |= ARTICORE_FEEDBACK_ISSUE_UNEXPECTED_POWER_STATE;
      motor_faults.push_back(name);
      faulted_presence.push_back(motor.descriptor.motor);
    }

    if (item.issues == ARTICORE_FEEDBACK_ISSUE_NONE) continue;
    side_ok[item.side] = false;
    const uint32_t severe_issues =
        ARTICORE_FEEDBACK_ISSUE_NONFINITE |
        ARTICORE_FEEDBACK_ISSUE_MOTOR_FAULT |
        ARTICORE_FEEDBACK_ISSUE_UNEXPECTED_POWER_STATE;
    if ((item.issues & severe_issues) != 0) actionable_failure = true;
    if ((item.issues & ARTICORE_FEEDBACK_ISSUE_MOTOR_FAULT) != 0) {
      motor_faults.push_back(name);
      faulted_presence.push_back(motor.descriptor.motor);
    }
    if (recovery_check &&
        (item.issues & (ARTICORE_FEEDBACK_ISSUE_MISSING |
                        ARTICORE_FEEDBACK_ISSUE_STALE |
                        ARTICORE_FEEDBACK_ISSUE_STATE_UNAVAILABLE |
                        ARTICORE_FEEDBACK_ISSUE_NONFINITE)) != 0) {
      mark_unconfirmed(name);
    }

    std::ostringstream detail;
    detail << "CH" << item.side << "/" << item.role << " (CAN ID ";
    if (item.can_id_valid) detail << item.can_id;
    else detail << "unavailable";
    detail << "): ";
    bool wrote = false;
    const auto append = [&](const std::string& value) {
      if (wrote) detail << ", ";
      detail << value;
      wrote = true;
    };
    if ((item.issues & ARTICORE_FEEDBACK_ISSUE_MISSING) != 0) {
      append("feedback statistics unavailable; actual=missing; "
             "threshold=valid fresh feedback required");
    }
    if ((item.issues & ARTICORE_FEEDBACK_ISSUE_STALE) != 0) {
      append("feedback exceeds maximum age; actual_age_ns=" +
             std::to_string(item.feedback_age_ns) + "; threshold_age_ns<=" +
             std::to_string(feedback_max_age_ns()));
    }
    if ((item.issues & ARTICORE_FEEDBACK_ISSUE_STATE_UNAVAILABLE) != 0) {
      append("motor state unavailable; actual=missing; "
             "threshold=valid motor state required");
    }
    if ((item.issues & ARTICORE_FEEDBACK_ISSUE_NONFINITE) != 0) {
      std::ostringstream values;
      values << "motor feedback is not finite; actual={position="
             << item.position << ", velocity=" << item.velocity
             << ", torque=" << item.torque
             << "}; threshold=all feedback values finite";
      append(values.str());
    }
    if ((item.issues & ARTICORE_FEEDBACK_ISSUE_MOTOR_FAULT) != 0) {
      append("motor fault status reported; actual_status=" +
             std::to_string(item.status_code) + "; threshold_status<=1");
    }
    if ((item.issues & ARTICORE_FEEDBACK_ISSUE_UNEXPECTED_POWER_STATE) != 0) {
      append(recovery_check
                 ? "motor is not disabled; actual_status=" +
                       std::to_string(item.status_code) + "; threshold_status=0"
                 : "motor unexpectedly disabled; actual_status=0; "
                   "threshold_status=1");
    }
    side_errors[item.side].push_back(detail.str());
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
        sides_[side].last_error.clear();
        for (const auto& detail : side_errors[side]) {
          if (!sides_[side].last_error.empty()) {
            sides_[side].last_error += "; ";
          }
          sides_[side].last_error += detail;
        }
        ++sides_[side].feedback_failures;
      } else {
        sides_[side].feedback_failures = 0;
        if (sides_[side].connected && sides_[side].transport_healthy) {
          sides_[side].last_error.clear();
        }
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
  for (uint8_t side = 0; side < 2; ++side) {
    if (!active_sides_[side] || side_ok[side] || side_errors[side].empty()) {
      continue;
    }
    if (!error.empty()) error += "; ";
    error += "CH" + std::to_string(side) + " feedback unhealthy: ";
    for (std::size_t index = 0; index < side_errors[side].size(); ++index) {
      if (index > 0) error += "; ";
      error += side_errors[side][index];
    }
  }
  if (error.empty() && has_motor_faults) {
    error = "motor fault status reported";
  }
  if (error.empty()) {
    error = "not all motors are confirmed disabled";
    if (!unconfirmed_detail.empty()) error += ": " + unconfirmed_detail;
  }
  return false;
}

bool SafetyRuntime::refresh_transport_health(std::string& error) {
  if (!backend_->has_transport_health()) return true;
  bool healthy = true;
  for (uint8_t side = 0; side < 2; ++side) {
    if (!active_sides_[side]) continue;
    ArticoreDriverTransportHealth native{};
    const auto rc = backend_->get_transport_health(controllers_[side], &native);
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
        ARTICORE_MOTION_FAULT,
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
    reset_bimanual_follow_locked();
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
  result.struct_size = sizeof(result);
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
  auto motor_feedback = collect_motor_feedback_health();
  result.motor_feedback_count = static_cast<uint32_t>(
      std::min<std::size_t>(motor_feedback.size(), 32));
  for (uint32_t index = 0; index < result.motor_feedback_count; ++index) {
    auto& item = motor_feedback[index];
    const bool power_expected =
        state_ == ARTICORE_ENABLED || state_ == ARTICORE_RUNNING ||
        state_ == ARTICORE_DEGRADED ||
        state_ == ARTICORE_PARTIALLY_ENABLED ||
        state_ == ARTICORE_SAFE_HOLD || state_ == ARTICORE_SAFE_STOP;
    if (item.has_state && item.status_code == 0 && power_expected &&
        !intentionally_disabled_motors_.count(
            motors_[index].descriptor.motor)) {
      item.issues |= ARTICORE_FEEDBACK_ISSUE_UNEXPECTED_POWER_STATE;
    }
    if (item.issues != ARTICORE_FEEDBACK_ISSUE_NONE) {
      ++result.feedback_issue_count;
    }
    result.motor_feedback[index] = item;
  }
  result.feedback_issue_scope =
      classify_feedback_issue_scope(motor_feedback);
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
                backend_->get_feedback_stats(motor.descriptor.motor,
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
  result.last_operation = last_operation_;
  result.last_operation_code = last_operation_code_;
  result.operation_failed_motor_count = static_cast<uint32_t>(
      std::min<std::size_t>(operation_failed_motors_.size(), 32));
  for (uint32_t i = 0; i < result.operation_failed_motor_count; ++i) {
    copy_text(result.operation_failed_motors[i], operation_failed_motors_[i]);
  }
  copy_text(result.last_operation_error, last_operation_error_);
  result.degraded = result.state == ARTICORE_DEGRADED;
  result.safe_stopped = result.state == ARTICORE_SAFE_STOP;
  result.requires_resynchronization =
      result.state == ARTICORE_DEGRADED || result.state == ARTICORE_SAFE_STOP;
  result.command_scale = result.state == ARTICORE_DEGRADED
      ? 0.25f
      : (result.state == ARTICORE_SAFE_STOP ? 0.0f : 1.0f);
  copy_text(result.safety_reason, safety_reason_);
  result.control_ticks = control_ticks_.load(std::memory_order_relaxed);
  result.control_overruns = control_overruns_.load(std::memory_order_relaxed);
  result.maximum_control_period_ns =
      maximum_control_period_ns_.load(std::memory_order_relaxed);
  result.maximum_send_time_ns =
      maximum_send_time_ns_.load(std::memory_order_relaxed);
  return result;
}

}  // namespace articore

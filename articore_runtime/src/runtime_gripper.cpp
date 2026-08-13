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
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    require_state_for_command();
  }
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

  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    require_state_for_command();
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

void SafetyRuntime::seed_gripper_targets_from_feedback(bool activate) {
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
      motor.has_gripper_target = activate;
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
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (hardware_transition_ ||
        (state_ != ARTICORE_ENABLED && state_ != ARTICORE_RUNNING)) {
      return true;
    }
  }
  const auto now = Clock::now();
  for (auto& motor : motors_) {
    if (!motor.descriptor.is_gripper || !motor.has_gripper_target) continue;
    ArticoreFeedbackStats stats{};
    ArticoreMotorState state{};
    if (api_.motor_get_feedback_stats(motor.descriptor.motor, &stats) != 0 ||
        !stats.has_feedback ||
        api_.motor_get_state(motor.descriptor.motor, &state) != 0 ||
        !state.has_value) {
      ++motor.consecutive_feedback_failures;
      if (motor.consecutive_feedback_failures >=
          config_.feedback_failure_threshold) {
        motor.gripper_state = ARTICORE_GRIPPER_FAULT;
        motor.gripper_fault_reason =
            motor_error("gripper feedback unavailable");
        mark_motor_faulted(motor.descriptor.motor);
        error = std::string(motor.descriptor.name) + ": " +
                motor.gripper_fault_reason;
        return false;
      }
      {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        const auto previous = std::find_if(
            safe_grippers_.begin(), safe_grippers_.end(),
            [&](const ArticoreMitCommand& command) {
              return command.motor == motor.descriptor.motor;
            });
        if (previous != safe_grippers_.end()) commands.push_back(*previous);
      }
      continue;
    }
    motor.consecutive_feedback_failures = 0;
    motor.feedback_age_ns = stats.age_ns;
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

bool SafetyRuntime::send_gripper_hold_once(std::string& error) {
  std::vector<ArticoreMitCommand> commands;
  ArticoreSafetyState hold_state;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    commands = safe_grippers_;
    hold_state = state_;
  }
  if (commands.empty()) return true;

  std::lock_guard<std::mutex> command_lock(command_mutex_);
  const auto now = Clock::now();
  std::vector<ArticoreMitCommand> sendable;
  sendable.reserve(commands.size());
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
      ++found->consecutive_feedback_failures;
      sendable.push_back(command);
      continue;
    }
    found->consecutive_feedback_failures = 0;
    found->feedback_age_ns = stats.age_ns;
    found->last_position = state.pos;
    found->last_torque = state.torq;
    found->has_position = true;
    if (state.status_code == 0 || state.status_code > 1) {
      found->gripper_state = ARTICORE_GRIPPER_FAULT;
      found->gripper_fault_reason = state.status_code == 0
          ? "safe-hold gripper unexpectedly disabled"
          : "safe-hold gripper motor fault";
      mark_motor_faulted(command.motor);
      continue;
    }
    if (stats.age_ns >
        static_cast<uint64_t>(config_.feedback_max_age_ms) * 1'000'000ULL) {
      ++found->consecutive_feedback_failures;
      sendable.push_back(command);
      continue;
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
    sendable.push_back(command);
  }
  if (sendable.empty()) {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    safe_grippers_.clear();
    return true;
  }
  bool sent = true;
  if (hold_state == ARTICORE_FAULT) {
    for (uint8_t side = 0; side < 2; ++side) {
      std::vector<ArticoreMitCommand> side_commands;
      for (const auto& command : sendable) {
        const auto record = std::find_if(
            motors_.begin(), motors_.end(), [&](const MotorRecord& value) {
              return value.descriptor.motor == command.motor;
            });
        if (record != motors_.end() && record->descriptor.side == side) {
          side_commands.push_back(command);
        }
      }
      if (side_commands.empty()) continue;
      if (api_.group_send_mit(controller_group_, side_commands.data(),
                              static_cast<uint32_t>(side_commands.size())) != 0) {
        sent = false;
        if (!error.empty()) error += "; ";
        error += "CH" + std::to_string(side) +
                 " protective gripper hold send failed";
      }
    }
  } else if (api_.group_send_mit(
                 controller_group_, sendable.data(),
                 static_cast<uint32_t>(sendable.size())) != 0) {
    sent = false;
    error = motor_error("safe-hold gripper send failed");
  }
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    safe_grippers_ = sendable;
  }
  return sent;
}

}  // namespace articore

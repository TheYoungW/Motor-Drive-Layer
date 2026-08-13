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

bool SafetyRuntime::request_feedback_parallel(
    uint32_t timeout_ms, std::vector<MissingMotor>& missing_motors,
    std::string& error) {
  struct SideResult {
    int32_t code = 0;
    ArticoreFeedbackReport report{};
    std::vector<uint32_t> missing;
    std::string error;
  } results[2];
  std::vector<std::thread> workers;
  for (uint8_t side = 0; side < 2; ++side) {
    if (!active_sides_[side]) continue;
    workers.emplace_back([&, side] {
      const auto motor_count = static_cast<uint32_t>(std::count_if(
          motors_.begin(), motors_.end(), [&](const MotorRecord& motor) {
            return motor.descriptor.side == side;
          }));
      auto& result = results[side];
      result.missing.assign(std::max<uint32_t>(motor_count, 1),
                            std::numeric_limits<uint32_t>::max());
      result.report.struct_size = sizeof(result.report);
      result.code = api_.controller_request_feedback_all_ex(
          controllers_[side], timeout_ms, &result.report,
          result.missing.data(), static_cast<uint32_t>(result.missing.size()));
      const auto reported = std::min<std::size_t>(
          result.report.missing_count, result.missing.size());
      result.missing.resize(reported);
      result.missing.erase(
          std::remove(result.missing.begin(), result.missing.end(),
                      std::numeric_limits<uint32_t>::max()),
          result.missing.end());
      if (result.code != 0) {
        const char* detail = api_.last_error_message();
        if (detail && detail[0]) result.error = detail;
      }
    });
  }
  for (auto& worker : workers) worker.join();

  bool ok = true;
  missing_motors.clear();
  for (uint8_t side = 0; side < 2; ++side) {
    if (!active_sides_[side]) continue;
    const auto& result = results[side];
    for (const auto id : result.missing) {
      missing_motors.push_back(MissingMotor{side, id});
    }
    if (result.code == 0) continue;
    ok = false;
    // A transport may report a side-wide timeout without filling the optional
    // ID array. Preserve deterministic per-motor diagnostics in that case.
    if (result.missing.empty()) {
      for (const auto& motor : motors_) {
        if (motor.descriptor.side != side) continue;
        ArticoreMotorState state{};
        const uint32_t id =
            api_.motor_get_state(motor.descriptor.motor, &state) == 0 &&
                    state.has_value
                ? state.can_id
                : 0;
        missing_motors.push_back(MissingMotor{side, id});
      }
    }
    if (!error.empty()) error += "; ";
    error += std::string(side == 0 ? "CH0" : "CH1") +
             " feedback code=" + std::to_string(result.code) +
             ", expected=" + std::to_string(result.report.expected_count) +
             ", received=" + std::to_string(result.report.received_count) +
             ", missing=" + std::to_string(result.report.missing_count);
    if (!result.missing.empty()) {
      error += ", missing motor IDs:";
      for (const auto id : result.missing) {
        error += " " + std::to_string(id);
      }
    }
    if (!result.error.empty()) error += ": " + result.error;
  }
  return ok;
}

bool SafetyRuntime::confirm_enabled_feedback(
    Clock::time_point deadline, std::vector<MissingMotor>& missing_motors,
    std::string& error) {
  std::string latest_error;
  std::set<void*> retried_motors;
  while (Clock::now() < deadline) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - Clock::now());
    const auto timeout_ms = static_cast<uint32_t>(std::max<int64_t>(
        1, std::min<int64_t>(config_.disable_feedback_timeout_ms,
                             remaining.count())));
    std::string request_error;
    std::vector<MissingMotor> current_missing;
    const bool complete = request_feedback_parallel(
        timeout_ms, current_missing, request_error);
    missing_motors = std::move(current_missing);
    if (!request_error.empty()) latest_error = request_error;

    bool all_enabled = complete;
    std::string state_error;
    std::vector<const MotorRecord*> retry_by_side[2];
    for (const auto& motor : motors_) {
      ArticoreFeedbackStats stats{};
      ArticoreMotorState state{};
      const std::string name(motor.descriptor.name);
      if (api_.motor_get_feedback_stats(motor.descriptor.motor, &stats) != 0 ||
          !stats.has_feedback ||
          stats.age_ns > static_cast<uint64_t>(config_.feedback_max_age_ms) *
                             1'000'000ULL ||
          api_.motor_get_state(motor.descriptor.motor, &state) != 0 ||
          !state.has_value) {
        all_enabled = false;
        if (state_error.empty()) {
          state_error = std::string(motor.descriptor.side == 0 ? "CH0/" : "CH1/") +
                        name + ": enabled feedback is missing or stale";
        }
        continue;
      }
      if (state.status_code > 1) {
        error = std::string(motor.descriptor.side == 0 ? "CH0/" : "CH1/") +
                name + ": motor ID " + std::to_string(state.can_id) +
                " reported status 0x";
        std::ostringstream status;
        status << std::hex << static_cast<unsigned>(state.status_code);
        error += status.str();
        return false;
      }
      if (state.status_code != 1) {
        all_enabled = false;
        if (motor_enable_ &&
            retried_motors.insert(motor.descriptor.motor).second) {
          retry_by_side[motor.descriptor.side].push_back(&motor);
        }
        if (state_error.empty()) {
          state_error = std::string(motor.descriptor.side == 0 ? "CH0/" : "CH1/") +
                        name + ": motor ID " + std::to_string(state.can_id) +
                        " is not enabled";
        }
      }
    }
    if (all_enabled) return true;
    if (!state_error.empty()) latest_error = state_error;
    struct RetryResult {
      std::string error;
    } retry_results[2];
    std::vector<std::thread> retry_workers;
    for (uint8_t side = 0; side < 2; ++side) {
      if (retry_by_side[side].empty()) continue;
      retry_workers.emplace_back([&, side] {
        for (const auto* motor : retry_by_side[side]) {
          if (motor_enable_(motor->descriptor.motor) == 0) continue;
          const char* detail = api_.last_error_message();
          retry_results[side].error =
              std::string(side == 0 ? "CH0/" : "CH1/") +
              motor->descriptor.name + ": one-shot enable retry failed";
          if (detail && detail[0]) retry_results[side].error += ": " +
              std::string(detail);
          break;
        }
      });
    }
    for (auto& worker : retry_workers) worker.join();
    for (const auto& result : retry_results) {
      if (result.error.empty()) continue;
      error = result.error;
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  error = latest_error.empty()
      ? "enable grace expired before all motors reported ENABLED"
      : latest_error;
  return false;
}

void SafetyRuntime::update_enable_report(
    bool success, bool disable_confirmed,
    const std::vector<MissingMotor>& missing_motors,
    const std::string& error) {
  ArticoreEnableReport report{};
  report.struct_size = sizeof(report);
  report.success = success ? 1 : 0;
  report.disable_confirmed = disable_confirmed ? 1 : 0;
  report.expected_count = static_cast<uint32_t>(motors_.size());
  report.missing_count = static_cast<uint32_t>(
      std::min<std::size_t>(missing_motors.size(), 32));
  for (uint32_t i = 0; i < report.missing_count; ++i) {
    report.missing_motor_sides[i] = missing_motors[i].side;
    report.missing_motor_ids[i] = missing_motors[i].id;
  }
  report.motor_count = static_cast<uint32_t>(
      std::min<std::size_t>(motors_.size(), 32));
  for (uint32_t i = 0; i < report.motor_count; ++i) {
    const auto& motor = motors_[i];
    auto& output = report.motors[i];
    output.side = motor.descriptor.side;
    copy_text(output.name, std::string(motor.descriptor.name));
    ArticoreFeedbackStats stats{};
    ArticoreMotorState state{};
    const bool has_stats =
        api_.motor_get_feedback_stats(motor.descriptor.motor, &stats) == 0 &&
        stats.has_feedback;
    const bool has_state =
        api_.motor_get_state(motor.descriptor.motor, &state) == 0 &&
        state.has_value;
    output.has_feedback = has_stats && has_state ? 1 : 0;
    output.feedback_fresh = output.has_feedback &&
        stats.age_ns <= static_cast<uint64_t>(config_.feedback_max_age_ms) *
                            1'000'000ULL;
    bool failed = !output.has_feedback || !output.feedback_fresh;
    if (has_state) {
      output.can_id = state.can_id;
      output.status_code = state.status_code;
      output.enabled = state.status_code == 1 ? 1 : 0;
      if (output.enabled) ++report.enabled_count;
      if (!output.enabled) failed = true;
    }
    if (failed) ++report.failure_count;
  }
  copy_text(report.error, error);
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_enable_report_ = report;
}

ArticoreEnableReport SafetyRuntime::last_enable_report() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_enable_report_;
}

void SafetyRuntime::initialize_enabled_state(ArticoreControlMode mode) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  mode_ = mode;
  state_ = ARTICORE_ENABLED;
  disable_confirmed_ = false;
  has_successful_command_ = false;
  gripper_command_generation_ = 0;
  gripper_sent_generation_ = 0;
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
    if (motor.descriptor.is_gripper) {
      motor.requested_speed = 1000.0f;
      motor.command_speed = motor.descriptor.close_speed;
      motor.force_level = ARTICORE_GRIPPER_FORCE_NORMAL;
    }
  }
  next_feedback_check_ = enabled_at_;
  next_gripper_control_ = enabled_at_;
  next_control_tick_ = enabled_at_;
}

bool SafetyRuntime::send_initial_hold(ArticoreControlMode mode,
                                      std::string& error) {
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  const int32_t arm_result = mode == ARTICORE_MODE_PV
      ? api_.group_send_pos_vel(controller_group_, arm_mailbox_.pv.data(),
                                static_cast<uint32_t>(arm_mailbox_.pv.size()))
      : api_.group_send_mit(controller_group_, arm_mailbox_.mit.data(),
                           static_cast<uint32_t>(arm_mailbox_.mit.size()));
  if (arm_result != 0) {
    error = motor_error("initial arm hold send failed");
    return false;
  }
  if (mode == ARTICORE_MODE_PV) {
    last_sent_pv_ = arm_mailbox_.pv;
    last_sent_mit_.clear();
  } else {
    last_sent_mit_ = arm_mailbox_.mit;
    last_sent_pv_.clear();
  }
  if (!safe_grippers_.empty() &&
      api_.group_send_mit(controller_group_, safe_grippers_.data(),
                          static_cast<uint32_t>(safe_grippers_.size())) != 0) {
    error = motor_error("initial gripper hold send failed");
    return false;
  }
  return true;
}

void SafetyRuntime::enable(ArticoreControlMode mode) {
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  if (mode != ARTICORE_MODE_PV && mode != ARTICORE_MODE_MIT) {
    throw std::invalid_argument("control mode must be PV or MIT");
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ != ARTICORE_READY || fault_latched_ || hardware_transition_) {
      throw std::runtime_error("Articore runtime can only enable from READY");
    }
    hardware_transition_ = true;
    enable_transaction_ = true;
  }

  // Direct C++ users may omit the native enable callback. Preserve the 1.3
  // contract for them while the exported runtime ABI always supplies it.
  if (!controller_enable_all_) {
    try {
      {
        std::lock_guard<std::mutex> command_lock(command_mutex_);
        initialize_arm_mailbox_from_feedback(mode, true);
      }
      initialize_enabled_state(mode);
      seed_gripper_targets_from_feedback(false);
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        hardware_transition_ = false;
        enable_transaction_ = false;
      }
      update_enable_report(true, false, {}, "");
      wakeup_.notify_all();
      return;
    } catch (...) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      hardware_transition_ = false;
      enable_transaction_ = false;
      throw;
    }
  }

  std::vector<MissingMotor> missing_motors;
  std::string enable_error;
  try {
    if (!request_feedback_parallel(config_.disable_feedback_timeout_ms,
                                   missing_motors, enable_error)) {
      throw std::runtime_error("initial position feedback failed: " + enable_error);
    }
    {
      std::lock_guard<std::mutex> command_lock(command_mutex_);
      initialize_arm_mailbox_from_feedback(mode, false);
    }

    struct EnableSideResult {
      int32_t code = 0;
      std::string error;
    } side_results[2];
    std::vector<std::thread> enable_workers;
    for (uint8_t side = 0; side < 2; ++side) {
      if (!active_sides_[side]) continue;
      enable_workers.emplace_back([&, side] {
        side_results[side].code = controller_enable_all_(controllers_[side]);
        if (side_results[side].code != 0) {
          const char* detail = api_.last_error_message();
          if (detail && detail[0]) side_results[side].error = detail;
        }
      });
    }
    for (auto& worker : enable_workers) worker.join();
    for (uint8_t side = 0; side < 2; ++side) {
      if (!active_sides_[side] || side_results[side].code == 0) continue;
      if (!enable_error.empty()) enable_error += "; ";
      enable_error += std::string(side == 0 ? "CH0" : "CH1") +
                      " enable failed";
      if (!side_results[side].error.empty()) {
        enable_error += ": " + side_results[side].error;
      }
    }
    if (!enable_error.empty()) throw std::runtime_error(enable_error);

    initialize_enabled_state(mode);
    seed_gripper_targets_from_feedback(true);
    if (!send_initial_hold(mode, enable_error)) {
      throw std::runtime_error(enable_error);
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      hardware_transition_ = false;
    }
    wakeup_.notify_all();

    const auto deadline = Clock::now() +
        std::chrono::milliseconds(config_.enable_grace_ms);
    if (!confirm_enabled_feedback(deadline, missing_motors, enable_error)) {
      throw std::runtime_error(enable_error);
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      enable_transaction_ = false;
    }
    update_enable_report(true, false, missing_motors, "");
  } catch (const std::exception& failure) {
    enable_error = failure.what();
    {
      std::lock_guard<std::mutex> command_lock(command_mutex_);
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_ = ARTICORE_FAULT;
      fault_latched_ = true;
      hardware_transition_ = true;
      disable_confirmed_ = false;
      fault_reason_ = "runtime enable failed: " + enable_error;
      arm_mailbox_ = ArmMailbox{};
      cancel_active_trajectory_locked(
          ARTICORE_TRAJECTORY_FAILED, fault_reason_);
    }
    update_enable_report(false, false, missing_motors, enable_error);
    std::string disable_error;
    const bool disabled = disable_hardware(true, false, disable_error);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      disable_confirmed_ = disabled;
      last_enable_report_.disable_confirmed = disabled ? 1 : 0;
      hardware_transition_ = false;
      enable_transaction_ = false;
      if (!disable_error.empty()) fault_reason_ += "; rollback: " + disable_error;
    }
    wakeup_.notify_all();
    std::string message = enable_error;
    if (!disable_error.empty()) message += "; rollback: " + disable_error;
    throw std::runtime_error(message);
  }
}

bool SafetyRuntime::disable_hardware(bool request_feedback,
                                     bool preserve_grippers,
                                     std::string& error) {
  bool ok = true;
  bool barrier_confirmed = false;
  std::vector<void*> initially_sent;
  std::vector<void*> retried;
  std::vector<MissingMotor> unconfirmed;
  bool retry_attempted = false;
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
    std::string barrier_error;
    barrier_confirmed = establish_disable_barrier(barrier_error);
    if (!barrier_confirmed) {
      ok = false;
      error = "disable queue barrier failed: " + barrier_error;
    }

    auto send_disable_parallel = [&](const std::vector<MotorRecord*>& targets,
                                     std::vector<void*>& sent) {
      struct SideResult {
        std::vector<void*> sent;
        std::vector<std::string> errors;
      } side_results[2];
      std::vector<std::thread> workers;
      for (uint8_t side = 0; side < 2; ++side) {
        workers.emplace_back([&, side] {
          for (auto* motor : targets) {
            if (motor->descriptor.side != side) continue;
            if (api_.motor_disable(motor->descriptor.motor) == 0) {
              side_results[side].sent.push_back(motor->descriptor.motor);
              continue;
            }
            const auto detail = motor_error("motor disable failed");
            side_results[side].errors.push_back(
                std::string(side == 0 ? "CH0/" : "CH1/") +
                motor->descriptor.name + ": " + detail);
          }
        });
      }
      for (auto& worker : workers) worker.join();
      for (uint8_t side = 0; side < 2; ++side) {
        sent.insert(sent.end(), side_results[side].sent.begin(),
                    side_results[side].sent.end());
        for (const auto& detail : side_results[side].errors) {
          if (!error.empty()) error += "; ";
          error += detail;
          std::lock_guard<std::mutex> state_lock(state_mutex_);
          set_side_error_locked(side, detail, true);
        }
      }
    };

    std::vector<MotorRecord*> targets;
    for (auto& motor : motors_) {
      if (preserve_grippers && motor.descriptor.is_gripper) continue;
      targets.push_back(&motor);
    }
    send_disable_parallel(targets, initially_sent);

    if (request_feedback) {
      auto collect_unconfirmed = [&](bool complete,
                                     const std::vector<MissingMotor>& missing) {
        std::vector<MotorRecord*> result;
        const auto max_age =
            static_cast<uint64_t>(config_.feedback_max_age_ms) * 1'000'000ULL;
        for (auto* motor : targets) {
          ArticoreFeedbackStats stats{};
          ArticoreMotorState state{};
          const bool has_stats =
              api_.motor_get_feedback_stats(motor->descriptor.motor, &stats) == 0 &&
              stats.has_feedback;
          const bool has_state =
              api_.motor_get_state(motor->descriptor.motor, &state) == 0 &&
              state.has_value;
          const bool specifically_missing = has_state && std::any_of(
              missing.begin(), missing.end(), [&](const MissingMotor& item) {
                return item.side == motor->descriptor.side &&
                       item.id == state.can_id;
              });
          if (!complete && missing.empty()) {
            result.push_back(motor);
          } else if (specifically_missing || !has_stats || stats.age_ns > max_age ||
                     !has_state || state.status_code != 0) {
            result.push_back(motor);
          }
        }
        return result;
      };

      auto confirm_until_deadline = [&](std::string& latest_error) {
        std::vector<MotorRecord*> result = targets;
        const auto deadline = Clock::now() +
            std::chrono::milliseconds(config_.disable_feedback_timeout_ms);
        do {
          const auto remaining =
              std::chrono::duration_cast<std::chrono::milliseconds>(
                  deadline - Clock::now());
          const auto timeout_ms = static_cast<uint32_t>(
              std::max<int64_t>(1, remaining.count()));
          std::vector<MissingMotor> missing;
          std::string request_error;
          const bool complete = request_feedback_parallel(
              timeout_ms, missing, request_error);
          result = collect_unconfirmed(complete, missing);
          if (result.empty()) return result;
          if (!request_error.empty()) latest_error = request_error;
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } while (Clock::now() < deadline);
        return result;
      };

      std::string first_error;
      auto retry_targets = confirm_until_deadline(first_error);
      if (!retry_targets.empty()) {
        retry_attempted = true;
        // Exactly one directed retry: only motors whose fresh disabled state
        // was not confirmed are addressed again.
        send_disable_parallel(retry_targets, retried);
        std::string retry_error;
        unconfirmed.clear();
        const auto final_targets = confirm_until_deadline(retry_error);
        for (auto* motor : final_targets) {
          ArticoreMotorState state{};
          const uint32_t id =
              api_.motor_get_state(motor->descriptor.motor, &state) == 0 &&
                      state.has_value
                  ? state.can_id
                  : 0;
          unconfirmed.push_back({motor->descriptor.side, id});
        }
        if (!retry_error.empty() && !unconfirmed.empty()) first_error = retry_error;
      } else {
        unconfirmed.clear();
      }

      if (!unconfirmed.empty()) {
        if (!error.empty()) error += "; ";
        error += "disable feedback confirmation failed";
        if (!first_error.empty()) error += ": " + first_error;
        error += "; unconfirmed motors:";
        for (const auto& item : unconfirmed) {
          error += std::string(" CH") + std::to_string(item.side) + "/ID" +
                   std::to_string(item.id);
        }
      }
    }
    if (request_feedback) {
      ok = barrier_confirmed && unconfirmed.empty();
    } else {
      ok = barrier_confirmed && initially_sent.size() == targets.size();
    }
    for (auto* motor : targets) {
      if (!motor->descriptor.is_gripper) continue;
      ArticoreMotorState state{};
      if (api_.motor_get_state(motor->descriptor.motor, &state) == 0 &&
          state.has_value && state.status_code == 0) {
        motor->gripper_state = ARTICORE_GRIPPER_DISABLED;
      }
    }
  }
  update_disable_report(ok, barrier_confirmed, unconfirmed, initially_sent,
                        retried, retry_attempted, preserve_grippers, error);
  return ok;
}

bool SafetyRuntime::establish_disable_barrier(std::string& error) {
  // Empty group dispatch acquires the shared ControllerGroup/controller locks
  // and waits for every persistent group worker. A subsequent fresh feedback
  // burst is an end-to-end marker: all previously accepted control frames are
  // ahead of it in the USB/CAN FIFO, so disable frames submitted afterwards
  // cannot be followed by an old Runtime motion frame.
  const int32_t dispatch = mode_ == ARTICORE_MODE_MIT
      ? api_.group_send_mit(controller_group_, nullptr, 0)
      : api_.group_send_pos_vel(controller_group_, nullptr, 0);
  if (dispatch != 0) {
    error = motor_error("controller group drain failed");
    return false;
  }
  std::string latest_error;
  for (uint32_t attempt = 0; attempt < 2; ++attempt) {
    std::vector<MissingMotor> missing;
    std::string request_error;
    if (request_feedback_parallel(config_.disable_feedback_timeout_ms,
                                  missing, request_error)) {
      return true;
    }
    latest_error = request_error;
  }
  error = latest_error.empty() ? "feedback marker timed out" : latest_error;
  return false;
}

void SafetyRuntime::update_disable_report(
    bool success, bool barrier_confirmed,
    const std::vector<MissingMotor>& missing_motors,
    const std::vector<void*>& initially_sent,
    const std::vector<void*>& retried,
    bool retry_attempted,
    bool preserve_grippers,
    const std::string& error) {
  ArticoreDisableReport report{};
  report.struct_size = sizeof(report);
  report.success = success ? 1 : 0;
  report.barrier_confirmed = barrier_confirmed ? 1 : 0;
  report.expected_count = static_cast<uint32_t>(std::count_if(
      motors_.begin(), motors_.end(), [&](const MotorRecord& motor) {
        return !(preserve_grippers && motor.descriptor.is_gripper);
      }));
  report.retry_count = retry_attempted ? 1 : 0;
  report.missing_count = static_cast<uint32_t>(
      std::min<std::size_t>(missing_motors.size(), 32));
  for (uint32_t i = 0; i < report.missing_count; ++i) {
    report.missing_motor_sides[i] = missing_motors[i].side;
    report.missing_motor_ids[i] = missing_motors[i].id;
  }
  report.motor_count = static_cast<uint32_t>(
      std::min<std::size_t>(motors_.size(), 32));
  const auto max_age =
      static_cast<uint64_t>(config_.feedback_max_age_ms) * 1'000'000ULL;
  for (uint32_t i = 0; i < report.motor_count; ++i) {
    const auto& motor = motors_[i];
    auto& output = report.motors[i];
    output.side = motor.descriptor.side;
    copy_text(output.name, std::string(motor.descriptor.name));
    output.disable_sent = std::find(initially_sent.begin(), initially_sent.end(),
                                    motor.descriptor.motor) != initially_sent.end();
    output.retry_sent = std::find(retried.begin(), retried.end(),
                                  motor.descriptor.motor) != retried.end();
    ArticoreFeedbackStats stats{};
    ArticoreMotorState state{};
    const bool has_stats =
        api_.motor_get_feedback_stats(motor.descriptor.motor, &stats) == 0 &&
        stats.has_feedback;
    const bool has_state =
        api_.motor_get_state(motor.descriptor.motor, &state) == 0 && state.has_value;
    output.has_feedback = has_stats && has_state ? 1 : 0;
    output.feedback_fresh = output.has_feedback && stats.age_ns <= max_age;
    if (has_state) {
      output.can_id = state.can_id;
      output.status_code = state.status_code;
    }
    const bool excluded = preserve_grippers && motor.descriptor.is_gripper;
    const bool explicitly_missing = std::any_of(
        missing_motors.begin(), missing_motors.end(), [&](const MissingMotor& item) {
          return item.side == output.side && item.id == output.can_id;
        });
    output.disabled = !excluded && output.has_feedback && output.feedback_fresh &&
                      !explicitly_missing && output.status_code == 0;
    if (output.disabled) ++report.disabled_count;
    if (!excluded && !output.disabled) {
      ++report.failure_count;
    }
  }
  copy_text(report.error, error);
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_disable_report_ = report;
  unconfirmed_disable_.clear();
  for (const auto& item : missing_motors) {
    unconfirmed_disable_.push_back(
        std::string("CH") + std::to_string(item.side) + "/ID" +
        std::to_string(item.id));
  }
}

ArticoreDisableReport SafetyRuntime::last_disable_report() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_disable_report_;
}

void SafetyRuntime::disable() {
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  bool preserve_fault = false;
  std::string preserved_reason;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ == ARTICORE_DISCONNECTED) {
      throw std::runtime_error("cannot disable a disconnected runtime");
    }
    preserve_fault = state_ == ARTICORE_FAULT && fault_latched_;
    preserved_reason = fault_reason_;
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
    last_sent_pv_.clear();
    last_sent_mit_.clear();
    fault_hold_active_ = false;
    arm_mailbox_ = ArmMailbox{};
    cancel_active_trajectory_locked(
        confirmed ? ARTICORE_TRAJECTORY_CANCELED
                  : ARTICORE_TRAJECTORY_FAILED,
        confirmed ? "runtime disabled" : "disable confirmation failed");
    has_successful_command_ = false;
    gripper_command_generation_ = 0;
    gripper_sent_generation_ = 0;
    hardware_transition_ = false;
    if (confirmed) {
      if (preserve_fault) {
        state_ = ARTICORE_FAULT;
        fault_latched_ = true;
        fault_reason_ = preserved_reason;
      } else {
        state_ = ARTICORE_READY;
        fault_reason_.clear();
      }
    } else {
      state_ = ARTICORE_FAULT;
      fault_latched_ = true;
      fault_reason_ = preserve_fault && !preserved_reason.empty()
          ? preserved_reason + "; disable confirmation failed: " + error
          : "disable confirmation failed: " + error;
    }
  }
  if (!confirmed) throw std::runtime_error(error);
}

void SafetyRuntime::estop(const std::string& reason) {
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  enter_fault(reason.empty() ? "emergency stop" : reason, true);
}

void SafetyRuntime::recover() {
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
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
    if (!disable_confirmed_) {
      throw std::runtime_error(
          "physical disable must be confirmed before recover");
    }
    hardware_transition_ = true;
  }
  std::string error;
  std::vector<MissingMotor> missing_motors;
  if (!request_feedback_parallel(config_.disable_feedback_timeout_ms,
                                 missing_motors, error)) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    hardware_transition_ = false;
    throw std::runtime_error(error);
  }
  {
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    if (!refresh_feedback_health(true, false, error)) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      hardware_transition_ = false;
      throw std::runtime_error(error);
    }
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
  last_sent_pv_.clear();
  last_sent_mit_.clear();
  fault_hold_active_ = false;
  arm_mailbox_ = ArmMailbox{};
  cancel_active_trajectory_locked(
      ARTICORE_TRAJECTORY_CANCELED, "runtime recovered");
  has_successful_command_ = false;
  gripper_command_generation_ = 0;
  gripper_sent_generation_ = 0;
  hardware_transition_ = false;
  for (auto& entry : presence_) {
    if (entry.second == ARTICORE_FAULTED) entry.second = ARTICORE_PRESENT;
  }
}

}  // namespace articore

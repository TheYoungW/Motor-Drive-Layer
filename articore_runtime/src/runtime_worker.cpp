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

void SafetyRuntime::worker_loop() {
  const auto control_period = std::chrono::nanoseconds(
      1'000'000'000ULL / config_.control_hz);
  const auto idle_poll_period = std::chrono::milliseconds(2);
  const auto feedback_period = std::chrono::nanoseconds(
      1'000'000'000ULL / config_.feedback_check_hz);
  const auto ready_feedback_period = std::chrono::nanoseconds(
      1'000'000'000ULL / std::min(config_.feedback_check_hz, 10U));
  const auto hold_period = std::chrono::nanoseconds(
      1'000'000'000ULL / config_.safe_hold_hz);
  // Product grippers are part of the normal control surface. While ENABLED
  // RUNNING or DEGRADED they use the same cycle as the arm; protective stop
  // states use the separately configured safe-hold rate.
  const auto gripper_period = control_period;
  for (;;) {
    {
      std::unique_lock<std::mutex> lock(state_mutex_);
      const auto now = Clock::now();
      auto deadline = now + idle_poll_period;
      if (!hardware_transition_ &&
          (state_ == ARTICORE_READY ||
           state_ == ARTICORE_PARTIALLY_ENABLED) &&
          next_ready_feedback_ != Clock::time_point{}) {
        deadline = std::min(deadline, next_ready_feedback_);
      } else if (!hardware_transition_ &&
                 (state_ == ARTICORE_ENABLED ||
                  state_ == ARTICORE_RUNNING ||
                  state_ == ARTICORE_DEGRADED) &&
                 next_control_tick_ != Clock::time_point{}) {
        deadline = std::min(deadline, next_control_tick_);
      }
      wakeup_.wait_until(lock, deadline, [&] { return stopping_; });
      if (stopping_) return;
      if (hardware_transition_) continue;
    }

    const auto now = Clock::now();
    bool grace_fault = false;
    bool command_timeout = false;
    bool run_ready_feedback = false;
    bool run_feedback_check = false;
    bool run_arm_control = false;
    bool run_hold = false;
    bool run_gripper_control = false;
    bool combine_mit_arm_and_gripper = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (state_ == ARTICORE_ENABLED &&
          !enable_transaction_ &&
          now - enabled_at_ >= std::chrono::milliseconds(config_.enable_grace_ms) &&
          !has_successful_command_) {
        grace_fault = true;
      } else if ((state_ == ARTICORE_RUNNING ||
                  state_ == ARTICORE_DEGRADED) &&
                 has_successful_command_ &&
                 arm_mailbox_.user_command &&
                 arm_mailbox_.lifetime == ARTICORE_COMMAND_STREAMING &&
                 now - last_successful_command_ >=
                     std::chrono::milliseconds(config_.command_timeout_ms)) {
        command_timeout = true;
      }
      if ((state_ == ARTICORE_ENABLED || state_ == ARTICORE_RUNNING ||
           state_ == ARTICORE_DEGRADED) &&
          now >= next_control_tick_) {
        run_arm_control = true;
        detail::advance_periodic_deadline(
            next_control_tick_, control_period, now);
      }
      if ((state_ == ARTICORE_READY ||
           state_ == ARTICORE_PARTIALLY_ENABLED) &&
          now >= next_ready_feedback_) {
        run_ready_feedback = true;
        detail::advance_periodic_deadline(
            next_ready_feedback_, ready_feedback_period, now);
      }
      if ((state_ == ARTICORE_RUNNING || state_ == ARTICORE_DEGRADED ||
           state_ == ARTICORE_SAFE_HOLD ||
           state_ == ARTICORE_SAFE_STOP) &&
          now >= next_feedback_check_) {
        run_feedback_check = true;
        detail::advance_periodic_deadline(
            next_feedback_check_, feedback_period, now);
      }
      if ((state_ == ARTICORE_SAFE_HOLD ||
           state_ == ARTICORE_SAFE_STOP) && now >= next_safe_hold_) {
        run_hold = true;
        detail::advance_periodic_deadline(next_safe_hold_, hold_period, now);
      }
      if (!enable_transaction_ &&
          (state_ == ARTICORE_ENABLED || state_ == ARTICORE_RUNNING ||
           state_ == ARTICORE_DEGRADED) &&
          now >= next_gripper_control_ &&
          std::any_of(motors_.begin(), motors_.end(),
                      [](const MotorRecord& motor) {
                        return motor.descriptor.is_gripper &&
                               motor.has_gripper_target;
                      })) {
        run_gripper_control = true;
        detail::advance_periodic_deadline(
            next_gripper_control_, gripper_period, now);
      }
      combine_mit_arm_and_gripper =
          run_arm_control && run_gripper_control && mode_ == ARTICORE_MODE_MIT;
      if (combine_mit_arm_and_gripper) run_gripper_control = false;
      if (state_ == ARTICORE_FAULT && fault_hold_active_ &&
          now >= next_safe_hold_) {
        run_hold = true;
        detail::advance_periodic_deadline(next_safe_hold_, hold_period, now);
      }
    }

    if (grace_fault) {
      std::string stop_error;
      if (!enter_safe_stop(
              "enable grace expired before the first successful command",
              stop_error)) {
        enter_fault("enable grace expired and protective hold is unavailable: " +
                    stop_error);
      }
      continue;
    }
    if (command_timeout) {
      std::string stop_error;
      if (!enter_safe_stop("command watchdog timed out", stop_error)) {
        enter_fault("command watchdog timed out; protective hold unavailable: " +
                    stop_error);
      }
      continue;
    }
    if (run_ready_feedback) {
      std::string error;
      std::vector<MissingMotor> missing_motors;
      bool complete = false;
      bool still_ready = false;
      {
        std::lock_guard<std::mutex> command_lock(command_mutex_);
        {
          std::lock_guard<std::mutex> state_lock(state_mutex_);
          still_ready = !hardware_transition_ &&
              (state_ == ARTICORE_READY ||
               state_ == ARTICORE_PARTIALLY_ENABLED);
        }
        if (still_ready) {
          complete = request_feedback_parallel(
              config_.disable_feedback_timeout_ms, missing_motors, error);
          complete = validate_fresh_feedback_snapshot(missing_motors, error) &&
                     complete;
        }
      }
      if (!still_ready) continue;
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (complete) {
        for (uint8_t side = 0; side < 2; ++side) {
          if (!active_sides_[side]) continue;
          sides_[side].healthy =
              sides_[side].connected && sides_[side].transport_healthy;
          sides_[side].last_error.clear();
        }
      } else {
        ++consecutive_feedback_failures_;
        for (uint8_t side = 0; side < 2; ++side) {
          if (!active_sides_[side]) continue;
          sides_[side].healthy = false;
          if (!error.empty()) sides_[side].last_error = error;
        }
      }
      continue;
    }
    if (run_arm_control) {
      std::string error;
      if (!run_arm_control_cycle(now, combine_mit_arm_and_gripper, error)) {
        std::string stop_error;
        if (!enter_safe_stop(
                "arm control cycle failed: " + error, stop_error)) {
          enter_fault("arm control cycle failed: " + error +
                      "; protective hold unavailable: " + stop_error);
        }
        continue;
      }
    }
    if (run_feedback_check) {
      std::string error;
      bool healthy = false;
      bool still_active = false;
      {
        std::lock_guard<std::mutex> command_lock(command_mutex_);
        {
          std::lock_guard<std::mutex> state_lock(state_mutex_);
          still_active = !hardware_transition_ &&
              (state_ == ARTICORE_RUNNING || state_ == ARTICORE_DEGRADED ||
               state_ == ARTICORE_SAFE_HOLD ||
               state_ == ARTICORE_SAFE_STOP);
        }
        if (still_active) {
          healthy = refresh_feedback_health(
              false, false, error, nullptr);
        }
      }
      if (!still_active) continue;
      if (!healthy) {
        const bool severe =
            error.find("motor fault status") != std::string::npos ||
            error.find("unexpectedly disabled") != std::string::npos ||
            error.find("transport disconnected") != std::string::npos ||
            error.find("not finite") != std::string::npos;
        bool enter_degraded_state = false;
        bool enter_safe_stop_state = false;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          ++consecutive_feedback_failures_;
          const uint64_t degraded_threshold =
              config_.feedback_failure_threshold;
          const uint64_t safe_stop_threshold = degraded_threshold * 3ULL;
          if (!severe && state_ == ARTICORE_RUNNING &&
              consecutive_feedback_failures_ >= degraded_threshold) {
            enter_degraded_state = true;
          }
          if (!severe && state_ == ARTICORE_DEGRADED &&
              consecutive_feedback_failures_ >= safe_stop_threshold) {
            enter_safe_stop_state = true;
          }
        }
        if (severe) {
          enter_fault(error);
        } else if (enter_safe_stop_state) {
          std::string stop_error;
          if (!enter_safe_stop(
                  "feedback safety timeout: " + error, stop_error)) {
            enter_fault("feedback safety timeout; protective hold unavailable: " +
                        stop_error);
          }
        } else if (enter_degraded_state) {
          enter_degraded("sustained feedback delay: " + error);
        }
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
          std::string stop_error;
          if (!enter_safe_stop(
                  "gripper control send failed: " + error, stop_error)) {
            enter_fault("gripper control send failed: " + error +
                        "; protective hold unavailable: " + stop_error);
          }
        } else {
          enter_fault("gripper control fault: " + error);
        }
        continue;
      }
    }
    if (run_hold) {
      std::string error;
      const bool held = send_safe_hold_once(error);
      if (!held) {
        bool fault = false;
        bool already_faulted = false;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          ++consecutive_hold_failures_;
          ++consecutive_send_failures_;
          already_faulted = state_ == ARTICORE_FAULT;
          fault = consecutive_hold_failures_ >= config_.safe_hold_failure_threshold;
          if (already_faulted && !error.empty() &&
              fault_reason_.find(error) == std::string::npos) {
            fault_reason_ += "; protective hold send: " + error;
          }
        }
        if (fault && !already_faulted) {
          enter_fault("safe hold failed: " + error);
        }
      } else {
        std::lock_guard<std::mutex> lock(state_mutex_);
        consecutive_hold_failures_ = 0;
      }
    }
  }
}

}  // namespace articore

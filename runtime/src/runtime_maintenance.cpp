#include "articore/detail/runtime.hpp"
#include "articore/detail/runtime_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <thread>

namespace articore {

namespace {

constexpr float kStationaryVelocityRadPerSecond = 0.05f;
constexpr float kZeroPositionToleranceRad = 0.02f;
constexpr uint32_t kControlModeTransactionTimeoutMs = 300;
constexpr auto kClearFaultRetryWindow = std::chrono::seconds(5);
constexpr auto kClearFaultRetryDelay = std::chrono::milliseconds(50);

const char* operation_name(ArticoreRuntimeOperation operation) {
  switch (operation) {
    case ARTICORE_OPERATION_CONNECT: return "connect";
    case ARTICORE_OPERATION_ENABLE: return "enable";
    case ARTICORE_OPERATION_DISABLE: return "disable";
    case ARTICORE_OPERATION_CONFIGURE_MODE: return "configure mode";
    case ARTICORE_OPERATION_CLEAR_FAULTS: return "clear faults";
    case ARTICORE_OPERATION_SET_ZERO: return "set zero";
    case ARTICORE_OPERATION_DISCONNECT: return "disconnect";
    case ARTICORE_OPERATION_COMMAND: return "command";
    case ARTICORE_OPERATION_RECOVER: return "recover";
    case ARTICORE_OPERATION_CANCEL_MOTION: return "cancel motion";
    case ARTICORE_OPERATION_STOP_MOTION: return "stop motion";
    case ARTICORE_OPERATION_MOVE_POSE: return "move pose";
    case ARTICORE_OPERATION_MOVE_LINEAR: return "move linear";
    case ARTICORE_OPERATION_MOVE_CIRCULAR: return "move circular";
    case ARTICORE_OPERATION_START_BIMANUAL_FOLLOW:
      return "start bimanual follow";
    case ARTICORE_OPERATION_STOP_BIMANUAL_FOLLOW:
      return "stop bimanual follow";
    case ARTICORE_OPERATION_SET_TCP_OFFSET: return "set TCP offset";
    default: return "maintenance";
  }
}

const char* state_name(ArticoreSafetyState state) {
  switch (state) {
    case ARTICORE_DISCONNECTED: return "DISCONNECTED";
    case ARTICORE_READY: return "READY";
    case ARTICORE_ENABLED: return "ENABLED";
    case ARTICORE_RUNNING: return "RUNNING";
    case ARTICORE_SAFE_HOLD: return "SAFE_HOLD";
    case ARTICORE_FAULT: return "FAULT";
    case ARTICORE_DEGRADED: return "DEGRADED";
    case ARTICORE_SAFE_STOP: return "SAFE_STOP";
    case ARTICORE_PARTIALLY_ENABLED: return "PARTIALLY_ENABLED";
  }
  return "UNKNOWN";
}

void append_names(std::ostringstream& output,
                  const std::vector<std::string>& names) {
  output << '[';
  for (std::size_t index = 0; index < names.size(); ++index) {
    if (index != 0) output << ", ";
    output << names[index];
  }
  output << ']';
}

}  // namespace

void SafetyRuntime::record_operation_result(
    ArticoreRuntimeOperation operation, int32_t code,
    const std::string& error,
    const std::vector<std::string>& failed_motors) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_operation_ = operation;
  last_operation_code_ = code;
  last_operation_error_ = error;
  operation_failed_motors_ = failed_motors;
  if (!emergency_stop_latched_ && operation != ARTICORE_OPERATION_COMMAND &&
      operation != ARTICORE_OPERATION_CANCEL_MOTION &&
      operation != ARTICORE_OPERATION_CANCEL_ALL_MOTIONS &&
      code != ARTICORE_OPERATION_OK && !error.empty() &&
      !(fault_latched_ && code == ARTICORE_OPERATION_INVALID_STATE)) {
    fault_reason_ = std::string(operation_name(operation)) + " failed: " + error;
  }
}

int32_t SafetyRuntime::finish_maintenance(
    ArticoreRuntimeOperation operation, int32_t code,
    const std::string& error, const std::vector<std::string>& failed_motors,
    bool latch_fault) {
  const bool motor_fault_reported =
      code != ARTICORE_OPERATION_OK && current_motor_fault_reported();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    hardware_transition_ = false;
    last_operation_ = operation;
    last_operation_code_ = code;
    last_operation_error_ = error;
    operation_failed_motors_ = failed_motors;
    if (code == ARTICORE_OPERATION_OK) {
      state_ = ARTICORE_READY;
      disable_confirmed_ = true;
      if (operation == ARTICORE_OPERATION_CLEAR_FAULTS) {
        fault_latched_ = false;
        motor_fault_latched_ = false;
        fault_reason_.clear();
        motor_faults_.clear();
        for (auto& entry : presence_) {
          if (entry.second == ARTICORE_FAULTED) {
            entry.second = ARTICORE_PRESENT;
          }
        }
      }
      if (!fault_latched_) fault_reason_.clear();
      const auto ready_refresh_hz = std::min(config_.feedback_check_hz, 10U);
      next_ready_feedback_ = Clock::now() + std::chrono::nanoseconds(
          1'000'000'000ULL / ready_refresh_hz);
    } else if (latch_fault) {
      state_ = ARTICORE_FAULT;
      fault_latched_ = true;
      motor_fault_latched_ =
          motor_fault_latched_ || motor_fault_reported;
      // A failed maintenance transaction cannot claim physical disable unless
      // a separate disable barrier has actually verified every installed Motor.
      disable_confirmed_ = false;
      fault_reason_ = std::string(operation_name(operation)) + " failed: " + error;
      for (const auto& name : failed_motors) {
        if (std::find(motor_faults_.begin(), motor_faults_.end(), name) ==
            motor_faults_.end()) {
          motor_faults_.push_back(name);
        }
      }
    }
  }
  wakeup_.notify_all();
  return code;
}

int32_t SafetyRuntime::maintenance_precheck(
    ArticoreRuntimeOperation operation, std::string& error,
    std::vector<std::string>& failed_motors, bool require_stationary) {
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool valid_state = state_ == ARTICORE_READY ||
        (operation == ARTICORE_OPERATION_CLEAR_FAULTS &&
         state_ == ARTICORE_FAULT && !emergency_stop_latched_);
    if (!valid_state || hardware_transition_) {
      failed_motors = motor_faults_;
      for (const auto& name : operation_failed_motors_) {
        if (std::find(failed_motors.begin(), failed_motors.end(), name) ==
            failed_motors.end()) {
          failed_motors.push_back(name);
        }
      }
      std::ostringstream detail;
      detail << operation_name(operation) << " rejected: current_state="
             << state_name(state_) << ", required_states="
             << (operation == ARTICORE_OPERATION_CLEAR_FAULTS
                     ? "[READY, FAULT(non_estop)]"
                     : "[READY]")
             << ", emergency_stop_latched="
             << (emergency_stop_latched_ ? "true" : "false")
             << ", hardware_transition="
             << (hardware_transition_ ? "true" : "false")
             << ", fault_reason="
             << (fault_reason_.empty() ? "<none>" : fault_reason_)
             << ", failed_motors=";
      append_names(detail, failed_motors);
      error = detail.str();
      last_operation_ = operation;
      last_operation_code_ = ARTICORE_OPERATION_INVALID_STATE;
      last_operation_error_ = error;
      operation_failed_motors_ = failed_motors;
      return ARTICORE_OPERATION_INVALID_STATE;
    }
    last_operation_ = operation;
    last_operation_code_ = ARTICORE_OPERATION_OK;
    last_operation_error_.clear();
    operation_failed_motors_.clear();
    hardware_transition_ = true;
  }
  wakeup_.notify_all();

  if (!refresh_transport_health(error)) {
    for (const auto& motor : motors_) {
      failed_motors.emplace_back(motor.descriptor.name);
    }
    return ARTICORE_OPERATION_TRANSPORT;
  }
  std::vector<MissingMotor> missing;
  if (!request_feedback_parallel(config_.disable_feedback_timeout_ms,
                                 missing, error) ||
      !validate_fresh_feedback_snapshot(missing, error)) {
    for (const auto& item : missing) {
      for (const auto& motor : motors_) {
        if (motor.descriptor.side == item.side &&
            motor.motor_identity_configured &&
            motor.configured_can_id == item.id) {
          failed_motors.emplace_back(motor.descriptor.name);
        }
      }
    }
    if (failed_motors.empty()) {
      for (const auto& motor : motors_) {
        failed_motors.emplace_back(motor.descriptor.name);
      }
    }
    return ARTICORE_OPERATION_FEEDBACK;
  }
  for (const auto& motor : motors_) {
    ArticoreMotorState state{};
    if (backend_->get_state(motor.descriptor.motor, &state) != 0 ||
        !state.has_value) {
      error = std::string(motor.descriptor.name) + ": feedback unavailable";
      failed_motors.emplace_back(motor.descriptor.name);
      return ARTICORE_OPERATION_FEEDBACK;
    }
    if (operation != ARTICORE_OPERATION_CLEAR_FAULTS &&
        state.status_code != 0) {
      error = std::string(motor.descriptor.name) +
              ": motor is not physically disabled";
      failed_motors.emplace_back(motor.descriptor.name);
      return ARTICORE_OPERATION_NOT_DISABLED;
    }
    if (!finite(state.vel)) {
      error = std::string(motor.descriptor.name) +
              ": velocity feedback is not finite";
      failed_motors.emplace_back(motor.descriptor.name);
      return ARTICORE_OPERATION_FEEDBACK;
    }
    if (require_stationary &&
        std::fabs(state.vel) > kStationaryVelocityRadPerSecond) {
      error = std::string(motor.descriptor.name) +
              ": motor is not stationary";
      failed_motors.emplace_back(motor.descriptor.name);
      return ARTICORE_OPERATION_NOT_STATIONARY;
    }
  }
  return ARTICORE_OPERATION_OK;
}

int32_t SafetyRuntime::configure_hardware_mode(
    ArticoreControlMode mode, std::string& error,
    std::vector<std::string>& failed_motors) {
  if (!backend_->can_ensure_mode()) {
    error = "the Runtime was created without native mode configuration callbacks";
    return ARTICORE_OPERATION_UNSUPPORTED;
  }

  struct MotorResult {
    bool attempted = false;
    std::string failed;
    std::string error;
  };
  std::vector<MotorResult> results(motors_.size());
  std::array<std::vector<std::size_t>, 2> motors_by_side;
  for (std::size_t index = 0; index < motors_.size(); ++index) {
    const auto side = motors_[index].descriptor.side;
    if (!active_sides_[side]) continue;
    results[index].attempted = true;
    motors_by_side[side].push_back(index);
  }

  // Maintenance concurrency is bounded to one worker per CAN channel.  This
  // keeps the non-RT side of the service at a stable 1..2 workers while both
  // channels still progress in parallel.  Each side's PacingBus already
  // serializes its wire traffic, so one thread per Motor only added scheduler
  // contention without increasing physical CAN concurrency.
  const uint32_t transaction_timeout_ms =
      std::max(config_.disable_feedback_timeout_ms,
               kControlModeTransactionTimeoutMs);
  const auto mode_deadline =
      Clock::now() + std::chrono::milliseconds(transaction_timeout_ms);
  detail::run_active_sides(active_sides_, [&](uint8_t side) {
    for (const auto index : motors_by_side[side]) {
      const auto& target = motors_[index];
      const uint32_t native_mode =
          target.descriptor.is_gripper || mode == ARTICORE_MODE_MIT ? 1U : 2U;
      const auto now = Clock::now();
      int32_t rc = -1;
      if (now < mode_deadline) {
        // Every Motor receives only the time left in the one product-wide
        // transaction window. The two CAN channels progress independently.
        const auto remaining_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                mode_deadline - now);
        const auto remaining_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                remaining_ns + std::chrono::microseconds(999));
        rc = backend_->ensure_mode(
            target.descriptor.motor, native_mode,
            static_cast<uint32_t>(std::max(remaining_ms,
                                           std::chrono::milliseconds(1))
                                      .count()));
      }
      if (rc == 0 && backend_->can_set_timeout() &&
          backend_->communication_timeout_ms() > 0) {
        rc = backend_->set_timeout_ms(
            target.descriptor.motor, backend_->communication_timeout_ms());
      }
      if (rc == 0) continue;
      results[index].failed = target.descriptor.name;
      const char* detail = backend_->last_error_message();
      results[index].error = detail && detail[0]
          ? detail
          : (now >= mode_deadline ? "control mode batch deadline expired"
                                  : "native motor command failed");
    }
  });
  for (const auto& result : results) {
    if (!result.attempted || result.failed.empty()) continue;
    failed_motors.emplace_back(result.failed);
    if (!error.empty()) error += "; ";
    error += result.failed + ": " + result.error;
  }
  return failed_motors.empty() ? ARTICORE_OPERATION_OK
                               : ARTICORE_OPERATION_MOTOR_COMMAND;
}

int32_t SafetyRuntime::run_motor_maintenance(
    ArticoreRuntimeOperation operation, ArticoreControlMode mode,
    bool require_stationary) {
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  std::unique_lock<std::mutex> command_lock(command_mutex_);
  std::string error;
  std::vector<std::string> failed;
  const int32_t precheck = maintenance_precheck(
      operation, error, failed, require_stationary);
  if (precheck != ARTICORE_OPERATION_OK) {
    return finish_maintenance(operation, precheck, error, failed,
                              precheck == ARTICORE_OPERATION_TRANSPORT ||
                                  precheck == ARTICORE_OPERATION_FEEDBACK ||
                                  precheck == ARTICORE_OPERATION_NOT_DISABLED);
  }

  const bool configure = operation == ARTICORE_OPERATION_CONFIGURE_MODE;
  if (configure) {
    const int32_t configured = configure_hardware_mode(mode, error, failed);
    if (configured != ARTICORE_OPERATION_OK) {
      return finish_maintenance(
          operation, configured, error, failed,
          configured == ARTICORE_OPERATION_MOTOR_COMMAND);
    }
  }

  const bool supported = operation == ARTICORE_OPERATION_CLEAR_FAULTS
      ? backend_->can_clear_error() : backend_->can_set_zero();
  if (!configure && !supported) {
    return finish_maintenance(
        operation, ARTICORE_OPERATION_UNSUPPORTED,
        "the Runtime was created without native maintenance callbacks", failed,
        false);
  }

  if (!configure) {
    struct SideResult {
      std::vector<std::string> failed;
      std::vector<std::string> errors;
    } results[2];
    detail::run_active_sides(active_sides_, [&](uint8_t side) {
      for (const auto& motor : motors_) {
        if (motor.descriptor.side != side) continue;
        const int32_t rc = operation == ARTICORE_OPERATION_CLEAR_FAULTS
            ? backend_->clear_error(motor.descriptor.motor)
            : backend_->set_zero(motor.descriptor.motor);
        if (rc == 0) continue;
        results[side].failed.emplace_back(motor.descriptor.name);
        const char* detail = backend_->last_error_message();
        results[side].errors.emplace_back(
            detail && detail[0] ? detail : "native motor command failed");
      }
    });
    for (const auto& result : results) {
      failed.insert(failed.end(), result.failed.begin(), result.failed.end());
      for (std::size_t index = 0; index < result.errors.size(); ++index) {
        if (!error.empty()) error += "; ";
        error += result.failed[index] + ": " + result.errors[index];
      }
    }
    if (!failed.empty()) {
      return finish_maintenance(operation, ARTICORE_OPERATION_MOTOR_COMMAND,
                                error, failed, true);
    }
  }

  std::vector<MissingMotor> missing;
  if (!request_feedback_parallel(config_.disable_feedback_timeout_ms,
                                 missing, error) ||
      !validate_fresh_feedback_snapshot(missing, error)) {
    for (const auto& item : missing) {
      for (const auto& motor : motors_) {
        if (motor.descriptor.side == item.side &&
            motor.motor_identity_configured &&
            motor.configured_can_id == item.id) {
          failed.emplace_back(motor.descriptor.name);
        }
      }
    }
    return finish_maintenance(operation, ARTICORE_OPERATION_FEEDBACK,
                              error, failed, true);
  }

  for (const auto& motor : motors_) {
    ArticoreMotorState state{};
    const bool readable =
        backend_->get_state(motor.descriptor.motor, &state) == 0 &&
        state.has_value;
    const bool disabled = readable && state.status_code == 0;
    const bool stationary = readable && finite(state.vel) &&
        (!require_stationary ||
         std::fabs(state.vel) <= kStationaryVelocityRadPerSecond);
    const bool zeroed = operation != ARTICORE_OPERATION_SET_ZERO ||
        (readable && finite(state.pos) &&
         std::fabs(state.pos) <= kZeroPositionToleranceRad);
    if (disabled && stationary && zeroed) continue;
    failed.emplace_back(motor.descriptor.name);
    if (!error.empty()) error += "; ";
    error += std::string(motor.descriptor.name) +
             ": post-operation verification failed";
  }
  if (!failed.empty()) {
    return finish_maintenance(operation, ARTICORE_OPERATION_VERIFICATION,
                              error, failed, true);
  }
  // Fault clearing and control-mode configuration are separate product
  // transactions. CLEAR_FAULTS must only restore a verified, disabled READY
  // Runtime. The controlling client may explicitly CONFIGURE_MODE afterwards;
  // maintenance-only clients deliberately leave the hardware unconfigured.
  if (configure) mode_ = mode;
  return finish_maintenance(operation, ARTICORE_OPERATION_OK, {}, {}, false);
}

int32_t SafetyRuntime::configure_mode(ArticoreControlMode mode) {
  if (mode != ARTICORE_MODE_PV && mode != ARTICORE_MODE_MIT) {
    std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    record_operation_result(ARTICORE_OPERATION_CONFIGURE_MODE,
                            ARTICORE_OPERATION_INVALID_ARGUMENT,
                            "unsupported control mode");
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
  // Changing the controller mode is allowed while a physically disabled arm
  // still has residual motion. Fresh, finite feedback and disabled motor state
  // are still required; enable() will seed its first hold from the latest
  // measured positions instead of commanding an old target.
  return run_motor_maintenance(
      ARTICORE_OPERATION_CONFIGURE_MODE, mode, false);
}

int32_t SafetyRuntime::configure_mode_for_connect(ArticoreControlMode mode) {
  if (mode != ARTICORE_MODE_PV && mode != ARTICORE_MODE_MIT) {
    std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    record_operation_result(ARTICORE_OPERATION_CONFIGURE_MODE,
                            ARTICORE_OPERATION_INVALID_ARGUMENT,
                            "unsupported control mode");
    return ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
  const int32_t result = run_motor_maintenance(
      ARTICORE_OPERATION_CONFIGURE_MODE, mode, false);
  if (result != ARTICORE_OPERATION_OK) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ == ARTICORE_FAULT) {
      fault_reason_ = "connect detected mode configuration failure";
      if (!last_operation_error_.empty()) {
        fault_reason_ += ": " + last_operation_error_;
      }
    }
  }
  return result;
}

int32_t SafetyRuntime::clear_faults() {
  const auto deadline = Clock::now() + kClearFaultRetryWindow;
  int32_t result = ARTICORE_OPERATION_OK;
  for (;;) {
    result = run_motor_maintenance(
        ARTICORE_OPERATION_CLEAR_FAULTS, mode_, false);
    if (result == ARTICORE_OPERATION_OK) return result;

    // A clear command can take effect before every USB-CAN feedback response
    // becomes observable. Retry only feedback/verification failures; invalid
    // state, emergency stop, unsupported hardware, transport, and native send
    // failures remain immediate errors rather than being hidden for 5 seconds.
    if (result != ARTICORE_OPERATION_FEEDBACK &&
        result != ARTICORE_OPERATION_VERIFICATION) {
      return result;
    }
    const auto now = Clock::now();
    if (now >= deadline) break;
    std::this_thread::sleep_until(
        std::min(deadline, now + kClearFaultRetryDelay));
  }

  std::string last_error;
  std::vector<std::string> failed_motors;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_error = last_operation_error_;
    failed_motors = operation_failed_motors_;
  }
  std::ostringstream timeout;
  timeout << "clear faults timed out after "
          << std::chrono::duration_cast<std::chrono::milliseconds>(
                 kClearFaultRetryWindow)
                 .count()
          << " ms";
  if (!last_error.empty()) timeout << ": " << last_error;
  return finish_maintenance(ARTICORE_OPERATION_CLEAR_FAULTS, result,
                            timeout.str(), failed_motors, true);
}

int32_t SafetyRuntime::set_zero() {
  return run_motor_maintenance(ARTICORE_OPERATION_SET_ZERO, mode_, true);
}

}  // namespace articore

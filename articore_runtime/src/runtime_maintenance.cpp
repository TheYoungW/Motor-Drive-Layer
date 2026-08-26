#include "articore/detail/runtime.hpp"
#include "articore/detail/runtime_utils.hpp"

#include <algorithm>
#include <cmath>
#include <thread>

namespace articore {

namespace {

constexpr float kStationaryVelocityRadPerSecond = 0.05f;
constexpr float kZeroPositionToleranceRad = 0.02f;

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
    case ARTICORE_OPERATION_START_TRAJECTORY: return "start trajectory";
    case ARTICORE_OPERATION_CANCEL_MOTION: return "cancel motion";
    case ARTICORE_OPERATION_CANCEL_ALL_MOTIONS: return "cancel all motions";
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
      operation != ARTICORE_OPERATION_START_TRAJECTORY &&
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
    last_operation_ = operation;
    last_operation_code_ = ARTICORE_OPERATION_OK;
    last_operation_error_.clear();
    operation_failed_motors_.clear();
    const bool valid_state = state_ == ARTICORE_READY ||
        (operation == ARTICORE_OPERATION_CLEAR_FAULTS &&
         state_ == ARTICORE_FAULT && !emergency_stop_latched_);
    if (!valid_state || hardware_transition_) {
      error = std::string(operation_name(operation)) +
              " requires Runtime READY";
      return ARTICORE_OPERATION_INVALID_STATE;
    }
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

  struct SideResult {
    std::vector<std::string> failed;
    std::vector<std::string> errors;
  } results[2];
  std::vector<std::thread> workers;
  for (uint8_t side = 0; side < 2; ++side) {
    if (!active_sides_[side]) continue;
    workers.emplace_back([&, side] {
      for (const auto& motor : motors_) {
        if (motor.descriptor.side != side) continue;
        const uint32_t native_mode =
            motor.descriptor.is_gripper || mode == ARTICORE_MODE_MIT ? 1U : 2U;
        int32_t rc = backend_->ensure_mode(
            motor.descriptor.motor, native_mode,
            config_.disable_feedback_timeout_ms);
        if (rc == 0 && backend_->can_set_timeout() &&
            backend_->communication_timeout_ms() > 0) {
          rc = backend_->set_timeout_ms(
              motor.descriptor.motor, backend_->communication_timeout_ms());
        }
        if (rc == 0) continue;
        results[side].failed.emplace_back(motor.descriptor.name);
        const char* detail = backend_->last_error_message();
        results[side].errors.emplace_back(
            detail && detail[0] ? detail : "native motor command failed");
      }
    });
  }
  for (auto& worker : workers) worker.join();
  for (const auto& result : results) {
    failed_motors.insert(failed_motors.end(), result.failed.begin(),
                         result.failed.end());
    for (std::size_t index = 0; index < result.errors.size(); ++index) {
      if (!error.empty()) error += "; ";
      error += result.failed[index] + ": " + result.errors[index];
    }
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
    std::vector<std::thread> workers;
    for (uint8_t side = 0; side < 2; ++side) {
      if (!active_sides_[side]) continue;
      workers.emplace_back([&, side] {
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
    }
    for (auto& worker : workers) worker.join();
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
  if (operation == ARTICORE_OPERATION_CLEAR_FAULTS &&
      backend_->can_ensure_mode()) {
    const int32_t configured = configure_hardware_mode(mode, error, failed);
    if (configured != ARTICORE_OPERATION_OK) {
      return finish_maintenance(
          operation, configured,
          error.empty() ? "post-clear mode configuration failed" : error,
          failed, true);
    }

    missing.clear();
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
      if (backend_->get_state(motor.descriptor.motor, &state) == 0 &&
          state.has_value && state.status_code == 0) {
        continue;
      }
      failed.emplace_back(motor.descriptor.name);
      if (!error.empty()) error += "; ";
      error += std::string(motor.descriptor.name) +
               ": post-clear mode configuration did not preserve disable";
    }
    if (!failed.empty()) {
      return finish_maintenance(operation, ARTICORE_OPERATION_VERIFICATION,
                                error, failed, true);
    }
  }
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
  return run_motor_maintenance(
      ARTICORE_OPERATION_CONFIGURE_MODE, mode, true);
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
  return run_motor_maintenance(
      ARTICORE_OPERATION_CLEAR_FAULTS, mode_, false);
}

int32_t SafetyRuntime::set_zero() {
  return run_motor_maintenance(ARTICORE_OPERATION_SET_ZERO, mode_, true);
}

}  // namespace articore

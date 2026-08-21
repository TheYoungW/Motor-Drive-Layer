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
    case ARTICORE_OPERATION_CLOSE: return "close";
    case ARTICORE_OPERATION_DISCONNECT: return "disconnect";
    case ARTICORE_OPERATION_COMMAND: return "command";
    case ARTICORE_OPERATION_RECOVER: return "recover";
    case ARTICORE_OPERATION_START_TRAJECTORY: return "start trajectory";
    case ARTICORE_OPERATION_CANCEL_TRAJECTORY: return "cancel trajectory";
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
      operation != ARTICORE_OPERATION_CANCEL_TRAJECTORY &&
      code != ARTICORE_OPERATION_OK && !error.empty()) {
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
      }
      if (!fault_latched_) fault_reason_.clear();
      const auto ready_refresh_hz = std::min(config_.feedback_check_hz, 10U);
      next_ready_feedback_ = Clock::now() + std::chrono::nanoseconds(
          1'000'000'000ULL / ready_refresh_hz);
    } else if (latch_fault) {
      state_ = ARTICORE_FAULT;
      fault_latched_ = true;
      disable_confirmed_ = true;
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
    std::vector<std::string>& failed_motors) {
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
    if (!disable_confirmed_) {
      error = "physical disable is not confirmed";
      return ARTICORE_OPERATION_NOT_DISABLED;
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
    if (!finite(state.vel) ||
        std::fabs(state.vel) > kStationaryVelocityRadPerSecond) {
      error = std::string(motor.descriptor.name) +
              ": motor is not stationary";
      failed_motors.emplace_back(motor.descriptor.name);
      return ARTICORE_OPERATION_NOT_STATIONARY;
    }
  }
  return ARTICORE_OPERATION_OK;
}

int32_t SafetyRuntime::run_motor_maintenance(
    ArticoreRuntimeOperation operation, ArticoreControlMode mode) {
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  std::unique_lock<std::mutex> command_lock(command_mutex_);
  std::string error;
  std::vector<std::string> failed;
  const int32_t precheck = maintenance_precheck(operation, error, failed);
  if (precheck != ARTICORE_OPERATION_OK) {
    return finish_maintenance(operation, precheck, error, failed,
                              precheck == ARTICORE_OPERATION_TRANSPORT ||
                                  precheck == ARTICORE_OPERATION_FEEDBACK);
  }

  const bool configure = operation == ARTICORE_OPERATION_CONFIGURE_MODE;
  const bool supported = configure ? backend_->can_ensure_mode()
      : operation == ARTICORE_OPERATION_CLEAR_FAULTS
          ? backend_->can_clear_error() : backend_->can_set_zero();
  if (!supported) {
    return finish_maintenance(
        operation, ARTICORE_OPERATION_UNSUPPORTED,
        "the Runtime was created without native maintenance callbacks", failed,
        false);
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
        int32_t rc = 0;
        if (configure) {
          // The product control mode applies only to the fourteen arm joints.
          // Yunyi grippers always run MIT because a position/velocity gripper
          // can keep driving into an obstructed target and stall. The Motor core
          // uses MIT=1 and POS_VEL=2.
          const uint32_t native_mode =
              motor.descriptor.is_gripper || mode == ARTICORE_MODE_MIT
                  ? 1U : 2U;
          rc = backend_->ensure_mode(
              motor.descriptor.motor, native_mode,
              config_.disable_feedback_timeout_ms);
          if (rc == 0 && backend_->can_set_timeout() &&
              backend_->communication_timeout_ms() > 0) {
            rc = backend_->set_timeout_ms(
                motor.descriptor.motor,
                backend_->communication_timeout_ms());
          }
        } else if (operation == ARTICORE_OPERATION_CLEAR_FAULTS) {
          rc = backend_->clear_error(motor.descriptor.motor);
        } else {
          rc = backend_->set_zero(motor.descriptor.motor);
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
        std::fabs(state.vel) <= kStationaryVelocityRadPerSecond;
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
  return run_motor_maintenance(ARTICORE_OPERATION_CONFIGURE_MODE, mode);
}

int32_t SafetyRuntime::clear_faults() {
  return run_motor_maintenance(ARTICORE_OPERATION_CLEAR_FAULTS, mode_);
}

int32_t SafetyRuntime::set_zero() {
  return run_motor_maintenance(ARTICORE_OPERATION_SET_ZERO, mode_);
}

}  // namespace articore

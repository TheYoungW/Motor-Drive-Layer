#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <motor_abi.h>

#include "articore/runtime_abi.h"

namespace articore {

class RuntimeError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

class ConnectError : public RuntimeError {
 public:
  ConnectError(std::string message, ArticoreConnectReport report)
      : RuntimeError(std::move(message)), report_(report) {}

  const ArticoreConnectReport& report() const noexcept { return report_; }

 private:
  ArticoreConnectReport report_{};
};

namespace detail {

inline int32_t group_send_pos_vel(void* group,
                                  const ArticorePosVelCommand* commands,
                                  uint32_t count) {
  static_assert(sizeof(ArticorePosVelCommand) ==
                sizeof(MotorPosVelBatchCommand));
  static_assert(alignof(ArticorePosVelCommand) ==
                alignof(MotorPosVelBatchCommand));
  return motor_controller_group_send_pos_vel(
      static_cast<MotorControllerGroup*>(group),
      reinterpret_cast<const MotorPosVelBatchCommand*>(commands), count);
}

inline int32_t group_send_mit(void* group,
                              const ArticoreMitCommand* commands,
                              uint32_t count) {
  static_assert(sizeof(ArticoreMitCommand) == sizeof(MotorMitBatchCommand));
  static_assert(alignof(ArticoreMitCommand) == alignof(MotorMitBatchCommand));
  return motor_controller_group_send_mit(
      static_cast<MotorControllerGroup*>(group),
      reinterpret_cast<const MotorMitBatchCommand*>(commands), count);
}

inline int32_t controller_disable_all(void* controller) {
  return motor_controller_disable_all(static_cast<MotorController*>(controller));
}

inline int32_t controller_enable_all(void* controller) {
  return motor_controller_enable_all(static_cast<MotorController*>(controller));
}

inline int32_t controller_request_feedback_all_ex(
    void* controller, uint32_t timeout_ms, ArticoreFeedbackReport* report,
    uint32_t* missing_motor_ids, uint32_t capacity) {
  static_assert(sizeof(ArticoreFeedbackReport) == sizeof(MotorFeedbackReport));
  static_assert(alignof(ArticoreFeedbackReport) == alignof(MotorFeedbackReport));
  return motor_controller_request_feedback_all_ex(
      static_cast<MotorController*>(controller), timeout_ms,
      reinterpret_cast<MotorFeedbackReport*>(report), missing_motor_ids,
      capacity);
}

inline int32_t motor_get_state(void* motor, ArticoreMotorState* state) {
  static_assert(sizeof(ArticoreMotorState) == sizeof(MotorState));
  static_assert(alignof(ArticoreMotorState) == alignof(MotorState));
  return motor_handle_get_state(static_cast<MotorHandle*>(motor),
                                reinterpret_cast<MotorState*>(state));
}

inline int32_t motor_get_feedback_stats(void* motor,
                                        ArticoreFeedbackStats* stats) {
  static_assert(sizeof(ArticoreFeedbackStats) == sizeof(MotorFeedbackStats));
  static_assert(alignof(ArticoreFeedbackStats) == alignof(MotorFeedbackStats));
  return motor_handle_get_feedback_stats(
      static_cast<MotorHandle*>(motor),
      reinterpret_cast<MotorFeedbackStats*>(stats));
}

inline int32_t controller_get_transport_health(
    void* controller, ArticoreDriverTransportHealth* health) {
  static_assert(sizeof(ArticoreDriverTransportHealth) ==
                sizeof(MotorTransportHealth));
  static_assert(alignof(ArticoreDriverTransportHealth) ==
                alignof(MotorTransportHealth));
  return motor_controller_get_transport_health(
      static_cast<MotorController*>(controller),
      reinterpret_cast<MotorTransportHealth*>(health));
}

inline int32_t motor_disable(void* motor) {
  return motor_handle_disable(static_cast<MotorHandle*>(motor));
}

inline int32_t motor_enable(void* motor) {
  return motor_handle_enable(static_cast<MotorHandle*>(motor));
}

inline ArticoreMotorApi motor_api() noexcept {
  return ArticoreMotorApi{
      &group_send_pos_vel,
      &group_send_mit,
      &controller_disable_all,
      &controller_request_feedback_all_ex,
      &motor_get_state,
      &motor_get_feedback_stats,
      &motor_last_error_message,
      &controller_get_transport_health,
      &motor_disable,
  };
}

inline ArticoreRuntimeTransportCapabilities transport_capabilities(
    MotorController* controller, uint32_t side) {
  MotorTransportCapabilitiesV2 native{};
  native.struct_size = sizeof(native);
  if (motor_controller_get_transport_capabilities_v2(controller, &native) != 0) {
    const char* reason = motor_last_error_message();
    throw RuntimeError(
        std::string("get transport capabilities failed: ") +
        (reason ? reason : "unknown motor error"));
  }
  ArticoreRuntimeTransportCapabilities result{};
  result.struct_size = sizeof(result);
  result.side = side;
  result.can_fd = native.can_fd;
  result.can_fd_brs = native.can_fd_brs;
  std::strncpy(result.transport, native.transport,
               sizeof(result.transport) - 1);
  return result;
}

inline void check(int32_t result, const char* operation) {
  if (result == 0) return;
  const char* reason = articore_runtime_last_error();
  throw RuntimeError(std::string(operation) + " failed: " +
                     (reason ? reason : "unknown Runtime error"));
}

}  // namespace detail

class Runtime final {
 public:
  Runtime(const ArticoreRuntimeConfig& config, MotorControllerGroup* group,
          MotorController* left_controller,
          MotorController* right_controller,
          const std::vector<ArticoreMotorDescriptor>& motors)
      : motor_api_(detail::motor_api()) {
    if (motors.empty()) {
      throw RuntimeError("Articore runtime requires at least one motor");
    }
    std::vector<ArticoreRuntimeTransportCapabilities> transports;
    if (left_controller) {
      transports.push_back(detail::transport_capabilities(left_controller, 0));
    }
    if (right_controller) {
      transports.push_back(detail::transport_capabilities(right_controller, 1));
    }
    runtime_ = articore_runtime_create_ex2(
        &config, &motor_api_, group, left_controller, right_controller,
        motors.data(), static_cast<uint32_t>(motors.size()),
        &detail::controller_enable_all, &detail::motor_enable,
        transports.data(), static_cast<uint32_t>(transports.size()));
    if (!runtime_) {
      const char* reason = articore_runtime_last_error();
      throw RuntimeError(std::string("articore_runtime_create_ex failed: ") +
                         (reason ? reason : "unknown Runtime error"));
    }
  }

  ~Runtime() noexcept {
    if (!runtime_) return;
    (void)articore_runtime_close(runtime_);
    articore_runtime_free(runtime_);
  }

  Runtime(const Runtime&) = delete;
  Runtime& operator=(const Runtime&) = delete;

  Runtime(Runtime&& other) noexcept
      : runtime_(std::exchange(other.runtime_, nullptr)),
        motor_api_(other.motor_api_) {}

  Runtime& operator=(Runtime&& other) noexcept {
    if (this == &other) return *this;
    if (runtime_) {
      (void)articore_runtime_close(runtime_);
      articore_runtime_free(runtime_);
    }
    runtime_ = std::exchange(other.runtime_, nullptr);
    motor_api_ = other.motor_api_;
    return *this;
  }

  ArticoreRuntime* native_handle() const noexcept { return runtime_; }
  explicit operator bool() const noexcept { return runtime_ != nullptr; }

  uint32_t control_hz() const {
    uint32_t result = 0;
    detail::check(articore_runtime_get_control_hz(checked(), &result),
                  "get_control_hz");
    return result;
  }

  void configure_joints(
      const std::vector<ArticoreJointControlConfig>& configs) {
    detail::check(articore_runtime_configure_joints(
                      checked(), configs.data(), size(configs)),
                  "configure_joints");
  }

  void configure_joint_safety_limits(
      const std::vector<ArticoreJointSafetyLimits>& limits) {
    auto native = limits;
    for (auto& value : native) value.struct_size = sizeof(value);
    detail::check(articore_runtime_configure_joint_safety_limits(
                      checked(), native.data(), size(native)),
                  "configure_joint_safety_limits");
  }

  void configure_motor_identities(
      const std::vector<ArticoreMotorIdentity>& identities) {
    auto native = identities;
    for (auto& value : native) value.struct_size = sizeof(value);
    detail::check(articore_runtime_configure_motor_identities(
                      checked(), native.data(), size(native)),
                  "configure_motor_identities");
  }

  void configure_gripper_products(
      const std::vector<ArticoreGripperProductBinding>& bindings) {
    auto native = bindings;
    for (auto& value : native) value.struct_size = sizeof(value);
    detail::check(articore_runtime_configure_gripper_products(
                      checked(), native.data(), size(native)),
                  "configure_gripper_products");
  }

  void configure_gravity_products(
      const std::vector<ArticoreGravityProductBinding>& bindings) {
    auto native = bindings;
    for (auto& value : native) value.struct_size = sizeof(value);
    detail::check(articore_runtime_configure_gravity_products(
                      checked(), native.data(), size(native)),
                  "configure_gravity_products");
  }

  void declare_motor_presence(const std::string& role,
                              ArticorePresenceState state) {
    detail::check(articore_runtime_declare_motor_presence(
                      checked(), role.c_str(), state),
                  "declare_motor_presence");
  }

  ArticorePresenceState motor_presence(const std::string& role) const {
    int32_t state = 0;
    detail::check(
        articore_runtime_motor_presence(checked(), role.c_str(), &state),
        "motor_presence");
    return static_cast<ArticorePresenceState>(state);
  }

  uint64_t active_capabilities() const {
    return articore_runtime_active_capabilities(checked());
  }

  ArticoreConnectReport connect() {
    const auto result = articore_runtime_connect(checked());
    const std::string failure = result == 0 || !articore_runtime_last_error()
        ? std::string{}
        : articore_runtime_last_error();
    auto report = last_connect_report();
    if (result != 0) {
      throw ConnectError("connect failed: " + failure, report);
    }
    return report;
  }

  ArticoreConnectReport last_connect_report() const {
    ArticoreConnectReport report{};
    report.struct_size = sizeof(report);
    detail::check(
        articore_runtime_get_last_connect_report(checked(), &report),
        "get_last_connect_report");
    return report;
  }

  ArticoreEnableReport enable(ArticoreControlMode mode) {
    const auto result = articore_runtime_enable(checked(), mode);
    const std::string failure = result == 0 || !articore_runtime_last_error()
        ? std::string{}
        : articore_runtime_last_error();
    auto report = last_enable_report();
    if (result != 0) throw RuntimeError("enable failed: " + failure);
    return report;
  }

  ArticoreMotorPowerState set_motor_power(const std::string& motor_name,
                                           bool enabled) {
    int32_t state = ARTICORE_MOTOR_POWER_UNKNOWN;
    detail::check(articore_runtime_set_motor_power(
                      checked(), motor_name.empty() ? nullptr
                                                    : motor_name.c_str(),
                      enabled ? 1 : 0, &state),
                  enabled ? "enable motor" : "disable motor");
    return static_cast<ArticoreMotorPowerState>(state);
  }

  ArticoreMotorPowerState motor_power_state(
      const std::string& motor_name = {}) const {
    int32_t state = ARTICORE_MOTOR_POWER_UNKNOWN;
    detail::check(articore_runtime_get_motor_power(
                      checked(), motor_name.empty() ? nullptr
                                                    : motor_name.c_str(),
                      &state),
                  "get motor power");
    return static_cast<ArticoreMotorPowerState>(state);
  }

  ArticoreEnableReport last_enable_report() const {
    ArticoreEnableReport report{};
    report.struct_size = sizeof(report);
    detail::check(
        articore_runtime_get_last_enable_report(checked(), &report),
        "get_last_enable_report");
    return report;
  }

  void start_gravity_compensation(uint32_t transition_ms = 0) {
    ArticoreGravityCompensationConfig config{};
    config.struct_size = sizeof(config);
    config.transition_ms = transition_ms;
    detail::check(articore_runtime_start_gravity_compensation(
                      checked(), &config),
                  "start_gravity_compensation");
  }

  void stop_gravity_compensation() {
    detail::check(articore_runtime_stop_gravity_compensation(checked()),
                  "stop_gravity_compensation");
  }

  ArticoreGravityCompensationStatus gravity_compensation_status() const {
    ArticoreGravityCompensationStatus result{};
    result.struct_size = sizeof(result);
    detail::check(articore_runtime_get_gravity_compensation_status(
                      checked(), &result),
                  "get_gravity_compensation_status");
    return result;
  }

  void set_joint_mit(const std::vector<ArticoreJointMitTarget>& targets,
                     float max_reference_velocity) {
    auto native = targets;
    for (auto& value : native) value.struct_size = sizeof(value);
    detail::check(articore_runtime_set_joint_mit(
                      checked(), native.data(), size(native),
                      max_reference_velocity),
                  "set_joint_mit");
  }

  void set_joint_pv(const std::vector<ArticoreJointPvTarget>& targets,
                    float max_reference_velocity) {
    auto native = targets;
    for (auto& value : native) value.struct_size = sizeof(value);
    detail::check(articore_runtime_set_joint_pv(
                      checked(), native.data(), size(native),
                      max_reference_velocity),
                  "set_joint_pv");
  }

  void submit_mit(const std::vector<ArticoreMitCommand>& commands,
                  ArticoreCommandLifetime lifetime =
                      ARTICORE_COMMAND_STREAMING) {
    detail::check(articore_runtime_submit_mit_ex(
                      checked(), commands.data(), size(commands), lifetime),
                  "submit_mit");
  }

  void submit_pos_vel(const std::vector<ArticorePosVelCommand>& commands,
                      ArticoreCommandLifetime lifetime =
                          ARTICORE_COMMAND_STREAMING) {
    detail::check(articore_runtime_submit_pos_vel_ex(
                      checked(), commands.data(), size(commands), lifetime),
                  "submit_pos_vel");
  }

  void set_gripper_commands(
      const std::vector<ArticoreGripperCommand>& commands) {
    auto native = commands;
    for (auto& value : native) value.struct_size = sizeof(value);
    detail::check(articore_runtime_set_gripper_commands(
                      checked(), native.data(), size(native)),
                  "set_gripper_commands");
  }

  ArticoreSafetyHealth health() const {
    ArticoreSafetyHealth result{};
    detail::check(articore_runtime_get_health(checked(), &result),
                  "get_health");
    return result;
  }

  ArticoreMitTorqueLimitStats mit_torque_limit_stats() const {
    ArticoreMitTorqueLimitStats result{};
    result.struct_size = sizeof(result);
    detail::check(articore_runtime_get_mit_torque_limit_stats(
                      checked(), &result),
                  "get_mit_torque_limit_stats");
    return result;
  }

  void estop(const std::string& reason) {
    detail::check(articore_runtime_estop(checked(), reason.c_str()), "estop");
  }

  void recover() {
    detail::check(articore_runtime_recover(checked()), "recover");
  }

  ArticoreDisableReport disable() {
    const auto result = articore_runtime_disable(checked());
    const std::string failure = result == 0 || !articore_runtime_last_error()
        ? std::string{}
        : articore_runtime_last_error();
    auto report = last_disable_report();
    if (result != 0) throw RuntimeError("disable failed: " + failure);
    return report;
  }

  ArticoreDisableReport last_disable_report() const {
    ArticoreDisableReport report{};
    report.struct_size = sizeof(report);
    detail::check(
        articore_runtime_get_last_disable_report(checked(), &report),
        "get_last_disable_report");
    return report;
  }

  void close() {
    if (!runtime_) return;
    detail::check(articore_runtime_close(runtime_), "close");
    articore_runtime_free(runtime_);
    runtime_ = nullptr;
  }

 private:
  template <typename T>
  static uint32_t size(const std::vector<T>& values) {
    if (values.size() > UINT32_MAX) {
      throw RuntimeError("Runtime command vector exceeds uint32 capacity");
    }
    return static_cast<uint32_t>(values.size());
  }

  ArticoreRuntime* checked() const {
    if (!runtime_) throw RuntimeError("Articore Runtime is closed");
    return runtime_;
  }

  ArticoreRuntime* runtime_ = nullptr;
  ArticoreMotorApi motor_api_{};
};

}  // namespace articore

#include "articore/detail/runtime.hpp"
#include "articore/detail/runtime_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace articore {

namespace {

class FunctionMotorBackend final : public MotorBackend {
 public:
  FunctionMotorBackend(ArticoreMotorApi api,
                       ArticoreControllerCallFn controller_enable_all,
                       ArticoreControllerCallFn motor_enable,
                       ArticoreMotorMaintenanceApi maintenance)
      : api_(api), controller_enable_all_(controller_enable_all),
        motor_enable_(motor_enable), maintenance_(maintenance) {}

  int32_t send_pos_vel(void* group, const ArticorePosVelCommand* commands,
                       uint32_t count) override {
    return api_.group_send_pos_vel(group, commands, count);
  }
  int32_t send_mit(void* group, const ArticoreMitCommand* commands,
                   uint32_t count) override {
    return api_.group_send_mit(group, commands, count);
  }
  int32_t disable_all(void* controller) override {
    return api_.controller_disable_all(controller);
  }
  int32_t request_feedback(
      void* controller, uint32_t timeout_ms, ArticoreFeedbackReport* report,
      uint32_t* missing, uint32_t capacity) override {
    return api_.controller_request_feedback_all_ex(
        controller, timeout_ms, report, missing, capacity);
  }
  int32_t get_state(void* motor, ArticoreMotorState* state) override {
    return api_.motor_get_state(motor, state);
  }
  int32_t get_feedback_stats(
      void* motor, ArticoreFeedbackStats* stats) override {
    return api_.motor_get_feedback_stats(motor, stats);
  }
  int32_t get_transport_health(
      void* controller, ArticoreDriverTransportHealth* health) override {
    return api_.controller_get_transport_health
        ? api_.controller_get_transport_health(controller, health) : 0;
  }
  bool has_transport_health() const override {
    return api_.controller_get_transport_health != nullptr;
  }
  int32_t disable_motor(void* motor) override {
    return api_.motor_disable(motor);
  }
  const char* last_error_message() const override {
    return api_.last_error_message ? api_.last_error_message() : "Motor error";
  }
  bool can_enable_all() const override { return controller_enable_all_ != nullptr; }
  int32_t enable_all(void* controller) override {
    return controller_enable_all_(controller);
  }
  bool can_enable_motor() const override { return motor_enable_ != nullptr; }
  int32_t enable_motor(void* motor) override { return motor_enable_(motor); }
  bool can_clear_error() const override {
    return maintenance_.motor_clear_error != nullptr;
  }
  int32_t clear_error(void* motor) override {
    return maintenance_.motor_clear_error(motor);
  }
  bool can_set_zero() const override {
    return maintenance_.motor_set_zero_position != nullptr;
  }
  int32_t set_zero(void* motor) override {
    return maintenance_.motor_set_zero_position(motor);
  }
  bool can_ensure_mode() const override {
    return maintenance_.motor_ensure_mode != nullptr;
  }
  int32_t ensure_mode(void* motor, uint32_t mode,
                      uint32_t timeout_ms) override {
    return maintenance_.motor_ensure_mode(motor, mode, timeout_ms);
  }
  bool can_set_timeout() const override {
    return maintenance_.motor_set_can_timeout_ms != nullptr;
  }
  int32_t set_timeout_ms(void* motor, uint32_t timeout_ms) override {
    return maintenance_.motor_set_can_timeout_ms(motor, timeout_ms);
  }
  uint32_t communication_timeout_ms() const override {
    return maintenance_.communication_timeout_ms;
  }

 private:
  ArticoreMotorApi api_{};
  ArticoreControllerCallFn controller_enable_all_ = nullptr;
  ArticoreControllerCallFn motor_enable_ = nullptr;
  ArticoreMotorMaintenanceApi maintenance_{};
};

std::shared_ptr<MotorBackend> make_function_backend(
    ArticoreMotorApi api, ArticoreControllerCallFn controller_enable_all,
    ArticoreControllerCallFn motor_enable,
    ArticoreMotorMaintenanceApi maintenance) {
  if (!api.group_send_pos_vel || !api.group_send_mit ||
      !api.controller_disable_all || !api.controller_request_feedback_all_ex ||
      !api.motor_get_state || !api.motor_get_feedback_stats ||
      !api.last_error_message || !api.motor_disable) {
    throw std::invalid_argument("Articore runtime motor API is incomplete");
  }
  return std::make_shared<FunctionMotorBackend>(
      api, controller_enable_all, motor_enable, maintenance);
}

}  // namespace

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
                             ArticoreControllerCallFn motor_enable,
                             bool require_gripper_product_profiles,
                             std::vector<ArticoreRuntimeTransportCapabilities>
                                 transport_capabilities,
                             ArticoreMotorMaintenanceApi maintenance_api,
                             uint32_t internal_control_rate_override)
    : SafetyRuntime(
          config,
          make_function_backend(api, controller_enable_all, motor_enable,
                                maintenance_api),
          controller_group, left_controller, right_controller,
          std::move(motors), require_gripper_product_profiles,
          std::move(transport_capabilities), internal_control_rate_override) {}

SafetyRuntime::SafetyRuntime(
    ArticoreRuntimeConfig config, std::shared_ptr<MotorBackend> backend,
    void* controller_group, void* left_controller, void* right_controller,
    std::vector<ArticoreMotorDescriptor> motors,
    bool require_gripper_product_profiles,
    std::vector<ArticoreRuntimeTransportCapabilities> transport_capabilities,
    uint32_t internal_control_rate_override)
    : config_(config), backend_(std::move(backend)),
      controller_group_(controller_group),
      require_gripper_product_profiles_(require_gripper_product_profiles) {
  controllers_[0] = left_controller;
  controllers_[1] = right_controller;
  if (!controller_group_) {
    throw std::invalid_argument("Articore runtime requires a controller group");
  }
  if (!backend_) throw std::invalid_argument("Articore runtime requires a Motor backend");
  if (config_.command_timeout_ms == 0 || config_.enable_grace_ms == 0 ||
      config_.safe_hold_hz == 0 ||
      config_.feedback_check_hz == 0 || config_.feedback_failure_threshold == 0 ||
      config_.feedback_max_age_ms == 0 ||
      config_.safe_hold_failure_threshold == 0 ||
      config_.disable_feedback_timeout_ms == 0 ||
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
      // ABI 2.2 production runtimes receive all product-owned values from a
      // named built-in profile before connect. Direct legacy construction is
      // retained for internal compatibility tests and advanced embedders.
      const bool legacy_descriptor = !require_gripper_product_profiles_;
      if (motor.open_position == motor.closed_position ||
          motor.normal_kp <= 0.0f || motor.close_speed <= 0.0f) {
        if (!legacy_descriptor) {
          record.gripper_state = ARTICORE_GRIPPER_DISABLED;
          motors_.push_back(std::move(record));
          continue;
        }
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
  mit_torque_limit_stats_.struct_size =
      sizeof(ArticoreMitTorqueLimitStats);
  gravity_control_.status.struct_size =
      sizeof(ArticoreGravityCompensationStatus);
  gravity_control_.status.phase = ARTICORE_GRAVITY_INACTIVE;
  bimanual_follow_.status.struct_size =
      sizeof(ArticoreBimanualFollowStatus);
  bimanual_follow_.status.phase = ARTICORE_BIMANUAL_FOLLOW_INACTIVE;
  std::fill(std::begin(mit_torque_limit_stats_.applied_scale),
            std::end(mit_torque_limit_stats_.applied_scale), 1.0f);
  mit_torque_limited_commands_.reserve(static_cast<std::size_t>(std::count_if(
      motors_.begin(), motors_.end(), [](const MotorRecord& motor) {
        return motor.descriptor.is_gripper == 0;
      })));
  degraded_pv_commands_.reserve(mit_torque_limited_commands_.capacity());
  degraded_mit_commands_.reserve(mit_torque_limited_commands_.capacity());
  const bool has_gripper = std::any_of(
      motors_.begin(), motors_.end(), [](const MotorRecord& motor) {
        return motor.descriptor.is_gripper != 0;
      });
  if (has_gripper && !require_gripper_product_profiles_ &&
      config_.gripper_fault_action != ARTICORE_GRIPPER_FAULT_HOLD &&
      config_.gripper_fault_action != ARTICORE_GRIPPER_FAULT_DISABLE) {
    throw std::invalid_argument("invalid legacy gripper fault action");
  }
  if ((!active_sides_[0] && !active_sides_[1]) ||
      (active_sides_[0] && !controllers_[0]) ||
      (active_sides_[1] && !controllers_[1])) {
    throw std::invalid_argument(
        "Articore runtime requires a controller for every active side");
  }
  std::array<bool, 2> capability_present{};
  for (const auto& capability : transport_capabilities) {
    if (capability.struct_size <
        sizeof(ArticoreRuntimeTransportCapabilities)) {
      throw std::invalid_argument(
          "Articore transport capability struct_size is too small");
    }
    if (capability.side > 1 || !active_sides_[capability.side] ||
        capability_present[capability.side]) {
      throw std::invalid_argument(
          "Articore transport capabilities must uniquely cover active sides");
    }
    capability_present[capability.side] = true;
    const auto* transport_end = capability.transport + sizeof(capability.transport);
    const auto* terminator =
        std::find(capability.transport, transport_end, '\0');
    if (terminator == transport_end) {
      throw std::invalid_argument(
          "Articore transport capability name is not NUL-terminated");
    }
  }
  if (!transport_capabilities.empty()) {
    for (uint8_t side = 0; side < 2; ++side) {
      if (active_sides_[side] && !capability_present[side]) {
        throw std::invalid_argument(
            "Articore transport capabilities do not cover every active side");
      }
    }
  }
  // Product control always runs at the fixed native 500 Hz cadence.
  control_hz_ = 500;
  if (internal_control_rate_override != 0) {
    control_hz_ = internal_control_rate_override;
  }
  if (const char* path = std::getenv("ARTICORE_RUNTIME_CONTROL_TRACE")) {
    if (path[0] != '\0') {
      control_trace_path_ = path;
      control_trace_.reserve(static_cast<std::size_t>(control_hz_) * 120U);
    }
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
  last_connect_report_.struct_size = sizeof(last_connect_report_);
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

void SafetyRuntime::configure_motor_identities(
    const ArticoreMotorIdentity* identities, uint32_t count) {
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_ != ARTICORE_DISCONNECTED || hardware_transition_) {
    throw std::runtime_error(
        "motor identities can only be configured before connect");
  }
  if (!identities || count != motors_.size()) {
    throw std::invalid_argument(
        "motor identities must cover the complete active motor set");
  }
  std::set<void*> handles;
  std::set<std::pair<uint8_t, uint32_t>> channel_ids;
  std::vector<std::pair<MotorRecord*, uint32_t>> resolved;
  resolved.reserve(count);
  for (uint32_t index = 0; index < count; ++index) {
    const auto& identity = identities[index];
    if (identity.struct_size < sizeof(ArticoreMotorIdentity) ||
        !identity.motor || identity.can_id > 0xFFU) {
      throw std::invalid_argument("invalid motor identity");
    }
    auto found = std::find_if(
        motors_.begin(), motors_.end(), [&](const MotorRecord& motor) {
          return motor.descriptor.motor == identity.motor;
        });
    if (found == motors_.end() || !handles.insert(identity.motor).second ||
        !channel_ids.emplace(found->descriptor.side, identity.can_id).second) {
      throw std::invalid_argument(
          "motor identities contain an unknown handle or duplicate channel ID");
    }
    resolved.emplace_back(&*found, identity.can_id);
  }
  for (const auto& [motor, can_id] : resolved) {
    motor->configured_can_id = can_id;
    motor->motor_identity_configured = true;
  }
}

ArticoreConnectReport SafetyRuntime::last_connect_report() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_connect_report_;
}

void SafetyRuntime::connect() {
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  ArticoreConnectReport report{};
  report.struct_size = sizeof(report);
  report.expected_count = static_cast<uint32_t>(motors_.size());
  report.channel_count = static_cast<uint32_t>(active_sides_[0]) +
                         static_cast<uint32_t>(active_sides_[1]);
  report.motor_count = static_cast<uint32_t>(motors_.size());
  for (uint8_t side = 0; side < 2; ++side) {
    report.channels[side].side = side;
    report.channels[side].active = active_sides_[side];
  }
  for (std::size_t index = 0; index < motors_.size(); ++index) {
    const auto& motor = motors_[index];
    auto& result = report.motors[index];
    result.side = motor.descriptor.side;
    result.configured_can_id = motor.motor_identity_configured
        ? motor.configured_can_id
        : 0;
    result.feedback_age_ns = std::numeric_limits<uint64_t>::max();
    copy_text(result.name, motor.descriptor.name);
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (stopping_) {
      throw std::runtime_error(
          "Runtime was terminally disconnected and cannot reconnect");
    }
    if (state_ != ARTICORE_DISCONNECTED) {
      return;
    }
    if (require_gripper_product_profiles_) {
      for (std::size_t index = 0; index < motors_.size(); ++index) {
        const auto& motor = motors_[index];
        if (motor.descriptor.is_gripper &&
            !motor.gripper_product_profile_bound) {
          report.error_code = ARTICORE_CONNECT_CONFIGURATION;
          report.failure_count = 1;
          copy_text(report.motors[index].error,
                    "built-in gripper product profile is required before connect");
          copy_text(report.error,
                    std::string(motor.descriptor.name) +
                        ": built-in gripper product profile is required before connect");
          last_connect_report_ = report;
          throw std::runtime_error(
              std::string(motor.descriptor.name) +
              ": built-in gripper product profile is required before connect");
        }
      }
    }
    hardware_transition_ = true;
    for (uint8_t side = 0; side < 2; ++side) {
      sides_[side] = SideHealth{};
      sides_[side].connected = active_sides_[side];
      sides_[side].healthy = false;
    }
  }

  std::vector<MissingMotor> missing_motors;
  std::string error;
  FeedbackTransactionResults side_results{};
  const bool request_complete = request_feedback_parallel(
      config_.disable_feedback_timeout_ms, missing_motors, error, &side_results);
  for (uint8_t side = 0; side < 2; ++side) {
    const auto& source = side_results[side];
    auto& target = report.channels[side];
    target.side = side;
    target.active = source.active;
    target.request_code = source.code;
    target.expected_count = source.report.expected_count;
    target.received_count = source.report.received_count;
    report.received_count += source.report.received_count;
    target.missing_count = static_cast<uint32_t>(source.missing.size());
    for (std::size_t index = 0;
         index < source.missing.size() && index < 32; ++index) {
      target.missing_motor_ids[index] = source.missing[index];
    }
    copy_text(target.error, source.error);
  }
  const bool snapshot_complete =
      validate_fresh_feedback_snapshot(missing_motors, error, &report);
  report.missing_count = static_cast<uint32_t>(missing_motors.size());
  if (!request_complete || !snapshot_complete) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    bool transport = false;
    bool any_received = false;
    bool unexpected_request_error = false;
    for (const auto& side : side_results) {
      transport = transport || side.code == 2;
      any_received = any_received || side.report.received_count > 0;
      unexpected_request_error = unexpected_request_error ||
          (side.active && side.code != 0 && side.code != 2 &&
           side.code != 3 && side.code != 4);
    }
    report.error_code = transport
        ? ARTICORE_CONNECT_TRANSPORT
        : (unexpected_request_error
               ? ARTICORE_CONNECT_FEEDBACK_INVALID
               : (!any_received
                      ? ARTICORE_CONNECT_FEEDBACK_TIMEOUT
                      : (request_complete
                             ? ARTICORE_CONNECT_FEEDBACK_INVALID
                             : ARTICORE_CONNECT_FEEDBACK_INCOMPLETE)));
    copy_text(report.error, error.empty()
                                ? std::string("incomplete motor feedback")
                                : error);
    last_connect_report_ = report;
    hardware_transition_ = false;
    for (uint8_t side = 0; side < 2; ++side) {
      if (!active_sides_[side]) continue;
      sides_[side].connected = false;
      sides_[side].healthy = false;
      if (!error.empty()) sides_[side].last_error = error;
    }
    throw std::runtime_error(
        "connect initial feedback transaction failed: " +
        (error.empty() ? std::string("incomplete motor feedback") : error));
  }

  bool physical_disable_confirmed = true;
  std::vector<std::string> detected_motor_faults;
  bool side_has_motor_fault[2] = {false, false};
  std::string side_motor_fault_error[2];
  for (const auto& motor : motors_) {
    ArticoreMotorState motor_state{};
    if (backend_->get_state(motor.descriptor.motor, &motor_state) != 0 ||
        !motor_state.has_value || motor_state.status_code != 0) {
      physical_disable_confirmed = false;
    }
    if (motor_state.has_value && motor_state.status_code > 1) {
      const std::string name(motor.descriptor.name);
      detected_motor_faults.push_back(name);
      side_has_motor_fault[motor.descriptor.side] = true;
      if (!side_motor_fault_error[motor.descriptor.side].empty()) {
        side_motor_fault_error[motor.descriptor.side] += "; ";
      }
      side_motor_fault_error[motor.descriptor.side] +=
          name + ": motor fault status " +
          std::to_string(motor_state.status_code);
    }
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    report.success = 1;
    report.error_code = ARTICORE_CONNECT_OK;
    report.missing_count = 0;
    report.failure_count = 0;
    report.received_count = static_cast<uint32_t>(motors_.size());
    report.error[0] = '\0';
    last_connect_report_ = report;
    const bool recoverable_motor_fault = !detected_motor_faults.empty();
    state_ = emergency_stop_latched_ || recoverable_motor_fault
        ? ARTICORE_FAULT : ARTICORE_READY;
    fault_latched_ = emergency_stop_latched_ || recoverable_motor_fault;
    disable_confirmed_ = physical_disable_confirmed;
    if (emergency_stop_latched_) {
      fault_reason_ = "emergency stop requested";
    } else if (recoverable_motor_fault) {
      fault_reason_ = "connect detected recoverable motor fault";
      for (const auto& name : detected_motor_faults) {
        fault_reason_ += ": " + name;
        presence_[name] = ARTICORE_FAULTED;
      }
    } else {
      fault_reason_.clear();
    }
    safety_reason_.clear();
    motor_faults_ = detected_motor_faults;
    unconfirmed_disable_.clear();
    hardware_transition_ = false;
    const auto now = Clock::now();
    const auto ready_refresh_hz = std::min(config_.feedback_check_hz, 10U);
    next_ready_feedback_ = now + std::chrono::nanoseconds(
        1'000'000'000ULL / ready_refresh_hz);
    for (uint8_t side = 0; side < 2; ++side) {
      if (!active_sides_[side]) continue;
      sides_[side].connected = true;
      sides_[side].healthy = !side_has_motor_fault[side];
      sides_[side].feedback_failures = 0;
      sides_[side].last_error = side_motor_fault_error[side];
    }
  }
  wakeup_.notify_all();
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

ArticoreControlMode SafetyRuntime::control_mode() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return mode_;
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

ArticoreMitTorqueLimitStats SafetyRuntime::mit_torque_limit_stats() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return mit_torque_limit_stats_;
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
    terminate_trajectory_locked(
        ARTICORE_TRAJECTORY_CANCELLED,
        "trajectory cancelled by Runtime disconnect");
    clear_pending_arm_mailbox();
    arm_mailbox_ = ArmMailbox{};
    first_command_accepted_ = false;
    enable_grace_transition_ = false;
    active_command_planning_token_ = 0;
    gravity_control_.phase = ARTICORE_GRAVITY_INACTIVE;
    gravity_control_.status.active = 0;
    gravity_control_.status.phase = ARTICORE_GRAVITY_INACTIVE;
    reset_bimanual_follow_locked();
    fault_hold_active_ = false;
  }
  wakeup_.notify_all();
  if (worker_.joinable()) worker_.join();
  write_control_trace();
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_ = ARTICORE_DISCONNECTED;
  safety_reason_.clear();
  for (auto& side : sides_) {
    side.connected = false;
    side.healthy = false;
  }
}

void SafetyRuntime::write_control_trace() noexcept {
  if (control_trace_written_ || control_trace_path_.empty()) return;
  control_trace_written_ = true;
  try {
    std::ofstream output(control_trace_path_, std::ios::out | std::ios::trunc);
    if (!output) return;
    output << "sequence,timestamp_ns,runtime_state,motion_state,trajectory_id,progress"
              ",tracking_time_scale,tracking_position_error";
    constexpr const char* roles[ARTICORE_PRODUCT_DUAL_ARM_DOF] = {
        "l-joint1", "l-joint2", "l-joint3", "l-joint4", "l-joint5",
        "l-joint6", "l-joint7", "r-joint1", "r-joint2", "r-joint3",
        "r-joint4", "r-joint5", "r-joint6", "r-joint7"};
    for (const char* role : roles) output << ",planned_q_" << role;
    for (const char* role : roles) output << ",planned_dq_" << role;
    for (const char* role : roles) output << ",command_q_" << role;
    for (const char* role : roles) output << ",pv_velocity_limit_" << role;
    for (const char* role : roles) output << ",actual_q_" << role;
    for (const char* role : roles) output << ",actual_dq_" << role;
    output << ",planned_valid_mask,command_valid_mask,actual_valid_mask\n";
    output << std::setprecision(9);
    for (const auto& sample : control_trace_) {
      output << sample.sequence << ',' << sample.timestamp_ns << ','
             << sample.runtime_state << ',' << sample.motion_state << ','
             << sample.trajectory_id << ',' << sample.progress << ','
             << sample.tracking_time_scale << ','
             << sample.tracking_position_error;
      const auto write_values = [&](const auto& values) {
        for (float value : values) output << ',' << value;
      };
      write_values(sample.planned_positions);
      write_values(sample.planned_velocities);
      write_values(sample.command_positions);
      write_values(sample.pv_velocity_limits);
      write_values(sample.actual_positions);
      write_values(sample.actual_velocities);
      output << ',' << sample.planned_valid_mask << ','
             << sample.command_valid_mask << ',' << sample.actual_valid_mask
             << '\n';
    }
  } catch (...) {
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
                      state_ == ARTICORE_DEGRADED ||
                      state_ == ARTICORE_SAFE_HOLD ||
                      state_ == ARTICORE_SAFE_STOP ||
                      state_ == ARTICORE_PARTIALLY_ENABLED;
    }
  }
  std::string failure;
  if (needs_disable) {
    try {
      disable();
    } catch (const std::exception& error) {
      failure = error.what();
    }
  }
  // Shutdown is terminal even when physical-disable confirmation fails. The
  // caller receives the failure after command production has stopped and the
  // worker has been joined, so no orphan background sender can survive.
  stop_worker();
  if (!failure.empty()) throw std::runtime_error(failure);
}

void SafetyRuntime::disconnect() {
  close();
}

}  // namespace articore

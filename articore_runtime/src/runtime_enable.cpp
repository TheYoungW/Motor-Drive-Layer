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
    std::string& error, FeedbackTransactionResults* output_results) {
  FeedbackTransactionResults local_results{};
  std::vector<std::thread> workers;
  for (uint8_t side = 0; side < 2; ++side) {
    if (!active_sides_[side]) continue;
    local_results[side].active = true;
    workers.emplace_back([&, side] {
      const auto motor_count = static_cast<uint32_t>(std::count_if(
          motors_.begin(), motors_.end(), [&](const MotorRecord& motor) {
            return motor.descriptor.side == side;
          }));
      auto& result = local_results[side];
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
    const auto& result = local_results[side];
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
        const uint32_t id = motor.motor_identity_configured
            ? motor.configured_can_id
            : (api_.motor_get_state(motor.descriptor.motor, &state) == 0 &&
                       state.has_value
                   ? state.can_id
                   : 0);
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
  if (output_results) *output_results = std::move(local_results);
  return ok;
}

bool SafetyRuntime::validate_fresh_feedback_snapshot(
    const std::vector<MissingMotor>& request_missing, std::string& error,
    ArticoreConnectReport* connect_report) {
  struct Snapshot {
    MotorRecord* motor = nullptr;
    uint32_t can_id = 0;
    bool has_can_id = false;
    uint32_t reported_can_id = 0;
    bool has_reported_can_id = false;
    bool valid = false;
    bool has_feedback = false;
    uint64_t update_count = 0;
    uint64_t age_ns = std::numeric_limits<uint64_t>::max();
    std::string reason;
  };

  std::vector<uint32_t> missing_ids[2];
  for (const auto& missing : request_missing) {
    if (missing.side > 1) continue;
    auto& ids = missing_ids[missing.side];
    if (std::find(ids.begin(), ids.end(), missing.id) == ids.end()) {
      ids.push_back(missing.id);
    }
  }

  const auto maximum_allowed_age =
      static_cast<uint64_t>(config_.feedback_max_age_ms) * 1'000'000ULL;
  uint64_t maximum_age[2] = {0, 0};
  bool side_valid[2] = {active_sides_[0], active_sides_[1]};
  std::vector<Snapshot> snapshots;
  snapshots.reserve(motors_.size());
  for (auto& motor : motors_) {
    Snapshot snapshot;
    snapshot.motor = &motor;
    ArticoreFeedbackStats stats{};
    ArticoreMotorState state{};
    const bool has_stats =
        api_.motor_get_feedback_stats(motor.descriptor.motor, &stats) == 0 &&
        stats.has_feedback;
    const bool has_state =
        api_.motor_get_state(motor.descriptor.motor, &state) == 0 &&
        state.has_value;
    if (has_state) {
      snapshot.has_can_id = true;
      snapshot.can_id = state.can_id;
      snapshot.has_reported_can_id = true;
      snapshot.reported_can_id = state.can_id;
    }
    snapshot.has_feedback = has_stats;
    snapshot.update_count = has_stats ? stats.update_count : 0;
    if (motor.motor_identity_configured) {
      snapshot.can_id = motor.configured_can_id;
      snapshot.has_can_id = true;
    }
    if (!has_stats) {
      snapshot.reason = "no motor feedback";
    } else {
      snapshot.age_ns = stats.age_ns;
      maximum_age[motor.descriptor.side] =
          std::max(maximum_age[motor.descriptor.side], stats.age_ns);
      if (stats.age_ns > maximum_allowed_age) {
        snapshot.reason = "feedback is stale: actual_age_ns=" +
            std::to_string(stats.age_ns) + ", threshold_age_ns<=" +
            std::to_string(maximum_allowed_age);
      } else if (!has_state) {
        snapshot.reason = "motor state is unavailable";
      } else if (motor.motor_identity_configured &&
                 state.can_id != motor.configured_can_id) {
        snapshot.reason = "feedback CAN ID does not match configured identity: actual=" +
            std::to_string(state.can_id) + ", configured=" +
            std::to_string(motor.configured_can_id);
      } else if (!finite(state.pos) || !finite(state.vel) ||
                 !finite(state.torq)) {
        snapshot.reason = "feedback contains a non-finite value";
      } else {
        const auto& ids = missing_ids[motor.descriptor.side];
        const bool missed_transaction = std::find(
            ids.begin(), ids.end(), static_cast<uint32_t>(state.can_id)) !=
            ids.end();
        if (missed_transaction) {
          snapshot.reason =
              "did not return new feedback in the connect/READY transaction";
        } else {
          snapshot.valid = true;
          if (motor.descriptor.is_gripper) {
            motor.last_position = state.pos;
            motor.has_position = true;
            motor.feedback_age_ns = stats.age_ns;
            motor.last_torque = state.torq;
          }
        }
      }
    }
    if (!snapshot.valid) side_valid[motor.descriptor.side] = false;
    snapshots.push_back(std::move(snapshot));
  }

  // A motor with no prior cache cannot reveal its CAN ID through get_state().
  // The controller's structured report still contains the missing IDs. Pair
  // them only when the mapping is unambiguous within that channel; otherwise
  // report the complete channel ID list without inventing an association.
  for (uint8_t side = 0; side < 2; ++side) {
    std::vector<Snapshot*> unknown;
    std::vector<uint32_t> remaining = missing_ids[side];
    for (auto& snapshot : snapshots) {
      if (snapshot.motor->descriptor.side != side || snapshot.valid) continue;
      if (!snapshot.has_can_id) {
        unknown.push_back(&snapshot);
        continue;
      }
      remaining.erase(
          std::remove(remaining.begin(), remaining.end(), snapshot.can_id),
          remaining.end());
    }
    if (unknown.size() == remaining.size()) {
      for (std::size_t index = 0; index < unknown.size(); ++index) {
        unknown[index]->can_id = remaining[index];
        unknown[index]->has_can_id = true;
      }
    }
  }

  std::string snapshot_error;
  for (const auto& snapshot : snapshots) {
    if (snapshot.valid) continue;
    if (!snapshot_error.empty()) snapshot_error += "; ";
    snapshot_error += "CH" +
        std::to_string(snapshot.motor->descriptor.side) + "/" +
        std::string(snapshot.motor->descriptor.name) + " (CAN ID ";
    snapshot_error += snapshot.has_can_id
        ? std::to_string(snapshot.can_id)
        : std::string("unavailable");
    snapshot_error += "): " + snapshot.reason;
  }
  for (uint8_t side = 0; side < 2; ++side) {
    const bool has_unknown = std::any_of(
        snapshots.begin(), snapshots.end(), [side](const Snapshot& snapshot) {
          return !snapshot.valid && snapshot.motor->descriptor.side == side &&
                 !snapshot.has_can_id;
        });
    if (!has_unknown || missing_ids[side].empty()) continue;
    snapshot_error += "; CH" + std::to_string(side) + " missing CAN IDs:";
    for (const auto id : missing_ids[side]) {
      snapshot_error += " " + std::to_string(id);
    }
  }

  const bool valid = snapshot_error.empty();
  if (connect_report) {
    connect_report->motor_count = static_cast<uint32_t>(
        std::min<std::size_t>(snapshots.size(), 32));
    uint32_t failure_count = 0;
    for (uint32_t index = 0; index < connect_report->motor_count; ++index) {
      const auto& snapshot = snapshots[index];
      auto& result = connect_report->motors[index];
      result.side = snapshot.motor->descriptor.side;
      result.has_feedback = snapshot.has_feedback;
      result.feedback_fresh = snapshot.has_feedback &&
          snapshot.age_ns <= maximum_allowed_age;
      result.feedback_valid = snapshot.valid;
      result.configured_can_id = snapshot.motor->motor_identity_configured
          ? snapshot.motor->configured_can_id
          : 0;
      result.reported_can_id = snapshot.has_reported_can_id
          ? snapshot.reported_can_id
          : 0;
      result.update_count = snapshot.update_count;
      result.feedback_age_ns = snapshot.age_ns;
      copy_text(result.name, snapshot.motor->descriptor.name);
      copy_text(result.error, snapshot.reason);
      if (!snapshot.valid) {
        ++failure_count;
      }
    }
    connect_report->failure_count = failure_count;
  }
  const auto now = Clock::now();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (uint8_t side = 0; side < 2; ++side) {
      if (!active_sides_[side]) continue;
      sides_[side].last_feedback_age_ns = maximum_age[side];
      if (side_valid[side]) {
        sides_[side].feedback_failures = 0;
      } else {
        ++sides_[side].feedback_failures;
        sides_[side].healthy = false;
        sides_[side].last_error = snapshot_error;
      }
    }
    if (valid) {
      const auto maximum_active_age = std::max(
          active_sides_[0] ? maximum_age[0] : 0,
          active_sides_[1] ? maximum_age[1] : 0);
      last_fresh_feedback_ =
          now - std::chrono::nanoseconds(maximum_active_age);
      consecutive_feedback_failures_ = 0;
    }
  }
  if (!snapshot_error.empty()) {
    if (!error.empty()) error += "; ";
    error += snapshot_error;
  }
  return valid;
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
  safety_reason_.clear();
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
  intentionally_disabled_motors_.clear();
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
      motor.force_level = ARTICORE_GRIPPER_FORCE_DEFAULT;
    }
  }
  next_feedback_check_ = enabled_at_;
  next_ready_feedback_ = {};
  next_gripper_control_ = enabled_at_;
  next_control_tick_ = enabled_at_;
  gravity_control_.phase = ARTICORE_GRAVITY_INACTIVE;
  gravity_control_.hold_positions.clear();
  gravity_control_.control_cycles = 0;
  gravity_control_.status = {};
  gravity_control_.status.struct_size = sizeof(gravity_control_.status);
  gravity_control_.status.phase = ARTICORE_GRAVITY_INACTIVE;
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

SafetyRuntime::MotorRecord* SafetyRuntime::resolve_motor_role(
    const std::string& role) {
  if (role.empty()) return nullptr;
  for (auto& motor : motors_) {
    const std::string name = motor.descriptor.name;
    if (name == role) return &motor;
    const auto stable = stable_motor_role(motor);
    if (stable == role) return &motor;
    const std::string stable_prefix = motor.descriptor.side == 0 ? "l-" : "r-";
    const std::string legacy_prefix = motor.descriptor.side == 0 ? "left/" : "right/";
    if (stable.compare(0, stable_prefix.size(), stable_prefix) == 0 &&
        legacy_prefix + stable.substr(stable_prefix.size()) == role) return &motor;
  }
  throw std::invalid_argument("unknown motor role: " + role);
}

std::string SafetyRuntime::stable_motor_role(const MotorRecord& motor) const {
  const std::string name = motor.descriptor.name;
  const std::string side_prefix =
      motor.descriptor.side == 0 ? "left/" : "right/";
  const std::string stable_prefix = motor.descriptor.side == 0 ? "l-" : "r-";
  if (name.compare(0, side_prefix.size(), side_prefix) == 0) {
    const auto suffix = name.substr(side_prefix.size());
    return suffix.compare(0, stable_prefix.size(), stable_prefix) == 0
        ? suffix : stable_prefix + suffix;
  }
  return name;
}

ArticoreMotorPowerState SafetyRuntime::cached_motor_power_state(
    const MotorRecord* selected) const {
  bool saw_enabled = false;
  bool saw_disabled = false;
  for (const auto& motor : motors_) {
    if (selected && &motor != selected) continue;
    ArticoreMotorState state{};
    if (api_.motor_get_state(motor.descriptor.motor, &state) != 0 ||
        !state.has_value || state.status_code > 1) {
      return ARTICORE_MOTOR_POWER_UNKNOWN;
    }
    saw_enabled = saw_enabled || state.status_code == 1;
    saw_disabled = saw_disabled || state.status_code == 0;
  }
  if (saw_enabled && saw_disabled) return ARTICORE_MOTOR_POWER_MIXED;
  if (saw_enabled) return ARTICORE_MOTOR_POWER_ENABLED;
  if (saw_disabled) return ARTICORE_MOTOR_POWER_DISABLED;
  return ARTICORE_MOTOR_POWER_UNKNOWN;
}

ArticoreMotorPowerState SafetyRuntime::motor_power_state(
    const std::string& motor_name) {
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  MotorRecord* selected = resolve_motor_role(motor_name);
  bool refresh = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ == ARTICORE_DISCONNECTED) {
      throw std::runtime_error("cannot query motor power while disconnected");
    }
    refresh = state_ == ARTICORE_READY ||
              state_ == ARTICORE_PARTIALLY_ENABLED;
    if (refresh) hardware_transition_ = true;
  }
  if (refresh) {
    wakeup_.notify_all();
    std::string error;
    std::vector<MissingMotor> missing;
    bool complete = false;
    {
      std::lock_guard<std::mutex> command_lock(command_mutex_);
      complete = request_feedback_parallel(
          config_.disable_feedback_timeout_ms, missing, error) &&
          validate_fresh_feedback_snapshot(missing, error);
    }
    if (!complete) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      hardware_transition_ = false;
      wakeup_.notify_all();
      throw std::runtime_error(
          error.empty() ? "motor power feedback verification failed" : error);
    }
  }
  const auto result = cached_motor_power_state(selected);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (refresh) {
      const auto all = cached_motor_power_state();
      if (all == ARTICORE_MOTOR_POWER_DISABLED) {
        state_ = ARTICORE_READY;
        disable_confirmed_ = true;
      } else {
        state_ = ARTICORE_PARTIALLY_ENABLED;
        disable_confirmed_ = false;
      }
      hardware_transition_ = false;
    }
  }
  if (refresh) wakeup_.notify_all();
  return result;
}

ArticoreMotorPowerReport SafetyRuntime::set_motor_power_batch(
    const std::vector<std::string>& motor_names, bool enabled) {
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  if (motor_names.empty()) {
    throw std::invalid_argument("motor power batch requires at least one role");
  }
  if (motor_names.size() > 32) {
    throw std::invalid_argument("motor power batch exceeds 32 roles");
  }

  std::vector<MotorRecord*> selected;
  selected.reserve(motor_names.size());
  for (const auto& role : motor_names) {
    if (role.empty()) throw std::invalid_argument("motor role is empty");
    auto* motor = resolve_motor_role(role);
    if (std::find(selected.begin(), selected.end(), motor) != selected.end()) {
      throw std::invalid_argument("duplicate motor role: " + role);
    }
    selected.push_back(motor);
  }

  ArticoreMotorPowerReport report{};
  report.struct_size = sizeof(report);
  report.requested_enabled = enabled ? 1 : 0;
  report.requested_count = static_cast<uint32_t>(selected.size());
  report.motor_count = report.requested_count;
  for (uint32_t i = 0; i < report.motor_count; ++i) {
    auto& output = report.motors[i];
    output.side = selected[i]->descriptor.side;
    output.requested_enabled = enabled ? 1 : 0;
    output.can_id = selected[i]->motor_identity_configured
        ? static_cast<uint8_t>(selected[i]->configured_can_id) : 0;
    copy_text(output.role, stable_motor_role(*selected[i]));
  }

  bool started_ready = false;
  ArticoreSafetyState started_state = ARTICORE_DISCONNECTED;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ == ARTICORE_DISCONNECTED || stopping_) {
      throw std::runtime_error(
          "motor power batch requires a connected Runtime");
    }
    if (enabled &&
        ((state_ != ARTICORE_READY &&
          state_ != ARTICORE_PARTIALLY_ENABLED) || fault_latched_)) {
      throw std::runtime_error(
          "motor enable batch requires READY or PARTIALLY_ENABLED");
    }
    if (enabled && !motor_enable_) {
      throw std::runtime_error("motor enable batch is unavailable");
    }
    if (hardware_transition_) {
      throw std::runtime_error("another Runtime hardware transaction is active");
    }
    started_state = state_;
    started_ready = started_state == ARTICORE_READY;
    terminate_trajectory_locked(
        ARTICORE_TRAJECTORY_CANCELLED,
        enabled ? "trajectory cancelled by motor enable"
                : "trajectory cancelled by motor disable");
    hardware_transition_ = true;
    if (!enabled) {
      for (auto* motor : selected) {
        intentionally_disabled_motors_.insert(motor->descriptor.motor);
      }
      gravity_control_.phase = ARTICORE_GRAVITY_INACTIVE;
      gravity_control_.status.active = 0;
      gravity_control_.status.phase = ARTICORE_GRAVITY_INACTIVE;
    }
  }
  wakeup_.notify_all();

  std::string error;
  std::vector<ArticoreMotorState> initial(selected.size());
  std::vector<MotorRecord*> newly_enabled;
  std::set<void*> command_sent;
  std::set<void*> rollback_sent;
  bool rollback_confirmed = true;
  bool transaction_ok = true;
  const auto max_age =
      static_cast<uint64_t>(config_.feedback_max_age_ms) * 1'000'000ULL;

  auto refresh = [&](std::string& detail) {
    std::vector<MissingMotor> missing;
    const bool requested = request_feedback_parallel(
        config_.disable_feedback_timeout_ms, missing, detail);
    const bool valid = validate_fresh_feedback_snapshot(missing, detail);
    return requested && valid;
  };

  auto selected_confirmed = [&](bool expected_enabled) {
    bool ok = true;
    for (auto* motor : selected) {
      ArticoreFeedbackStats stats{};
      ArticoreMotorState state{};
      const bool fresh =
          api_.motor_get_feedback_stats(motor->descriptor.motor, &stats) == 0 &&
          stats.has_feedback && stats.age_ns <= max_age;
      const bool readable =
          api_.motor_get_state(motor->descriptor.motor, &state) == 0 &&
          state.has_value;
      if (!fresh || !readable ||
          state.status_code != (expected_enabled ? 1 : 0)) {
        ok = false;
      }
    }
    return ok;
  };

  std::unique_lock<std::mutex> command_lock(command_mutex_);
  try {
    std::string preflight_error;
    if (!refresh(preflight_error)) {
      throw std::runtime_error(
          preflight_error.empty() ? "motor power preflight feedback failed"
                                  : preflight_error);
    }
    for (std::size_t i = 0; i < selected.size(); ++i) {
      if (api_.motor_get_state(selected[i]->descriptor.motor, &initial[i]) != 0 ||
          !initial[i].has_value || !finite(initial[i].pos)) {
        throw std::runtime_error(
            stable_motor_role(*selected[i]) +
            ": finite motor feedback is unavailable");
      }
      if (enabled && initial[i].status_code > 1) {
        throw std::runtime_error(
            stable_motor_role(*selected[i]) + ": motor reports fault status " +
            std::to_string(initial[i].status_code));
      }
    }
    if (enabled && started_ready) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      for (const auto& motor : motors_) {
        ArticoreMotorState state{};
        if (api_.motor_get_state(motor.descriptor.motor, &state) == 0 &&
            state.has_value && state.status_code == 0) {
          intentionally_disabled_motors_.insert(motor.descriptor.motor);
        }
      }
    }

    if (!enabled) {
      std::string barrier_error;
      if (!establish_disable_barrier(barrier_error)) {
        transaction_ok = false;
        error = "motor disable queue barrier failed: " + barrier_error;
      }
    }

    if (enabled) {
      for (std::size_t i = 0; i < selected.size(); ++i) {
        auto* motor = selected[i];
        if (initial[i].status_code == 1) continue;
        // A callback may report failure after the device accepted the command.
        // Include it in rollback before calling so the transaction remains
        // fail-closed even for ambiguous transport outcomes.
        newly_enabled.push_back(motor);
        if (motor_enable_(motor->descriptor.motor) != 0) {
          throw std::runtime_error(
              stable_motor_role(*motor) + ": " +
              motor_error("motor enable failed"));
        }
        command_sent.insert(motor->descriptor.motor);
      }

      std::vector<ArticorePosVelCommand> pv_holds;
      std::vector<ArticoreMitCommand> mit_holds;
      for (std::size_t i = 0; i < selected.size(); ++i) {
        auto* motor = selected[i];
        if (motor->descriptor.is_gripper || mode_ == ARTICORE_MODE_MIT) {
          float kp = motor->descriptor.is_gripper
              ? motor->descriptor.normal_kp : motor->descriptor.safe_kp;
          float kd = motor->descriptor.is_gripper
              ? motor->descriptor.normal_kd : motor->descriptor.safe_kd;
          const auto configured = joint_configs_.find(motor->descriptor.motor);
          if (!motor->descriptor.is_gripper && configured != joint_configs_.end()) {
            kp = configured->second.mit_kp;
            kd = configured->second.mit_kd;
          }
          mit_holds.push_back(ArticoreMitCommand{
              motor->descriptor.motor, initial[i].pos, 0.0f, kp, kd, 0.0f});
        } else {
          pv_holds.push_back(ArticorePosVelCommand{
              motor->descriptor.motor, initial[i].pos,
              config_.safe_pv_velocity_limit});
        }
      }
      if ((!pv_holds.empty() && api_.group_send_pos_vel(
              controller_group_, pv_holds.data(), pv_holds.size()) != 0) ||
          (!mit_holds.empty() && api_.group_send_mit(
              controller_group_, mit_holds.data(), mit_holds.size()) != 0)) {
        throw std::runtime_error(
            motor_error("motor enable current-position hold failed"));
      }
      std::string confirmation_error;
      if (!refresh(confirmation_error) || !selected_confirmed(true)) {
        throw std::runtime_error(
            confirmation_error.empty()
                ? "motor enable feedback confirmation failed"
                : confirmation_error);
      }
    } else {
      for (auto* motor : selected) {
        if (api_.motor_disable(motor->descriptor.motor) == 0) {
          command_sent.insert(motor->descriptor.motor);
        } else {
          transaction_ok = false;
          if (!error.empty()) error += "; ";
          error += stable_motor_role(*motor) + ": " +
                   motor_error("motor disable failed");
        }
      }
      std::string confirmation_error;
      bool confirmed = refresh(confirmation_error) && selected_confirmed(false);
      if (!confirmed) {
        for (auto* motor : selected) {
          ArticoreMotorState state{};
          if (api_.motor_get_state(motor->descriptor.motor, &state) == 0 &&
              state.has_value && state.status_code == 0) continue;
          if (api_.motor_disable(motor->descriptor.motor) == 0) {
            command_sent.insert(motor->descriptor.motor);
          }
        }
        confirmation_error.clear();
        confirmed = refresh(confirmation_error) && selected_confirmed(false);
      }
      if (!confirmed) {
        transaction_ok = false;
        if (!error.empty()) error += "; ";
        error += confirmation_error.empty()
            ? "motor disable feedback confirmation failed" : confirmation_error;
      }
    }
  } catch (const std::exception& failure) {
    transaction_ok = false;
    error = failure.what();
    if (enabled && !newly_enabled.empty()) {
      report.rollback_attempted = 1;
      for (auto* motor : newly_enabled) {
        if (api_.motor_disable(motor->descriptor.motor) == 0) {
          rollback_sent.insert(motor->descriptor.motor);
        }
      }
      std::string rollback_error;
      if (!refresh(rollback_error)) rollback_confirmed = false;
      for (auto* motor : newly_enabled) {
        ArticoreFeedbackStats stats{};
        ArticoreMotorState state{};
        if (api_.motor_get_feedback_stats(motor->descriptor.motor, &stats) != 0 ||
            !stats.has_feedback || stats.age_ns > max_age ||
            api_.motor_get_state(motor->descriptor.motor, &state) != 0 ||
            !state.has_value || state.status_code != 0) {
          rollback_confirmed = false;
        }
      }
      if (!rollback_confirmed) {
        error += "; enable rollback was not confirmed";
        if (!rollback_error.empty()) error += ": " + rollback_error;
      }
    }
  }
  command_lock.unlock();

  report.rollback_confirmed =
      report.rollback_attempted && rollback_confirmed ? 1 : 0;
  report.success = transaction_ok ? 1 : 0;
  report.command_sent_count = static_cast<uint32_t>(command_sent.size());
  for (uint32_t i = 0; i < report.motor_count; ++i) {
    auto* motor = selected[i];
    auto& output = report.motors[i];
    output.command_sent = command_sent.count(motor->descriptor.motor) ? 1 : 0;
    output.rollback_sent = rollback_sent.count(motor->descriptor.motor) ? 1 : 0;
    ArticoreFeedbackStats stats{};
    ArticoreMotorState state{};
    const bool has_stats =
        api_.motor_get_feedback_stats(motor->descriptor.motor, &stats) == 0 &&
        stats.has_feedback;
    const bool has_state =
        api_.motor_get_state(motor->descriptor.motor, &state) == 0 &&
        state.has_value;
    output.has_feedback = has_stats && has_state ? 1 : 0;
    output.feedback_fresh = output.has_feedback && stats.age_ns <= max_age;
    if (has_state) {
      output.can_id = static_cast<uint8_t>(state.can_id);
      output.status_code = state.status_code;
    }
    output.confirmed = transaction_ok && output.feedback_fresh &&
        output.status_code == (enabled ? 1 : 0);
    if (output.confirmed) {
      ++report.confirmed_count;
    } else {
      ++report.failure_count;
      copy_text(output.error, error.empty() ? "motor state was not confirmed"
                                            : error);
    }
  }
  copy_text(report.error, error);

  const auto all = cached_motor_power_state();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool rollback_safety_failure = enabled &&
        report.rollback_attempted && !report.rollback_confirmed;
    const bool preserve_abnormal_state = !enabled &&
        (started_state == ARTICORE_DEGRADED ||
         started_state == ARTICORE_SAFE_HOLD ||
         started_state == ARTICORE_SAFE_STOP ||
         started_state == ARTICORE_FAULT);
    if (enabled) {
      if (transaction_ok) {
        for (auto* motor : selected) {
          intentionally_disabled_motors_.erase(motor->descriptor.motor);
        }
      } else {
        for (std::size_t i = 0; i < selected.size(); ++i) {
          if (initial[i].has_value && initial[i].status_code == 0) {
            intentionally_disabled_motors_.insert(
                selected[i]->descriptor.motor);
          }
        }
      }
    }
    if (rollback_safety_failure) {
      state_ = ARTICORE_FAULT;
      fault_latched_ = true;
      disable_confirmed_ = false;
      fault_reason_ = error;
    } else if (all == ARTICORE_MOTOR_POWER_DISABLED) {
      disable_confirmed_ = true;
      if (preserve_abnormal_state) {
        state_ = started_state;
        for (const auto& motor : motors_) {
          intentionally_disabled_motors_.insert(motor.descriptor.motor);
        }
      } else if (!fault_latched_) {
        state_ = ARTICORE_READY;
        intentionally_disabled_motors_.clear();
      }
    } else {
      disable_confirmed_ = false;
      if (preserve_abnormal_state) {
        state_ = started_state;
      } else if (!fault_latched_) {
        state_ = ARTICORE_PARTIALLY_ENABLED;
      }
    }
    if (enabled && transaction_ok && !fault_latched_) {
      state_ = ARTICORE_PARTIALLY_ENABLED;
      disable_confirmed_ = false;
    }
    hardware_transition_ = false;
    next_control_tick_ = Clock::now();
    next_feedback_check_ = Clock::now();
    next_ready_feedback_ = Clock::now();
  }
  wakeup_.notify_all();
  return report;
}

ArticoreMotorPowerState SafetyRuntime::set_motor_power(
    const std::string& motor_name, bool enabled) {
  const auto report = set_motor_power_batch({motor_name}, enabled);
  if (!report.success) {
    throw std::runtime_error(report.error[0]
        ? report.error : "motor power transaction failed");
  }
  return enabled ? ARTICORE_MOTOR_POWER_ENABLED
                 : ARTICORE_MOTOR_POWER_DISABLED;
}

void SafetyRuntime::enable(ArticoreControlMode mode) {
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  if (mode != ARTICORE_MODE_PV && mode != ARTICORE_MODE_MIT) {
    throw std::invalid_argument("control mode must be PV or MIT");
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if ((state_ != ARTICORE_READY &&
         state_ != ARTICORE_PARTIALLY_ENABLED) ||
        fault_latched_ || hardware_transition_) {
      throw std::runtime_error(
          "Articore runtime can only enable from READY or PARTIALLY_ENABLED");
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
      clear_pending_arm_mailbox();
      arm_mailbox_ = ArmMailbox{};
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
    terminate_trajectory_locked(
        ARTICORE_TRAJECTORY_CANCELLED,
        "trajectory cancelled by Runtime disable");
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
    clear_pending_arm_mailbox();
    arm_mailbox_ = ArmMailbox{};
    gravity_control_.phase = ARTICORE_GRAVITY_INACTIVE;
    gravity_control_.hold_positions.clear();
    gravity_control_.status.active = 0;
    gravity_control_.status.phase = ARTICORE_GRAVITY_INACTIVE;
    has_successful_command_ = false;
    gripper_command_generation_ = 0;
    gripper_sent_generation_ = 0;
    hardware_transition_ = false;
    if (confirmed) {
      intentionally_disabled_motors_.clear();
      if (preserve_fault) {
        state_ = ARTICORE_FAULT;
        fault_latched_ = true;
        fault_reason_ = preserved_reason;
      } else {
        state_ = ARTICORE_READY;
        fault_reason_.clear();
        safety_reason_.clear();
        const auto ready_refresh_hz = std::min(config_.feedback_check_hz, 10U);
        next_ready_feedback_ = Clock::now() + std::chrono::nanoseconds(
            1'000'000'000ULL / ready_refresh_hz);
      }
    } else {
      state_ = ARTICORE_FAULT;
      fault_latched_ = true;
      fault_reason_ = preserve_fault && !preserved_reason.empty()
          ? preserved_reason + "; disable confirmation failed: " + error
          : "disable confirmation failed: " + error;
    }
  }
  wakeup_.notify_all();
  if (!confirmed) throw std::runtime_error(error);
}

void SafetyRuntime::estop() {
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  bool requires_hold = false;
  bool was_disable_confirmed = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (emergency_stop_latched_) {
      fault_latched_ = true;
      fault_reason_ = "emergency stop requested";
      safety_reason_.clear();
      return;
    }
    emergency_stop_latched_ = true;
    fault_latched_ = true;
    fault_reason_ = "emergency stop requested";
    safety_reason_.clear();
    if (state_ == ARTICORE_DISCONNECTED) return;
    was_disable_confirmed = disable_confirmed_;
    requires_hold = !disable_confirmed_ && state_ != ARTICORE_READY;
  }
  // Product emergency stop is a latched current-position stop, not torque-off.
  // Capture the replacement hold before clearing every user command/trajectory,
  // then let the existing native safety cadence keep transmitting it. A call
  // made while already physically disabled latches without re-enabling Motors.
  enter_fault("emergency stop requested", false, requires_hold);
  if (!requires_hold) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    disable_confirmed_ = was_disable_confirmed;
    fault_reason_ = "emergency stop requested";
    return;
  }

  bool arm_hold_available = false;
  {
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    arm_hold_available = mode_ == ARTICORE_MODE_PV
        ? !safe_pv_.empty() : !safe_mit_.empty();
    // These are diagnostic/fallback copies of the superseded user output. The
    // new safe_* vectors above are now the only active command reference.
    last_sent_pv_.clear();
    last_sent_mit_.clear();
  }
  if (!arm_hold_available) {
    const std::string error =
        "emergency stop current-position hold is unavailable";
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (fault_reason_.find(error) == std::string::npos) {
        fault_reason_ += "; " + error;
      }
    }
    throw std::runtime_error(error);
  }

  std::string hold_error;
  if (!send_safe_hold_once(hold_error)) {
    const std::string error = "emergency stop initial hold failed" +
        (hold_error.empty() ? std::string{} : ": " + hold_error);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (fault_reason_.find(error) == std::string::npos) {
        fault_reason_ += "; " + error;
      }
    }
    throw std::runtime_error(error);
  }
}

void SafetyRuntime::recover() {
  std::lock_guard<std::recursive_mutex> lifecycle_lock(lifecycle_mutex_);
  constexpr float kRecoverySpeedPercent = 5.0f;
  constexpr float kRecoveryPositionToleranceRad = 0.02f;
  constexpr float kRecoveryVelocityToleranceRadPerSecond = 0.05f;

  auto all_motor_names = [&] {
    std::vector<std::string> names;
    names.reserve(motors_.size());
    for (const auto& motor : motors_) {
      names.emplace_back(motor.descriptor.name);
    }
    return names;
  };
  auto arm_motor_names = [&] {
    std::vector<std::string> names;
    for (const auto& motor : motors_) {
      if (!motor.descriptor.is_gripper) {
        names.emplace_back(motor.descriptor.name);
      }
    }
    return names;
  };
  auto append_error = [](std::string& destination,
                         const std::string& addition) {
    if (addition.empty()) return;
    if (!destination.empty()) destination += "; ";
    destination += addition;
  };
  auto fail_recovery = [&](const char* stage, int32_t code,
                           const std::string& stage_error,
                           std::vector<std::string> failed_motors) -> void {
    {
      std::lock_guard<std::mutex> command_lock(command_mutex_);
      safe_pv_.clear();
      safe_mit_.clear();
      safe_grippers_.clear();
      last_sent_pv_.clear();
      last_sent_mit_.clear();
      fault_hold_active_ = false;
      clear_pending_arm_mailbox();
      arm_mailbox_ = ArmMailbox{};
    }
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      state_ = ARTICORE_FAULT;
      fault_latched_ = true;
      hardware_transition_ = true;
    }

    std::string disable_error;
    const bool disabled = disable_hardware(true, false, disable_error);
    std::string detail = std::string("recover failed during ") + stage +
                         ": " + stage_error;
    if (!disabled) append_error(detail, "fallback disable failed: " +
                                           disable_error);
    if (failed_motors.empty()) failed_motors = all_motor_names();
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      state_ = ARTICORE_FAULT;
      fault_latched_ = true;
      disable_confirmed_ = disabled;
      hardware_transition_ = false;
      enable_transaction_ = false;
      fault_reason_ = detail;
      safety_reason_.clear();
      last_operation_ = ARTICORE_OPERATION_RECOVER;
      last_operation_code_ = code;
      last_operation_error_ = detail;
      operation_failed_motors_ = failed_motors;
    }
    wakeup_.notify_all();
    throw std::runtime_error(detail);
  };

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_operation_ = ARTICORE_OPERATION_RECOVER;
    last_operation_code_ = ARTICORE_OPERATION_OK;
    last_operation_error_.clear();
    operation_failed_motors_.clear();
    const bool recoverable_state = state_ == ARTICORE_DEGRADED ||
        state_ == ARTICORE_SAFE_STOP ||
        (state_ == ARTICORE_FAULT && fault_latched_);
    if (!recoverable_state || hardware_transition_) {
      last_operation_code_ = ARTICORE_OPERATION_INVALID_STATE;
      last_operation_error_ =
          "recover requires DEGRADED, SAFE_STOP, or latched FAULT";
      throw std::runtime_error(
          "recover is only valid from DEGRADED, SAFE_STOP, or latched FAULT");
    }
    // Freeze the worker before the first recovery action. No old user target
    // may be sent while the product is being cleared, checked, or homed.
    state_ = ARTICORE_FAULT;
    fault_latched_ = true;
    hardware_transition_ = true;
  }
  {
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    safe_pv_.clear();
    safe_mit_.clear();
    safe_grippers_.clear();
    last_sent_pv_.clear();
    last_sent_mit_.clear();
    fault_hold_active_ = false;
    clear_pending_arm_mailbox();
    arm_mailbox_ = ArmMailbox{};
    gravity_control_.phase = ARTICORE_GRAVITY_INACTIVE;
    gravity_control_.hold_positions.clear();
    gravity_control_.status.active = 0;
    gravity_control_.status.phase = ARTICORE_GRAVITY_INACTIVE;
  }

  std::string error;
  if (!disable_hardware(true, false, error)) {
    fail_recovery("initial disable", ARTICORE_OPERATION_VERIFICATION, error,
                  all_motor_names());
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    disable_confirmed_ = true;
  }

  if (!maintenance_api_.motor_clear_error) {
    fail_recovery("clear recoverable faults", ARTICORE_OPERATION_UNSUPPORTED,
                  "native clear-fault callback is unavailable",
                  all_motor_names());
  }
  struct ClearResult {
    std::vector<std::string> failed;
    std::vector<std::string> errors;
  } clear_results[2];
  std::vector<std::thread> clear_workers;
  for (uint8_t side = 0; side < 2; ++side) {
    if (!active_sides_[side]) continue;
    clear_workers.emplace_back([&, side] {
      for (const auto& motor : motors_) {
        if (motor.descriptor.side != side) continue;
        if (maintenance_api_.motor_clear_error(motor.descriptor.motor) == 0) {
          continue;
        }
        clear_results[side].failed.emplace_back(motor.descriptor.name);
        clear_results[side].errors.emplace_back(
            motor_error("native motor clear-fault command failed"));
      }
    });
  }
  for (auto& worker : clear_workers) worker.join();
  std::vector<std::string> failed_motors;
  error.clear();
  for (const auto& result : clear_results) {
    failed_motors.insert(failed_motors.end(), result.failed.begin(),
                         result.failed.end());
    for (std::size_t index = 0; index < result.failed.size(); ++index) {
      append_error(error, result.failed[index] + ": " + result.errors[index]);
    }
  }
  if (!failed_motors.empty()) {
    fail_recovery("clear recoverable faults",
                  ARTICORE_OPERATION_MOTOR_COMMAND, error, failed_motors);
  }

  error.clear();
  if (!refresh_transport_health(error)) {
    fail_recovery("dual-arm health validation", ARTICORE_OPERATION_TRANSPORT,
                  error, all_motor_names());
  }
  std::vector<MissingMotor> missing_motors;
  if (!request_feedback_parallel(config_.disable_feedback_timeout_ms,
                                 missing_motors, error) ||
      !refresh_feedback_health(true, false, error)) {
    failed_motors.clear();
    for (const auto& motor : motors_) {
      ArticoreFeedbackStats stats{};
      ArticoreMotorState state{};
      const bool healthy =
          api_.motor_get_feedback_stats(motor.descriptor.motor, &stats) == 0 &&
          stats.has_feedback &&
          stats.age_ns <= static_cast<uint64_t>(config_.feedback_max_age_ms) *
                              1'000'000ULL &&
          api_.motor_get_state(motor.descriptor.motor, &state) == 0 &&
          state.has_value && finite(state.pos) && finite(state.vel) &&
          state.status_code == 0;
      if (!healthy) failed_motors.emplace_back(motor.descriptor.name);
    }
    fail_recovery("dual-arm health validation", ARTICORE_OPERATION_FEEDBACK,
                  error.empty() ? "complete fresh disabled feedback is required"
                                : error,
                  failed_motors);
  }

  float maximum_distance = 0.0f;
  for (const auto& motor : motors_) {
    if (motor.descriptor.is_gripper) continue;
    ArticoreMotorState state{};
    if (api_.motor_get_state(motor.descriptor.motor, &state) != 0 ||
        !state.has_value || !finite(state.pos)) {
      fail_recovery("dual-arm health validation", ARTICORE_OPERATION_FEEDBACK,
                    std::string(motor.descriptor.name) +
                        ": calibrated position is unavailable",
                    {motor.descriptor.name});
    }
    maximum_distance = std::max(maximum_distance, std::fabs(state.pos));
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = ARTICORE_READY;
    fault_latched_ = false;
    disable_confirmed_ = true;
    hardware_transition_ = false;
    fault_reason_.clear();
    safety_reason_.clear();
    motor_faults_.clear();
  }

  try {
    enable(mode_);
  } catch (const std::exception& exception) {
    fail_recovery("enable for zero return", ARTICORE_OPERATION_MOTOR_COMMAND,
                  exception.what(), all_motor_names());
  }

  try {
    std::vector<ArticoreJointMitTarget> mit_targets;
    std::vector<ArticoreJointPvTarget> pv_targets;
    for (const auto& motor : motors_) {
      if (motor.descriptor.is_gripper) continue;
      if (mode_ == ARTICORE_MODE_MIT) {
        mit_targets.push_back(ArticoreJointMitTarget{
            sizeof(ArticoreJointMitTarget), motor.descriptor.motor, 0.0f});
      } else {
        pv_targets.push_back(ArticoreJointPvTarget{
            sizeof(ArticoreJointPvTarget), motor.descriptor.motor, 0.0f});
      }
    }
    if (mode_ == ARTICORE_MODE_MIT) {
      set_joint_mit_speed(mit_targets.data(),
                          static_cast<uint32_t>(mit_targets.size()),
                          kRecoverySpeedPercent);
    } else {
      set_joint_pv_speed(pv_targets.data(),
                         static_cast<uint32_t>(pv_targets.size()),
                         kRecoverySpeedPercent);
    }
  } catch (const std::exception& exception) {
    fail_recovery("submit calibrated-zero return",
                  ARTICORE_OPERATION_MOTOR_COMMAND, exception.what(),
                  arm_motor_names());
  }

  const float recovery_velocity =
      ordinary_velocity_from_percent(mode_, kRecoverySpeedPercent);
  const auto expected_seconds = recovery_velocity > 0.0f
      ? maximum_distance / recovery_velocity + 3.0f
      : 120.0f;
  const auto timeout_seconds = std::clamp(expected_seconds, 5.0f, 120.0f);
  const auto deadline = Clock::now() + std::chrono::milliseconds(
      static_cast<int64_t>(timeout_seconds * 1000.0f));
  while (Clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    missing_motors.clear();
    error.clear();
    if (!request_feedback_parallel(config_.disable_feedback_timeout_ms,
                                   missing_motors, error)) {
      fail_recovery("calibrated-zero return feedback",
                    ARTICORE_OPERATION_FEEDBACK, error, arm_motor_names());
    }
    bool runtime_stopped = false;
    std::string runtime_stop_reason;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      runtime_stopped = state_ == ARTICORE_FAULT ||
          state_ == ARTICORE_SAFE_STOP || state_ == ARTICORE_DEGRADED;
      if (runtime_stopped) {
        runtime_stop_reason =
            fault_reason_.empty() ? safety_reason_ : fault_reason_;
      }
    }
    if (runtime_stopped) {
      if (runtime_stop_reason.empty()) {
        runtime_stop_reason = "Runtime stopped during calibrated-zero return";
      }
      fail_recovery("calibrated-zero return",
                    ARTICORE_OPERATION_VERIFICATION, runtime_stop_reason,
                    arm_motor_names());
    }
    failed_motors.clear();
    for (const auto& motor : motors_) {
      if (motor.descriptor.is_gripper) continue;
      ArticoreFeedbackStats stats{};
      ArticoreMotorState state{};
      const bool reached =
          api_.motor_get_feedback_stats(motor.descriptor.motor, &stats) == 0 &&
          stats.has_feedback &&
          stats.age_ns <= static_cast<uint64_t>(config_.feedback_max_age_ms) *
                              1'000'000ULL &&
          api_.motor_get_state(motor.descriptor.motor, &state) == 0 &&
          state.has_value && state.status_code == 1 && finite(state.pos) &&
          finite(state.vel) &&
          std::fabs(state.pos) <= kRecoveryPositionToleranceRad &&
          std::fabs(state.vel) <= kRecoveryVelocityToleranceRadPerSecond;
      if (!reached) failed_motors.emplace_back(motor.descriptor.name);
    }
    if (failed_motors.empty()) break;
  }
  if (!failed_motors.empty()) {
    fail_recovery("calibrated-zero return", ARTICORE_OPERATION_VERIFICATION,
                  "timed out waiting for all arm joints to reach calibrated zero",
                  failed_motors);
  }

  try {
    disable();
  } catch (const std::exception& exception) {
    fail_recovery("final whole-product disable",
                  ARTICORE_OPERATION_VERIFICATION, exception.what(),
                  all_motor_names());
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = ARTICORE_READY;
    fault_latched_ = false;
    emergency_stop_latched_ = false;
    disable_confirmed_ = true;
    fault_reason_.clear();
    safety_reason_.clear();
    consecutive_send_failures_ = 0;
    consecutive_feedback_failures_ = 0;
    motor_faults_.clear();
    unconfirmed_disable_.clear();
    last_operation_ = ARTICORE_OPERATION_RECOVER;
    last_operation_code_ = ARTICORE_OPERATION_OK;
    last_operation_error_.clear();
    operation_failed_motors_.clear();
    const auto ready_refresh_hz = std::min(config_.feedback_check_hz, 10U);
    next_ready_feedback_ = Clock::now() + std::chrono::nanoseconds(
        1'000'000'000ULL / ready_refresh_hz);
    for (auto& entry : presence_) {
      if (entry.second == ARTICORE_FAULTED) entry.second = ARTICORE_PRESENT;
    }
  }
  wakeup_.notify_all();
}

}  // namespace articore

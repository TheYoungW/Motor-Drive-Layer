#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <map>
#include <limits>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>
#include <vector>

#include "runtime.hpp"

namespace {

using namespace std::chrono_literals;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

template <typename Function>
void require_throws(const Function& function, const char* expected,
                    const char* message) {
  try {
    function();
  } catch (const std::exception& error) {
    if (!expected || std::string(error.what()).find(expected) !=
                         std::string::npos) {
      return;
    }
  }
  throw std::runtime_error(message);
}

struct FakeMotor {
  uint8_t status = 1;
  float position = 0.0f;
  float velocity = 0.0f;
  float torque = 0.0f;
  uint64_t age_ns = 0;
  bool has_feedback = true;
};

struct FakeDriver {
  std::mutex mutex;
  std::condition_variable send_cv;
  std::map<void*, FakeMotor> motors;
  std::vector<ArticorePosVelCommand> last_pv;
  std::vector<ArticoreMitCommand> last_mit;
  std::vector<ArticoreMitCommand> last_arm_mit;
  std::vector<std::vector<ArticorePosVelCommand>> pv_history;
  std::vector<std::vector<ArticoreMitCommand>> mit_history;
  std::vector<std::vector<ArticoreMitCommand>> group_mit_history;
  std::vector<std::vector<ArticoreMitCommand>> arm_mit_history;
  uint32_t pv_sends = 0;
  uint32_t mit_sends = 0;
  uint32_t disable_calls[2]{};
  uint32_t enable_calls[2]{};
  uint32_t motor_enable_calls = 0;
  void* fail_motor_enable = nullptr;
  uint32_t clear_fault_calls = 0;
  uint32_t set_zero_calls = 0;
  uint32_t configure_mode_calls = 0;
  uint32_t configure_timeout_calls = 0;
  void* fail_maintenance_motor = nullptr;
  uint32_t feedback_requests = 0;
  uint32_t feedback_stats_calls = 0;
  int32_t feedback_code = 0;
  uint32_t feedback_expected = 2;
  uint32_t feedback_received = 2;
  std::vector<uint32_t> feedback_missing_ids;
  bool feedback_uses_per_side_results = false;
  int32_t feedback_code_by_side[2]{};
  uint32_t feedback_expected_by_side[2]{1, 2};
  uint32_t feedback_received_by_side[2]{1, 2};
  std::vector<uint32_t> feedback_missing_ids_by_side[2];
  uint32_t feedback_timeouts_remaining = 0;
  std::set<uint32_t> feedback_timeout_calls;
  std::map<uint32_t, void*> feedback_enable_on_call;
  bool feedback_timeout_consumes_deadline = false;
  bool block_nonempty_send = false;
  float block_target_position = std::numeric_limits<float>::quiet_NaN();
  bool send_entered = false;
  bool release_send = false;
  std::vector<std::string> events;
  bool fail_group = false;
  uint32_t group_failures_remaining = 0;
  uint32_t group_send_failures = 0;
  bool fail_send_side[2]{};
  bool fail_left_disable = false;
  bool fail_enable[2]{};
  bool skip_left_enable_once = false;
  bool skip_gripper_enable_once = false;
  uint32_t motor_enable_delay_ms = 0;
  bool transport_connected[2]{true, true};
  bool transport_healthy[2]{true, true};
  bool emulate_arm_feedback = false;
  std::string error = "injected failure";
};

FakeDriver* g_driver = nullptr;
void* g_left_controller = reinterpret_cast<void*>(0x101);
void* g_right_controller = reinterpret_cast<void*>(0x102);

int32_t send_pv(void*, const ArticorePosVelCommand* commands, uint32_t count) {
  std::unique_lock<std::mutex> lock(g_driver->mutex);
  if (g_driver->fail_group) return -1;
  if (g_driver->group_failures_remaining > 0) {
    --g_driver->group_failures_remaining;
    ++g_driver->group_send_failures;
    return -1;
  }
  if (count > 0) {
    const bool left = commands[0].motor == reinterpret_cast<void*>(0x201);
    if (g_driver->fail_send_side[left ? 0 : 1]) return -1;
  }
  if (count == 0) {
    g_driver->events.emplace_back("barrier-pv");
  } else {
    if (g_driver->block_nonempty_send &&
        (!std::isfinite(g_driver->block_target_position) ||
         std::abs(commands[0].target_position -
                  g_driver->block_target_position) < 1e-6f)) {
      g_driver->send_entered = true;
      g_driver->send_cv.notify_all();
      g_driver->send_cv.wait(lock, [] { return g_driver->release_send; });
    }
    g_driver->last_pv.assign(commands, commands + count);
    g_driver->pv_history.emplace_back(commands, commands + count);
    if (g_driver->emulate_arm_feedback && count == 2) {
      for (uint32_t index = 0; index < count; ++index) {
        auto& motor = g_driver->motors[commands[index].motor];
        motor.position = commands[index].target_position;
        motor.velocity = 0.0f;
      }
    }
    g_driver->events.emplace_back("send-pv");
  }
  ++g_driver->pv_sends;
  return 0;
}

int32_t send_mit(void*, const ArticoreMitCommand* commands, uint32_t count) {
  std::unique_lock<std::mutex> lock(g_driver->mutex);
  if (g_driver->fail_group) return -1;
  if (g_driver->group_failures_remaining > 0) {
    --g_driver->group_failures_remaining;
    ++g_driver->group_send_failures;
    return -1;
  }
  if (count > 0) {
    const bool contains_left = std::any_of(
        commands, commands + count, [](const ArticoreMitCommand& command) {
          return command.motor == reinterpret_cast<void*>(0x201);
        });
    const bool contains_right = std::any_of(
        commands, commands + count, [](const ArticoreMitCommand& command) {
          return command.motor != reinterpret_cast<void*>(0x201);
        });
    if ((contains_left && g_driver->fail_send_side[0]) ||
        (contains_right && g_driver->fail_send_side[1])) {
      return -1;
    }
  }
  if (count == 0) {
    g_driver->events.emplace_back("barrier-mit");
    ++g_driver->mit_sends;
    return 0;
  }
  if (g_driver->block_nonempty_send &&
      (!std::isfinite(g_driver->block_target_position) ||
       std::abs(commands[0].target_position -
                g_driver->block_target_position) < 1e-6f)) {
    g_driver->send_entered = true;
    g_driver->send_cv.notify_all();
    g_driver->send_cv.wait(lock, [] { return g_driver->release_send; });
  }
  g_driver->group_mit_history.emplace_back(commands, commands + count);
  std::vector<ArticoreMitCommand> arm_commands;
  std::vector<ArticoreMitCommand> gripper_commands;
  for (uint32_t index = 0; index < count; ++index) {
    (commands[index].motor == reinterpret_cast<void*>(0x203)
         ? gripper_commands
         : arm_commands).push_back(commands[index]);
  }
  if (!arm_commands.empty()) {
    g_driver->mit_history.push_back(arm_commands);
    g_driver->arm_mit_history.push_back(arm_commands);
  }
  if (!gripper_commands.empty()) {
    g_driver->mit_history.push_back(gripper_commands);
    g_driver->last_mit = gripper_commands;
  } else {
    g_driver->last_mit = arm_commands;
  }
  g_driver->events.emplace_back("send-mit");
  if (!arm_commands.empty()) {
    g_driver->last_arm_mit = arm_commands;
    if (g_driver->emulate_arm_feedback) {
      for (const auto& command : arm_commands) {
        auto& motor = g_driver->motors[command.motor];
        motor.position = command.target_position;
        motor.velocity = command.target_velocity;
      }
    }
  }
  ++g_driver->mit_sends;
  return 0;
}

int32_t disable_all(void* controller) {
  std::lock_guard<std::mutex> lock(g_driver->mutex);
  const uint8_t side = controller == g_left_controller ? 0 : 1;
  ++g_driver->disable_calls[side];
  for (auto& entry : g_driver->motors) entry.second.status = 0;
  return side == 0 && g_driver->fail_left_disable ? -1 : 0;
}

int32_t enable_all(void* controller) {
  std::lock_guard<std::mutex> lock(g_driver->mutex);
  const uint8_t side = controller == g_left_controller ? 0 : 1;
  ++g_driver->enable_calls[side];
  if (g_driver->fail_enable[side]) return -1;
  for (auto& entry : g_driver->motors) {
    const bool is_left = entry.first == reinterpret_cast<void*>(0x201);
    if ((side == 0) != is_left) continue;
    if (side == 0 && g_driver->skip_left_enable_once) continue;
    if (side == 1 && entry.first == reinterpret_cast<void*>(0x203) &&
        g_driver->skip_gripper_enable_once) {
      continue;
    }
    entry.second.status = 1;
  }
  if (side == 0) g_driver->skip_left_enable_once = false;
  if (side == 1) g_driver->skip_gripper_enable_once = false;
  return 0;
}

int32_t enable_motor(void* handle) {
  uint32_t delay_ms = 0;
  {
    std::lock_guard<std::mutex> lock(g_driver->mutex);
    if (g_driver->motors.find(handle) == g_driver->motors.end()) return -1;
    ++g_driver->motor_enable_calls;
    if (g_driver->fail_motor_enable == handle) return -1;
    delay_ms = g_driver->motor_enable_delay_ms;
  }
  if (delay_ms > 0) std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
  std::lock_guard<std::mutex> lock(g_driver->mutex);
  g_driver->motors[handle].status = 1;
  return 0;
}

int32_t disable_motor(void* handle) {
  std::lock_guard<std::mutex> lock(g_driver->mutex);
  const auto found = g_driver->motors.find(handle);
  if (found == g_driver->motors.end()) return -1;
  const uint8_t side = handle == reinterpret_cast<void*>(0x201) ? 0 : 1;
  ++g_driver->disable_calls[side];
  g_driver->events.emplace_back(
      std::string("disable-") + std::to_string(side) + "-" +
      std::to_string(reinterpret_cast<std::uintptr_t>(handle) - 0x200U));
  if (side == 0 && g_driver->fail_left_disable) return -1;
  found->second.status = 0;
  return 0;
}

int32_t clear_motor_error(void* handle) {
  std::lock_guard<std::mutex> lock(g_driver->mutex);
  if (handle == g_driver->fail_maintenance_motor) return -1;
  ++g_driver->clear_fault_calls;
  const auto found = g_driver->motors.find(handle);
  if (found == g_driver->motors.end()) return -1;
  found->second.status = 0;
  return 0;
}

int32_t set_motor_zero(void* handle) {
  std::lock_guard<std::mutex> lock(g_driver->mutex);
  if (handle == g_driver->fail_maintenance_motor) return -1;
  ++g_driver->set_zero_calls;
  const auto found = g_driver->motors.find(handle);
  if (found == g_driver->motors.end()) return -1;
  found->second.position = 0.0f;
  found->second.velocity = 0.0f;
  return 0;
}

int32_t ensure_motor_mode(void* handle, uint32_t, uint32_t) {
  std::lock_guard<std::mutex> lock(g_driver->mutex);
  if (handle == g_driver->fail_maintenance_motor) return -1;
  ++g_driver->configure_mode_calls;
  return g_driver->motors.count(handle) ? 0 : -1;
}

int32_t set_motor_can_timeout(void* handle, uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(g_driver->mutex);
  if (handle == g_driver->fail_maintenance_motor || timeout_ms != 500) return -1;
  ++g_driver->configure_timeout_calls;
  return g_driver->motors.count(handle) ? 0 : -1;
}

int32_t request_feedback(void* controller, uint32_t timeout_ms,
                         ArticoreFeedbackReport* report,
                         uint32_t* missing_ids, uint32_t missing_capacity) {
  std::unique_lock<std::mutex> lock(g_driver->mutex);
  const uint8_t side = controller == g_left_controller ? 0 : 1;
  ++g_driver->feedback_requests;
  g_driver->events.emplace_back("feedback");
  const auto enable_on_call =
      g_driver->feedback_enable_on_call.find(g_driver->feedback_requests);
  if (enable_on_call != g_driver->feedback_enable_on_call.end()) {
    g_driver->motors[enable_on_call->second].status = 1;
  }
  const bool injected_timeout = g_driver->feedback_timeouts_remaining > 0 ||
      g_driver->feedback_timeout_calls.count(g_driver->feedback_requests) != 0;
  if (g_driver->feedback_timeouts_remaining > 0) {
    --g_driver->feedback_timeouts_remaining;
  }
  const auto feedback_code = g_driver->feedback_uses_per_side_results
      ? g_driver->feedback_code_by_side[side]
      : g_driver->feedback_code;
  const auto feedback_expected = g_driver->feedback_uses_per_side_results
      ? g_driver->feedback_expected_by_side[side]
      : g_driver->feedback_expected;
  const auto feedback_received = g_driver->feedback_uses_per_side_results
      ? g_driver->feedback_received_by_side[side]
      : g_driver->feedback_received;
  const auto& feedback_missing_ids = g_driver->feedback_uses_per_side_results
      ? g_driver->feedback_missing_ids_by_side[side]
      : g_driver->feedback_missing_ids;
  if (report) {
    report->struct_size = sizeof(*report);
    report->timeout_ms = timeout_ms;
    report->expected_count = feedback_expected;
    report->received_count = injected_timeout ? 0 : feedback_received;
    report->missing_count = injected_timeout
        ? feedback_expected
        : static_cast<uint32_t>(feedback_missing_ids.size());
    const auto copied = std::min<std::size_t>(
        feedback_missing_ids.size(), missing_capacity);
    for (std::size_t i = 0; i < copied; ++i) {
      missing_ids[i] = feedback_missing_ids[i];
    }
  }
  if (!injected_timeout) {
    for (auto& entry : g_driver->motors) {
      const bool motor_is_left =
          entry.first == reinterpret_cast<void*>(0x201);
      if ((side == 0) != motor_is_left) continue;
      const auto can_id = static_cast<uint32_t>(
          reinterpret_cast<std::uintptr_t>(entry.first) - 0x200U);
      if (std::find(feedback_missing_ids.begin(), feedback_missing_ids.end(),
                    can_id) == feedback_missing_ids.end()) {
        entry.second.has_feedback = true;
        entry.second.age_ns = 0;
      }
    }
  }
  const auto result = injected_timeout ? 3 : feedback_code;
  const bool consume_deadline =
      injected_timeout && g_driver->feedback_timeout_consumes_deadline;
  lock.unlock();
  if (consume_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms + 1));
  }
  return result;
}

int32_t get_state(void* handle, ArticoreMotorState* state) {
  std::lock_guard<std::mutex> lock(g_driver->mutex);
  const auto found = g_driver->motors.find(handle);
  if (found == g_driver->motors.end()) return -1;
  *state = {};
  state->has_value = found->second.has_feedback ? 1 : 0;
  state->can_id = static_cast<uint8_t>(
      reinterpret_cast<std::uintptr_t>(handle) - 0x200U);
  state->status_code = found->second.status;
  state->pos = found->second.position;
  state->vel = found->second.velocity;
  state->torq = found->second.torque;
  return 0;
}

int32_t get_feedback_stats(void* handle, ArticoreFeedbackStats* stats) {
  std::lock_guard<std::mutex> lock(g_driver->mutex);
  ++g_driver->feedback_stats_calls;
  const auto found = g_driver->motors.find(handle);
  if (found == g_driver->motors.end()) return -1;
  *stats = {};
  stats->has_feedback = found->second.has_feedback ? 1 : 0;
  stats->update_count = 1;
  stats->age_ns = found->second.age_ns;
  return 0;
}

const char* last_error() { return g_driver->error.c_str(); }

int32_t get_transport_health(void* controller,
                             ArticoreDriverTransportHealth* health) {
  std::lock_guard<std::mutex> lock(g_driver->mutex);
  const uint8_t side = controller == g_left_controller ? 0 : 1;
  *health = {};
  health->connected = g_driver->transport_connected[side] ? 1 : 0;
  health->healthy = g_driver->transport_healthy[side] ? 1 : 0;
  health->last_tx_age_ns = std::numeric_limits<uint64_t>::max();
  health->last_rx_age_ns = std::numeric_limits<uint64_t>::max();
  return 0;
}

ArticoreRuntimeConfig config() {
  ArticoreRuntimeConfig value{};
  value.reserved_control_rate = 0;
  value.command_timeout_ms = 30;
  value.enable_grace_ms = 60;
  value.safe_hold_hz = 100;
  value.feedback_check_hz = 100;
  value.feedback_failure_threshold = 3;
  value.feedback_max_age_ms = 200;
  value.safe_hold_failure_threshold = 1;
  value.disable_feedback_timeout_ms = 20;
  value.safe_pv_velocity_limit = 0.15f;
  value.reserved_gripper_control_rate = 0;
  value.gripper_fault_action = ARTICORE_GRIPPER_FAULT_DISABLE;
  return value;
}

std::vector<ArticoreRuntimeTransportCapabilities> transport_capabilities(
    const char* transport, bool can_fd, bool can_fd_brs) {
  std::vector<ArticoreRuntimeTransportCapabilities> values(2);
  for (uint32_t side = 0; side < values.size(); ++side) {
    values[side].struct_size = sizeof(values[side]);
    values[side].side = side;
    values[side].can_fd = can_fd ? 1 : 0;
    values[side].can_fd_brs = can_fd_brs ? 1 : 0;
    std::strncpy(values[side].transport, transport,
                 sizeof(values[side].transport) - 1);
  }
  return values;
}

ArticoreMotorApi api() {
  return ArticoreMotorApi{send_pv, send_mit, disable_all, request_feedback,
                          get_state, get_feedback_stats, last_error,
                          get_transport_health, disable_motor};
}

ArticoreMotorMaintenanceApi maintenance_api() {
  return ArticoreMotorMaintenanceApi{sizeof(ArticoreMotorMaintenanceApi),
                                     clear_motor_error, set_motor_zero,
                                     ensure_motor_mode,
                                     set_motor_can_timeout, 500};
}

std::vector<ArticoreMotorDescriptor> descriptors(FakeDriver& driver) {
  std::vector<ArticoreMotorDescriptor> values(3);
  void* handles[] = {reinterpret_cast<void*>(0x201), reinterpret_cast<void*>(0x202),
                     reinterpret_cast<void*>(0x203)};
  const char* names[] = {"left/joint1", "right/joint1", "right/gripper"};
  for (std::size_t i = 0; i < values.size(); ++i) {
    values[i].motor = handles[i];
    values[i].side = i == 0 ? 0 : 1;
    values[i].is_gripper = i == 2;
    std::strncpy(values[i].name, names[i], sizeof(values[i].name) - 1);
    values[i].safe_kp = i == 2 ? 2.0f : 5.0f;
    values[i].safe_kd = 0.5f;
    values[i].overload_torque = 1.5f;
    values[i].retreat_distance = 0.1f;
    values[i].retreat_retry_ms = 100;
    values[i].open_position = 0.0f;
    values[i].closed_position = 2.0f;
    values[i].normal_kp = i == 2 ? 4.0f : 0.0f;
    values[i].normal_kd = 0.5f;
    values[i].close_speed = i == 2 ? 1.0f : 0.0f;
    values[i].max_step_interval_ms = 50;
    values[i].closing_direction = 1.0f;
    values[i].lower_position = -2.0f;
    values[i].upper_position = 2.0f;
    driver.motors[handles[i]] =
        FakeMotor{1, static_cast<float>(i), 0.0f, 0.0f, 0, true};
  }
  return values;
}

std::vector<ArticoreMotorIdentity> motor_identities(
    const std::vector<ArticoreMotorDescriptor>& motors) {
  std::vector<ArticoreMotorIdentity> values;
  values.reserve(motors.size());
  for (const auto& motor : motors) {
    values.push_back(ArticoreMotorIdentity{
        sizeof(ArticoreMotorIdentity), motor.motor,
        static_cast<uint32_t>(reinterpret_cast<std::uintptr_t>(motor.motor) -
                              0x200U)});
  }
  return values;
}

std::vector<ArticoreJointControlConfig> joint_configs(
    const std::vector<ArticoreMotorDescriptor>& motors) {
  std::vector<ArticoreJointControlConfig> values;
  for (const auto& motor : motors) {
    if (motor.is_gripper) continue;
    values.push_back(ArticoreJointControlConfig{
        motor.motor, -2.0f, 2.0f, 5.0f, 10.0f, 20.0f, 3.0f, 0.0f});
  }
  return values;
}

std::vector<ArticoreGripperForceProfile> gripper_force_profiles(
    const std::vector<ArticoreMotorDescriptor>& motors) {
  const auto gripper = std::find_if(
      motors.begin(), motors.end(), [](const ArticoreMotorDescriptor& motor) {
        return motor.is_gripper != 0;
      });
  require(gripper != motors.end(), "force profile test requires a gripper");
  std::vector<ArticoreGripperForceProfile> values;
  values.reserve(10);
  for (int32_t level = ARTICORE_GRIPPER_FORCE_MIN;
       level <= ARTICORE_GRIPPER_FORCE_MAX; ++level) {
    // Keep level 5 identical to the former NORMAL product calibration while
    // interpolating four lighter and five stronger user-facing levels.
    const bool lighter = level <= ARTICORE_GRIPPER_FORCE_DEFAULT;
    const float ratio = lighter
        ? static_cast<float>(level - ARTICORE_GRIPPER_FORCE_MIN) / 4.0f
        : static_cast<float>(level - ARTICORE_GRIPPER_FORCE_DEFAULT) / 5.0f;
    const auto interpolate = [=](float low, float normal, float high) {
      return lighter ? low + (normal - low) * ratio
                     : normal + (high - normal) * ratio;
    };
    values.push_back(ArticoreGripperForceProfile{
        sizeof(ArticoreGripperForceProfile), gripper->motor, level,
        interpolate(0.5f, 0.8f, 1.2f),
        interpolate(1.0f, 1.5f, 2.0f),
        interpolate(3.0f, 4.0f, 6.0f),
        interpolate(0.3f, 0.5f, 0.8f),
        interpolate(1.0f, 2.0f, 3.0f),
        interpolate(0.2f, 0.5f, 0.7f)});
  }
  return values;
}

ArticoreGripperProductBinding gripper_product_binding(
    void* motor, const char* profile_id = "yunyi_gripper_v1") {
  ArticoreGripperProductBinding binding{};
  binding.struct_size = sizeof(binding);
  binding.motor = motor;
  std::strncpy(binding.profile_id, profile_id,
               sizeof(binding.profile_id) - 1);
  return binding;
}

std::vector<ArticoreJointSafetyLimits> layered_joint_limits(
    const std::vector<ArticoreMotorDescriptor>& motors,
    float hard_lower = -2.0f, float hard_upper = 2.0f,
    float soft_lower = -1.5f, float soft_upper = 1.5f,
    float braking_zone = 0.3f, float braking_acceleration = 2.0f) {
  std::vector<ArticoreJointSafetyLimits> values;
  for (const auto& motor : motors) {
    if (motor.is_gripper) continue;
    values.push_back(ArticoreJointSafetyLimits{
        sizeof(ArticoreJointSafetyLimits), motor.motor,
        hard_lower, hard_upper, soft_lower, soft_upper,
        braking_zone, braking_acceleration});
  }
  return values;
}

template <typename Predicate>
bool wait_for(Predicate predicate, std::chrono::milliseconds timeout = 300ms) {
#if defined(__APPLE__) || defined(_WIN32)
  // Hosted macOS and Windows runners have substantially coarser scheduling
  // under load. The assertions below validate generated cycle counts and
  // per-cycle values, so allow wall-clock jitter without weakening those
  // deterministic checks.
  timeout *= 4;
#endif
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) return true;
    std::this_thread::sleep_for(2ms);
  }
  return predicate();
}

void test_connect_is_a_complete_feedback_barrier_and_ready_refreshes_cache() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  for (auto& entry : driver.motors) {
    entry.second.has_feedback = false;
    entry.second.age_ns = std::numeric_limits<uint64_t>::max();
  }
  articore::SafetyRuntime runtime(config(), api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  require(runtime.health().state == ARTICORE_READY,
          "connect enters READY only after the feedback barrier");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.feedback_requests == 2,
            "connect requests CH0 and CH1 feedback in parallel");
    require(std::all_of(
                driver.motors.begin(), driver.motors.end(),
                [](const auto& entry) { return entry.second.has_feedback; }),
            "connect populates every joint and gripper cache");
    driver.motors[motors[2].motor].has_feedback = false;
  }
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.feedback_requests >= 4 &&
                   driver.motors[motors[2].motor].has_feedback;
          }),
          "READY performs a bounded low-rate full-cache refresh");
}

void test_runtime_motor_power_supports_single_and_whole_product_queries() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  for (auto& entry : driver.motors) entry.second.status = 0;
  articore::SafetyRuntime runtime(
      config(), api(), reinterpret_cast<void*>(0x100), g_left_controller,
      g_right_controller, motors, enable_all, enable_motor);
  runtime.connect();

  require(runtime.motor_power_state("") == ARTICORE_MOTOR_POWER_DISABLED &&
              runtime.motor_power_state("left/joint1") ==
                  ARTICORE_MOTOR_POWER_DISABLED,
          "whole-product and single-motor power queries report disabled");
  require(runtime.set_motor_power("left/joint1", true) ==
              ARTICORE_MOTOR_POWER_ENABLED &&
              runtime.health().state == ARTICORE_PARTIALLY_ENABLED &&
              runtime.motor_power_state("") == ARTICORE_MOTOR_POWER_MIXED,
          "single-motor enable is confirmed and enters PARTIALLY_ENABLED");

  ArticorePosVelCommand commands[] = {
      {motors[0].motor, 0.1f, 1.0f},
      {motors[1].motor, 0.1f, 1.0f},
  };
  std::size_t baseline = 0;
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    baseline = driver.pv_history.size();
  }
  runtime.submit_pos_vel(commands, 2);
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.pv_history.size() > baseline;
          }), "complete control frames remain accepted while partially enabled");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    const auto& sent = driver.pv_history.back();
    require(sent.size() == 1 && sent[0].motor == motors[0].motor,
            "the worker filters intentionally disabled motors at dispatch");
  }

  require(runtime.set_motor_power("left/joint1", false) ==
              ARTICORE_MOTOR_POWER_DISABLED &&
              runtime.health().state == ARTICORE_READY &&
              runtime.motor_power_state("") == ARTICORE_MOTOR_POWER_DISABLED,
          "single-motor disable is confirmed and restores READY");
  require_throws([&] { runtime.set_motor_power("left/not-a-motor", true); },
                 "unknown motor role",
                 "unknown product motor selectors are rejected");

  runtime.set_motor_power("left/joint1", true);
  runtime.enable(ARTICORE_MODE_PV);
  require(runtime.health().state == ARTICORE_ENABLED &&
              runtime.motor_power_state("") == ARTICORE_MOTOR_POWER_ENABLED,
          "whole-product enable can safely complete from PARTIALLY_ENABLED");
  runtime.disable();
  require(runtime.motor_power_state("") == ARTICORE_MOTOR_POWER_DISABLED,
          "whole-product disable returns a confirmed disabled query result");
}

void test_motor_power_batch_rolls_back_failed_enable_atomically() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  for (auto& entry : driver.motors) entry.second.status = 0;
  driver.fail_motor_enable = motors[1].motor;
  articore::SafetyRuntime runtime(
      config(), api(), reinterpret_cast<void*>(0x100), g_left_controller,
      g_right_controller, motors, enable_all, enable_motor);
  runtime.connect();

  const auto report = runtime.set_motor_power_batch(
      {"l-joint1", "r-joint1"}, true);
  require(!report.success && report.rollback_attempted &&
              report.rollback_confirmed && report.motor_count == 2 &&
              report.failure_count == 2,
          "failed batch enable returns a complete rollback report");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.motors[motors[0].motor].status == 0 &&
                driver.motors[motors[1].motor].status == 0,
            "a later enable failure rolls back motors changed by the batch");
  }
  require(runtime.health().state == ARTICORE_READY,
          "confirmed rollback restores READY rather than partial power");
  require_throws(
      [&] { runtime.set_motor_power_batch({"l-joint1", "left/joint1"}, true); },
      "duplicate motor role", "batch aliases cannot select one motor twice");
  require_throws(
      [&] { runtime.set_motor_power_batch({}, false); },
      "at least one role", "empty batch selections are rejected natively");
}

void test_motor_power_batch_latches_fault_when_rollback_is_unconfirmed() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  for (auto& entry : driver.motors) entry.second.status = 0;
  driver.fail_motor_enable = motors[1].motor;
  driver.fail_left_disable = true;
  articore::SafetyRuntime runtime(
      config(), api(), reinterpret_cast<void*>(0x100), g_left_controller,
      g_right_controller, motors, enable_all, enable_motor);
  runtime.connect();

  const auto report = runtime.set_motor_power_batch(
      {"l-joint1", "r-joint1"}, true);
  require(!report.success && report.rollback_attempted &&
              !report.rollback_confirmed &&
              runtime.health().state == ARTICORE_FAULT,
          "unconfirmed rollback is latched as a Runtime fault");
}

void test_partial_mit_filters_disabled_motors_before_torque_validation() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  for (auto& entry : driver.motors) entry.second.status = 0;
  articore::SafetyRuntime runtime(
      config(), api(), reinterpret_cast<void*>(0x100), g_left_controller,
      g_right_controller, motors, enable_all, enable_motor);
  const auto controls = joint_configs(motors);
  runtime.configure_joints(controls.data(),
                           static_cast<uint32_t>(controls.size()));
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);
  require(runtime.set_motor_power_batch({"r-joint1"}, false).success,
          "partial MIT test disables the selected motor");

  ArticoreMitCommand commands[] = {
      {motors[0].motor, 0.1f, 0.0f, 1.0f, 0.1f, 0.0f},
      {motors[1].motor, 0.1f, 0.0f, 1.0f, 0.1f, 0.0f},
  };
  runtime.submit_mit(commands, 2);
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return !driver.last_arm_mit.empty() &&
                   driver.last_arm_mit.size() == 1 &&
                   driver.last_arm_mit[0].motor == motors[0].motor;
          }), "partial MIT dispatch validates and sends only enabled motors");
  require(runtime.health().state == ARTICORE_PARTIALLY_ENABLED,
          "partial MIT control does not misclassify disabled feedback as a fault");
}

void test_runtime_maintenance_keeps_ready_and_reports_partial_failure() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  driver.emulate_arm_feedback = true;
  for (auto& entry : driver.motors) entry.second.status = 0;
  articore::SafetyRuntime runtime(
      config(), api(), reinterpret_cast<void*>(0x100), g_left_controller,
      g_right_controller, motors, enable_all, enable_motor, false, {},
      maintenance_api());
  const auto controls = joint_configs(motors);
  runtime.configure_joints(controls.data(),
                           static_cast<uint32_t>(controls.size()));
  runtime.connect();

  require(runtime.set_zero() == ARTICORE_OPERATION_OK,
          "whole-runtime zero succeeds");
  auto health = runtime.health_v2();
  require(health.health.state == ARTICORE_READY &&
              health.last_operation == ARTICORE_OPERATION_SET_ZERO &&
              health.last_operation_code == ARTICORE_OPERATION_OK &&
              health.operation_failed_motor_count == 0 &&
              health.health.fault_reason[0] == '\0',
          "successful zero remains READY and clears operation diagnostics");
  require(driver.set_zero_calls == motors.size(),
          "zero covers every installed arm and gripper motor");

  require(runtime.configure_mode(ARTICORE_MODE_MIT) == ARTICORE_OPERATION_OK,
          "Runtime-owned mode configuration succeeds");
  require(driver.configure_mode_calls == motors.size(),
          "mode configuration covers both channels");
  require(driver.configure_timeout_calls == motors.size(),
          "mode configuration also owns the product communication watchdog");
  require(runtime.configure_mode(static_cast<ArticoreControlMode>(99)) ==
              ARTICORE_OPERATION_INVALID_ARGUMENT &&
              runtime.health_v2().last_operation_code ==
                  ARTICORE_OPERATION_INVALID_ARGUMENT,
          "invalid mode is reported without entering a hardware transaction");
  require(runtime.clear_faults() == ARTICORE_OPERATION_OK,
          "Runtime-owned fault clear succeeds");

  runtime.record_operation_result(
      ARTICORE_OPERATION_COMMAND, ARTICORE_OPERATION_INVALID_ARGUMENT,
      "joint 4 exceeds product position limits");
  health = runtime.health_v2();
  require(health.health.state == ARTICORE_READY &&
              health.health.fault_reason[0] == '\0' &&
              health.last_operation == ARTICORE_OPERATION_COMMAND &&
              health.last_operation_code == ARTICORE_OPERATION_INVALID_ARGUMENT &&
              std::string(health.last_operation_error).find("joint 4") !=
                  std::string::npos,
          "rejected product command is diagnostic without inventing a fault");
  runtime.record_operation_result(
      ARTICORE_OPERATION_COMMAND, ARTICORE_OPERATION_OK);

  runtime.estop();
  health = runtime.health_v2();
  require(health.health.state == ARTICORE_FAULT &&
              health.health.disable_confirmed == 1 &&
              std::string(health.health.fault_reason) ==
                  "emergency stop requested",
          "estop records the standard reason and confirms full disable");
  require(runtime.clear_faults() == ARTICORE_OPERATION_INVALID_STATE &&
              runtime.health().state == ARTICORE_FAULT,
          "clear faults cannot release an emergency-stop latch");
  runtime.recover();
  health = runtime.health_v2();
  require(health.health.state == ARTICORE_READY &&
              health.health.disable_confirmed == 1 &&
              health.last_operation == ARTICORE_OPERATION_RECOVER &&
              health.last_operation_code == ARTICORE_OPERATION_OK,
          "recover returns calibrated-zero product to confirmed READY");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[0].motor].status = 8;
  }
  require(runtime.clear_faults() == ARTICORE_OPERATION_OK,
          "fault clear is available while a disabled Runtime is FAULT");
  health = runtime.health_v2();
  require(health.health.state == ARTICORE_READY &&
              health.health.fault_reason[0] == '\0' &&
              health.health.motor_fault_count == 0,
          "successful fault clear removes the latch and returns READY");

  driver.fail_maintenance_motor = motors[1].motor;
  require(runtime.set_zero() == ARTICORE_OPERATION_MOTOR_COMMAND,
          "partial zero returns a stable motor-command error");
  health = runtime.health_v2();
  require(health.health.state == ARTICORE_FAULT &&
              health.last_operation_code == ARTICORE_OPERATION_MOTOR_COMMAND &&
              health.operation_failed_motor_count == 1 &&
              std::string(health.operation_failed_motors[0]) ==
                  motors[1].name &&
              health.health.fault_reason[0] != '\0',
          "partial zero is never reported as success and is visible in health");

  require_throws([&] { runtime.recover(); },
                 "recover failed during clear recoverable faults",
                 "recover reports the exact failed whole-product stage");
  health = runtime.health_v2();
  require(health.health.state == ARTICORE_FAULT &&
              health.health.disable_confirmed == 1 &&
              health.last_operation == ARTICORE_OPERATION_RECOVER &&
              health.last_operation_code == ARTICORE_OPERATION_MOTOR_COMMAND &&
              std::string(health.last_operation_error).find(
                  "clear recoverable faults") != std::string::npos &&
              health.operation_failed_motor_count == 1 &&
              std::string(health.operation_failed_motors[0]) ==
                  motors[1].name,
          "failed recover falls back to confirmed disable and records motor identity");
}

void test_disconnect_is_terminal_and_idempotent() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  for (auto& entry : driver.motors) entry.second.status = 0;
  articore::SafetyRuntime runtime(
      config(), api(), reinterpret_cast<void*>(0x100), g_left_controller,
      g_right_controller, motors, nullptr, nullptr, false, {},
      maintenance_api());
  runtime.connect();
  runtime.disconnect();
  require(runtime.health().state == ARTICORE_DISCONNECTED,
          "disconnect stops the worker and returns DISCONNECTED");
  runtime.disconnect();
  require(runtime.health().state == ARTICORE_DISCONNECTED,
          "repeated disconnect is an idempotent no-op");
  require_throws([&] { runtime.connect(); }, "terminally disconnected",
                 "a terminally disconnected Runtime cannot reconnect");
}

void test_connect_failure_names_missing_installed_motor_and_can_id() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  driver.motors[motors[2].motor].has_feedback = false;
  driver.feedback_uses_per_side_results = true;
  driver.feedback_code_by_side[1] = 4;
  driver.feedback_received_by_side[1] = 1;
  driver.feedback_missing_ids_by_side[1] = {3};
  articore::SafetyRuntime runtime(config(), api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  const auto identities = motor_identities(motors);
  runtime.configure_motor_identities(
      identities.data(), static_cast<uint32_t>(identities.size()));
  require_throws(
      [&] { runtime.connect(); },
      "CH1/right/gripper (CAN ID 3): no motor feedback",
      "connect failure reports channel, configured name, and CAN ID");
  const auto failed = runtime.last_connect_report();
  require(failed.success == 0 &&
              failed.error_code == ARTICORE_CONNECT_FEEDBACK_INCOMPLETE &&
              failed.expected_count == 3 && failed.received_count == 2 &&
              failed.missing_count == 1,
          "connect failure exposes stable counts and error classification");
  require(failed.channels[1].missing_count == 1 &&
              failed.channels[1].missing_motor_ids[0] == 3,
          "connect report preserves the failing channel and missing CAN ID");
  require(failed.motors[2].configured_can_id == 3 &&
              std::string(failed.motors[2].name) == "right/gripper" &&
              failed.motors[2].feedback_valid == 0,
          "connect report maps a cacheless motor to its configured identity");
  require(runtime.health().state == ARTICORE_DISCONNECTED,
          "failed feedback barrier does not expose READY");
  driver.feedback_code_by_side[1] = 0;
  driver.feedback_received_by_side[1] = 2;
  driver.feedback_missing_ids_by_side[1].clear();
  runtime.connect();
  const auto succeeded = runtime.last_connect_report();
  require(succeeded.success == 1 &&
              succeeded.error_code == ARTICORE_CONNECT_OK &&
              succeeded.received_count == 3 && succeeded.failure_count == 0,
          "successful connect report covers every configured motor");
  require(runtime.health().state == ARTICORE_READY,
          "connect can be retried after feedback becomes complete");
}

void test_connect_report_classifies_zero_feedback_and_transport_failures() {
  {
    FakeDriver driver;
    g_driver = &driver;
    auto motors = descriptors(driver);
    for (auto& entry : driver.motors) entry.second.has_feedback = false;
    driver.feedback_uses_per_side_results = true;
    driver.feedback_code_by_side[0] = 3;
    driver.feedback_code_by_side[1] = 3;
    driver.feedback_received_by_side[0] = 0;
    driver.feedback_received_by_side[1] = 0;
    driver.feedback_missing_ids_by_side[0] = {1};
    driver.feedback_missing_ids_by_side[1] = {2, 3};
    articore::SafetyRuntime runtime(
        config(), api(), reinterpret_cast<void*>(0x100), g_left_controller,
        g_right_controller, motors);
    const auto identities = motor_identities(motors);
    runtime.configure_motor_identities(
        identities.data(), static_cast<uint32_t>(identities.size()));
    require_throws([&] { runtime.connect(); }, "initial feedback transaction",
                   "zero-feedback connect must fail");
    const auto report = runtime.last_connect_report();
    require(report.error_code == ARTICORE_CONNECT_FEEDBACK_TIMEOUT &&
                report.received_count == 0 && report.missing_count == 3,
            "zero valid transaction feedback has a stable timeout code");
  }
  {
    FakeDriver driver;
    g_driver = &driver;
    auto motors = descriptors(driver);
    driver.feedback_uses_per_side_results = true;
    driver.feedback_code_by_side[0] = 2;
    driver.feedback_received_by_side[0] = 0;
    driver.feedback_missing_ids_by_side[0] = {1};
    articore::SafetyRuntime runtime(
        config(), api(), reinterpret_cast<void*>(0x100), g_left_controller,
        g_right_controller, motors);
    const auto identities = motor_identities(motors);
    runtime.configure_motor_identities(
        identities.data(), static_cast<uint32_t>(identities.size()));
    require_throws([&] { runtime.connect(); }, "initial feedback transaction",
                   "transport-error connect must fail");
    require(runtime.last_connect_report().error_code ==
                ARTICORE_CONNECT_TRANSPORT,
            "transport feedback failure has a stable transport code");
  }
}

void test_transient_arm_send_failures_preserve_active_command() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  cfg.feedback_failure_threshold = 3;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  ArticorePosVelCommand commands[] = {
      {motors[0].motor, 0.25f, 1.0f},
      {motors[1].motor, -0.25f, 1.0f},
  };
  runtime.submit_pos_vel_ex(commands, 2,
                            ARTICORE_COMMAND_HOLD_UNTIL_REPLACED);
  require(wait_for([&] { return runtime.health().state == ARTICORE_RUNNING; }),
          "initial command reaches RUNNING before send jitter injection");
  uint32_t successful_sends = 0;
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    successful_sends = driver.pv_sends;
    driver.group_failures_remaining = 2;
  }
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.group_send_failures == 2;
          }),
          "two consecutive product frame sends are rejected");
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.pv_sends > successful_sends;
          }),
          "native worker retries and resumes successful product sends");
  require(runtime.health().state == ARTICORE_RUNNING,
          "sub-threshold send jitter does not stop or fault the Runtime");
  runtime.disable();
}

void test_pv_watchdog_safe_hold_and_fault() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  articore::SafetyRuntime runtime(
      config(), api(), reinterpret_cast<void*>(0x100), g_left_controller,
      g_right_controller, motors, enable_all, enable_motor, false, {},
      maintenance_api());
  const auto controls = joint_configs(motors);
  runtime.configure_joints(controls.data(),
                           static_cast<uint32_t>(controls.size()));
  runtime.connect();
  uint32_t feedback_requests_after_connect = 0;
  require(runtime.health().state == ARTICORE_READY, "connect enters READY");
  runtime.enable(ARTICORE_MODE_PV);
  require(runtime.health().state == ARTICORE_ENABLED, "enable enters ENABLED");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    feedback_requests_after_connect = driver.feedback_requests;
  }

  ArticorePosVelCommand commands[] = {
      {motors[0].motor, 0.5f, 2.0f},
      {motors[1].motor, -0.5f, 2.0f},
  };
  runtime.submit_pos_vel(commands, 2);
  require(wait_for([&] { return runtime.health().state == ARTICORE_RUNNING; }),
          "first command enters RUNNING on the next control cycle");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[0].motor].position = 0.2f;
    driver.motors[motors[1].motor].position = 0.8f;
  }
  require(wait_for([&] { return runtime.health().state == ARTICORE_SAFE_STOP; }),
          "command timeout enters recoverable SAFE_STOP");
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.last_pv.size() == 2 &&
                   std::abs(driver.last_pv[0].target_position - 0.2f) < 1e-6f &&
                   std::abs(driver.last_pv[1].target_position - 0.8f) < 1e-6f;
          }),
          "safe hold sends stored PV target");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.last_pv.size() == 2 &&
                std::abs(driver.last_pv[0].target_position - 0.2f) < 1e-6f &&
                std::abs(driver.last_pv[1].target_position - 0.8f) < 1e-6f &&
                std::abs(driver.last_pv[0].velocity_limit - 0.15f) < 1e-6f,
            "PV safe hold captures actual positions and limits velocity");
    require(driver.feedback_requests == feedback_requests_after_connect,
            "safe-hold entry reads cache without a blocking feedback request");
    driver.fail_group = true;
  }
  require(wait_for([&] {
            const auto health = runtime.health();
            return health.state == ARTICORE_FAULT &&
                   health.safe_holding == 1 &&
                   health.disable_confirmed == 0;
          }),
          "failed safe hold latches FAULT without automatic torque-off");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.disable_calls[0] == 0 && driver.disable_calls[1] == 0,
            "ordinary FAULT does not disable either controller");
    driver.fail_group = false;
  }
  runtime.disable();
  driver.emulate_arm_feedback = true;
  runtime.recover();
  require(runtime.health().state == ARTICORE_READY,
          "recover returns only to READY");
}

void test_mit_hold_removes_motion_and_feedforward() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  articore::SafetyRuntime runtime(config(), api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);
  ArticoreMitCommand commands[] = {
      {motors[0].motor, 0.4f, 2.0f, 20.0f, 3.0f, 4.0f},
      {motors[1].motor, -0.4f, -2.0f, 20.0f, 3.0f, -4.0f},
  };
  runtime.submit_mit(commands, 2);
  require(wait_for([&] { return runtime.health().state == ARTICORE_RUNNING; }),
          "MIT command enters RUNNING on the next control cycle");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[0].motor].position = -0.1f;
    driver.motors[motors[1].motor].position = 0.7f;
  }
  require(wait_for([&] { return runtime.health().state == ARTICORE_SAFE_STOP; }),
          "MIT watchdog enters recoverable SAFE_STOP");
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.last_arm_mit.size() == 2 &&
                   std::abs(driver.last_arm_mit[0].target_position + 0.1f) < 1e-6f &&
                   std::abs(driver.last_arm_mit[1].target_position - 0.7f) < 1e-6f;
          }),
          "MIT safe hold is transmitted");
  std::lock_guard<std::mutex> lock(driver.mutex);
  require(driver.last_arm_mit.size() == 2, "MIT arm hold captured");
  const float expected_positions[] = {-0.1f, 0.7f};
  for (std::size_t index = 0; index < driver.last_arm_mit.size(); ++index) {
    const auto& command = driver.last_arm_mit[index];
    require(std::abs(command.target_position - expected_positions[index]) < 1e-6f &&
                command.target_velocity == 0.0f &&
                command.feedforward_torque == 0.0f &&
                command.stiffness == 5.0f && command.damping == 0.5f,
            "MIT hold captures actual position, zeros motion/torque, and uses safe gains");
  }
}

void test_safe_hold_rejects_stale_current_position() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  cfg.feedback_check_hz = 1;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  uint32_t feedback_stats_baseline = 0;
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    feedback_stats_baseline = driver.feedback_stats_calls;
  }
  ArticorePosVelCommand commands[] = {
      {motors[0].motor, 0.5f, 2.0f},
      {motors[1].motor, -0.5f, 2.0f},
  };
  runtime.submit_pos_vel(commands, 2);
  require(wait_for([&] { return runtime.health().state == ARTICORE_RUNNING; }),
          "first PV target is transmitted before failure injection");
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.feedback_stats_calls >=
                   feedback_stats_baseline + motors.size();
          }),
          "initial running feedback check completes before failure injection");
  uint32_t sends_before_failure = 0;
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    sends_before_failure = driver.pv_sends;
    driver.motors[motors[0].motor].age_ns = 300'000'000ULL;
    driver.fail_group = true;
  }
  require(wait_for([&] { return runtime.health().state == ARTICORE_FAULT; }),
          "asynchronous send failure reaches FAULT");
  const auto health = runtime.health();
  require(health.state == ARTICORE_FAULT,
          "stale current-position feedback faults instead of holding an old target");
  require(std::string(health.fault_reason).find("safe hold failed") !=
              std::string::npos,
          "confirmed protective-hold send failure remains diagnosable");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.pv_sends == sends_before_failure,
            "stale feedback never sends a prior user target as safe hold");
  }
}

void test_gripper_hold_retreats_once_on_overload() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  // Keep this assertion independent of slow CI scheduling: the test verifies
  // that the first retreat target is held before the configured retry window.
  motors[2].retreat_retry_ms = 1000;
  articore::SafetyRuntime runtime(config(), api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);
  ArticoreMitCommand arm_commands[] = {
      {motors[0].motor, 0.4f, 0.0f, 20.0f, 3.0f, 0.0f},
      {motors[1].motor, -0.4f, 0.0f, 20.0f, 3.0f, 0.0f},
  };
  runtime.submit_mit(arm_commands, 2);
  ArticoreMitCommand gripper_command{
      motors[2].motor, 1.8f, 0.5f, 8.0f, 2.0f, 1.0f};
  runtime.submit_gripper_mit(&gripper_command, 1);
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[2].motor].position = 1.5f;
    driver.motors[motors[2].motor].torque = 2.0f;
  }
  require(wait_for([&] {
            const auto state = runtime.health().state;
            return state == ARTICORE_SAFE_HOLD || state == ARTICORE_SAFE_STOP;
          }),
          "gripper overload test reaches a protective hold state");
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.last_mit.size() == 1 &&
                   std::abs(driver.last_mit[0].target_position - 1.4f) < 1e-6f;
          }),
          "overloaded gripper retreats from actual position exactly once");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[2].motor].position = 1.3f;
  }
  std::this_thread::sleep_for(25ms);
  std::lock_guard<std::mutex> lock(driver.mutex);
  require(driver.last_mit.size() == 1 &&
              std::abs(driver.last_mit[0].target_position - 1.4f) < 1e-6f &&
              driver.last_mit[0].target_velocity == 0.0f &&
              driver.last_mit[0].feedforward_torque == 0.0f &&
              driver.last_mit[0].stiffness == 2.0f &&
              driver.last_mit[0].damping == 0.5f,
          "gripper hold keeps one retreat target with low gains and zero torque");
}

void test_gripper_stall_switches_to_contact_hold_target() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  motors[2].contact_torque = 0.8f;
  motors[2].motion_window_ms = 20;
  motors[2].stall_movement = 0.01f;
  motors[2].min_position_error = 0.05f;
  motors[2].contact_hold_ms = 10;
  motors[2].hold_offset = 0.08f;
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  cfg.reserved_gripper_control_rate = 500;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);
  ArticoreMitCommand arm_commands[] = {
      {motors[0].motor, 0.4f, 0.0f, 20.0f, 3.0f, 0.0f},
      {motors[1].motor, -0.4f, 0.0f, 20.0f, 3.0f, 0.0f},
  };
  runtime.submit_mit(arm_commands, 2);
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[2].motor].position = 1.5f;
    driver.motors[motors[2].motor].torque = 1.0f;
  }
  ArticoreGripperTarget target{motors[2].motor, 0.0f};
  runtime.set_gripper_openings(&target, 1);
  const bool contact_reached = wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.last_mit.size() == 1 &&
                   std::abs(driver.last_mit[0].target_position - 1.58f) < 1e-5f;
          });
  require(contact_reached,
          "stalled gripper holds a low-gain contact offset from actual position");
  const auto health = runtime.health();
  require(health.gripper_count == 1 &&
              health.grippers[0].contact_detected == 1 &&
              health.grippers[0].stalled == 1 &&
              (health.grippers[0].control_state == ARTICORE_GRIPPER_CONTACT ||
               health.grippers[0].control_state == ARTICORE_GRIPPER_HOLDING),
          "structured gripper health exposes contact and stall state");

  ArticoreGripperTarget open_target{motors[2].motor, 1000.0f};
  runtime.set_gripper_openings(&open_target, 1);
  require(wait_for([&] {
            const auto opening_health = runtime.health();
            return opening_health.grippers[0].control_state ==
                       ARTICORE_GRIPPER_MOVING &&
                   opening_health.grippers[0].contact_detected == 0 &&
                   opening_health.grippers[0].stalled == 0;
          }),
          "opening immediately exits contact and low-gain hold state");
}

void test_gripper_torque_spike_does_not_trigger_contact() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  motors[2].contact_torque = 0.8f;
  motors[2].motion_window_ms = 20;
  motors[2].stall_movement = 0.01f;
  motors[2].min_position_error = 0.05f;
  // A short injected spike must remain well below the sustained-contact
  // window even when a loaded CI runner overshoots the sleeps below.
  motors[2].contact_hold_ms = 500;
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  cfg.reserved_gripper_control_rate = 500;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);
  ArticoreMitCommand arm_commands[] = {
      {motors[0].motor, 0.0f, 0.0f, 5.0f, 1.0f, 0.0f},
      {motors[1].motor, 0.0f, 0.0f, 5.0f, 1.0f, 0.0f},
  };
  runtime.submit_mit(arm_commands, 2);
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[2].motor].position = 1.5f;
    driver.motors[motors[2].motor].torque = 1.0f;
  }
  ArticoreGripperTarget target{motors[2].motor, 0.0f};
  runtime.set_gripper_openings(&target, 1);
  std::this_thread::sleep_for(30ms);
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[2].motor].torque = 0.0f;
  }
  std::this_thread::sleep_for(60ms);
  const auto health = runtime.health();
  require(health.grippers[0].control_state == ARTICORE_GRIPPER_MOVING &&
              health.grippers[0].contact_detected == 0 &&
              health.grippers[0].stalled == 0,
          "short torque spike does not satisfy sustained contact detection");
}

void test_gripper_command_profiles_and_bidirectional_ramp() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  cfg.reserved_control_rate = 500;
  articore::SafetyRuntime runtime(
      cfg, api(), reinterpret_cast<void*>(0x100),
      g_left_controller, g_right_controller, motors);
  const auto profiles = gripper_force_profiles(motors);
  runtime.configure_gripper_force_profiles(
      profiles.data(), static_cast<uint32_t>(profiles.size()));
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);
  ArticoreMitCommand arm_commands[] = {
      {motors[0].motor, 0.0f, 0.0f, 5.0f, 1.0f, 0.0f},
      {motors[1].motor, 0.0f, 0.0f, 5.0f, 1.0f, 0.0f},
  };
  runtime.submit_mit_ex(
      arm_commands, 2, ARTICORE_COMMAND_HOLD_UNTIL_REPLACED);

  ArticoreGripperCommand opening{
      sizeof(ArticoreGripperCommand), motors[2].motor,
      1000.0f, 100.0f, ARTICORE_GRIPPER_FORCE_LOW};
  runtime.set_gripper_commands(&opening, 1);
  ArticoreMitCommand first_open{};
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            for (const auto& batch : driver.mit_history) {
              if (batch.size() == 1 && batch[0].motor == motors[2].motor &&
                  batch[0].target_position < 2.0f) {
                first_open = batch[0];
                return true;
              }
            }
            return false;
          }),
          "opening begins with a bounded position ramp");
  require(first_open.target_position > 1.9f &&
              first_open.stiffness == 3.0f &&
              first_open.damping == 0.3f,
          "opening does not jump to its endpoint and applies LOW profile");

  std::size_t switch_baseline = 0;
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    switch_baseline = driver.mit_history.size();
  }
  opening.speed = 1000.0f;
  opening.force_level = ARTICORE_GRIPPER_FORCE_HIGH;
  runtime.set_gripper_commands(&opening, 1);
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            for (std::size_t i = switch_baseline;
                 i < driver.mit_history.size(); ++i) {
              const auto& batch = driver.mit_history[i];
              if (batch.size() == 1 && batch[0].motor == motors[2].motor &&
                  batch[0].stiffness == 6.0f && batch[0].damping == 0.8f &&
                  batch[0].target_position < first_open.target_position) {
                return true;
              }
            }
            return false;
          }),
          "speed and force profile switch together on a native control tick");

  ArticoreMitCommand before_close{};
  std::size_t close_baseline = 0;
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    close_baseline = driver.mit_history.size();
    for (auto it = driver.mit_history.rbegin();
         it != driver.mit_history.rend(); ++it) {
      if (it->size() == 1 && (*it)[0].motor == motors[2].motor) {
        before_close = (*it)[0];
        break;
      }
    }
  }
  ArticoreGripperCommand closing{
      sizeof(ArticoreGripperCommand), motors[2].motor,
      0.0f, 100.0f, ARTICORE_GRIPPER_FORCE_NORMAL};
  runtime.set_gripper_commands(&closing, 1);
  ArticoreMitCommand first_close{};
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            for (std::size_t i = close_baseline;
                 i < driver.mit_history.size(); ++i) {
              const auto& batch = driver.mit_history[i];
              if (batch.size() == 1 && batch[0].motor == motors[2].motor &&
                  batch[0].target_position > before_close.target_position) {
                first_close = batch[0];
                return true;
              }
            }
            return false;
          }),
          "closing reverses through the same bounded ramp");
  require(first_close.target_position < 2.0f &&
              first_close.stiffness == 4.0f &&
              first_close.damping == 0.5f,
          "closing also avoids an endpoint jump and applies NORMAL profile");
}

void test_gripper_only_command_satisfies_enable_grace_without_masking_arm_watchdog() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.enable_grace_ms = 60;
  cfg.command_timeout_ms = 30;
  articore::SafetyRuntime runtime(
      cfg, api(), reinterpret_cast<void*>(0x100),
      g_left_controller, g_right_controller, motors);
  const auto profiles = gripper_force_profiles(motors);
  runtime.configure_gripper_force_profiles(
      profiles.data(), static_cast<uint32_t>(profiles.size()));
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);

  ArticoreGripperCommand gripper{
      sizeof(ArticoreGripperCommand), motors[2].motor,
      1000.0f, 500.0f, ARTICORE_GRIPPER_FORCE_NORMAL};
  runtime.set_gripper_commands(&gripper, 1);
  require(wait_for([&] { return runtime.health().state == ARTICORE_RUNNING; }),
          "a successfully sent gripper-only batch enters RUNNING");
  std::this_thread::sleep_for(100ms);
  require(runtime.health().state == ARTICORE_RUNNING,
          "a persistent gripper command does not expire enable grace");

  ArticoreMitCommand streaming[] = {
      {motors[0].motor, 0.0f, 0.0f, 5.0f, 1.0f, 0.0f},
      {motors[1].motor, 0.0f, 0.0f, 5.0f, 1.0f, 0.0f},
  };
  runtime.submit_mit_ex(streaming, 2, ARTICORE_COMMAND_STREAMING);
  require(wait_for([&] {
            return runtime.health().state == ARTICORE_SAFE_STOP;
          }),
          "persistent gripper retransmission cannot mask an arm streaming timeout");
}

void test_gripper_force_profiles_are_product_configuration() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  articore::SafetyRuntime runtime(
      config(), api(), reinterpret_cast<void*>(0x100),
      g_left_controller, g_right_controller, motors);
  auto profiles = gripper_force_profiles(motors);
  profiles.pop_back();
  bool incomplete_rejected = false;
  try {
    runtime.configure_gripper_force_profiles(
        profiles.data(), static_cast<uint32_t>(profiles.size()));
  } catch (const std::invalid_argument&) {
    incomplete_rejected = true;
  }
  require(incomplete_rejected,
          "product calibration requires every stable force level");

  profiles = gripper_force_profiles(motors);
  runtime.configure_gripper_force_profiles(
      profiles.data(), static_cast<uint32_t>(profiles.size()));
  runtime.connect();
  bool immutable_after_connect = false;
  try {
    runtime.configure_gripper_force_profiles(
        profiles.data(), static_cast<uint32_t>(profiles.size()));
  } catch (const std::runtime_error&) {
    immutable_after_connect = true;
  }
  require(immutable_after_connect,
          "force calibration cannot be changed by a runtime motion client");
}

void test_builtin_yunyi_gripper_profile_owns_product_calibration() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.gripper_fault_action = 0;
  auto& source_gripper = motors[2];
  source_gripper.safe_kp = 0.0f;
  source_gripper.safe_kd = 0.0f;
  source_gripper.overload_torque = 0.0f;
  source_gripper.retreat_distance = 0.0f;
  source_gripper.contact_torque = 0.0f;
  source_gripper.motion_window_ms = 0;
  source_gripper.stall_movement = 0.0f;
  source_gripper.min_position_error = 0.0f;
  source_gripper.contact_hold_ms = 0;
  source_gripper.overload_hold_ms = 0;
  source_gripper.hold_offset = 0.0f;
  source_gripper.retreat_retry_ms = 0;
  source_gripper.open_position = 0.0f;
  source_gripper.closed_position = 0.0f;
  source_gripper.normal_kp = 0.0f;
  source_gripper.normal_kd = 0.0f;
  source_gripper.close_speed = 0.0f;
  source_gripper.max_step_interval_ms = 0;
  source_gripper.closing_direction = 0.0f;
  source_gripper.lower_position = 0.0f;
  source_gripper.upper_position = 0.0f;

  articore::SafetyRuntime runtime(
      cfg, api(), reinterpret_cast<void*>(0x100), g_left_controller,
      g_right_controller, motors, nullptr, nullptr, true);
  require_throws([&] { runtime.connect(); }, "profile is required",
                 "production runtime rejects an unbound active gripper");

  auto unknown = gripper_product_binding(
      source_gripper.motor, "unknown_gripper_profile");
  require_throws(
      [&] { runtime.configure_gripper_products(&unknown, 1); },
      "unknown built-in gripper profile_id",
      "unknown built-in profile is rejected before connect");

  auto binding = gripper_product_binding(source_gripper.motor);
  runtime.configure_gripper_products(&binding, 1);
  runtime.connect();
  require_throws(
      [&] { runtime.configure_gripper_products(&binding, 1); },
      "fixed after connect",
      "product profile cannot change after connect");
  runtime.enable(ARTICORE_MODE_PV);

  ArticoreGripperCommand command{
      sizeof(ArticoreGripperCommand), source_gripper.motor,
      1000.0f, 1.0f, ARTICORE_GRIPPER_FORCE_LEVEL_1};
  runtime.set_gripper_commands(&command, 1);
  float slow_position = 0.0f;
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            if (driver.last_mit.size() != 1 ||
                driver.last_mit[0].motor != source_gripper.motor ||
                driver.last_mit[0].target_position <= 2.0f) {
              return false;
            }
            slow_position = driver.last_mit[0].target_position;
            return slow_position <= 2.00025f &&
                   std::abs(driver.last_mit[0].stiffness - 3.0f) < 1e-6f &&
                   std::abs(driver.last_mit[0].damping - 0.3f) < 1e-6f;
          }),
          "speed=1 maps to 0.01 rad/s and force level 1 calibration");
  require(runtime.gripper_force_level(source_gripper.side) == 1,
          "Runtime state reports the executed force level 1");

  command.speed = 1000.0f;
  command.force_level = ARTICORE_GRIPPER_FORCE_LEVEL_5;
  runtime.set_gripper_commands(&command, 1);
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.last_mit.size() == 1 &&
                   driver.last_mit[0].motor == source_gripper.motor &&
                   std::abs(driver.last_mit[0].stiffness - 4.0f) < 1e-6f &&
                   std::abs(driver.last_mit[0].damping - 0.5f) < 1e-6f;
          }),
          "force level 5 selects the middle product calibration directly");
  require(runtime.gripper_force_level(source_gripper.side) == 5,
          "Runtime state reports the executed force level 5");

  command.force_level = ARTICORE_GRIPPER_FORCE_LEVEL_10;
  runtime.set_gripper_commands(&command, 1);
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.last_mit.size() == 1 &&
                   driver.last_mit[0].motor == source_gripper.motor &&
                   driver.last_mit[0].target_position > slow_position + 0.001f &&
                   driver.last_mit[0].target_position <= 2.64f &&
                   std::abs(driver.last_mit[0].stiffness - 6.0f) < 1e-6f &&
                   std::abs(driver.last_mit[0].damping - 0.8f) < 1e-6f;
          }),
          "speed=1000 maps to 10 rad/s and force level 10 calibration");
  require(runtime.gripper_force_level(source_gripper.side) == 10,
          "Runtime state reports the executed force level 10");

  runtime.estop();
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.motors[motors[0].motor].status == 0 &&
                driver.motors[motors[1].motor].status == 0 &&
                driver.motors[source_gripper.motor].status == 0,
            "emergency stop overrides the gripper hold-on-fault policy");
  }
}

void test_builtin_gripper_binding_is_complete_and_optional() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto left_gripper = motors[2];
  left_gripper.motor = reinterpret_cast<void*>(0x204);
  left_gripper.side = 0;
  std::memset(left_gripper.name, 0, sizeof(left_gripper.name));
  std::strncpy(left_gripper.name, "left/gripper",
               sizeof(left_gripper.name) - 1);
  driver.motors[left_gripper.motor] = FakeMotor{1, 1.0f, 0.0f, 0.0f, 0, true};
  motors.push_back(left_gripper);
  auto cfg = config();
  cfg.gripper_fault_action = 0;
  articore::SafetyRuntime dual(
      cfg, api(), reinterpret_cast<void*>(0x100), g_left_controller,
      g_right_controller, motors, nullptr, nullptr, true);
  ArticoreGripperProductBinding bindings[] = {
      gripper_product_binding(left_gripper.motor),
      gripper_product_binding(motors[2].motor),
  };
  require_throws(
      [&] { dual.configure_gripper_products(bindings, 1); },
      "cover every active gripper",
      "partial dual-gripper binding is rejected atomically");
  dual.configure_gripper_products(bindings, 2);
  dual.connect();
  require(dual.health().state == ARTICORE_READY,
          "left and right grippers share one built-in profile");

  auto arm_only = descriptors(driver);
  arm_only.resize(2);
  articore::SafetyRuntime no_gripper(
      cfg, api(), reinterpret_cast<void*>(0x101), g_left_controller,
      g_right_controller, arm_only, nullptr, nullptr, true);
  no_gripper.configure_gripper_products(nullptr, 0);
  no_gripper.connect();
  require(no_gripper.health().state == ARTICORE_READY,
          "a runtime without installed grippers needs no product binding");
}

void test_legacy_three_level_gripper_profiles_expand_to_ten_levels() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  articore::SafetyRuntime runtime(
      config(), api(), reinterpret_cast<void*>(0x100),
      g_left_controller, g_right_controller, motors);
  const auto full = gripper_force_profiles(motors);
  auto low = full.front();
  auto normal = full[4];
  auto high = full.back();
  low.force_level = 1;
  normal.force_level = 2;
  high.force_level = 3;
  const ArticoreGripperForceProfile legacy[] = {low, normal, high};
  runtime.configure_gripper_force_profiles(legacy, 3);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);

  ArticoreGripperCommand command{
      sizeof(ArticoreGripperCommand), motors[2].motor,
      1000.0f, 500.0f, 2};
  runtime.set_gripper_commands(&command, 1);
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return !driver.last_mit.empty() &&
                   driver.last_mit[0].motor == motors[2].motor &&
                   driver.last_mit[0].stiffness == 4.0f;
          }),
          "legacy NORMAL command value 2 retains the level-5 calibration");
  command.force_level = ARTICORE_GRIPPER_FORCE_LEVEL_7;
  runtime.set_gripper_commands(&command, 1);
  require(wait_for([&] { return runtime.health().state == ARTICORE_RUNNING; }),
          "legacy three-level profiles expose interpolated force levels 1..10");
}

void test_estop_disables_complete_product_and_is_idempotent() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  driver.emulate_arm_feedback = true;
  auto cfg = config();
  cfg.gripper_fault_action = ARTICORE_GRIPPER_FAULT_HOLD;
  articore::SafetyRuntime runtime(
      cfg, api(), reinterpret_cast<void*>(0x100), g_left_controller,
      g_right_controller, motors, enable_all, enable_motor, false, {},
      maintenance_api());
  const auto controls = joint_configs(motors);
  runtime.configure_joints(controls.data(),
                           static_cast<uint32_t>(controls.size()));
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  runtime.estop();

  const auto fault = runtime.health();
  require(fault.state == ARTICORE_FAULT && fault.disable_confirmed == 1 &&
              fault.safe_holding == 0 &&
              std::string(fault.fault_reason) == "emergency stop requested",
          "estop latches the standard reason after disabling the product");
  uint32_t left_disable_calls = 0;
  uint32_t right_disable_calls = 0;
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    left_disable_calls = driver.disable_calls[0];
    right_disable_calls = driver.disable_calls[1];
    require(driver.motors[motors[0].motor].status == 0 &&
                driver.motors[motors[1].motor].status == 0 &&
                driver.motors[motors[2].motor].status == 0,
            "estop disables arm joints and grippers regardless of hold policy");
  }
  runtime.estop();
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.disable_calls[0] == left_disable_calls &&
                driver.disable_calls[1] == right_disable_calls,
            "repeated estop is an idempotent no-op after confirmed disable");
  }
  require(runtime.clear_faults() == ARTICORE_OPERATION_INVALID_STATE &&
              runtime.health().state == ARTICORE_FAULT,
          "motor fault clear cannot release estop");
  runtime.recover();
  require(runtime.health().state == ARTICORE_READY &&
              runtime.health().disable_confirmed == 1,
          "recover is the only operation that clears the estop latch");
  runtime.disconnect();
  runtime.disconnect();
  require(runtime.health().state == ARTICORE_DISCONNECTED,
          "terminal disconnect remains idempotent after estop recovery");
}

void test_estop_can_disable_gripper_by_product_policy() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.gripper_fault_action = ARTICORE_GRIPPER_FAULT_DISABLE;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  runtime.estop();

  const auto fault = runtime.health();
  require(fault.state == ARTICORE_FAULT && fault.disable_confirmed == 1 &&
              fault.safe_holding == 0,
          "disable-policy estop confirms full torque-off");
  std::lock_guard<std::mutex> lock(driver.mutex);
  require(driver.motors[motors[0].motor].status == 0 &&
              driver.motors[motors[1].motor].status == 0 &&
              driver.motors[motors[2].motor].status == 0,
          "disable-policy estop disables arms and gripper");
}

void test_missing_feedback_degrades_then_safe_stops_and_resynchronizes() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  driver.emulate_arm_feedback = true;
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  articore::SafetyRuntime runtime(
      cfg, api(), reinterpret_cast<void*>(0x100), g_left_controller,
      g_right_controller, motors, enable_all, enable_motor, false, {},
      maintenance_api());
  const auto controls = joint_configs(motors);
  runtime.configure_joints(controls.data(),
                           static_cast<uint32_t>(controls.size()));
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  ArticorePosVelCommand commands[] = {
      {motors[0].motor, 0.0f, 1.0f}, {motors[1].motor, 0.0f, 1.0f}};
  runtime.submit_pos_vel_ex(
      commands, 2, ARTICORE_COMMAND_HOLD_UNTIL_REPLACED);
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[0].motor].has_feedback = false;
  }
  require(wait_for([&] {
            return runtime.health().state == ARTICORE_DEGRADED;
          }),
          "sustained missing feedback enters DEGRADED");
  {
    const auto health = runtime.health_v2();
    require(health.degraded == 1 && health.safe_stopped == 0 &&
                health.requires_resynchronization == 1 &&
                std::abs(health.command_scale - 0.25f) < 1e-6f &&
                health.health.fault_reason[0] == '\0' &&
                health.safety_reason[0] != '\0',
            "DEGRADED is derated and is not reported as a motor fault");
  }
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return std::any_of(
                driver.pv_history.rbegin(), driver.pv_history.rend(),
                [](const auto& batch) {
                  return batch.size() == 2 &&
                         std::abs(batch[0].velocity_limit - 0.25f) < 1e-6f &&
                         std::abs(batch[1].velocity_limit - 0.25f) < 1e-6f;
                });
          }),
          "DEGRADED scales PV velocity limits to 25 percent");
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return std::any_of(
                driver.pv_history.begin(), driver.pv_history.end(),
                [&](const auto& batch) {
                  return batch.size() == 2;
                });
          }),
          "the complete arm command continues while feedback is missing");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.disable_calls[0] == 0 && driver.disable_calls[1] == 0,
            "missing feedback does not automatically disable either side");
  }
  require(wait_for([&] {
            return runtime.health().state == ARTICORE_SAFE_STOP;
          }),
          "continued missing feedback enters SAFE_STOP");
  const auto stopped = runtime.health_v2();
  require(stopped.health.safe_holding == 1 &&
              stopped.health.disable_confirmed == 0 &&
              stopped.health.left_transport.healthy == 0 &&
              stopped.health.fault_reason[0] == '\0' &&
              stopped.safe_stopped == 1 && stopped.command_scale == 0.0f,
          "SAFE_STOP holds without disabling or reporting motor FAULT");
  require(runtime.motor_presence("left/joint1") == ARTICORE_PRESENT,
          "feedback delay never relabels the physical motor as faulted");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[0].motor].has_feedback = true;
  }
  require(wait_for([&] {
            return runtime.health().consecutive_feedback_failures == 0;
          }),
          "feedback recovery is observed while remaining stopped");
  require(runtime.health().state == ARTICORE_SAFE_STOP,
          "feedback recovery never resumes an old trajectory automatically");
  runtime.recover();
  require(runtime.health().state == ARTICORE_READY &&
              runtime.health().disable_confirmed == 1,
          "explicit recover returns to calibrated zero and finishes disabled");
}

void test_single_gripper_feedback_miss_reuses_current_output() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  cfg.feedback_check_hz = 1;
  cfg.reserved_gripper_control_rate = 100;
  // The assertion restores feedback after observing a retransmitted frame,
  // not after an exact number of scheduler ticks. Keep the threshold well
  // above host scheduling jitter so this remains a one-gap behavior test on
  // fast macOS and slower Linux/Windows runners alike.
  cfg.feedback_failure_threshold = 100;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);
  ArticoreMitCommand arm_commands[] = {
      {motors[0].motor, 0.2f, 0.0f, 5.0f, 0.5f, 0.0f},
      {motors[1].motor, 0.8f, 0.0f, 5.0f, 0.5f, 0.0f},
  };
  runtime.submit_mit_ex(
      arm_commands, 2, ARTICORE_COMMAND_HOLD_UNTIL_REPLACED);
  ArticoreGripperTarget gripper{motors[2].motor, 625.0f};
  runtime.set_gripper_openings(&gripper, 1);

  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return std::any_of(
                driver.mit_history.begin(), driver.mit_history.end(),
                [&](const auto& batch) {
                  return batch.size() == 1 &&
                         batch[0].motor == motors[2].motor;
                });
          }),
          "gripper establishes a successful current output before the miss");
  require(wait_for([&] { return runtime.health().state == ARTICORE_RUNNING; }),
          "arm command is transmitted before gripper feedback is removed");

  std::size_t history_before = 0;
  ArticoreMitCommand output_before{};
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    history_before = driver.mit_history.size();
    for (auto it = driver.mit_history.rbegin();
         it != driver.mit_history.rend(); ++it) {
      if (it->size() == 1 && (*it)[0].motor == motors[2].motor) {
        output_before = (*it)[0];
        break;
      }
    }
    driver.motors[motors[2].motor].has_feedback = false;
  }
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            for (std::size_t i = history_before; i < driver.mit_history.size(); ++i) {
              const auto& batch = driver.mit_history[i];
              if (batch.size() == 1 && batch[0].motor == motors[2].motor &&
                  std::abs(batch[0].target_position -
                           output_before.target_position) < 1e-6f &&
                  batch[0].target_velocity == output_before.target_velocity &&
                  batch[0].stiffness == output_before.stiffness &&
                  batch[0].damping == output_before.damping &&
                  batch[0].feedforward_torque ==
                      output_before.feedforward_torque) {
                return true;
              }
            }
            return false;
          }, 100ms),
          "one missing gripper feedback sample resends the current output");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[2].motor].has_feedback = true;
  }
  require(runtime.health().state == ARTICORE_RUNNING,
          "one gripper feedback miss only counts and does not stop motion");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.disable_calls[0] == 0 && driver.disable_calls[1] == 0,
            "one gripper feedback miss never triggers automatic disable");
  }
}

void test_feedback_measurements_do_not_reuse_command_limits() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  cfg.feedback_check_hz = 200;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  const auto configured = joint_configs(motors);
  runtime.configure_joints(configured.data(),
                           static_cast<uint32_t>(configured.size()));
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  ArticorePosVelCommand valid[] = {
      {motors[0].motor, 0.25f, 1.0f}, {motors[1].motor, 0.75f, 1.0f}};
  runtime.submit_pos_vel(valid, 2);
  require(wait_for([&] { return runtime.health().state == ARTICORE_RUNNING; }),
          "valid command reaches RUNNING before feedback excursion");

  uint32_t feedback_checks = 0;
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    feedback_checks = driver.feedback_stats_calls;
    driver.motors[motors[0].motor].position = 2.001f;
    driver.motors[motors[0].motor].velocity = 5.1f;
    driver.motors[motors[0].motor].torque = 10.1f;
  }
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.feedback_stats_calls >= feedback_checks + motors.size() * 3;
          }),
          "multiple feedback-health cycles observe out-of-command-limit data");
  require(runtime.health().state == ARTICORE_RUNNING,
          "finite fresh feedback outside command limits does not fault");

  bool command_rejected = false;
  ArticorePosVelCommand invalid[] = {
      {motors[0].motor, 2.001f, 1.0f}, {motors[1].motor, 0.75f, 1.0f}};
  try {
    runtime.submit_pos_vel(invalid, 2);
  } catch (const std::invalid_argument&) {
    command_rejected = true;
  }
  require(command_rejected,
          "the same out-of-limit value is still rejected as a user command");
}

void test_feedback_seed_ignores_command_limits() {
  FakeDriver driver;
  driver.emulate_arm_feedback = true;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[0].motor].position = 2.001f;
    driver.motors[motors[0].motor].velocity = 5.1f;
    driver.motors[motors[0].motor].torque = 10.1f;
  }
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  const auto configured = joint_configs(motors);
  runtime.configure_joints(configured.data(),
                           static_cast<uint32_t>(configured.size()));
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  require(runtime.health().state == ARTICORE_ENABLED,
          "feedback-seeded current-position hold does not apply command limits");
}

void test_feedback_fault_diagnostics_include_identity_value_and_threshold() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  cfg.feedback_check_hz = 200;
  cfg.feedback_failure_threshold = 1;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  ArticorePosVelCommand valid[] = {
      {motors[0].motor, 0.25f, 1.0f}, {motors[1].motor, 0.75f, 1.0f}};
  runtime.submit_pos_vel(valid, 2);
  require(wait_for([&] { return runtime.health().state == ARTICORE_RUNNING; }),
          "diagnostic test reaches RUNNING");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[0].motor].velocity =
        std::numeric_limits<float>::quiet_NaN();
  }
  require(wait_for([&] { return runtime.health().state == ARTICORE_FAULT; }),
          "non-finite feedback eventually faults");
  const std::string reason(runtime.health().fault_reason);
  require(reason.find("CH0/left/joint1") != std::string::npos &&
              reason.find("CAN ID 1") != std::string::npos &&
              reason.find("velocity=nan") != std::string::npos &&
              reason.find("threshold=all feedback values finite") !=
                  std::string::npos,
          "feedback fault identifies channel, motor, CAN ID, actual value, and threshold");
}

void test_single_stale_feedback_sample_keeps_persistent_position_control() {
  FakeDriver driver;
  driver.emulate_arm_feedback = true;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  cfg.feedback_check_hz = 20;
  cfg.feedback_failure_threshold = 3;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  const auto configured = joint_configs(motors);
  runtime.configure_joints(configured.data(),
                           static_cast<uint32_t>(configured.size()));
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);
  ArticoreJointMitTarget targets[] = {
      {sizeof(ArticoreJointMitTarget), motors[0].motor, 0.05f},
      {sizeof(ArticoreJointMitTarget), motors[1].motor, 0.95f}};
  runtime.set_joint_mit(targets, 2, 0.5f);
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[2].motor].age_ns = 201'000'000ULL;
  }
  require(wait_for([&] {
            return runtime.health().consecutive_feedback_failures == 1;
          }, 200ms),
          "one stale feedback-health sample is counted");
  require(runtime.health().state == ARTICORE_RUNNING,
          "one stale sample keeps persistent position control running");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[2].motor].age_ns = 0;
  }
  require(wait_for([&] {
            return runtime.health().state == ARTICORE_RUNNING &&
                   runtime.health().consecutive_feedback_failures == 0;
          }, 200ms),
          "fresh feedback clears the transient failure without interrupting control");
}

void test_ordinary_position_can_recover_from_feedback_outside_hard_limit() {
  for (const auto mode : {ARTICORE_MODE_MIT, ARTICORE_MODE_PV}) {
    FakeDriver driver;
    driver.emulate_arm_feedback = true;
    g_driver = &driver;
    auto motors = descriptors(driver);
    auto cfg = config();
    cfg.command_timeout_ms = 500;
    {
      std::lock_guard<std::mutex> lock(driver.mutex);
      driver.motors[motors[0].motor].position = 2.1f;
      driver.motors[motors[0].motor].velocity = 0.0f;
    }
    articore::SafetyRuntime runtime(
        cfg, api(), reinterpret_cast<void*>(0x100),
        g_left_controller, g_right_controller, motors);
    const auto configured = joint_configs(motors);
    runtime.configure_joints(configured.data(),
                             static_cast<uint32_t>(configured.size()));
    const auto layered = layered_joint_limits(motors);
    runtime.configure_joint_safety_limits(
        layered.data(), static_cast<uint32_t>(layered.size()));
    runtime.connect();
    runtime.enable(mode);

    if (mode == ARTICORE_MODE_MIT) {
      ArticoreJointMitTarget inward[] = {
          {sizeof(ArticoreJointMitTarget), motors[0].motor, 1.0f},
          {sizeof(ArticoreJointMitTarget), motors[1].motor, 0.0f},
      };
      runtime.set_joint_mit(inward, 2, 1.0f);
    } else {
      ArticoreJointPvTarget inward[] = {
          {sizeof(ArticoreJointPvTarget), motors[0].motor, 1.0f},
          {sizeof(ArticoreJointPvTarget), motors[1].motor, 0.0f},
      };
      runtime.set_joint_pv(inward, 2, 1.0f);
    }

    require(wait_for([&] {
              std::lock_guard<std::mutex> lock(driver.mutex);
              const auto position = driver.motors[motors[0].motor].position;
              return position < 2.1f && position > 1.5f;
            }, 100ms),
            "ordinary PV/MIT starts continuously from out-of-limit feedback");
    require(runtime.health().state == ARTICORE_RUNNING,
            "out-of-limit feedback initialization does not fault the runtime");
  }
}

void test_disable_does_not_stop_after_one_side_fails() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  articore::SafetyRuntime runtime(config(), api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.fail_left_disable = true;
  }
  bool failed = false;
  try {
    runtime.disable();
  } catch (const std::exception&) {
    failed = true;
  }
  require(failed && runtime.health().state == ARTICORE_FAULT,
          "failed disable locks FAULT");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.disable_calls[0] >= 1 && driver.disable_calls[1] >= 1,
            "right side is still disabled after left side failure");
  }
}

void test_disable_uses_structured_feedback_report() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  articore::SafetyRuntime runtime(config(), api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.feedback_code = 4;
    driver.feedback_expected = 2;
    driver.feedback_received = 1;
    driver.feedback_missing_ids = {15};
  }
  bool failed = false;
  try {
    runtime.disable();
  } catch (const std::runtime_error& error) {
    const std::string message(error.what());
    failed = message.find("code=4") != std::string::npos &&
             message.find("expected=2") != std::string::npos &&
             message.find("received=1") != std::string::npos &&
             message.find("missing motor IDs: 15") != std::string::npos;
  }
  require(failed && runtime.health().state == ARTICORE_FAULT,
          "disable consumes stable feedback code, counts, and missing IDs");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.feedback_requests >= 2 && driver.feedback_requests % 2 == 0,
            "structured feedback confirmation retries both active sides together");
  }
}

void test_disable_retries_one_full_feedback_deadline() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.disable_feedback_timeout_ms = 10;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  uint32_t requests_before = 0;
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    requests_before = driver.feedback_requests;
    driver.feedback_timeouts_remaining = 1;
    driver.feedback_timeout_consumes_deadline = true;
  }
  runtime.disable();
  require(runtime.health().state == ARTICORE_READY &&
              runtime.health().disable_confirmed == 1,
          "one full-deadline feedback timeout is retried after physical disable");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.feedback_requests >= requests_before + 3,
            "disable confirmation performs one bounded outer retry");
  }
}

void test_disable_barrier_and_targeted_motor_retry() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  articore::SafetyRuntime runtime(config(), api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    // During the first post-disable feedback batch, make only the right
    // gripper appear enabled. Use a relative call index because connect now
    // owns an initial two-channel feedback transaction.
    // The deterministic queue barrier itself first performs one request on
    // each active channel, so the first disable-state confirmation starts at
    // the third subsequent request.
    driver.feedback_enable_on_call[driver.feedback_requests + 3] =
        motors[2].motor;
  }
  runtime.disable();
  const auto report = runtime.last_disable_report();
  require(report.success == 1 && report.barrier_confirmed == 1 &&
              report.retry_count == 1 && report.missing_count == 0 &&
              report.disabled_count == 3,
          "deterministic disable reports barrier, directed retry, and confirmation");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.disable_calls[0] == 1 && driver.disable_calls[1] == 3,
            "only the one unconfirmed right gripper is disabled a second time");
    const auto barrier = std::find(driver.events.begin(), driver.events.end(),
                                   "barrier-pv");
    const auto first_disable = std::find_if(
        driver.events.begin(), driver.events.end(), [](const std::string& event) {
          return event.rfind("disable-", 0) == 0;
        });
    require(barrier != driver.events.end() && first_disable != driver.events.end() &&
                barrier < first_disable,
            "controller-group drain barrier precedes every disable frame");
    require(std::none_of(first_disable, driver.events.end(),
                         [](const std::string& event) {
                           return event == "send-pv" || event == "send-mit";
                         }),
            "no old Runtime motion frame is emitted after disable begins");
  }
}

void test_disable_waits_for_inflight_batch_and_rejects_new_commands() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  articore::SafetyRuntime runtime(config(), api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  ArticorePosVelCommand commands[] = {
      {motors[0].motor, 0.1f, 1.0f}, {motors[1].motor, 0.2f, 1.0f}};
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.block_nonempty_send = true;
    driver.block_target_position = commands[0].target_position;
  }
  runtime.submit_pos_vel_ex(
      commands, 2, ARTICORE_COMMAND_HOLD_UNTIL_REPLACED);
  {
    std::unique_lock<std::mutex> lock(driver.mutex);
    require(driver.send_cv.wait_for(lock, 200ms,
                                    [&] { return driver.send_entered; }),
            "test control batch entered the fake transport");
  }
  bool rejected = false;
  std::thread concurrent_submit([&] {
    std::this_thread::sleep_for(5ms);
    try {
      runtime.submit_pos_vel_ex(
          commands, 2, ARTICORE_COMMAND_HOLD_UNTIL_REPLACED);
    } catch (const std::runtime_error&) {
      rejected = true;
    }
  });
  bool disable_waited_for_batch = false;
  std::thread releaser([&] {
    std::this_thread::sleep_for(20ms);
    std::lock_guard<std::mutex> lock(driver.mutex);
    disable_waited_for_batch =
        driver.disable_calls[0] == 0 && driver.disable_calls[1] == 0;
    driver.release_send = true;
    driver.send_cv.notify_all();
  });
  std::string disable_error;
  try {
    runtime.disable();
  } catch (const std::exception& error) {
    disable_error = error.what();
  }
  releaser.join();
  concurrent_submit.join();
  require(disable_waited_for_batch,
          "disable frames wait until the in-flight control batch completes");
  require(rejected && disable_error.empty() &&
              runtime.health().disable_confirmed == 1,
          "transition rejects new commands and completes after the batch barrier");
}

void test_raw_mit_publish_does_not_wait_for_inflight_transport_send() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);
  ArticoreMitCommand first[] = {
      {motors[0].motor, 0.1f, 0.0f, 5.0f, 1.0f, 0.0f},
      {motors[1].motor, 0.2f, 0.0f, 5.0f, 1.0f, 0.0f}};
  ArticoreMitCommand replacement[] = {
      {motors[0].motor, 0.3f, 0.0f, 5.0f, 1.0f, 0.0f},
      {motors[1].motor, 0.4f, 0.0f, 5.0f, 1.0f, 0.0f}};
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.block_nonempty_send = true;
    driver.block_target_position = first[0].target_position;
  }
  runtime.submit_mit(first, 2);
  {
    std::unique_lock<std::mutex> lock(driver.mutex);
    require(driver.send_cv.wait_for(lock, 200ms,
                                    [&] { return driver.send_entered; }),
            "test MIT batch entered the fake transport");
  }

  std::atomic<bool> publish_finished{false};
  std::string publish_error;
  std::thread publisher([&] {
    try {
      runtime.submit_mit(replacement, 2);
    } catch (const std::exception& error) {
      publish_error = error.what();
    }
    publish_finished.store(true, std::memory_order_release);
  });
  const bool published_while_send_blocked = wait_for(
      [&] { return publish_finished.load(std::memory_order_acquire); }, 50ms);
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.release_send = true;
    driver.send_cv.notify_all();
  }
  publisher.join();

  require(published_while_send_blocked && publish_error.empty(),
          "raw MIT publish completes while the previous transport send is in flight");
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return !driver.last_arm_mit.empty() &&
                   std::abs(driver.last_arm_mit[0].target_position -
                            replacement[0].target_position) < 1e-6f;
          }),
          "worker consumes the newest pending raw MIT generation");
  runtime.disable();
}

void test_raw_mit_torque_limit_recomputes_on_every_native_cycle() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.reserved_control_rate = 500;
  cfg.command_timeout_ms = 500;
  articore::SafetyRuntime runtime(
      cfg, api(), reinterpret_cast<void*>(0x100),
      g_left_controller, g_right_controller, motors);
  const auto configured = joint_configs(motors);
  runtime.configure_joints(configured.data(),
                           static_cast<uint32_t>(configured.size()));
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);

  // Left requested output is 10*(1-0) + 4*(1-(-1)) + 3 = 21. The
  // configured torque limit is 10, which is also the native bound.
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[0].motor].position = 0.0f;
    driver.motors[motors[0].motor].velocity = -1.0f;
  }
  ArticoreMitCommand commands[] = {
      {motors[0].motor, 1.0f, 1.0f, 10.0f, 4.0f, 3.0f},
      {motors[1].motor, 1.0f, 0.0f, 10.0f, 4.0f, 0.0f},
  };
  runtime.submit_mit(commands, 2);
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return !driver.last_arm_mit.empty() &&
                driver.last_arm_mit[0].target_position == 1.0f &&
                driver.last_arm_mit[0].stiffness < 10.0f;
          }),
          "native MIT cycle limits complete P+D+FF output");
  ArticoreMitCommand limited{};
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    limited = driver.last_arm_mit[0];
  }
  const float expected_scale = 10.0f / 21.0f;
  require(std::abs(limited.stiffness - 10.0f * expected_scale) < 1e-5f &&
              std::abs(limited.damping - 4.0f * expected_scale) < 1e-5f &&
              std::abs(limited.feedforward_torque -
                       3.0f * expected_scale) < 1e-5f,
          "Kp, Kd and feedforward retain their ratio when limited");
  auto stats = runtime.mit_torque_limit_stats();
  require(stats.torque_limit_activation_count > 0 &&
              stats.torque_limited_joint_mask == 1 &&
              stats.joint_count == 2 &&
              stats.joints[0] == motors[0].motor &&
              std::abs(stats.requested_resultant_torque[0] - 21.0f) < 1e-5f &&
              std::abs(stats.applied_scale[0] - expected_scale) < 1e-5f &&
              std::abs(stats.applied_resultant_torque[0] - 10.0f) < 1e-5f,
          "MIT limiter statistics describe the actual native send cycle");

  std::size_t history_before = 0;
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    history_before = driver.arm_mit_history.size();
    // With no new submit, the same target now requests exactly the complete
    // configured range: 10*(1-.5) + 4*(1-.5) + 3 = 10.
    driver.motors[motors[0].motor].position = 0.5f;
    driver.motors[motors[0].motor].velocity = 0.5f;
  }
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.arm_mit_history.size() > history_before &&
                std::abs(driver.arm_mit_history.back()[0].stiffness - 10.0f) <
                    1e-6f;
          }),
          "repeated mailbox target is recomputed from newer feedback");
  stats = runtime.mit_torque_limit_stats();
  require(stats.torque_limited_joint_mask == 0 &&
              std::abs(stats.requested_resultant_torque[0] - 10.0f) < 1e-5f &&
              stats.applied_scale[0] == 1.0f &&
              std::abs(stats.applied_resultant_torque[0] - 10.0f) < 1e-5f,
          "complete configured range remains available without scaling");
  runtime.disable();
}

void test_raw_mit_torque_limit_keeps_sending_with_stale_feedback() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  articore::SafetyRuntime runtime(
      cfg, api(), reinterpret_cast<void*>(0x100),
      g_left_controller, g_right_controller, motors);
  const auto configured = joint_configs(motors);
  runtime.configure_joints(configured.data(),
                           static_cast<uint32_t>(configured.size()));
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);
  ArticoreMitCommand commands[] = {
      {motors[0].motor, 0.1f, 0.0f, 10.0f, 1.0f, 0.0f},
      {motors[1].motor, 0.9f, 0.0f, 10.0f, 1.0f, 0.0f},
  };
  runtime.submit_mit_ex(
      commands, 2, ARTICORE_COMMAND_HOLD_UNTIL_REPLACED);
  require(wait_for([&] { return runtime.health().state == ARTICORE_RUNNING; }),
          "stale-feedback test reaches RUNNING");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[0].motor].age_ns = 201'000'000ULL;
  }
  std::size_t history_before = 0;
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    history_before = driver.arm_mit_history.size();
  }
  require(wait_for([&] {
            if (runtime.health().state != ARTICORE_RUNNING ||
                runtime.health().consecutive_feedback_failures == 0) {
              return false;
            }
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.arm_mit_history.size() > history_before;
          }),
          "stale feedback is diagnosed while native MIT sending continues");
  require(runtime.health().safe_holding == 0,
          "stale feedback alone does not enter protective hold");
  runtime.disable();
}

void test_close_reuses_checked_disable_transaction() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  articore::SafetyRuntime runtime(config(), api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);
  runtime.close();
  const auto health = runtime.health();
  const auto report = runtime.last_disable_report();
  require(health.state == ARTICORE_DISCONNECTED &&
              health.disable_confirmed == 1 && report.success == 1 &&
              report.expected_count == 3 && report.disabled_count == 3,
          "close confirms physical disable before disconnecting the Runtime");
}

void test_close_stops_worker_after_unconfirmed_disable() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  articore::SafetyRuntime runtime(config(), api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.feedback_code = 3;
    driver.feedback_expected = 2;
    driver.feedback_received = 0;
    driver.feedback_missing_ids = {1, 2, 3};
  }
  bool failed = false;
  try {
    runtime.close();
  } catch (const std::runtime_error&) {
    failed = true;
  }
  const auto report = runtime.last_disable_report();
  require(failed && runtime.health().state == ARTICORE_DISCONNECTED &&
              runtime.health().disable_confirmed == 0 && report.success == 0 &&
              report.missing_count > 0,
          "close reports missing motors after terminally stopping the worker");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.feedback_code = 0;
    driver.feedback_received = 2;
    driver.feedback_missing_ids.clear();
  }
  runtime.close();
  require(runtime.health().state == ARTICORE_DISCONNECTED &&
              runtime.health().disable_confirmed == 0,
          "repeated close is idempotent after an unconfirmed terminal shutdown");
}

void test_transport_disconnect_holds_the_connected_side() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  ArticorePosVelCommand commands[] = {
      {motors[0].motor, 0.0f, 1.0f}, {motors[1].motor, 0.0f, 1.0f}};
  runtime.submit_pos_vel(commands, 2);
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.transport_connected[1] = false;
    driver.transport_healthy[1] = false;
  }
  require(wait_for([&] {
            if (runtime.health().state != ARTICORE_FAULT) return false;
            std::lock_guard<std::mutex> lock(driver.mutex);
            return std::any_of(
                driver.pv_history.begin(), driver.pv_history.end(),
                [&](const auto& batch) {
                  return batch.size() == 1 &&
                         batch[0].motor == motors[0].motor;
                });
          }),
          "a disconnected transport faults normal motion and holds the connected side");
  const auto health = runtime.health();
  require(health.right_transport.connected == 0 &&
              health.right_transport.healthy == 0,
          "structured health identifies the disconnected side");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.disable_calls[0] == 0 && driver.disable_calls[1] == 0,
            "transport disconnect does not torque off the controllable side");
  }
}

void test_reported_feedback_failure_is_diagnostic_only() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  ArticorePosVelCommand commands[] = {
      {motors[0].motor, 0.25f, 1.0f}, {motors[1].motor, 0.75f, 1.0f}};
  runtime.submit_pos_vel_ex(commands, 2, ARTICORE_COMMAND_HOLD_UNTIL_REPLACED);
  require(wait_for([&] { return runtime.health().state == ARTICORE_RUNNING; }),
          "per-channel hold test reaches RUNNING");

  runtime.report_feedback_failure(0, "injected miss 1");
  runtime.report_feedback_failure(0, "injected miss 2");
  runtime.report_feedback_failure(0, "injected miss 3");
  const auto health = runtime.health();
  require(health.state == ARTICORE_RUNNING && health.safe_holding == 0 &&
              health.consecutive_feedback_failures == 3 &&
              health.left_transport.consecutive_feedback_failures >= 3,
          "reported feedback misses update diagnostics without changing state");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.disable_calls[0] == 0 && driver.disable_calls[1] == 0,
            "reported feedback misses do not disable either channel");
  }
  runtime.disable();
}

void test_enable_grace_enters_safe_stop() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  articore::SafetyRuntime runtime(config(), api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  ArticorePosVelCommand invalid_commands[] = {
      {motors[0].motor, std::numeric_limits<float>::quiet_NaN(), 1.0f},
      {motors[1].motor, 0.0f, 1.0f},
  };
  bool invalid_rejected = false;
  try {
    runtime.submit_pos_vel(invalid_commands, 2);
  } catch (const std::invalid_argument&) {
    invalid_rejected = true;
  }
  require(invalid_rejected && runtime.health().state == ARTICORE_ENABLED,
          "invalid command is rejected without refreshing the enable grace");
  require(wait_for([&] { return runtime.health().state == ARTICORE_SAFE_STOP; }),
          "missing first command safely stops after enable grace");
  ArticorePosVelCommand commands[] = {
      {motors[0].motor, 0.0f, 1.0f}, {motors[1].motor, 0.0f, 1.0f}};
  bool rejected = false;
  try {
    runtime.submit_pos_vel(commands, 2);
  } catch (const std::exception&) {
    rejected = true;
  }
  require(rejected && runtime.health().state == ARTICORE_SAFE_STOP &&
              runtime.health().fault_reason[0] == '\0',
          "ordinary command cannot bypass SAFE_STOP or create a motor fault");
}

void test_atomic_enable_starts_hold_and_confirms_both_sides() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  for (auto& entry : driver.motors) entry.second.status = 0;
  auto cfg = config();
  cfg.enable_grace_ms = 200;
  articore::SafetyRuntime runtime(
      cfg, api(), reinterpret_cast<void*>(0x100), g_left_controller,
      g_right_controller, motors, enable_all, enable_motor);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  const auto health = runtime.health();
  const auto report = runtime.last_enable_report();
  require(health.state == ARTICORE_ENABLED && health.disable_confirmed == 0,
          "atomic enable enters ENABLED after confirmation");
  require(report.success == 1 && report.expected_count == 3 &&
              report.enabled_count == 3 && report.failure_count == 0,
          "atomic enable exposes a successful structured report");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.enable_calls[0] == 1 && driver.enable_calls[1] == 1,
            "atomic enable invokes both active controllers");
    require(driver.pv_sends > 0 && driver.mit_sends > 0,
            "atomic enable sends immediate arm and gripper holds");
    require(driver.feedback_requests >= 4,
            "atomic enable refreshes both channels before and after enable");
  }
}

void test_atomic_enable_retries_one_disabled_motor_once() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  for (auto& entry : driver.motors) entry.second.status = 0;
  driver.skip_left_enable_once = true;
  articore::SafetyRuntime runtime(
      config(), api(), reinterpret_cast<void*>(0x100), g_left_controller,
      g_right_controller, motors, enable_all, enable_motor);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  const auto report = runtime.last_enable_report();
  require(report.success == 1 && report.enabled_count == 3,
          "one missed enable frame is recovered inside the transaction");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.motor_enable_calls == 1,
            "only the still-disabled motor receives one enable retry");
  }
}

void test_gripper_control_waits_for_atomic_enable_confirmation() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  for (auto& entry : driver.motors) entry.second.status = 0;
  driver.skip_gripper_enable_once = true;
  driver.motor_enable_delay_ms = 15;
  auto cfg = config();
  cfg.enable_grace_ms = 100;
  articore::SafetyRuntime runtime(
      cfg, api(), reinterpret_cast<void*>(0x100), g_left_controller,
      g_right_controller, motors, enable_all, enable_motor);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);
  const auto health = runtime.health();
  require(health.state == ARTICORE_ENABLED && health.fault_reason[0] == '\0' &&
              runtime.last_enable_report().success == 1,
          "normal gripper control cannot observe transient DISABLED feedback "
          "inside the atomic enable transaction");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.motor_enable_calls == 1 &&
                driver.motors[motors[2].motor].status == 1,
            "the delayed gripper is retried once and confirmed before control starts");
  }
}

void test_atomic_enable_failure_rolls_back_and_fault_disable_is_allowed() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  driver.emulate_arm_feedback = true;
  for (auto& entry : driver.motors) entry.second.status = 0;
  driver.fail_enable[1] = true;
  articore::SafetyRuntime runtime(
      config(), api(), reinterpret_cast<void*>(0x100), g_left_controller,
      g_right_controller, motors, enable_all, enable_motor, false, {},
      maintenance_api());
  const auto controls = joint_configs(motors);
  runtime.configure_joints(controls.data(),
                           static_cast<uint32_t>(controls.size()));
  runtime.connect();
  bool rejected = false;
  try {
    runtime.enable(ARTICORE_MODE_PV);
  } catch (const std::runtime_error&) {
    rejected = true;
  }
  auto health = runtime.health();
  auto report = runtime.last_enable_report();
  require(rejected && health.state == ARTICORE_FAULT &&
              health.disable_confirmed == 1,
          "failed atomic enable faults and confirms rollback disable");
  require(report.success == 0 && report.disable_confirmed == 1,
          "failed atomic enable preserves a structured rollback report");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    for (const auto& entry : driver.motors) {
      require(entry.second.status == 0,
              "failed atomic enable leaves no motor enabled");
    }
  }

  runtime.disable();
  health = runtime.health();
  require(health.state == ARTICORE_FAULT && health.disable_confirmed == 1,
          "disable is idempotent in latched FAULT without clearing it");
  driver.fail_enable[1] = false;
  runtime.recover();
  require(runtime.health().state == ARTICORE_READY,
          "recover clears the latch only after confirmed physical disable");
}

void test_repeated_runtime_lifecycle() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  for (int i = 0; i < 10; ++i) {
    articore::SafetyRuntime runtime(config(), api(), reinterpret_cast<void*>(0x100),
                                    g_left_controller, g_right_controller, motors);
    runtime.connect();
    require(runtime.health().state == ARTICORE_READY,
            "repeated runtime connect reaches READY");
    runtime.close();
    require(runtime.health().state == ARTICORE_DISCONNECTED,
            "repeated runtime close reaches DISCONNECTED");
  }
}

void test_native_scheduler_selects_rate_from_transport_capabilities() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.reserved_control_rate = 17;
  articore::SafetyRuntime legacy_dual(
      cfg, api(), reinterpret_cast<void*>(0x100), g_left_controller,
      g_right_controller, motors);
  require(legacy_dual.control_hz() == 400,
          "legacy dual runtime ignores the public placeholder");

  articore::SafetyRuntime socketcanfd_brs_dual(
      cfg, api(), reinterpret_cast<void*>(0x101), g_left_controller,
      g_right_controller, motors, nullptr, nullptr, false,
      transport_capabilities("socketcanfd", true, true));
  require(socketcanfd_brs_dual.control_hz() == 500,
          "native scheduler selects its verified SocketCAN-FD+BRS cadence");

  articore::SafetyRuntime dm_device_dual(
      cfg, api(), reinterpret_cast<void*>(0x102), g_left_controller,
      g_right_controller, motors, nullptr, nullptr, false,
      transport_capabilities("dm-device", true, true));
  require(dm_device_dual.control_hz() == 400,
          "dual DM Device runtime remains capped at 400 Hz");

  auto mixed_capabilities = transport_capabilities("socketcanfd", true, true);
  std::strncpy(mixed_capabilities[1].transport, "socketcan",
               sizeof(mixed_capabilities[1].transport) - 1);
  mixed_capabilities[1].can_fd_brs = 0;
  articore::SafetyRuntime mixed_dual(
      cfg, api(), reinterpret_cast<void*>(0x103), g_left_controller,
      g_right_controller, motors, nullptr, nullptr, false,
      mixed_capabilities);
  require(mixed_dual.control_hz() == 400,
          "both dual transports must report SocketCAN-FD+BRS for 500 Hz");

  std::vector<ArticoreMotorDescriptor> single_motors{motors[0]};
  articore::SafetyRuntime single(
      cfg, api(), reinterpret_cast<void*>(0x104), g_left_controller, nullptr,
      single_motors);
  require(single.control_hz() == 400,
          "single-side scheduling is selected internally");
}

void test_single_side_runtime_and_gripper() {
  FakeDriver driver;
  g_driver = &driver;
  auto all_motors = descriptors(driver);
  std::vector<ArticoreMotorDescriptor> motors{
      all_motors[0], all_motors[2]};
  motors[1].side = 0;
  std::strncpy(motors[1].name, "gripper", sizeof(motors[1].name) - 1);
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  articore::SafetyRuntime runtime(
      cfg, api(), reinterpret_cast<void*>(0x100), g_left_controller, nullptr,
      motors);
  runtime.connect();
  auto health = runtime.health();
  require(health.state == ARTICORE_READY &&
              health.left_transport.connected == 1 &&
              health.right_transport.connected == 0,
          "single-side runtime exposes only its active transport");

  runtime.enable(ARTICORE_MODE_PV);
  ArticorePosVelCommand command{motors[0].motor, 0.25f, 1.0f};
  runtime.submit_pos_vel(&command, 1);
  ArticoreGripperTarget target{motors[1].motor, 500.0f};
  runtime.set_gripper_openings(&target, 1);
  require(wait_for([&] {
            const auto current = runtime.health();
            return current.state == ARTICORE_RUNNING &&
                   current.gripper_count == 1 &&
                   current.grippers[0].side == 0 &&
                   current.grippers[0].control_state ==
                       ARTICORE_GRIPPER_MOVING;
          }),
          "single-side runtime drives its gripper state machine");
  runtime.disable();
  health = runtime.health();
  require(health.state == ARTICORE_READY && health.disable_confirmed == 1,
          "single-side disable confirms only active motors");
}

void test_normal_gripper_uses_arm_control_rate() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.reserved_control_rate = 1;
  cfg.command_timeout_ms = 500;
  // Public rate placeholders are ignored. The private override keeps this
  // scheduler-mechanics test deterministic without exposing a product API.
  cfg.reserved_gripper_control_rate = 1;
  articore::SafetyRuntime runtime(
      cfg, api(), reinterpret_cast<void*>(0x100), g_left_controller,
      g_right_controller, motors, nullptr, nullptr, false, {}, {}, 500);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);
  ArticoreMitCommand arm_commands[] = {
      {motors[0].motor, 0.2f, 0.0f, 5.0f, 0.5f, 0.0f},
      {motors[1].motor, 0.8f, 0.0f, 5.0f, 0.5f, 0.0f},
  };
  runtime.submit_mit_ex(
      arm_commands, 2, ARTICORE_COMMAND_HOLD_UNTIL_REPLACED);
  ArticoreGripperTarget target{motors[2].motor, 500.0f};
  runtime.set_gripper_openings(&target, 1);

  std::size_t arm_baseline = 0;
  std::size_t gripper_baseline = 0;
  std::size_t group_baseline = 0;
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    arm_baseline = driver.arm_mit_history.size();
    group_baseline = driver.group_mit_history.size();
    gripper_baseline = static_cast<std::size_t>(std::count_if(
        driver.mit_history.begin(), driver.mit_history.end(),
        [&](const auto& batch) {
          return batch.size() == 1 && batch[0].motor == motors[2].motor;
        }));
  }

  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.arm_mit_history.size() >= arm_baseline + 10;
          }, 1000ms),
          "normal arm control produces periodic output");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    const auto arm_count = driver.arm_mit_history.size() - arm_baseline;
    const auto gripper_total = static_cast<std::size_t>(std::count_if(
        driver.mit_history.begin(), driver.mit_history.end(),
        [&](const auto& batch) {
          return batch.size() == 1 && batch[0].motor == motors[2].motor;
        }));
    const auto gripper_count = gripper_total - gripper_baseline;
    require(gripper_count + 2 >= arm_count,
            "normal gripper output follows arm control_hz instead of the legacy 100 Hz field");
    const auto combined = std::find_if(
        driver.group_mit_history.begin() +
            static_cast<std::ptrdiff_t>(group_baseline),
        driver.group_mit_history.end(), [&](const auto& batch) {
          return batch.size() == 3 && batch.front().motor == motors[2].motor;
        });
    require(combined != driver.group_mit_history.end(),
            "MIT arm and gripper commands share one native ControllerGroup batch");
  }
}

void test_gripper_health_reads_live_feedback_age() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[2].motor].age_ns = 7'654'321ULL;
  }
  const auto health = runtime.health();
  require(health.gripper_count == 1 &&
              health.grippers[0].feedback_age_ns == 7'654'321ULL,
          "gripper health reports the current Motor cache age instead of a prior control tick");
}

void test_motor_presence_is_fixed_and_fault_aware() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.feedback_check_hz = 500;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.declare_motor_presence("left/optional_tool", ARTICORE_NOT_INSTALLED);
  require(runtime.motor_presence("left/optional_tool") == ARTICORE_NOT_INSTALLED &&
              runtime.motor_presence("right/gripper") == ARTICORE_PRESENT,
          "runtime distinguishes absent optional roles from present motors");
  const auto capabilities = runtime.active_capabilities();
  require((capabilities & ARTICORE_ACTIVE_ARM_SIDE_0) != 0 &&
              (capabilities & ARTICORE_ACTIVE_ARM_SIDE_1) != 0 &&
              (capabilities & ARTICORE_ACTIVE_GRIPPER_SIDE_1) != 0 &&
              (capabilities & ARTICORE_ACTIVE_GRIPPER_SIDE_0) == 0,
          "active capabilities reflect the fixed discovered descriptor set");
  runtime.connect();
  bool mutation_rejected = false;
  try {
    runtime.declare_motor_presence("late_tool", ARTICORE_NOT_INSTALLED);
  } catch (const std::runtime_error&) {
    mutation_rejected = true;
  }
  require(mutation_rejected,
          "presence declarations cannot change after connect");
  runtime.enable(ARTICORE_MODE_PV);
  ArticorePosVelCommand commands[] = {
      {motors[0].motor, 0.1f, 1.0f},
      {motors[1].motor, -0.1f, 1.0f},
  };
  runtime.submit_pos_vel_ex(
      commands, 2, ARTICORE_COMMAND_HOLD_UNTIL_REPLACED);
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[2].motor].has_feedback = false;
  }
  require(wait_for([&] {
            return runtime.health().right_transport.healthy == 0 &&
                   runtime.health().consecutive_feedback_failures > 0;
          }),
          "a present motor that loses feedback is reported as unhealthy");
  require(runtime.motor_presence("right/gripper") == ARTICORE_PRESENT &&
              runtime.health().state == ARTICORE_RUNNING,
          "diagnostic-only feedback loss preserves fixed presence and RUNNING state");
  runtime.disable();
}

void test_latest_value_mailbox_drops_superseded_targets() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.reserved_control_rate = 5000;
  cfg.command_timeout_ms = 500;
  // This test intentionally stretches the control period to 50 ms so all
  // three submissions land before the next tick. Keep the enable grace well
  // above that period; otherwise a delayed CI runner can fault before the
  // pending mailbox command gets its first send opportunity.
  cfg.enable_grace_ms = 1000;
  articore::SafetyRuntime runtime(
      cfg, api(), reinterpret_cast<void*>(0x100), g_left_controller,
      g_right_controller, motors, nullptr, nullptr, false, {}, {}, 20);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return !driver.pv_history.empty();
          }),
          "enable sends a feedback-seeded target");

  ArticorePosVelCommand a[] = {
      {motors[0].motor, 0.1f, 1.0f}, {motors[1].motor, 0.9f, 1.0f}};
  ArticorePosVelCommand b[] = {
      {motors[0].motor, 0.2f, 1.0f}, {motors[1].motor, 0.8f, 1.0f}};
  ArticorePosVelCommand c[] = {
      {motors[0].motor, 0.3f, 1.0f}, {motors[1].motor, 0.7f, 1.0f}};
  std::size_t baseline = 0;
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    baseline = driver.pv_history.size();
  }
  runtime.submit_pos_vel(a, 2);
  runtime.submit_pos_vel(b, 2);
  runtime.submit_pos_vel(c, 2);
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.pv_history.size() > baseline;
          }, 200ms),
          "mailbox target is sent on the next control tick");
  std::lock_guard<std::mutex> lock(driver.mutex);
  const auto& first = driver.pv_history[baseline];
  require(first.size() == 2 &&
              std::abs(first[0].target_position - 0.3f) < 1e-6f &&
              std::abs(first[1].target_position - 0.7f) < 1e-6f,
          "A and B are overwritten; the next tick sends only C");
}

void test_latest_value_mailbox_stays_bounded_under_fast_producer() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.reserved_control_rate = 500;
  cfg.command_timeout_ms = 500;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);

  std::size_t baseline = 0;
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    baseline = driver.pv_history.size();
  }
  ArticorePosVelCommand commands[] = {
      {motors[0].motor, 0.0f, 1.0f},
      {motors[1].motor, 1.0f, 1.0f},
  };
  for (int sequence = 0; sequence < 5000; ++sequence) {
    const auto ratio = static_cast<float>(sequence % 100) / 100.0f;
    commands[0].target_position = 0.2f * ratio;
    commands[1].target_position = 1.0f - 0.2f * ratio;
    runtime.submit_pos_vel_ex(
        commands, 2, ARTICORE_COMMAND_HOLD_UNTIL_REPLACED);
  }
  commands[0].target_position = 0.333f;
  commands[1].target_position = 0.667f;
  runtime.submit_pos_vel_ex(
      commands, 2, ARTICORE_COMMAND_HOLD_UNTIL_REPLACED);

  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.last_pv.size() == 2 &&
                   std::abs(driver.last_pv[0].target_position - 0.333f) < 1e-6f &&
                   std::abs(driver.last_pv[1].target_position - 0.667f) < 1e-6f;
          }, 300ms),
          "the final producer value reaches the effective-rate sender");
  std::this_thread::sleep_for(10ms);
  std::lock_guard<std::mutex> lock(driver.mutex);
  require(driver.pv_history.size() - baseline < 5000 &&
              std::abs(driver.last_pv[0].target_position - 0.333f) < 1e-6f &&
              std::abs(driver.last_pv[1].target_position - 0.667f) < 1e-6f,
          "fast producer values are coalesced into one slot and no stale target reappears");
}

void test_persistent_setpoints_outlive_watchdog_but_streaming_still_times_out() {
  {
    FakeDriver driver;
    g_driver = &driver;
    auto motors = descriptors(driver);
    auto cfg = config();
    cfg.command_timeout_ms = 50;
    articore::SafetyRuntime runtime(
        cfg, api(), reinterpret_cast<void*>(0x100), g_left_controller,
        g_right_controller, motors);
    const auto configured = joint_configs(motors);
    runtime.configure_joints(configured.data(),
                             static_cast<uint32_t>(configured.size()));
    runtime.connect();
    runtime.enable(ARTICORE_MODE_MIT);

    ArticoreMitCommand persistent[] = {
        {motors[0].motor, 0.4f, 0.0f, 20.0f, 3.0f, 0.0f},
        {motors[1].motor, 0.6f, 0.0f, 20.0f, 3.0f, 0.0f},
    };
    runtime.submit_mit_ex(
        persistent, 2, ARTICORE_COMMAND_HOLD_UNTIL_REPLACED);
    require(wait_for([&] { return runtime.health().state == ARTICORE_RUNNING; }),
            "persistent MIT setpoint enters RUNNING");
    std::size_t baseline = 0;
    {
      std::lock_guard<std::mutex> lock(driver.mutex);
      baseline = driver.arm_mit_history.size();
    }
    std::this_thread::sleep_for(170ms);
    require(runtime.health().state == ARTICORE_RUNNING &&
                runtime.health().safe_holding == 0,
            "one-shot MIT setpoint remains RUNNING beyond three watchdog periods");
    {
      std::lock_guard<std::mutex> lock(driver.mutex);
      require(driver.arm_mit_history.size() >= baseline + 5 &&
                  std::abs(driver.arm_mit_history.back()[0].target_position -
                           0.4f) < 1e-6f,
              "persistent MIT setpoint continues to transmit while motion may be slow");
    }

    runtime.submit_mit_ex(persistent, 2, ARTICORE_COMMAND_STREAMING);
    require(wait_for([&] { return runtime.health().state == ARTICORE_SAFE_STOP; }),
            "explicit streaming MIT still times out when updates stop");
  }

  {
    FakeDriver driver;
    g_driver = &driver;
    auto motors = descriptors(driver);
    auto cfg = config();
    cfg.command_timeout_ms = 50;
    articore::SafetyRuntime runtime(
        cfg, api(), reinterpret_cast<void*>(0x100), g_left_controller,
        g_right_controller, motors);
    const auto configured = joint_configs(motors);
    runtime.configure_joints(configured.data(),
                             static_cast<uint32_t>(configured.size()));
    runtime.connect();
    runtime.enable(ARTICORE_MODE_PV);

    ArticorePosVelCommand persistent[] = {
        {motors[0].motor, 0.4f, 0.05f},
        {motors[1].motor, 0.6f, 0.05f},
    };
    runtime.submit_pos_vel_ex(
        persistent, 2, ARTICORE_COMMAND_HOLD_UNTIL_REPLACED);
    require(wait_for([&] { return runtime.health().state == ARTICORE_RUNNING; }),
            "persistent slow PV setpoint enters RUNNING");
    std::this_thread::sleep_for(170ms);
    require(runtime.health().state == ARTICORE_RUNNING &&
                runtime.health().safe_holding == 0,
            "slow PV motion may outlive the command watchdog");

    runtime.submit_pos_vel_ex(persistent, 2, ARTICORE_COMMAND_STREAMING);
    require(wait_for([&] { return runtime.health().state == ARTICORE_SAFE_STOP; }),
            "explicit streaming PV still times out when updates stop");
  }
}

void test_persistent_mit_rejects_unbounded_motion_terms() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  articore::SafetyRuntime runtime(config(), api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  const auto configured = joint_configs(motors);
  runtime.configure_joints(configured.data(),
                           static_cast<uint32_t>(configured.size()));
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);

  ArticoreMitCommand moving[] = {
      {motors[0].motor, 0.4f, 0.1f, 20.0f, 3.0f, 0.0f},
      {motors[1].motor, 0.6f, 0.0f, 20.0f, 3.0f, 0.0f},
  };
  bool velocity_rejected = false;
  try {
    runtime.submit_mit_ex(
        moving, 2, ARTICORE_COMMAND_HOLD_UNTIL_REPLACED);
  } catch (const std::invalid_argument&) {
    velocity_rejected = true;
  }
  require(velocity_rejected,
          "persistent MIT rejects nonzero target velocity");

  moving[0].target_velocity = 0.0f;
  moving[0].feedforward_torque = 0.1f;
  bool torque_rejected = false;
  try {
    runtime.submit_mit_ex(
        moving, 2, ARTICORE_COMMAND_HOLD_UNTIL_REPLACED);
  } catch (const std::invalid_argument&) {
    torque_rejected = true;
  }
  require(torque_rejected,
          "persistent MIT rejects nonzero feedforward torque");
}

void test_ordinary_mit_position_uses_constant_reference_speed() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.command_timeout_ms = 30;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  const auto configured = joint_configs(motors);
  runtime.configure_joints(configured.data(),
                           static_cast<uint32_t>(configured.size()));
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);

  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[0].motor].age_ns = 300'000'000ULL;
  }
  ArticoreJointMitTarget targets[] = {
      {sizeof(ArticoreJointMitTarget), motors[0].motor, 1.0f},
      {sizeof(ArticoreJointMitTarget), motors[1].motor, 2.0f},
  };
  bool stale_rejected = false;
  try {
    runtime.set_joint_mit(targets, 2, 1.0f);
  } catch (const std::runtime_error&) {
    stale_rejected = true;
  }
  require(stale_rejected,
          "ordinary MIT position requires complete fresh feedback initially");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[0].motor].age_ns = 0;
    driver.arm_mit_history.clear();
  }

  runtime.set_joint_mit(targets, 2, 1.0f);
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.arm_mit_history.size() >= 105;
          }, 1500ms),
          "ordinary MIT position emits control-rate references");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    const auto first_moving = std::find_if(
        driver.arm_mit_history.begin(), driver.arm_mit_history.end(),
        [](const std::vector<ArticoreMitCommand>& frame) {
          return frame[0].target_position > 0.0f;
        });
    require(first_moving != driver.arm_mit_history.end(),
            "ordinary MIT position starts moving from fresh feedback");
    require(first_moving->at(0).target_position <= 0.00251f &&
                first_moving->at(1).target_position <= 1.00251f,
            "first 400 Hz reference advances by at most velocity/control_hz");
    require(first_moving->at(0).target_velocity == 0.0f &&
                first_moving->at(0).stiffness == 20.0f &&
                first_moving->at(0).damping == 3.0f &&
                first_moving->at(0).feedforward_torque == 0.0f,
            "ordinary MIT position uses product gains with zero dq and tau");
    const auto available = static_cast<std::size_t>(
        driver.arm_mit_history.end() - first_moving);
    require(available >= 100,
            "ordinary MIT position produced one hundred moving references");
    require(std::abs(first_moving[99][0].target_position - 0.25f) < 0.003f,
            "one hundred 400 Hz cycles at 1 rad/s advance about 0.25 rad");
  }
  require(runtime.health().state == ARTICORE_RUNNING,
          "one-shot ordinary MIT position is persistent beyond the watchdog");
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return !driver.arm_mit_history.empty() &&
                std::abs(driver.arm_mit_history.back()[0].target_position -
                         1.0f) < 1e-5f;
          }, 3000ms),
          "ordinary MIT position reaches a 1 rad target at 1 rad/s");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    const auto first_moving = std::find_if(
        driver.arm_mit_history.begin(), driver.arm_mit_history.end(),
        [](const std::vector<ArticoreMitCommand>& frame) {
          return frame[0].target_position > 0.0f;
        });
    const auto first_reached = std::find_if(
        first_moving, driver.arm_mit_history.end(),
        [](const std::vector<ArticoreMitCommand>& frame) {
          return std::abs(frame[0].target_position - 1.0f) < 1e-5f;
        });
    require(first_reached != driver.arm_mit_history.end(),
            "ordinary MIT position records the exact final target");
    const auto moving_cycles = first_reached - first_moving + 1;
    require(moving_cycles >= 399 && moving_cycles <= 401,
            "1 rad at 1 rad/s takes approximately 400 native 400 Hz cycles");
  }
}

void test_ordinary_speed_percent_scales_and_zero_pauses() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  articore::SafetyRuntime runtime(
      config(), api(), reinterpret_cast<void*>(0x100),
      g_left_controller, g_right_controller, motors);
  const auto configured = joint_configs(motors);
  runtime.configure_joints(configured.data(),
                           static_cast<uint32_t>(configured.size()));
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  ArticoreJointPvTarget targets[] = {
      {sizeof(ArticoreJointPvTarget), motors[0].motor, 1.0f},
      {sizeof(ArticoreJointPvTarget), motors[1].motor, 2.0f},
  };

  runtime.set_joint_pv_speed(targets, 2, 0.0f);
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.last_pv.size() == 2;
          }),
          "zero ordinary speed still transmits a safe current-position hold");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(std::abs(driver.last_pv[0].target_position) < 1e-6f &&
                std::abs(driver.last_pv[1].target_position - 1.0f) < 1e-6f,
            "zero ordinary speed pauses reference advancement");
  }

  runtime.set_joint_pv_speed(targets, 2, 50.0f);
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.last_pv.size() == 2 &&
                   driver.last_pv[0].target_position > 0.0f &&
                   std::abs(driver.last_pv[0].velocity_limit - 2.5f) < 1e-6f;
          }),
          "fifty percent maps to half the shared physical velocity limit");

  bool invalid_rejected = false;
  try {
    runtime.set_joint_pv_speed(targets, 2, 100.1f);
  } catch (const std::invalid_argument&) {
    invalid_rejected = true;
  }
  require(invalid_rejected, "ordinary speed above 100 is rejected natively");
}

void test_ordinary_pv_position_latest_value_and_raw_pv_remains_direct() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.command_timeout_ms = 30;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  const auto configured = joint_configs(motors);
  runtime.configure_joints(configured.data(),
                           static_cast<uint32_t>(configured.size()));
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);

  ArticoreJointPvTarget forward[] = {
      {sizeof(ArticoreJointPvTarget), motors[0].motor, 1.0f},
      {sizeof(ArticoreJointPvTarget), motors[1].motor, 2.0f},
  };
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[0].motor].age_ns = 300'000'000ULL;
  }
  bool stale_rejected = false;
  try {
    runtime.set_joint_pv(forward, 2, 1.0f);
  } catch (const std::runtime_error&) {
    stale_rejected = true;
  }
  require(stale_rejected,
          "ordinary PV position requires complete fresh feedback initially");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[0].motor].age_ns = 0;
    driver.pv_history.clear();
  }

  runtime.set_joint_pv(forward, 2, 1.0f);
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.pv_history.size() >= 105;
          }, 1500ms),
          "ordinary PV position emits control-rate references");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    const auto first_moving = std::find_if(
        driver.pv_history.begin(), driver.pv_history.end(),
        [](const std::vector<ArticorePosVelCommand>& frame) {
          return frame[0].target_position > 0.0f;
        });
    require(first_moving != driver.pv_history.end() &&
                first_moving->at(0).target_position <= 0.00251f &&
                first_moving->at(1).target_position <= 1.00251f &&
                first_moving->at(0).velocity_limit == 1.0f,
            "ordinary PV starts from feedback with one shared reference speed");
    const auto available = static_cast<std::size_t>(
        driver.pv_history.end() - first_moving);
    require(available >= 100 &&
                std::abs(first_moving[99][0].target_position - 0.25f) < 0.003f,
            "ordinary PV advances about 0.25 rad in one hundred 400 Hz cycles");
  }
  require(runtime.health().state == ARTICORE_RUNNING,
          "one-shot ordinary PV remains active beyond the watchdog");

  ArticoreJointPvTarget reverse[] = {
      {sizeof(ArticoreJointPvTarget), motors[0].motor, -1.0f},
      {sizeof(ArticoreJointPvTarget), motors[1].motor, 0.0f},
  };
  runtime.set_joint_pv(reverse, 2, 1.0f);
  float before_reverse = 0.0f;
  std::size_t reverse_baseline = 0;
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    before_reverse = driver.pv_history.back()[0].target_position;
    reverse_baseline = driver.pv_history.size();
  }
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.pv_history.size() > reverse_baseline;
          }, 50ms),
          "ordinary PV reversal takes effect on the next native cycle");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(std::abs((before_reverse -
                      driver.pv_history[reverse_baseline][0].target_position) -
                     0.0025f) < 0.0002f,
            "ordinary PV discards the old endpoint and preserves current q");
  }

  runtime.set_joint_pv(reverse, 2, 2.0f);
  float before_speed = 0.0f;
  std::size_t speed_baseline = 0;
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    before_speed = driver.pv_history.back()[0].target_position;
    speed_baseline = driver.pv_history.size();
  }
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.pv_history.size() > speed_baseline;
          }, 50ms),
          "ordinary PV speed update is visible on the next native cycle");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    const auto& sent = driver.pv_history[speed_baseline];
    require(std::abs((before_speed - sent[0].target_position) - 0.005f) <
                    0.0002f &&
                sent[0].velocity_limit == 2.0f &&
                sent[1].velocity_limit == 2.0f,
            "ordinary PV atomically applies one 2 rad/s speed to both arms");
  }

  ArticorePosVelCommand raw[] = {
      {motors[0].motor, 0.75f, 0.7f},
      {motors[1].motor, 1.25f, 0.8f},
  };
  runtime.submit_pos_vel_ex(raw, 2, ARTICORE_COMMAND_STREAMING);
  std::size_t raw_baseline = 0;
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    raw_baseline = driver.pv_history.size();
  }
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.pv_history.size() > raw_baseline;
          }, 50ms),
          "raw PV replaces ordinary PV on the next native cycle");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    const auto& sent = driver.pv_history[raw_baseline];
    require(sent[0].target_position == raw[0].target_position &&
                sent[0].velocity_limit == raw[0].velocity_limit &&
                sent[1].target_position == raw[1].target_position &&
                sent[1].velocity_limit == raw[1].velocity_limit,
            "raw PV q and velocity limit remain direct and un-ramped");
  }
}

void test_ordinary_mit_position_reversal_and_speed_update_are_continuous() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.command_timeout_ms = 30;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  const auto configured = joint_configs(motors);
  runtime.configure_joints(configured.data(),
                           static_cast<uint32_t>(configured.size()));
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);

  ArticoreJointMitTarget forward[] = {
      {sizeof(ArticoreJointMitTarget), motors[0].motor, 1.0f},
      {sizeof(ArticoreJointMitTarget), motors[1].motor, 2.0f},
  };
  runtime.set_joint_mit(forward, 2, 1.0f);
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.arm_mit_history.size() >= 100;
          }, 500ms),
          "forward ordinary MIT references reach the reversal point");

  ArticoreJointMitTarget reverse[] = {
      {sizeof(ArticoreJointMitTarget), motors[0].motor, -1.0f},
      {sizeof(ArticoreJointMitTarget), motors[1].motor, 0.0f},
  };
  float before_reverse = 0.0f;
  std::size_t reverse_baseline = 0;
  runtime.set_joint_mit(reverse, 2, 1.0f);
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    before_reverse = driver.arm_mit_history.back()[0].target_position;
    reverse_baseline = driver.arm_mit_history.size();
  }
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.arm_mit_history.size() > reverse_baseline;
          }, 50ms),
          "reversed ordinary MIT target is sent on the next control cycle");
  float after_reverse = 0.0f;
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    after_reverse = driver.arm_mit_history[reverse_baseline][0].target_position;
  }
  require(std::abs((before_reverse - after_reverse) - 0.0025f) < 0.0002f,
          "target reversal continues from current reference at 1 rad/s");

  std::size_t speed_baseline = 0;
  float before_speed_change = 0.0f;
  runtime.set_joint_mit(reverse, 2, 2.0f);
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    before_speed_change = driver.arm_mit_history.back()[0].target_position;
    speed_baseline = driver.arm_mit_history.size();
  }
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.arm_mit_history.size() > speed_baseline;
          }, 50ms),
          "ordinary MIT speed update takes effect on the next cycle");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    const auto& next = driver.arm_mit_history[speed_baseline];
    require(std::abs((before_speed_change - next[0].target_position) -
                     0.005f) < 0.0002f &&
                std::abs((driver.arm_mit_history[speed_baseline - 1][1]
                              .target_position -
                          next[1].target_position) - 0.005f) < 0.0002f,
            "shared 2 rad/s update is atomic for both arm sides");
  }

  bool excessive_speed_rejected = false;
  try {
    runtime.set_joint_mit(reverse, 2, 6.0f);
  } catch (const std::invalid_argument&) {
    excessive_speed_rejected = true;
  }
  require(excessive_speed_rejected,
          "one unsafe shared velocity rejects the complete arm batch");
}

void test_raw_mit_targets_remain_direct_after_ordinary_position_control() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.command_timeout_ms = 200;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  const auto configured = joint_configs(motors);
  runtime.configure_joints(configured.data(),
                           static_cast<uint32_t>(configured.size()));
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);

  ArticoreJointMitTarget ordinary[] = {
      {sizeof(ArticoreJointMitTarget), motors[0].motor, 1.0f},
      {sizeof(ArticoreJointMitTarget), motors[1].motor, 2.0f},
  };
  runtime.set_joint_mit(ordinary, 2, 1.0f);
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return !driver.arm_mit_history.empty();
          }, 50ms),
          "ordinary MIT position starts before raw MIT replacement");

  ArticoreMitCommand raw[] = {
      {motors[0].motor, 1.0f, 0.3f, 11.0f, 1.0f, 0.2f},
      {motors[1].motor, 1.25f, -0.4f, 12.0f, 1.5f, -0.3f},
  };
  runtime.submit_mit_ex(raw, 2, ARTICORE_COMMAND_STREAMING);
  std::size_t baseline = 0;
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    baseline = driver.arm_mit_history.size();
  }
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.arm_mit_history.size() > baseline;
          }, 50ms),
          "raw MIT replaces ordinary position control on the next cycle");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    const auto& sent = driver.arm_mit_history[baseline];
    require(sent[0].target_position == raw[0].target_position &&
                sent[0].target_velocity == raw[0].target_velocity &&
                sent[1].target_position == raw[1].target_position &&
                sent[1].target_velocity == raw[1].target_velocity &&
                sent[1].stiffness == raw[1].stiffness &&
                sent[1].damping == raw[1].damping &&
                sent[1].feedforward_torque == raw[1].feedforward_torque &&
                sent[0].stiffness < raw[0].stiffness &&
                std::abs(sent[0].stiffness / raw[0].stiffness -
                         sent[0].damping / raw[0].damping) < 1e-6f &&
                std::abs(sent[0].stiffness / raw[0].stiffness -
                         sent[0].feedforward_torque /
                             raw[0].feedforward_torque) < 1e-6f,
            "raw MIT q/dq remains direct while the per-cycle limiter scales "
            "Kp/Kd/tau together only when required");
  }
}

void test_ordinary_mit_position_reinitializes_after_reenable() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  articore::SafetyRuntime runtime(
      config(), api(), reinterpret_cast<void*>(0x100),
      g_left_controller, g_right_controller, motors, enable_all, enable_motor);
  const auto configured = joint_configs(motors);
  runtime.configure_joints(configured.data(),
                           static_cast<uint32_t>(configured.size()));
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);

  ArticoreJointMitTarget old_session[] = {
      {sizeof(ArticoreJointMitTarget), motors[0].motor, 1.0f},
      {sizeof(ArticoreJointMitTarget), motors[1].motor, 2.0f},
  };
  runtime.set_joint_mit(old_session, 2, 1.0f);
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return !driver.arm_mit_history.empty() &&
                driver.arm_mit_history.back()[0].target_position > 0.0f;
          }, 50ms),
          "old ordinary MIT session advances before disable");
  runtime.disable();
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[0].motor].position = 0.4f;
    driver.motors[motors[1].motor].position = -0.4f;
    driver.arm_mit_history.clear();
  }
  runtime.enable(ARTICORE_MODE_MIT);

  ArticoreJointMitTarget new_session[] = {
      {sizeof(ArticoreJointMitTarget), motors[0].motor, 1.0f},
      {sizeof(ArticoreJointMitTarget), motors[1].motor, 0.0f},
  };
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.arm_mit_history.clear();
  }
  runtime.set_joint_mit(new_session, 2, 1.0f);
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return std::any_of(
                driver.arm_mit_history.begin(), driver.arm_mit_history.end(),
                [](const std::vector<ArticoreMitCommand>& frame) {
                  return frame[0].target_position > 0.4f;
                });
          }, 50ms),
          "new ordinary MIT session starts from new feedback position");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    const auto first = std::find_if(
        driver.arm_mit_history.begin(), driver.arm_mit_history.end(),
        [](const std::vector<ArticoreMitCommand>& frame) {
          return frame[0].target_position > 0.4f;
        });
    require(first != driver.arm_mit_history.end() &&
                std::abs(first->at(0).target_position - 0.4025f) < 0.0002f &&
                std::abs(first->at(1).target_position - (-0.3975f)) < 0.0002f,
            "reenable discards the old reference and reinitializes both arms "
            "from complete fresh feedback");
  }
}

void test_deadline_skips_missed_periods_and_reenable_seeds_feedback() {
  const auto base = std::chrono::steady_clock::time_point{1s};
  auto deadline = base;
  articore::detail::advance_periodic_deadline(deadline, 2ms, base + 10ms);
  require(deadline == base + 12ms,
          "a delayed cycle advances directly beyond all missed periods");
  articore::detail::advance_periodic_deadline(deadline, 2ms, base + 11ms);
  require(deadline == base + 12ms,
          "a future absolute deadline is not accumulated or moved early");

  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  ArticorePosVelCommand command[] = {
      {motors[0].motor, 0.4f, 1.0f}, {motors[1].motor, 0.6f, 1.0f}};
  runtime.submit_pos_vel(command, 2);
  require(wait_for([&] { return runtime.health().state == ARTICORE_RUNNING; }),
          "periodic sender accepts the direct target");

  runtime.disable();
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[0].motor].status = 1;
    driver.motors[motors[1].motor].status = 1;
    driver.motors[motors[2].motor].status = 1;
    driver.motors[motors[0].motor].position = -0.6f;
    driver.motors[motors[1].motor].position = 0.35f;
  }
  runtime.enable(ARTICORE_MODE_PV);
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.last_pv.size() == 2 &&
                   std::abs(driver.last_pv[0].target_position + 0.6f) < 1e-6f &&
                   std::abs(driver.last_pv[1].target_position - 0.35f) < 1e-6f;
          }),
          "re-enable seeds the target from current feedback, not an old command");
}

articore::NativeTrajectoryRequest trajectory_request(
    const std::vector<ArticoreMotorDescriptor>& motors,
    ArticoreControlMode mode, double duration_s = 0.4) {
  articore::NativeTrajectoryRequest request;
  request.mode = mode;
  for (const auto& motor : motors) {
    if (motor.is_gripper) continue;
    articore::NativeTrajectoryJoint joint;
    joint.motor = motor.motor;
    joint.lower_position = -2.0f;
    joint.upper_position = 2.0f;
    joint.velocity_limit = 5.0f;
    joint.acceleration_limit = 20.0f;
    joint.torque_limit = 10.0f;
    joint.mit_kp = 20.0f;
    joint.mit_kd = 3.0f;
    joint.mit_feedforward_torque = 0.25f;
    joint.pv_velocity_limit = 2.0f;
    request.joints.push_back(joint);
  }
  articore::NativeTrajectoryWaypoint start;
  start.time_s = 0.0;
  start.positions = {0.0f, 1.0f};
  start.velocities.resize(2);
  start.accelerations.resize(2);
  articore::NativeTrajectoryWaypoint end = start;
  end.time_s = duration_s;
  end.positions = {0.2f, 0.8f};
  request.waypoints = {start, end};
  return request;
}

void test_native_quintic_trajectory_executes_at_worker_rate() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  articore::SafetyRuntime runtime(
      cfg, api(), reinterpret_cast<void*>(0x100), g_left_controller,
      g_right_controller, motors, nullptr, nullptr, false, {}, {}, 500);
  auto configured = joint_configs(motors);
  runtime.configure_joints(configured.data(), configured.size());
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);

  std::size_t baseline_frames = 0;
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    baseline_frames = driver.pv_history.size();
  }
  const auto id = runtime.start_trajectory(
      trajectory_request(motors, ARTICORE_MODE_PV));
  require(id != 0, "native trajectory receives a stable id");
  require(wait_for([&] {
            return runtime.trajectory_status().state ==
                ARTICORE_TRAJECTORY_COMPLETED;
          }, 1000ms), "native trajectory completes asynchronously");
  const auto status = runtime.trajectory_status();
  require(status.trajectory_id == id && status.waypoint_count == 2 &&
              status.active_segment == 0 &&
              std::abs(status.progress - 1.0f) < 1e-6f &&
              std::abs(status.elapsed_s - 0.4) < 1e-6,
          "trajectory status reports complete native execution");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    const auto trajectory_frames = driver.pv_history.size() - baseline_frames;
    require(trajectory_frames >= 150 && trajectory_frames <= 260,
            "500 Hz worker executes the 0.4 second trajectory near 500 Hz");
    float previous = -1.0f;
    for (const auto& frame : driver.pv_history) {
      if (frame.size() != 2) continue;
      require(frame[0].target_position + 1e-5f >= previous &&
                  frame[0].target_position >= -1e-5f &&
                  frame[0].target_position <= 0.20001f,
              "quintic PV samples remain continuous and within segment bounds");
      previous = frame[0].target_position;
    }
    require(std::abs(driver.last_pv[0].target_position - 0.2f) < 1e-5f &&
                std::abs(driver.last_pv[1].target_position - 0.8f) < 1e-5f,
            "completed trajectory keeps the final complete-arm hold");
  }
  runtime.disable();
}

void test_native_trajectory_uses_raw_mit_and_cancel_is_idempotent() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  articore::SafetyRuntime runtime(
      cfg, api(), reinterpret_cast<void*>(0x100), g_left_controller,
      g_right_controller, motors, nullptr, nullptr, false, {}, {}, 500);
  auto configured = joint_configs(motors);
  runtime.configure_joints(configured.data(), configured.size());
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);

  runtime.start_trajectory(
      trajectory_request(motors, ARTICORE_MODE_MIT, 1.0));
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.arm_mit_history.size() >= 10;
          }), "MIT trajectory is emitted through the native raw frame path");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    const auto& frame = driver.arm_mit_history.back();
    require(frame.size() == 2 && frame[0].stiffness == 20.0f &&
                frame[0].damping == 3.0f &&
                frame[0].feedforward_torque == 0.25f &&
                frame[0].target_velocity > 0.0f,
            "MIT samples carry explicit dq, Kp, Kd and feedforward torque");
  }
  runtime.cancel_trajectory();
  runtime.cancel_trajectory();
  require(runtime.trajectory_status().state == ARTICORE_TRAJECTORY_CANCELLED,
          "trajectory cancellation is idempotent");
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return !driver.last_arm_mit.empty() &&
                driver.last_arm_mit[0].target_velocity == 0.0f &&
                driver.last_arm_mit[0].feedforward_torque == 0.0f;
          }), "MIT cancellation changes the in-flight sample into a stationary hold");
  runtime.disable();
}

void test_native_trajectory_checks_segment_extrema_and_partial_power() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  articore::SafetyRuntime runtime(
      cfg, api(), reinterpret_cast<void*>(0x100), g_left_controller,
      g_right_controller, motors, nullptr, enable_motor, false, {}, {}, 500);
  auto configured = joint_configs(motors);
  runtime.configure_joints(configured.data(), configured.size());
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);

  auto invalid = trajectory_request(motors, ARTICORE_MODE_PV, 1.0);
  invalid.joints[0].lower_position = -0.05f;
  invalid.joints[0].upper_position = 0.05f;
  invalid.waypoints[1].positions[0] = 0.0f;
  invalid.waypoints[0].velocity_valid_mask = 1U;
  invalid.waypoints[1].velocity_valid_mask = 1U;
  invalid.waypoints[0].velocities[0] = 1.0f;
  invalid.waypoints[1].velocities[0] = -1.0f;
  require_throws(
      [&] { runtime.start_trajectory(std::move(invalid)); },
      "inside the segment",
      "trajectory validation checks polynomial extrema, not only waypoints");

  const auto report =
      runtime.set_motor_power_batch({"r-joint1"}, false);
  require(report.success &&
              runtime.health().state == ARTICORE_PARTIALLY_ENABLED,
          "one intentionally disabled motor enters PARTIALLY_ENABLED");
  runtime.start_trajectory(
      trajectory_request(motors, ARTICORE_MODE_PV, 0.3));
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return std::any_of(
                driver.pv_history.begin(), driver.pv_history.end(),
                [](const auto& frame) {
                  return frame.size() == 1 &&
                      frame[0].motor == reinterpret_cast<void*>(0x201);
                });
          }), "partial trajectory sends only the intentionally enabled motor");
  runtime.disable();
  require(runtime.trajectory_status().state == ARTICORE_TRAJECTORY_CANCELLED,
          "disable terminates an active trajectory");
}

void test_gravity_compensation_is_an_exclusive_hand_guiding_mode() {
  FakeDriver driver;
  g_driver = &driver;
  driver.feedback_expected = 7;
  driver.feedback_received = 7;
  std::vector<ArticoreMotorDescriptor> motors(7);
  for (std::size_t index = 0; index < motors.size(); ++index) {
    auto& motor = motors[index];
    motor.motor = reinterpret_cast<void*>(0x201 + index);
    motor.side = 0;
    motor.is_gripper = 0;
    const auto name = std::string("l-joint") + std::to_string(index + 1);
    std::strncpy(motor.name, name.c_str(), sizeof(motor.name) - 1);
    motor.safe_kp = 5.0f;
    motor.safe_kd = 0.5f;
    motor.lower_position = -3.0f;
    motor.upper_position = 3.0f;
    driver.motors[motor.motor] = FakeMotor{
        1, 0.1f * static_cast<float>(index), 0.0f, 0.0f, 0, true};
  }
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  articore::SafetyRuntime runtime(
      cfg, api(), reinterpret_cast<void*>(0x100), g_left_controller, nullptr,
      motors);
  auto configs = joint_configs(motors);
  for (auto& item : configs) item.torque_limit = 100.0f;
  runtime.configure_joints(configs.data(), static_cast<uint32_t>(configs.size()));
  ArticoreGravityProductBinding binding{};
  binding.struct_size = sizeof(binding);
  binding.runtime_side = 0;
  binding.robot_side = ARTICORE_ROBOT_LEFT;
  std::strncpy(binding.product_id, "yunyi_v1_0",
               sizeof(binding.product_id) - 1);
  runtime.configure_gravity_products(&binding, 1);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);

  ArticoreGravityCompensationConfig gravity_config{};
  gravity_config.struct_size = sizeof(gravity_config);
  gravity_config.transition_ms = 1;
  runtime.start_gravity_compensation(&gravity_config);
  require(wait_for([&] {
            return runtime.gravity_compensation_status().phase ==
                ARTICORE_GRAVITY_ACTIVE;
          }), "gravity compensation reaches ACTIVE");
  const auto active = runtime.gravity_compensation_status();
  require(active.active && active.joint_count == 7 && active.control_cycles > 0,
          "gravity status reports the active seven-axis controller");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(!driver.group_mit_history.empty() &&
                driver.group_mit_history.back().size() == 7,
            "gravity mode sends one complete seven-axis MIT batch");
    const auto& commands = driver.group_mit_history.back();
    require(std::all_of(commands.begin(), commands.end(), [](const auto& command) {
              return command.stiffness == 0.0f && command.damping == 0.0f;
            }), "active hand guiding uses zero stiffness and damping");
    require(std::any_of(commands.begin(), commands.end(), [](const auto& command) {
              return std::abs(command.feedforward_torque) > 1e-5f;
            }), "active hand guiding sends posture-dependent gravity torque");
  }
  ArticoreMitCommand rejected[7]{};
  for (std::size_t index = 0; index < motors.size(); ++index) {
    rejected[index].motor = motors[index].motor;
  }
  require_throws([&] { runtime.submit_mit(rejected, 7); },
                 "owned by active gravity compensation",
                 "gravity mode exclusively owns arm output");

  runtime.stop_gravity_compensation();
  require(wait_for([&] {
            return runtime.gravity_compensation_status().phase ==
                ARTICORE_GRAVITY_INACTIVE;
          }), "stopping gravity compensation returns to MIT hold");
  runtime.disable();
}

}  // namespace

int main() {
  const char* current_test = "startup";
  try {
#define RUN_TEST(test) \
    current_test = #test; \
    test()
    RUN_TEST(test_connect_is_a_complete_feedback_barrier_and_ready_refreshes_cache);
    RUN_TEST(test_runtime_motor_power_supports_single_and_whole_product_queries);
    RUN_TEST(test_motor_power_batch_rolls_back_failed_enable_atomically);
    RUN_TEST(test_motor_power_batch_latches_fault_when_rollback_is_unconfirmed);
    RUN_TEST(test_partial_mit_filters_disabled_motors_before_torque_validation);
    RUN_TEST(test_runtime_maintenance_keeps_ready_and_reports_partial_failure);
    RUN_TEST(test_disconnect_is_terminal_and_idempotent);
    RUN_TEST(test_connect_failure_names_missing_installed_motor_and_can_id);
    RUN_TEST(test_connect_report_classifies_zero_feedback_and_transport_failures);
    RUN_TEST(test_transient_arm_send_failures_preserve_active_command);
    RUN_TEST(test_pv_watchdog_safe_hold_and_fault);
    RUN_TEST(test_mit_hold_removes_motion_and_feedforward);
    RUN_TEST(test_safe_hold_rejects_stale_current_position);
    RUN_TEST(test_gripper_hold_retreats_once_on_overload);
    RUN_TEST(test_gripper_stall_switches_to_contact_hold_target);
    RUN_TEST(test_gripper_torque_spike_does_not_trigger_contact);
    RUN_TEST(test_gripper_command_profiles_and_bidirectional_ramp);
    RUN_TEST(test_gripper_only_command_satisfies_enable_grace_without_masking_arm_watchdog);
    RUN_TEST(test_gripper_force_profiles_are_product_configuration);
    RUN_TEST(test_builtin_yunyi_gripper_profile_owns_product_calibration);
    RUN_TEST(test_builtin_gripper_binding_is_complete_and_optional);
    RUN_TEST(test_legacy_three_level_gripper_profiles_expand_to_ten_levels);
    RUN_TEST(test_estop_disables_complete_product_and_is_idempotent);
    RUN_TEST(test_estop_can_disable_gripper_by_product_policy);
    RUN_TEST(test_missing_feedback_degrades_then_safe_stops_and_resynchronizes);
    RUN_TEST(test_single_gripper_feedback_miss_reuses_current_output);
    RUN_TEST(test_feedback_measurements_do_not_reuse_command_limits);
    RUN_TEST(test_feedback_seed_ignores_command_limits);
    RUN_TEST(test_feedback_fault_diagnostics_include_identity_value_and_threshold);
    RUN_TEST(test_single_stale_feedback_sample_keeps_persistent_position_control);
    RUN_TEST(test_ordinary_position_can_recover_from_feedback_outside_hard_limit);
    RUN_TEST(test_disable_does_not_stop_after_one_side_fails);
    RUN_TEST(test_disable_uses_structured_feedback_report);
    RUN_TEST(test_disable_retries_one_full_feedback_deadline);
    RUN_TEST(test_disable_barrier_and_targeted_motor_retry);
    RUN_TEST(test_disable_waits_for_inflight_batch_and_rejects_new_commands);
    RUN_TEST(test_raw_mit_publish_does_not_wait_for_inflight_transport_send);
    RUN_TEST(test_raw_mit_torque_limit_recomputes_on_every_native_cycle);
    RUN_TEST(test_raw_mit_torque_limit_keeps_sending_with_stale_feedback);
    RUN_TEST(test_close_reuses_checked_disable_transaction);
    RUN_TEST(test_close_stops_worker_after_unconfirmed_disable);
    RUN_TEST(test_transport_disconnect_holds_the_connected_side);
    RUN_TEST(test_reported_feedback_failure_is_diagnostic_only);
    RUN_TEST(test_enable_grace_enters_safe_stop);
    RUN_TEST(test_atomic_enable_starts_hold_and_confirms_both_sides);
    RUN_TEST(test_atomic_enable_retries_one_disabled_motor_once);
    RUN_TEST(test_gripper_control_waits_for_atomic_enable_confirmation);
    RUN_TEST(test_atomic_enable_failure_rolls_back_and_fault_disable_is_allowed);
    RUN_TEST(test_repeated_runtime_lifecycle);
    RUN_TEST(test_native_scheduler_selects_rate_from_transport_capabilities);
    RUN_TEST(test_single_side_runtime_and_gripper);
    RUN_TEST(test_normal_gripper_uses_arm_control_rate);
    RUN_TEST(test_gripper_health_reads_live_feedback_age);
    RUN_TEST(test_motor_presence_is_fixed_and_fault_aware);
    RUN_TEST(test_latest_value_mailbox_drops_superseded_targets);
    RUN_TEST(test_latest_value_mailbox_stays_bounded_under_fast_producer);
    RUN_TEST(test_persistent_setpoints_outlive_watchdog_but_streaming_still_times_out);
    RUN_TEST(test_persistent_mit_rejects_unbounded_motion_terms);
    RUN_TEST(test_ordinary_mit_position_uses_constant_reference_speed);
    RUN_TEST(test_ordinary_speed_percent_scales_and_zero_pauses);
    RUN_TEST(test_ordinary_pv_position_latest_value_and_raw_pv_remains_direct);
    RUN_TEST(test_ordinary_mit_position_reversal_and_speed_update_are_continuous);
    RUN_TEST(test_raw_mit_targets_remain_direct_after_ordinary_position_control);
    RUN_TEST(test_ordinary_mit_position_reinitializes_after_reenable);
    RUN_TEST(test_deadline_skips_missed_periods_and_reenable_seeds_feedback);
    RUN_TEST(test_native_quintic_trajectory_executes_at_worker_rate);
    RUN_TEST(test_native_trajectory_uses_raw_mit_and_cancel_is_idempotent);
    RUN_TEST(test_native_trajectory_checks_segment_extrema_and_partial_power);
    RUN_TEST(test_gravity_compensation_is_an_exclusive_hand_guiding_mode);
#undef RUN_TEST
    std::cout << "Articore runtime tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Articore runtime test failed in " << current_test << ": "
              << error.what() << '\n';
    return 1;
  }
}

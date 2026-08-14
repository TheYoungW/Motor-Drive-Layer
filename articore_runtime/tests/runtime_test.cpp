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
  std::vector<std::vector<ArticoreMitCommand>> arm_mit_history;
  uint32_t pv_sends = 0;
  uint32_t mit_sends = 0;
  uint32_t disable_calls[2]{};
  uint32_t enable_calls[2]{};
  uint32_t motor_enable_calls = 0;
  uint32_t feedback_requests = 0;
  uint32_t feedback_stats_calls = 0;
  int32_t feedback_code = 0;
  uint32_t feedback_expected = 2;
  uint32_t feedback_received = 2;
  std::vector<uint32_t> feedback_missing_ids;
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
  if (count > 0) {
    const bool left = commands[0].motor == reinterpret_cast<void*>(0x201);
    if (g_driver->fail_send_side[left ? 0 : 1]) return -1;
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
  g_driver->last_mit.assign(commands, commands + count);
  g_driver->mit_history.emplace_back(commands, commands + count);
  g_driver->events.emplace_back("send-mit");
  if (count == 2) {
    g_driver->last_arm_mit.assign(commands, commands + count);
    g_driver->arm_mit_history.emplace_back(commands, commands + count);
    if (g_driver->emulate_arm_feedback) {
      for (uint32_t index = 0; index < count; ++index) {
        auto& motor = g_driver->motors[commands[index].motor];
        motor.position = commands[index].target_position;
        motor.velocity = commands[index].target_velocity;
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

int32_t request_feedback(void*, uint32_t timeout_ms,
                         ArticoreFeedbackReport* report,
                         uint32_t* missing_ids, uint32_t missing_capacity) {
  std::unique_lock<std::mutex> lock(g_driver->mutex);
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
  if (report) {
    report->struct_size = sizeof(*report);
    report->timeout_ms = timeout_ms;
    report->expected_count = g_driver->feedback_expected;
    report->received_count = injected_timeout ? 0 : g_driver->feedback_received;
    report->missing_count = injected_timeout
        ? g_driver->feedback_expected
        : static_cast<uint32_t>(g_driver->feedback_missing_ids.size());
    const auto copied = std::min<std::size_t>(
        g_driver->feedback_missing_ids.size(), missing_capacity);
    for (std::size_t i = 0; i < copied; ++i) {
      missing_ids[i] = g_driver->feedback_missing_ids[i];
    }
  }
  const auto result = injected_timeout ? 3 : g_driver->feedback_code;
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
  value.control_hz = 400;
  value.command_timeout_ms = 30;
  value.enable_grace_ms = 60;
  value.safe_hold_hz = 100;
  value.feedback_check_hz = 100;
  value.feedback_failure_threshold = 3;
  value.feedback_max_age_ms = 200;
  value.safe_hold_failure_threshold = 1;
  value.disable_feedback_timeout_ms = 20;
  value.safe_pv_velocity_limit = 0.15f;
  value.gripper_control_hz = 100;
  value.gripper_fault_action = ARTICORE_GRIPPER_FAULT_DISABLE;
  return value;
}

ArticoreMotorApi api() {
  return ArticoreMotorApi{send_pv, send_mit, disable_all, request_feedback,
                          get_state, get_feedback_stats, last_error,
                          get_transport_health, disable_motor};
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
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) return true;
    std::this_thread::sleep_for(2ms);
  }
  return predicate();
}

void test_pv_watchdog_safe_hold_and_fault() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  articore::SafetyRuntime runtime(config(), api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  require(runtime.health().state == ARTICORE_READY, "connect enters READY");
  runtime.enable(ARTICORE_MODE_PV);
  require(runtime.health().state == ARTICORE_ENABLED, "enable enters ENABLED");

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
  require(wait_for([&] { return runtime.health().state == ARTICORE_SAFE_HOLD; }),
          "command timeout enters SAFE_HOLD");
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
    require(driver.feedback_requests == 0,
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
  require(wait_for([&] { return runtime.health().state == ARTICORE_SAFE_HOLD; }),
          "MIT watchdog enters SAFE_HOLD");
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
  require(std::string(health.fault_reason).find("current-position hold unavailable") !=
              std::string::npos,
          "stale current-position fault explains why safe hold was rejected");
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
  require(wait_for([&] { return runtime.health().state == ARTICORE_SAFE_HOLD; }),
          "gripper overload test reaches SAFE_HOLD");
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
  cfg.gripper_control_hz = 500;
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
  cfg.gripper_control_hz = 500;
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
  cfg.control_hz = 500;
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
            return runtime.health().state == ARTICORE_SAFE_HOLD;
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

void test_estop_obeys_configured_gripper_hold_policy() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.gripper_fault_action = ARTICORE_GRIPPER_FAULT_HOLD;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  runtime.estop("test estop");

  const auto fault = runtime.health();
  require(fault.state == ARTICORE_FAULT && fault.disable_confirmed == 0 &&
              fault.safe_holding == 1,
          "estop latches FAULT while the configured gripper hold remains active");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.motors[motors[0].motor].status == 0 &&
                driver.motors[motors[1].motor].status == 0 &&
                driver.motors[motors[2].motor].status == 1,
            "estop disables arm joints but preserves the configured gripper hold");
  }
  runtime.disable();
  require(runtime.health().state == ARTICORE_FAULT &&
              runtime.health().disable_confirmed == 1,
          "explicit disable also disables the gripper without clearing FAULT");
  runtime.recover();
  require(runtime.health().state == ARTICORE_READY &&
              runtime.health().disable_confirmed == 1,
          "recover clears the latch only after physical disable is confirmed");
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
  runtime.estop("test torque-off estop");

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

void test_feedback_fault_uses_protective_hold_without_linked_disable() {
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
    driver.motors[motors[0].motor].has_feedback = false;
  }
  require(wait_for([&] { return runtime.health().state == ARTICORE_FAULT; }),
          "continuous missing feedback stops normal motion and latches FAULT");
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return std::any_of(
                driver.pv_history.begin(), driver.pv_history.end(),
                [&](const auto& batch) {
                  return batch.size() == 1 &&
                         batch[0].motor == motors[1].motor;
                });
          }),
          "healthy joint continues receiving a protective hold target");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.disable_calls[0] == 0 && driver.disable_calls[1] == 0,
            "feedback fault does not automatically disable either side");
  }
  const auto health = runtime.health();
  require(health.safe_holding == 1 && health.disable_confirmed == 0,
          "feedback fault reports active protective holding");
}

void test_single_gripper_feedback_miss_reuses_current_output() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  cfg.feedback_check_hz = 1;
  cfg.gripper_control_hz = 100;
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
    // Calls 1/2 establish the two-channel queue marker. During the initial
    // post-disable confirmation, make only the right gripper appear enabled.
    driver.feedback_enable_on_call[3] = motors[2].motor;
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

void test_close_refuses_to_disconnect_after_unconfirmed_disable() {
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
  require(failed && runtime.health().state == ARTICORE_FAULT &&
              runtime.health().disable_confirmed == 0 && report.success == 0 &&
              report.missing_count > 0,
          "close reports structured missing motors and keeps transports usable");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.feedback_code = 0;
    driver.feedback_received = 2;
    driver.feedback_missing_ids.clear();
  }
  runtime.close();
  require(runtime.health().state == ARTICORE_DISCONNECTED,
          "a later confirmed close can finish the retained Runtime");
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

void test_fault_hold_failure_isolated_per_channel() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  cfg.safe_hold_hz = 200;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  ArticorePosVelCommand commands[] = {
      {motors[0].motor, 0.25f, 1.0f}, {motors[1].motor, 0.75f, 1.0f}};
  runtime.submit_pos_vel_ex(
      commands, 2, ARTICORE_COMMAND_HOLD_UNTIL_REPLACED);
  require(wait_for([&] { return runtime.health().state == ARTICORE_RUNNING; }),
          "per-channel hold test reaches RUNNING");

  runtime.report_feedback_failure(0, "injected miss 1");
  runtime.report_feedback_failure(0, "injected miss 2");
  runtime.report_feedback_failure(0, "injected miss 3");
  require(runtime.health().state == ARTICORE_SAFE_HOLD,
          "consecutive feedback misses enter protective SAFE_HOLD");
  runtime.report_feedback_failure(0, "injected miss during hold");
  require(runtime.health().state == ARTICORE_FAULT &&
              runtime.health().safe_holding == 1,
          "feedback failure during hold latches FAULT but preserves holding");

  std::size_t history_before = 0;
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    history_before = driver.pv_history.size();
    driver.fail_send_side[0] = true;
  }
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            for (std::size_t i = history_before; i < driver.pv_history.size(); ++i) {
              const auto& batch = driver.pv_history[i];
              if (batch.size() == 1 && batch[0].motor == motors[1].motor) {
                return true;
              }
            }
            return false;
          }),
          "right channel keeps receiving hold frames when left hold send fails");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.disable_calls[0] == 0 && driver.disable_calls[1] == 0,
            "one failed hold channel does not disable the controllable channel");
  }
}

void test_enable_grace_and_fault_latch() {
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
  require(wait_for([&] { return runtime.health().state == ARTICORE_FAULT; }),
          "missing first command faults after enable grace");
  ArticorePosVelCommand commands[] = {
      {motors[0].motor, 0.0f, 1.0f}, {motors[1].motor, 0.0f, 1.0f}};
  bool rejected = false;
  try {
    runtime.submit_pos_vel(commands, 2);
  } catch (const std::exception&) {
    rejected = true;
  }
  require(rejected && runtime.health().state == ARTICORE_FAULT,
          "ordinary command cannot clear latched FAULT");
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
  for (auto& entry : driver.motors) entry.second.status = 0;
  driver.fail_enable[1] = true;
  articore::SafetyRuntime runtime(
      config(), api(), reinterpret_cast<void*>(0x100), g_left_controller,
      g_right_controller, motors, enable_all, enable_motor);
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

void test_dual_runtime_caps_effective_control_rate_at_400_hz() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.control_hz = 500;
  articore::SafetyRuntime dual(
      cfg, api(), reinterpret_cast<void*>(0x100), g_left_controller,
      g_right_controller, motors);
  require(dual.control_hz() == 400,
          "dual runtime caps a requested 500 Hz rate at the verified 400 Hz envelope");

  std::vector<ArticoreMotorDescriptor> single_motors{motors[0]};
  articore::SafetyRuntime single(
      cfg, api(), reinterpret_cast<void*>(0x101), g_left_controller, nullptr,
      single_motors);
  require(single.control_hz() == 500,
          "single-side runtime preserves an explicitly requested 500 Hz rate");
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
  cfg.control_hz = 500;
  cfg.command_timeout_ms = 500;
  // This legacy field used to down-sample normal gripper output. Keep it at
  // the old product value to prove that normal control now follows control_hz.
  cfg.gripper_control_hz = 100;
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
  ArticoreGripperTarget target{motors[2].motor, 500.0f};
  runtime.set_gripper_openings(&target, 1);

  std::size_t arm_baseline = 0;
  std::size_t gripper_baseline = 0;
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    arm_baseline = driver.arm_mit_history.size();
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
  }
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
  runtime.submit_pos_vel(commands, 2);
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[2].motor].has_feedback = false;
  }
  require(wait_for([&] {
            return runtime.motor_presence("right/gripper") == ARTICORE_FAULTED;
          }),
          "a present motor that loses feedback becomes Faulted, not NotInstalled");
}

void test_latest_value_mailbox_drops_superseded_targets() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.control_hz = 20;
  cfg.command_timeout_ms = 500;
  // This test intentionally stretches the control period to 50 ms so all
  // three submissions land before the next tick. Keep the enable grace well
  // above that period; otherwise a delayed CI runner can fault before the
  // pending mailbox command gets its first send opportunity.
  cfg.enable_grace_ms = 1000;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
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
  cfg.control_hz = 500;
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
    require(wait_for([&] { return runtime.health().state == ARTICORE_SAFE_HOLD; }),
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
    require(wait_for([&] { return runtime.health().state == ARTICORE_SAFE_HOLD; }),
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
          }, 500ms),
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
          }, 1500ms),
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
          }, 500ms),
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

void test_raw_mit_remains_direct_after_ordinary_position_control() {
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
      {motors[0].motor, 0.75f, 0.3f, 11.0f, 1.0f, 0.2f},
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
                sent[0].stiffness == raw[0].stiffness &&
                sent[0].damping == raw[0].damping &&
                sent[0].feedforward_torque == raw[0].feedforward_torque &&
                sent[1].target_position == raw[1].target_position &&
                sent[1].target_velocity == raw[1].target_velocity,
            "raw MIT q/dq/kp/kd/tau remains byte-for-field direct");
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

}  // namespace

int main() {
  const char* current_test = "startup";
  try {
#define RUN_TEST(test) \
    current_test = #test; \
    test()
    RUN_TEST(test_pv_watchdog_safe_hold_and_fault);
    RUN_TEST(test_mit_hold_removes_motion_and_feedforward);
    RUN_TEST(test_safe_hold_rejects_stale_current_position);
    RUN_TEST(test_gripper_hold_retreats_once_on_overload);
    RUN_TEST(test_gripper_stall_switches_to_contact_hold_target);
    RUN_TEST(test_gripper_torque_spike_does_not_trigger_contact);
    RUN_TEST(test_gripper_command_profiles_and_bidirectional_ramp);
    RUN_TEST(test_gripper_only_command_satisfies_enable_grace_without_masking_arm_watchdog);
    RUN_TEST(test_gripper_force_profiles_are_product_configuration);
    RUN_TEST(test_legacy_three_level_gripper_profiles_expand_to_ten_levels);
    RUN_TEST(test_estop_obeys_configured_gripper_hold_policy);
    RUN_TEST(test_estop_can_disable_gripper_by_product_policy);
    RUN_TEST(test_feedback_fault_uses_protective_hold_without_linked_disable);
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
    RUN_TEST(test_close_reuses_checked_disable_transaction);
    RUN_TEST(test_close_refuses_to_disconnect_after_unconfirmed_disable);
    RUN_TEST(test_transport_disconnect_holds_the_connected_side);
    RUN_TEST(test_fault_hold_failure_isolated_per_channel);
    RUN_TEST(test_enable_grace_and_fault_latch);
    RUN_TEST(test_atomic_enable_starts_hold_and_confirms_both_sides);
    RUN_TEST(test_atomic_enable_retries_one_disabled_motor_once);
    RUN_TEST(test_gripper_control_waits_for_atomic_enable_confirmation);
    RUN_TEST(test_atomic_enable_failure_rolls_back_and_fault_disable_is_allowed);
    RUN_TEST(test_repeated_runtime_lifecycle);
    RUN_TEST(test_dual_runtime_caps_effective_control_rate_at_400_hz);
    RUN_TEST(test_single_side_runtime_and_gripper);
    RUN_TEST(test_normal_gripper_uses_arm_control_rate);
    RUN_TEST(test_motor_presence_is_fixed_and_fault_aware);
    RUN_TEST(test_latest_value_mailbox_drops_superseded_targets);
    RUN_TEST(test_latest_value_mailbox_stays_bounded_under_fast_producer);
    RUN_TEST(test_persistent_setpoints_outlive_watchdog_but_streaming_still_times_out);
    RUN_TEST(test_persistent_mit_rejects_unbounded_motion_terms);
    RUN_TEST(test_ordinary_mit_position_uses_constant_reference_speed);
    RUN_TEST(test_ordinary_pv_position_latest_value_and_raw_pv_remains_direct);
    RUN_TEST(test_ordinary_mit_position_reversal_and_speed_update_are_continuous);
    RUN_TEST(test_raw_mit_remains_direct_after_ordinary_position_control);
    RUN_TEST(test_ordinary_mit_position_reinitializes_after_reenable);
    RUN_TEST(test_deadline_skips_missed_periods_and_reenable_seeds_feedback);
#undef RUN_TEST
    std::cout << "Articore runtime tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Articore runtime test failed in " << current_test << ": "
              << error.what() << '\n';
    return 1;
  }
}

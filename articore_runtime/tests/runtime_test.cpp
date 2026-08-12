#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <map>
#include <limits>
#include <mutex>
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
  std::map<void*, FakeMotor> motors;
  std::vector<ArticorePosVelCommand> last_pv;
  std::vector<ArticoreMitCommand> last_mit;
  std::vector<ArticoreMitCommand> last_arm_mit;
  std::vector<std::vector<ArticorePosVelCommand>> pv_history;
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
  bool fail_group = false;
  bool fail_left_disable = false;
  bool fail_enable[2]{};
  bool skip_left_enable_once = false;
  bool transport_connected[2]{true, true};
  bool transport_healthy[2]{true, true};
  std::string error = "injected failure";
};

FakeDriver* g_driver = nullptr;
void* g_left_controller = reinterpret_cast<void*>(0x101);
void* g_right_controller = reinterpret_cast<void*>(0x102);

int32_t send_pv(void*, const ArticorePosVelCommand* commands, uint32_t count) {
  std::lock_guard<std::mutex> lock(g_driver->mutex);
  if (g_driver->fail_group) return -1;
  g_driver->last_pv.assign(commands, commands + count);
  g_driver->pv_history.emplace_back(commands, commands + count);
  ++g_driver->pv_sends;
  return 0;
}

int32_t send_mit(void*, const ArticoreMitCommand* commands, uint32_t count) {
  std::lock_guard<std::mutex> lock(g_driver->mutex);
  if (g_driver->fail_group) return -1;
  g_driver->last_mit.assign(commands, commands + count);
  if (count == 2) {
    g_driver->last_arm_mit.assign(commands, commands + count);
    g_driver->arm_mit_history.emplace_back(commands, commands + count);
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
    entry.second.status = 1;
  }
  if (side == 0) g_driver->skip_left_enable_once = false;
  return 0;
}

int32_t enable_motor(void* handle) {
  std::lock_guard<std::mutex> lock(g_driver->mutex);
  const auto found = g_driver->motors.find(handle);
  if (found == g_driver->motors.end()) return -1;
  ++g_driver->motor_enable_calls;
  found->second.status = 1;
  return 0;
}

int32_t disable_motor(void* handle) {
  std::lock_guard<std::mutex> lock(g_driver->mutex);
  const auto found = g_driver->motors.find(handle);
  if (found == g_driver->motors.end()) return -1;
  const uint8_t side = handle == reinterpret_cast<void*>(0x201) ? 0 : 1;
  ++g_driver->disable_calls[side];
  found->second.status = 0;
  return side == 0 && g_driver->fail_left_disable ? -1 : 0;
}

int32_t request_feedback(void*, uint32_t timeout_ms,
                         ArticoreFeedbackReport* report,
                         uint32_t* missing_ids, uint32_t missing_capacity) {
  std::lock_guard<std::mutex> lock(g_driver->mutex);
  ++g_driver->feedback_requests;
  if (report) {
    report->struct_size = sizeof(*report);
    report->timeout_ms = timeout_ms;
    report->expected_count = g_driver->feedback_expected;
    report->received_count = g_driver->feedback_received;
    report->missing_count = static_cast<uint32_t>(
        g_driver->feedback_missing_ids.size());
    const auto copied = std::min<std::size_t>(
        g_driver->feedback_missing_ids.size(), missing_capacity);
    for (std::size_t i = 0; i < copied; ++i) {
      missing_ids[i] = g_driver->feedback_missing_ids[i];
    }
  }
  return g_driver->feedback_code;
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
  value.control_hz = 500;
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
                   health.disable_confirmed == 1;
          }),
          "FAULT confirms linked disable");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.disable_calls[0] > 0 && driver.disable_calls[1] > 0,
            "FAULT attempts both controllers");
  }
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

void test_fault_policy_can_hold_gripper_until_recovery() {
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
  require(fault.state == ARTICORE_FAULT && fault.disable_confirmed == 0,
          "hold fault policy reports that not every motor is disabled");
  require(fault.grippers[0].control_state == ARTICORE_GRIPPER_HOLDING,
          "fault policy keeps the gripper in low-gain hold");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.motors[motors[0].motor].status == 0 &&
                driver.motors[motors[1].motor].status == 0 &&
                driver.motors[motors[2].motor].status == 1,
            "fault hold disables both arms but preserves the gripper");
  }
  runtime.disable();
  require(runtime.health().state == ARTICORE_FAULT &&
              runtime.health().disable_confirmed == 1,
          "disable in FAULT physically disables the held gripper");
  runtime.recover();
  require(runtime.health().state == ARTICORE_READY &&
              runtime.health().disable_confirmed == 1,
          "recover clears the latch only after physical disable is confirmed");
}

void test_feedback_policy_and_linked_disable() {
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
          "missing feedback prevents current-position hold and enters FAULT");
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.disable_calls[0] > 0 && driver.disable_calls[1] > 0;
          }),
          "feedback fault disables both sides");
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

void test_feedback_seed_and_trajectory_start_ignore_command_limits() {
  FakeDriver driver;
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

  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    driver.motors[motors[0].motor].position = 0.25f;
    driver.motors[motors[0].motor].velocity = 5.1f;
    driver.motors[motors[0].motor].torque = 10.1f;
  }
  ArticoreJointTrajectoryTarget targets[] = {
      {motors[0].motor, 0.3f, 1.0f}, {motors[1].motor, 0.9f, 1.0f}};
  const auto id = runtime.start_joint_trajectory(
      targets, 2, ARTICORE_TRAJECTORY_MIN_JERK);
  const auto completed = runtime.wait_trajectory(id, 500ms);
  require(completed.status == ARTICORE_TRAJECTORY_COMPLETED,
          "trajectory accepts finite fresh start feedback outside command velocity and torque limits");
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

void test_transport_disconnect_faults_and_links_both_sides() {
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
            return driver.disable_calls[0] > 0 && driver.disable_calls[1] > 0;
          }),
          "a disconnected transport enters FAULT and attempts linked disable");
  const auto health = runtime.health();
  require(health.right_transport.connected == 0 &&
              health.right_transport.healthy == 0,
          "structured health identifies the disconnected side");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.disable_calls[0] > 0 && driver.disable_calls[1] > 0,
            "transport disconnect still attempts linked disable on both sides");
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

void test_min_jerk_trajectory_and_direct_preemption() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  auto cfg = config();
  cfg.command_timeout_ms = 500;
  articore::SafetyRuntime runtime(cfg, api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  const auto configured = joint_configs(motors);
  runtime.configure_joints(configured.data(),
                           static_cast<uint32_t>(configured.size()));
  runtime.connect();
  runtime.enable(ARTICORE_MODE_MIT);

  ArticoreJointTrajectoryTarget target[] = {
      {motors[0].motor, 0.1f, 5.0f}, {motors[1].motor, 0.9f, 5.0f}};
  const auto trajectory_id = runtime.start_joint_trajectory(
      target, 2, ARTICORE_TRAJECTORY_MIN_JERK);
  const auto completed = runtime.wait_trajectory(trajectory_id, 500ms);
  require(completed.status == ARTICORE_TRAJECTORY_COMPLETED &&
              completed.duration_ns >= 37'000'000ULL,
          "minimum-jerk trajectory completes with velocity-derived duration");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(!driver.arm_mit_history.empty(), "trajectory emits MIT frames");
    for (const auto& frame : driver.arm_mit_history) {
      for (const auto& command : frame) {
        require(std::abs(command.target_velocity) <= 5.0001f,
                "minimum-jerk interpolation respects velocity limits");
      }
    }
    const auto& endpoint = driver.arm_mit_history.back();
    require(std::abs(endpoint[0].target_position - 0.1f) < 1e-6f &&
                std::abs(endpoint[1].target_position - 0.9f) < 1e-6f &&
                endpoint[0].target_velocity == 0.0f &&
                endpoint[1].target_velocity == 0.0f,
            "minimum-jerk endpoint is exact and has zero velocity");
  }

  ArticoreJointTrajectoryTarget long_target[] = {
      {motors[0].motor, 1.0f, 1.0f}, {motors[1].motor, 0.0f, 1.0f}};
  const auto preempted_id = runtime.start_joint_trajectory(
      long_target, 2, ARTICORE_TRAJECTORY_LINEAR);
  std::this_thread::sleep_for(8ms);
  ArticoreMitCommand direct[] = {
      {motors[0].motor, -0.25f, 0.0f, 20.0f, 3.0f, 0.0f},
      {motors[1].motor, 0.25f, 0.0f, 20.0f, 3.0f, 0.0f},
  };
  runtime.submit_mit(direct, 2);
  require(runtime.trajectory_info(preempted_id).status ==
              ARTICORE_TRAJECTORY_PREEMPTED,
          "direct command synchronously preempts the active trajectory");
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.last_arm_mit.size() == 2 &&
                   std::abs(driver.last_arm_mit[0].target_position + 0.25f) < 1e-6f &&
                   std::abs(driver.last_arm_mit[1].target_position - 0.25f) < 1e-6f;
          }),
          "direct target is active within one control cycle");
  std::this_thread::sleep_for(8ms);
  std::lock_guard<std::mutex> lock(driver.mutex);
  require(std::abs(driver.last_arm_mit[0].target_position + 0.25f) < 1e-6f &&
              std::abs(driver.last_arm_mit[1].target_position - 0.25f) < 1e-6f,
          "no preempted trajectory frame reappears after direct command returns");
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
    RUN_TEST(test_fault_policy_can_hold_gripper_until_recovery);
    RUN_TEST(test_feedback_policy_and_linked_disable);
    RUN_TEST(test_feedback_measurements_do_not_reuse_command_limits);
    RUN_TEST(test_feedback_seed_and_trajectory_start_ignore_command_limits);
    RUN_TEST(test_feedback_fault_diagnostics_include_identity_value_and_threshold);
    RUN_TEST(test_disable_does_not_stop_after_one_side_fails);
    RUN_TEST(test_disable_uses_structured_feedback_report);
    RUN_TEST(test_transport_disconnect_faults_and_links_both_sides);
    RUN_TEST(test_enable_grace_and_fault_latch);
    RUN_TEST(test_atomic_enable_starts_hold_and_confirms_both_sides);
    RUN_TEST(test_atomic_enable_retries_one_disabled_motor_once);
    RUN_TEST(test_atomic_enable_failure_rolls_back_and_fault_disable_is_allowed);
    RUN_TEST(test_repeated_runtime_lifecycle);
    RUN_TEST(test_single_side_runtime_and_gripper);
    RUN_TEST(test_motor_presence_is_fixed_and_fault_aware);
    RUN_TEST(test_latest_value_mailbox_drops_superseded_targets);
    RUN_TEST(test_min_jerk_trajectory_and_direct_preemption);
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

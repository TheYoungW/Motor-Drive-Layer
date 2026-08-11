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
  uint32_t pv_sends = 0;
  uint32_t mit_sends = 0;
  uint32_t disable_calls[2]{};
  bool fail_group = false;
  bool fail_left_disable = false;
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
  ++g_driver->pv_sends;
  return 0;
}

int32_t send_mit(void*, const ArticoreMitCommand* commands, uint32_t count) {
  std::lock_guard<std::mutex> lock(g_driver->mutex);
  if (g_driver->fail_group) return -1;
  g_driver->last_mit.assign(commands, commands + count);
  if (count == 2) g_driver->last_arm_mit.assign(commands, commands + count);
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

int32_t disable_motor(void* handle) {
  std::lock_guard<std::mutex> lock(g_driver->mutex);
  const auto found = g_driver->motors.find(handle);
  if (found == g_driver->motors.end()) return -1;
  const uint8_t side = handle == reinterpret_cast<void*>(0x201) ? 0 : 1;
  ++g_driver->disable_calls[side];
  found->second.status = 0;
  return side == 0 && g_driver->fail_left_disable ? -1 : 0;
}

int32_t request_feedback(void*, uint32_t) { return 0; }

int32_t get_state(void* handle, ArticoreMotorState* state) {
  std::lock_guard<std::mutex> lock(g_driver->mutex);
  const auto found = g_driver->motors.find(handle);
  if (found == g_driver->motors.end()) return -1;
  *state = {};
  state->has_value = found->second.has_feedback ? 1 : 0;
  state->status_code = found->second.status;
  state->pos = found->second.position;
  state->torq = found->second.torque;
  return 0;
}

int32_t get_feedback_stats(void* handle, ArticoreFeedbackStats* stats) {
  std::lock_guard<std::mutex> lock(g_driver->mutex);
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
        FakeMotor{1, static_cast<float>(i), 0.0f, 0, true};
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
  require(runtime.health().state == ARTICORE_RUNNING, "first command enters RUNNING");
  require(wait_for([&] { return runtime.health().state == ARTICORE_SAFE_HOLD; }),
          "command timeout enters SAFE_HOLD");
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.pv_sends >= 2;
          }),
          "safe hold sends stored PV target");
  {
    std::lock_guard<std::mutex> lock(driver.mutex);
    require(driver.last_pv.size() == 2 &&
                std::abs(driver.last_pv[0].velocity_limit - 0.15f) < 1e-6f,
            "PV safe hold replaces user velocity limit");
    driver.fail_group = true;
  }
  require(wait_for([&] { return runtime.health().state == ARTICORE_FAULT; }),
          "safe-hold send failure enters FAULT");
  const auto health = runtime.health();
  require(health.disable_confirmed == 1, "FAULT confirms linked disable");
  require(driver.disable_calls[0] > 0 && driver.disable_calls[1] > 0,
          "FAULT attempts both controllers");
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
  require(wait_for([&] { return runtime.health().state == ARTICORE_SAFE_HOLD; }),
          "MIT watchdog enters SAFE_HOLD");
  require(wait_for([&] {
            std::lock_guard<std::mutex> lock(driver.mutex);
            return driver.mit_sends >= 2;
          }),
          "MIT safe hold is transmitted");
  std::lock_guard<std::mutex> lock(driver.mutex);
  require(driver.last_arm_mit.size() == 2, "MIT arm hold captured");
  for (const auto& command : driver.last_arm_mit) {
    require(command.target_velocity == 0.0f &&
                command.feedforward_torque == 0.0f &&
                command.stiffness == 5.0f && command.damping == 0.5f,
            "MIT hold zeros velocity/torque and uses safe gains");
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
  runtime.recover();
  require(runtime.health().state == ARTICORE_READY &&
              runtime.health().disable_confirmed == 1,
          "recover first disables the held gripper and returns only to READY");
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
  require(wait_for([&] { return runtime.health().state == ARTICORE_SAFE_HOLD; }),
          "consecutive feedback failures enter SAFE_HOLD");
  require(wait_for([&] { return runtime.health().state == ARTICORE_FAULT; }),
          "feedback failure during SAFE_HOLD enters FAULT");
  require(driver.disable_calls[0] > 0 && driver.disable_calls[1] > 0,
          "feedback fault disables both sides");
}

void test_disable_does_not_stop_after_one_side_fails() {
  FakeDriver driver;
  g_driver = &driver;
  auto motors = descriptors(driver);
  articore::SafetyRuntime runtime(config(), api(), reinterpret_cast<void*>(0x100),
                                  g_left_controller, g_right_controller, motors);
  runtime.connect();
  runtime.enable(ARTICORE_MODE_PV);
  driver.fail_left_disable = true;
  bool failed = false;
  try {
    runtime.disable();
  } catch (const std::exception&) {
    failed = true;
  }
  require(failed && runtime.health().state == ARTICORE_FAULT,
          "failed disable locks FAULT");
  require(driver.disable_calls[0] >= 1 && driver.disable_calls[1] >= 1,
          "right side is still disabled after left side failure");
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
  require(wait_for([&] { return runtime.health().state == ARTICORE_FAULT; }),
          "a disconnected transport enters latched FAULT");
  const auto health = runtime.health();
  require(health.right_transport.connected == 0 &&
              health.right_transport.healthy == 0,
          "structured health identifies the disconnected side");
  require(driver.disable_calls[0] > 0 && driver.disable_calls[1] > 0,
          "transport disconnect still attempts linked disable on both sides");
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

}  // namespace

int main() {
  try {
    test_pv_watchdog_safe_hold_and_fault();
    test_mit_hold_removes_motion_and_feedforward();
    test_gripper_hold_retreats_once_on_overload();
    test_gripper_stall_switches_to_contact_hold_target();
    test_gripper_torque_spike_does_not_trigger_contact();
    test_fault_policy_can_hold_gripper_until_recovery();
    test_feedback_policy_and_linked_disable();
    test_disable_does_not_stop_after_one_side_fails();
    test_transport_disconnect_faults_and_links_both_sides();
    test_enable_grace_and_fault_latch();
    test_repeated_runtime_lifecycle();
    test_single_side_runtime_and_gripper();
    std::cout << "Articore runtime tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Articore runtime test failed: " << error.what() << '\n';
    return 1;
  }
}

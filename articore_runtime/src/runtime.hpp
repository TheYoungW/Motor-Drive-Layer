#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "articore/runtime_abi.h"

namespace articore {

class SafetyRuntime {
 public:
  SafetyRuntime(ArticoreRuntimeConfig config,
                ArticoreMotorApi api,
                void* controller_group,
                void* left_controller,
                void* right_controller,
                std::vector<ArticoreMotorDescriptor> motors);
  ~SafetyRuntime();

  SafetyRuntime(const SafetyRuntime&) = delete;
  SafetyRuntime& operator=(const SafetyRuntime&) = delete;

  void connect();
  void enable(ArticoreControlMode mode);
  void submit_pos_vel(const ArticorePosVelCommand* commands, uint32_t count);
  void submit_mit(const ArticoreMitCommand* commands, uint32_t count);
  void submit_gripper_mit(const ArticoreMitCommand* commands, uint32_t count);
  void set_gripper_openings(const ArticoreGripperTarget* targets,
                            uint32_t count);
  void report_feedback_failure(uint8_t side, const std::string& reason);
  void disable();
  void estop(const std::string& reason);
  void recover();
  ArticoreSafetyHealth health() const;
  void declare_motor_presence(const std::string& role,
                              ArticorePresenceState state);
  ArticorePresenceState motor_presence(const std::string& role) const;
  uint64_t active_capabilities() const;
  void close();

 private:
  using Clock = std::chrono::steady_clock;

  struct MotorRecord {
    ArticoreMotorDescriptor descriptor{};
    float last_position = 0.0f;
    bool has_position = false;
    float retreat_target = 0.0f;
    bool retreat_active = false;
    float protective_target = 0.0f;
    bool protective_target_active = false;
    Clock::time_point contact_started_at{};
    Clock::time_point overload_started_at{};
    Clock::time_point last_retreat_at{};
    std::deque<std::pair<Clock::time_point, float>> motion_samples;
    ArticoreGripperControlState gripper_state = ARTICORE_GRIPPER_DISABLED;
    float requested_opening = 0.0f;
    float requested_position = 0.0f;
    float command_position = 0.0f;
    bool has_gripper_target = false;
    bool contact_detected = false;
    bool stalled = false;
    bool overload = false;
    uint64_t feedback_age_ns = ~uint64_t{0};
    float last_torque = 0.0f;
    std::string gripper_fault_reason;
    Clock::time_point last_gripper_update{};
  };

  struct SideHealth {
    bool connected = false;
    bool healthy = false;
    uint32_t send_failures = 0;
    uint32_t feedback_failures = 0;
    uint64_t last_feedback_age_ns = ~uint64_t{0};
    uint64_t tx_frames = 0;
    uint64_t rx_frames = 0;
    uint64_t send_errors = 0;
    uint64_t receive_errors = 0;
    uint64_t last_tx_age_ns = ~uint64_t{0};
    uint64_t last_rx_age_ns = ~uint64_t{0};
    bool transport_healthy = true;
    std::string last_error;
  };

  void worker_loop();
  void require_state_for_command() const;
  void validate_motor_set(const ArticorePosVelCommand* commands,
                          uint32_t count, bool grippers_only) const;
  void validate_motor_set(const ArticoreMitCommand* commands,
                          uint32_t count, bool grippers_only) const;
  bool enter_safe_hold_from_feedback(const std::string& reason,
                                     std::string& error);
  void enter_fault(const std::string& reason);
  bool send_safe_hold_once(std::string& error);
  bool run_gripper_control_once(std::string& error);
  bool send_gripper_hold_once(std::string& error);
  bool disable_hardware(bool request_feedback, bool preserve_grippers,
                        std::string& error);
  bool refresh_feedback_health(bool recovery_check, bool allow_held_grippers,
                               std::string& error);
  bool refresh_transport_health(std::string& error);
  void seed_gripper_targets_from_feedback();
  std::string motor_error(const std::string& fallback) const;
  void set_side_error_locked(uint8_t side, const std::string& error,
                             bool send_failure);
  void mark_motor_faulted(void* motor);
  static bool finite(float value);
  static float clamp_opening(float opening);
  static float opening_to_position(const MotorRecord& motor, float opening);
  static float position_to_opening(const MotorRecord& motor, float position);

  ArticoreRuntimeConfig config_{};
  ArticoreMotorApi api_{};
  void* controller_group_ = nullptr;
  void* controllers_[2]{};
  bool active_sides_[2]{};
  std::vector<MotorRecord> motors_;
  std::map<std::string, ArticorePresenceState> presence_;
  std::map<void*, std::string> motor_roles_;

  mutable std::mutex state_mutex_;
  mutable std::mutex command_mutex_;
  std::condition_variable wakeup_;
  std::thread worker_;
  bool stopping_ = false;
  bool hardware_transition_ = false;
  ArticoreSafetyState state_ = ARTICORE_DISCONNECTED;
  ArticoreControlMode mode_ = ARTICORE_MODE_PV;
  bool fault_latched_ = false;
  bool disable_confirmed_ = false;
  bool has_successful_command_ = false;
  Clock::time_point enabled_at_{};
  Clock::time_point last_successful_command_{};
  Clock::time_point last_fresh_feedback_{};
  Clock::time_point next_feedback_check_{};
  Clock::time_point next_safe_hold_{};
  Clock::time_point next_gripper_control_{};
  uint32_t consecutive_send_failures_ = 0;
  uint32_t consecutive_feedback_failures_ = 0;
  uint32_t consecutive_hold_failures_ = 0;
  SideHealth sides_[2];
  std::string fault_reason_;
  std::vector<std::string> motor_faults_;
  std::vector<std::string> unconfirmed_disable_;
  std::vector<ArticorePosVelCommand> safe_pv_;
  std::vector<ArticoreMitCommand> safe_mit_;
  std::vector<ArticoreMitCommand> safe_grippers_;
};

}  // namespace articore

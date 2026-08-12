#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "articore/runtime_abi.h"

namespace articore {

namespace detail {

inline void advance_periodic_deadline(
    std::chrono::steady_clock::time_point& deadline,
    std::chrono::nanoseconds period,
    std::chrono::steady_clock::time_point now) {
  if (deadline == std::chrono::steady_clock::time_point{}) deadline = now;
  if (deadline > now) return;
  const auto overdue = std::chrono::duration_cast<std::chrono::nanoseconds>(
      now - deadline);
  const auto periods = overdue.count() / period.count() + 1;
  deadline += period * periods;
}

}  // namespace detail

class SafetyRuntime {
 public:
  SafetyRuntime(ArticoreRuntimeConfig config,
                ArticoreMotorApi api,
                void* controller_group,
                void* left_controller,
                void* right_controller,
                std::vector<ArticoreMotorDescriptor> motors,
                ArticoreControllerCallFn controller_enable_all = nullptr,
                ArticoreControllerCallFn motor_enable = nullptr);
  ~SafetyRuntime();

  SafetyRuntime(const SafetyRuntime&) = delete;
  SafetyRuntime& operator=(const SafetyRuntime&) = delete;

  void connect();
  void configure_joints(const ArticoreJointControlConfig* configs,
                        uint32_t count);
  void enable(ArticoreControlMode mode);
  ArticoreEnableReport last_enable_report() const;
  void submit_pos_vel(const ArticorePosVelCommand* commands, uint32_t count);
  void submit_mit(const ArticoreMitCommand* commands, uint32_t count);
  void submit_pos_vel_ex(const ArticorePosVelCommand* commands,
                         uint32_t count,
                         ArticoreCommandLifetime lifetime);
  void submit_mit_ex(const ArticoreMitCommand* commands,
                     uint32_t count,
                     ArticoreCommandLifetime lifetime);
  void submit_gripper_mit(const ArticoreMitCommand* commands, uint32_t count);
  void set_gripper_openings(const ArticoreGripperTarget* targets,
                            uint32_t count);
  uint64_t start_joint_trajectory(
      const ArticoreJointTrajectoryTarget* targets,
      uint32_t count,
      ArticoreTrajectoryProfile profile);
  ArticoreTrajectoryInfo trajectory_info(uint64_t trajectory_id) const;
  ArticoreTrajectoryInfo wait_trajectory(uint64_t trajectory_id,
                                         std::chrono::milliseconds timeout);
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
    uint32_t consecutive_feedback_failures = 0;
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

  struct MissingMotor {
    uint8_t side = 0;
    uint32_t id = 0;
  };

  struct JointControlConfig {
    float lower_position = 0.0f;
    float upper_position = 0.0f;
    float velocity_limit = 0.0f;
    float torque_limit = 0.0f;
    float mit_kp = 0.0f;
    float mit_kd = 0.0f;
    float mit_feedforward_torque = 0.0f;
  };

  struct ArmMailbox {
    bool valid = false;
    bool user_command = false;
    bool trajectory_endpoint_hold = false;
    ArticoreCommandLifetime lifetime = ARTICORE_COMMAND_STREAMING;
    uint64_t generation = 0;
    uint64_t sent_generation = 0;
    Clock::time_point submitted_at{};
    std::vector<ArticorePosVelCommand> pv;
    std::vector<ArticoreMitCommand> mit;
  };

  struct TrajectoryJoint {
    void* motor = nullptr;
    float start_position = 0.0f;
    float goal_position = 0.0f;
    float velocity_limit = 0.0f;
  };

  struct TrajectoryRecord {
    uint64_t id = 0;
    ArticoreTrajectoryStatus status = ARTICORE_TRAJECTORY_RUNNING;
    ArticoreTrajectoryProfile profile = ARTICORE_TRAJECTORY_MIN_JERK;
    Clock::time_point start_time{};
    std::chrono::nanoseconds duration{0};
    Clock::time_point finished_at{};
    std::vector<TrajectoryJoint> joints;
    std::string error;
  };

  void worker_loop();
  bool run_arm_control_cycle(Clock::time_point now, std::string& error);
  void initialize_arm_mailbox_from_feedback(ArticoreControlMode mode,
                                            bool require_enabled);
  bool request_feedback_parallel(uint32_t timeout_ms,
                                 std::vector<MissingMotor>& missing_motors,
                                 std::string& error);
  bool confirm_enabled_feedback(Clock::time_point deadline,
                                std::vector<MissingMotor>& missing_motors,
                                std::string& error);
  void update_enable_report(bool success,
                            bool disable_confirmed,
                            const std::vector<MissingMotor>& missing_motors,
                            const std::string& error);
  void initialize_enabled_state(ArticoreControlMode mode);
  bool send_initial_hold(ArticoreControlMode mode, std::string& error);
  void cancel_active_trajectory_locked(ArticoreTrajectoryStatus status,
                                       const std::string& error);
  void finish_trajectory_locked(uint64_t id,
                                ArticoreTrajectoryStatus status,
                                const std::string& error,
                                Clock::time_point now);
  void trim_trajectory_history_locked();
  const JointControlConfig& joint_config(void* motor) const;
  void validate_position_velocity_torque(void* motor, float position,
                                         float velocity, float torque) const;
  void require_state_for_command() const;
  void validate_motor_set(const ArticorePosVelCommand* commands,
                          uint32_t count, bool grippers_only) const;
  void validate_motor_set(const ArticoreMitCommand* commands,
                          uint32_t count, bool grippers_only) const;
  bool enter_safe_hold_from_feedback(const std::string& reason,
                                     std::string& error);
  void enter_fault(const std::string& reason, bool torque_off = false);
  bool send_safe_hold_once(std::string& error);
  bool run_gripper_control_once(std::string& error);
  bool send_gripper_hold_once(std::string& error);
  bool prepare_protective_hold(std::string& error);
  bool disable_hardware(bool request_feedback, bool preserve_grippers,
                        std::string& error);
  bool refresh_feedback_health(bool recovery_check, bool allow_held_grippers,
                               std::string& error);
  bool refresh_transport_health(std::string& error);
  void seed_gripper_targets_from_feedback(bool activate);
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
  ArticoreControllerCallFn controller_enable_all_ = nullptr;
  ArticoreControllerCallFn motor_enable_ = nullptr;
  bool active_sides_[2]{};
  std::vector<MotorRecord> motors_;
  std::map<std::string, ArticorePresenceState> presence_;
  std::map<void*, std::string> motor_roles_;
  std::unordered_map<void*, JointControlConfig> joint_configs_;

  mutable std::mutex state_mutex_;
  mutable std::mutex command_mutex_;
  std::condition_variable wakeup_;
  std::condition_variable trajectory_cv_;
  std::thread worker_;
  bool stopping_ = false;
  bool hardware_transition_ = false;
  bool enable_transaction_ = false;
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
  ArmMailbox arm_mailbox_;
  std::optional<TrajectoryRecord> active_trajectory_;
  std::map<uint64_t, TrajectoryRecord> trajectory_history_;
  std::deque<uint64_t> trajectory_history_order_;
  uint64_t next_trajectory_id_ = 1;
  Clock::time_point next_control_tick_{};
  SideHealth sides_[2];
  std::string fault_reason_;
  std::vector<std::string> motor_faults_;
  std::vector<std::string> unconfirmed_disable_;
  std::vector<ArticorePosVelCommand> safe_pv_;
  std::vector<ArticoreMitCommand> safe_mit_;
  std::vector<ArticoreMitCommand> safe_grippers_;
  std::vector<ArticorePosVelCommand> last_sent_pv_;
  std::vector<ArticoreMitCommand> last_sent_mit_;
  bool fault_hold_active_ = false;
  ArticoreEnableReport last_enable_report_{};
};

}  // namespace articore

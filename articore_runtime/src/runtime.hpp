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
#include <unordered_map>
#include <utility>
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
                ArticoreControllerCallFn motor_enable = nullptr,
                bool require_gripper_product_profiles = false);
  ~SafetyRuntime();

  SafetyRuntime(const SafetyRuntime&) = delete;
  SafetyRuntime& operator=(const SafetyRuntime&) = delete;

  void connect();
  void configure_joints(const ArticoreJointControlConfig* configs,
                        uint32_t count);
  void configure_joint_safety_limits(
      const ArticoreJointSafetyLimits* limits, uint32_t count);
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
  void set_joint_mit(const ArticoreJointMitTarget* targets,
                     uint32_t count,
                     float max_reference_velocity);
  void set_joint_pv(const ArticoreJointPvTarget* targets,
                    uint32_t count,
                    float max_reference_velocity);
  void submit_gripper_mit(const ArticoreMitCommand* commands, uint32_t count);
  void set_gripper_openings(const ArticoreGripperTarget* targets,
                            uint32_t count);
  void configure_gripper_force_profiles(
      const ArticoreGripperForceProfile* profiles, uint32_t count);
  void configure_gripper_products(
      const ArticoreGripperProductBinding* bindings, uint32_t count);
  void set_gripper_commands(const ArticoreGripperCommand* commands,
                            uint32_t count);
  void report_feedback_failure(uint8_t side, const std::string& reason);
  void disable();
  ArticoreDisableReport last_disable_report() const;
  void estop(const std::string& reason);
  void recover();
  ArticoreSafetyHealth health() const;
  uint32_t control_hz() const noexcept { return config_.control_hz; }
  void declare_motor_presence(const std::string& role,
                              ArticorePresenceState state);
  ArticorePresenceState motor_presence(const std::string& role) const;
  uint64_t active_capabilities() const;
  void close();

 private:
  using Clock = std::chrono::steady_clock;

  struct MotorRecord {
    struct GripperForceProfile {
      float contact_torque = 0.0f;
      float overload_torque = 0.0f;
      float moving_kp = 0.0f;
      float moving_kd = 0.0f;
      float hold_kp = 0.0f;
      float hold_kd = 0.0f;
    };

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
    float requested_speed = 1000.0f;
    float command_speed = 0.0f;
    ArticoreGripperForceLevel force_level = ARTICORE_GRIPPER_FORCE_DEFAULT;
    std::unordered_map<int32_t, GripperForceProfile> force_profiles;
    std::string gripper_product_profile_id;
    bool gripper_product_profile_bound = false;
    // ABI 1.10 callers encode LOW/NORMAL/HIGH as 1/2/3. When their legacy
    // three-profile calibration is detected, preserve those command meanings
    // while still making interpolated levels 4..10 available.
    bool legacy_force_level_mapping = false;
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
    float hard_lower_position = 0.0f;
    float hard_upper_position = 0.0f;
    float soft_lower_position = 0.0f;
    float soft_upper_position = 0.0f;
    float soft_limit_braking_zone = 0.0f;
    float braking_acceleration = 0.0f;
    float velocity_limit = 0.0f;
    float torque_limit = 0.0f;
    float mit_kp = 0.0f;
    float mit_kd = 0.0f;
    float mit_feedforward_torque = 0.0f;
    bool layered_limits_configured = false;
  };

  struct ArmMailbox {
    bool valid = false;
    bool user_command = false;
    ArticoreCommandLifetime lifetime = ARTICORE_COMMAND_STREAMING;
    uint64_t generation = 0;
    uint64_t sent_generation = 0;
    Clock::time_point submitted_at{};
    // Ordinary PV/MIT position setting owns current q in the active mode's
    // command vector and matching user endpoints in final_positions.
    bool joint_position = false;
    float max_reference_velocity = 0.0f;
    std::vector<ArticorePosVelCommand> pv;
    std::vector<ArticoreMitCommand> mit;
    std::vector<float> final_positions;
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
  const JointControlConfig& joint_config(void* motor) const;
  void validate_position_velocity_torque(void* motor, float position,
                                         float velocity, float torque) const;
  void require_state_for_command() const;
  void validate_motor_set(const ArticorePosVelCommand* commands,
                          uint32_t count, bool grippers_only) const;
  void validate_motor_set(const ArticoreMitCommand* commands,
                          uint32_t count, bool grippers_only) const;
  void install_joint_position(
      ArticoreControlMode mode,
      const std::vector<std::pair<void*, float>>& targets,
      float max_reference_velocity);
  bool enter_safe_hold_from_feedback(const std::string& reason,
                                     std::string& error);
  void enter_fault(const std::string& reason, bool torque_off = false);
  bool send_safe_hold_once(std::string& error);
  bool run_gripper_control_once(std::string& error);
  bool send_gripper_hold_once(std::string& error);
  static const MotorRecord::GripperForceProfile& active_gripper_profile(
      const MotorRecord& motor);
  bool prepare_protective_hold(std::string& error);
  bool disable_hardware(bool request_feedback, bool preserve_grippers,
                        std::string& error);
  bool establish_disable_barrier(std::string& error);
  void update_disable_report(bool success, bool barrier_confirmed,
                             const std::vector<MissingMotor>& missing_motors,
                             const std::vector<void*>& initially_sent,
                             const std::vector<void*>& retried,
                             bool retry_attempted,
                             bool preserve_grippers,
                             const std::string& error);
  void stop_worker();
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
  bool require_gripper_product_profiles_ = false;
  bool active_sides_[2]{};
  std::vector<MotorRecord> motors_;
  std::map<std::string, ArticorePresenceState> presence_;
  std::map<void*, std::string> motor_roles_;
  std::unordered_map<void*, JointControlConfig> joint_configs_;
  mutable std::mutex state_mutex_;
  mutable std::mutex command_mutex_;
  mutable std::recursive_mutex lifecycle_mutex_;
  std::condition_variable wakeup_;
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
  // Per-call gripper commands are persistent setpoints. Their generation is
  // acknowledged only after the complete native gripper batch is sent. This
  // lets a gripper-only command satisfy enable_grace without letting the
  // continuously retransmitted hold mask an arm STREAMING watchdog timeout.
  uint64_t gripper_command_generation_ = 0;
  uint64_t gripper_sent_generation_ = 0;
  uint32_t consecutive_send_failures_ = 0;
  uint32_t consecutive_feedback_failures_ = 0;
  uint32_t consecutive_hold_failures_ = 0;
  ArmMailbox arm_mailbox_;
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
  ArticoreDisableReport last_disable_report_{};
};

}  // namespace articore

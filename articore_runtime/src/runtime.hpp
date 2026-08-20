#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "articore/runtime_abi.h"
#include "robot_model.hpp"

namespace articore {

struct NativeTrajectoryJoint {
  void* motor = nullptr;
  float direction = 1.0f;
  float velocity_command_scale = 1.0f;
  float velocity_feedback_scale = 1.0f;
  float torque_command_scale = 1.0f;
  float lower_position = 0.0f;
  float upper_position = 0.0f;
  float velocity_limit = 0.0f;
  float acceleration_limit = 0.0f;
  float torque_limit = 0.0f;
  float mit_kp = 0.0f;
  float mit_kd = 0.0f;
  float mit_feedforward_torque = 0.0f;
  float pv_velocity_limit = 0.0f;
};

struct NativeTrajectoryWaypoint {
  double time_s = 0.0;
  std::vector<float> positions;
  std::vector<float> velocities;
  std::vector<float> accelerations;
  uint32_t velocity_valid_mask = 0;
  uint32_t acceleration_valid_mask = 0;
};

struct NativeTrajectoryRequest {
  ArticoreControlMode mode = ARTICORE_MODE_MIT;
  ArticoreRuntimeOperation operation = ARTICORE_OPERATION_START_TRAJECTORY;
  std::vector<NativeTrajectoryJoint> joints;
  std::vector<NativeTrajectoryWaypoint> waypoints;
};

struct NativeTrajectorySample {
  bool active = false;
  uint64_t trajectory_id = 0;
  ArticoreRuntimeOperation operation = ARTICORE_OPERATION_NONE;
  std::vector<float> positions;
  std::vector<float> velocities;
  std::vector<float> accelerations;
};

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
                bool require_gripper_product_profiles = false,
                std::vector<ArticoreRuntimeTransportCapabilities>
                    transport_capabilities = {},
                ArticoreMotorMaintenanceApi maintenance_api = {},
                uint32_t internal_control_rate_override = 0);
  ~SafetyRuntime();

  SafetyRuntime(const SafetyRuntime&) = delete;
  SafetyRuntime& operator=(const SafetyRuntime&) = delete;

  void connect();
  void configure_motor_identities(const ArticoreMotorIdentity* identities,
                                  uint32_t count);
  ArticoreConnectReport last_connect_report() const;
  void configure_joints(const ArticoreJointControlConfig* configs,
                        uint32_t count);
  void configure_joint_safety_limits(
      const ArticoreJointSafetyLimits* limits, uint32_t count);
  void enable(ArticoreControlMode mode);
  ArticoreMotorPowerReport set_motor_power_batch(
      const std::vector<std::string>& motor_names, bool enabled);
  ArticoreMotorPowerState set_motor_power(const std::string& motor_name,
                                          bool enabled);
  ArticoreMotorPowerState motor_power_state(const std::string& motor_name);
  int32_t configure_mode(ArticoreControlMode mode);
  int32_t clear_faults();
  int32_t set_zero();
  void record_operation_result(ArticoreRuntimeOperation operation,
                               int32_t code,
                               const std::string& error = {},
                               const std::vector<std::string>& failed_motors = {});
  ArticoreEnableReport last_enable_report() const;
  void submit_pos_vel(const ArticorePosVelCommand* commands, uint32_t count);
  void submit_mit(const ArticoreMitCommand* commands, uint32_t count);
  void submit_pos_vel_ex(const ArticorePosVelCommand* commands,
                         uint32_t count,
                         ArticoreCommandLifetime lifetime);
  void submit_mit_ex(const ArticoreMitCommand* commands,
                     uint32_t count,
                     ArticoreCommandLifetime lifetime);
  // replace_trajectory_id=0 preserves strict trajectory semantics and rejects
  // a concurrent plan. A non-zero value atomically replaces exactly that
  // running plan after the new request has passed all validation.
  uint64_t start_trajectory(NativeTrajectoryRequest request,
                            uint64_t replace_trajectory_id = 0);
  NativeTrajectorySample trajectory_sample() const;
  ArticoreTrajectoryStatus trajectory_status() const;
  void cancel_trajectory();
  void set_joint_mit(const ArticoreJointMitTarget* targets,
                     uint32_t count,
                     float max_reference_velocity);
  void set_joint_pv(const ArticoreJointPvTarget* targets,
                    uint32_t count,
                    float max_reference_velocity);
  void set_joint_mit_speed(const ArticoreJointMitTarget* targets,
                           uint32_t count, float speed_percent);
  void set_joint_pv_speed(const ArticoreJointPvTarget* targets,
                          uint32_t count, float speed_percent);
  void submit_gripper_mit(const ArticoreMitCommand* commands, uint32_t count);
  void set_gripper_openings(const ArticoreGripperTarget* targets,
                            uint32_t count);
  void configure_gripper_force_profiles(
      const ArticoreGripperForceProfile* profiles, uint32_t count);
  void configure_gripper_products(
      const ArticoreGripperProductBinding* bindings, uint32_t count);
  void configure_gravity_products(
      const ArticoreGravityProductBinding* bindings, uint32_t count);
  void start_gravity_compensation(
      const ArticoreGravityCompensationConfig* config);
  void stop_gravity_compensation();
  ArticoreGravityCompensationStatus gravity_compensation_status() const;
  void set_gripper_commands(const ArticoreGripperCommand* commands,
                            uint32_t count,
                            int32_t mode = ARTICORE_GRIPPER_MODE_PROTECTED);
  int32_t gripper_force_level(uint8_t side) const;
  void report_feedback_failure(uint8_t side, const std::string& reason);
  void disable();
  ArticoreDisableReport last_disable_report() const;
  ArticoreMitTorqueLimitStats mit_torque_limit_stats() const;
  void estop();
  void recover();
  ArticoreSafetyHealth health() const;
  ArticoreSafetyHealthV2 health_v2() const;
  uint32_t control_hz() const noexcept { return control_hz_; }
  ArticoreControlMode control_mode() const;
  uint64_t feedback_max_age_ns() const noexcept {
    return static_cast<uint64_t>(config_.feedback_max_age_ms) * 1'000'000ULL;
  }
  void declare_motor_presence(const std::string& role,
                              ArticorePresenceState state);
  ArticorePresenceState motor_presence(const std::string& role) const;
  uint64_t active_capabilities() const;
  void disconnect();
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
    uint32_t configured_can_id = 0;
    bool motor_identity_configured = false;
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
    ArticoreGripperMode gripper_mode = ARTICORE_GRIPPER_MODE_PROTECTED;
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

  struct FeedbackSideResult {
    bool active = false;
    int32_t code = 0;
    ArticoreFeedbackReport report{};
    std::vector<uint32_t> missing;
    std::string error;
  };
  using FeedbackTransactionResults = std::array<FeedbackSideResult, 2>;

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

  struct GravityArm {
    uint32_t runtime_side = 0;
    uint32_t robot_side = 0;
    std::string product_id;
    std::unique_ptr<RobotModel> model;
    std::array<void*, 7> joints{};
  };

  struct GravityControl {
    ArticoreGravityCompensationPhase phase = ARTICORE_GRAVITY_INACTIVE;
    Clock::time_point transition_started{};
    std::chrono::milliseconds transition_duration{500};
    float transition_start_gravity_scale = 0.0f;
    std::vector<float> hold_positions;
    uint64_t control_cycles = 0;
    ArticoreGravityCompensationStatus status{};
  };

  struct TrajectorySegment {
    double start_s = 0.0;
    double duration_s = 0.0;
    std::vector<std::array<double, 6>> coefficients;
  };

  struct TrajectoryControl {
    ArticoreTrajectoryState state = ARTICORE_TRAJECTORY_IDLE;
    uint64_t id = 0;
    uint32_t waypoint_count = 0;
    uint32_t active_segment = 0;
    double elapsed_s = 0.0;
    double duration_s = 0.0;
    Clock::time_point started_at{};
    Clock::time_point settling_started_at{};
    ArticoreRuntimeOperation operation = ARTICORE_OPERATION_START_TRAJECTORY;
    std::vector<NativeTrajectoryJoint> joints;
    std::vector<TrajectorySegment> segments;
    std::vector<uint64_t> settling_feedback_updates;
    uint32_t settled_feedback_samples = 0;
    bool settling_feedback_initialized = false;
    std::string error;
  };

  void worker_loop();
  bool run_arm_control_cycle(Clock::time_point now, bool include_grippers,
                             std::string& error);
  bool run_gravity_control_cycle(Clock::time_point now,
                                 bool include_grippers,
                                 std::string& error);
  bool prepare_trajectory_cycle(Clock::time_point now,
                                bool& completing,
                                std::string& error);
  void update_trajectory_completion(Clock::time_point now);
  void terminate_trajectory_locked(ArticoreTrajectoryState state,
                                   const std::string& error);
  void fault_trajectory(const std::string& error);
  bool prepare_mit_torque_limited_commands(
      const std::vector<ArticoreMitCommand>& requested,
      std::vector<ArticoreMitCommand>& applied,
      ArticoreMitTorqueLimitStats& cycle_stats,
      std::string& error,
      float torque_limit_scale = 1.0f) const;
  uint64_t next_arm_generation() noexcept;
  void consume_pending_arm_mailbox();
  void clear_pending_arm_mailbox();
  void initialize_arm_mailbox_from_feedback(ArticoreControlMode mode,
                                            bool require_enabled);
  bool request_feedback_parallel(uint32_t timeout_ms,
                                 std::vector<MissingMotor>& missing_motors,
                                 std::string& error,
                                 FeedbackTransactionResults* results = nullptr);
  bool validate_fresh_feedback_snapshot(
      const std::vector<MissingMotor>& request_missing,
      std::string& error,
      ArticoreConnectReport* connect_report = nullptr);
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
  void require_state_for_command(bool allow_gravity = false,
                                 bool allow_trajectory = false) const;
  void validate_motor_set(const ArticorePosVelCommand* commands,
                          uint32_t count, bool grippers_only) const;
  void validate_motor_set(const ArticoreMitCommand* commands,
                          uint32_t count, bool grippers_only) const;
  void install_joint_position(
      ArticoreControlMode mode,
      const std::vector<std::pair<void*, float>>& targets,
      float max_reference_velocity);
  float ordinary_velocity_from_percent(ArticoreControlMode mode,
                                       float speed_percent) const;
  bool enter_safe_hold_from_feedback(const std::string& reason,
                                     std::string& error);
  void enter_degraded(const std::string& reason);
  bool enter_safe_stop(const std::string& reason, std::string& error);
  void enter_fault(const std::string& reason, bool torque_off = false,
                   bool allow_protective_hold = true);
  bool send_safe_hold_once(std::string& error);
  bool run_gripper_control_once(std::string& error);
  bool prepare_gripper_commands_locked(
      Clock::time_point now, std::vector<ArticoreMitCommand>& commands,
      std::string& error);
  void commit_gripper_commands_sent(
      const std::vector<ArticoreMitCommand>& commands,
      Clock::time_point now);
  bool send_gripper_hold_once(std::string& error);
  static const MotorRecord::GripperForceProfile& active_gripper_profile(
      const MotorRecord& motor);
  static float gripper_gain_scale(const MotorRecord& motor);
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
                               std::string& error,
                               bool* diagnostic_only = nullptr);
  bool refresh_transport_health(std::string& error);
  void seed_gripper_targets_from_feedback(bool activate);
  std::string motor_error(const std::string& fallback) const;
  void set_side_error_locked(uint8_t side, const std::string& error,
                             bool send_failure);
  void mark_motor_faulted(void* motor);
  int32_t maintenance_precheck(ArticoreRuntimeOperation operation,
                               std::string& error,
                               std::vector<std::string>& failed_motors);
  int32_t finish_maintenance(ArticoreRuntimeOperation operation,
                             int32_t code,
                             const std::string& error,
                             const std::vector<std::string>& failed_motors,
                             bool latch_fault);
  int32_t run_motor_maintenance(ArticoreRuntimeOperation operation,
                                ArticoreControllerCallFn callback,
                                ArticoreControlMode mode);
  static bool finite(float value);
  MotorRecord* resolve_motor_role(const std::string& role);
  std::string stable_motor_role(const MotorRecord& motor) const;
  ArticoreMotorPowerState cached_motor_power_state(
      const MotorRecord* selected = nullptr) const;
  static float clamp_opening(float opening);
  static float opening_to_position(const MotorRecord& motor, float opening);
  static float position_to_opening(const MotorRecord& motor, float position);

  ArticoreRuntimeConfig config_{};
  // Product control scheduling is deliberately absent from the stable ABI.
  // The optional constructor override exists only in this private header for
  // deterministic native tests.
  uint32_t control_hz_ = 400;
  ArticoreMotorApi api_{};
  ArticoreMotorMaintenanceApi maintenance_api_{};
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
  std::vector<GravityArm> gravity_arms_;
  GravityControl gravity_control_;
  TrajectoryControl trajectory_control_;
  uint64_t next_trajectory_id_ = 1;
  mutable std::mutex state_mutex_;
  mutable std::mutex command_mutex_;
  // Raw arm submissions use a capacity-one pending mailbox so callers never
  // wait for the native worker's physical ControllerGroup dispatch. The
  // command mutex remains the transport/lifecycle serialization barrier.
  mutable std::mutex pending_arm_mutex_;
  mutable std::recursive_mutex lifecycle_mutex_;
  std::condition_variable wakeup_;
  std::thread worker_;
  bool stopping_ = false;
  bool hardware_transition_ = false;
  bool enable_transaction_ = false;
  ArticoreSafetyState state_ = ARTICORE_DISCONNECTED;
  ArticoreControlMode mode_ = ARTICORE_MODE_PV;
  bool fault_latched_ = false;
  bool emergency_stop_latched_ = false;
  bool disable_confirmed_ = false;
  bool has_successful_command_ = false;
  Clock::time_point enabled_at_{};
  Clock::time_point last_successful_command_{};
  Clock::time_point last_fresh_feedback_{};
  Clock::time_point next_ready_feedback_{};
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
  std::atomic<uint64_t> arm_generation_{0};
  ArmMailbox arm_mailbox_;
  ArmMailbox pending_arm_mailbox_;
  Clock::time_point next_control_tick_{};
  SideHealth sides_[2];
  std::string fault_reason_;
  std::string safety_reason_;
  std::vector<std::string> motor_faults_;
  std::vector<std::string> unconfirmed_disable_;
  ArticoreRuntimeOperation last_operation_ = ARTICORE_OPERATION_NONE;
  int32_t last_operation_code_ = ARTICORE_OPERATION_OK;
  std::string last_operation_error_;
  std::vector<std::string> operation_failed_motors_;
  // Motors disabled through the product-level subset transaction. Complete
  // frames remain valid, but worker dispatch filters these handles until an
  // explicit subset or whole-product enable succeeds.
  std::set<void*> intentionally_disabled_motors_;
  std::vector<ArticorePosVelCommand> safe_pv_;
  std::vector<ArticoreMitCommand> safe_mit_;
  std::vector<ArticoreMitCommand> safe_grippers_;
  std::vector<ArticorePosVelCommand> last_sent_pv_;
  std::vector<ArticoreMitCommand> last_sent_mit_;
  // Reused by the single native worker so per-cycle MIT limiting does not
  // allocate after the first complete arm batch.
  std::vector<ArticoreMitCommand> mit_torque_limited_commands_;
  std::vector<ArticorePosVelCommand> degraded_pv_commands_;
  std::vector<ArticoreMitCommand> degraded_mit_commands_;
  std::vector<ArticorePosVelCommand> filtered_pv_commands_;
  std::vector<ArticoreMitCommand> filtered_mit_commands_;
  bool fault_hold_active_ = false;
  ArticoreEnableReport last_enable_report_{};
  ArticoreDisableReport last_disable_report_{};
  ArticoreConnectReport last_connect_report_{};
  ArticoreMitTorqueLimitStats mit_torque_limit_stats_{};
};

}  // namespace articore

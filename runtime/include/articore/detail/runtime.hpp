#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "articore/detail/runtime_bridge.hpp"
#include "articore/detail/robot_model.hpp"
#include "articore/detail/runtime_types.hpp"

// Private callback adapter retained only for deterministic native tests. The
// installed product ABI no longer exposes caller-assembled Motor resources.
using ArticoreControllerCallFn = int32_t (*)(void*);
using ArticoreGroupSendPosVelFn = int32_t (*)(
    void*, const ArticorePosVelCommand*, uint32_t);
using ArticoreGroupSendMitFn = int32_t (*)(
    void*, const ArticoreMitCommand*, uint32_t);
using ArticoreControllerFeedbackFn = int32_t (*)(
    void*, uint32_t, ArticoreFeedbackReport*, uint32_t*, uint32_t);
using ArticoreMotorGetStateFn = int32_t (*)(void*, ArticoreMotorState*);
using ArticoreMotorGetFeedbackStatsFn = int32_t (*)(
    void*, ArticoreFeedbackStats*);
using ArticoreControllerTransportHealthFn = int32_t (*)(
    void*, ArticoreDriverTransportHealth*);
using ArticoreLastErrorFn = const char* (*)(void);
using ArticoreMotorEnsureModeFn = int32_t (*)(void*, uint32_t, uint32_t);
using ArticoreMotorSetCanTimeoutFn = int32_t (*)(void*, uint32_t);

struct ArticoreMotorApi {
  ArticoreGroupSendPosVelFn group_send_pos_vel;
  ArticoreGroupSendMitFn group_send_mit;
  ArticoreControllerCallFn controller_disable_all;
  ArticoreControllerFeedbackFn controller_request_feedback_all_ex;
  ArticoreMotorGetStateFn motor_get_state;
  ArticoreMotorGetFeedbackStatsFn motor_get_feedback_stats;
  ArticoreLastErrorFn last_error_message;
  ArticoreControllerTransportHealthFn controller_get_transport_health;
  ArticoreControllerCallFn motor_disable;
};

struct ArticoreMotorMaintenanceApi {
  uint32_t struct_size;
  ArticoreControllerCallFn motor_clear_error;
  ArticoreControllerCallFn motor_set_zero_position;
  ArticoreMotorEnsureModeFn motor_ensure_mode;
  ArticoreMotorSetCanTimeoutFn motor_set_can_timeout_ms;
  uint32_t communication_timeout_ms;
};

namespace articore {

inline constexpr uint32_t kNativeMaximumTrajectoryWaypoints = 30000;

class InvalidRuntimeState : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

class MotorBackend {
 public:
  virtual ~MotorBackend() = default;

  virtual int32_t send_pos_vel(
      void* group, const ArticorePosVelCommand* commands, uint32_t count) = 0;
  virtual int32_t send_mit(
      void* group, const ArticoreMitCommand* commands, uint32_t count) = 0;
  virtual int32_t disable_all(void* controller) = 0;
  virtual int32_t request_feedback(
      void* controller, uint32_t timeout_ms, ArticoreFeedbackReport* report,
      uint32_t* missing, uint32_t capacity) = 0;
  virtual int32_t get_state(void* motor, ArticoreMotorState* state) = 0;
  virtual int32_t get_feedback_stats(
      void* motor, ArticoreFeedbackStats* stats) = 0;
  virtual int32_t get_transport_health(
      void* controller, ArticoreDriverTransportHealth* health) = 0;
  virtual bool has_transport_health() const { return true; }
  virtual int32_t disable_motor(void* motor) = 0;
  virtual const char* last_error_message() const = 0;

  virtual bool can_enable_all() const { return false; }
  virtual int32_t enable_all(void*) { return -1; }
  virtual bool can_enable_motor() const { return false; }
  virtual int32_t enable_motor(void*) { return -1; }
  virtual bool can_clear_error() const { return false; }
  virtual int32_t clear_error(void*) { return -1; }
  virtual bool can_set_zero() const { return false; }
  virtual int32_t set_zero(void*) { return -1; }
  virtual bool can_ensure_mode() const { return false; }
  virtual int32_t ensure_mode(void*, uint32_t, uint32_t) { return -1; }
  virtual bool can_set_timeout() const { return false; }
  virtual int32_t set_timeout_ms(void*, uint32_t) { return -1; }
  virtual uint32_t communication_timeout_ms() const { return 0; }
};

// Damiao POS_VEL interprets the second float as a motion speed limit. Once the
// final position has physically arrived, zero keeps the position target armed
// without asking the drive to keep making finite-speed corrections around the
// target. The normal trajectory limit is restored automatically if feedback
// leaves the arrival window.
inline constexpr float kNativePvFinalHoldVelocityLimit = 0.0f;
inline constexpr float kNativePvSettlingVelocityLimit = 0.05f;
inline constexpr float kNativeOrdinaryPvDefaultAcceleration = 6.0f;
inline constexpr float kNativeTrajectoryPvAccelerationLimit = 6.0f;
inline constexpr float kNativeOrdinaryPvHoldPositionTolerance = 0.002f;
// Feedback is converted to logical joint velocity before this threshold is
// applied. It admits one reported velocity quantum without visible motion.
inline constexpr float kNativeOrdinaryPvHoldVelocityTolerance = 0.02f;
// A held joint must remain inside the same physical arrival window used to
// arm V=0. Any later drift releases the hold so strict endpoint accuracy is
// never traded for a visually quiet but offset joint.
inline constexpr float kNativeOrdinaryPvHoldReleaseTolerance =
    kNativeOrdinaryPvHoldPositionTolerance;
// Require 50 ms of stable feedback before arming V=0. Identical endpoint
// replacements preserve this counter, while a moving publisher keeps resetting
// it before V=0 can be armed.
inline constexpr uint16_t kNativeOrdinaryPvHoldConfirmationCycles = 25;
// Ordinary PV is a latest-endpoint-wins online step command. Runtime sends the
// final P on the first 500 Hz command frame and shapes only the Motor V ceiling
// from the configured speed, acceleration, and physical remaining distance.
// It is not a finite planned trajectory. Runtime-owned trajectory-PV execution
// retains a finite,
// time-stamped knot list selected by its planner and linearly resamples its
// variable-duration segments on the same 500 Hz send clock.
// POS_VEL P is transported as float32, but every installed Yunyi Damiao model
// reports position as 16 bits over [-12.5, 12.5] rad. Treat one feedback code
// as the minimum meaningful outgoing trajectory-P change. Runtime keeps the
// ideal time reference moving and accumulates the unsent difference, so this
// deadband neither shortens duration nor loses the exact final endpoint.
inline constexpr float kNativePvPositionFeedbackQuantum =
    25.0f / 65535.0f;
inline constexpr float kNativeOrdinaryPvHoldTargetTolerance =
    kNativePvPositionFeedbackQuantum;

inline float native_trajectory_pv_effective_position(
    float ideal_position, float previous_command_position,
    bool force_exact = false) {
  return force_exact ||
          std::abs(ideal_position - previous_command_position) >=
              kNativePvPositionFeedbackQuantum
      ? ideal_position
      : previous_command_position;
}

// Cartesian trajectory-PV references normally move much more slowly than the
// product ceiling. Keep the Damiao POS_VEL speed limit close to the native
// reference so the internal position loop cannot repeatedly sprint at the
// target. Finite trajectory plans use validated time-stamped knots; their
// outgoing references and ordinary PV both use the 500 Hz Runtime control
// clock.
inline constexpr float kNativeCartesianTrajectoryPvVelocityGain = 1.5f;
inline constexpr float kNativeCartesianTrajectoryPvVelocityMargin = 0.05f;
inline constexpr float kNativeCartesianTrajectoryPvTrackingCatchupGain = 5.0f;
inline constexpr float kNativeCartesianTrajectoryPvFinalCatchupGain = 10.0f;
inline constexpr float kNativeCartesianTrajectoryPvFinalCatchupMaximum = 0.50f;
inline constexpr float kNativeCartesianReferenceMinimumStep = 0.0005f;
inline constexpr double kNativeCartesianReferenceMaximumHoldSeconds = 0.010;
inline constexpr float kNativeCartesianContinuousReferenceVelocity = 0.10f;
inline constexpr float kNativeCartesianFifoHandoffMaximumError = 0.040f;
inline constexpr float kNativeCartesianTrackingPauseError = 0.060f;
inline constexpr auto kNativeCartesianTrackingPauseTimeout =
    std::chrono::seconds(1);

inline float native_trajectory_pv_drive_velocity_limit(
    float planned_velocity, float minimum_velocity, float product_ceiling) {
  return std::clamp(
      kNativeCartesianTrajectoryPvVelocityGain * std::abs(planned_velocity) +
          kNativeCartesianTrajectoryPvVelocityMargin,
      minimum_velocity, product_ceiling);
}

// Cartesian duration_s is the estimated complete task time. Planning reserves
// part of it for feedback catch-up and the 200 ms physical stability window,
// but a robot that arrives late keeps tracking instead of faulting exactly at
// duration_s. A separate bounded arrival timeout still catches stalled or
// unreachable motion. The drive keeps its tracking-error-based catch-up limit
// until the tighter final-position window is reached; only that final phase
// uses the quiet settling limit. Product planning validates the faster motion
// portion against velocity and acceleration limits before installing it.
inline constexpr double kNativeCartesianTargetAcquisitionSeconds = 1.60;
inline constexpr double kNativeCartesianMinimumAcquisitionSeconds = 0.25;

inline bool native_cartesian_operation(ArticoreRuntimeOperation operation) {
  return operation == ARTICORE_OPERATION_MOVE_LINEAR_TRAJECTORY ||
      operation == ARTICORE_OPERATION_MOVE_CIRCULAR_TRAJECTORY;
}

inline float native_cartesian_trajectory_pv_velocity_limit(
    float planned_velocity, float tracking_position_error,
    float minimum_velocity, float product_ceiling) {
  return std::clamp(
      std::max(
          kNativeCartesianTrajectoryPvVelocityGain *
                  std::abs(planned_velocity) +
              kNativeCartesianTrajectoryPvVelocityMargin,
          kNativeCartesianTrajectoryPvTrackingCatchupGain *
              std::abs(tracking_position_error)),
      minimum_velocity, product_ceiling);
}

inline float native_cartesian_trajectory_pv_final_velocity_limit(
    float tracking_position_error, float minimum_velocity,
    float product_ceiling) {
  return std::clamp(
      kNativeCartesianTrajectoryPvFinalCatchupGain *
          std::abs(tracking_position_error),
      minimum_velocity,
      std::min(
          product_ceiling,
          kNativeCartesianTrajectoryPvFinalCatchupMaximum));
}

inline float native_cartesian_tracking_scale(float position_error) {
  return std::isfinite(position_error) &&
          position_error >= kNativeCartesianTrackingPauseError
      ? 0.0f
      : 1.0f;
}

inline bool native_cartesian_reference_update_due(
    float maximum_position_change, double held_seconds,
    bool force_update = false) {
  return force_update || !std::isfinite(maximum_position_change) ||
      !std::isfinite(held_seconds) ||
      maximum_position_change >= kNativeCartesianReferenceMinimumStep ||
      held_seconds >= kNativeCartesianReferenceMaximumHoldSeconds;
}

inline float advance_pv_position_reference(
    float current_position, float target_position, float max_delta) {
  const float error = target_position - current_position;
  return current_position + std::clamp(error, -max_delta, max_delta);
}

struct NativePvReferenceStep {
  float position = 0.0f;
  float velocity = 0.0f;
};

// Native ordinary-PV and MIT-style online reference shaping.
inline NativePvReferenceStep advance_acceleration_limited_pv_reference(
    float current_position, float current_velocity, float target_position,
    float maximum_velocity, float maximum_acceleration, float period_s) {
  const float error = target_position - current_position;
  const float maximum_velocity_from_distance = std::sqrt(
      std::max(0.0f, 2.0f * maximum_acceleration * std::abs(error)));
  const float desired_velocity = std::copysign(
      std::min(maximum_velocity, maximum_velocity_from_distance), error);
  const float maximum_velocity_change = maximum_acceleration * period_s;
  const float next_velocity = std::clamp(
      desired_velocity,
      current_velocity - maximum_velocity_change,
      current_velocity + maximum_velocity_change);
  const float next_position = current_position +
      0.5f * (current_velocity + next_velocity) * period_s;

  // Snap only when the remaining velocity can be removed within this cycle.
  // Larger target replacements are allowed to brake continuously through the
  // endpoint instead of creating an unbounded reference acceleration.
  const bool crossed = (error > 0.0f && next_position >= target_position) ||
      (error < 0.0f && next_position <= target_position);
  if ((std::abs(error) <= 1.0e-7f || crossed) &&
      std::abs(current_velocity) <= maximum_velocity_change) {
    return {target_position, 0.0f};
  }
  return {next_position, next_velocity};
}

// Feedback-distance Motor-V envelope. maximum_velocity is a hard ceiling;
// sqrt(2*a*distance) provides the physical braking ceiling.
inline float advance_ordinary_pv_drive_velocity(
    float current_velocity_limit, float remaining_distance,
    float maximum_velocity, float maximum_acceleration, float period_s) {
  if (!std::isfinite(current_velocity_limit) ||
      current_velocity_limit < 0.0f ||
      !std::isfinite(remaining_distance) || remaining_distance < 0.0f ||
      !std::isfinite(maximum_velocity) || maximum_velocity <= 0.0f ||
      !std::isfinite(maximum_acceleration) || maximum_acceleration <= 0.0f ||
      !std::isfinite(period_s) || period_s <= 0.0f) {
    return 0.0f;
  }
  const float braking_velocity = std::sqrt(
      std::max(0.0f, 2.0f * maximum_acceleration * remaining_distance));
  const float desired_velocity = std::min(maximum_velocity, braking_velocity);
  const float maximum_change = maximum_acceleration * period_s;
  const float bounded_current = std::min(
      current_velocity_limit, maximum_velocity);
  if (desired_velocity >= bounded_current) {
    return std::min(desired_velocity, bounded_current + maximum_change);
  }
  return std::max(desired_velocity, bounded_current - maximum_change);
}

inline std::string yunyi_joint_role(uint32_t index) {
  const uint32_t side = index / ARTICORE_PRODUCT_ARM_DOF;
  const uint32_t joint = index % ARTICORE_PRODUCT_ARM_DOF;
  return std::string(side == ARTICORE_ROBOT_LEFT ? "left/l-joint"
                                                 : "right/r-joint") +
      std::to_string(joint + 1);
}

struct NativeTrajectoryJoint {
  std::string role;
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
  // PV trajectories use their normal limit while moving, then switch to this
  // lower limit for the final stationary hold.
  float pv_hold_velocity_limit = 0.0f;
};

struct NativeTrajectoryWaypoint {
  double time_s = 0.0;
  std::vector<float> positions;
  std::vector<float> velocities;
  std::vector<float> accelerations;
  uint32_t velocity_valid_mask = 0;
  uint32_t acceleration_valid_mask = 0;
};

enum class NativeTrajectoryExecution {
  Quintic,
  // Internal-only finite trajectory execution through POS_VEL. A planner must
  // provide a finite, validated time-stamped reference sequence. Runtime linearly
  // resamples adjacent knots through POS_VEL on its 500 Hz control clock; there
  // is deliberately no public raw/streaming PV command that can select this
  // path.
  TrajectoryPv,
};

struct NativeTrajectoryRequest {
  ArticoreControlMode mode = ARTICORE_MODE_MIT;
  ArticoreRuntimeOperation operation = ARTICORE_OPERATION_NONE;
  NativeTrajectoryExecution execution = NativeTrajectoryExecution::Quintic;
  float pv_reference_velocity = 0.0f;
  float pv_reference_acceleration = 0.0f;
  float pv_drive_velocity_limit = 0.0f;
  bool allow_out_of_limit_start_recovery = false;
  // Optional leading Cartesian trajectory approach segments. Runtime freezes at
  // the final approach waypoint until fresh physical feedback is stable, then
  // continues the already validated path under the same motion id.
  uint32_t approach_segment_count = 0;
  // Optional estimated schedule markers used by native Cartesian trajectories.
  // A zero completion_deadline_s retains the raw joint-trajectory behavior where
  // last waypoint time is followed by the normal arrival timeout. Cartesian
  // trajectories set both values so status exposes the estimated total task
  // time. Physical arrival may extend past these markers, up to the separate
  // bounded arrival timeout.
  double approach_deadline_s = 0.0;
  double completion_deadline_s = 0.0;
  // Joints that physically participate in Cartesian tracking. A zero mask
  // means all trajectory joints for backwards-compatible native requests.
  uint32_t cartesian_tracking_joint_mask = 0;
  std::vector<NativeTrajectoryJoint> joints;
  std::vector<NativeTrajectoryWaypoint> waypoints;
  std::function<bool(const std::vector<float>&, std::string&)>
      approach_convergence_check;
  // Product-owned optional endpoint verification. The callback receives one
  // coherent logical-joint snapshot and must not retain it.
  std::function<bool(const std::vector<float>&, std::string&)>
      final_convergence_check;
};

struct NativeTrajectorySample {
  bool active = false;
  uint64_t motion_id = 0;
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
                std::shared_ptr<MotorBackend> backend,
                void* controller_group,
                void* left_controller,
                void* right_controller,
                std::vector<ArticoreMotorDescriptor> motors,
                bool require_gripper_product_profiles = false,
                uint32_t test_control_rate_override = 0);
  // Callback constructor retained only for deterministic native unit tests.
  // Yunyi product creation uses the native C++ MotorBackend overload above.
  SafetyRuntime(ArticoreRuntimeConfig config,
                ArticoreMotorApi api,
                void* controller_group,
                void* left_controller,
                void* right_controller,
                std::vector<ArticoreMotorDescriptor> motors,
                ArticoreControllerCallFn controller_enable_all = nullptr,
                ArticoreControllerCallFn motor_enable = nullptr,
                bool require_gripper_product_profiles = false,
                ArticoreMotorMaintenanceApi maintenance_api = {},
                uint32_t test_control_rate_override = 0);
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
  int32_t configure_mode_for_connect(ArticoreControlMode mode);
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
  // Legacy Motion-ID calls may enqueue after planning against the FIFO tail.
  // The simple public move_* surface plans against the current reference and
  // rejects a second active finite motion.
  using CommandTransaction = std::unique_lock<std::mutex>;
  CommandTransaction begin_command_transaction();
  uint64_t begin_command_planning(
      const CommandTransaction& transaction,
      bool allow_trajectory = false);
  void cancel_command_planning(uint64_t token) noexcept;
  uint64_t start_trajectory(NativeTrajectoryRequest request,
                            uint64_t replace_motion_id = 0,
                            CommandTransaction* transaction = nullptr,
                            bool enqueue = false,
                            uint64_t planning_token = 0);
  NativeTrajectorySample trajectory_sample() const;
  NativeTrajectorySample planned_arm_sample(
      const std::vector<NativeTrajectoryJoint>& joints,
      const CommandTransaction& transaction) const;
  NativeTrajectorySample planned_trajectory_tail_sample(
      const std::vector<NativeTrajectoryJoint>& joints,
      const CommandTransaction& transaction) const;
  ArticoreMotionStatus motion_status() const;
  ArticoreMotionStatus motion_status(uint64_t motion_id) const;
  bool motion_arrived() const;
  void cancel_motion(uint64_t motion_id);
  void cancel_all_motions();
  void set_joint_mit(const ArticoreJointMitTarget* targets,
                     uint32_t count,
                     float max_reference_velocity);
  void set_joint_mit_direct(const ArticoreJointMitTarget* targets,
                            uint32_t count);
  void set_joint_mit_planned(
      const ArticoreJointMitTarget* targets, uint32_t count,
      float max_reference_velocity,
      CommandTransaction& transaction, uint64_t planning_token);
  void set_joint_mit_direct_planned(
      const ArticoreJointMitTarget* targets, uint32_t count,
      CommandTransaction& transaction, uint64_t planning_token);
  void set_joint_pv(const ArticoreJointPvTarget* targets,
                    uint32_t count,
                    float max_reference_velocity);
  // Ordinary PV sends final P directly. The explicit velocity limit is the
  // maximum Damiao POS_VEL V value shaped by the acceleration parameter.
  void set_joint_pv(const ArticoreJointPvTarget* targets,
                    uint32_t count,
                    float max_reference_velocity,
                    float pv_velocity_limit);
  void set_joint_pv(const ArticoreJointPvTarget* targets,
                    uint32_t count,
                    float max_reference_velocity,
                    float max_reference_acceleration,
                    float pv_velocity_limit);
  void set_joint_pv_planned(
      const ArticoreJointPvTarget* targets, uint32_t count,
      float max_reference_velocity, float max_reference_acceleration,
      float pv_velocity_limit,
      CommandTransaction& transaction, uint64_t planning_token);
  // Product ordinary PV uses independent limits for every joint. The vectors
  // follow target order and describe the current 1..100 percent time scale.
  void set_joint_pv_profile(
      const ArticoreJointPvTarget* targets, uint32_t count,
      const std::vector<float>& maximum_velocities,
      const std::vector<float>& maximum_accelerations);
  void set_joint_pv_profile_planned(
      const ArticoreJointPvTarget* targets, uint32_t count,
      const std::vector<float>& maximum_velocities,
      const std::vector<float>& maximum_accelerations,
      CommandTransaction& transaction, uint64_t planning_token);
  void set_joint_mit_speed(const ArticoreJointMitTarget* targets,
                           uint32_t count, float speed_percent);
  void set_joint_pv_speed(const ArticoreJointPvTarget* targets,
                          uint32_t count, float speed_percent);
  void update_joint_pv_motion_limits(float max_reference_velocity,
                                     float max_reference_acceleration,
                                     float pv_velocity_limit);
  void update_joint_pv_profile_limits(
      const std::vector<float>& maximum_velocities,
      const std::vector<float>& maximum_accelerations);
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
  void start_bimanual_follow(uint32_t leader_side);
  void stop_bimanual_follow();
  ArticoreBimanualFollowStatus bimanual_follow_status() const;
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
  uint32_t control_hz() const noexcept { return control_hz_; }
  ArticoreControlMode control_mode() const;
  uint64_t feedback_max_age_ns() const noexcept {
    return static_cast<uint64_t>(config_.feedback_max_age_ms) * 1'000'000ULL;
  }
  void declare_motor_presence(const std::string& role,
                              ArticorePresenceState state);
  ArticorePresenceState motor_presence(const std::string& role) const;
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
    float mit_fast_follow_kp = 0.0f;
    float mit_fast_follow_kd = 0.0f;
    float velocity_feedback_scale = 1.0f;
    bool layered_limits_configured = false;
  };

  struct ArmMailbox {
    bool valid = false;
    bool user_command = false;
    ArticoreCommandLifetime lifetime = ARTICORE_COMMAND_STREAMING;
    uint64_t generation = 0;
    uint64_t sent_generation = 0;
    Clock::time_point submitted_at{};
    // Ordinary PV/MIT own the latest endpoint without creating a trajectory
    // task. Product PV sends final P directly and shapes only Motor V.
    bool joint_position = false;
    // Direct MIT sends the newest validated endpoint without generating
    // intermediate position references. Stepped MIT retains the legacy
    // reference generator used by the fast-follow product API.
    bool mit_direct = false;
    bool pv_per_joint_profile = false;
    float max_reference_velocity = 0.0f;
    float max_reference_acceleration = 0.0f;
    float pv_velocity_limit = 0.0f;
    std::vector<ArticorePosVelCommand> pv;
    std::vector<ArticoreMitCommand> mit;
    std::vector<float> final_positions;
    std::vector<float> pv_reference_positions;
    std::vector<float> pv_reference_velocities;
    std::vector<float> pv_reference_velocity_limits;
    std::vector<float> pv_reference_acceleration_limits;
    std::vector<float> pv_drive_velocity_commands;
    // Ordinary PV has no motion status object, but it still needs the same
    // quiet final hold as native Cartesian paths.
    // Each joint transitions
    // to V=0 only after fresh feedback remains close to its final target for a
    // bounded ordinary-control window; a larger error or a new high-frequency
    // endpoint releases it for correction. The target anchor keeps cumulative
    // sub-quantum publisher motion from being mistaken for a stationary target.
    std::vector<uint16_t> pv_hold_confirmation_cycles;
    std::vector<uint8_t> pv_stationary_hold;
    std::vector<float> pv_hold_target_positions;
  };

  struct GravityArm {
    uint32_t runtime_side = 0;
    uint32_t robot_side = 0;
    std::string product_id;
    std::unique_ptr<RobotModel> model;
    std::array<void*, 7> joints{};
    std::array<float, 7> position_directions{};
    std::array<float, 7> torque_command_scales{};
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

  struct BimanualFollowControl {
    bool active = false;
    uint32_t leader_side = ARTICORE_ROBOT_LEFT;
    std::vector<float> start_positions;
    std::array<float, 7> follower_reference{};
    std::array<float, 7> follower_reference_velocity{};
    ArticoreBimanualFollowStatus status{};
  };

  struct TrajectorySegment {
    double start_s = 0.0;
    double duration_s = 0.0;
    std::vector<std::array<double, 6>> coefficients;
  };

  struct TrajectoryControl {
    ArticoreMotionState state = ARTICORE_MOTION_IDLE;
    uint64_t id = 0;
    uint32_t waypoint_count = 0;
    uint32_t active_segment = 0;
    double elapsed_s = 0.0;
    double duration_s = 0.0;
    double reference_duration_s = 0.0;
    double approach_duration_s = 0.0;
    double approach_deadline_s = 0.0;
    Clock::time_point started_at{};
    Clock::time_point tracking_updated_at{};
    Clock::time_point tracking_pause_started_at{};
    Clock::time_point settling_started_at{};
    Clock::time_point settling_stable_started_at{};
    Clock::time_point hold_unstable_started_at{};
    ArticoreRuntimeOperation operation = ARTICORE_OPERATION_NONE;
    NativeTrajectoryExecution execution = NativeTrajectoryExecution::Quintic;
    float pv_reference_velocity = 0.0f;
    float pv_reference_acceleration = 0.0f;
    float pv_drive_velocity_limit = 0.0f;
    uint32_t approach_segment_count = 0;
    uint32_t cartesian_tracking_joint_mask = 0;
    bool approach_complete = true;
    std::vector<NativeTrajectoryJoint> joints;
    std::vector<TrajectorySegment> segments;
    std::function<bool(const std::vector<float>&, std::string&)>
        final_convergence_check;
    std::function<bool(const std::vector<float>&, std::string&)>
        approach_convergence_check;
    std::vector<uint64_t> settling_feedback_updates;
    std::vector<float> settling_position_min;
    std::vector<float> settling_position_max;
    uint32_t settled_feedback_samples = 0;
    bool settling_feedback_initialized = false;
    bool stationary_hold_active = false;
    bool final_hold_limit_active = false;
    uint32_t final_hold_limit_mask = 0;
    float tracking_time_scale = 1.0f;
    float tracking_position_error = 0.0f;
    bool tracking_feedback_valid = false;
    double cartesian_reference_updated_elapsed_s = 0.0;
    std::vector<float> cartesian_reference_positions;
    Clock::time_point trajectory_pv_updated_at{};
    std::vector<float> trajectory_pv_reference_positions;
    std::vector<float> trajectory_pv_reference_velocities;
    std::vector<float> trajectory_pv_command_positions;
    std::string tracking_worst_role;
    std::string error;
  };

  struct ControlTraceSample {
    uint64_t sequence = 0;
    uint64_t timestamp_ns = 0;
    int32_t runtime_state = ARTICORE_DISCONNECTED;
    int32_t motion_state = ARTICORE_MOTION_IDLE;
    uint64_t motion_id = 0;
    float progress = 0.0f;
    float tracking_time_scale = 1.0f;
    float tracking_position_error = 0.0f;
    bool command_transmitted = false;
    std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> planned_positions{};
    std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> planned_velocities{};
    std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> command_positions{};
    std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> pv_velocity_limits{};
    std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> actual_positions{};
    std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> actual_velocities{};
    uint32_t planned_valid_mask = 0;
    uint32_t command_valid_mask = 0;
    uint32_t actual_valid_mask = 0;
  };

  void worker_loop();
  bool run_arm_control_cycle(Clock::time_point now, bool include_grippers,
                             std::string& error);
  bool run_gravity_control_cycle(Clock::time_point now,
                                 bool include_grippers,
                                 std::string& error);
  void reset_bimanual_follow_locked();
  bool prepare_trajectory_cycle(Clock::time_point now,
                                bool& completing,
                                std::string& error);
  void update_trajectory_completion(Clock::time_point now);
  void terminate_trajectory_locked(ArticoreMotionState state,
                                   const std::string& error);
  void activate_trajectory_locked(TrajectoryControl trajectory,
                                  Clock::time_point now);
  void archive_trajectory_locked(const TrajectoryControl& trajectory);
  ArticoreMotionStatus motion_status_locked(
      const TrajectoryControl& trajectory) const;
  NativeTrajectorySample trajectory_final_sample_locked(
      const TrajectoryControl& trajectory) const;
  NativeTrajectorySample trajectory_sample_locked(Clock::time_point now) const;
  void fault_trajectory(const std::string& error);
  void record_control_trace(Clock::time_point now, ArticoreControlMode mode,
                            const ArticorePosVelCommand* pv_commands,
                            uint32_t pv_count,
                            const ArticoreMitCommand* mit_commands,
                            uint32_t mit_count,
                            bool command_transmitted);
  void write_control_trace() noexcept;
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
                                 bool allow_trajectory = false,
                                 uint64_t planning_token = 0) const;
  void validate_motor_set(const ArticorePosVelCommand* commands,
                          uint32_t count, bool grippers_only) const;
  void validate_motor_set(const ArticoreMitCommand* commands,
                          uint32_t count, bool grippers_only) const;
  void install_joint_position(
      ArticoreControlMode mode,
      const std::vector<std::pair<void*, float>>& targets,
      float max_reference_velocity,
      float max_reference_acceleration = 0.0f,
      float pv_velocity_limit = 0.0f,
      CommandTransaction* transaction = nullptr,
      uint64_t planning_token = 0,
      const std::vector<float>* pv_reference_velocity_limits = nullptr,
      const std::vector<float>* pv_reference_acceleration_limits = nullptr,
      bool mit_direct = false);
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
  std::vector<ArticoreMotorFeedbackHealth>
  collect_motor_feedback_health() const;
  ArticoreFeedbackIssueScope classify_feedback_issue_scope(
      const std::vector<ArticoreMotorFeedbackHealth>& motors) const;
  bool refresh_transport_health(std::string& error);
  void seed_gripper_targets_from_feedback(bool activate);
  std::string motor_error(const std::string& fallback) const;
  void set_side_error_locked(uint8_t side, const std::string& error,
                             bool send_failure);
  void mark_motor_faulted(void* motor);
  int32_t maintenance_precheck(ArticoreRuntimeOperation operation,
                               std::string& error,
                               std::vector<std::string>& failed_motors,
                               bool require_stationary);
  int32_t finish_maintenance(ArticoreRuntimeOperation operation,
                             int32_t code,
                             const std::string& error,
                             const std::vector<std::string>& failed_motors,
                             bool latch_fault);
  int32_t run_motor_maintenance(ArticoreRuntimeOperation operation,
                                ArticoreControlMode mode,
                                bool require_stationary);
  int32_t configure_hardware_mode(
      ArticoreControlMode mode, std::string& error,
      std::vector<std::string>& failed_motors);
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
  uint32_t control_hz_ = 500;
  std::shared_ptr<MotorBackend> backend_;
  void* controller_group_ = nullptr;
  void* controllers_[2]{};
  bool require_gripper_product_profiles_ = false;
  bool active_sides_[2]{};
  std::vector<MotorRecord> motors_;
  std::map<std::string, ArticorePresenceState> presence_;
  std::map<void*, std::string> motor_roles_;
  std::unordered_map<void*, JointControlConfig> joint_configs_;
  std::vector<GravityArm> gravity_arms_;
  GravityControl gravity_control_;
  BimanualFollowControl bimanual_follow_;
  TrajectoryControl trajectory_control_;
  std::deque<TrajectoryControl> trajectory_queue_;
  std::deque<ArticoreMotionStatus> trajectory_history_;
  uint64_t next_motion_id_ = 1;
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
  bool first_command_accepted_ = false;
  bool enable_grace_transition_ = false;
  uint64_t active_command_planning_token_ = 0;
  uint64_t next_command_planning_token_ = 1;
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
  std::atomic<uint64_t> control_ticks_{0};
  std::atomic<uint64_t> control_overruns_{0};
  std::atomic<uint64_t> maximum_control_period_ns_{0};
  std::atomic<uint64_t> maximum_send_time_ns_{0};
  Clock::time_point last_control_tick_{};
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
  std::string control_trace_path_;
  std::vector<ControlTraceSample> control_trace_;
  uint64_t control_trace_sequence_ = 0;
  bool control_trace_written_ = false;
};

}  // namespace articore

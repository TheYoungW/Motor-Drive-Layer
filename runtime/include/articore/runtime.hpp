#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace articore {

inline constexpr std::size_t kArmDof = 7;
inline constexpr std::size_t kRobotDof = 14;
inline constexpr std::size_t kPoseDof = 6;

using JointArray = std::array<float, kRobotDof>;
using ArmArray = std::array<float, kArmDof>;
using Pose = std::array<float, kPoseDof>;

enum class RuntimeErrorCode : std::uint8_t {
  Ok = 0,
  InvalidArgument,
  WrongState,
  Busy,
  Timeout,
  TransportError,
  InternalError,
};

enum class ControlMode : std::uint8_t { Pv = 1, Mit = 2 };

enum class SafetyState : std::uint8_t {
  Disconnected = 0,
  Ready,
  Enabled,
  Running,
  SafeHold,
  Fault,
  Degraded,
  SafeStop,
  PartiallyEnabled,
};

enum class RobotSide : std::uint8_t { Left = 0, Right = 1 };
enum class GripperMode : std::uint8_t { Protected = 0, Direct = 1 };

class Status {
 public:
  Status() = default;
  Status(RuntimeErrorCode code, std::string message)
      : code_(code), message_(std::move(message)) {}

  static Status success() { return {}; }
  static Status failure(RuntimeErrorCode code, std::string message) {
    return {code, std::move(message)};
  }

  explicit operator bool() const noexcept {
    return code_ == RuntimeErrorCode::Ok;
  }
  RuntimeErrorCode code() const noexcept { return code_; }
  const std::string& message() const noexcept { return message_; }

 private:
  RuntimeErrorCode code_ = RuntimeErrorCode::Ok;
  std::string message_;
};

template <typename T>
class Result {
 public:
  Result(T value) : value_(std::move(value)) {}
  Result(Status error) : error_(std::move(error)) {}

  explicit operator bool() const noexcept { return value_.has_value(); }
  const Status& status() const noexcept { return error_; }
  T& value() & { return value_.value(); }
  const T& value() const& { return value_.value(); }
  T&& value() && { return std::move(value_.value()); }

 private:
  std::optional<T> value_;
  Status error_;
};

struct ThreadPolicy {
  bool realtime = true;
  int control_cpu = -1;
  int can_tx_cpu = -1;
  int can_rx_cpu = -1;
  int control_priority = 80;
  int can_tx_priority = 75;
  int can_rx_priority = 70;
  bool lock_memory = true;
};

struct YunyiRuntimeConfig {
  std::string left_can_interface = "can-left";
  std::string right_can_interface = "can-right";
  ControlMode initial_control_mode = ControlMode::Mit;
  bool with_grippers = true;
  ThreadPolicy threads{};
  std::chrono::milliseconds feedback_max_age{250};
  std::chrono::milliseconds motor_watchdog{500};
};

struct RuntimeState {
  JointArray positions{};
  JointArray velocities{};
  JointArray torques{};
  JointArray mos_temperatures{};
  JointArray rotor_temperatures{};
  std::uint32_t enabled_mask = 0;
  std::uint32_t enabled_valid_mask = 0;
  std::uint32_t temperature_valid_mask = 0;
  bool has_grippers = false;
  std::array<float, 2> gripper_openings{};
  std::array<bool, 2> gripper_available{};
  bool motion_arrived = false;
  std::chrono::nanoseconds timestamp{};
  std::uint64_t sequence = 0;
};

struct RuntimeMetrics {
  std::uint64_t ticks = 0;
  std::uint64_t overruns = 0;
  std::chrono::nanoseconds maximum_period{};
  std::chrono::nanoseconds maximum_send_time{};
};

struct RuntimeHealth {
  SafetyState state = SafetyState::Disconnected;
  bool safe_holding = false;
  bool disable_confirmed = false;
  bool degraded = false;
  bool safe_stopped = false;
  bool requires_resynchronization = false;
  std::uint32_t consecutive_send_failures = 0;
  std::uint32_t consecutive_feedback_failures = 0;
  std::string fault_reason;
  std::string safety_reason;
  std::string last_operation_error;
  std::vector<std::string> motor_faults;
  std::vector<std::string> operation_failed_motors;
  RuntimeMetrics metrics{};
};

struct MitCommand {
  JointArray positions{};
  JointArray velocities{};
  JointArray kp{};
  JointArray kd{};
  JointArray feedforward_torques{};
};

struct JointLimits {
  JointArray lower_angles{};
  JointArray upper_angles{};
  JointArray velocity_limits{};
};

struct GravityCompensationStatus {
  std::uint32_t phase = 0;
  bool active = false;
  float transition_progress = 0.0f;
  std::uint64_t control_cycles = 0;
  JointArray feedforward_torques{};
};

struct BimanualFollowStatus {
  std::uint32_t phase = 0;
  bool active = false;
  RobotSide leader = RobotSide::Left;
  float transition_progress = 0.0f;
  std::uint64_t control_cycles = 0;
  ArmArray leader_positions{};
  ArmArray follower_target_positions{};
  float maximum_tracking_error = 0.0f;
  std::string error;
};

// Product facade used by the DDS boundary. It owns every Motor, Controller,
// model and control worker through one private implementation; no handle or
// layout crosses the process boundary.
class YunyiRuntime final {
 public:
  static Result<std::unique_ptr<YunyiRuntime>> create(
      YunyiRuntimeConfig config = {});
  ~YunyiRuntime();

  YunyiRuntime(const YunyiRuntime&) = delete;
  YunyiRuntime& operator=(const YunyiRuntime&) = delete;

  Status connect();
  Status disconnect();
  Status configure_mode(ControlMode mode);
  Result<ControlMode> control_mode() const;
  Status enable();
  Status disable();
  Status set_zero();
  Status clear_faults();
  Status estop();
  Status recover();
  Status on_control_lost(const std::string& reason);

  Status set_joint_pv(const JointArray& positions, float speed_percent);
  Status set_joint_mit(const MitCommand& command);
  Status set_joint_mit_fast(const JointArray& positions,
                            float speed_percent = 100.0f);
  Status set_speed_percent(float speed_percent);
  Status set_max_speed(float radians_per_second);
  Status set_max_acceleration(float radians_per_second_squared);
  Result<float> speed_percent() const;
  Result<float> max_speed() const;
  Result<float> max_acceleration() const;
  Result<bool> has_grippers() const;

  Result<JointArray> solve_ik(const Pose& left, const Pose& right) const;
  Status move_pose(RobotSide side, const Pose& target);
  Status move_linear(RobotSide side, const Pose& target);
  Status move_circular(RobotSide side, const Pose& start, const Pose& via,
                       const Pose& end);
  Status stop_motion();

  Status set_grippers(float left_opening, float right_opening,
                      int strength = 5,
                      GripperMode mode = GripperMode::Protected);
  Status set_tcp_offset(RobotSide side, const Pose& offset);
  Status reset_tcp_offset(RobotSide side);
  Result<Pose> pose(RobotSide side) const;
  Result<Pose> tcp_offset(RobotSide side) const;
  Result<JointLimits> joint_limits() const;
  Status start_gravity_compensation(
      std::chrono::milliseconds transition = std::chrono::milliseconds{0});
  Status stop_gravity_compensation();
  Status start_bimanual_follow(RobotSide leader);
  Status stop_bimanual_follow();
  Result<GravityCompensationStatus> gravity_compensation_status() const;
  Result<BimanualFollowStatus> bimanual_follow_status() const;

  Result<RuntimeState> state() const;
  Result<RuntimeHealth> health() const;
  const YunyiRuntimeConfig& config() const noexcept;

 private:
  class Impl;
  explicit YunyiRuntime(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace articore

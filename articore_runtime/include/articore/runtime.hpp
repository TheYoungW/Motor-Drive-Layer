#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "articore/runtime_abi.h"

namespace articore {

class RuntimeError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

namespace detail {

inline void check(int32_t result, const char* operation) {
  if (result == 0) return;
  const char* reason = articore_runtime_last_error();
  throw RuntimeError(std::string(operation) + " failed: " +
                     (reason ? reason : "unknown Runtime error"));
}

inline uint32_t size(const std::vector<float>& values) {
  return static_cast<uint32_t>(values.size());
}

}  // namespace detail

// Product-level C++ wrapper for the only supported robot: Yunyi dual arm.
// Controllers, Motors, product calibration, models and worker lifetimes are
// created and owned entirely by the native Runtime.
class Runtime final {
 public:
  explicit Runtime(ArticoreControlMode mode = ARTICORE_MODE_MIT,
                   bool with_grippers = true) {
    runtime_ = articore_runtime_create_yunyi(
        static_cast<int32_t>(mode), with_grippers ? 1 : 0);
    if (!runtime_) {
      const char* reason = articore_runtime_last_error();
      throw RuntimeError(std::string("create Yunyi Runtime failed: ") +
                         (reason ? reason : "unknown Runtime error"));
    }
  }

  ~Runtime() noexcept { release(); }

  Runtime(const Runtime&) = delete;
  Runtime& operator=(const Runtime&) = delete;

  Runtime(Runtime&& other) noexcept
      : runtime_(std::exchange(other.runtime_, nullptr)) {}

  Runtime& operator=(Runtime&& other) noexcept {
    if (this == &other) return *this;
    release();
    runtime_ = std::exchange(other.runtime_, nullptr);
    return *this;
  }

  ArticoreRuntime* native_handle() const noexcept { return runtime_; }
  explicit operator bool() const noexcept { return runtime_ != nullptr; }

  void connect() {
    detail::check(articore_runtime_connect(checked()), "connect");
  }

  void disconnect() {
    if (!runtime_) return;
    const auto result = articore_runtime_disconnect(runtime_);
    articore_runtime_free(runtime_);
    runtime_ = nullptr;
    detail::check(result, "disconnect");
  }

  ArticoreControlMode control_mode() const {
    int32_t mode = 0;
    detail::check(
        articore_runtime_get_control_mode(checked(), &mode),
        "get_control_mode");
    return static_cast<ArticoreControlMode>(mode);
  }

  void configure_mode(ArticoreControlMode mode) {
    detail::check(
        articore_runtime_configure_mode(checked(), static_cast<int32_t>(mode)),
        "configure_mode");
  }

  bool enable() {
    detail::check(
        articore_runtime_enable(checked(), static_cast<int32_t>(control_mode())),
        "enable");
    return true;
  }

  bool disable() {
    detail::check(articore_runtime_disable(checked()), "disable");
    return true;
  }

  bool set_zero() {
    detail::check(articore_runtime_set_zero(checked()), "set_zero");
    return true;
  }

  void clear_faults() {
    detail::check(articore_runtime_clear_faults(checked()), "clear_faults");
  }

  void estop() {
    detail::check(articore_runtime_estop(checked()), "estop");
  }

  void recover() {
    detail::check(articore_runtime_recover(checked()), "recover");
  }

  bool has_grippers() const {
    int32_t result = 0;
    detail::check(
        articore_runtime_has_grippers(checked(), &result), "has_grippers");
    return result != 0;
  }

  void set_max_speed(float max_speed_percent) {
    detail::check(
        articore_runtime_set_max_speed(checked(), max_speed_percent),
        "set_max_speed");
  }

  float get_max_speed() const {
    float result = 0.0f;
    detail::check(
        articore_runtime_get_max_speed(checked(), &result), "get_max_speed");
    return result;
  }

  [[deprecated("legacy ordinary-motion compatibility API")]]
  void set_speed(float speed_percent) {
    detail::check(
        articore_runtime_set_speed(checked(), speed_percent), "set_speed");
  }

  [[deprecated("legacy ordinary-motion compatibility API")]]
  float get_speed() const {
    float result = 0.0f;
    detail::check(
        articore_runtime_get_speed(checked(), &result), "get_speed");
    return result;
  }

  void set_joint_positions(const std::vector<float>& positions) {
    detail::check(
        articore_runtime_set_joint_positions_v2(
            checked(), positions.data(), detail::size(positions)),
        "set_joint_positions_v2");
  }

  void set_joint_positions(const std::vector<float>& positions,
                           float speed_percent) {
    detail::check(
        articore_runtime_set_joint_positions(
            checked(), positions.data(), detail::size(positions),
            speed_percent),
        "set_joint_positions");
  }

  void submit_mit_frame(const std::vector<float>& positions,
                        const std::vector<float>& velocities,
                        const std::vector<float>& feedforward_torques,
                        const std::vector<float>& kp,
                        const std::vector<float>& kd) {
    detail::check(
        articore_runtime_submit_mit_frame(
            checked(), positions.data(), velocities.data(),
            feedforward_torques.data(), kp.data(), kd.data(),
            detail::size(positions)),
        "submit_mit_frame");
  }

  void start_trajectory(
      const std::vector<ArticoreTrajectoryWaypoint>& waypoints,
      const ArticoreTrajectoryConfig& config) {
    detail::check(
        articore_runtime_start_trajectory(
            checked(), waypoints.data(),
            static_cast<uint32_t>(waypoints.size()), &config),
        "start_trajectory");
  }

  ArticoreTrajectoryStatus trajectory_status() const {
    ArticoreTrajectoryStatus result{};
    result.struct_size = sizeof(result);
    detail::check(
        articore_runtime_get_trajectory_status(checked(), &result),
        "get_trajectory_status");
    return result;
  }

  void cancel_trajectory() {
    detail::check(
        articore_runtime_cancel_trajectory(checked()), "cancel_trajectory");
  }

  uint64_t move_pose(uint32_t side, const std::array<float, 6>& target_pose,
                     float speed_percent = 100.0f) {
    uint64_t motion_id = 0;
    detail::check(
        articore_runtime_move_pose(
            checked(), side, target_pose.data(), speed_percent, &motion_id),
        "move_pose");
    return motion_id;
  }

  uint64_t move_cartesian(
      uint32_t side, const std::array<float, 6>& target_pose,
      ArticoreCartesianInterpolation interpolation,
      float speed_percent = 100.0f) {
    uint64_t motion_id = 0;
    detail::check(
        articore_runtime_move_cartesian(
            checked(), side, target_pose.data(), speed_percent,
            static_cast<int32_t>(interpolation), &motion_id),
        "move_cartesian");
    return motion_id;
  }

  uint64_t move_linear(uint32_t side,
                       const std::array<float, 6>& target_pose,
                       float speed_percent = 100.0f) {
    uint64_t motion_id = 0;
    detail::check(
        articore_runtime_move_linear(
            checked(), side, target_pose.data(), speed_percent, &motion_id),
        "move_linear");
    return motion_id;
  }

  uint64_t move_circular(
      uint32_t side,
      const std::array<float, 6>& start_pose,
      const std::array<float, 6>& via_pose,
      const std::array<float, 6>& end_pose,
      float speed_percent = 100.0f) {
    uint64_t motion_id = 0;
    detail::check(
        articore_runtime_move_circular(
            checked(), side, start_pose.data(), via_pose.data(),
            end_pose.data(), speed_percent, &motion_id),
        "move_circular");
    return motion_id;
  }

  uint64_t move_circular(
      uint32_t side,
      const std::array<float, 6>& via_pose,
      const std::array<float, 6>& end_pose,
      float speed_percent = 100.0f) {
    uint64_t motion_id = 0;
    detail::check(
        articore_runtime_move_circular_v2(
            checked(), side, via_pose.data(), end_pose.data(),
            speed_percent, &motion_id),
        "move_circular_v2");
    return motion_id;
  }

  ArticoreMovePoseStatus move_pose_status() const {
    ArticoreMovePoseStatus result{};
    result.struct_size = sizeof(result);
    detail::check(
        articore_runtime_get_move_pose_status(checked(), &result),
        "get_move_pose_status");
    return result;
  }

  void cancel_move_pose() {
    detail::check(
        articore_runtime_cancel_move_pose(checked()), "cancel_move_pose");
  }

  ArticoreCartesianMotionStatus cartesian_motion_status() const {
    ArticoreCartesianMotionStatus result{};
    result.struct_size = sizeof(result);
    detail::check(
        articore_runtime_get_cartesian_motion_status(checked(), &result),
        "get_cartesian_motion_status");
    return result;
  }

  void cancel_cartesian_motion() {
    detail::check(
        articore_runtime_cancel_cartesian_motion(checked()),
        "cancel_cartesian_motion");
  }

  void set_grippers(float left_opening, float right_opening,
                    int32_t gripper_level = ARTICORE_GRIPPER_FORCE_DEFAULT) {
    detail::check(
        articore_runtime_set_grippers(
            checked(), left_opening, right_opening, gripper_level),
        "set_grippers");
  }

  void set_grippers(float left_opening, float right_opening,
                    int32_t strength, ArticoreGripperMode mode) {
    detail::check(
        articore_runtime_set_grippers_v2(
            checked(), left_opening, right_opening, strength,
            static_cast<int32_t>(mode)),
        "set_grippers_v2");
  }

  ArticoreProductState state() const {
    ArticoreProductState result{};
    result.struct_size = sizeof(result);
    detail::check(articore_runtime_get_state(checked(), &result), "get_state");
    return result;
  }

  ArticoreProductStateV2 state_v2() const {
    ArticoreProductStateV2 result{};
    result.struct_size = sizeof(result);
    detail::check(
        articore_runtime_get_state_v2(checked(), &result), "get_state_v2");
    return result;
  }

  ArticoreProductStateV3 state_v3() const {
    ArticoreProductStateV3 result{};
    result.struct_size = sizeof(result);
    detail::check(
        articore_runtime_get_state_v3(checked(), &result), "get_state_v3");
    return result;
  }

  ArticoreProductJointAngleVelLimits joint_angle_vel_limits() const {
    ArticoreProductJointAngleVelLimits result{};
    result.struct_size = sizeof(result);
    detail::check(
        articore_runtime_get_joint_angle_vel_limits(checked(), &result),
        "get_joint_angle_vel_limits");
    return result;
  }

  ArticoreProductPose pose(uint32_t side) const {
    ArticoreProductPose result{};
    result.struct_size = sizeof(result);
    detail::check(
        articore_runtime_get_pose(checked(), side, &result), "get_pose");
    return result;
  }

  ArticoreSafetyHealthV2 health() const {
    ArticoreSafetyHealthV2 result{};
    result.struct_size = sizeof(result);
    detail::check(
        articore_runtime_get_health_v2(checked(), &result), "get_health");
    return result;
  }

  ArticoreMotorPowerState set_motor_power(const std::string& motor_name,
                                          bool enable_motor) {
    int32_t result = ARTICORE_MOTOR_POWER_UNKNOWN;
    detail::check(
        articore_runtime_set_motor_power(
            checked(), motor_name.c_str(), enable_motor ? 1 : 0, &result),
        "set_motor_power");
    return static_cast<ArticoreMotorPowerState>(result);
  }

  ArticoreMotorPowerState motor_power_state(
      const std::string& motor_name) const {
    int32_t result = ARTICORE_MOTOR_POWER_UNKNOWN;
    detail::check(
        articore_runtime_get_motor_power(
            checked(), motor_name.c_str(), &result),
        "get_motor_power");
    return static_cast<ArticoreMotorPowerState>(result);
  }

  void start_gravity_compensation(uint32_t transition_ms = 0) {
    ArticoreGravityCompensationConfig config{};
    config.struct_size = sizeof(config);
    config.transition_ms = transition_ms;
    detail::check(
        articore_runtime_start_gravity_compensation(checked(), &config),
        "start_gravity_compensation");
  }

  void stop_gravity_compensation() {
    detail::check(
        articore_runtime_stop_gravity_compensation(checked()),
        "stop_gravity_compensation");
  }

  ArticoreGravityCompensationStatus gravity_compensation_status() const {
    ArticoreGravityCompensationStatus result{};
    result.struct_size = sizeof(result);
    detail::check(
        articore_runtime_get_gravity_compensation_status(checked(), &result),
        "get_gravity_compensation_status");
    return result;
  }

 private:
  ArticoreRuntime* checked() const {
    if (!runtime_) throw RuntimeError("Yunyi Runtime is disconnected");
    return runtime_;
  }

  void release() noexcept {
    if (!runtime_) return;
    (void)articore_runtime_disconnect(runtime_);
    articore_runtime_free(runtime_);
    runtime_ = nullptr;
  }

  ArticoreRuntime* runtime_ = nullptr;
};

}  // namespace articore

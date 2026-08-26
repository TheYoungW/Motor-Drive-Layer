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
    detail::check(
        articore_runtime_create_yunyi(
            static_cast<int32_t>(mode), with_grippers ? 1 : 0, &runtime_),
        "create Yunyi Runtime");
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
    detail::check(articore_runtime_enable(checked()), "enable");
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

  void set_max_speed(float max_speed_rad_s) {
    detail::check(
        articore_runtime_set_max_speed(checked(), max_speed_rad_s),
        "set_max_speed");
  }

  float get_max_speed() const {
    float result = 0.0f;
    detail::check(
        articore_runtime_get_max_speed(checked(), &result), "get_max_speed");
    return result;
  }

  void set_max_acceleration(float max_acceleration_rad_s2) {
    detail::check(
        articore_runtime_set_max_acceleration(
            checked(), max_acceleration_rad_s2),
        "set_max_acceleration");
  }

  float get_max_acceleration() const {
    float result = 0.0f;
    detail::check(
        articore_runtime_get_max_acceleration(checked(), &result),
        "get_max_acceleration");
    return result;
  }

  void set_joint_pv(const std::vector<float>& positions,
                    float speed_percent) {
    detail::check(
        articore_runtime_set_joint_pv(
            checked(), positions.data(), detail::size(positions),
            speed_percent),
        "set_joint_pv");
  }

  void set_joint_mit(const std::vector<float>& positions,
                     float speed_percent) {
    detail::check(
        articore_runtime_set_joint_mit(
            checked(), positions.data(), detail::size(positions),
            speed_percent),
        "set_joint_mit");
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

  uint64_t start_trajectory(
      const std::vector<ArticoreTrajectoryWaypoint>& waypoints,
      const ArticoreTrajectoryConfig& config) {
    uint64_t motion_id = 0;
    detail::check(
        articore_runtime_start_trajectory(
            checked(), waypoints.data(),
            static_cast<uint32_t>(waypoints.size()), &config, &motion_id),
        "start_trajectory");
    return motion_id;
  }

  ArticoreMotionStatus motion_status(uint64_t motion_id) const {
    ArticoreMotionStatus result{};
    result.struct_size = sizeof(result);
    detail::check(
        articore_runtime_get_motion_status(checked(), motion_id, &result),
        "get_motion_status");
    return result;
  }

  void cancel_motion(uint64_t motion_id) {
    detail::check(
        articore_runtime_cancel_motion(checked(), motion_id),
        "cancel_motion");
  }

  void cancel_all_motions() {
    detail::check(
        articore_runtime_cancel_all_motions(checked()),
        "cancel_all_motions");
  }

  void move_pose(const std::array<float, 6>& left_target_pose,
                 const std::array<float, 6>& right_target_pose,
                 float speed_percent = 50.0f) {
    detail::check(
        articore_runtime_move_pose(
            checked(), left_target_pose.data(), right_target_pose.data(),
            speed_percent),
        "move_pose");
  }

  uint64_t move_linear(uint32_t side,
                       const std::array<float, 6>& start_pose,
                       const std::array<float, 6>& end_pose,
                       double duration_s) {
    uint64_t motion_id = 0;
    detail::check(
        articore_runtime_move_linear(
            checked(), side, start_pose.data(), end_pose.data(),
            duration_s, &motion_id),
        "move_linear");
    return motion_id;
  }

  uint64_t move_circular(
      uint32_t side,
      const std::array<float, 6>& start_pose,
      const std::array<float, 6>& via_pose,
      const std::array<float, 6>& end_pose,
      double duration_s) {
    uint64_t motion_id = 0;
    detail::check(
        articore_runtime_move_circular(
            checked(), side, start_pose.data(), via_pose.data(),
            end_pose.data(), duration_s, &motion_id),
        "move_circular");
    return motion_id;
  }

  void set_grippers(float left_opening, float right_opening,
                    int32_t strength = ARTICORE_GRIPPER_STRENGTH_DEFAULT,
                    ArticoreGripperMode mode = ARTICORE_GRIPPER_MODE_PROTECTED) {
    detail::check(
        articore_runtime_set_grippers(
            checked(), left_opening, right_opening, strength,
            static_cast<int32_t>(mode)),
        "set_grippers");
  }

  ArticoreProductState state() const {
    ArticoreProductState result{};
    result.struct_size = sizeof(result);
    detail::check(articore_runtime_get_state(checked(), &result), "get_state");
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

  void set_tcp_offset(uint32_t side,
                      const std::array<float, ARTICORE_PRODUCT_POSE_DOF>& values) {
    ArticoreTcpOffset offset{};
    offset.struct_size = sizeof(offset);
    offset.side = side;
    std::copy(values.begin(), values.end(), offset.values);
    detail::check(
        articore_runtime_set_tcp_offset(checked(), &offset), "set_tcp_offset");
  }

  ArticoreTcpOffset tcp_offset(uint32_t side) const {
    ArticoreTcpOffset result{};
    result.struct_size = sizeof(result);
    detail::check(
        articore_runtime_get_tcp_offset(checked(), side, &result),
        "get_tcp_offset");
    return result;
  }

  void reset_tcp_offset(uint32_t side) {
    detail::check(
        articore_runtime_reset_tcp_offset(checked(), side),
        "reset_tcp_offset");
  }

  ArticoreSafetyHealth health() const {
    ArticoreSafetyHealth result{};
    result.struct_size = sizeof(result);
    detail::check(articore_runtime_get_health(checked(), &result),
                  "get_health");
    return result;
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

  void start_bimanual_follow(uint32_t leader_side) {
    detail::check(
        articore_runtime_start_bimanual_follow(checked(), leader_side),
        "start_bimanual_follow");
  }

  void stop_bimanual_follow() {
    detail::check(
        articore_runtime_stop_bimanual_follow(checked()),
        "stop_bimanual_follow");
  }

  ArticoreBimanualFollowStatus bimanual_follow_status() const {
    ArticoreBimanualFollowStatus result{};
    result.struct_size = sizeof(result);
    detail::check(
        articore_runtime_get_bimanual_follow_status(checked(), &result),
        "get_bimanual_follow_status");
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

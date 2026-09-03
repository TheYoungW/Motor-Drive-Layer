#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "articore/detail/native_types.hpp"

struct YunyiRuntimeCoreState;

namespace articore {

// Internal C++ product core used only by YunyiRuntime. It owns the complete
// SafetyRuntime/resource lifetime and is never installed as an SDK surface.
class YunyiRuntimeCore final {
 public:
  YunyiRuntimeCore(
      ArticoreControlMode mode, const std::string& left_can_interface,
      const std::string& right_can_interface, bool realtime,
      bool lock_memory, int control_cpu, int can_tx_cpu, int can_rx_cpu,
      int control_priority, int can_tx_priority, int can_rx_priority,
      uint32_t feedback_max_age_ms, uint32_t motor_watchdog_ms,
      uint32_t motor_discovery_timeout_ms,
      uint32_t motor_discovery_retries);
  ~YunyiRuntimeCore() noexcept;

  YunyiRuntimeCore(const YunyiRuntimeCore&) = delete;
  YunyiRuntimeCore& operator=(const YunyiRuntimeCore&) = delete;
  YunyiRuntimeCore(YunyiRuntimeCore&& other) noexcept;
  YunyiRuntimeCore& operator=(YunyiRuntimeCore&& other) noexcept;

  void connect();
  void disconnect();
  ArticoreControlMode control_mode() const;
  void configure_mode(ArticoreControlMode mode);
  void enable();
  void disable();
  void set_zero();
  void clear_faults();
  void estop();
  void recover();

  bool has_grippers() const;
  std::array<bool, 2> gripper_presence() const;
  void set_joint_pv(const std::vector<float>& positions, float speed_percent);
  void set_joint_mit(
      const std::vector<float>& positions,
      const std::vector<float>& velocities,
      const std::vector<float>& kp,
      const std::vector<float>& kd,
      const std::vector<float>& feedforward_torques);
  void set_joint_mit_fast(
      const std::vector<float>& positions, float speed_percent);
  void set_speed_percent(float speed_percent);
  float speed_percent() const;
  void set_max_speed(float max_speed_rad_s);
  float max_speed() const;
  void set_max_acceleration(float max_acceleration_rad_s2);
  float max_acceleration() const;

  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> solve_ik(
      const std::array<float, ARTICORE_PRODUCT_POSE_DOF>& left_target_pose,
      const std::array<float, ARTICORE_PRODUCT_POSE_DOF>& right_target_pose)
      const;
  void move_pose(
      uint32_t side,
      const std::array<float, ARTICORE_PRODUCT_POSE_DOF>& target_pose);
  void move_linear(
      uint32_t side,
      const std::array<float, ARTICORE_PRODUCT_POSE_DOF>& end_pose);
  void move_circular(
      uint32_t side,
      const std::array<float, ARTICORE_PRODUCT_POSE_DOF>& start_pose,
      const std::array<float, ARTICORE_PRODUCT_POSE_DOF>& via_pose,
      const std::array<float, ARTICORE_PRODUCT_POSE_DOF>& end_pose);
  void stop_motion();

  void set_grippers(float left_opening, float right_opening,
                    int32_t strength, ArticoreGripperMode mode);
  ArticoreProductState state() const;
  ArticoreProductJointAngleVelLimits joint_angle_vel_limits() const;
  ArticoreProductPose pose(uint32_t side) const;
  void set_tcp_offset(
      uint32_t side,
      const std::array<float, ARTICORE_PRODUCT_POSE_DOF>& values);
  ArticoreTcpOffset tcp_offset(uint32_t side) const;
  void reset_tcp_offset(uint32_t side);
  ArticoreSafetyHealth health() const;

  void start_gravity_compensation(uint32_t transition_ms);
  void stop_gravity_compensation();
  ArticoreGravityCompensationStatus gravity_compensation_status() const;
  void start_bimanual_follow(uint32_t leader_side);
  void stop_bimanual_follow();
  ArticoreBimanualFollowStatus bimanual_follow_status() const;

 private:
  YunyiRuntimeCoreState* checked() const;
  void release() noexcept;

  std::unique_ptr<YunyiRuntimeCoreState> state_;
};

}  // namespace articore

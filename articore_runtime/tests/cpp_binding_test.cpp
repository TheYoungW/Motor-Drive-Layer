#include <type_traits>
#include "articore/runtime.hpp"

int main() {
  static_assert(!std::is_copy_constructible_v<articore::Runtime>);
  static_assert(!std::is_copy_assignable_v<articore::Runtime>);
  static_assert(std::is_move_constructible_v<articore::Runtime>);
  static_assert(std::is_constructible_v<
                articore::Runtime, ArticoreControlMode, bool>);
  using JointLimitsGetter =
      ArticoreProductJointAngleVelLimits (articore::Runtime::*)() const;
  static_assert(std::is_same_v<
      decltype(&articore::Runtime::joint_angle_vel_limits),
      JointLimitsGetter>);
  using SpeedGetter = float (articore::Runtime::*)() const;
  using SpeedSetter = void (articore::Runtime::*)(float);
  static_assert(std::is_same_v<
      decltype(&articore::Runtime::get_max_acceleration), SpeedGetter>);
  static_assert(std::is_same_v<
      decltype(&articore::Runtime::set_max_acceleration), SpeedSetter>);
  using JointPvSetter =
      void (articore::Runtime::*)(const std::vector<float>&, float);
  using JointMitSetter =
      void (articore::Runtime::*)(const std::vector<float>&, float);
  static_assert(std::is_same_v<decltype(&articore::Runtime::set_joint_pv),
                               JointPvSetter>);
  static_assert(std::is_same_v<decltype(&articore::Runtime::set_joint_mit),
                               JointMitSetter>);
  using MoveJointTrajectory = uint64_t (articore::Runtime::*)(
      const std::vector<ArticoreTrajectoryWaypoint>&,
      const ArticoreTrajectoryConfig&);
  static_assert(std::is_same_v<
      decltype(&articore::Runtime::move_joint_trajectory),
      MoveJointTrajectory>);
  using SetPose = void (articore::Runtime::*)(
      const std::array<float, 6>&, const std::array<float, 6>&, float);
  static_assert(std::is_same_v<decltype(&articore::Runtime::set_pose),
                               SetPose>);
  using MoveLinearTrajectory = uint64_t (articore::Runtime::*)(
      uint32_t, const std::array<float, 6>&,
      const std::array<float, 6>&, double);
  static_assert(std::is_same_v<
      decltype(static_cast<MoveLinearTrajectory>(
          &articore::Runtime::move_linear_trajectory)),
      MoveLinearTrajectory>);
  using MoveLinearPathTrajectory = uint64_t (articore::Runtime::*)(
      uint32_t, const std::vector<std::array<float, 6>>&, double);
  static_assert(std::is_same_v<
      decltype(static_cast<MoveLinearPathTrajectory>(
          &articore::Runtime::move_linear_trajectory)),
      MoveLinearPathTrajectory>);
  using MoveCircularTrajectory = uint64_t (articore::Runtime::*)(
      uint32_t, const std::array<float, 6>&,
      const std::array<float, 6>&, const std::array<float, 6>&, double);
  static_assert(std::is_same_v<decltype(&articore::Runtime::move_circular_trajectory),
                               MoveCircularTrajectory>);

  // This target is a cross-platform compile/link smoke test for the public
  // RAII wrapper. Runtime behavior and invalid construction are exercised by
  // runtime_test through the C ABI without fabricated native handles.
  return 0;
}

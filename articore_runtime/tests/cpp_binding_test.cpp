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
  using JointPvSetter =
      void (articore::Runtime::*)(const std::vector<float>&, float);
  using JointMitSetter = void (articore::Runtime::*)(
      const std::vector<float>&, const std::vector<float>&,
      const std::vector<float>&, const std::vector<float>&,
      const std::vector<float>&);
  using JointMitFastSetter =
      void (articore::Runtime::*)(const std::vector<float>&, float);
  static_assert(std::is_same_v<decltype(&articore::Runtime::set_joint_pv),
                               JointPvSetter>);
  using PvLimitSetter = void (articore::Runtime::*)(float);
  using PvLimitGetter = float (articore::Runtime::*)() const;
  static_assert(std::is_same_v<decltype(&articore::Runtime::set_speed_percent),
                               PvLimitSetter>);
  static_assert(std::is_same_v<decltype(&articore::Runtime::speed_percent),
                               PvLimitGetter>);
  static_assert(std::is_same_v<decltype(&articore::Runtime::set_max_speed),
                               PvLimitSetter>);
  static_assert(std::is_same_v<decltype(&articore::Runtime::max_speed),
                               PvLimitGetter>);
  static_assert(std::is_same_v<
      decltype(&articore::Runtime::set_max_acceleration), PvLimitSetter>);
  static_assert(std::is_same_v<decltype(&articore::Runtime::max_acceleration),
                               PvLimitGetter>);
  static_assert(std::is_same_v<
      decltype(&articore::Runtime::set_joint_mit),
      JointMitSetter>);
  static_assert(std::is_same_v<
      decltype(&articore::Runtime::set_joint_mit_fast),
      JointMitFastSetter>);
  using SolveIk = std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>
      (articore::Runtime::*)(
          const std::array<float, ARTICORE_PRODUCT_POSE_DOF>&,
          const std::array<float, ARTICORE_PRODUCT_POSE_DOF>&) const;
  static_assert(std::is_same_v<decltype(&articore::Runtime::solve_ik), SolveIk>);
  using MovePose = void (articore::Runtime::*)(
      uint32_t, const std::array<float, 6>&);
  static_assert(std::is_same_v<decltype(&articore::Runtime::move_pose),
                               MovePose>);
  using MoveLinear = void (articore::Runtime::*)(
      uint32_t, const std::array<float, 6>&,
      const std::array<float, 6>&);
  static_assert(std::is_same_v<
      decltype(static_cast<MoveLinear>(&articore::Runtime::move_linear)),
      MoveLinear>);
  using MoveLinearPath = void (articore::Runtime::*)(
      uint32_t, const std::vector<std::array<float, 6>>&);
  static_assert(std::is_same_v<
      decltype(static_cast<MoveLinearPath>(&articore::Runtime::move_linear)),
      MoveLinearPath>);
  using MoveCircular = void (articore::Runtime::*)(
      uint32_t, const std::array<float, 6>&,
      const std::array<float, 6>&, const std::array<float, 6>&);
  static_assert(std::is_same_v<decltype(&articore::Runtime::move_circular),
                               MoveCircular>);
  using StopMotion = void (articore::Runtime::*)();
  static_assert(std::is_same_v<decltype(&articore::Runtime::stop_motion),
                               StopMotion>);

  // This target is a cross-platform compile/link smoke test for the public
  // RAII wrapper. Runtime behavior and invalid construction are exercised by
  // runtime_test through the C ABI without fabricated native handles.
  return 0;
}

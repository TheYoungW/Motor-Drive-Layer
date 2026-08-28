#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <net/if.h>
#include <type_traits>

#include "articore/runtime_abi.h"

namespace {

template <typename Actual, typename Expected>
constexpr bool same_signature =
    std::is_same_v<std::remove_cv_t<Actual>, Expected>;

}  // namespace

int main() {
  using CreateYunyi = int32_t (*)(int32_t, int32_t, ArticoreRuntime**);
  using RuntimeCall = int32_t (*)(ArticoreRuntime*);
  using ProductCommand = int32_t (*)(
      ArticoreRuntime*, const float*, uint32_t, float);
  using ProductIk = int32_t (*)(
      ArticoreRuntime*, const float*, const float*, float*, uint32_t);
  using MoveJointTrajectory = int32_t (*)(
      ArticoreRuntime*, const ArticoreTrajectoryWaypoint*, uint32_t,
      const ArticoreTrajectoryConfig*, uint64_t*);
  using MoveLinearTrajectory = int32_t (*)(
      ArticoreRuntime*, uint32_t, const float*, const float*, double,
      uint64_t*);
  using MoveLinearTrajectoryWithPointCount = int32_t (*)(
      ArticoreRuntime*, uint32_t, const float*, const float*, double,
      uint32_t, uint64_t*);
  using MoveLinearPathTrajectory = int32_t (*)(
      ArticoreRuntime*, uint32_t, const float*, uint32_t, double, uint64_t*);
  using MoveCircularTrajectory = int32_t (*)(
      ArticoreRuntime*, uint32_t, const float*, const float*, const float*,
      double, uint64_t*);
  using MotionStatus = int32_t (*)(
      ArticoreRuntime*, uint64_t, ArticoreMotionStatus*);
  using ProductGrippers = int32_t (*)(
      ArticoreRuntime*, float, float, int32_t, int32_t);
  using ProductState = int32_t (*)(ArticoreRuntime*, ArticoreProductState*);
  using ProductHealth = int32_t (*)(ArticoreRuntime*, ArticoreSafetyHealth*);
  using MotorPowerBatch = int32_t (*)(
      ArticoreRuntime*, const char* const*, uint32_t,
      ArticoreMotorPowerReport*);

  static_assert(same_signature<
      decltype(&articore_runtime_create_yunyi), CreateYunyi>);
  static_assert(same_signature<decltype(&articore_runtime_enable), RuntimeCall>);
  static_assert(same_signature<
      decltype(&articore_runtime_set_joint_pv), ProductCommand>);
  static_assert(same_signature<
      decltype(&articore_runtime_set_joint_mit), ProductCommand>);
  static_assert(same_signature<
      decltype(&articore_runtime_solve_ik), ProductIk>);
  static_assert(same_signature<
      decltype(&articore_runtime_move_joint_trajectory),
      MoveJointTrajectory>);
  static_assert(same_signature<
      decltype(&articore_runtime_move_linear_trajectory), MoveLinearTrajectory>);
  static_assert(same_signature<
      decltype(&articore_runtime_move_linear_trajectory_with_point_count),
      MoveLinearTrajectoryWithPointCount>);
  static_assert(same_signature<
      decltype(&articore_runtime_move_linear_path_trajectory),
      MoveLinearPathTrajectory>);
  static_assert(same_signature<
      decltype(&articore_runtime_move_circular_trajectory), MoveCircularTrajectory>);
  static_assert(same_signature<
      decltype(&articore_runtime_get_motion_status), MotionStatus>);
  static_assert(same_signature<
      decltype(&articore_runtime_set_grippers), ProductGrippers>);
  static_assert(same_signature<
      decltype(&articore_runtime_get_state), ProductState>);
  static_assert(same_signature<
      decltype(&articore_runtime_get_health), ProductHealth>);
  static_assert(same_signature<
      decltype(&articore_runtime_enable_motors), MotorPowerBatch>);
  static_assert(same_signature<
      decltype(&articore_runtime_disable_motors), MotorPowerBatch>);

  static_assert(sizeof(ArticoreTrajectoryWaypoint) == 192);
  static_assert(sizeof(ArticoreTrajectoryConfig) == 236);
  static_assert(sizeof(ArticoreMotionStatus) == 568);
  static_assert(sizeof(ArticoreProductArmState) == 152);
  static_assert(sizeof(ArticoreProductState) == 392);
  static_assert(sizeof(ArticoreProductJointAngleVelLimits) == 176);
  static_assert(sizeof(ArticoreTcpOffset) == 32);
  static_assert(sizeof(ArticoreMotorFeedbackHealth) == 136);
  static_assert(ARTICORE_FEEDBACK_ISSUE_UNEXPECTED_POWER_STATE == (1U << 5));
  static_assert(ARTICORE_FEEDBACK_SCOPE_BOTH_CHANNELS == 5);

  bool gripper_validation = true;
  for (const int32_t strength : {0, 5, 10}) {
    gripper_validation = gripper_validation &&
        articore_runtime_set_grippers(
            nullptr, 0.0f, 1000.0f, strength,
            ARTICORE_GRIPPER_MODE_DIRECT) ==
            ARTICORE_OPERATION_INVALID_ARGUMENT &&
        std::strcmp(articore_runtime_last_error(), "runtime is null") == 0;
  }
  for (const int32_t strength : {-1, 11}) {
    gripper_validation = gripper_validation &&
        articore_runtime_set_grippers(
            nullptr, 0.0f, 1000.0f, strength,
            ARTICORE_GRIPPER_MODE_PROTECTED) ==
            ARTICORE_OPERATION_INVALID_ARGUMENT &&
        std::strcmp(
            articore_runtime_last_error(),
            "gripper strength must be in the range 0..10") == 0;
  }
  gripper_validation = gripper_validation &&
      articore_runtime_set_grippers(nullptr, 0.0f, 1000.0f, 5, 99) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      std::strcmp(
          articore_runtime_last_error(),
          "gripper mode must be PROTECTED or DIRECT") == 0;

  ArticoreProductState undersized_state{};
  undersized_state.struct_size = sizeof(undersized_state) - 1;
  const bool state_size_checked =
      articore_runtime_get_state(nullptr, &undersized_state) == -1 &&
      std::strcmp(
          articore_runtime_last_error(),
          "product state output is null or too small") == 0;
  ArticoreProductState null_runtime_state{};
  null_runtime_state.struct_size = sizeof(null_runtime_state);
  const bool state_runtime_checked =
      articore_runtime_get_state(nullptr, &null_runtime_state) == -1 &&
      std::strcmp(articore_runtime_last_error(), "runtime is null") == 0;

  ArticoreSafetyHealth undersized_health{};
  undersized_health.struct_size = sizeof(undersized_health) - 1;
  const bool health_size_checked =
      articore_runtime_get_health(nullptr, &undersized_health) == -1 &&
      std::strcmp(
          articore_runtime_last_error(),
          "health output is null or too small") == 0;

  ArticoreProductJointAngleVelLimits undersized_joint_limits{};
  undersized_joint_limits.struct_size = sizeof(undersized_joint_limits) - 1;
  const bool joint_limits_size_checked =
      articore_runtime_get_joint_angle_vel_limits(
          nullptr, &undersized_joint_limits) == -1 &&
      std::strcmp(
          articore_runtime_last_error(),
          "joint angle/velocity limits output is null or too small") == 0;

  float acceleration_rad_s2 = 0.0f;
  const bool maximum_acceleration_validation =
      articore_runtime_get_max_acceleration(nullptr, nullptr) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      std::strcmp(
          articore_runtime_last_error(),
          "max_acceleration_rad_s2 output is null") == 0 &&
      articore_runtime_get_max_acceleration(
          nullptr, &acceleration_rad_s2) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      std::strcmp(articore_runtime_last_error(), "runtime is null") == 0 &&
      articore_runtime_set_max_acceleration(nullptr, 0.0f) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      articore_runtime_set_max_acceleration(nullptr, 8.01f) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      articore_runtime_set_max_acceleration(nullptr, 4.005f) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      articore_runtime_set_max_acceleration(
          nullptr, std::numeric_limits<float>::quiet_NaN()) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT;

  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> positions{};
  std::array<float, ARTICORE_PRODUCT_POSE_DOF> pose{};
  const bool product_ik_validation =
      articore_runtime_solve_ik(
          nullptr, nullptr, pose.data(), positions.data(), positions.size()) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      std::strcmp(
          articore_runtime_last_error(),
          "solve_ik requires both left and right target poses") == 0 &&
      articore_runtime_solve_ik(
          nullptr, pose.data(), pose.data(), nullptr, positions.size()) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      std::strcmp(
          articore_runtime_last_error(), "solve_ik joint output is null") == 0 &&
      articore_runtime_solve_ik(
          nullptr, pose.data(), pose.data(), positions.data(),
          positions.size() - 1) == ARTICORE_OPERATION_INVALID_ARGUMENT &&
      std::strcmp(
          articore_runtime_last_error(),
          "solve_ik joint output count must be 14") == 0 &&
      articore_runtime_solve_ik(
          nullptr, pose.data(), pose.data(), positions.data(),
          positions.size()) == ARTICORE_OPERATION_INVALID_ARGUMENT &&
      std::strcmp(articore_runtime_last_error(), "runtime is null") == 0;
  const bool joint_command_validation =
      articore_runtime_set_joint_pv(
          nullptr, positions.data(), positions.size(), 50.0f) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      std::strcmp(
          articore_runtime_last_error(),
          "PV joint command: runtime is null") == 0 &&
      articore_runtime_set_joint_pv(
          nullptr, positions.data(), positions.size(), -0.1f) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      articore_runtime_set_joint_mit(
          nullptr, positions.data(), positions.size(), 50.0f) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      std::strcmp(
          articore_runtime_last_error(),
          "MIT joint command: runtime is null") == 0;

  ArticoreRuntime* invalid_runtime =
      reinterpret_cast<ArticoreRuntime*>(static_cast<uintptr_t>(1));
  const bool factory_validation =
      articore_runtime_create_yunyi(ARTICORE_MODE_PV, 1, nullptr) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      articore_runtime_create_yunyi(99, 1, &invalid_runtime) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      invalid_runtime == nullptr &&
      articore_runtime_create_yunyi(
          ARTICORE_MODE_PV, 2, &invalid_runtime) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      invalid_runtime == nullptr;

  ArticoreRuntime* metadata_runtime = nullptr;
  ArticoreProductJointAngleVelLimits limits{};
  limits.struct_size = sizeof(limits);
  float configured_acceleration = 0.0f;
  bool product_limits_checked = true;
  if (if_nametoindex("can-left") != 0 && if_nametoindex("can-right") != 0) {
    product_limits_checked =
        articore_runtime_create_yunyi(
            ARTICORE_MODE_PV, 0, &metadata_runtime) ==
            ARTICORE_OPERATION_OK &&
        metadata_runtime != nullptr &&
        articore_runtime_get_joint_angle_vel_limits(
            metadata_runtime, &limits) == ARTICORE_OPERATION_OK &&
        limits.joint_count == ARTICORE_PRODUCT_DUAL_ARM_DOF &&
        std::fabs(limits.lower_angles[0] + 2.745f) < 1e-6f &&
        std::fabs(limits.velocity_limits[0] - 5.0f) < 1e-6f &&
        articore_runtime_get_max_acceleration(
            metadata_runtime, &configured_acceleration) ==
            ARTICORE_OPERATION_OK &&
        std::fabs(configured_acceleration - 6.0f) < 1e-6f &&
        articore_runtime_set_max_acceleration(metadata_runtime, 4.56f) ==
            ARTICORE_OPERATION_OK &&
        articore_runtime_get_max_acceleration(
            metadata_runtime, &configured_acceleration) ==
            ARTICORE_OPERATION_OK &&
        std::fabs(configured_acceleration - 4.56f) < 1e-6f &&
        articore_runtime_set_max_acceleration(metadata_runtime, 4.565f) ==
            ARTICORE_OPERATION_INVALID_ARGUMENT;
  }
  articore_runtime_free(metadata_runtime);

  const bool symbols_present =
      &articore_runtime_configure_mode &&
      &articore_runtime_clear_faults && &articore_runtime_set_zero &&
      &articore_runtime_disconnect &&
      &articore_runtime_set_max_acceleration &&
      &articore_runtime_get_max_acceleration &&
      &articore_runtime_submit_mit_frame &&
      &articore_runtime_move_joint_trajectory &&
      &articore_runtime_get_motion_status &&
      &articore_runtime_cancel_motion &&
      &articore_runtime_cancel_all_motions &&
      &articore_runtime_solve_ik &&
      &articore_runtime_set_pose &&
      &articore_runtime_move_linear_path_trajectory &&
      &articore_runtime_move_linear_trajectory_with_point_count &&
      &articore_runtime_has_grippers &&
      &articore_runtime_get_joint_angle_vel_limits &&
      &articore_runtime_get_pose && &articore_runtime_set_tcp_offset &&
      &articore_runtime_get_tcp_offset &&
      &articore_runtime_reset_tcp_offset &&
      &articore_runtime_get_control_mode &&
      &articore_runtime_get_mit_torque_limit_stats &&
      &articore_runtime_start_gravity_compensation &&
      &articore_runtime_stop_gravity_compensation &&
      &articore_runtime_get_gravity_compensation_status &&
      &articore_runtime_start_bimanual_follow &&
      &articore_runtime_stop_bimanual_follow &&
      &articore_runtime_get_bimanual_follow_status &&
      &articore_runtime_estop && &articore_runtime_recover &&
      &articore_robot_model_create && &articore_robot_model_fk;

  if (articore_runtime_abi_version() != 0x000B0003U ||
      !symbols_present || !gripper_validation || !state_size_checked ||
      !state_runtime_checked || !health_size_checked ||
      !joint_limits_size_checked || !maximum_acceleration_validation ||
      !product_ik_validation || !joint_command_validation ||
      !factory_validation || !product_limits_checked) {
    std::cerr << "Articore Runtime ABI 11.3 contract is incomplete\n";
    return 1;
  }
  std::cout << "Articore Runtime ABI 11.3 smoke test passed\n";
  return 0;
}

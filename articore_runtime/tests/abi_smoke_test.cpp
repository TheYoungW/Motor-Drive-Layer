#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <net/if.h>
#include <type_traits>

#include "articore/runtime_abi.h"

int main() {
  const auto create_yunyi = &articore_runtime_create_yunyi;
  using CreateYunyi = int32_t (*)(
      int32_t, int32_t, ArticoreRuntime**);
  static_assert(std::is_same_v<
      std::remove_cv_t<decltype(create_yunyi)>, CreateYunyi>);
  const auto configure_mode = &articore_runtime_configure_mode;
  const auto clear_faults = &articore_runtime_clear_faults;
  const auto set_zero = &articore_runtime_set_zero;
  const auto disconnect = &articore_runtime_disconnect;
  const auto product_pv = &articore_runtime_set_joint_pv;
  const auto product_mit = &articore_runtime_set_joint_mit;
  using ProductPv = int32_t (*)(
      ArticoreRuntime*, const float*, uint32_t, float);
  using ProductMit = int32_t (*)(
      ArticoreRuntime*, const float*, uint32_t, float);
  static_assert(std::is_same_v<
      std::remove_cv_t<decltype(product_pv)>, ProductPv>);
  static_assert(std::is_same_v<
      std::remove_cv_t<decltype(product_mit)>, ProductMit>);
  const auto set_product_max_speed = &articore_runtime_set_max_speed;
  const auto get_product_max_speed = &articore_runtime_get_max_speed;
  const auto product_mit_frame = &articore_runtime_submit_mit_frame;
  const auto start_trajectory = &articore_runtime_start_trajectory;
  const auto trajectory_status = &articore_runtime_get_trajectory_status;
  const auto cancel_trajectory = &articore_runtime_cancel_trajectory;
  const auto move_pose = &articore_runtime_move_pose;
  const auto move_poses = &articore_runtime_move_poses;
  const auto move_linear = &articore_runtime_move_linear;
  const auto move_linear_v2 = &articore_runtime_move_linear_v2;
  const auto cartesian_motion_status =
      &articore_runtime_get_cartesian_motion_status;
  const auto cartesian_motion_status_v2 =
      &articore_runtime_get_cartesian_motion_status_v2;
  const auto cancel_cartesian_motion =
      &articore_runtime_cancel_cartesian_motion;
  const auto move_circular = &articore_runtime_move_circular;
  const auto move_circular_v2 = &articore_runtime_move_circular_v2;
  using StartTrajectory = int32_t (*)(
      ArticoreRuntime*, const ArticoreTrajectoryWaypoint*, uint32_t,
      const ArticoreTrajectoryConfig*);
  static_assert(std::is_same_v<
      std::remove_cv_t<decltype(start_trajectory)>, StartTrajectory>);
  static_assert(sizeof(ArticoreTrajectoryWaypoint) == 192);
  static_assert(sizeof(ArticoreTrajectoryConfig) == 236);
  static_assert(sizeof(ArticoreTrajectoryStatus) == 560);
  using MovePose = int32_t (*)(
      ArticoreRuntime*, uint32_t, const float*, float);
  static_assert(std::is_same_v<
      std::remove_cv_t<decltype(move_pose)>, MovePose>);
  using MovePoses = int32_t (*)(
      ArticoreRuntime*, const float*, const float*, float);
  static_assert(std::is_same_v<
      std::remove_cv_t<decltype(move_poses)>, MovePoses>);
  using MoveLinear = int32_t (*)(
      ArticoreRuntime*, uint32_t, const float*, float, uint64_t*);
  static_assert(std::is_same_v<
      std::remove_cv_t<decltype(move_linear)>, MoveLinear>);
  using MoveLinearV2 = int32_t (*)(
      ArticoreRuntime*, uint32_t, const float*, const float*, float,
      uint64_t*);
  static_assert(std::is_same_v<
      std::remove_cv_t<decltype(move_linear_v2)>, MoveLinearV2>);
  static_assert(sizeof(ArticoreCartesianMotionStatus) == 600);
  using CartesianMotionStatusV2 = int32_t (*)(
      ArticoreRuntime*, uint64_t, ArticoreCartesianMotionStatus*);
  static_assert(std::is_same_v<
      std::remove_cv_t<decltype(cartesian_motion_status_v2)>,
      CartesianMotionStatusV2>);
  using MoveCircular = int32_t (*)(
      ArticoreRuntime*, uint32_t, const float*, const float*, const float*,
      float, uint64_t*);
  static_assert(std::is_same_v<
      std::remove_cv_t<decltype(move_circular)>, MoveCircular>);
  using MoveCircularV2 = int32_t (*)(
      ArticoreRuntime*, uint32_t, const float*, const float*, float,
      uint64_t*);
  static_assert(std::is_same_v<
      std::remove_cv_t<decltype(move_circular_v2)>, MoveCircularV2>);
  const auto product_grippers = &articore_runtime_set_grippers;
  const auto product_grippers_v2 = &articore_runtime_set_grippers_v2;
  using ProductGrippersV2 = int32_t (*)(
      ArticoreRuntime*, float, float, int32_t, int32_t);
  static_assert(std::is_same_v<
      std::remove_cv_t<decltype(product_grippers_v2)>, ProductGrippersV2>);
  const auto has_product_grippers = &articore_runtime_has_grippers;
  const auto product_state = &articore_runtime_get_state;
  const auto product_state_v2 = &articore_runtime_get_state_v2;
  const auto product_state_v3 = &articore_runtime_get_state_v3;
  const auto product_joint_limits =
      &articore_runtime_get_joint_angle_vel_limits;
  using ProductStateV2Getter = int32_t (*)(
      ArticoreRuntime*, ArticoreProductStateV2*);
  static_assert(std::is_same_v<
      std::remove_cv_t<decltype(product_state_v2)>, ProductStateV2Getter>);
  static_assert(sizeof(ArticoreProductStateV2) == 248);
  using ProductStateV3Getter = int32_t (*)(
      ArticoreRuntime*, ArticoreProductStateV3*);
  static_assert(std::is_same_v<
      std::remove_cv_t<decltype(product_state_v3)>, ProductStateV3Getter>);
  static_assert(sizeof(ArticoreProductArmStateV3) == 152);
  static_assert(sizeof(ArticoreProductStateV3) == 392);
  using ProductJointLimitsGetter = int32_t (*)(
      ArticoreRuntime*, ArticoreProductJointAngleVelLimits*);
  static_assert(std::is_same_v<
      std::remove_cv_t<decltype(product_joint_limits)>,
      ProductJointLimitsGetter>);
  static_assert(sizeof(ArticoreProductJointAngleVelLimits) == 176);
  const auto product_pose = &articore_runtime_get_pose;
  const auto set_tcp_offset = &articore_runtime_set_tcp_offset;
  const auto get_tcp_offset = &articore_runtime_get_tcp_offset;
  const auto reset_tcp_offset = &articore_runtime_reset_tcp_offset;
  static_assert(sizeof(ArticoreTcpOffset) == 32);
  const auto control_mode = &articore_runtime_get_control_mode;
  const auto enable_report = &articore_runtime_get_last_enable_report;
  const auto enable_motors = &articore_runtime_enable_motors;
  const auto disable_motors = &articore_runtime_disable_motors;
  using MotorPowerBatch = int32_t (*)(
      ArticoreRuntime*, const char* const*, uint32_t,
      ArticoreMotorPowerReport*);
  static_assert(std::is_same_v<
      std::remove_cv_t<decltype(enable_motors)>, MotorPowerBatch>);
  static_assert(std::is_same_v<
      std::remove_cv_t<decltype(disable_motors)>, MotorPowerBatch>);
  const auto set_motor_power = &articore_runtime_set_motor_power;
  const auto get_motor_power = &articore_runtime_get_motor_power;
  const auto configure_joint_safety_limits =
      &articore_runtime_configure_joint_safety_limits;
  const auto configure_gripper_force_profiles =
      &articore_runtime_configure_gripper_force_profiles;
  const auto configure_gripper_products =
      &articore_runtime_configure_gripper_products;
  const auto set_gripper_commands =
      &articore_runtime_set_gripper_commands;
  const auto disable_report = &articore_runtime_get_last_disable_report;
  const auto configure_motor_identities =
      &articore_runtime_configure_motor_identities;
  const auto connect_report = &articore_runtime_get_last_connect_report;
  const auto mit_torque_limit_stats =
      &articore_runtime_get_mit_torque_limit_stats;
  const auto robot_model_create = &articore_robot_model_create;
  const auto robot_model_fk = &articore_robot_model_fk;
  const auto configure_gravity_products =
      &articore_runtime_configure_gravity_products;
  const auto start_gravity = &articore_runtime_start_gravity_compensation;
  const auto stop_gravity = &articore_runtime_stop_gravity_compensation;
  const auto gravity_status =
      &articore_runtime_get_gravity_compensation_status;
  const auto start_bimanual_follow =
      &articore_runtime_start_bimanual_follow;
  const auto stop_bimanual_follow =
      &articore_runtime_stop_bimanual_follow;
  const auto bimanual_follow_status =
      &articore_runtime_get_bimanual_follow_status;
  const auto health_v2 = &articore_runtime_get_health_v2;
  auto estop = &articore_runtime_estop;
  using ParameterlessEstop = int32_t (*)(ArticoreRuntime*);
  static_assert(std::is_same_v<decltype(estop), ParameterlessEstop>);
  const auto version = articore_runtime_abi_version();
  const auto capabilities = articore_runtime_capabilities();
  const uint64_t required = ARTICORE_CAP_COMMAND_WATCHDOG |
                            ARTICORE_CAP_SAFE_HOLD |
                            ARTICORE_CAP_GRIPPER_PROTECTION |
                            ARTICORE_CAP_DUAL_CHANNEL |
                            ARTICORE_CAP_CURRENT_POSITION_HOLD |
                            ARTICORE_CAP_MOTOR_PRESENCE |
                            ARTICORE_CAP_REALTIME_JOINT_MAILBOX |
                            ARTICORE_CAP_ATOMIC_ENABLE |
                            ARTICORE_CAP_PROTECTIVE_FAULT_HOLD |
                            ARTICORE_CAP_DETERMINISTIC_DISABLE |
                            ARTICORE_CAP_LAYERED_JOINT_LIMITS |
                            ARTICORE_CAP_GRIPPER_COMMAND_PROFILES |
                            ARTICORE_CAP_GRIPPER_FORCE_10_LEVELS |
                            ARTICORE_CAP_JOINT_MIT_POSITION |
                            ARTICORE_CAP_JOINT_PV_POSITION |
                            ARTICORE_CAP_BUILTIN_GRIPPER_PRODUCT_PROFILES |
                            ARTICORE_CAP_CONNECT_FEEDBACK_BARRIER |
                            ARTICORE_CAP_STRUCTURED_CONNECT_REPORT |
                            ARTICORE_CAP_PER_CYCLE_MIT_TORQUE_LIMIT |
                            ARTICORE_CAP_NATIVE_ROBOT_MODEL |
                            ARTICORE_CAP_NATIVE_GRAVITY_COMPENSATION |
                            ARTICORE_CAP_RUNTIME_MAINTENANCE |
                            ARTICORE_CAP_PRODUCT_RUNTIME_FACTORY |
                            ARTICORE_CAP_UNIFIED_OPERATION_HEALTH |
                            ARTICORE_CAP_PRODUCT_COMMAND_FRAMES |
                            ARTICORE_CAP_PRODUCT_STATE |
                            ARTICORE_CAP_OPTIONAL_PRODUCT_GRIPPERS |
                            ARTICORE_CAP_GRADED_FEEDBACK_SAFETY |
                            ARTICORE_CAP_NORMALIZED_ORDINARY_SPEED |
                            ARTICORE_CAP_RUNTIME_MOTOR_POWER;
  const uint64_t required_with_pose_and_estop = required |
      ARTICORE_CAP_PRODUCT_POSE |
      ARTICORE_CAP_PARAMETERLESS_ESTOP |
      ARTICORE_CAP_PRODUCT_RECOVERY |
      ARTICORE_CAP_TERMINAL_PRODUCT_DISCONNECT |
      ARTICORE_CAP_YUNYI_DUAL_ARM_RUNTIME |
      ARTICORE_CAP_ATOMIC_MOTOR_POWER_BATCH |
      ARTICORE_CAP_PRODUCT_POWER_STATE_SNAPSHOT |
      ARTICORE_CAP_PRODUCT_QUINTIC_TRAJECTORY |
      ARTICORE_CAP_PRODUCT_GRIPPER_FORCE_10_LEVELS |
      ARTICORE_CAP_PRODUCT_GRIPPER_DIRECT_MODE |
      ARTICORE_CAP_FIXED_GRIPPER_MIT_MODE |
      ARTICORE_CAP_DIRECT_GRIPPER_GAIN_X10 |
      ARTICORE_CAP_PRODUCT_CARTESIAN_POINT_TO_POINT |
      ARTICORE_CAP_PRODUCT_CARTESIAN_LINEAR;
  const uint64_t required_with_circular = required_with_pose_and_estop |
      ARTICORE_CAP_PRODUCT_CARTESIAN_CIRCULAR |
      ARTICORE_CAP_PRODUCT_CARTESIAN_CIRCULAR_AUTO_START |
      ARTICORE_CAP_PRODUCT_TEMPERATURE_STATE |
      ARTICORE_CAP_LATCHED_ESTOP_POSITION_HOLD |
      ARTICORE_CAP_PRODUCT_JOINT_ANGLE_VEL_LIMITS |
      ARTICORE_CAP_PRODUCT_PV_COMMAND_SPEED |
      ARTICORE_CAP_PRODUCT_MAX_SPEED_SETTING |
      ARTICORE_CAP_PRODUCT_TOOL_CENTER_POSE |
      ARTICORE_CAP_DIRECT_CPP_MOTOR_CORE;
  bool product_gripper_levels_valid = true;
  for (const int32_t level : {1, 5, 10}) {
    product_gripper_levels_valid = product_gripper_levels_valid &&
        articore_runtime_set_grippers(nullptr, 0.0f, 1000.0f, level) ==
            ARTICORE_OPERATION_INVALID_ARGUMENT &&
        std::strcmp(articore_runtime_last_error(), "runtime is null") == 0;
  }
  for (const int32_t level : {0, 11}) {
    product_gripper_levels_valid = product_gripper_levels_valid &&
        articore_runtime_set_grippers(nullptr, 0.0f, 1000.0f, level) ==
            ARTICORE_OPERATION_INVALID_ARGUMENT &&
        std::strcmp(articore_runtime_last_error(),
                    "gripper_level must be in the range 1..10") == 0;
  }
  bool product_gripper_direct_valid = true;
  for (const int32_t strength : {0, 5, 10}) {
    product_gripper_direct_valid = product_gripper_direct_valid &&
        articore_runtime_set_grippers_v2(
            nullptr, 0.0f, 1000.0f, strength,
            ARTICORE_GRIPPER_MODE_DIRECT) ==
            ARTICORE_OPERATION_INVALID_ARGUMENT &&
        std::strcmp(articore_runtime_last_error(), "runtime is null") == 0;
  }
  for (const int32_t strength : {-1, 11}) {
    product_gripper_direct_valid = product_gripper_direct_valid &&
        articore_runtime_set_grippers_v2(
            nullptr, 0.0f, 1000.0f, strength,
            ARTICORE_GRIPPER_MODE_DIRECT) ==
            ARTICORE_OPERATION_INVALID_ARGUMENT &&
        std::strcmp(articore_runtime_last_error(),
                    "gripper strength must be in the range 0..10") == 0;
  }
  product_gripper_direct_valid = product_gripper_direct_valid &&
      articore_runtime_set_grippers_v2(nullptr, 0.0f, 1000.0f, 5, 99) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      std::strcmp(articore_runtime_last_error(),
                  "gripper mode must be PROTECTED or DIRECT") == 0;
  const uint64_t removed_trajectory_bits =
      (1ULL << 9) | (1ULL << 12) | (1ULL << 15) |
      (1ULL << 16) | (1ULL << 17);
  const uint64_t removed_public_rate_bits = (1ULL << 23) | (1ULL << 27);
  ArticoreProductStateV3 undersized_state_v3{};
  undersized_state_v3.struct_size = sizeof(undersized_state_v3) - 1;
  const bool state_v3_size_checked =
      articore_runtime_get_state_v3(nullptr, &undersized_state_v3) == -1 &&
      std::strcmp(articore_runtime_last_error(),
                  "product state v3 output is null or too small") == 0;
  ArticoreProductStateV3 null_runtime_state_v3{};
  null_runtime_state_v3.struct_size = sizeof(null_runtime_state_v3);
  const bool state_v3_runtime_checked =
      articore_runtime_get_state_v3(nullptr, &null_runtime_state_v3) == -1 &&
      std::strcmp(articore_runtime_last_error(), "runtime is null") == 0;
  ArticoreProductJointAngleVelLimits undersized_joint_limits{};
  undersized_joint_limits.struct_size = sizeof(undersized_joint_limits) - 1;
  const bool joint_limits_size_checked =
      articore_runtime_get_joint_angle_vel_limits(
          nullptr, &undersized_joint_limits) == -1 &&
      std::strcmp(
          articore_runtime_last_error(),
          "joint angle/velocity limits output is null or too small") == 0;
  ArticoreProductJointAngleVelLimits null_runtime_joint_limits{};
  null_runtime_joint_limits.struct_size = sizeof(null_runtime_joint_limits);
  const bool joint_limits_runtime_checked =
      articore_runtime_get_joint_angle_vel_limits(
          nullptr, &null_runtime_joint_limits) == -1 &&
      std::strcmp(articore_runtime_last_error(), "runtime is null") == 0;
  float speed_percent = 0.0f;
  const bool product_speed_validation_checked =
      articore_runtime_get_max_speed(nullptr, nullptr) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      std::strcmp(articore_runtime_last_error(),
                  "max_speed_percent output is null") == 0 &&
      articore_runtime_get_max_speed(nullptr, &speed_percent) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      std::strcmp(articore_runtime_last_error(), "runtime is null") == 0 &&
      articore_runtime_set_max_speed(nullptr, 70.0f) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      std::strcmp(articore_runtime_last_error(), "runtime is null") == 0 &&
      articore_runtime_set_max_speed(nullptr, -0.1f) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      std::strcmp(articore_runtime_last_error(),
                  "ordinary maximum speed must be finite and within 0..100") == 0 &&
      articore_runtime_set_max_speed(nullptr, 100.1f) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      std::strcmp(articore_runtime_last_error(),
                  "ordinary maximum speed must be finite and within 0..100") == 0 &&
      articore_runtime_set_max_speed(
          nullptr, std::numeric_limits<float>::quiet_NaN()) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      std::strcmp(articore_runtime_last_error(),
                  "ordinary maximum speed must be finite and within 0..100") == 0;
  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> joint_positions{};
  const bool product_joint_command_validation_checked =
      articore_runtime_set_joint_pv(
          nullptr, joint_positions.data(), joint_positions.size(), 50.0f) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      std::strcmp(articore_runtime_last_error(),
                  "PV joint command: runtime is null") == 0 &&
      articore_runtime_set_joint_pv(
          nullptr, joint_positions.data(), joint_positions.size(), -0.1f) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      std::strcmp(articore_runtime_last_error(),
                  "PV joint command: PV command speed must be finite "
                  "and within 0..100") == 0 &&
      articore_runtime_set_joint_pv(
          nullptr, joint_positions.data(), joint_positions.size(), 100.1f) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      std::strcmp(articore_runtime_last_error(),
                  "PV joint command: PV command speed must be finite "
                  "and within 0..100") == 0 &&
      articore_runtime_set_joint_pv(
          nullptr, joint_positions.data(), joint_positions.size(),
          std::numeric_limits<float>::quiet_NaN()) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      std::strcmp(articore_runtime_last_error(),
                  "PV joint command: PV command speed must be finite "
                  "and within 0..100") == 0 &&
      articore_runtime_set_joint_mit(
          nullptr, joint_positions.data(), joint_positions.size(), 50.0f) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      std::strcmp(articore_runtime_last_error(),
                  "MIT joint command: runtime is null") == 0;
  ArticoreRuntime* invalid_created_runtime =
      reinterpret_cast<ArticoreRuntime*>(static_cast<uintptr_t>(1));
  const bool factory_validation_checked =
      articore_runtime_create_yunyi(
          ARTICORE_MODE_PV, 1, nullptr) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      std::strcmp(articore_runtime_last_error(),
                  "runtime output is null") == 0 &&
      articore_runtime_create_yunyi(
          99, 1, &invalid_created_runtime) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      invalid_created_runtime == nullptr &&
      std::strcmp(articore_runtime_last_error(),
                  "unsupported Yunyi control mode") == 0 &&
      articore_runtime_create_yunyi(
          ARTICORE_MODE_PV, 2, &invalid_created_runtime) ==
          ARTICORE_OPERATION_INVALID_ARGUMENT &&
      invalid_created_runtime == nullptr &&
      std::strcmp(articore_runtime_last_error(),
                  "with_grippers must be 0 or 1") == 0;
  ArticoreRuntime* metadata_runtime = nullptr;
  const bool product_channels_available =
      if_nametoindex("can-left") != 0 && if_nametoindex("can-right") != 0;
  ArticoreProductJointAngleVelLimits product_limits{};
  product_limits.struct_size = sizeof(product_limits);
  bool product_joint_limits_checked = true;
  if (product_channels_available) {
    const bool metadata_runtime_created =
        articore_runtime_create_yunyi(
            ARTICORE_MODE_PV, 0, &metadata_runtime) == ARTICORE_OPERATION_OK &&
        metadata_runtime != nullptr;
    product_joint_limits_checked = metadata_runtime_created &&
        articore_runtime_get_joint_angle_vel_limits(
            metadata_runtime, &product_limits) == ARTICORE_OPERATION_OK &&
        product_limits.joint_count == ARTICORE_PRODUCT_DUAL_ARM_DOF &&
        std::fabs(product_limits.lower_angles[0] - -2.745f) < 1e-6f &&
        std::fabs(product_limits.upper_angles[0] - 2.745f) < 1e-6f &&
        std::fabs(product_limits.lower_angles[1] - -0.3489f) < 1e-6f &&
        std::fabs(product_limits.upper_angles[8] - 0.3489f) < 1e-6f &&
        std::fabs(product_limits.velocity_limits[0] - 5.0f) < 1e-6f &&
        std::fabs(product_limits.velocity_limits[13] - 5.0f) < 1e-6f;
  }
  if (metadata_runtime != nullptr) {
    articore_runtime_free(metadata_runtime);
  }
  if (!create_yunyi ||
      !configure_mode || !clear_faults || !set_zero || !disconnect ||
      !set_product_max_speed || !get_product_max_speed ||
      !product_mit || !product_pv || !product_mit_frame ||
      !start_trajectory || !trajectory_status || !cancel_trajectory ||
      !move_pose || !move_linear || !move_linear_v2 ||
      !cartesian_motion_status ||
      !move_poses || !cancel_cartesian_motion ||
      !move_circular || !move_circular_v2 ||
      !product_grippers || !product_grippers_v2 || !has_product_grippers ||
      !product_state ||
      !product_state_v2 ||
      !product_state_v3 ||
      !product_joint_limits ||
      !product_pose ||
      !set_tcp_offset || !get_tcp_offset || !reset_tcp_offset ||
      !control_mode ||
      !enable_report || !enable_motors || !disable_motors ||
      !set_motor_power || !get_motor_power ||
      !disable_report || !configure_motor_identities || !connect_report ||
      !mit_torque_limit_stats || !robot_model_create || !robot_model_fk ||
      !configure_gravity_products || !start_gravity || !stop_gravity ||
      !gravity_status || !start_bimanual_follow || !stop_bimanual_follow ||
      !bimanual_follow_status || !health_v2 || !estop ||
      !configure_joint_safety_limits || !configure_gripper_products ||
      !configure_gripper_force_profiles || !set_gripper_commands ||
      version != 0x00040001U ||
      (capabilities & required_with_circular) != required_with_circular ||
      !product_gripper_levels_valid ||
      !product_gripper_direct_valid ||
      !state_v3_size_checked || !state_v3_runtime_checked ||
      !joint_limits_size_checked || !joint_limits_runtime_checked ||
      !product_speed_validation_checked ||
      !product_joint_command_validation_checked ||
      !factory_validation_checked ||
      !product_joint_limits_checked ||
      (capabilities & removed_trajectory_bits) != 0 ||
      (capabilities & removed_public_rate_bits) != 0) {
    std::cerr << "Articore runtime ABI metadata is incomplete\n";
    return 1;
  }
  std::cout << "Articore runtime ABI smoke test passed\n";
  return 0;
}

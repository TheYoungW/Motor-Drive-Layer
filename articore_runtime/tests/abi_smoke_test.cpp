#include <cstdint>
#include <cstring>
#include <iostream>
#include <type_traits>

#include "articore/runtime_abi.h"

int main() {
  const auto create_ex = &articore_runtime_create_ex;
  const auto create_ex2 = &articore_runtime_create_ex2;
  const auto create_ex3 = &articore_runtime_create_ex3;
  const auto create_yunyi = &articore_runtime_create_yunyi;
  const auto create_product = &articore_runtime_create_product;
  const auto configure_mode = &articore_runtime_configure_mode;
  const auto clear_faults = &articore_runtime_clear_faults;
  const auto set_zero = &articore_runtime_set_zero;
  const auto disconnect = &articore_runtime_disconnect;
  const auto product_positions = &articore_runtime_set_joint_positions;
  const auto product_mit = &articore_runtime_submit_mit_frame;
  const auto product_pv = &articore_runtime_submit_pv_frame;
  const auto start_trajectory = &articore_runtime_start_trajectory;
  const auto trajectory_status = &articore_runtime_get_trajectory_status;
  const auto cancel_trajectory = &articore_runtime_cancel_trajectory;
  using StartTrajectory = int32_t (*)(
      ArticoreRuntime*, const ArticoreTrajectoryWaypoint*, uint32_t,
      const ArticoreTrajectoryConfig*);
  static_assert(std::is_same_v<
      std::remove_cv_t<decltype(start_trajectory)>, StartTrajectory>);
  static_assert(sizeof(ArticoreTrajectoryWaypoint) == 192);
  static_assert(sizeof(ArticoreTrajectoryConfig) == 236);
  static_assert(sizeof(ArticoreTrajectoryStatus) == 560);
  const auto product_grippers = &articore_runtime_set_grippers;
  const auto product_grippers_v2 = &articore_runtime_set_grippers_v2;
  using ProductGrippersV2 = int32_t (*)(
      ArticoreRuntime*, float, float, int32_t, int32_t);
  static_assert(std::is_same_v<
      std::remove_cv_t<decltype(product_grippers_v2)>, ProductGrippersV2>);
  const auto has_product_grippers = &articore_runtime_has_grippers;
  const auto product_state = &articore_runtime_get_state;
  const auto product_state_v2 = &articore_runtime_get_state_v2;
  using ProductStateV2Getter = int32_t (*)(
      ArticoreRuntime*, ArticoreProductStateV2*);
  static_assert(std::is_same_v<
      std::remove_cv_t<decltype(product_state_v2)>, ProductStateV2Getter>);
  static_assert(sizeof(ArticoreProductStateV2) == 248);
  const auto product_pose = &articore_runtime_get_pose;
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
  const auto submit_pos_vel_ex = &articore_runtime_submit_pos_vel_ex;
  const auto submit_mit_ex = &articore_runtime_submit_mit_ex;
  const auto set_joint_mit = &articore_runtime_set_joint_mit;
  const auto set_joint_pv = &articore_runtime_set_joint_pv;
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
                            ARTICORE_CAP_COMMAND_LIFETIME |
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
      ARTICORE_CAP_FIXED_GRIPPER_MIT_MODE;
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
  if (!create_ex || !create_ex2 || !create_ex3 || !create_yunyi ||
      !create_product ||
      !configure_mode || !clear_faults || !set_zero || !disconnect ||
      !product_positions || !product_mit || !product_pv ||
      !start_trajectory || !trajectory_status || !cancel_trajectory ||
      !product_grippers || !product_grippers_v2 || !has_product_grippers ||
      !product_state ||
      !product_state_v2 ||
      !product_pose ||
      !control_mode ||
      !enable_report || !enable_motors || !disable_motors ||
      !set_motor_power || !get_motor_power ||
      !submit_pos_vel_ex || !submit_mit_ex ||
      !set_joint_mit || !set_joint_pv || !disable_report ||
      !configure_motor_identities || !connect_report ||
      !mit_torque_limit_stats || !robot_model_create || !robot_model_fk ||
      !configure_gravity_products || !start_gravity || !stop_gravity ||
      !gravity_status || !health_v2 || !estop ||
      !configure_joint_safety_limits || !configure_gripper_products ||
      !configure_gripper_force_profiles || !set_gripper_commands ||
      version != 0x0002001AU ||
      (capabilities & required_with_pose_and_estop) !=
          required_with_pose_and_estop ||
      !product_gripper_levels_valid ||
      !product_gripper_direct_valid ||
      (capabilities & removed_trajectory_bits) != 0 ||
      (capabilities & removed_public_rate_bits) != 0) {
    std::cerr << "Articore runtime ABI metadata is incomplete\n";
    return 1;
  }
  std::cout << "Articore runtime ABI smoke test passed\n";
  return 0;
}

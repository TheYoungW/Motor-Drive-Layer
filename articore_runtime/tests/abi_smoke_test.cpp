#include <cstdint>
#include <iostream>

#include "articore/runtime_abi.h"

int main() {
  const auto create_ex = &articore_runtime_create_ex;
  const auto create_ex2 = &articore_runtime_create_ex2;
  const auto create_ex3 = &articore_runtime_create_ex3;
  const auto create_product = &articore_runtime_create_product;
  const auto configure_mode = &articore_runtime_configure_mode;
  const auto clear_faults = &articore_runtime_clear_faults;
  const auto set_zero = &articore_runtime_set_zero;
  const auto disconnect = &articore_runtime_disconnect;
  const auto product_positions = &articore_runtime_set_joint_positions;
  const auto product_mit = &articore_runtime_submit_mit_frame;
  const auto product_pv = &articore_runtime_submit_pv_frame;
  const auto product_grippers = &articore_runtime_set_grippers;
  const auto has_product_grippers = &articore_runtime_has_grippers;
  const auto product_state = &articore_runtime_get_state;
  const auto product_pose = &articore_runtime_get_pose;
  const auto control_mode = &articore_runtime_get_control_mode;
  const auto enable_report = &articore_runtime_get_last_enable_report;
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
  const auto effective_control_hz = &articore_runtime_get_control_hz;
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
  const auto version = articore_runtime_abi_version();
  const auto capabilities = articore_runtime_capabilities();
  const uint64_t required = ARTICORE_CAP_COMMAND_WATCHDOG |
                            ARTICORE_CAP_SAFE_HOLD |
                            ARTICORE_CAP_GRIPPER_PROTECTION |
                            ARTICORE_CAP_SINGLE_CHANNEL |
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
                            ARTICORE_CAP_EFFECTIVE_CONTROL_RATE |
                            ARTICORE_CAP_BUILTIN_GRIPPER_PRODUCT_PROFILES |
                            ARTICORE_CAP_CONNECT_FEEDBACK_BARRIER |
                            ARTICORE_CAP_STRUCTURED_CONNECT_REPORT |
                            ARTICORE_CAP_TRANSPORT_AWARE_CONTROL_RATE |
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
  const uint64_t required_with_pose = required |
      ARTICORE_CAP_PRODUCT_POSE;
  const uint64_t removed_trajectory_bits =
      (1ULL << 9) | (1ULL << 12) | (1ULL << 15) |
      (1ULL << 16) | (1ULL << 17);
  if (!create_ex || !create_ex2 || !create_ex3 || !create_product ||
      !configure_mode || !clear_faults || !set_zero || !disconnect ||
      !product_positions || !product_mit || !product_pv ||
      !product_grippers || !has_product_grippers || !product_state ||
      !product_pose ||
      !control_mode ||
      !enable_report || !set_motor_power || !get_motor_power ||
      !submit_pos_vel_ex || !submit_mit_ex ||
      !set_joint_mit || !set_joint_pv || !disable_report ||
      !effective_control_hz ||
      !configure_motor_identities || !connect_report ||
      !mit_torque_limit_stats || !robot_model_create || !robot_model_fk ||
      !configure_gravity_products || !start_gravity || !stop_gravity ||
      !gravity_status || !health_v2 ||
      !configure_joint_safety_limits || !configure_gripper_products ||
      !configure_gripper_force_profiles || !set_gripper_commands ||
      version != 0x0002000FU ||
      (capabilities & required_with_pose) != required_with_pose ||
      (capabilities & removed_trajectory_bits) != 0) {
    std::cerr << "Articore runtime ABI metadata is incomplete\n";
    return 1;
  }
  std::cout << "Articore runtime ABI smoke test passed\n";
  return 0;
}

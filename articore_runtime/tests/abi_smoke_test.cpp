#include <cstdint>
#include <iostream>

#include "articore/runtime_abi.h"

int main() {
  const auto create_ex = &articore_runtime_create_ex;
  const auto enable_report = &articore_runtime_get_last_enable_report;
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
                            ARTICORE_CAP_CONNECT_FEEDBACK_BARRIER;
  const uint64_t removed_trajectory_bits =
      (1ULL << 9) | (1ULL << 12) | (1ULL << 15) |
      (1ULL << 16) | (1ULL << 17);
  if (!create_ex || !enable_report || !submit_pos_vel_ex || !submit_mit_ex ||
      !set_joint_mit || !set_joint_pv || !disable_report ||
      !effective_control_hz ||
      !configure_joint_safety_limits || !configure_gripper_products ||
      !configure_gripper_force_profiles || !set_gripper_commands ||
      version != 0x00020003U ||
      (capabilities & required) != required ||
      (capabilities & removed_trajectory_bits) != 0) {
    std::cerr << "Articore runtime ABI metadata is incomplete\n";
    return 1;
  }
  std::cout << "Articore runtime ABI smoke test passed\n";
  return 0;
}

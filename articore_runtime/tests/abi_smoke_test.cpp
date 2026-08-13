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
  const auto start_trajectory_ex =
      &articore_runtime_start_joint_trajectory_ex;
  const auto cancel_trajectory = &articore_runtime_cancel_trajectory;
  const auto configure_trajectory_execution =
      &articore_runtime_configure_trajectory_execution;
  const auto configure_joint_safety_limits =
      &articore_runtime_configure_joint_safety_limits;
  const auto start_trajectory_report =
      &articore_runtime_start_joint_trajectory_report;
  const auto configure_gripper_force_profiles =
      &articore_runtime_configure_gripper_force_profiles;
  const auto set_gripper_commands =
      &articore_runtime_set_gripper_commands;
  const auto disable_report = &articore_runtime_get_last_disable_report;
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
                            ARTICORE_CAP_JOINT_TRAJECTORY |
                            ARTICORE_CAP_ATOMIC_ENABLE |
                            ARTICORE_CAP_COMMAND_LIFETIME |
                            ARTICORE_CAP_NONPREEMPTIVE_TRAJECTORY |
                            ARTICORE_CAP_PROTECTIVE_FAULT_HOLD |
                            ARTICORE_CAP_DETERMINISTIC_DISABLE |
                            ARTICORE_CAP_TRAJECTORY_MANAGEMENT |
                            ARTICORE_CAP_TRAJECTORY_SETTLING |
                            ARTICORE_CAP_TRAJECTORY_REPLACE_OR_HOLD |
                            ARTICORE_CAP_LAYERED_JOINT_LIMITS |
                            ARTICORE_CAP_GRIPPER_COMMAND_PROFILES |
                            ARTICORE_CAP_GRIPPER_FORCE_10_LEVELS |
                            ARTICORE_CAP_JOINT_MIT_POSITION |
                            ARTICORE_CAP_JOINT_PV_POSITION;
  if (!create_ex || !enable_report || !submit_pos_vel_ex || !submit_mit_ex ||
      !set_joint_mit || !set_joint_pv ||
      !start_trajectory_ex || !cancel_trajectory || !disable_report ||
      !configure_trajectory_execution || !configure_joint_safety_limits ||
      !start_trajectory_report || !configure_gripper_force_profiles ||
      !set_gripper_commands || version != 0x0001000DU ||
      (capabilities & required) != required) {
    std::cerr << "Articore runtime ABI metadata is incomplete\n";
    return 1;
  }
  std::cout << "Articore runtime ABI smoke test passed\n";
  return 0;
}

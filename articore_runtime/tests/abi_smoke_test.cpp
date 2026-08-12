#include <cstdint>
#include <iostream>

#include "articore/runtime_abi.h"

int main() {
  const auto create_ex = &articore_runtime_create_ex;
  const auto enable_report = &articore_runtime_get_last_enable_report;
  const auto submit_pos_vel_ex = &articore_runtime_submit_pos_vel_ex;
  const auto submit_mit_ex = &articore_runtime_submit_mit_ex;
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
                            ARTICORE_CAP_PROTECTIVE_FAULT_HOLD;
  if (!create_ex || !enable_report || !submit_pos_vel_ex || !submit_mit_ex ||
      version != 0x00010005U ||
      (capabilities & required) != required) {
    std::cerr << "Articore runtime ABI metadata is incomplete\n";
    return 1;
  }
  std::cout << "Articore runtime ABI smoke test passed\n";
  return 0;
}

#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

#include "motor_abi.h"

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

}  // namespace

int main() {
  const char* version = motor_abi_version();
  require(version != nullptr, "version pointer");
  require(std::string(version) == "0.5.0-cpp", "ABI minor version reflects additive API");

  const std::string capabilities = motor_abi_capabilities_json();
  require(capabilities.find("\"vendors\":[\"damiao\"]") != std::string::npos ||
              capabilities.find("\"vendors\": [\"damiao\"]") != std::string::npos,
          "capabilities include damiao only");
  require(capabilities.find("socketcan") != std::string::npos, "capabilities include socketcan");
  require(capabilities.find("dm-device") != std::string::npos, "capabilities include dm-device");
  require(capabilities.find("dm_device_abis") != std::string::npos,
          "capabilities include DM_Device ABI versions");
  require(capabilities.find("dm_device_configurable_bitrates") != std::string::npos,
          "capabilities include configurable DM_Device bitrates");
  require(capabilities.find("dm_device_v10_process_session_reuse") != std::string::npos,
          "capabilities include v1.0 process-session reuse");
  require(capabilities.find("register_metadata") != std::string::npos,
          "capabilities include canonical register metadata");
  require(capabilities.find("parallel_controller_batches") != std::string::npos,
          "capabilities include parallel controller batches");
  require(capabilities.find("transport_capabilities") != std::string::npos,
          "capabilities include per-transport capability query");

  require(motor_controller_get_transport_capabilities(nullptr, nullptr) != 0,
          "transport capabilities rejects null controller and output");

  MotorRegisterInfo register_info{};
  require(motor_damiao_register_info(11, &register_info) == 0,
          "register metadata query succeeds");
  require(register_info.has_value == 1 &&
              register_info.access == MOTOR_REGISTER_ACCESS_READ_ONLY &&
              register_info.data_type == MOTOR_REGISTER_DATA_FLOAT,
          "register 11 metadata comes from the C++ table");
  require(motor_damiao_register_info(37, &register_info) == 0 &&
              register_info.has_value == 0,
          "unknown register metadata is represented explicitly");
  require(motor_damiao_register_info(0, nullptr) != 0,
          "register metadata rejects a null output pointer");

  require(motor_controller_enable_all(nullptr) != 0, "null controller fails");
  require(std::string(motor_last_error_message()).find("controller is null") !=
              std::string::npos,
          "last error is set");

  require(motor_controller_set_tx_gap_us(nullptr, 120) != 0,
          "TX-gap setter rejects a null controller");
  require(motor_controller_request_feedback_all(nullptr, 50) != 0,
          "batch feedback rejects a null controller");
  require(motor_controller_group_new(nullptr, 0) == nullptr,
          "controller group rejects an empty controller list");
  require(motor_controller_group_send_mit(nullptr, nullptr, 0) != 0,
          "MIT group send rejects a null group");
  require(motor_controller_group_send_pos_vel(nullptr, nullptr, 0) != 0,
          "POS_VEL group send rejects a null group");
  MotorFeedbackStats feedback_stats{};
  require(motor_handle_get_feedback_stats(nullptr, &feedback_stats) != 0,
          "feedback stats reject a null motor");
  MotorState fresh_state{};
  require(motor_handle_request_fresh_state(nullptr, 50, &fresh_state) != 0,
          "fresh-state request rejects a null motor");

  MotorController* invalid = motor_controller_new_socketcan(nullptr);
  require(invalid == nullptr, "null socketcan channel fails");
  require(std::string(motor_last_error_message()).find("channel is null") != std::string::npos,
          "socketcan null channel reports clear error");
  require(motor_controller_new_dm_device_ex("usb2canfd-dual", "0", 0, 5000000) == nullptr,
          "DM_Device factory rejects zero arbitration bitrate");

  std::cout << "motor ABI smoke tests passed\n";
  return 0;
}

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
  require(std::string(version) == "0.3.0-cpp", "ABI minor version reflects additive API");

  const std::string capabilities = motor_abi_capabilities_json();
  require(capabilities.find("\"vendors\":[\"damiao\"]") != std::string::npos ||
              capabilities.find("\"vendors\": [\"damiao\"]") != std::string::npos,
          "capabilities include damiao only");
  require(capabilities.find("socketcan") != std::string::npos, "capabilities include socketcan");
  require(capabilities.find("dm-device") != std::string::npos, "capabilities include dm-device");

  require(motor_controller_enable_all(nullptr) != 0, "null controller fails");
  require(std::string(motor_last_error_message()).find("controller is null") !=
              std::string::npos,
          "last error is set");

  require(motor_controller_set_tx_gap_us(nullptr, 120) != 0,
          "TX-gap setter rejects a null controller");
  MotorFeedbackStats feedback_stats{};
  require(motor_handle_get_feedback_stats(nullptr, &feedback_stats) != 0,
          "feedback stats reject a null motor");

  MotorController* invalid = motor_controller_new_socketcan(nullptr);
  require(invalid == nullptr, "null socketcan channel fails");
  require(std::string(motor_last_error_message()).find("channel is null") != std::string::npos,
          "socketcan null channel reports clear error");

  std::cout << "motor ABI smoke tests passed\n";
  return 0;
}

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include "articore/runtime_abi.h"

namespace {

using JointArray = std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>;

void check(int32_t result, const char* operation) {
  if (result == 0) return;
  throw std::runtime_error(
      std::string(operation) + " failed: " + articore_runtime_last_error());
}

ArticoreProductStateV2 state(ArticoreRuntime* runtime) {
  ArticoreProductStateV2 value{};
  value.struct_size = sizeof(value);
  check(articore_runtime_get_state_v2(runtime, &value), "get_state_v2");
  return value;
}

ArticoreSafetyHealthV2 health(ArticoreRuntime* runtime) {
  ArticoreSafetyHealthV2 value{};
  value.struct_size = sizeof(value);
  check(articore_runtime_get_health_v2(runtime, &value), "get_health_v2");
  if (value.health.state == ARTICORE_FAULT ||
      value.health.state == ARTICORE_SAFE_STOP) {
    throw std::runtime_error(
        std::string("unsafe Runtime state: ") +
        (value.health.fault_reason[0] ? value.health.fault_reason
                                     : value.safety_reason));
  }
  return value;
}

float maximum_zero_error(const ArticoreProductStateV2& value) {
  float error = 0.0f;
  for (const float position : value.left.positions) {
    error = std::max(error, std::abs(position));
  }
  for (const float position : value.right.positions) {
    error = std::max(error, std::abs(position));
  }
  return error;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 ||
      std::string(argv[1]) != "--i-understand-both-arms-will-move") {
    std::cerr << "Refusing to enable hardware without acknowledgement\n";
    return 2;
  }
  ArticoreRuntime* runtime = nullptr;
  if (articore_runtime_create_yunyi(
          ARTICORE_MODE_PV, 0, &runtime) != ARTICORE_OPERATION_OK ||
      !runtime) {
    std::cerr << "create failed: " << articore_runtime_last_error() << '\n';
    return 1;
  }
  bool connected = false;
  bool enabled = false;
  try {
    check(articore_runtime_connect(runtime), "connect");
    connected = true;
    check(articore_runtime_enable(runtime, ARTICORE_MODE_PV), "enable");
    enabled = true;
    check(articore_runtime_set_max_speed(runtime, 10.0f), "set_max_speed");
    JointArray zero{};
    check(articore_runtime_set_joint_pv(
              runtime, zero.data(), zero.size()),
          "set_joint_pv");

    const auto arrival_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(12);
    while (std::chrono::steady_clock::now() < arrival_deadline) {
      health(runtime);
      const auto current = state(runtime);
      if (maximum_zero_error(current) <= 0.02f) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (maximum_zero_error(state(runtime)) > 0.02f) {
      throw std::runtime_error("zero hold did not converge within 0.02 rad");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    const auto before = health(runtime);
    const auto wall_start = std::chrono::steady_clock::now();
    const auto cpu_start = std::clock();
    std::this_thread::sleep_for(std::chrono::seconds(10));
    const auto cpu_end = std::clock();
    const auto wall_end = std::chrono::steady_clock::now();
    const auto after = health(runtime);
    const double elapsed = std::chrono::duration<double>(wall_end - wall_start).count();
    const double cpu_seconds = static_cast<double>(cpu_end - cpu_start) /
                               static_cast<double>(CLOCKS_PER_SEC);
    const auto left_frames = after.health.left_transport.tx_frames -
                             before.health.left_transport.tx_frames;
    const auto right_frames = after.health.right_transport.tx_frames -
                              before.health.right_transport.tx_frames;
    constexpr double frames_per_arm_cycle = ARTICORE_PRODUCT_ARM_DOF;
    const double left_hz = left_frames / frames_per_arm_cycle / elapsed;
    const double right_hz = right_frames / frames_per_arm_cycle / elapsed;

    if (left_hz < 490.0 || left_hz > 510.0 ||
        right_hz < 490.0 || right_hz > 510.0) {
      throw std::runtime_error("native arm dispatch did not remain near 500 Hz");
    }
    if (after.health.left_transport.send_errors != 0 ||
        after.health.right_transport.send_errors != 0) {
      throw std::runtime_error("transport send errors observed during soak");
    }
    std::cout << "elapsed_s=" << elapsed
              << " left_tx_frames=" << left_frames
              << " right_tx_frames=" << right_frames
              << " left_control_hz=" << left_hz
              << " right_control_hz=" << right_hz
              << " process_cpu_percent=" << (cpu_seconds / elapsed * 100.0)
              << " final_zero_error_rad=" << maximum_zero_error(state(runtime))
              << std::endl;

    check(articore_runtime_disable(runtime), "disable");
    enabled = false;
    check(articore_runtime_disconnect(runtime), "disconnect");
    connected = false;
    articore_runtime_free(runtime);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    if (enabled) articore_runtime_disable(runtime);
    if (connected) articore_runtime_disconnect(runtime);
    articore_runtime_free(runtime);
    return 1;
  }
}

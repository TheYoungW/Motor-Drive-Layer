#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include "articore/runtime_abi.h"

namespace {

using Pose = std::array<float, ARTICORE_PRODUCT_POSE_DOF>;

constexpr Pose kReportedStart{
    0.079761505f, 0.366678715f, 0.035763200f,
    0.546720803f, -0.132921576f, -0.313994825f};
constexpr Pose kTarget{
    0.307545f, 0.021073f, 0.203161f,
    -0.345181f, -1.157594f, -0.422116f};

void check(int32_t result, const char* operation) {
  if (result == ARTICORE_OPERATION_OK) return;
  const char* detail = articore_runtime_last_error();
  throw std::runtime_error(
      std::string(operation) + " failed: " +
      (detail && detail[0] ? detail : "unknown Runtime error"));
}

void check_health(ArticoreRuntime* runtime) {
  ArticoreSafetyHealth health{};
  health.struct_size = sizeof(health);
  check(articore_runtime_get_health(runtime, &health), "get_health");
  if (health.state == ARTICORE_FAULT ||
      health.state == ARTICORE_SAFE_STOP) {
    const char* reason = health.fault_reason[0]
                             ? health.fault_reason
                             : health.safety_reason;
    throw std::runtime_error(
        std::string("Runtime safety failure: ") +
        (reason[0] ? reason : "unknown safety error"));
  }
}

Pose read_pose(ArticoreRuntime* runtime, uint32_t side) {
  ArticoreProductPose pose{};
  pose.struct_size = sizeof(pose);
  check(articore_runtime_get_pose(runtime, side, &pose), "get_pose");
  Pose result{};
  std::copy(std::begin(pose.values), std::end(pose.values), result.begin());
  return result;
}

void print_pose(const char* label, const Pose& pose) {
  std::cout << label << '=';
  for (std::size_t index = 0; index < pose.size(); ++index) {
    if (index) std::cout << ',';
    std::cout << pose[index];
  }
  std::cout << std::endl;
}

void wait_until_enabled_and_stationary(ArticoreRuntime* runtime,
                                       std::chrono::seconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    check_health(runtime);
    ArticoreProductState state{};
    state.struct_size = sizeof(state);
    check(articore_runtime_get_state(runtime, &state), "get_state");
    float maximum_speed = 0.0f;
    for (float velocity : state.left.velocities) {
      maximum_speed = std::max(maximum_speed, std::abs(velocity));
    }
    for (float velocity : state.right.velocities) {
      maximum_speed = std::max(maximum_speed, std::abs(velocity));
    }
    if (state.left.enabled_valid_mask == 0x7fU &&
        state.right.enabled_valid_mask == 0x7fU &&
        state.left.enabled_mask == 0x7fU &&
        state.right.enabled_mask == 0x7fU && maximum_speed < 0.08f) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  throw std::runtime_error("arm feedback did not become enabled and stationary");
}

void run_ptp(ArticoreRuntime* runtime, const Pose& target,
             float speed_percent, const char* label) {
  const Pose right_target = read_pose(runtime, ARTICORE_ROBOT_RIGHT);
  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> joint_target{};
  check(articore_runtime_solve_ik(
            runtime, target.data(), right_target.data(), joint_target.data(),
            static_cast<uint32_t>(joint_target.size())),
        "solve_ik");
  check(articore_runtime_set_joint_pv(
            runtime, joint_target.data(),
            static_cast<uint32_t>(joint_target.size()), speed_percent),
        label);
  std::cout << "stage=" << label
            << " speed_percent=" << speed_percent << std::endl;

  const auto started = std::chrono::steady_clock::now();
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(30);
  uint32_t stable_samples = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    check_health(runtime);
    const Pose actual = read_pose(runtime, ARTICORE_ROBOT_LEFT);
    float position_error_squared = 0.0f;
    float maximum_orientation_error = 0.0f;
    for (uint32_t index = 0; index < 3; ++index) {
      const float error = actual[index] - target[index];
      position_error_squared += error * error;
    }
    for (uint32_t index = 3; index < ARTICORE_PRODUCT_POSE_DOF; ++index) {
      maximum_orientation_error = std::max(
          maximum_orientation_error,
          std::abs(std::remainder(actual[index] - target[index],
                                  2.0f * 3.14159265358979323846f)));
    }

    ArticoreProductState state{};
    state.struct_size = sizeof(state);
    check(articore_runtime_get_state(runtime, &state), "get_state");
    float maximum_speed = 0.0f;
    for (float velocity : state.left.velocities) {
      maximum_speed = std::max(maximum_speed, std::abs(velocity));
    }
    const bool arrived = std::sqrt(position_error_squared) <= 0.005f &&
                         maximum_orientation_error <= 0.02f &&
                         maximum_speed <= 0.05f;
    stable_samples = arrived ? stable_samples + 1 : 0;
    if (stable_samples >= 100) {
      const double elapsed_s = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - started).count();
      std::cout << "stage=" << label << "_arrived"
                << " elapsed_s=" << elapsed_s << std::endl;
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  throw std::runtime_error(std::string(label) + " timed out");
}

}  // namespace

int main(int argc, char** argv) {
  if ((argc != 2 && argc != 4) ||
      std::strcmp(argv[1], "--i-understand-left-arm-will-move") != 0 ||
      (argc == 4 && std::strcmp(argv[2], "--speed") != 0)) {
    std::cerr << "Refusing to move hardware. Pass "
                 "--i-understand-left-arm-will-move [--speed PERCENT]\n";
    return 2;
  }
  const float diagnostic_speed = argc == 4 ? std::stof(argv[3]) : 100.0f;
  if (!std::isfinite(diagnostic_speed) || diagnostic_speed <= 0.0f ||
      diagnostic_speed > 100.0f) {
    std::cerr << "Diagnostic speed must be within (0,100]\n";
    return 2;
  }

  ArticoreRuntime* runtime = nullptr;
  if (articore_runtime_create_yunyi(ARTICORE_MODE_PV, 1, &runtime) !=
          ARTICORE_OPERATION_OK ||
      !runtime) {
    std::cerr << "create failed: " << articore_runtime_last_error() << '\n';
    return 1;
  }

  bool connected = false;
  bool enabled = false;
  try {
    check(articore_runtime_connect(runtime), "connect");
    connected = true;
    const Pose original = read_left_pose(runtime);
    print_pose("original_pose", original);

    check(articore_runtime_enable(runtime), "enable");
    enabled = true;
    wait_until_enabled_and_stationary(runtime, std::chrono::seconds(4));

    run_ptp(runtime, kReportedStart, 20.0f, "position_reported_start");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    print_pose("measured_reported_start", read_left_pose(runtime));

    run_ptp(runtime, kTarget, diagnostic_speed, "diagnostic_ptp");
    std::this_thread::sleep_for(std::chrono::seconds(3));
    print_pose("measured_target", read_left_pose(runtime));

    run_ptp(runtime, original, 20.0f, "return_original");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    print_pose("measured_return", read_left_pose(runtime));

    check(articore_runtime_disable(runtime), "disable");
    enabled = false;
    check(articore_runtime_disconnect(runtime), "disconnect");
    connected = false;
    std::cout << "diagnostic_complete=true" << std::endl;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    if (enabled) articore_runtime_disable(runtime);
    if (connected) articore_runtime_disconnect(runtime);
    articore_runtime_free(runtime);
    return 1;
  }
  articore_runtime_free(runtime);
  return 0;
}

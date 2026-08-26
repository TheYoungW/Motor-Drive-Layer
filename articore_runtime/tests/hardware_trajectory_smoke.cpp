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

using JointArray = std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>;

constexpr float kPiOverTwo = 1.57079632679f;
constexpr float kPvVelocityLimit = 0.35f;
constexpr float kArrivalTolerance = 0.05f;

void check(int32_t result, const char* operation) {
  if (result == 0) return;
  const char* detail = articore_runtime_last_error();
  throw std::runtime_error(
      std::string(operation) + " failed: " +
      (detail && detail[0] ? detail : "unknown Runtime error"));
}

JointArray positions(const ArticoreProductStateV2& state) {
  JointArray result{};
  std::copy(std::begin(state.left.positions), std::end(state.left.positions),
            result.begin());
  std::copy(std::begin(state.right.positions), std::end(state.right.positions),
            result.begin() + ARTICORE_PRODUCT_ARM_DOF);
  return result;
}

JointArray velocities(const ArticoreProductStateV2& state) {
  JointArray result{};
  std::copy(std::begin(state.left.velocities), std::end(state.left.velocities),
            result.begin());
  std::copy(std::begin(state.right.velocities),
            std::end(state.right.velocities),
            result.begin() + ARTICORE_PRODUCT_ARM_DOF);
  return result;
}

void set_waypoint_positions(ArticoreTrajectoryWaypoint& waypoint,
                            const JointArray& values) {
  std::copy(values.begin(), values.begin() + ARTICORE_PRODUCT_ARM_DOF,
            std::begin(waypoint.left_positions));
  std::copy(values.begin() + ARTICORE_PRODUCT_ARM_DOF, values.end(),
            std::begin(waypoint.right_positions));
}

ArticoreProductStateV2 read_state(ArticoreRuntime* runtime) {
  ArticoreProductStateV2 state{};
  state.struct_size = sizeof(state);
  check(articore_runtime_get_state_v2(runtime, &state), "get_state_v2");
  return state;
}

void check_health(ArticoreRuntime* runtime,
                  const ArticoreTrajectoryStatus* status = nullptr) {
  ArticoreSafetyHealthV2 health{};
  health.struct_size = sizeof(health);
  check(articore_runtime_get_health_v2(runtime, &health), "get_health_v2");
  if (health.health.state == ARTICORE_FAULT ||
      health.health.state == ARTICORE_SAFE_STOP ||
      (status && status->state == ARTICORE_TRAJECTORY_FAULT)) {
    const char* reason = status && status->error[0]
                             ? status->error
                             : (health.health.fault_reason[0]
                                    ? health.health.fault_reason
                                    : health.safety_reason);
    throw std::runtime_error(std::string("trajectory safety failure: ") +
                             (reason[0] ? reason : "unknown safety error"));
  }
}

ArticoreProductStateV2 wait_until_stationary(ArticoreRuntime* runtime,
                                             std::chrono::seconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  ArticoreProductStateV2 state{};
  while (std::chrono::steady_clock::now() < deadline) {
    check_health(runtime);
    state = read_state(runtime);
    const auto speed = velocities(state);
    const float maximum_speed =
        *std::max_element(speed.begin(), speed.end(),
                          [](float lhs, float rhs) {
                            return std::abs(lhs) < std::abs(rhs);
                          });
    if (state.left.enabled_valid_mask == 0x7fU &&
        state.right.enabled_valid_mask == 0x7fU &&
        state.left.enabled_mask == 0x7fU &&
        state.right.enabled_mask == 0x7fU &&
        std::abs(maximum_speed) < 0.08f) {
      return state;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  throw std::runtime_error(
      "all 14 arm joints did not report fresh enabled stationary feedback");
}

double safe_duration(const JointArray& start, const JointArray& target) {
  float maximum_delta = 0.0f;
  for (std::size_t i = 0; i < start.size(); ++i) {
    maximum_delta = std::max(maximum_delta, std::abs(target[i] - start[i]));
  }
  // A zero-boundary quintic smoothstep has peak normalized speed 1.875.
  // Leave margin below the configured 0.35 rad/s PV limit.
  return std::max(8.0, 1.875 * static_cast<double>(maximum_delta) / 0.30);
}

ArticoreTrajectoryStatus run_trajectory(ArticoreRuntime* runtime,
                                        const JointArray& start,
                                        const JointArray& target,
                                        double duration_s,
                                        const char* label) {
  ArticoreTrajectoryWaypoint waypoints[2]{};
  for (auto& waypoint : waypoints) waypoint.struct_size = sizeof(waypoint);
  waypoints[0].time_s = 0.0;
  waypoints[1].time_s = duration_s;
  set_waypoint_positions(waypoints[0], start);
  set_waypoint_positions(waypoints[1], target);

  ArticoreTrajectoryConfig config{};
  config.struct_size = sizeof(config);
  config.interpolation = ARTICORE_TRAJECTORY_QUINTIC;
  config.control_mode = ARTICORE_MODE_PV;
  std::fill(std::begin(config.pv_velocity_limits),
            std::end(config.pv_velocity_limits), kPvVelocityLimit);
  check(articore_runtime_start_trajectory(runtime, waypoints, 2, &config),
        label);

  ArticoreTrajectoryStatus status{};
  status.struct_size = sizeof(status);
  const auto timeout = std::chrono::duration<double>(duration_s + 4.0);
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::milliseconds>(
                            timeout);
  while (std::chrono::steady_clock::now() < deadline) {
    check(articore_runtime_get_trajectory_status(runtime, &status),
          "get_trajectory_status");
    check_health(runtime, &status);
    if (status.state == ARTICORE_TRAJECTORY_COMPLETED) return status;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  throw std::runtime_error(std::string(label) + " timed out");
}

float wait_for_target(ArticoreRuntime* runtime, const JointArray& target,
                      std::chrono::seconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  float maximum_error = INFINITY;
  while (std::chrono::steady_clock::now() < deadline) {
    check_health(runtime);
    const auto actual = positions(read_state(runtime));
    maximum_error = 0.0f;
    for (std::size_t i = 0; i < target.size(); ++i) {
      maximum_error = std::max(maximum_error,
                               std::abs(actual[i] - target[i]));
    }
    if (maximum_error <= kArrivalTolerance) return maximum_error;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return maximum_error;
}

}  // namespace

int main(int argc, char** argv) {
  const bool read_state_only =
      argc == 2 && std::strcmp(argv[1], "--read-state-only") == 0;
  const bool return_zero_only =
      argc == 3 &&
      std::strcmp(argv[1], "--i-understand-both-arms-will-move") == 0 &&
      std::strcmp(argv[2], "--return-zero-only") == 0;
  if (!read_state_only &&
      (argc != 3 ||
       std::strcmp(argv[1], "--i-understand-both-arms-will-move") != 0 ||
       (std::strcmp(argv[2], "--joint4-90-others-zero") != 0 &&
        !return_zero_only))) {
    std::cerr << "Refusing to move hardware. Pass "
                 "--i-understand-both-arms-will-move "
                 "--joint4-90-others-zero (or --return-zero-only)\n";
    return 2;
  }

  ArticoreRuntime* runtime = nullptr;
  if (articore_runtime_create_yunyi(
          ARTICORE_MODE_PV, 1, &runtime) != ARTICORE_OPERATION_OK ||
      !runtime) {
    std::cerr << "create failed: " << articore_runtime_last_error() << '\n';
    return 1;
  }
  bool connected = false;
  bool enabled = false;
  try {
    check(articore_runtime_connect(runtime), "connect");
    connected = true;
    if (read_state_only) {
      const auto state = read_state(runtime);
      const auto current = positions(state);
      std::cout << "positions_rad=";
      for (std::size_t i = 0; i < current.size(); ++i) {
        if (i) std::cout << ',';
        std::cout << current[i];
      }
      std::cout << " left_enabled_mask=" << state.left.enabled_mask
                << " right_enabled_mask=" << state.right.enabled_mask
                << std::endl;
      check(articore_runtime_disconnect(runtime), "disconnect");
      connected = false;
      articore_runtime_free(runtime);
      return 0;
    }
    check(articore_runtime_enable(runtime, ARTICORE_MODE_PV), "enable");
    enabled = true;

    wait_until_stationary(runtime, std::chrono::seconds(4));
    // A powered-off arm can be physically outside a logical product limit.
    // Recover with one legal ordinary target first; never interpolate a chain
    // of out-of-range waypoints from the measured pose.
    JointArray zero_target{};
    check(articore_runtime_set_max_speed(runtime, 10.0f), "set_max_speed");
    check(articore_runtime_set_joint_pv(
              runtime, zero_target.data(), zero_target.size()),
          "recover_to_legal_zero");
    const float normalization_error =
        wait_for_target(runtime, zero_target, std::chrono::seconds(12));
    if (normalization_error > kArrivalTolerance) {
      throw std::runtime_error("legal zero recovery feedback exceeded tolerance");
    }
    const auto initial_state =
        wait_until_stationary(runtime, std::chrono::seconds(4));
    const auto initial = positions(initial_state);
    if (return_zero_only) {
      const double duration = safe_duration(initial, zero_target);
      std::cout << "stage=safety_return duration_s=" << duration << std::endl;
      const auto returned = run_trajectory(runtime, initial, zero_target,
                                           duration, "safety_return_to_zero");
      const float error =
          wait_for_target(runtime, zero_target, std::chrono::seconds(4));
      if (error > kArrivalTolerance) {
        throw std::runtime_error(
            "safety all-zero return feedback exceeded tolerance");
      }
      check(articore_runtime_disable(runtime), "disable");
      enabled = false;
      check(articore_runtime_disconnect(runtime), "disconnect");
      connected = false;
      std::cout << "stage=safety_return_complete trajectory_id="
                << returned.trajectory_id << " max_error_rad=" << error
                << std::endl;
      articore_runtime_free(runtime);
      return 0;
    }
    JointArray ninety_target{};
    ninety_target[3] = kPiOverTwo;
    ninety_target[10] = kPiOverTwo;
    const double outward_duration = safe_duration(initial, ninety_target);
    std::cout << "stage=outward duration_s=" << outward_duration << std::endl;
    const auto outward = run_trajectory(runtime, initial, ninety_target,
                                        outward_duration, "move_to_joint4_90");
    const float outward_error =
        wait_for_target(runtime, ninety_target, std::chrono::seconds(4));
    std::cout << "stage=outward_complete trajectory_id="
              << outward.trajectory_id << " max_error_rad=" << outward_error
              << std::endl;

    const auto return_state =
        wait_until_stationary(runtime, std::chrono::seconds(4));
    const auto return_start = positions(return_state);
    const double return_duration = safe_duration(return_start, zero_target);
    std::cout << "stage=return duration_s=" << return_duration << std::endl;
    const auto returned = run_trajectory(runtime, return_start, zero_target,
                                         return_duration, "return_to_zero");
    const float return_error =
        wait_for_target(runtime, zero_target, std::chrono::seconds(4));
    std::cout << "stage=return_complete trajectory_id="
              << returned.trajectory_id << " max_error_rad=" << return_error
              << std::endl;

    if (outward_error > kArrivalTolerance) {
      throw std::runtime_error(
          "joint4=90/all-other-joints=0 target feedback exceeded tolerance");
    }
    if (return_error > kArrivalTolerance) {
      throw std::runtime_error("all-zero return feedback exceeded tolerance");
    }

    check(articore_runtime_disable(runtime), "disable");
    enabled = false;
    check(articore_runtime_disconnect(runtime), "disconnect");
    connected = false;
    std::cout << "outward_trajectory_id=" << outward.trajectory_id
              << " outward_duration_s=" << outward_duration
              << " outward_max_error_rad=" << outward_error
              << " return_trajectory_id=" << returned.trajectory_id
              << " return_duration_s=" << return_duration
              << " return_max_error_rad=" << return_error << '\n';
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    articore_runtime_cancel_trajectory(runtime);
    if (enabled) articore_runtime_disable(runtime);
    if (connected) articore_runtime_disconnect(runtime);
    articore_runtime_free(runtime);
    return 1;
  }
  articore_runtime_free(runtime);
  return 0;
}

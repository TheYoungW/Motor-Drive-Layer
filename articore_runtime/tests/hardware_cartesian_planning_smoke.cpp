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
        ? health.fault_reason : health.safety_reason;
    throw std::runtime_error(
        std::string("Runtime safety failure: ") +
        (reason[0] ? reason : "unknown safety error"));
  }
}

Pose read_pose(ArticoreRuntime* runtime, uint32_t side) {
  ArticoreProductPose output{};
  output.struct_size = sizeof(output);
  check(articore_runtime_get_pose(runtime, side, &output), "get_pose");
  Pose pose{};
  std::copy(std::begin(output.values), std::end(output.values), pose.begin());
  return pose;
}

float position_error(const Pose& actual, const Pose& target) {
  float squared = 0.0f;
  for (uint32_t index = 0; index < 3; ++index) {
    const float error = actual[index] - target[index];
    squared += error * error;
  }
  return std::sqrt(squared);
}

ArticoreMotionStatus motion_status(
    ArticoreRuntime* runtime, uint64_t motion_id) {
  ArticoreMotionStatus status{};
  status.struct_size = sizeof(status);
  check(articore_runtime_get_motion_status(runtime, motion_id, &status),
        "get_motion_status");
  return status;
}

void wait_motion(ArticoreRuntime* runtime, uint64_t motion_id,
                 std::chrono::seconds timeout, const char* label) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    check_health(runtime);
    const auto status = motion_status(runtime, motion_id);
    if (status.state == ARTICORE_MOTION_COMPLETED) {
      std::cout << "stage=" << label << " motion_id=" << motion_id
                << " state=completed elapsed_s=" << status.elapsed_s
                << std::endl;
      return;
    }
    if (status.state == ARTICORE_MOTION_CANCELLED ||
        status.state == ARTICORE_MOTION_FAULT) {
      throw std::runtime_error(
          std::string(label) + " ended unsuccessfully: " + status.error);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  throw std::runtime_error(std::string(label) + " timed out");
}

void wait_pose(ArticoreRuntime* runtime, uint32_t side, const Pose& target,
               std::chrono::seconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  uint32_t stable_samples = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    check_health(runtime);
    ArticoreProductState state{};
    state.struct_size = sizeof(state);
    check(articore_runtime_get_state(runtime, &state), "get_state");
    const auto& velocities = side == ARTICORE_ROBOT_LEFT
        ? state.left.velocities : state.right.velocities;
    float maximum_speed = 0.0f;
    for (float velocity : velocities) {
      maximum_speed = std::max(maximum_speed, std::abs(velocity));
    }
    const bool settled = position_error(read_pose(runtime, side), target) <=
            0.006f &&
        maximum_speed <= 0.05f;
    stable_samples = settled ? stable_samples + 1 : 0;
    if (stable_samples >= 30) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  throw std::runtime_error("Cartesian return pose did not settle");
}

uint64_t submit_linear(ArticoreRuntime* runtime, const Pose& start,
                       const Pose& end, const char* label) {
  uint64_t id = 0;
  check(articore_runtime_move_linear_trajectory(
            runtime, ARTICORE_ROBOT_RIGHT, start.data(), end.data(),
            &id),
        label);
  if (id == 0) throw std::runtime_error("Linear returned a zero motion id");
  std::cout << "stage=" << label << " motion_id=" << id
            << " submitted=true" << std::endl;
  return id;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 ||
      std::strcmp(argv[1], "--i-understand-right-arm-will-move") != 0) {
    std::cerr << "Refusing to move hardware. Pass "
                 "--i-understand-right-arm-will-move\n";
    return 2;
  }

  ArticoreRuntime* runtime = nullptr;
  if (articore_runtime_create_yunyi(ARTICORE_MODE_PV, 0, &runtime) !=
          ARTICORE_OPERATION_OK ||
      runtime == nullptr) {
    std::cerr << "create failed: " << articore_runtime_last_error() << '\n';
    return 1;
  }

  bool connected = false;
  bool enabled = false;
  try {
    check(articore_runtime_connect(runtime), "connect");
    connected = true;
    const Pose original_right = read_pose(runtime, ARTICORE_ROBOT_RIGHT);
    check(articore_runtime_enable(runtime), "enable");
    enabled = true;

    // This fails after CommandPlanningScope::begin() and therefore proves the
    // RAII failure path releases the token before the first real submission.
    uint64_t invalid_id = 0;
    const int32_t invalid_result = articore_runtime_move_linear_trajectory(
        runtime, ARTICORE_ROBOT_RIGHT, original_right.data(),
        nullptr, &invalid_id);
    if (invalid_result != ARTICORE_OPERATION_INVALID_ARGUMENT ||
        invalid_id != 0) {
      throw std::runtime_error(
          "invalid Linear did not fail atomically as expected");
    }
    std::cout << "stage=planning_failure token_released=pending_probe"
              << std::endl;

    Pose first_end = original_right;
    first_end[2] += 0.005f;
    const uint64_t first_id = submit_linear(
        runtime, original_right, first_end, "first_linear");
    wait_motion(runtime, first_id, std::chrono::seconds(12), "first_linear");

    Pose fifo_one = first_end;
    fifo_one[0] += 0.005f;
    Pose fifo_two = fifo_one;
    fifo_two[1] -= 0.005f;
    const uint64_t fifo_ids[3] = {
        submit_linear(runtime, first_end, fifo_one, "fifo_linear_1"),
        submit_linear(runtime, fifo_one, fifo_two, "fifo_linear_2"),
        submit_linear(runtime, fifo_two, original_right,
                      "fifo_linear_3"),
    };
    if (motion_status(runtime, fifo_ids[0]).state !=
            ARTICORE_MOTION_RUNNING ||
        motion_status(runtime, fifo_ids[1]).state !=
            ARTICORE_MOTION_QUEUED ||
        motion_status(runtime, fifo_ids[2]).state !=
            ARTICORE_MOTION_QUEUED) {
      throw std::runtime_error("three Linear motions did not enter FIFO order");
    }
    for (uint32_t index = 0; index < 3; ++index) {
      wait_motion(runtime, fifo_ids[index], std::chrono::seconds(20),
                  index == 0 ? "fifo_linear_1" :
                  index == 1 ? "fifo_linear_2" : "fifo_linear_3");
    }

    Pose circular_via = original_right;
    circular_via[0] += 0.005f;
    circular_via[2] += 0.005f;
    Pose circular_end = original_right;
    circular_end[2] += 0.010f;
    uint64_t circular_id = 0;
    check(articore_runtime_move_circular_trajectory(
              runtime, ARTICORE_ROBOT_RIGHT, original_right.data(),
              circular_via.data(), circular_end.data(), &circular_id),
          "circular");
    if (circular_id == 0) {
      throw std::runtime_error("Circular returned a zero motion id");
    }
    std::cout << "stage=circular motion_id=" << circular_id
              << " submitted=true" << std::endl;
    wait_motion(runtime, circular_id, std::chrono::seconds(15), "circular");

    check(articore_runtime_set_speed_percent(runtime, 20.0f),
          "return_speed");
    check(articore_runtime_move_pose(
              runtime, ARTICORE_ROBOT_RIGHT, original_right.data()),
          "return_original");
    wait_pose(runtime, ARTICORE_ROBOT_RIGHT, original_right,
              std::chrono::seconds(15));
    check(articore_runtime_disable(runtime), "disable");
    enabled = false;
    check(articore_runtime_disconnect(runtime), "disconnect");
    connected = false;
    std::cout << "hardware_acceptance=passed" << std::endl;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    articore_runtime_cancel_all_motions(runtime);
    if (enabled) articore_runtime_disable(runtime);
    if (connected) articore_runtime_disconnect(runtime);
    articore_runtime_free(runtime);
    return 1;
  }
  articore_runtime_free(runtime);
  return 0;
}

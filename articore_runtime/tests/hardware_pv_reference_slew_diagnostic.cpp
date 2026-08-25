#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "articore/detail/yunyi_runtime.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using Arm = std::array<float, ARTICORE_PRODUCT_ARM_DOF>;
using Product = std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>;

constexpr Arm kPointA{
    -0.029563904f, 0.089074135f, 0.064278603f, 1.250286102f,
    -0.002479553f, 0.033379555f, 0.197413445f};
constexpr Arm kPointB{
    0.147821426f, -0.123788834f, 0.348859787f, 1.147668839f,
    -0.269512177f, -0.125696182f, 0.041771889f};
constexpr std::array<float, 6> kReferenceSlewCandidates{
    0.75f, 1.0f, 1.25f, 1.5f, 2.0f, 3.0f};
constexpr std::array<float, 6> kLowReferenceSlewCandidates{
    0.25f, 0.35f, 0.45f, 0.55f, 0.65f, 0.75f};
constexpr std::array<float, 6> kConfirmReferenceSlewCandidates{
    0.45f, 0.5f, 0.55f, 0.45f, 0.5f, 0.55f};
constexpr float kDriveVelocityLimit = 3.0f;
constexpr float kStagingReferenceSlew = 0.75f;
constexpr float kAbortVelocity = 3.5f;

uint64_t now_ns() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          Clock::now().time_since_epoch()).count());
}

void require_healthy(const articore::SafetyRuntime& runtime) {
  const auto health = runtime.health_v2();
  if (health.health.state == ARTICORE_FAULT ||
      health.health.state == ARTICORE_SAFE_STOP ||
      health.health.state == ARTICORE_DEGRADED ||
      !health.health.left_transport.healthy ||
      !health.health.right_transport.healthy) {
    throw std::runtime_error(
        std::string("unsafe Runtime state: ") +
        (health.health.fault_reason[0] ? health.health.fault_reason
                                      : health.safety_reason));
  }
}

Product read_product_state(const articore::YunyiRuntimeResources& resources,
                           Product* velocities = nullptr,
                           bool require_enabled = true) {
  Product positions{};
  for (std::size_t index = 0; index < resources.joints.size(); ++index) {
    ArticoreMotorState state{};
    ArticoreFeedbackStats stats{};
    if (!articore::read_yunyi_motor_state(
            resources.joints[index].motor, state, stats) ||
        !state.has_value || !stats.has_feedback || stats.age_ns > 20'000'000ULL ||
        !std::isfinite(state.pos) || !std::isfinite(state.vel)) {
      throw std::runtime_error("joint feedback is incomplete or stale at index " +
                               std::to_string(index));
    }
    if (require_enabled && state.status_code != 1) {
      throw std::runtime_error("joint is not enabled at index " +
                               std::to_string(index));
    }
    const auto& joint = resources.joints[index];
    positions[index] = joint.direction * state.pos;
    if (velocities) {
      (*velocities)[index] =
          joint.direction * state.vel * joint.velocity_feedback_scale;
    }
  }
  return positions;
}

std::vector<ArticoreJointPvTarget> make_targets(
    const articore::YunyiRuntimeResources& resources,
    const Product& logical_positions) {
  std::vector<ArticoreJointPvTarget> targets;
  targets.reserve(resources.joints.size());
  for (std::size_t index = 0; index < resources.joints.size(); ++index) {
    targets.push_back(ArticoreJointPvTarget{
        sizeof(ArticoreJointPvTarget), resources.joints[index].motor,
        resources.joints[index].direction * logical_positions[index]});
  }
  return targets;
}

struct Arrival {
  uint64_t stable_ns = 0;
  float peak_velocity = 0.0f;
  float endpoint_error = 0.0f;
};

Arrival wait_stable(articore::SafetyRuntime& runtime,
                    const articore::YunyiRuntimeResources& resources,
                    const Product& target,
                    std::chrono::seconds timeout = std::chrono::seconds(12)) {
  const auto deadline = Clock::now() + timeout;
  Clock::time_point stable_since{};
  Arrival result;
  while (Clock::now() < deadline) {
    require_healthy(runtime);
    Product velocities{};
    const auto positions = read_product_state(resources, &velocities);
    float max_error = 0.0f;
    float max_velocity = 0.0f;
    for (std::size_t index = 0; index < positions.size(); ++index) {
      max_error = std::max(max_error, std::abs(positions[index] - target[index]));
      max_velocity = std::max(max_velocity, std::abs(velocities[index]));
    }
    result.peak_velocity = std::max(result.peak_velocity, max_velocity);
    result.endpoint_error = max_error;
    if (result.peak_velocity > kAbortVelocity) {
      throw std::runtime_error("measured velocity exceeded 3.5 rad/s abort limit");
    }
    if (max_error <= 0.01f && max_velocity <= 0.05f) {
      if (stable_since == Clock::time_point{}) stable_since = Clock::now();
      if (Clock::now() - stable_since >= std::chrono::milliseconds(400)) {
        result.stable_ns = now_ns();
        return result;
      }
    } else {
      stable_since = Clock::time_point{};
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  throw std::runtime_error("ordinary PV target did not settle within 12 s");
}

Arrival move(articore::SafetyRuntime& runtime,
             const articore::YunyiRuntimeResources& resources,
             const Product& target, float reference_slew,
             uint64_t* command_ns = nullptr) {
  const auto targets = make_targets(resources, target);
  const uint64_t start_ns = now_ns();
  runtime.set_joint_pv(targets.data(), static_cast<uint32_t>(targets.size()),
                       reference_slew, kDriveVelocityLimit);
  if (command_ns) *command_ns = start_ns;
  return wait_stable(runtime, resources, target);
}

void seed_trace_joint_layout(
    articore::SafetyRuntime& runtime,
    const articore::YunyiRuntimeResources& resources,
    const Product& current) {
  articore::NativeTrajectoryRequest request;
  request.mode = ARTICORE_MODE_PV;
  request.joints.reserve(resources.joints.size());
  for (std::size_t index = 0; index < resources.joints.size(); ++index) {
    const auto& source = resources.joints[index];
    articore::NativeTrajectoryJoint joint;
    joint.role = articore::yunyi_joint_role(static_cast<uint32_t>(index));
    joint.motor = source.motor;
    joint.direction = source.direction;
    joint.velocity_command_scale = source.velocity_command_scale;
    joint.velocity_feedback_scale = source.velocity_feedback_scale;
    joint.torque_command_scale = source.torque_command_scale;
    joint.lower_position = source.lower;
    joint.upper_position = source.upper;
    joint.velocity_limit = source.velocity_limit;
    joint.acceleration_limit = source.acceleration_limit;
    joint.torque_limit = source.torque_limit;
    joint.pv_velocity_limit = kDriveVelocityLimit;
    joint.pv_hold_velocity_limit = 0.0f;
    request.joints.push_back(joint);
  }
  for (double time_s : {0.0, 0.1}) {
    articore::NativeTrajectoryWaypoint waypoint;
    waypoint.time_s = time_s;
    waypoint.positions.assign(current.begin(), current.end());
    waypoint.velocities.assign(current.size(), 0.0f);
    waypoint.accelerations.assign(current.size(), 0.0f);
    request.waypoints.push_back(std::move(waypoint));
  }
  runtime.start_trajectory(std::move(request));
  const auto deadline = Clock::now() + std::chrono::seconds(3);
  while (Clock::now() < deadline) {
    const auto status = runtime.trajectory_status();
    if (status.state == ARTICORE_TRAJECTORY_COMPLETED) return;
    if (status.state == ARTICORE_TRAJECTORY_FAULT ||
        status.state == ARTICORE_TRAJECTORY_CANCELLED) {
      throw std::runtime_error(std::string("trace seed trajectory failed: ") +
                               status.error);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  throw std::runtime_error("trace seed trajectory timed out");
}

}  // namespace

int main(int argc, char** argv) {
  if ((argc != 2 && argc != 3) ||
      std::strcmp(argv[1], "--i-understand-left-arm-will-move") != 0) {
    std::cerr << "Refusing to move hardware. Pass "
                 "--i-understand-left-arm-will-move "
                 "[--low-sweep|--confirm-sweep]\n";
    return 2;
  }
  const bool low_sweep = argc == 3 &&
      std::strcmp(argv[2], "--low-sweep") == 0;
  const bool confirm_sweep = argc == 3 &&
      std::strcmp(argv[2], "--confirm-sweep") == 0;
  if (argc == 3 && !low_sweep && !confirm_sweep) {
    std::cerr << "Unknown diagnostic option\n";
    return 2;
  }

  auto bundle = articore::create_yunyi_runtime(ARTICORE_MODE_PV, false);
  auto& runtime = *bundle.runtime;
  auto& resources = *bundle.resources;
  bool connected = false;
  bool enabled = false;
  Product initial{};
  Product right_hold{};
  const char* const segments_path = confirm_sweep
      ? "/tmp/articore_pv_reference_slew_confirm_segments.csv"
      : (low_sweep
            ? "/tmp/articore_pv_reference_slew_low_segments.csv"
            : "/tmp/articore_pv_reference_slew_segments.csv");
  std::ofstream segments(segments_path);
  segments << "reference_slew_rad_s,command_ns,stable_ns,hold_end_ns,"
              "peak_velocity_rad_s,endpoint_error_max_rad\n";
  segments << std::setprecision(9);
  try {
    runtime.connect();
    connected = true;
    initial = read_product_state(resources, nullptr, false);
    for (const auto& joint : resources.joints) {
      ArticoreMotorState state{};
      ArticoreFeedbackStats stats{};
      if (!articore::read_yunyi_motor_state(joint.motor, state, stats) ||
          state.status_code != 0) {
        throw std::runtime_error("all arm joints must start disabled");
      }
    }
    right_hold = initial;

    runtime.enable(ARTICORE_MODE_PV);
    enabled = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    require_healthy(runtime);
    read_product_state(resources);
    seed_trace_joint_layout(runtime, resources, read_product_state(resources));

    Product point_a = right_hold;
    Product point_b = right_hold;
    std::copy(kPointA.begin(), kPointA.end(), point_a.begin());
    std::copy(kPointB.begin(), kPointB.end(), point_b.begin());

    std::cout << "STAGE initial_to_A reference_slew="
              << kStagingReferenceSlew << std::endl;
    move(runtime, resources, point_a, kStagingReferenceSlew);

    const auto& candidates = confirm_sweep
        ? kConfirmReferenceSlewCandidates
        : (low_sweep
              ? kLowReferenceSlewCandidates : kReferenceSlewCandidates);
    for (float candidate : candidates) {
      uint64_t command_ns = 0;
      std::cout << "TEST A_to_B reference_slew=" << candidate
                << " drive_limit=" << kDriveVelocityLimit << std::endl;
      const auto arrival = move(runtime, resources, point_b, candidate,
                                &command_ns);
      std::this_thread::sleep_for(std::chrono::milliseconds(750));
      const uint64_t hold_end_ns = now_ns();
      segments << candidate << ',' << command_ns << ',' << arrival.stable_ns
               << ',' << hold_end_ns << ',' << arrival.peak_velocity << ','
               << arrival.endpoint_error << '\n';
      segments.flush();
      std::cout << "DONE reference_slew=" << candidate
                << " peak_velocity=" << arrival.peak_velocity
                << " endpoint_error=" << arrival.endpoint_error << std::endl;
      move(runtime, resources, point_a, kStagingReferenceSlew);
    }

    std::cout << "RESTORE A_to_initial" << std::endl;
    move(runtime, resources, initial, kStagingReferenceSlew);
    runtime.disable();
    enabled = false;
    runtime.disconnect();
    connected = false;
    std::cout << "RESULT segments=" << segments_path << std::endl;
  } catch (const std::exception& error) {
    std::cerr << "RUN_ERROR " << error.what() << '\n';
    if (enabled) {
      try {
        move(runtime, resources, initial, kStagingReferenceSlew);
      } catch (const std::exception& restore_error) {
        std::cerr << "RESTORE_ERROR " << restore_error.what() << '\n';
      }
      try {
        runtime.disable();
      } catch (...) {
      }
    }
    if (connected) {
      try {
        runtime.disconnect();
      } catch (...) {
      }
    }
    return 1;
  }
  return 0;
}

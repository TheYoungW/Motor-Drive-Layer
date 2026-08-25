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

constexpr float kTestReferenceVelocity = 2.0f;
constexpr float kStagingReferenceVelocity = 0.5f;
constexpr float kPvDriveVelocityLimit = 3.0f;
constexpr float kAbortMoveVelocity = 3.5f;
constexpr float kAbortHoldVelocity = 0.30f;
constexpr float kAbortHoldError = 0.030f;
constexpr auto kHoldDuration = std::chrono::milliseconds(1500);

struct Snapshot {
  Product q{};
  Product dq{};
};

struct Arrival {
  uint64_t stable_ns = 0;
  float peak_velocity = 0.0f;
  float endpoint_error = 0.0f;
};

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

void require_all_disabled(articore::YunyiRuntimeResources& resources) {
  for (std::size_t index = 0; index < resources.joints.size(); ++index) {
    const auto state = resources.arm_motors[index]->request_fresh_state(
        std::chrono::milliseconds(100));
    if (!state || state->status_code != 0) {
      throw std::runtime_error("joint is not feedback-confirmed disabled at index " +
                               std::to_string(index));
    }
  }
}

Snapshot read_snapshot(const articore::YunyiRuntimeResources& resources,
                       bool require_enabled = true) {
  Snapshot result;
  for (std::size_t index = 0; index < resources.joints.size(); ++index) {
    ArticoreMotorState state{};
    ArticoreFeedbackStats stats{};
    if (!articore::read_yunyi_motor_state(
            resources.joints[index].motor, state, stats) ||
        !state.has_value || !stats.has_feedback || stats.age_ns > 20'000'000ULL ||
        !std::isfinite(state.pos) || !std::isfinite(state.vel)) {
      throw std::runtime_error("joint feedback incomplete/stale at index " +
                               std::to_string(index));
    }
    if (require_enabled && state.status_code != 1) {
      throw std::runtime_error("joint is not enabled at index " +
                               std::to_string(index));
    }
    const auto& joint = resources.joints[index];
    result.q[index] = joint.direction * state.pos;
    result.dq[index] =
        joint.direction * state.vel * joint.velocity_feedback_scale;
  }
  return result;
}

void wait_all_disabled_stationary(
    articore::YunyiRuntimeResources& resources) {
  const auto deadline = Clock::now() + std::chrono::seconds(5);
  Clock::time_point stable_since{};
  while (Clock::now() < deadline) {
    require_all_disabled(resources);
    const auto state = read_snapshot(resources, false);
    float max_velocity = 0.0f;
    for (float velocity : state.dq) {
      max_velocity = std::max(max_velocity, std::abs(velocity));
    }
    if (max_velocity <= 0.05f) {
      if (stable_since == Clock::time_point{}) stable_since = Clock::now();
      if (Clock::now() - stable_since >= std::chrono::milliseconds(200)) {
        return;
      }
    } else {
      stable_since = Clock::time_point{};
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  throw std::runtime_error("disabled joints did not become stationary");
}

void command_target(articore::SafetyRuntime& runtime,
                    const articore::YunyiRuntimeResources& resources,
                    ArticoreControlMode mode, const Product& positions,
                    float reference_velocity) {
  if (mode == ARTICORE_MODE_PV) {
    std::vector<ArticoreJointPvTarget> targets;
    targets.reserve(resources.joints.size());
    for (std::size_t index = 0; index < resources.joints.size(); ++index) {
      targets.push_back(ArticoreJointPvTarget{
          sizeof(ArticoreJointPvTarget), resources.joints[index].motor,
          resources.joints[index].direction * positions[index]});
    }
    runtime.set_joint_pv(targets.data(), static_cast<uint32_t>(targets.size()),
                         reference_velocity, kPvDriveVelocityLimit);
  } else {
    std::vector<ArticoreJointMitTarget> targets;
    targets.reserve(resources.joints.size());
    for (std::size_t index = 0; index < resources.joints.size(); ++index) {
      targets.push_back(ArticoreJointMitTarget{
          sizeof(ArticoreJointMitTarget), resources.joints[index].motor,
          resources.joints[index].direction * positions[index]});
    }
    runtime.set_joint_mit(targets.data(), static_cast<uint32_t>(targets.size()),
                          reference_velocity);
  }
}

Arrival wait_stable(articore::SafetyRuntime& runtime,
                    const articore::YunyiRuntimeResources& resources,
                    ArticoreControlMode mode, const Product& target) {
  const auto deadline = Clock::now() + std::chrono::seconds(15);
  const float position_tolerance =
      mode == ARTICORE_MODE_MIT ? 0.05f : 0.01f;
  Clock::time_point stable_since{};
  Arrival result;
  while (Clock::now() < deadline) {
    require_healthy(runtime);
    const auto state = read_snapshot(resources);
    float max_error = 0.0f;
    float max_velocity = 0.0f;
    for (std::size_t index = 0; index < state.q.size(); ++index) {
      max_error = std::max(max_error, std::abs(state.q[index] - target[index]));
      max_velocity = std::max(max_velocity, std::abs(state.dq[index]));
    }
    result.peak_velocity = std::max(result.peak_velocity, max_velocity);
    result.endpoint_error = max_error;
    if (max_velocity > kAbortMoveVelocity) {
      throw std::runtime_error("move velocity exceeded 3.5 rad/s abort limit");
    }
    if (max_error <= position_tolerance && max_velocity <= 0.05f) {
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
  throw std::runtime_error("joint target did not settle within 15 s");
}

Arrival move(articore::SafetyRuntime& runtime,
             const articore::YunyiRuntimeResources& resources,
             ArticoreControlMode mode, const Product& target,
             float reference_velocity, uint64_t* command_ns = nullptr) {
  const uint64_t start_ns = now_ns();
  command_target(runtime, resources, mode, target, reference_velocity);
  if (command_ns) *command_ns = start_ns;
  return wait_stable(runtime, resources, mode, target);
}

void validate_hold(articore::SafetyRuntime& runtime,
                   const articore::YunyiRuntimeResources& resources,
                   ArticoreControlMode mode, const Product& target) {
  const auto deadline = Clock::now() + kHoldDuration;
  const float error_limit =
      mode == ARTICORE_MODE_MIT ? 0.08f : kAbortHoldError;
  while (Clock::now() < deadline) {
    require_healthy(runtime);
    const auto state = read_snapshot(resources);
    float max_velocity = 0.0f;
    float max_error = 0.0f;
    for (std::size_t index = 0; index < state.q.size(); ++index) {
      max_velocity = std::max(max_velocity, std::abs(state.dq[index]));
      max_error = std::max(max_error, std::abs(state.q[index] - target[index]));
    }
    if (max_velocity > kAbortHoldVelocity || max_error > error_limit) {
      throw std::runtime_error("hold safety threshold exceeded");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void seed_trace_joint_layout(
    articore::SafetyRuntime& runtime,
    const articore::YunyiRuntimeResources& resources,
    ArticoreControlMode mode, const Product& current) {
  articore::NativeTrajectoryRequest request;
  request.mode = mode;
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
    joint.mit_kp = source.kp;
    joint.mit_kd = source.kd;
    joint.pv_velocity_limit = kPvDriveVelocityLimit;
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
  const bool pv = argc == 2 &&
      std::strcmp(argv[1], "--i-understand-compare-pv-2rad") == 0;
  const bool mit = argc == 2 &&
      std::strcmp(argv[1], "--i-understand-compare-mit-2rad") == 0;
  const bool restore_only = argc == 2 &&
      std::strcmp(argv[1], "--restore-pv-mode-disabled") == 0;
  const bool recover_left = argc == 9 &&
      std::strcmp(argv[1], "--recover-left-pv-disabled") == 0;
  Arm recovery_left{};
  if (recover_left) {
    for (std::size_t index = 0; index < recovery_left.size(); ++index) {
      recovery_left[index] = std::stof(argv[index + 2]);
      if (!std::isfinite(recovery_left[index])) {
        std::cerr << "Recovery target contains a non-finite value\n";
        return 2;
      }
    }
  }
  if (!pv && !mit && !restore_only && !recover_left) {
    std::cerr << "Refusing hardware movement. Pass "
                 "--i-understand-compare-pv-2rad or "
                 "--i-understand-compare-mit-2rad; use "
                 "--restore-pv-mode-disabled for a no-motion recovery\n";
    return 2;
  }
  const auto mode = (pv || restore_only || recover_left)
      ? ARTICORE_MODE_PV : ARTICORE_MODE_MIT;
  const char* const label = (pv || restore_only || recover_left)
      ? "pv" : "mit";
  const char* const output_label =
      (restore_only || recover_left) ? "recovery" : label;
  const std::string segments_path = std::string("/tmp/articore_") +
      output_label + "_2rad_compare_segments.csv";
  std::ofstream segments(segments_path);
  segments << "mode,leg,reference_velocity_rad_s,command_ns,stable_ns,"
              "hold_end_ns,peak_velocity_rad_s,endpoint_error_max_rad\n";
  segments << std::setprecision(9);

  auto bundle = articore::create_yunyi_runtime(mode, false);
  auto& runtime = *bundle.runtime;
  auto& resources = *bundle.resources;
  bool connected = false;
  bool enabled = false;
  bool have_initial = false;
  bool mode_configured = false;
  Product initial{};
  try {
    runtime.connect();
    connected = true;
    const auto configured = runtime.configure_mode_for_connect(mode);
    if (configured != ARTICORE_OPERATION_OK) {
      throw std::runtime_error(
          std::string("failed to configure ") + label + " motor mode: " +
          runtime.health_v2().last_operation_error);
    }
    mode_configured = true;
    require_all_disabled(resources);
    if (restore_only) {
      runtime.disconnect();
      connected = false;
      std::cout << "RESTORED motor_mode=pv axes=disabled" << std::endl;
      return 0;
    }
    initial = read_snapshot(resources, false).q;
    have_initial = true;

    if (recover_left) {
      Product recovery_target = initial;
      std::copy(recovery_left.begin(), recovery_left.end(),
                recovery_target.begin());
      runtime.enable(ARTICORE_MODE_PV);
      enabled = true;
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      require_healthy(runtime);
      std::cout << "RECOVERY mode=pv reference_velocity="
                << kStagingReferenceVelocity << std::endl;
      move(runtime, resources, ARTICORE_MODE_PV, recovery_target,
           kStagingReferenceVelocity);
      runtime.disable();
      enabled = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      require_all_disabled(resources);
      runtime.disconnect();
      connected = false;
      std::cout << "RECOVERY_COMPLETE axes=disabled" << std::endl;
      return 0;
    }

    runtime.enable(mode);
    enabled = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    require_healthy(runtime);
    seed_trace_joint_layout(runtime, resources, mode,
                            read_snapshot(resources).q);

    Product point_a = initial;
    Product point_b = initial;
    std::copy(kPointA.begin(), kPointA.end(), point_a.begin());
    std::copy(kPointB.begin(), kPointB.end(), point_b.begin());
    std::cout << "STAGE mode=" << label << " initial_to_A" << std::endl;
    move(runtime, resources, mode, point_a, kStagingReferenceVelocity);

    for (int leg = 0; leg < 2; ++leg) {
      const Product& target = leg == 0 ? point_b : point_a;
      uint64_t command_ns = 0;
      std::cout << "TEST mode=" << label << " leg="
                << (leg == 0 ? "A_to_B" : "B_to_A")
                << " reference_velocity=" << kTestReferenceVelocity
                << std::endl;
      const auto arrival = move(runtime, resources, mode, target,
                                kTestReferenceVelocity, &command_ns);
      validate_hold(runtime, resources, mode, target);
      const uint64_t hold_end_ns = now_ns();
      segments << label << ',' << (leg == 0 ? "A_to_B" : "B_to_A")
               << ',' << kTestReferenceVelocity << ',' << command_ns << ','
               << arrival.stable_ns << ',' << hold_end_ns << ','
               << arrival.peak_velocity << ',' << arrival.endpoint_error
               << '\n';
      segments.flush();
    }

    std::cout << "STAGE mode=" << label << " return_initial" << std::endl;
    move(runtime, resources, mode, initial, kStagingReferenceVelocity);
    runtime.disable();
    enabled = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    require_all_disabled(resources);
    if (mode == ARTICORE_MODE_MIT) {
      const auto restored = runtime.configure_mode(ARTICORE_MODE_PV);
      if (restored != ARTICORE_OPERATION_OK) {
        throw std::runtime_error(
            std::string("failed to restore PV motor mode: ") +
            runtime.health_v2().last_operation_error);
      }
      mode_configured = false;
      std::cout << "RESTORED motor_mode=pv" << std::endl;
    }
    runtime.disconnect();
    connected = false;
    std::cout << "RESULT mode=" << label
              << " segments=" << segments_path << std::endl;
  } catch (const std::exception& error) {
    std::cerr << "RUN_ERROR " << error.what() << '\n';
    if (enabled) {
      if (have_initial) {
        try {
          move(runtime, resources, mode, initial, kStagingReferenceVelocity);
        } catch (const std::exception& restore_error) {
          std::cerr << "POSE_RESTORE_ERROR " << restore_error.what() << '\n';
        }
      }
      try {
        runtime.disable();
        enabled = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      } catch (...) {
      }
    }
    if (connected && mode_configured && mode == ARTICORE_MODE_MIT) {
      try {
        wait_all_disabled_stationary(resources);
        const auto restored = runtime.configure_mode(ARTICORE_MODE_PV);
        if (restored == ARTICORE_OPERATION_OK) {
          mode_configured = false;
          std::cerr << "MOTOR_MODE_RESTORED pv\n";
        } else {
          std::cerr << "MOTOR_MODE_RESTORE_ERROR "
                    << runtime.health_v2().last_operation_error << '\n';
        }
      } catch (const std::exception& restore_error) {
        std::cerr << "MOTOR_MODE_RESTORE_ERROR " << restore_error.what()
                  << '\n';
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

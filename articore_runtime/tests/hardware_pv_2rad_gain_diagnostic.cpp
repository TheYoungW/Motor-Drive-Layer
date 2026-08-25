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
constexpr float kDriveVelocityLimit = 3.0f;
constexpr float kAbortMoveVelocity = 3.5f;
constexpr float kAbortHoldVelocity = 0.30f;
constexpr float kAbortHoldError = 0.030f;
constexpr auto kHoldDuration = std::chrono::milliseconds(1000);

struct PvGains {
  float kp_asr;
  float ki_asr;
  float kp_apr;
  float ki_apr;
};

struct Profile {
  const char* label;
  float kp_asr;
  float ki_asr;
};

// Baseline is deliberately repeated at the end so that a warmer drivetrain or
// time drift cannot make a candidate look better than it is.
constexpr std::array<Profile, 5> kProfiles{{
    {"baseline_a1", 0.00372f, 0.0020f},
    {"ki_0.0015", 0.00372f, 0.0015f},
    {"kp_0.0045", 0.00450f, 0.0020f},
    {"kp_0.0045_ki_0.0015", 0.00450f, 0.0015f},
    {"baseline_a2", 0.00372f, 0.0020f},
}};

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

bool close(float lhs, float rhs) {
  return std::abs(lhs - rhs) <=
      1.0e-6f * std::max(1.0f, std::max(std::abs(lhs), std::abs(rhs)));
}

PvGains read_gains(damiao::MotorHandle* motor) {
  const auto timeout = std::chrono::milliseconds(100);
  return {motor->get_register_f32(25, timeout),
          motor->get_register_f32(26, timeout),
          motor->get_register_f32(27, timeout),
          motor->get_register_f32(28, timeout)};
}

bool gains_match(const PvGains& lhs, const PvGains& rhs) {
  return close(lhs.kp_asr, rhs.kp_asr) && close(lhs.ki_asr, rhs.ki_asr) &&
      close(lhs.kp_apr, rhs.kp_apr) && close(lhs.ki_apr, rhs.ki_apr);
}

void write_gains(damiao::MotorHandle* motor, const PvGains& gains) {
  motor->write_register_f32(25, gains.kp_asr);
  motor->write_register_f32(26, gains.ki_asr);
  motor->write_register_f32(27, gains.kp_apr);
  motor->write_register_f32(28, gains.ki_apr);
  if (!gains_match(read_gains(motor), gains)) {
    throw std::runtime_error("PV gain readback mismatch");
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

std::vector<ArticoreJointPvTarget> make_targets(
    const articore::YunyiRuntimeResources& resources,
    const Product& positions) {
  std::vector<ArticoreJointPvTarget> targets;
  targets.reserve(resources.joints.size());
  for (std::size_t index = 0; index < resources.joints.size(); ++index) {
    targets.push_back(ArticoreJointPvTarget{
        sizeof(ArticoreJointPvTarget), resources.joints[index].motor,
        resources.joints[index].direction * positions[index]});
  }
  return targets;
}

Arrival wait_stable(articore::SafetyRuntime& runtime,
                    const articore::YunyiRuntimeResources& resources,
                    const Product& target) {
  const auto deadline = Clock::now() + std::chrono::seconds(15);
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
  throw std::runtime_error("ordinary PV target did not settle within 15 s");
}

Arrival move(articore::SafetyRuntime& runtime,
             const articore::YunyiRuntimeResources& resources,
             const Product& target, float reference_velocity,
             uint64_t* command_ns = nullptr) {
  const auto commands = make_targets(resources, target);
  const uint64_t start_ns = now_ns();
  runtime.set_joint_pv(commands.data(), static_cast<uint32_t>(commands.size()),
                       reference_velocity, kDriveVelocityLimit);
  if (command_ns) *command_ns = start_ns;
  return wait_stable(runtime, resources, target);
}

void validate_hold(articore::SafetyRuntime& runtime,
                   const articore::YunyiRuntimeResources& resources,
                   const Product& target) {
  const auto deadline = Clock::now() + kHoldDuration;
  while (Clock::now() < deadline) {
    require_healthy(runtime);
    const auto state = read_snapshot(resources);
    float max_velocity = 0.0f;
    float max_error = 0.0f;
    for (std::size_t index = 0; index < state.q.size(); ++index) {
      max_velocity = std::max(max_velocity, std::abs(state.dq[index]));
      max_error = std::max(max_error, std::abs(state.q[index] - target[index]));
    }
    if (max_velocity > kAbortHoldVelocity || max_error > kAbortHoldError) {
      throw std::runtime_error("hold safety threshold exceeded");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void seed_trace_joint_layout(
    articore::SafetyRuntime& runtime,
    const articore::YunyiRuntimeResources& resources,
    const Product& current) {
  articore::NativeTrajectoryRequest request;
  request.mode = ARTICORE_MODE_PV;
  request.joints.reserve(resources.joints.size());
  for (const auto& source : resources.joints) {
    articore::NativeTrajectoryJoint joint;
    joint.role = articore::yunyi_joint_role(
        static_cast<uint32_t>(request.joints.size()));
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
      std::strcmp(argv[1], "--i-understand-test-left-j67-gains-at-2rad") != 0) {
    std::cerr << "Refusing hardware movement/register writes. Pass "
                 "--i-understand-test-left-j67-gains-at-2rad [profile]\n";
    return 2;
  }
  const char* const selected_profile = argc == 3 ? argv[2] : nullptr;
  if (selected_profile &&
      std::none_of(kProfiles.begin(), kProfiles.end(), [&](const Profile& p) {
        return std::strcmp(selected_profile, p.label) == 0;
      })) {
    std::cerr << "Unknown profile: " << selected_profile << '\n';
    return 2;
  }

  constexpr const char* kSegmentsPath =
      "/tmp/articore_left_j67_2rad_gain_segments.csv";
  std::ofstream segments(kSegmentsPath);
  segments << "profile,leg,kp_asr,ki_asr,command_ns,stable_ns,hold_end_ns,"
              "peak_velocity_rad_s,endpoint_error_max_rad\n";
  segments << std::setprecision(9);

  auto bundle = articore::create_yunyi_runtime(ARTICORE_MODE_PV, false);
  auto& runtime = *bundle.runtime;
  auto& resources = *bundle.resources;
  auto* motor_j6 = resources.arm_motors[5];
  auto* motor_j7 = resources.arm_motors[6];
  bool connected = false;
  bool enabled = false;
  bool have_initial = false;
  bool have_originals = false;
  Product initial{};
  PvGains original_j6{};
  PvGains original_j7{};

  const auto restore_gains = [&] {
    require_all_disabled(resources);
    write_gains(motor_j6, original_j6);
    write_gains(motor_j7, original_j7);
  };

  try {
    runtime.connect();
    connected = true;
    require_all_disabled(resources);
    initial = read_snapshot(resources, false).q;
    have_initial = true;
    original_j6 = read_gains(motor_j6);
    original_j7 = read_gains(motor_j7);
    have_originals = true;
    const PvGains expected{0.00372f, 0.0020f, 54.0f, 0.0f};
    if (!gains_match(original_j6, expected) ||
        !gains_match(original_j7, expected)) {
      throw std::runtime_error("left J6/J7 gains do not match expected baseline");
    }

    Product point_a = initial;
    Product point_b = initial;
    std::copy(kPointA.begin(), kPointA.end(), point_a.begin());
    std::copy(kPointB.begin(), kPointB.end(), point_b.begin());

    for (const auto& profile : kProfiles) {
      if (selected_profile &&
          std::strcmp(selected_profile, profile.label) != 0) {
        continue;
      }
      require_all_disabled(resources);
      const PvGains gains{profile.kp_asr, profile.ki_asr,
                          original_j6.kp_apr, original_j6.ki_apr};
      write_gains(motor_j6, gains);
      try {
        write_gains(motor_j7, gains);
      } catch (...) {
        write_gains(motor_j6, original_j6);
        throw;
      }
      std::cout << "PROFILE " << profile.label
                << " kp_asr=" << profile.kp_asr
                << " ki_asr=" << profile.ki_asr << std::endl;

      runtime.enable(ARTICORE_MODE_PV);
      enabled = true;
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      require_healthy(runtime);
      seed_trace_joint_layout(runtime, resources, read_snapshot(resources).q);
      move(runtime, resources, point_a, kStagingReferenceVelocity);

      for (int leg = 0; leg < 2; ++leg) {
        const Product& target = leg == 0 ? point_b : point_a;
        uint64_t command_ns = 0;
        const auto arrival = move(runtime, resources, target,
                                  kTestReferenceVelocity, &command_ns);
        validate_hold(runtime, resources, target);
        const uint64_t hold_end_ns = now_ns();
        segments << profile.label << ',' << (leg == 0 ? "A_to_B" : "B_to_A")
                 << ',' << profile.kp_asr << ',' << profile.ki_asr << ','
                 << command_ns << ',' << arrival.stable_ns << ',' << hold_end_ns
                 << ',' << arrival.peak_velocity << ','
                 << arrival.endpoint_error << '\n';
        segments.flush();
      }

      move(runtime, resources, initial, kStagingReferenceVelocity);
      runtime.disable();
      enabled = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      restore_gains();
      std::cout << "RESTORED " << profile.label << std::endl;
    }

    runtime.disconnect();
    connected = false;
    std::cout << "RESULT segments=" << kSegmentsPath << std::endl;
  } catch (const std::exception& error) {
    std::cerr << "RUN_ERROR " << error.what() << '\n';
    if (enabled) {
      if (have_initial) {
        try {
          move(runtime, resources, initial, kStagingReferenceVelocity);
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
    if (have_originals) {
      try {
        restore_gains();
        std::cerr << "GAINS_RESTORED joints=J6/J7\n";
      } catch (const std::exception& restore_error) {
        std::cerr << "GAIN_RESTORE_ERROR " << restore_error.what() << '\n';
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

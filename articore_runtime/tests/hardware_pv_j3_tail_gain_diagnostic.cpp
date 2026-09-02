#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "articore/detail/yunyi_runtime.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using Product = std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>;

constexpr std::size_t kLeftJ3 = 2;
constexpr std::size_t kRightJ3 = 9;
constexpr float kSpeedPercent = 50.0f;
constexpr float kMoveAmplitude = 0.25f;
constexpr float kPositionTolerance = 0.002f;
constexpr float kVelocityTolerance = 0.020f;
constexpr auto kStableDuration = std::chrono::milliseconds(50);
constexpr auto kPostArrivalObservation = std::chrono::milliseconds(700);
constexpr auto kMoveTimeout = std::chrono::seconds(12);
constexpr float kAbortVelocity = 3.0f;
constexpr float kAbortOvershoot = 0.020f;

struct PvGains {
  float kp_asr;
  float ki_asr;
  float kp_apr;
  float ki_apr;
};

struct Profile {
  const char* label;
  float kp_apr;
  float ki_apr;
  float direction;
};

constexpr std::array<Profile, 3> kProfiles{{
    {"kp198_positive_a", 198.0f, 0.0f, 1.0f},
    {"kp198_negative", 198.0f, 0.0f, -1.0f},
    {"kp198_positive_b", 198.0f, 0.0f, 1.0f},
}};

struct Snapshot {
  uint64_t update_count = 0;
  Product q{};
  Product dq{};
};

struct Result {
  double elapsed_s = 0.0;
  double left_tail_s = 0.0;
  double right_tail_s = 0.0;
  float left_peak_velocity = 0.0f;
  float right_peak_velocity = 0.0f;
  float left_overshoot = 0.0f;
  float right_overshoot = 0.0f;
  float left_post_range = 0.0f;
  float right_post_range = 0.0f;
  float left_final_error = 0.0f;
  float right_final_error = 0.0f;
};

bool close(float lhs, float rhs) {
  return std::abs(lhs - rhs) <=
      1.0e-5f * std::max(1.0f, std::max(std::abs(lhs), std::abs(rhs)));
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
    throw std::runtime_error("J3 PV gain readback mismatch");
  }
}

void require_all_disabled(articore::YunyiRuntimeResources& resources) {
  for (std::size_t index = 0; index < resources.arm_motors.size(); ++index) {
    const auto state = resources.arm_motors[index]->request_fresh_state(
        std::chrono::milliseconds(100));
    if (!state || state->status_code != 0) {
      throw std::runtime_error(
          "joint is not feedback-confirmed disabled at index " +
          std::to_string(index));
    }
  }
}

void require_healthy(const articore::SafetyRuntime& runtime) {
  const auto health = runtime.health();
  if (health.state == ARTICORE_FAULT ||
      health.state == ARTICORE_SAFE_STOP ||
      health.state == ARTICORE_DEGRADED ||
      !health.left_transport.healthy || !health.right_transport.healthy) {
    throw std::runtime_error(
        std::string("unsafe Runtime state: ") +
        (health.fault_reason[0] ? health.fault_reason : health.safety_reason));
  }
}

Snapshot read_snapshot(const articore::YunyiRuntimeResources& resources,
                       bool require_enabled = true) {
  Snapshot result;
  for (std::size_t index = 0; index < resources.joints.size(); ++index) {
    ArticoreMotorState state{};
    ArticoreFeedbackStats stats{};
    const auto& joint = resources.joints[index];
    if (!articore::read_yunyi_motor_state(joint.motor, state, stats) ||
        !state.has_value || !stats.has_feedback ||
        stats.age_ns > 20'000'000ULL || !std::isfinite(state.pos) ||
        !std::isfinite(state.vel)) {
      throw std::runtime_error(
          "joint feedback incomplete or stale at index " +
          std::to_string(index));
    }
    if (require_enabled && state.status_code != 1) {
      throw std::runtime_error(
          "joint is not enabled at index " + std::to_string(index));
    }
    result.q[index] = joint.direction * state.pos;
    result.dq[index] =
        joint.direction * state.vel * joint.velocity_feedback_scale;
    if (index == 0) result.update_count = stats.update_count;
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

void submit_speed50(articore::SafetyRuntime& runtime,
                    const articore::YunyiRuntimeResources& resources,
                    const Product& target) {
  const auto commands = make_targets(resources, target);
  std::vector<float> velocities;
  std::vector<float> accelerations;
  velocities.reserve(commands.size());
  accelerations.reserve(commands.size());
  for (std::size_t index = 0; index < commands.size(); ++index) {
    velocities.push_back(articore::yunyi_ordinary_pv_velocity_limit(
        static_cast<uint32_t>(index), kSpeedPercent));
    accelerations.push_back(articore::yunyi_ordinary_pv_acceleration_limit(
        static_cast<uint32_t>(index), kSpeedPercent));
  }
  runtime.set_joint_pv_profile(
      commands.data(), static_cast<uint32_t>(commands.size()),
      velocities, accelerations);
}

float directed_overshoot(float position, float start, float target) {
  const float direction = std::copysign(1.0f, target - start);
  return std::max(0.0f, direction * (position - target));
}

void write_sample(std::ofstream& output, const Profile& profile,
                  const char* phase, double elapsed_s,
                  const Snapshot& state, const Product& target) {
  output << profile.label << ',' << profile.kp_apr << ',' << profile.ki_apr
         << ',' << phase << ',' << elapsed_s;
  for (float value : state.q) output << ',' << value;
  for (float value : state.dq) output << ',' << value;
  output << ',' << std::abs(state.q[kLeftJ3] - target[kLeftJ3])
         << ',' << std::abs(state.q[kRightJ3] - target[kRightJ3]) << '\n';
}

Result measure_move(articore::SafetyRuntime& runtime,
                    const articore::YunyiRuntimeResources& resources,
                    const Profile& profile, const Product& start,
                    const Product& target, std::ofstream& output,
                    bool record) {
  submit_speed50(runtime, resources, target);
  const auto started = Clock::now();
  const auto deadline = started + kMoveTimeout;
  Clock::time_point stable_since{};
  double left_at_005 = std::numeric_limits<double>::quiet_NaN();
  double right_at_005 = std::numeric_limits<double>::quiet_NaN();
  Result result;
  uint64_t last_update_count = 0;
  Snapshot arrival{};
  while (Clock::now() < deadline) {
    require_healthy(runtime);
    const auto state = read_snapshot(resources);
    if (state.update_count == last_update_count) {
      std::this_thread::sleep_for(std::chrono::microseconds(250));
      continue;
    }
    last_update_count = state.update_count;
    const double elapsed = std::chrono::duration<double>(
        Clock::now() - started).count();
    if (record) write_sample(output, profile, "move", elapsed, state, target);
    const float left_error = std::abs(state.q[kLeftJ3] - target[kLeftJ3]);
    const float right_error = std::abs(state.q[kRightJ3] - target[kRightJ3]);
    if (!std::isfinite(left_at_005) && left_error <= 0.050f) {
      left_at_005 = elapsed;
    }
    if (!std::isfinite(right_at_005) && right_error <= 0.050f) {
      right_at_005 = elapsed;
    }
    result.left_peak_velocity = std::max(
        result.left_peak_velocity, std::abs(state.dq[kLeftJ3]));
    result.right_peak_velocity = std::max(
        result.right_peak_velocity, std::abs(state.dq[kRightJ3]));
    result.left_overshoot = std::max(
        result.left_overshoot,
        directed_overshoot(state.q[kLeftJ3], start[kLeftJ3], target[kLeftJ3]));
    result.right_overshoot = std::max(
        result.right_overshoot,
        directed_overshoot(state.q[kRightJ3], start[kRightJ3], target[kRightJ3]));
    if (std::abs(state.dq[kLeftJ3]) > kAbortVelocity ||
        std::abs(state.dq[kRightJ3]) > kAbortVelocity ||
        result.left_overshoot > kAbortOvershoot ||
        result.right_overshoot > kAbortOvershoot) {
      throw std::runtime_error("J3 gain trial exceeded movement safety limit");
    }
    const bool stable = left_error <= kPositionTolerance &&
        right_error <= kPositionTolerance &&
        std::abs(state.dq[kLeftJ3]) <= kVelocityTolerance &&
        std::abs(state.dq[kRightJ3]) <= kVelocityTolerance;
    if (stable) {
      if (stable_since == Clock::time_point{}) stable_since = Clock::now();
      if (Clock::now() - stable_since >= kStableDuration) {
        result.elapsed_s = elapsed;
        result.left_tail_s = std::isfinite(left_at_005)
            ? elapsed - left_at_005 : elapsed;
        result.right_tail_s = std::isfinite(right_at_005)
            ? elapsed - right_at_005 : elapsed;
        arrival = state;
        break;
      }
    } else {
      stable_since = Clock::time_point{};
    }
  }
  if (result.elapsed_s <= 0.0) {
    throw std::runtime_error("J3 gain trial did not settle within 12 s");
  }

  float left_min = arrival.q[kLeftJ3];
  float left_max = arrival.q[kLeftJ3];
  float right_min = arrival.q[kRightJ3];
  float right_max = arrival.q[kRightJ3];
  const auto observe_deadline = Clock::now() + kPostArrivalObservation;
  while (Clock::now() < observe_deadline) {
    require_healthy(runtime);
    const auto state = read_snapshot(resources);
    left_min = std::min(left_min, state.q[kLeftJ3]);
    left_max = std::max(left_max, state.q[kLeftJ3]);
    right_min = std::min(right_min, state.q[kRightJ3]);
    right_max = std::max(right_max, state.q[kRightJ3]);
    if (record) {
      const double elapsed = std::chrono::duration<double>(
          Clock::now() - started).count();
      write_sample(output, profile, "hold", elapsed, state, target);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const auto final = read_snapshot(resources);
  result.left_post_range = left_max - left_min;
  result.right_post_range = right_max - right_min;
  result.left_final_error = final.q[kLeftJ3] - target[kLeftJ3];
  result.right_final_error = final.q[kRightJ3] - target[kRightJ3];
  return result;
}

Product make_test_target(const articore::YunyiRuntimeResources& resources,
                         const Product& home, float requested_direction) {
  Product target = home;
  for (const auto index : {kLeftJ3, kRightJ3}) {
    const auto& joint = resources.joints[index];
    const float plus_room = joint.upper - home[index];
    const float minus_room = home[index] - joint.lower;
    const float direction = requested_direction > 0.0f
        ? (plus_room >= kMoveAmplitude + 0.05f ? 1.0f : -1.0f)
        : (minus_room >= kMoveAmplitude + 0.05f ? -1.0f : 1.0f);
    target[index] += direction * kMoveAmplitude;
  }
  return target;
}

void print_result(const Profile& profile, const Result& result) {
  std::cout << std::fixed << std::setprecision(6)
            << "RESULT profile=" << profile.label
            << " KP_APR=" << profile.kp_apr
            << " KI_APR=" << profile.ki_apr
            << " settle_s=" << result.elapsed_s
            << " tail_left_s=" << result.left_tail_s
            << " tail_right_s=" << result.right_tail_s
            << " peak_v_left=" << result.left_peak_velocity
            << " peak_v_right=" << result.right_peak_velocity
            << " overshoot_left=" << result.left_overshoot
            << " overshoot_right=" << result.right_overshoot
            << " post_range_left=" << result.left_post_range
            << " post_range_right=" << result.right_post_range
            << " final_error_left=" << result.left_final_error
            << " final_error_right=" << result.right_final_error << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 || std::strcmp(
          argv[1], "--i-understand-both-j3-will-move-and-gains-change") != 0) {
    std::cerr << "Refusing hardware movement/register writes. Pass "
                 "--i-understand-both-j3-will-move-and-gains-change\n";
    return 2;
  }

  const std::string trace_path =
      "/tmp/articore_both_j3_tail_gain_tuning_phase4.csv";
  std::ofstream output(trace_path);
  output << "profile,kp_apr,ki_apr,phase,elapsed_s";
  for (const char* side : {"left", "right"}) {
    for (int joint = 1; joint <= 7; ++joint) {
      output << ",q_" << side << "_j" << joint;
    }
  }
  for (const char* side : {"left", "right"}) {
    for (int joint = 1; joint <= 7; ++joint) {
      output << ",dq_" << side << "_j" << joint;
    }
  }
  output << ",error_left_j3,error_right_j3\n" << std::setprecision(9);

  auto bundle = articore::create_yunyi_runtime(ARTICORE_MODE_PV, false);
  auto& runtime = *bundle.runtime;
  auto& resources = *bundle.resources;
  auto* left_j3 = resources.arm_motors[kLeftJ3];
  auto* right_j3 = resources.arm_motors[kRightJ3];
  bool connected = false;
  bool enabled = false;
  bool have_original = false;
  PvGains original_left{};
  PvGains original_right{};
  Product home{};
  try {
    runtime.connect();
    connected = true;
    if (runtime.configure_mode_for_connect(ARTICORE_MODE_PV) !=
        ARTICORE_OPERATION_OK) {
      throw std::runtime_error("failed to configure native POS_VEL mode");
    }
    require_all_disabled(resources);
    home = read_snapshot(resources, false).q;
    original_left = read_gains(left_j3);
    original_right = read_gains(right_j3);
    have_original = true;
    std::cout << std::setprecision(9)
              << "ORIGINAL left=[" << original_left.kp_asr << ','
              << original_left.ki_asr << ',' << original_left.kp_apr << ','
              << original_left.ki_apr << "] right=[" << original_right.kp_asr
              << ',' << original_right.ki_asr << ',' << original_right.kp_apr
              << ',' << original_right.ki_apr << "]\n"
              << "SPEED_PERCENT " << kSpeedPercent << '\n';

    for (std::size_t profile_index = 0;
         profile_index < kProfiles.size(); ++profile_index) {
      const auto& profile = kProfiles[profile_index];
      const Product target = make_test_target(resources, home, profile.direction);
      require_all_disabled(resources);
      const PvGains left_profile{
          original_left.kp_asr, original_left.ki_asr,
          profile.kp_apr, profile.ki_apr};
      const PvGains right_profile{
          original_right.kp_asr, original_right.ki_asr,
          profile.kp_apr, profile.ki_apr};
      write_gains(left_j3, left_profile);
      write_gains(right_j3, right_profile);
      std::cout << "PROFILE label=" << profile.label
                << " KP_APR=" << profile.kp_apr
                << " KI_APR=" << profile.ki_apr
                << " left_delta=" << target[kLeftJ3] - home[kLeftJ3]
                << " right_delta=" << target[kRightJ3] - home[kRightJ3]
                << std::endl;

      runtime.enable(ARTICORE_MODE_PV);
      enabled = true;
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      require_healthy(runtime);
      const auto result = measure_move(
          runtime, resources, profile, home, target, output, true);
      print_result(profile, result);
      (void)measure_move(
          runtime, resources, profile, target, home, output, false);
      runtime.disable();
      enabled = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
      require_all_disabled(resources);
    }

    write_gains(left_j3, original_left);
    write_gains(right_j3, original_right);
    std::cout << "RESTORED both_J3 gains=original persisted=false\n";
    runtime.disconnect();
    connected = false;
    std::cout << "TRACE " << trace_path << '\n';
  } catch (const std::exception& error) {
    std::cerr << "RUN_ERROR " << error.what() << '\n';
    if (enabled) {
      try {
        runtime.disable();
        enabled = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
      } catch (...) {
      }
    }
    if (have_original) {
      try {
        require_all_disabled(resources);
        write_gains(left_j3, original_left);
        write_gains(right_j3, original_right);
        std::cerr << "GAINS_RESTORED both_J3\n";
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

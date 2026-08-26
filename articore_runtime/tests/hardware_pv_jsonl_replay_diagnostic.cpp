#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "articore/runtime_abi.h"

namespace {

using Clock = std::chrono::steady_clock;
using Product = std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>;

constexpr float kReplaySpeedPercent = 100.0f;
constexpr float kApproachSpeedPercent = 50.0f;
constexpr float kPositionToleranceRad = 0.02f;
constexpr float kVelocityToleranceRadS = 0.05f;
constexpr float kAbortVelocityRadS = 4.0f;
constexpr auto kStableWindow = std::chrono::milliseconds(300);
constexpr auto kSettleTimeout = std::chrono::seconds(12);
constexpr const char* kTracePath =
    "/tmp/articore_pv_jsonl_replay_trace.csv";

struct Frame {
  uint64_t source_ns = 0;
  Product positions{};
};

void check(int32_t result, const char* operation) {
  if (result != ARTICORE_OPERATION_OK) {
    throw std::runtime_error(
        std::string(operation) + " failed: " + articore_runtime_last_error());
  }
}

uint64_t parse_u64(const std::string& line, const std::string& key) {
  const auto key_at = line.find(key);
  if (key_at == std::string::npos) {
    throw std::runtime_error("missing JSON key " + key);
  }
  const char* begin = line.c_str() + key_at + key.size();
  char* end = nullptr;
  const auto value = std::strtoull(begin, &end, 10);
  if (end == begin) throw std::runtime_error("invalid integer after " + key);
  return value;
}

std::array<float, ARTICORE_PRODUCT_ARM_DOF> parse_arm(
    const std::string& line, const std::string& side) {
  const std::string prefix = "\"" + side + "\":{";
  const auto side_at = line.find(prefix);
  if (side_at == std::string::npos) {
    throw std::runtime_error("missing " + side + " arm object");
  }
  const std::string key = "\"command_target_rad\":[";
  const auto key_at = line.find(key, side_at + prefix.size());
  if (key_at == std::string::npos) {
    throw std::runtime_error("missing command_target_rad for " + side);
  }
  const char* cursor = line.c_str() + key_at + key.size();
  std::array<float, ARTICORE_PRODUCT_ARM_DOF> values{};
  for (auto& value : values) {
    char* end = nullptr;
    value = std::strtof(cursor, &end);
    if (end == cursor || !std::isfinite(value)) {
      throw std::runtime_error("invalid command target for " + side);
    }
    cursor = end;
    if (&value != &values.back()) {
      if (*cursor != ',') {
        throw std::runtime_error("invalid command target separator for " + side);
      }
      ++cursor;
    }
  }
  return values;
}

std::vector<Frame> load_frames(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open JSONL input: " + path);
  std::vector<Frame> frames;
  std::string line;
  while (std::getline(input, line)) {
    if (line.find("\"record_type\":\"control_tick\"") ==
            std::string::npos ||
        line.find("\"sdk\":{\"submitted\":true,\"accepted\":true}") ==
            std::string::npos) {
      continue;
    }
    Frame frame;
    frame.source_ns = parse_u64(line, "\"monotonic_ns\":");
    const auto left = parse_arm(line, "left");
    const auto right = parse_arm(line, "right");
    std::copy(left.begin(), left.end(), frame.positions.begin());
    std::copy(right.begin(), right.end(),
              frame.positions.begin() + ARTICORE_PRODUCT_ARM_DOF);
    if (!frames.empty() && frame.source_ns <= frames.back().source_ns) {
      throw std::runtime_error("accepted JSONL timestamps are not increasing");
    }
    frames.push_back(frame);
  }
  if (frames.size() < 2) {
    throw std::runtime_error("JSONL contains fewer than two accepted PV frames");
  }
  return frames;
}

ArticoreProductState read_state(ArticoreRuntime* runtime) {
  ArticoreProductState state{};
  state.struct_size = sizeof(state);
  check(articore_runtime_get_state(runtime, &state), "get_state");
  return state;
}

Product state_positions(const ArticoreProductState& state) {
  Product result{};
  std::copy(std::begin(state.left.positions), std::end(state.left.positions),
            result.begin());
  std::copy(std::begin(state.right.positions), std::end(state.right.positions),
            result.begin() + ARTICORE_PRODUCT_ARM_DOF);
  return result;
}

void require_healthy(ArticoreRuntime* runtime) {
  ArticoreSafetyHealth health{};
  health.struct_size = sizeof(health);
  check(articore_runtime_get_health(runtime, &health), "get_health");
  if (health.state == ARTICORE_FAULT || health.state == ARTICORE_SAFE_STOP ||
      health.degraded || !health.left_transport.healthy ||
      !health.right_transport.healthy) {
    const char* reason = health.fault_reason[0] != '\0'
        ? health.fault_reason : health.safety_reason;
    throw std::runtime_error(
        std::string("unsafe Runtime state: ") + reason);
  }
}

void validate_frames(
    const std::vector<Frame>& frames,
    const ArticoreProductJointAngleVelLimits& limits) {
  for (std::size_t frame_index = 0; frame_index < frames.size(); ++frame_index) {
    for (std::size_t joint = 0; joint < frames[frame_index].positions.size();
         ++joint) {
      const float value = frames[frame_index].positions[joint];
      if (!std::isfinite(value) || value < limits.lower_angles[joint] ||
          value > limits.upper_angles[joint]) {
        throw std::runtime_error(
            "JSONL target exceeds product limits at frame " +
            std::to_string(frame_index) + ", joint " +
            std::to_string(joint));
      }
    }
  }
}

void wait_stable(
    ArticoreRuntime* runtime, const Product& target,
    std::chrono::seconds timeout = kSettleTimeout) {
  const auto deadline = Clock::now() + timeout;
  Clock::time_point stable_since{};
  uint64_t last_sequence = 0;
  while (Clock::now() < deadline) {
    require_healthy(runtime);
    const auto state = read_state(runtime);
    if (state.sequence == last_sequence) {
      std::this_thread::sleep_for(std::chrono::microseconds(250));
      continue;
    }
    last_sequence = state.sequence;
    float maximum_error = 0.0f;
    float maximum_velocity = 0.0f;
    for (std::size_t joint = 0; joint < ARTICORE_PRODUCT_ARM_DOF; ++joint) {
      maximum_error = std::max(
          maximum_error,
          std::abs(state.left.positions[joint] - target[joint]));
      maximum_error = std::max(
          maximum_error,
          std::abs(state.right.positions[joint] -
                   target[joint + ARTICORE_PRODUCT_ARM_DOF]));
      maximum_velocity = std::max(
          maximum_velocity, std::abs(state.left.velocities[joint]));
      maximum_velocity = std::max(
          maximum_velocity, std::abs(state.right.velocities[joint]));
    }
    if (maximum_velocity > kAbortVelocityRadS) {
      throw std::runtime_error("feedback exceeded 4 rad/s abort threshold");
    }
    if (maximum_error <= kPositionToleranceRad &&
        maximum_velocity <= kVelocityToleranceRadS) {
      if (stable_since == Clock::time_point{}) stable_since = Clock::now();
      if (Clock::now() - stable_since >= kStableWindow) return;
    } else {
      stable_since = Clock::time_point{};
    }
  }
  throw std::runtime_error("PV target did not settle within 12 s");
}

struct ReplayMetrics {
  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> peak_velocity{};
  std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF> peak_tracking_error{};
  double maximum_submit_lateness_ms = 0.0;
  uint64_t submitted = 0;
};

ReplayMetrics replay(
    ArticoreRuntime* runtime, const std::vector<Frame>& frames,
    std::ofstream& trace) {
  ReplayMetrics metrics;
  const auto source_start = frames.front().source_ns;
  const auto replay_start = Clock::now() + std::chrono::milliseconds(100);
  uint64_t last_sequence = 0;
  for (std::size_t index = 0; index < frames.size(); ++index) {
    const auto offset = std::chrono::nanoseconds(
        frames[index].source_ns - source_start);
    const auto deadline = replay_start + offset;
    std::this_thread::sleep_until(deadline);
    const double lateness_ms = std::max(
        0.0, std::chrono::duration<double, std::milli>(
                 Clock::now() - deadline).count());
    metrics.maximum_submit_lateness_ms = std::max(
        metrics.maximum_submit_lateness_ms, lateness_ms);
    check(articore_runtime_set_joint_pv(
              runtime, frames[index].positions.data(),
              static_cast<uint32_t>(frames[index].positions.size()),
              kReplaySpeedPercent),
          "set_joint_pv replay");
    ++metrics.submitted;
    require_healthy(runtime);
    const auto state = read_state(runtime);
    if (state.sequence == last_sequence) continue;
    last_sequence = state.sequence;
    trace << index << ',' << frames[index].source_ns << ','
          << std::setprecision(10) << lateness_ms << ',' << state.sequence;
    for (std::size_t joint = 0; joint < ARTICORE_PRODUCT_ARM_DOF; ++joint) {
      const float left_error = std::abs(
          state.left.positions[joint] - frames[index].positions[joint]);
      const float right_error = std::abs(
          state.right.positions[joint] -
          frames[index].positions[joint + ARTICORE_PRODUCT_ARM_DOF]);
      metrics.peak_velocity[joint] = std::max(
          metrics.peak_velocity[joint], std::abs(state.left.velocities[joint]));
      metrics.peak_velocity[joint + ARTICORE_PRODUCT_ARM_DOF] = std::max(
          metrics.peak_velocity[joint + ARTICORE_PRODUCT_ARM_DOF],
          std::abs(state.right.velocities[joint]));
      metrics.peak_tracking_error[joint] = std::max(
          metrics.peak_tracking_error[joint], left_error);
      metrics.peak_tracking_error[joint + ARTICORE_PRODUCT_ARM_DOF] = std::max(
          metrics.peak_tracking_error[joint + ARTICORE_PRODUCT_ARM_DOF],
          right_error);
      trace << ',' << frames[index].positions[joint] << ','
            << state.left.positions[joint] << ',' << state.left.velocities[joint];
      trace << ',' << frames[index].positions[joint + ARTICORE_PRODUCT_ARM_DOF]
            << ',' << state.right.positions[joint] << ','
            << state.right.velocities[joint];
    }
    trace << '\n';
  }
  trace.flush();
  return metrics;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3 || std::string(argv[1]) !=
          "--i-understand-both-arms-will-replay") {
    std::cerr << "Refusing to move hardware. Pass "
                 "--i-understand-both-arms-will-replay <control.jsonl>\n";
    return 2;
  }

  ArticoreRuntime* runtime = nullptr;
  bool connected = false;
  bool enabled = false;
  Product initial{};
  try {
    const auto frames = load_frames(argv[2]);
    const double source_duration_s = static_cast<double>(
        frames.back().source_ns - frames.front().source_ns) / 1.0e9;
    std::cout << "SOURCE accepted_frames=" << frames.size()
              << " duration_s=" << source_duration_s
              << " replay_speed_percent=" << kReplaySpeedPercent << '\n';

    check(articore_runtime_create_yunyi(ARTICORE_MODE_PV, 0, &runtime),
          "create_yunyi");
    ArticoreProductJointAngleVelLimits limits{};
    limits.struct_size = sizeof(limits);
    check(articore_runtime_get_joint_angle_vel_limits(runtime, &limits),
          "get_joint_angle_vel_limits");
    validate_frames(frames, limits);

    check(articore_runtime_connect(runtime), "connect");
    connected = true;
    require_healthy(runtime);
    initial = state_positions(read_state(runtime));
    check(articore_runtime_enable(runtime), "enable");
    enabled = true;

    std::cout << "APPROACH first accepted frame at 50 percent\n";
    check(articore_runtime_set_joint_pv(
              runtime, frames.front().positions.data(),
              static_cast<uint32_t>(frames.front().positions.size()),
              kApproachSpeedPercent),
          "approach first frame");
    wait_stable(runtime, frames.front().positions);

    std::ofstream trace(kTracePath);
    trace << "frame,source_ns,submit_lateness_ms,feedback_sequence";
    for (std::size_t joint = 0; joint < ARTICORE_PRODUCT_ARM_DOF; ++joint) {
      trace << ",left_target_j" << joint + 1 << ",left_actual_j" << joint + 1
            << ",left_velocity_j" << joint + 1;
      trace << ",right_target_j" << joint + 1 << ",right_actual_j" << joint + 1
            << ",right_velocity_j" << joint + 1;
    }
    trace << '\n';
    const auto metrics = replay(runtime, frames, trace);
    wait_stable(runtime, frames.back().positions);

    std::cout << "REPLAY submitted=" << metrics.submitted
              << " maximum_submit_lateness_ms="
              << metrics.maximum_submit_lateness_ms << '\n';
    for (std::size_t joint = 0; joint < ARTICORE_PRODUCT_DUAL_ARM_DOF; ++joint) {
      std::cout << (joint < ARTICORE_PRODUCT_ARM_DOF ? "left/l-joint"
                                                     : "right/r-joint")
                << (joint % ARTICORE_PRODUCT_ARM_DOF) + 1
                << " peak_velocity_rad_s=" << metrics.peak_velocity[joint]
                << " peak_tracking_error_rad="
                << metrics.peak_tracking_error[joint] << '\n';
    }

    std::cout << "RESTORE initial position at 50 percent\n";
    check(articore_runtime_set_joint_pv(
              runtime, initial.data(), static_cast<uint32_t>(initial.size()),
              kApproachSpeedPercent),
          "restore initial");
    wait_stable(runtime, initial);
    check(articore_runtime_disable(runtime), "disable");
    enabled = false;
    check(articore_runtime_disconnect(runtime), "disconnect");
    connected = false;
    articore_runtime_free(runtime);
    runtime = nullptr;
    std::cout << "RESULT trace=" << kTracePath << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "RUN_ERROR " << error.what() << '\n';
    if (runtime && enabled) {
      articore_runtime_set_joint_pv(
          runtime, initial.data(), static_cast<uint32_t>(initial.size()), 20.0f);
      std::this_thread::sleep_for(std::chrono::seconds(3));
      articore_runtime_disable(runtime);
    }
    if (runtime && connected) articore_runtime_disconnect(runtime);
    if (runtime) articore_runtime_free(runtime);
    return 1;
  }
}

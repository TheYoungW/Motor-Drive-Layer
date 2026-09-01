#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "articore/runtime_abi.h"

namespace {

using Clock = std::chrono::steady_clock;
using Product = std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>;

constexpr float kSpeedPercent = 50.0f;
constexpr float kMinimumMaximumDeltaRad = 0.950f;
constexpr float kMaximumMaximumDeltaRad = 1.100f;
constexpr float kActiveJointDeltaRad = 0.100f;
constexpr std::size_t kMinimumActiveJoints = 8;
constexpr float kPositionToleranceRad = 0.002f;
constexpr float kVelocityToleranceRadS = 0.020f;
constexpr float kAbortVelocityRadS = 4.0f;
constexpr auto kTimeout = std::chrono::seconds(30);
constexpr auto kStableDuration = std::chrono::milliseconds(50);
constexpr auto kObservationDuration = std::chrono::seconds(1);
constexpr const char* kTracePath =
    "/tmp/articore_pv_single_random_large_speed50_trace.csv";

struct Frame {
  uint64_t tick = 0;
  Product positions{};
};

struct Snapshot {
  uint64_t timestamp_ns = 0;
  uint64_t sequence = 0;
  Product positions{};
  Product velocities{};
};

void check(int32_t result, const char* operation) {
  if (result != ARTICORE_OPERATION_OK) {
    throw std::runtime_error(
        std::string(operation) + " failed: " + articore_runtime_last_error());
  }
}

Snapshot read_snapshot(ArticoreRuntime* runtime) {
  ArticoreProductState state{};
  state.struct_size = sizeof(state);
  check(articore_runtime_get_state(runtime, &state), "get_state");
  Snapshot result;
  result.timestamp_ns = state.timestamp_ns;
  result.sequence = state.sequence;
  std::copy(std::begin(state.left.positions), std::end(state.left.positions),
            result.positions.begin());
  std::copy(std::begin(state.right.positions), std::end(state.right.positions),
            result.positions.begin() + ARTICORE_PRODUCT_ARM_DOF);
  std::copy(std::begin(state.left.velocities), std::end(state.left.velocities),
            result.velocities.begin());
  std::copy(std::begin(state.right.velocities), std::end(state.right.velocities),
            result.velocities.begin() + ARTICORE_PRODUCT_ARM_DOF);
  return result;
}

void require_healthy(ArticoreRuntime* runtime) {
  ArticoreSafetyHealth health{};
  health.struct_size = sizeof(health);
  check(articore_runtime_get_health(runtime, &health), "get_health");
  if (health.state == ARTICORE_SAFE_HOLD || health.state == ARTICORE_FAULT ||
      health.state == ARTICORE_SAFE_STOP || health.degraded ||
      !health.left_transport.healthy || !health.right_transport.healthy) {
    const char* reason = health.last_operation_error[0] != '\0'
        ? health.last_operation_error
        : (health.fault_reason[0] != '\0' ? health.fault_reason
                                         : health.safety_reason);
    throw std::runtime_error(std::string("unsafe Runtime state: ") + reason);
  }
}

void print_product(const char* label, const Product& values) {
  std::cout << label << "=[" << std::setprecision(7);
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0) std::cout << ',';
    std::cout << values[index];
  }
  std::cout << "]\n";
}

uint64_t parse_u64(const std::string& line, const std::string& key) {
  const auto key_at = line.find(key);
  if (key_at == std::string::npos) {
    throw std::runtime_error("missing JSON key " + key);
  }
  const char* begin = line.c_str() + key_at + key.size();
  char* end = nullptr;
  const uint64_t value = std::strtoull(begin, &end, 10);
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
  for (std::size_t index = 0; index < values.size(); ++index) {
    char* end = nullptr;
    values[index] = std::strtof(cursor, &end);
    if (end == cursor || !std::isfinite(values[index])) {
      throw std::runtime_error("invalid command target for " + side);
    }
    cursor = end;
    if (index + 1 < values.size()) {
      if (*cursor != ',') {
        throw std::runtime_error("invalid target separator for " + side);
      }
      ++cursor;
    }
  }
  return values;
}

std::vector<Frame> load_frames(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open JSONL input: " + path);
  std::vector<Frame> result;
  std::string line;
  while (std::getline(input, line)) {
    if (line.find("\"record_type\":\"control_tick\"") ==
            std::string::npos ||
        line.find("\"sdk\":{\"submitted\":true,\"accepted\":true") ==
            std::string::npos) {
      continue;
    }
    Frame frame;
    frame.tick = parse_u64(line, "\"tick\":");
    const auto left = parse_arm(line, "left");
    const auto right = parse_arm(line, "right");
    std::copy(left.begin(), left.end(), frame.positions.begin());
    std::copy(right.begin(), right.end(),
              frame.positions.begin() + ARTICORE_PRODUCT_ARM_DOF);
    result.push_back(frame);
  }
  if (result.empty()) throw std::runtime_error("no accepted frames in JSONL");
  return result;
}

bool inside_limits(
    const Product& target,
    const ArticoreProductJointAngleVelLimits& limits) {
  for (std::size_t joint = 0; joint < target.size(); ++joint) {
    if (target[joint] < limits.lower_angles[joint] ||
        target[joint] > limits.upper_angles[joint]) {
      return false;
    }
  }
  return true;
}

struct Selection {
  Frame frame;
  Product delta{};
  float maximum_delta = 0.0f;
  std::size_t active_joints = 0;
  std::size_t candidate_count = 0;
};

Selection select_random_large_frame(
    const std::vector<Frame>& frames,
    const Product& current,
    const ArticoreProductJointAngleVelLimits& limits,
    uint64_t seed) {
  std::vector<Selection> candidates;
  for (const auto& frame : frames) {
    if (!inside_limits(frame.positions, limits)) continue;
    Selection candidate;
    candidate.frame = frame;
    for (std::size_t joint = 0; joint < current.size(); ++joint) {
      candidate.delta[joint] = frame.positions[joint] - current[joint];
      const float magnitude = std::abs(candidate.delta[joint]);
      candidate.maximum_delta = std::max(candidate.maximum_delta, magnitude);
      if (magnitude >= kActiveJointDeltaRad) ++candidate.active_joints;
    }
    if (candidate.maximum_delta >= kMinimumMaximumDeltaRad &&
        candidate.maximum_delta <= kMaximumMaximumDeltaRad &&
        candidate.active_joints >= kMinimumActiveJoints) {
      candidates.push_back(candidate);
    }
  }
  if (candidates.empty()) {
    throw std::runtime_error(
        "no in-limit accepted target matches the 0.950-1.100 rad large-move band");
  }
  const auto random_priority = [seed](uint64_t tick) {
    uint64_t value = tick + seed + 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31U);
  };
  const auto selected_at = std::min_element(
      candidates.begin(), candidates.end(),
      [&random_priority](const Selection& left, const Selection& right) {
        return random_priority(left.frame.tick) <
               random_priority(right.frame.tick);
      });
  Selection selected = *selected_at;
  selected.candidate_count = candidates.size();
  return selected;
}

}  // namespace

int main(int argc, char** argv) {
  const bool inspect = argc == 4 && std::string(argv[1]) == "--inspect";
  const bool move = argc == 4 &&
      std::string(argv[1]) ==
          "--i-understand-both-arms-will-move-once";
  if (!inspect && !move) {
    std::cerr << "Usage:\n  " << argv[0]
              << " --inspect <control.jsonl> <seed>\n  " << argv[0]
              << " --i-understand-both-arms-will-move-once "
                 "<control.jsonl> <seed>\n";
    return 2;
  }

  ArticoreRuntime* runtime = nullptr;
  bool connected = false;
  bool enabled = false;
  try {
    check(articore_runtime_create_yunyi(ARTICORE_MODE_PV, 0, &runtime),
          "create_yunyi");
    ArticoreProductJointAngleVelLimits limits{};
    limits.struct_size = sizeof(limits);
    check(articore_runtime_get_joint_angle_vel_limits(runtime, &limits),
          "get_joint_angle_vel_limits");
    check(articore_runtime_connect(runtime), "connect");
    connected = true;
    require_healthy(runtime);
    const Snapshot start = read_snapshot(runtime);
    const uint64_t seed = std::stoull(argv[3]);
    const auto frames = load_frames(argv[2]);
    const Selection selected =
        select_random_large_frame(frames, start.positions, limits, seed);
    const Product& target = selected.frame.positions;
    print_product("CURRENT_RAD", start.positions);
    print_product("TARGET_RAD", target);
    print_product("DELTA_RAD", selected.delta);
    std::cout << "SELECTED source_tick=" << selected.frame.tick
              << " seed=" << seed
              << " candidates=" << selected.candidate_count
              << " max_delta_rad=" << selected.maximum_delta
              << " active_joints_ge_0.100_rad=" << selected.active_joints
              << '\n';
    std::cout << "PLAN command_count=1 speed_percent=" << kSpeedPercent << '\n';

    if (inspect) {
      check(articore_runtime_disconnect(runtime), "disconnect");
      connected = false;
      articore_runtime_free(runtime);
      runtime = nullptr;
      std::cout << "INSPECT_ONLY motor_enable=false command_submitted=false\n";
      return 0;
    }

    std::ofstream trace(kTracePath);
    if (!trace) throw std::runtime_error("cannot open trace output");
    trace << "timestamp_ns,sequence,max_error_rad,max_velocity_rad_s";
    for (const char* side : {"left", "right"}) {
      for (int joint = 1; joint <= ARTICORE_PRODUCT_ARM_DOF; ++joint) {
        trace << ",target_" << side << "_j" << joint
              << ",q_" << side << "_j" << joint
              << ",dq_" << side << "_j" << joint;
      }
    }
    trace << '\n' << std::setprecision(10);

    check(articore_runtime_enable(runtime), "enable");
    enabled = true;
    check(articore_runtime_set_joint_pv(
              runtime, target.data(), static_cast<uint32_t>(target.size()),
              kSpeedPercent),
          "set_joint_pv single endpoint");
    std::cout << "SUBMITTED ordinary_pv_commands=1\n";

    const auto deadline = Clock::now() + kTimeout;
    uint64_t last_sequence = start.sequence;
    Clock::time_point stable_since{};
    Clock::time_point arrived_at{};
    float final_error = 0.0f;
    float peak_velocity = 0.0f;
    while (Clock::now() < deadline) {
      require_healthy(runtime);
      const Snapshot state = read_snapshot(runtime);
      if (state.sequence == last_sequence) {
        std::this_thread::sleep_for(std::chrono::microseconds(250));
        continue;
      }
      last_sequence = state.sequence;
      float maximum_error = 0.0f;
      float maximum_velocity = 0.0f;
      for (std::size_t index = 0; index < target.size(); ++index) {
        maximum_error = std::max(
            maximum_error, std::abs(target[index] - state.positions[index]));
        maximum_velocity = std::max(
            maximum_velocity, std::abs(state.velocities[index]));
      }
      if (maximum_velocity > kAbortVelocityRadS) {
        throw std::runtime_error("feedback exceeded 4 rad/s abort threshold");
      }
      peak_velocity = std::max(peak_velocity, maximum_velocity);
      trace << state.timestamp_ns << ',' << state.sequence << ','
            << maximum_error << ',' << maximum_velocity;
      for (std::size_t index = 0; index < target.size(); ++index) {
        trace << ',' << target[index] << ',' << state.positions[index]
              << ',' << state.velocities[index];
      }
      trace << '\n';

      const bool stable = maximum_error <= kPositionToleranceRad &&
                          maximum_velocity <= kVelocityToleranceRadS;
      if (stable) {
        if (stable_since == Clock::time_point{}) stable_since = Clock::now();
        if (arrived_at == Clock::time_point{} &&
            Clock::now() - stable_since >= kStableDuration) {
          arrived_at = Clock::now();
          final_error = maximum_error;
          std::cout << "ARRIVED max_error_rad=" << final_error
                    << " peak_velocity_rad_s=" << peak_velocity << '\n';
        }
      } else if (arrived_at == Clock::time_point{}) {
        stable_since = Clock::time_point{};
      }
      if (arrived_at != Clock::time_point{} &&
          Clock::now() - arrived_at >= kObservationDuration) {
        check(articore_runtime_disable(runtime), "disable");
        enabled = false;
        check(articore_runtime_disconnect(runtime), "disconnect");
        connected = false;
        articore_runtime_free(runtime);
        runtime = nullptr;
        std::cout << "DONE observed_after_arrival_s=1 trace=" << kTracePath
                  << '\n';
        return 0;
      }
    }
    throw std::runtime_error("single endpoint did not settle within 30 s");
  } catch (const std::exception& error) {
    std::cerr << "RUN_ERROR " << error.what() << '\n';
    if (runtime && enabled) articore_runtime_disable(runtime);
    if (runtime && connected) articore_runtime_disconnect(runtime);
    if (runtime) articore_runtime_free(runtime);
    return 1;
  }
}

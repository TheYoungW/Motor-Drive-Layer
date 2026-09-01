#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "articore/runtime_abi.h"

namespace {

using Clock = std::chrono::steady_clock;
using Product = std::array<float, ARTICORE_PRODUCT_DUAL_ARM_DOF>;

constexpr uint32_t kDefaultRandomSeed = 20260831U;
constexpr std::size_t kRandomPairCount = 6;
constexpr std::size_t kMinimumPairGap = 20;
constexpr std::size_t kMaximumPairGap = 150;
constexpr float kMinimumPairMaximumDeltaRad = 0.06f;
constexpr float kMaximumPairMaximumDeltaRad = 0.12f;
constexpr float kSelectionActiveJointDeltaRad = 0.04f;
constexpr std::size_t kMinimumSelectionActiveJoints = 3;
constexpr float kOtherJointPairMaximumJ34DeltaRad = 0.015f;
constexpr float kOtherJointPairMinimumMaximumDeltaRad = 0.060f;
constexpr float kOtherJointPairMaximumMaximumDeltaRad = 0.150f;
constexpr std::size_t kOtherJointPairMinimumActiveJoints = 3;
constexpr float kMeasuredSpeedPercent = 50.0f;
constexpr float kApproachSpeedPercent = 50.0f;
constexpr float kActiveJointDeltaRad = 0.02f;
constexpr float kPositionToleranceRad = 0.002f;
constexpr float kVelocityToleranceRadS = 0.02f;
constexpr float kReverseVelocityThresholdRadS = 0.02f;
constexpr float kAbortVelocityRadS = 4.0f;
constexpr uint64_t kStableWindowNs = 50'000'000ULL;
constexpr uint64_t kObviousReverseDurationNs = 10'000'000ULL;
constexpr uint64_t kPostArrivalObservationNs = 1'000'000'000ULL;
constexpr double kTailGateMs = 150.0;
constexpr double kVisualQuietGateMs = 20.0;
constexpr float kVisualWindowPeakVelocityRadS = 0.030f;
constexpr float kVisualMaximumPositionSpanRad = 0.002f;
constexpr auto kMoveTimeout = std::chrono::seconds(8);
constexpr const char* kTracePath =
    "/tmp/articore_pv_tail_speed50_go_j34_kp54_trace.csv";
constexpr const char* kOtherJointTracePath =
    "/tmp/articore_pv_tail_speed50_go_j34_small_kp54_trace.csv";

struct Frame {
  uint64_t tick = 0;
  Product positions{};
};

struct TestPair {
  std::size_t from = 0;
  std::size_t to = 0;
};

struct Snapshot {
  uint64_t timestamp_ns = 0;
  uint64_t sequence = 0;
  Product positions{};
  Product velocities{};
};

struct TrialMetrics {
  int trial = 0;
  uint64_t from_tick = 0;
  uint64_t to_tick = 0;
  double elapsed_ms = 0.0;
  double tail_ms = 0.0;
  float final_max_error_rad = 0.0f;
  float peak_velocity_rad_s = 0.0f;
  float peak_reverse_velocity_rad_s = 0.0f;
  double longest_reverse_ms = 0.0;
  double position_window_to_stable_ms = 0.0;
  double position_window_to_quiet_ms = 0.0;
  float position_window_peak_velocity_rad_s = 0.0f;
  float position_window_max_position_span_rad = 0.0f;
  float post_hold_peak_velocity_rad_s = 0.0f;
  float post_hold_max_position_span_rad = 0.0f;
  double post_hold_longest_motion_ms = 0.0;
  bool obvious_reverse = false;
  std::size_t samples = 0;
  std::size_t post_hold_samples = 0;
};

uint64_t clock_now_ns() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          Clock::now().time_since_epoch()).count());
}

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
  for (std::size_t index = 0; index < values.size(); ++index) {
    char* end = nullptr;
    values[index] = std::strtof(cursor, &end);
    if (end == cursor || !std::isfinite(values[index])) {
      throw std::runtime_error("invalid command target for " + side);
    }
    cursor = end;
    if (index + 1 < values.size()) {
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
  if (result.size() < kRandomPairCount * (kMaximumPairGap + 1)) {
    throw std::runtime_error("too few accepted frames for random pair selection");
  }
  return result;
}

std::pair<float, std::size_t> pair_shape(
    const Frame& from, const Frame& to) {
  float maximum_delta = 0.0f;
  std::size_t active = 0;
  for (std::size_t joint = 0; joint < from.positions.size(); ++joint) {
    const float delta = std::abs(to.positions[joint] - from.positions[joint]);
    maximum_delta = std::max(maximum_delta, delta);
    if (delta >= kSelectionActiveJointDeltaRad) ++active;
  }
  return {maximum_delta, active};
}

struct OtherJointPairShape {
  float maximum_j34_delta = 0.0f;
  float maximum_other_delta = 0.0f;
  std::size_t active_other_joints = 0;
};

OtherJointPairShape other_joint_pair_shape(
    const Frame& from, const Frame& to) {
  OtherJointPairShape result;
  for (std::size_t joint = 0; joint < from.positions.size(); ++joint) {
    const float delta = std::abs(to.positions[joint] - from.positions[joint]);
    const std::size_t arm_joint = joint % ARTICORE_PRODUCT_ARM_DOF;
    const bool joint34 = arm_joint == 2 || arm_joint == 3;
    if (joint34) {
      result.maximum_j34_delta = std::max(
          result.maximum_j34_delta, delta);
    } else {
      result.maximum_other_delta = std::max(
          result.maximum_other_delta, delta);
      if (delta >= kSelectionActiveJointDeltaRad) {
        ++result.active_other_joints;
      }
    }
  }
  return result;
}

std::vector<TestPair> select_random_pairs(
    const std::vector<Frame>& frames, uint32_t random_seed,
    bool other_joints_only) {
  std::mt19937 random(random_seed);
  std::uniform_int_distribution<std::size_t> gap_distribution(
      kMinimumPairGap, kMaximumPairGap);
  std::vector<TestPair> result;
  result.reserve(kRandomPairCount);
  for (std::size_t stratum = 0; stratum < kRandomPairCount; ++stratum) {
    const std::size_t lower = stratum * frames.size() / kRandomPairCount;
    const std::size_t upper =
        (stratum + 1) * frames.size() / kRandomPairCount;
    if (upper <= lower + kMaximumPairGap + 1) {
      throw std::runtime_error("random-pair stratum is too small");
    }
    if (other_joints_only) {
      std::vector<TestPair> candidates;
      for (std::size_t from = lower;
           from + kMinimumPairGap < upper; ++from) {
        const std::size_t maximum_gap = std::min(
            kMaximumPairGap, upper - from - 1);
        for (std::size_t gap = kMinimumPairGap;
             gap <= maximum_gap; ++gap) {
          const std::size_t to = from + gap;
          const auto shape = other_joint_pair_shape(frames[from], frames[to]);
          if (shape.maximum_j34_delta <=
                  kOtherJointPairMaximumJ34DeltaRad &&
              shape.maximum_other_delta >=
                  kOtherJointPairMinimumMaximumDeltaRad &&
              shape.maximum_other_delta <=
                  kOtherJointPairMaximumMaximumDeltaRad &&
              shape.active_other_joints >=
                  kOtherJointPairMinimumActiveJoints) {
            candidates.push_back({from, to});
          }
        }
      }
      if (candidates.empty()) {
        throw std::runtime_error(
            "could not select a J3/J4-small random pair in stratum " +
            std::to_string(stratum));
      }
      std::uniform_int_distribution<std::size_t> candidate_distribution(
          0, candidates.size() - 1);
      result.push_back(candidates[candidate_distribution(random)]);
      continue;
    }
    std::uniform_int_distribution<std::size_t> start_distribution(
        lower, upper - kMaximumPairGap - 1);
    bool found = false;
    for (std::size_t attempt = 0; attempt < 20'000; ++attempt) {
      const std::size_t from = start_distribution(random);
      const std::size_t to = from + gap_distribution(random);
      const auto [maximum_delta, active] = pair_shape(frames[from], frames[to]);
      if (maximum_delta >= kMinimumPairMaximumDeltaRad &&
          maximum_delta <= kMaximumPairMaximumDeltaRad &&
          active >= kMinimumSelectionActiveJoints) {
        result.push_back({from, to});
        found = true;
        break;
      }
    }
    if (!found) {
      throw std::runtime_error(
          "could not select a bounded random pair in stratum " +
          std::to_string(stratum));
    }
  }
  return result;
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

void validate_frames(
    const std::vector<Frame>& frames,
    const ArticoreProductJointAngleVelLimits& limits) {
  for (const auto& frame : frames) {
    for (std::size_t joint = 0; joint < frame.positions.size(); ++joint) {
      const float value = frame.positions[joint];
      if (!std::isfinite(value) || value < limits.lower_angles[joint] ||
          value > limits.upper_angles[joint]) {
        throw std::runtime_error(
            "accepted JSONL frame exceeds product limit at tick " +
            std::to_string(frame.tick) + ", joint " +
            std::to_string(joint));
      }
    }
  }
}

void print_plan(const std::vector<Frame>& frames,
                const std::vector<TestPair>& pairs, uint32_t random_seed,
                bool other_joints_only) {
  std::cout << "PLAN accepted_frames=" << frames.size()
            << " random_seed=" << random_seed
            << " stratified_pairs=" << pairs.size()
            << " measured_speed_percent=" << kMeasuredSpeedPercent
            << " selection="
            << (other_joints_only ? "j34_small_other_joints_large"
                                  : "general_random")
            << " measured_direction=from_to_only\n";
  for (std::size_t index = 0; index < pairs.size(); ++index) {
    const auto [maximum_delta, active] = pair_shape(
        frames[pairs[index].from], frames[pairs[index].to]);
    std::cout << "PAIR index=" << index + 1
              << " from_tick=" << frames[pairs[index].from].tick
              << " to_tick=" << frames[pairs[index].to].tick
              << " accepted_frame_gap="
              << pairs[index].to - pairs[index].from
              << " maximum_joint_delta_rad=" << maximum_delta
              << " joints_delta_ge_0p04=" << active;
    if (other_joints_only) {
      const auto other_shape = other_joint_pair_shape(
          frames[pairs[index].from], frames[pairs[index].to]);
      std::cout << " maximum_j34_delta_rad="
                << other_shape.maximum_j34_delta
                << " maximum_other_delta_rad="
                << other_shape.maximum_other_delta
                << " other_joints_delta_ge_0p04="
                << other_shape.active_other_joints;
    }
    std::cout << '\n';
  }
}

void write_trace_header(std::ofstream& output) {
  output << "trial,from_tick,to_tick,speed_percent,timestamp_ns,sequence,"
            "tail_active";
  for (const char* side : {"left", "right"}) {
    for (int joint = 1; joint <= ARTICORE_PRODUCT_ARM_DOF; ++joint) {
      output << ",target_" << side << "_j" << joint
             << ",q_" << side << "_j" << joint
             << ",dq_" << side << "_j" << joint;
    }
  }
  output << '\n' << std::setprecision(10);
}

void write_trace_sample(std::ofstream& output, int trial,
                        uint64_t from_tick, uint64_t to_tick,
                        bool tail_active, const Product& target,
                        const Snapshot& state) {
  output << trial << ',' << from_tick << ',' << to_tick << ','
         << kMeasuredSpeedPercent << ',' << state.timestamp_ns << ','
         << state.sequence << ','
         << (tail_active ? 1 : 0);
  for (std::size_t joint = 0; joint < target.size(); ++joint) {
    output << ',' << target[joint] << ',' << state.positions[joint]
           << ',' << state.velocities[joint];
  }
  output << '\n';
}

TrialMetrics measure_move(ArticoreRuntime* runtime, int trial,
                          const Frame& from, const Frame& to,
                          std::ofstream& trace) {
  require_healthy(runtime);
  const Snapshot start = read_snapshot(runtime);
  Product delta{};
  Product direction{};
  std::array<bool, ARTICORE_PRODUCT_DUAL_ARM_DOF> active{};
  std::size_t active_count = 0;
  for (std::size_t joint = 0; joint < delta.size(); ++joint) {
    delta[joint] = to.positions[joint] - start.positions[joint];
    active[joint] = std::abs(delta[joint]) >= kActiveJointDeltaRad;
    if (active[joint]) {
      direction[joint] = std::copysign(1.0f, delta[joint]);
      ++active_count;
    }
  }
  if (active_count < 3) {
    throw std::runtime_error("measured move has fewer than three active joints");
  }

  const uint64_t command_ns = clock_now_ns();
  check(articore_runtime_set_joint_pv(
            runtime, to.positions.data(),
            static_cast<uint32_t>(to.positions.size()),
            kMeasuredSpeedPercent),
        "set_joint_pv measured move");

  const auto deadline = Clock::now() + kMoveTimeout;
  uint64_t last_sequence = start.sequence;
  uint64_t tail_start_ns = 0;
  uint64_t position_window_start_ns = 0;
  uint64_t stable_start_ns = 0;
  uint64_t arrival_confirmed_ns = 0;
  uint64_t post_hold_motion_start_ns = 0;
  std::array<uint64_t, ARTICORE_PRODUCT_DUAL_ARM_DOF> reverse_start_ns{};
  Product post_hold_minimum{};
  Product post_hold_maximum{};
  Product position_window_minimum{};
  Product position_window_maximum{};
  Snapshot last_state = start;
  TrialMetrics result;
  result.trial = trial;
  result.from_tick = from.tick;
  result.to_tick = to.tick;

  while (Clock::now() < deadline) {
    require_healthy(runtime);
    const Snapshot state = read_snapshot(runtime);
    if (state.sequence == last_sequence) {
      std::this_thread::sleep_for(std::chrono::microseconds(250));
      continue;
    }
    last_sequence = state.sequence;
    last_state = state;
    ++result.samples;

    float maximum_error = 0.0f;
    float maximum_velocity = 0.0f;
    bool inside_last_five_percent = true;
    for (std::size_t joint = 0; joint < delta.size(); ++joint) {
      const float error = std::abs(to.positions[joint] - state.positions[joint]);
      maximum_error = std::max(maximum_error, error);
      maximum_velocity = std::max(
          maximum_velocity, std::abs(state.velocities[joint]));
      if (active[joint]) {
        const float tail_band = std::max(
            kPositionToleranceRad, 0.05f * std::abs(delta[joint]));
        if (error > tail_band) inside_last_five_percent = false;
      }
    }
    result.peak_velocity_rad_s = std::max(
        result.peak_velocity_rad_s, maximum_velocity);
    if (maximum_velocity > kAbortVelocityRadS) {
      throw std::runtime_error("feedback exceeded 4 rad/s abort threshold");
    }
    if (tail_start_ns == 0 && inside_last_five_percent) {
      tail_start_ns = state.timestamp_ns;
    }

    if (tail_start_ns != 0) {
      for (std::size_t joint = 0; joint < delta.size(); ++joint) {
        if (!active[joint]) continue;
        const float projected_velocity =
            state.velocities[joint] * direction[joint];
        result.peak_reverse_velocity_rad_s = std::max(
            result.peak_reverse_velocity_rad_s,
            std::max(0.0f, -projected_velocity));
        if (projected_velocity < -kReverseVelocityThresholdRadS) {
          if (reverse_start_ns[joint] == 0) {
            reverse_start_ns[joint] = state.timestamp_ns;
          }
          const uint64_t duration_ns =
              state.timestamp_ns - reverse_start_ns[joint];
          result.longest_reverse_ms = std::max(
              result.longest_reverse_ms, duration_ns / 1.0e6);
          if (duration_ns >= kObviousReverseDurationNs) {
            result.obvious_reverse = true;
          }
        } else {
          reverse_start_ns[joint] = 0;
        }
      }
    }

    const bool stable = maximum_error <= kPositionToleranceRad &&
                        maximum_velocity <= kVelocityToleranceRadS;
    if (position_window_start_ns == 0 &&
        maximum_error <= kPositionToleranceRad) {
      position_window_start_ns = state.timestamp_ns;
      position_window_minimum = state.positions;
      position_window_maximum = state.positions;
    }
    if (position_window_start_ns != 0 && arrival_confirmed_ns == 0) {
      result.position_window_peak_velocity_rad_s = std::max(
          result.position_window_peak_velocity_rad_s, maximum_velocity);
      for (std::size_t joint = 0; joint < delta.size(); ++joint) {
        position_window_minimum[joint] = std::min(
            position_window_minimum[joint], state.positions[joint]);
        position_window_maximum[joint] = std::max(
            position_window_maximum[joint], state.positions[joint]);
        result.position_window_max_position_span_rad = std::max(
            result.position_window_max_position_span_rad,
            position_window_maximum[joint] - position_window_minimum[joint]);
      }
    }
    if (stable) {
      if (stable_start_ns == 0) stable_start_ns = state.timestamp_ns;
    } else {
      stable_start_ns = 0;
    }
    write_trace_sample(trace, trial, from.tick, to.tick,
                       tail_start_ns != 0, to.positions, state);
    if (arrival_confirmed_ns == 0 && stable_start_ns != 0 &&
        state.timestamp_ns - stable_start_ns >= kStableWindowNs) {
      if (tail_start_ns == 0) {
        throw std::runtime_error("stable arrival occurred before tail entry");
      }
      result.elapsed_ms = (state.timestamp_ns - command_ns) / 1.0e6;
      result.tail_ms = (state.timestamp_ns - tail_start_ns) / 1.0e6;
      result.position_window_to_stable_ms = position_window_start_ns == 0
          ? 0.0
          : (state.timestamp_ns - position_window_start_ns) / 1.0e6;
      result.position_window_to_quiet_ms = position_window_start_ns == 0
          ? 0.0
          : (stable_start_ns - position_window_start_ns) / 1.0e6;
      result.final_max_error_rad = maximum_error;
      arrival_confirmed_ns = state.timestamp_ns;
      post_hold_minimum = state.positions;
      post_hold_maximum = state.positions;
    }
    if (arrival_confirmed_ns != 0) {
      ++result.post_hold_samples;
      result.post_hold_peak_velocity_rad_s = std::max(
          result.post_hold_peak_velocity_rad_s, maximum_velocity);
      for (std::size_t joint = 0; joint < delta.size(); ++joint) {
        post_hold_minimum[joint] = std::min(
            post_hold_minimum[joint], state.positions[joint]);
        post_hold_maximum[joint] = std::max(
            post_hold_maximum[joint], state.positions[joint]);
        result.post_hold_max_position_span_rad = std::max(
            result.post_hold_max_position_span_rad,
            post_hold_maximum[joint] - post_hold_minimum[joint]);
      }
      if (maximum_velocity > kVelocityToleranceRadS) {
        if (post_hold_motion_start_ns == 0) {
          post_hold_motion_start_ns = state.timestamp_ns;
        }
        result.post_hold_longest_motion_ms = std::max(
            result.post_hold_longest_motion_ms,
            (state.timestamp_ns - post_hold_motion_start_ns) / 1.0e6);
      } else {
        post_hold_motion_start_ns = 0;
      }
      if (state.timestamp_ns - arrival_confirmed_ns >=
          kPostArrivalObservationNs) {
        trace.flush();
        return result;
      }
    }
  }
  std::size_t worst_error_joint = 0;
  std::size_t worst_velocity_joint = 0;
  float worst_error = 0.0f;
  float worst_velocity = 0.0f;
  for (std::size_t joint = 0; joint < delta.size(); ++joint) {
    const float error =
        std::abs(to.positions[joint] - last_state.positions[joint]);
    const float velocity = std::abs(last_state.velocities[joint]);
    if (error > worst_error) {
      worst_error = error;
      worst_error_joint = joint;
    }
    if (velocity > worst_velocity) {
      worst_velocity = velocity;
      worst_velocity_joint = joint;
    }
  }
  const auto joint_name = [](std::size_t joint) {
    return std::string(joint < ARTICORE_PRODUCT_ARM_DOF ? "left_j"
                                                        : "right_j") +
        std::to_string(joint % ARTICORE_PRODUCT_ARM_DOF + 1);
  };
  throw std::runtime_error(
      "measured PV move did not settle within 8 s; worst_error_joint=" +
      joint_name(worst_error_joint) +
      " worst_error_rad=" + std::to_string(worst_error) +
      " worst_velocity_joint=" + joint_name(worst_velocity_joint) +
      " worst_velocity_rad_s=" + std::to_string(worst_velocity));
}

void wait_stable(ArticoreRuntime* runtime, const Product& target,
                 std::chrono::seconds timeout = std::chrono::seconds(12)) {
  const auto deadline = Clock::now() + timeout;
  uint64_t stable_start_ns = 0;
  uint64_t last_sequence = 0;
  float last_maximum_error = 0.0f;
  float last_maximum_velocity = 0.0f;
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
    for (std::size_t joint = 0; joint < target.size(); ++joint) {
      maximum_error = std::max(
          maximum_error, std::abs(target[joint] - state.positions[joint]));
      maximum_velocity = std::max(
          maximum_velocity, std::abs(state.velocities[joint]));
    }
    last_maximum_error = maximum_error;
    last_maximum_velocity = maximum_velocity;
    if (maximum_velocity > kAbortVelocityRadS) {
      throw std::runtime_error("feedback exceeded 4 rad/s abort threshold");
    }
    if (maximum_error <= 0.01f && maximum_velocity <= 0.05f) {
      if (stable_start_ns == 0) stable_start_ns = state.timestamp_ns;
      if (state.timestamp_ns - stable_start_ns >= 100'000'000ULL) return;
    } else {
      stable_start_ns = 0;
    }
  }
  throw std::runtime_error(
      "approach target did not settle within 12 s; max_error_rad=" +
      std::to_string(last_maximum_error) +
      " max_velocity_rad_s=" + std::to_string(last_maximum_velocity));
}

void print_trial(const TrialMetrics& result) {
  std::cout << "TRIAL index=" << result.trial
            << " from_tick=" << result.from_tick
            << " to_tick=" << result.to_tick
            << " elapsed_ms=" << result.elapsed_ms
            << " Ttail_ms=" << result.tail_ms
            << " peak_velocity_rad_s=" << result.peak_velocity_rad_s
            << " peak_reverse_velocity_rad_s="
            << result.peak_reverse_velocity_rad_s
            << " longest_reverse_ms=" << result.longest_reverse_ms
            << " position_window_to_stable_ms="
            << result.position_window_to_stable_ms
            << " position_window_to_quiet_ms="
            << result.position_window_to_quiet_ms
            << " position_window_peak_velocity_rad_s="
            << result.position_window_peak_velocity_rad_s
            << " position_window_max_position_span_rad="
            << result.position_window_max_position_span_rad
            << " post_hold_peak_velocity_rad_s="
            << result.post_hold_peak_velocity_rad_s
            << " post_hold_max_position_span_rad="
            << result.post_hold_max_position_span_rad
            << " post_hold_longest_motion_ms="
            << result.post_hold_longest_motion_ms
            << " obvious_reverse=" << (result.obvious_reverse ? "true" : "false")
            << " final_max_error_rad=" << result.final_max_error_rad
            << " samples=" << result.samples << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  const bool valid_argument_count = argc == 3 || argc == 4;
  const bool inspect_only = valid_argument_count &&
      (std::string(argv[1]) == "--inspect-random-pairs" ||
       std::string(argv[1]) == "--inspect-other-joints-pairs");
  const bool move_hardware = valid_argument_count &&
      (std::string(argv[1]) == "--i-understand-both-arms-will-move" ||
       std::string(argv[1]) ==
           "--i-understand-both-arms-will-move-other-joints");
  const bool other_joints_only = valid_argument_count &&
      (std::string(argv[1]) == "--inspect-other-joints-pairs" ||
       std::string(argv[1]) ==
           "--i-understand-both-arms-will-move-other-joints");
  if (!inspect_only && !move_hardware) {
    std::cerr << "Refusing to move hardware. Pass "
                 "--inspect-random-pairs <control.jsonl> or "
                 "--inspect-other-joints-pairs <control.jsonl> or "
                 "--i-understand-both-arms-will-move <control.jsonl> "
                 "or --i-understand-both-arms-will-move-other-joints "
                 "<control.jsonl> "
                 "[random-seed]\n";
    return 2;
  }

  ArticoreRuntime* runtime = nullptr;
  bool connected = false;
  bool enabled = false;
  Product initial{};
  try {
    uint32_t random_seed = kDefaultRandomSeed;
    if (argc == 4) {
      char* end = nullptr;
      const unsigned long parsed = std::strtoul(argv[3], &end, 10);
      if (end == argv[3] || *end != '\0' || parsed > 0xffffffffUL) {
        throw std::runtime_error("random seed must be a uint32 integer");
      }
      random_seed = static_cast<uint32_t>(parsed);
    }
    const auto frames = load_frames(argv[2]);
    const auto pairs = select_random_pairs(
        frames, random_seed, other_joints_only);
    check(articore_runtime_create_yunyi(ARTICORE_MODE_PV, 0, &runtime),
          "create_yunyi");
    ArticoreProductJointAngleVelLimits limits{};
    limits.struct_size = sizeof(limits);
    check(articore_runtime_get_joint_angle_vel_limits(runtime, &limits),
          "get_joint_angle_vel_limits");
    validate_frames(frames, limits);
    print_plan(frames, pairs, random_seed, other_joints_only);
    if (inspect_only) {
      articore_runtime_free(runtime);
      runtime = nullptr;
      return 0;
    }

    check(articore_runtime_connect(runtime), "connect");
    connected = true;
    require_healthy(runtime);
    initial = read_snapshot(runtime).positions;
    check(articore_runtime_enable(runtime), "enable");
    enabled = true;

    const char* trace_path = other_joints_only
        ? kOtherJointTracePath : kTracePath;
    std::ofstream trace(trace_path);
    if (!trace) throw std::runtime_error("cannot open PV tail trace output");
    write_trace_header(trace);
    std::vector<TrialMetrics> results;
    int trial = 0;
    for (std::size_t pair_index = 0; pair_index < pairs.size(); ++pair_index) {
      const auto& from = frames[pairs[pair_index].from];
      const auto& to = frames[pairs[pair_index].to];
      std::cout << "APPROACH pair=" << pair_index + 1
                << " tick=" << from.tick << " at "
                << kApproachSpeedPercent << " percent\n";
      check(articore_runtime_set_joint_pv(
                runtime, from.positions.data(),
                static_cast<uint32_t>(from.positions.size()),
                kApproachSpeedPercent),
            "approach random pair start");
      wait_stable(runtime, from.positions);

      results.push_back(measure_move(
          runtime, ++trial, from, to, trace));
      print_trial(results.back());
    }

    double maximum_tail_ms = 0.0;
    double maximum_visual_quiet_ms = 0.0;
    float maximum_visual_window_velocity = 0.0f;
    float maximum_visual_window_position_span = 0.0f;
    float maximum_post_hold_span = 0.0f;
    double maximum_post_hold_motion_ms = 0.0;
    bool any_obvious_reverse = false;
    for (const auto& result : results) {
      maximum_tail_ms = std::max(maximum_tail_ms, result.tail_ms);
      maximum_visual_quiet_ms = std::max(
          maximum_visual_quiet_ms, result.position_window_to_quiet_ms);
      maximum_visual_window_velocity = std::max(
          maximum_visual_window_velocity,
          result.position_window_peak_velocity_rad_s);
      maximum_visual_window_position_span = std::max(
          maximum_visual_window_position_span,
          result.position_window_max_position_span_rad);
      maximum_post_hold_span = std::max(
          maximum_post_hold_span, result.post_hold_max_position_span_rad);
      maximum_post_hold_motion_ms = std::max(
          maximum_post_hold_motion_ms, result.post_hold_longest_motion_ms);
      any_obvious_reverse = any_obvious_reverse || result.obvious_reverse;
    }
    const bool tail_gate_pass = maximum_tail_ms < kTailGateMs;
    const bool strict_velocity_gate_pass =
        maximum_visual_quiet_ms <= kVisualQuietGateMs &&
        maximum_visual_window_velocity <= kVisualWindowPeakVelocityRadS &&
        !any_obvious_reverse;
    const bool visual_gate_pass =
        maximum_visual_window_position_span <=
            kVisualMaximumPositionSpanRad &&
        maximum_post_hold_span <= kVisualMaximumPositionSpanRad &&
        maximum_post_hold_motion_ms <
            kObviousReverseDurationNs / 1.0e6 &&
        !any_obvious_reverse;
    std::cout << "SUMMARY trials=" << results.size()
              << " maximum_Ttail_ms=" << maximum_tail_ms
              << " maximum_visual_quiet_ms=" << maximum_visual_quiet_ms
              << " maximum_visual_window_velocity_rad_s="
              << maximum_visual_window_velocity
              << " maximum_visual_window_position_span_rad="
              << maximum_visual_window_position_span
              << " maximum_post_hold_position_span_rad="
              << maximum_post_hold_span
              << " maximum_post_hold_motion_ms="
              << maximum_post_hold_motion_ms
              << " any_obvious_reverse="
              << (any_obvious_reverse ? "true" : "false")
              << " Ttail_gate_pass="
              << (tail_gate_pass ? "true" : "false")
              << " strict_velocity_gate_pass="
              << (strict_velocity_gate_pass ? "true" : "false")
              << " visual_gate_pass="
              << (visual_gate_pass ? "true" : "false") << '\n';

    std::cout << "RESTORE initial at " << kApproachSpeedPercent << " percent\n";
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
    std::cout << "RESULT trace=" << trace_path << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "RUN_ERROR " << error.what() << '\n';
    if (runtime && enabled) {
      articore_runtime_disable(runtime);
      enabled = false;
    }
    if (runtime && connected) articore_runtime_disconnect(runtime);
    if (runtime) articore_runtime_free(runtime);
    return 1;
  }
}

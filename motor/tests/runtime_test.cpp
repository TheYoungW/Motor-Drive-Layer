#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <vector>

#include "damiao/protocol.hpp"
#include "damiao/runtime.hpp"

namespace {

void set_test_env(const char* name, const char* value) {
  ::setenv(name, value, 1);
}

void unset_test_env(const char* name) {
  ::unsetenv(name);
}

static_assert(!std::is_copy_constructible_v<damiao::MotorHandle>,
              "MotorHandle owns synchronized runtime state and must not be copied");
static_assert(!std::is_move_constructible_v<damiao::MotorHandle>,
              "MotorHandle addresses must remain stable while registered");

class FakeBus final : public damiao::CanBus {
 public:
  damiao::TransportCapabilities capabilities() const override {
    return {"fake-test", 8, 3, true, false, true, true, false};
  }

  void send(const damiao::CanFrame& frame) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (send_failures_ > 0) {
      --send_failures_;
      throw std::runtime_error("injected send failure");
    }
    sent.push_back(frame);
    sent_at.push_back(std::chrono::steady_clock::now());
    if (auto_ack_writes_ && frame.id == 0x7FF && frame.data[2] == 0x55) {
      if (drop_register_write_acks_ > 0) {
        --drop_register_write_acks_;
      } else {
        incoming.push_back(damiao::CanFrame{0x11, frame.data});
      }
    }
    if (auto_register_io_ && frame.id == 0x7FF && frame.data[2] == 0x33) {
      if (drop_register_reads_ > 0) {
        --drop_register_reads_;
      } else {
        auto reply = frame.data;
        const auto found = registers_.find(frame.data[3]);
        if (found != registers_.end()) {
          for (std::size_t i = 0; i < 4; ++i) reply[4 + i] = found->second[i];
        }
        incoming.push_back(damiao::CanFrame{0x11, reply});
      }
    }
    if (auto_register_io_ && frame.id == 0x7FF && frame.data[2] == 0x55) {
      std::array<uint8_t, 4> value{frame.data[4], frame.data[5], frame.data[6], frame.data[7]};
      registers_[frame.data[3]] = value;
      if (drop_register_write_acks_ > 0) {
        --drop_register_write_acks_;
      } else {
        incoming.push_back(damiao::CanFrame{0x11, frame.data});
      }
    }
  }

  std::optional<damiao::CanFrame> receive_for(std::chrono::milliseconds timeout) override {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    for (;;) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (always_fail_receive_) {
          throw std::runtime_error("injected persistent receive failure");
        }
        if (receive_failures_ > 0) {
          --receive_failures_;
          throw std::runtime_error("injected transient receive failure");
        }
        if (!incoming.empty()) {
          auto frame = incoming.front();
          incoming.erase(incoming.begin());
          return frame;
        }
      }
      if (timeout.count() == 0 || std::chrono::steady_clock::now() >= deadline) {
        return std::nullopt;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
  }

  void push_rx(const damiao::CanFrame& frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    incoming.push_back(frame);
  }

  void fail_next_receives(int count) {
    std::lock_guard<std::mutex> lock(mutex_);
    receive_failures_ = count;
  }

  void fail_next_sends(int count) {
    std::lock_guard<std::mutex> lock(mutex_);
    send_failures_ = count;
  }

  void set_always_fail_receive(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    always_fail_receive_ = enabled;
  }

  std::vector<damiao::CanFrame> sent_snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return sent;
  }

  std::vector<std::chrono::steady_clock::time_point> sent_times_snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return sent_at;
  }

  void set_auto_ack_writes(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto_ack_writes_ = enabled;
  }

  void set_auto_register_io(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto_register_io_ = enabled;
  }

  void drop_next_register_reads(int count) {
    std::lock_guard<std::mutex> lock(mutex_);
    drop_register_reads_ = count;
  }

  void drop_next_register_write_acks(int count) {
    std::lock_guard<std::mutex> lock(mutex_);
    drop_register_write_acks_ = count;
  }

  void set_register_u32(uint8_t rid, uint32_t value) {
    std::lock_guard<std::mutex> lock(mutex_);
    registers_[rid] = {static_cast<uint8_t>(value & 0xFF),
                       static_cast<uint8_t>((value >> 8) & 0xFF),
                       static_cast<uint8_t>((value >> 16) & 0xFF),
                       static_cast<uint8_t>((value >> 24) & 0xFF)};
  }

  void set_register_f32(uint8_t rid, float value) {
    std::array<uint8_t, 4> raw{};
    std::memcpy(raw.data(), &value, sizeof(value));
    std::lock_guard<std::mutex> lock(mutex_);
    registers_[rid] = raw;
  }

  void shutdown() override {
    std::lock_guard<std::mutex> lock(mutex_);
    ++shutdown_count_;
  }

  int shutdown_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return shutdown_count_;
  }

 private:
  mutable std::mutex mutex_;
  std::vector<damiao::CanFrame> incoming;
  std::vector<damiao::CanFrame> sent;
  std::vector<std::chrono::steady_clock::time_point> sent_at;
  bool auto_ack_writes_ = false;
  bool auto_register_io_ = false;
  std::map<uint8_t, std::array<uint8_t, 4>> registers_;
  int shutdown_count_ = 0;
  int receive_failures_ = 0;
  int send_failures_ = 0;
  int drop_register_reads_ = 0;
  int drop_register_write_acks_ = 0;
  bool always_fail_receive_ = false;
};

class ImmediateWriteAckBus final : public damiao::CanBus {
 public:
  void send(const damiao::CanFrame& frame) override {
    sent.push_back(frame);
    if (on_write && frame.id == 0x7FF && frame.data[2] == 0x55) {
      on_write(damiao::CanFrame{0x11, frame.data});
    }
  }

  std::optional<damiao::CanFrame> receive_for(
      std::chrono::milliseconds) override {
    return std::nullopt;
  }

  std::function<void(const damiao::CanFrame&)> on_write;
  std::vector<damiao::CanFrame> sent;
};

class DispatchBarrier {
 public:
  void arrive_and_wait() {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto generation = generation_;
    ++arrivals_;
    if (arrivals_ == 2) {
      arrivals_ = 0;
      ++generation_;
      cv_.notify_all();
      return;
    }
    if (!cv_.wait_for(lock, std::chrono::milliseconds(200),
                      [&] { return generation_ != generation; })) {
      throw std::runtime_error("controller workers did not start together");
    }
  }

 private:
  std::mutex mutex_;
  std::condition_variable cv_;
  int arrivals_ = 0;
  uint64_t generation_ = 0;
};

class ParallelBatchBus final : public damiao::CanBus {
 public:
  explicit ParallelBatchBus(std::shared_ptr<DispatchBarrier> barrier)
      : barrier_(std::move(barrier)) {}

  void send(const damiao::CanFrame& frame) override {
    std::size_t index = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      index = sent_.size();
    }
    if (index % 7 == 0) barrier_->arrive_and_wait();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (fail_id_.has_value() && frame.id == *fail_id_) {
        throw std::runtime_error("injected send failure");
      }
      sent_.push_back(frame);
      sent_at_.push_back(std::chrono::steady_clock::now());
    }
  }

  std::optional<damiao::CanFrame> receive_for(std::chrono::milliseconds) override {
    return std::nullopt;
  }

  void fail_on(uint32_t arbitration_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    fail_id_ = arbitration_id;
  }

  std::vector<damiao::CanFrame> sent_snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return sent_;
  }

  std::vector<std::chrono::steady_clock::time_point> sent_times_snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return sent_at_;
  }

 private:
  std::shared_ptr<DispatchBarrier> barrier_;
  mutable std::mutex mutex_;
  std::vector<damiao::CanFrame> sent_;
  std::vector<std::chrono::steady_clock::time_point> sent_at_;
  std::optional<uint32_t> fail_id_;
};

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void require_close(float actual, float expected, float tolerance, const char* message) {
  if (std::fabs(actual - expected) > tolerance) {
    std::cerr << message << ": actual=" << actual << " expected=" << expected << '\n';
    throw std::runtime_error(message);
  }
}

float read_f32_le(const std::array<uint8_t, 8>& data, std::size_t offset) {
  float value = 0.0f;
  std::memcpy(&value, data.data() + offset, sizeof(value));
  return value;
}

damiao::CanFrame feedback_frame(uint32_t arbitration_id,
                                uint8_t can_id,
                                uint8_t status,
                                float pos,
                                float vel,
                                float torq,
                                damiao::Limits limits) {
  const auto cmd = damiao::encode_mit_command(pos, vel, torq, 0.0f, 0.0f, limits);
  return damiao::CanFrame{
      arbitration_id,
      {static_cast<uint8_t>((status << 4) | (can_id & 0x0F)),
       cmd[0],
       cmd[1],
       cmd[2],
       cmd[3],
       cmd[7],
       31,
       32},
  };
}

std::size_t feedback_request_count(const std::vector<damiao::CanFrame>& frames) {
  std::size_t count = 0;
  for (const auto& frame : frames) {
    if (frame.id == 0x7FF && frame.data[2] == 0xCC) ++count;
  }
  return count;
}

}  // namespace

int main() {
  try {
  auto capability_bus = std::make_shared<FakeBus>();
  damiao::Controller capability_controller(capability_bus);
  const auto transport_capabilities = capability_controller.transport_capabilities();
  require(transport_capabilities.transport == "fake-test" &&
              transport_capabilities.channel_count == 3 &&
              transport_capabilities.can_fd &&
              !transport_capabilities.parallel_batches &&
              transport_capabilities.hardware_rx_timestamps &&
              transport_capabilities.reconnect &&
              !transport_capabilities.process_session_reuse,
          "controller forwards the active bus capabilities");
  capability_controller.close_bus();
  require(!capability_controller.transport_health().connected,
          "closed controller reports disconnected transport health");

  {
    auto integrity_bus = std::make_shared<FakeBus>();
    damiao::Controller integrity_controller(integrity_bus,
                                            "feedback integrity endpoint");
    const auto integrity_motor =
        integrity_controller.add_damiao_motor(0x04, 0x14, "4310");

    auto wrong_identity = feedback_frame(
        0x14, 0x05, 0x01, -1.0f, 0.0f, 0.0f,
        damiao::model_limits("4310"));
    wrong_identity.channel = 1;
    integrity_bus->push_rx(wrong_identity);
    integrity_controller.poll_feedback_once();
    require(!integrity_motor->latest_state().has_value(),
            "matching arbitration ID with a different payload CAN ID is rejected");
    auto integrity = integrity_motor->feedback_integrity_stats();
    require(integrity.identity_mismatch_count == 1 &&
                integrity.last_reason ==
                    damiao::FeedbackRejectionReason::IdentityMismatch &&
                integrity.channel == 1,
            "identity rejection is recorded with physical channel context");

    auto initial = feedback_frame(0x14, 0x04, 0x01, 0.0f, 0.0f, 0.0f,
                                  damiao::model_limits("4310"));
    initial.channel = 1;
    integrity_bus->push_rx(initial);
    integrity_controller.poll_feedback_once();
    require(integrity_motor->latest_state().has_value(),
            "strict identity matching accepts the configured feedback frame");

    auto socketcanfd_identity = feedback_frame(
        0x204, 0x04, 0x01, 0.01f, 0.0f, 0.0f,
        damiao::model_limits("4310"));
    socketcanfd_identity.channel = 1;
    integrity_bus->push_rx(socketcanfd_identity);
    integrity_controller.poll_feedback_once();
    require(integrity_motor->latest_state().has_value() &&
                integrity_motor->latest_state()->arbitration_id == 0x204,
            "strict identity matching accepts DM-USB2FDCAN SocketCAN-FD feedback");

    auto mixed_payload = feedback_frame(
        0x14, 0x04, 0x01, 2.0f, 0.0f, 0.0f,
        damiao::model_limits("4310"));
    mixed_payload.channel = 1;
    integrity_bus->push_rx(mixed_payload);
    integrity_controller.poll_feedback_once();
    const auto retained = integrity_motor->latest_state();
    integrity = integrity_motor->feedback_integrity_stats();
    require(retained.has_value() && std::fabs(retained->pos) < 0.01f &&
                integrity.implausible_position_jump_count == 1 &&
                integrity.last_reason ==
                    damiao::FeedbackRejectionReason::ImplausiblePositionJump,
            "a one-frame cross-channel position jump is dropped without replacing cache");

    integrity_bus->push_rx(feedback_frame(
        0x14, 0x04, 0x01, 0.02f, 0.0f, 0.0f,
        damiao::model_limits("4310")));
    integrity_controller.poll_feedback_once();
    require(std::fabs(integrity_motor->latest_state()->pos - 0.02f) < 0.01f,
            "normal feedback resumes immediately after a rejected outlier");
  }

  auto discovery_bus = std::make_shared<FakeBus>();
  damiao::Controller discovery_controller(discovery_bus, "test discovery endpoint");
  const std::vector<damiao::MotorCandidate> discovery_candidates{
      {"joint", 0x09, 0x19, "4310", damiao::PresencePolicy::Required},
      {"gripper", 0x01, 0x11, "4340P", damiao::PresencePolicy::Optional},
      {"disabled_tool", 0x02, 0x12, "4340P", damiao::PresencePolicy::Disabled},
  };
  std::thread discovery_responder([&] {
    for (int i = 0; i < 200; ++i) {
      if (feedback_request_count(discovery_bus->sent_snapshot()) >= 2) break;
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    discovery_bus->push_rx(feedback_frame(
        0x19, 0x09, 0x01, 0.0f, 0.0f, 0.0f, damiao::model_limits("4310")));
  });
  std::vector<damiao::MotorDiscoveryResult> discovery;
  try {
    discovery = discovery_controller.discover_damiao_motors(
        discovery_candidates, std::chrono::milliseconds(20), 1);
  } catch (...) {
    discovery_responder.join();
    throw;
  }
  discovery_responder.join();
  require(discovery.size() == 3 &&
              discovery[0].state == damiao::PresenceState::Present &&
              discovery[0].motor &&
              discovery[1].state == damiao::PresenceState::NotInstalled &&
              !discovery[1].motor &&
              discovery[2].state == damiao::PresenceState::NotInstalled &&
              !discovery[2].motor,
          "discovery keeps required present and tolerates optional/disabled absence");
  const auto discovery_sent = discovery_bus->sent_snapshot();
  require(std::none_of(discovery_sent.begin(), discovery_sent.end(),
                       [](const damiao::CanFrame& frame) {
                         return frame.id == 0x7FF && frame.data[2] == 0xCC &&
                                frame.data[0] == 0x02;
                       }),
          "disabled candidates are never probed");
  bool frozen_rejected = false;
  try {
    discovery_controller.add_damiao_motor(0x03, 0x13, "4310");
  } catch (const std::logic_error&) {
    frozen_rejected = true;
  }
  require(frozen_rejected,
          "successful discovery freezes the active motor set for the connection");
  discovery_controller.close_bus();

  auto required_bus = std::make_shared<FakeBus>();
  damiao::Controller required_controller(required_bus, "test required endpoint");
  std::string required_error;
  try {
    required_controller.discover_damiao_motors(
        {{"required_joint", 0x0F, 0x1F, "4310",
          damiao::PresencePolicy::Required}},
        std::chrono::milliseconds(5), 0);
  } catch (const std::runtime_error& error) {
    required_error = error.what();
  }
  require(required_error.find("test required endpoint") != std::string::npos &&
              required_error.find("required_joint") != std::string::npos &&
              required_error.find("motor_id=15") != std::string::npos,
          "required discovery failure identifies endpoint, role, and motor ID");
  auto after_failed_discovery =
      required_controller.add_damiao_motor(0x0F, 0x1F, "4310");
  require(after_failed_discovery != nullptr,
          "failed discovery rolls back candidate registration");
  required_controller.close_bus();

  auto health_bus = std::make_shared<FakeBus>();
  damiao::Controller health_controller(health_bus);
  auto health_motor = health_controller.add_damiao_motor(0x01, 0x11, "4340P");
  auto initial_health = health_controller.transport_health();
  require(initial_health.connected && initial_health.healthy &&
              initial_health.tx_frames == 0 && initial_health.rx_frames == 0 &&
              !initial_health.last_tx_age.has_value() &&
              !initial_health.last_rx_age.has_value(),
          "new controller exposes an empty healthy transport snapshot");
  health_motor->request_feedback();
  health_bus->push_rx(feedback_frame(0x11, 0x01, 0x01, 0.0f, 0.0f, 0.0f,
                                     damiao::model_limits("4340P")));
  for (int i = 0; i < 100; ++i) {
    const auto health = health_controller.transport_health();
    if (health.tx_frames >= 1 && health.rx_frames >= 1) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  auto active_health = health_controller.transport_health();
  require(active_health.tx_frames >= 1 && active_health.rx_frames >= 1 &&
              active_health.last_tx_age.has_value() &&
              active_health.last_rx_age.has_value(),
          "transport health counts successful TX and RX frames");
  health_bus->fail_next_receives(1);
  for (int i = 0; i < 100; ++i) {
    if (health_controller.transport_health().receive_errors >= 1) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const auto failed_health = health_controller.transport_health();
  require(!failed_health.healthy && failed_health.receive_errors >= 1 &&
              failed_health.last_error.find("injected transient") != std::string::npos,
          "transport receive errors are retained in structured health");
  health_controller.close_bus();
  auto bus = std::make_shared<FakeBus>();
  damiao::Controller controller(bus);
  auto motor1 = controller.add_damiao_motor(0x01, 0x11, "4340P");
  auto motor2 = controller.add_damiao_motor(0x02, 0x12, "4310");

  bus->push_rx(feedback_frame(0x11, 0x01, 0x01, 1.2f, 0.3f, 0.4f,
                              damiao::model_limits("4340P")));

  std::optional<damiao::MotorState> state;
  for (int i = 0; i < 50; ++i) {
    state = motor1->latest_state();
    if (state.has_value()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  require(state.has_value(), "background polling updates state cache");
  require(state->can_id == 0x01, "state can id");
  require(state->status_code == 0x01, "state status");
  require_close(state->pos, 1.2f, 0.05f, "state position");
  require(state->t_mos == 31.0f, "state cache preserves MOS temperature");
  require(state->t_rotor == 32.0f, "state cache preserves rotor temperature");
  require(!motor2->latest_state().has_value(), "unmatched motor state stays empty");

  const auto first_feedback_stats = motor1->feedback_stats();
  require(first_feedback_stats.has_feedback, "feedback stats report an available sample");
  require(first_feedback_stats.update_count == 1, "first sensor frame increments feedback count");
  require(first_feedback_stats.age >= std::chrono::nanoseconds::zero(),
          "feedback age is non-negative");
  const auto coherent_snapshot = motor1->state_snapshot();
  require(coherent_snapshot.state.has_value() &&
              coherent_snapshot.state->status_code == 0x01 &&
              coherent_snapshot.state->t_mos == 31.0f &&
              coherent_snapshot.state->t_rotor == 32.0f &&
              coherent_snapshot.feedback.has_feedback &&
              coherent_snapshot.feedback.update_count == 1,
          "state and feedback metadata share one coherent cache snapshot");
  const auto empty_feedback_stats = motor2->feedback_stats();
  require(!empty_feedback_stats.has_feedback, "unmatched motor has no feedback timestamp");
  require(empty_feedback_stats.update_count == 0, "unmatched motor feedback count stays zero");

  auto delayed_bus = std::make_shared<FakeBus>();
  damiao::Controller delayed_controller(delayed_bus);
  auto delayed_motor1 = delayed_controller.add_damiao_motor(0x01, 0x11, "4340P");
  auto delayed_motor2 = delayed_controller.add_damiao_motor(0x02, 0x12, "4310");
  std::thread batch_responder([&] {
    for (int i = 0; i < 200; ++i) {
      if (feedback_request_count(delayed_bus->sent_snapshot()) >= 2) break;
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    delayed_bus->push_rx(feedback_frame(0x11, 0x01, 0x01, 0.5f, 0.1f, 0.0f,
                                        damiao::model_limits("4340P")));
    delayed_bus->push_rx(feedback_frame(0x12, 0x02, 0x01, -0.25f, 0.0f, 0.0f,
                                        damiao::model_limits("4310")));
  });
  try {
    delayed_controller.request_feedback_all(std::chrono::milliseconds(250));
  } catch (...) {
    batch_responder.join();
    throw;
  }
  batch_responder.join();
  require(delayed_motor1->feedback_stats().update_count == 1,
          "batch feedback waits for delayed motor 1 response");
  require(delayed_motor2->feedback_stats().update_count == 1,
          "batch feedback waits for delayed motor 2 response");

  std::thread single_responder([&] {
    for (int i = 0; i < 200; ++i) {
      if (feedback_request_count(delayed_bus->sent_snapshot()) >= 3) break;
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    delayed_bus->push_rx(feedback_frame(0x11, 0x01, 0x01, 0.55f, 0.0f, 0.0f,
                                        damiao::model_limits("4340P")));
  });
  std::optional<damiao::MotorState> fresh_state;
  try {
    fresh_state = delayed_motor1->request_fresh_state(std::chrono::milliseconds(250));
  } catch (...) {
    single_responder.join();
    throw;
  }
  single_responder.join();
  require(fresh_state.has_value(), "single-motor fresh-state request waits for delayed response");
  require_close(fresh_state->pos, 0.55f, 0.05f, "fresh-state position");
  delayed_controller.close_bus();

  auto timeout_bus = std::make_shared<FakeBus>();
  damiao::Controller timeout_controller(timeout_bus);
  timeout_controller.add_damiao_motor(0x01, 0x11, "4340P");
  timeout_controller.add_damiao_motor(0x02, 0x12, "4310");
  std::thread partial_responder([&] {
    for (int i = 0; i < 200; ++i) {
      if (feedback_request_count(timeout_bus->sent_snapshot()) >= 2) break;
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    timeout_bus->push_rx(feedback_frame(0x11, 0x01, 0x01, 0.0f, 0.0f, 0.0f,
                                        damiao::model_limits("4340P")));
  });
  const auto timeout_started = std::chrono::steady_clock::now();
  std::string timeout_message;
  try {
    timeout_controller.request_feedback_all(std::chrono::milliseconds(100));
  } catch (const std::runtime_error& error) {
    timeout_message = error.what();
  }
  partial_responder.join();
  const auto batch_timeout_elapsed = std::chrono::steady_clock::now() - timeout_started;
  require(timeout_message == "fresh feedback timed out; missing motor IDs: 2",
          "batch timeout reports only the missing motor ID");
  require(batch_timeout_elapsed < std::chrono::milliseconds(300),
          "batch feedback uses one shared timeout instead of one timeout per motor");
  timeout_controller.close_bus();

  auto report_bus = std::make_shared<FakeBus>();
  damiao::Controller report_controller(report_bus);
  report_controller.add_damiao_motor(0x01, 0x11, "4340P");
  report_controller.add_damiao_motor(0x02, 0x12, "4310");
  std::thread report_responder([&] {
    for (int i = 0; i < 200; ++i) {
      if (feedback_request_count(report_bus->sent_snapshot()) >= 2) break;
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    report_bus->push_rx(feedback_frame(0x11, 0x01, 0x01, 0.0f, 0.0f, 0.0f,
                                       damiao::model_limits("4340P")));
  });
  damiao::FeedbackBatchReport incomplete_report;
  try {
    incomplete_report = report_controller.request_feedback_all_report(
        std::chrono::milliseconds(500));
  } catch (...) {
    report_responder.join();
    throw;
  }
  report_responder.join();
  require(incomplete_report.status == damiao::FeedbackBatchStatus::Incomplete &&
              incomplete_report.timeout == std::chrono::milliseconds(500) &&
              incomplete_report.expected_count == 2 &&
              incomplete_report.received_count == 1 &&
              incomplete_report.missing_motor_ids == std::vector<uint16_t>{0x02},
          "structured feedback report distinguishes a partial response");
  report_controller.close_bus();

  auto no_feedback_bus = std::make_shared<FakeBus>();
  damiao::Controller no_feedback_controller(no_feedback_bus);
  no_feedback_controller.add_damiao_motor(0x01, 0x11, "4340P");
  no_feedback_controller.add_damiao_motor(0x02, 0x12, "4310");
  const auto no_feedback_report =
      no_feedback_controller.request_feedback_all_report(std::chrono::milliseconds(10));
  require(no_feedback_report.status == damiao::FeedbackBatchStatus::Timeout &&
              no_feedback_report.expected_count == 2 &&
              no_feedback_report.received_count == 0 &&
              no_feedback_report.missing_motor_ids ==
                  std::vector<uint16_t>({0x01, 0x02}),
          "structured feedback report distinguishes a complete timeout");
  no_feedback_controller.close_bus();

  auto transport_failure_bus = std::make_shared<FakeBus>();
  damiao::Controller transport_failure_controller(transport_failure_bus);
  transport_failure_controller.add_damiao_motor(0x01, 0x11, "4340P");
  transport_failure_bus->fail_next_sends(1);
  const auto transport_failure_report =
      transport_failure_controller.request_feedback_all_report(
          std::chrono::milliseconds(10));
  require(transport_failure_report.status ==
                  damiao::FeedbackBatchStatus::TransportError &&
              transport_failure_report.expected_count == 1 &&
              transport_failure_report.received_count == 0 &&
              transport_failure_report.missing_motor_ids ==
                  std::vector<uint16_t>{0x01} &&
              transport_failure_report.error.find("injected send failure") !=
                  std::string::npos,
          "structured feedback report distinguishes transport failure");
  transport_failure_controller.close_bus();

  auto retry_bus = std::make_shared<FakeBus>();
  damiao::Controller retry_controller(retry_bus);
  const std::vector<uint16_t> retry_motor_ids{0x01, 0x09, 0x0A, 0x0B,
                                               0x0C, 0x0D, 0x0E, 0x0F};
  std::vector<std::shared_ptr<damiao::MotorHandle>> retry_motors;
  for (const auto motor_id : retry_motor_ids) {
    retry_motors.push_back(
        retry_controller.add_damiao_motor(motor_id, 0x10 + motor_id, "4310"));
  }
  retry_controller.set_tx_gap(std::chrono::microseconds(0));
  std::thread retry_responder([&] {
    for (int i = 0; i < 200; ++i) {
      if (feedback_request_count(retry_bus->sent_snapshot()) >= retry_motor_ids.size()) break;
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    for (std::size_t i = 0; i + 1 < retry_motor_ids.size(); ++i) {
      const auto motor_id = static_cast<uint8_t>(retry_motor_ids[i]);
      retry_bus->push_rx(feedback_frame(0x10 + motor_id, motor_id, 0x01, 0.0f, 0.0f, 0.0f,
                                        damiao::model_limits("4310")));
    }
    // Emulate the bridge dropping only the eighth response, then replying to
    // the controller's targeted retry.
    for (int i = 0; i < 200; ++i) {
      const auto sent = retry_bus->sent_snapshot();
      bool retried_final_motor = false;
      for (std::size_t j = retry_motor_ids.size(); j < sent.size(); ++j) {
        if (sent[j].id == 0x7FF && sent[j].data[2] == 0xCC && sent[j].data[0] == 0x0F) {
          retried_final_motor = true;
          break;
        }
      }
      if (retried_final_motor) break;
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    retry_bus->push_rx(feedback_frame(0x1F, 0x0F, 0x01, 0.0f, 0.0f, 0.0f,
                                      damiao::model_limits("4310")));
  });
  try {
    retry_controller.request_feedback_all(std::chrono::milliseconds(50));
  } catch (...) {
    retry_responder.join();
    throw;
  }
  retry_responder.join();
  const auto retry_sent = retry_bus->sent_snapshot();
  bool retried_final_motor = false;
  for (std::size_t i = retry_motor_ids.size(); i < retry_sent.size(); ++i) {
    if (retry_sent[i].id == 0x7FF && retry_sent[i].data[2] == 0xCC &&
        retry_sent[i].data[0] == 0x0F) {
      retried_final_motor = true;
      break;
    }
  }
  require(retried_final_motor, "eight-motor batch retries the missing final motor");
  for (const auto& motor : retry_motors) {
    require(motor->feedback_stats().update_count == 1,
            "eight-motor retry batch updates every feedback count");
  }
  retry_controller.close_bus();

  bus->push_rx(feedback_frame(0x11, 0x01, 0x01, 1.3f, 0.2f, 0.1f,
                              damiao::model_limits("4340P")));
  for (int i = 0; i < 50 && motor1->feedback_stats().update_count < 2; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  require(motor1->feedback_stats().update_count == 2,
          "each sensor frame increments feedback count exactly once");

  auto extended_feedback = feedback_frame(0x11, 0x01, 0x01, 0.0f, 0.0f, 0.0f,
                                          damiao::model_limits("4340P"));
  extended_feedback.is_extended = true;
  require(!motor1->accepts_frame(extended_feedback), "extended feedback frames are rejected");
  damiao::CanFrame empty_payload_match{0x55, {0x01, 0, 0, 0, 0, 0, 0, 0}};
  empty_payload_match.dlc = 0;
  require(!motor1->accepts_frame(empty_payload_match),
          "zero-DLC frames cannot match by payload motor id");

  controller.set_tx_gap(std::chrono::microseconds(1000));
  controller.enable_all();
  const auto sent = bus->sent_snapshot();
  const auto sent_at = bus->sent_times_snapshot();
  require(sent.size() >= 2, "enable_all sends both motors");
  require(sent[sent.size() - 2].id == 0x01, "enable_all sorted first motor");
  require(sent[sent.size() - 1].id == 0x02, "enable_all sorted second motor");
  const auto gap = sent_at.back() - sent_at[sent_at.size() - 2];
  require(gap >= std::chrono::microseconds(900), "tx pacing gap is applied");

  bus->set_auto_ack_writes(true);

  bool read_only_rejected = false;
  try {
    motor1->write_register_f32(11, 1.0f);
  } catch (const std::invalid_argument&) {
    read_only_rejected = true;
  }
  require(read_only_rejected, "writes to read-only registers are rejected");

  bool wrong_type_rejected = false;
  try {
    motor1->write_register_f32(9, 1.0f);
  } catch (const std::invalid_argument&) {
    wrong_type_rejected = true;
  }
  require(wrong_type_rejected, "register writes reject the wrong data type");

  bool unknown_register_rejected = false;
  try {
    motor1->write_register_u32(37, 1);
  } catch (const std::invalid_argument&) {
    unknown_register_rejected = true;
  }
  require(unknown_register_rejected, "unknown register writes are rejected");

  motor1->set_can_timeout_ms(25);
  const auto timeout_frames = bus->sent_snapshot();
  const auto& timeout_write = timeout_frames.back();
  require(timeout_write.id == 0x7FF, "CAN timeout uses management arbitration id");
  require(timeout_write.data[2] == 0x55 && timeout_write.data[3] == 9,
          "CAN timeout writes register 9");
  require(timeout_write.data[4] == 0xF4 && timeout_write.data[5] == 0x01 &&
              timeout_write.data[6] == 0 && timeout_write.data[7] == 0,
          "CAN timeout converts milliseconds to 50us ticks");

  motor1->enable();
  const auto before_rejected_zero = bus->sent_snapshot().size();
  bool zero_rejected = false;
  try {
    motor1->set_zero_position();
  } catch (const std::invalid_argument&) {
    zero_rejected = true;
  }
  require(zero_rejected, "set-zero rejects an enabled motor");
  require(bus->sent_snapshot().size() == before_rejected_zero,
          "rejected set-zero does not transmit");
  motor1->disable();
  motor1->set_zero_position();
  require(bus->sent_snapshot().back().data == damiao::encode_set_zero_command(),
          "set-zero transmits after disable");

  motor1->enable();
  const auto before_store = bus->sent_snapshot().size();
  motor1->store_parameters();
  const auto store_frames = bus->sent_snapshot();
  std::size_t disable_index = store_frames.size();
  std::size_t store_index = store_frames.size();
  for (std::size_t i = before_store; i < store_frames.size(); ++i) {
    if (store_frames[i].id == motor1->motor_id() &&
        store_frames[i].data == damiao::encode_disable_command()) {
      disable_index = i;
    }
    if (store_frames[i].id == 0x7FF &&
        store_frames[i].data == damiao::encode_store_parameters_command(motor1->motor_id())) {
      store_index = i;
    }
  }
  require(disable_index < store_index, "store parameters disables an active motor first");

  controller.shutdown();
  const auto shutdown_frames = bus->sent_snapshot();
  require(shutdown_frames.size() >= 2, "shutdown sends disable frames");
  require(shutdown_frames[shutdown_frames.size() - 2].data == damiao::encode_disable_command() &&
              shutdown_frames[shutdown_frames.size() - 1].data ==
                  damiao::encode_disable_command(),
          "shutdown disables every motor before closing the bus");
  require(bus->shutdown_count() == 1, "shutdown closes the bus once");

  auto close_bus = std::make_shared<FakeBus>();
  {
    damiao::Controller close_controller(close_bus);
    close_controller.add_damiao_motor(0x03, 0x13, "4310");
    close_controller.close_bus();
    require(close_bus->sent_snapshot().empty(), "close_bus does not disable motors");
    require(close_bus->shutdown_count() == 1, "close_bus closes the bus once");
  }
  require(close_bus->shutdown_count() == 1, "controller destructor keeps close_bus idempotent");

  auto mode_bus = std::make_shared<FakeBus>();
  mode_bus->set_register_u32(10, 1);
  mode_bus->set_register_f32(80, 1.25f);
  mode_bus->set_auto_register_io(true);
  damiao::Controller mode_controller(mode_bus);
  auto mode_motor = mode_controller.add_damiao_motor(0x01, 0x11, "4340P");
  mode_motor->ensure_mode(2, std::chrono::milliseconds(300));
  const auto mode_frames = mode_bus->sent_snapshot();
  bool read_position = false;
  bool verified_mode = false;
  std::size_t mode_read_count = 0;
  std::optional<damiao::CanFrame> final_pos_command;
  for (const auto& frame : mode_frames) {
    if (frame.id == 0x7FF && frame.data[2] == 0x33 && frame.data[3] == 80) {
      read_position = true;
    }
    if (frame.id == 0x7FF && frame.data[2] == 0x33 && frame.data[3] == 10) {
      ++mode_read_count;
      verified_mode = mode_read_count >= 2;
    }
    if (frame.id == 0x101) final_pos_command = frame;
  }
  require(read_position, "mode switch reads the current position");
  require(verified_mode, "mode switch verifies register 10 after writing");
  require(final_pos_command.has_value(), "mode switch sends a final position hold command");
  require_close(read_f32_le(final_pos_command->data, 0), 1.25f, 0.0001f,
                "mode switch holds the current position");
  require_close(read_f32_le(final_pos_command->data, 4), 0.0f, 0.0001f,
                "mode switch holds with zero velocity limit");
  mode_controller.close_bus();

  auto concurrent_bus = std::make_shared<FakeBus>();
  damiao::Controller concurrent_controller(concurrent_bus);
  auto concurrent_motor1 =
      concurrent_controller.add_damiao_motor(0x01, 0x11, "4340P");
  auto concurrent_motor2 = concurrent_controller.add_damiao_motor(0x02, 0x12, "4310");
  concurrent_controller.set_tx_gap(std::chrono::microseconds(10000));
  std::atomic<bool> start_sends{false};
  std::vector<std::thread> senders;
  for (int i = 0; i < 4; ++i) {
    senders.emplace_back([&, i] {
      while (!start_sends.load(std::memory_order_acquire)) std::this_thread::yield();
      (i % 2 == 0 ? concurrent_motor1 : concurrent_motor2)->send_vel(static_cast<float>(i));
    });
  }
  start_sends.store(true, std::memory_order_release);
  for (auto& sender : senders) sender.join();
  const auto concurrent_times = concurrent_bus->sent_times_snapshot();
  require(concurrent_times.size() == 4, "concurrent test sends four frames");
  for (std::size_t i = 1; i < concurrent_times.size(); ++i) {
    require(concurrent_times[i] - concurrent_times[i - 1] >= std::chrono::microseconds(8000),
            "concurrent sends preserve the configured TX gap");
  }
  concurrent_controller.close_bus();

  auto default_gap_bus = std::make_shared<FakeBus>();
  damiao::Controller default_gap_controller(default_gap_bus);
  auto default_gap_motor1 =
      default_gap_controller.add_damiao_motor(0x01, 0x11, "4340P");
  auto default_gap_motor2 =
      default_gap_controller.add_damiao_motor(0x02, 0x12, "4310");
  default_gap_motor1->send_vel(1.0f);
  default_gap_motor2->send_vel(2.0f);
  const auto default_gap_times = default_gap_bus->sent_times_snapshot();
  require(default_gap_times.size() == 2,
          "default multi-motor TX-gap test sends two frames");
  require(default_gap_times[1] - default_gap_times[0] >=
              std::chrono::microseconds(110),
          "multi-motor controller defaults to a 120 us TX gap");
  default_gap_controller.close_bus();

  auto resilient_bus = std::make_shared<FakeBus>();
  resilient_bus->fail_next_receives(1);
  damiao::Controller resilient_controller(resilient_bus);
  auto resilient_motor = resilient_controller.add_damiao_motor(0x01, 0x11, "4340P");
  resilient_bus->push_rx(feedback_frame(0x11, 0x01, 0x01, 0.5f, 0.0f, 0.0f,
                                        damiao::model_limits("4340P")));
  std::optional<damiao::MotorState> resilient_state;
  for (int i = 0; i < 100; ++i) {
    resilient_state = resilient_motor->latest_state();
    if (resilient_state.has_value()) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  require(resilient_state.has_value(),
          "background polling survives a transient receive exception");
  resilient_controller.close_bus();

  set_test_env("MOTOR_DRIVE_LAYER_TX_GAP_US", "5000");
  auto env_gap_bus = std::make_shared<FakeBus>();
  damiao::Controller env_gap_controller(env_gap_bus);
  auto env_gap_motor = env_gap_controller.add_damiao_motor(0x01, 0x11, "4340P");
  env_gap_motor->send_vel(1.0f);
  env_gap_motor->send_vel(2.0f);
  const auto env_gap_times = env_gap_bus->sent_times_snapshot();
  require(env_gap_times.size() == 2, "environment TX-gap test sends two frames");
  require(env_gap_times[1] - env_gap_times[0] >= std::chrono::microseconds(4000),
          "MOTOR_DRIVE_LAYER_TX_GAP_US configures single-motor pacing");
  env_gap_controller.close_bus();
  unset_test_env("MOTOR_DRIVE_LAYER_TX_GAP_US");

  set_test_env("MOTOR_DRIVE_LAYER_TX_GAP_US", "0");
  set_test_env("MOTOR_DRIVE_LAYER_BULK_OP_GAP_MS", "7");
  auto bulk_gap_bus = std::make_shared<FakeBus>();
  damiao::Controller bulk_gap_controller(bulk_gap_bus);
  bulk_gap_controller.add_damiao_motor(0x01, 0x11, "4340P");
  bulk_gap_controller.add_damiao_motor(0x02, 0x12, "4310");
  bulk_gap_controller.enable_all();
  const auto bulk_gap_times = bulk_gap_bus->sent_times_snapshot();
  require(bulk_gap_times.size() == 2, "bulk-gap test enables two motors");
  require(bulk_gap_times[1] - bulk_gap_times[0] >= std::chrono::milliseconds(6),
          "MOTOR_DRIVE_LAYER_BULK_OP_GAP_MS configures bulk pacing");
  bulk_gap_controller.close_bus();
  unset_test_env("MOTOR_DRIVE_LAYER_TX_GAP_US");
  unset_test_env("MOTOR_DRIVE_LAYER_BULK_OP_GAP_MS");

  auto immediate_ack_bus = std::make_shared<ImmediateWriteAckBus>();
  damiao::Controller immediate_ack_controller(immediate_ack_bus);
  auto immediate_ack_motor =
      immediate_ack_controller.add_damiao_motor(0x01, 0x11, "4340P");
  immediate_ack_bus->on_write =
      [immediate_ack_motor](const damiao::CanFrame& frame) {
        immediate_ack_motor->process_feedback_frame(frame);
      };
  immediate_ack_motor->write_register_u32(35, 1);
  immediate_ack_motor->write_register_f32(21, 1.25f);
  require(immediate_ack_bus->sent.size() == 2,
          "register writes accept ACKs received before send returns");
  immediate_ack_controller.close_bus();

  auto failing_drain_bus = std::make_shared<FakeBus>();
  damiao::Controller failing_drain_controller(failing_drain_bus);
  failing_drain_controller.add_damiao_motor(0x01, 0x11, "4340P");
  failing_drain_controller.add_damiao_motor(0x02, 0x12, "4310");
  failing_drain_bus->set_always_fail_receive(true);
  failing_drain_controller.shutdown();
  const auto failing_drain_frames = failing_drain_bus->sent_snapshot();
  std::size_t disable_count = 0;
  for (const auto& frame : failing_drain_frames) {
    if (frame.data == damiao::encode_disable_command()) ++disable_count;
  }
  require(disable_count == 2, "RX drain failures do not skip later shutdown disables");

  auto write_timeout_bus = std::make_shared<FakeBus>();
  damiao::Controller write_timeout_controller(write_timeout_bus);
  auto write_timeout_motor =
      write_timeout_controller.add_damiao_motor(0x01, 0x11, "4340P");
  const auto write_started = std::chrono::steady_clock::now();
  bool write_timed_out = false;
  try {
    write_timeout_motor->write_register_u32(35, 1);
  } catch (const std::runtime_error&) {
    write_timed_out = true;
  }
  const auto write_elapsed = std::chrono::steady_clock::now() - write_started;
  require(write_timed_out, "register write without ACK times out");
  require(write_timeout_bus->sent_snapshot().size() > 3,
          "register write retransmits within its fixed ACK deadline");
  require(write_elapsed < std::chrono::seconds(1),
          "register write keeps a bounded ACK retry budget");
  write_timeout_controller.close_bus();

  auto register_retry_bus = std::make_shared<FakeBus>();
  register_retry_bus->set_register_u32(35, 7);
  register_retry_bus->set_auto_register_io(true);
  register_retry_bus->drop_next_register_reads(4);
  damiao::Controller register_retry_controller(register_retry_bus);
  auto register_retry_motor =
      register_retry_controller.add_damiao_motor(0x01, 0x11, "4340P");
  require(register_retry_motor->get_register_u32(
              35, std::chrono::milliseconds(20)) == 7,
          "register reads recover from several dropped replies");
  register_retry_bus->drop_next_register_write_acks(4);
  register_retry_motor->write_register_u32(35, 9);
  require(register_retry_motor->get_register_u32(
              35, std::chrono::milliseconds(100)) == 9,
          "register writes recover from several dropped ACKs");
  register_retry_controller.close_bus();

  auto dispatch_barrier = std::make_shared<DispatchBarrier>();
  auto channel0_bus = std::make_shared<ParallelBatchBus>(dispatch_barrier);
  auto channel1_bus = std::make_shared<ParallelBatchBus>(dispatch_barrier);
  damiao::Controller channel0(channel0_bus, "CH0");
  damiao::Controller channel1(channel1_bus, "CH1");
  std::vector<std::shared_ptr<damiao::MotorHandle>> channel0_motors;
  std::vector<std::shared_ptr<damiao::MotorHandle>> channel1_motors;
  for (uint16_t id = 1; id <= 7; ++id) {
    channel0_motors.push_back(channel0.add_damiao_motor(id, 0x10 + id, "4310"));
  }
  for (uint16_t id = 9; id <= 15; ++id) {
    channel1_motors.push_back(channel1.add_damiao_motor(id, 0x10 + id, "4310"));
  }
  channel0.set_tx_gap(std::chrono::microseconds(3000));
  channel1.set_tx_gap(std::chrono::microseconds(3000));
  damiao::ControllerGroup controller_group({&channel0, &channel1});

  std::vector<damiao::PosVelBatchCommand> pos_vel_commands;
  for (const auto& motor : channel0_motors) {
    pos_vel_commands.push_back({motor, 0.25f, 1.0f});
  }
  for (const auto& motor : channel1_motors) {
    pos_vel_commands.push_back({motor, -0.25f, 1.0f});
  }
  controller_group.send_pos_vel(pos_vel_commands);
  require(channel0_bus->sent_snapshot().size() == 7 &&
              channel1_bus->sent_snapshot().size() == 7,
          "parallel POS_VEL dispatch sends all commands on both controllers");
  const auto ch0_times = channel0_bus->sent_times_snapshot();
  const auto ch1_times = channel1_bus->sent_times_snapshot();
  const auto start_delta = ch0_times.front() > ch1_times.front()
                               ? ch0_times.front() - ch1_times.front()
                               : ch1_times.front() - ch0_times.front();
  require(start_delta < std::chrono::milliseconds(5),
          "controller workers begin from one dispatch generation");

  std::vector<damiao::MitBatchCommand> mit_commands;
  for (const auto& motor : channel0_motors) {
    mit_commands.push_back({motor, 0.1f, 0.0f, 1.0f, 0.1f, 0.0f});
  }
  for (const auto& motor : channel1_motors) {
    mit_commands.push_back({motor, -0.1f, 0.0f, 1.0f, 0.1f, 0.0f});
  }
  controller_group.send_mit(mit_commands);
  require(channel0_bus->sent_snapshot().size() == 14 &&
              channel1_bus->sent_snapshot().size() == 14,
          "persistent workers support a subsequent MIT dispatch");

  channel1_bus->fail_on(0x100 + channel1_motors.back()->motor_id());
  bool detailed_group_error = false;
  try {
    controller_group.send_pos_vel(pos_vel_commands);
  } catch (const std::runtime_error& error) {
    const std::string message = error.what();
    detailed_group_error = message.find("controller index 1") != std::string::npos &&
                           message.find("CH1") != std::string::npos &&
                           message.find("motor ID 15") != std::string::npos &&
                           message.find("injected send failure") != std::string::npos;
  }
  require(detailed_group_error,
          "parallel dispatch errors identify controller, channel label, motor, and reason");
  channel0.close_bus();
  channel1.close_bus();

  std::cout << "damiao runtime tests passed\n";
  return 0;
  } catch (const std::exception& error) {
    std::cerr << "damiao runtime test failed: " << error.what() << '\n';
    return 1;
  }
}

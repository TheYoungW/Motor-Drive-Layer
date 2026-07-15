#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
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
#if defined(_WIN32)
  _putenv_s(name, value);
#else
  ::setenv(name, value, 1);
#endif
}

void unset_test_env(const char* name) {
#if defined(_WIN32)
  _putenv_s(name, "");
#else
  ::unsetenv(name);
#endif
}

static_assert(!std::is_copy_constructible_v<damiao::MotorHandle>,
              "MotorHandle owns synchronized runtime state and must not be copied");
static_assert(!std::is_move_constructible_v<damiao::MotorHandle>,
              "MotorHandle addresses must remain stable while registered");

class FakeBus final : public damiao::CanBus {
 public:
  void send(const damiao::CanFrame& frame) override {
    std::lock_guard<std::mutex> lock(mutex_);
    sent.push_back(frame);
    sent_at.push_back(std::chrono::steady_clock::now());
    if (auto_ack_writes_ && frame.id == 0x7FF && frame.data[2] == 0x55) {
      incoming.push_back(damiao::CanFrame{0x11, frame.data});
    }
    if (auto_register_io_ && frame.id == 0x7FF && frame.data[2] == 0x33) {
      auto reply = frame.data;
      const auto found = registers_.find(frame.data[3]);
      if (found != registers_.end()) {
        for (std::size_t i = 0; i < 4; ++i) reply[4 + i] = found->second[i];
      }
      incoming.push_back(damiao::CanFrame{0x11, reply});
    }
    if (auto_register_io_ && frame.id == 0x7FF && frame.data[2] == 0x55) {
      std::array<uint8_t, 4> value{frame.data[4], frame.data[5], frame.data[6], frame.data[7]};
      registers_[frame.data[3]] = value;
      incoming.push_back(damiao::CanFrame{0x11, frame.data});
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
  bool always_fail_receive_ = false;
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
  require(!motor2->latest_state().has_value(), "unmatched motor state stays empty");

  const auto first_feedback_stats = motor1->feedback_stats();
  require(first_feedback_stats.has_feedback, "feedback stats report an available sample");
  require(first_feedback_stats.update_count == 1, "first sensor frame increments feedback count");
  require(first_feedback_stats.age >= std::chrono::nanoseconds::zero(),
          "feedback age is non-negative");
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
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
    delayed_bus->push_rx(feedback_frame(0x12, 0x02, 0x01, -0.25f, 0.0f, 0.0f,
                                        damiao::model_limits("4310")));
  });
  try {
    delayed_controller.request_feedback_all(std::chrono::milliseconds(50));
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
    delayed_bus->push_rx(feedback_frame(0x11, 0x01, 0x01, 0.75f, 0.0f, 0.0f,
                                        damiao::model_limits("4340P")));
  });
  std::optional<damiao::MotorState> fresh_state;
  try {
    fresh_state = delayed_motor1->request_fresh_state(std::chrono::milliseconds(50));
  } catch (...) {
    single_responder.join();
    throw;
  }
  single_responder.join();
  require(fresh_state.has_value(), "single-motor fresh-state request waits for delayed response");
  require_close(fresh_state->pos, 0.75f, 0.05f, "fresh-state position");
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
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    timeout_bus->push_rx(feedback_frame(0x11, 0x01, 0x01, 0.0f, 0.0f, 0.0f,
                                        damiao::model_limits("4340P")));
  });
  const auto timeout_started = std::chrono::steady_clock::now();
  std::string timeout_message;
  try {
    timeout_controller.request_feedback_all(std::chrono::milliseconds(20));
  } catch (const std::runtime_error& error) {
    timeout_message = error.what();
  }
  partial_responder.join();
  const auto batch_timeout_elapsed = std::chrono::steady_clock::now() - timeout_started;
  require(timeout_message == "fresh feedback timed out; missing motor IDs: 2",
          "batch timeout reports only the missing motor ID");
  require(batch_timeout_elapsed < std::chrono::milliseconds(150),
          "batch feedback uses one shared timeout instead of one timeout per motor");
  timeout_controller.close_bus();

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
  require(write_timeout_bus->sent_snapshot().size() == 3,
          "register write uses exactly three ACK attempts");
  require(write_elapsed < std::chrono::seconds(1),
          "register write uses the Rust ACK retry budget");
  write_timeout_controller.close_bus();
  std::cout << "damiao runtime tests passed\n";
  return 0;
  } catch (const std::exception& error) {
    std::cerr << "damiao runtime test failed: " << error.what() << '\n';
    return 1;
  }
}

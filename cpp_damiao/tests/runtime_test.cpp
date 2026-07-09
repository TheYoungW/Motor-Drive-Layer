#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <vector>

#include "damiao/protocol.hpp"
#include "damiao/runtime.hpp"

namespace {

class FakeBus final : public damiao::CanBus {
 public:
  void send(const damiao::CanFrame& frame) override {
    std::lock_guard<std::mutex> lock(mutex_);
    sent.push_back(frame);
    sent_at.push_back(std::chrono::steady_clock::now());
  }

  std::optional<damiao::CanFrame> receive_for(std::chrono::milliseconds timeout) override {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    for (;;) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
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

  std::vector<damiao::CanFrame> sent_snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return sent;
  }

  std::vector<std::chrono::steady_clock::time_point> sent_times_snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return sent_at;
  }

 private:
  mutable std::mutex mutex_;
  std::vector<damiao::CanFrame> incoming;
  std::vector<damiao::CanFrame> sent;
  std::vector<std::chrono::steady_clock::time_point> sent_at;
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

}  // namespace

int main() {
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

  controller.set_tx_gap(std::chrono::microseconds(1000));
  controller.enable_all();
  const auto sent = bus->sent_snapshot();
  const auto sent_at = bus->sent_times_snapshot();
  require(sent.size() >= 2, "enable_all sends both motors");
  require(sent[sent.size() - 2].id == 0x01, "enable_all sorted first motor");
  require(sent[sent.size() - 1].id == 0x02, "enable_all sorted second motor");
  const auto gap = sent_at.back() - sent_at[sent_at.size() - 2];
  require(gap >= std::chrono::microseconds(900), "tx pacing gap is applied");

  controller.shutdown();
  std::cout << "damiao runtime tests passed\n";
  return 0;
}

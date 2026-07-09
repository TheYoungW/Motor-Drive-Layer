#include <array>
#include <cstdint>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <vector>

#include "damiao/motor.hpp"

namespace {

class FakeBus final : public damiao::CanBus {
 public:
  std::vector<damiao::CanFrame> sent;
  std::vector<damiao::CanFrame> incoming;

  void send(const damiao::CanFrame& frame) override { sent.push_back(frame); }

  std::optional<damiao::CanFrame> receive_for(std::chrono::milliseconds) override {
    if (incoming.empty()) {
      return std::nullopt;
    }
    auto frame = incoming.front();
    incoming.erase(incoming.begin());
    return frame;
  }
};

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

}  // namespace

int main() {
  FakeBus bus;
  damiao::Motor motor(bus, 0x01, 0x11, damiao::model_limits("4340P"));

  motor.enable();
  require(bus.sent.size() == 1, "enable sends one frame");
  require(bus.sent.back().id == 0x01, "enable arbitration id");
  require(bus.sent.back().data[7] == 0xFC, "enable payload");

  motor.send_position_velocity(1.25f, 2.5f);
  require(bus.sent.size() == 2, "pos-vel sends one more frame");
  require(bus.sent.back().id == 0x101, "pos-vel arbitration id");

  motor.request_feedback();
  require(bus.sent.back().data == damiao::encode_feedback_request_command(0x01),
          "feedback request payload");

  const auto feedback_payload =
      damiao::encode_mit_command(1.0f, 0.5f, 0.1f, 20.0f, 0.4f, damiao::model_limits("4340P"));
  bus.incoming.push_back(
      damiao::CanFrame{0x11, {0x11, feedback_payload[0], feedback_payload[1], feedback_payload[2],
                              feedback_payload[3], feedback_payload[7], 33, 44}});
  const auto feedback = motor.receive_feedback(std::chrono::milliseconds(1));
  require(feedback.has_value(), "feedback received");
  require(feedback->can_id == 0x01, "feedback motor id");
  require(feedback->status_code == 0x01, "feedback status");

  std::cout << "damiao motor tests passed\n";
  return 0;
}

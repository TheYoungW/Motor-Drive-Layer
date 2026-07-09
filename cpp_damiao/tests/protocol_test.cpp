#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>

#include "damiao/protocol.hpp"

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void require_close(float actual, float expected, float tolerance, const char* message) {
  if (std::fabs(actual - expected) > tolerance) {
    std::cerr << message << ": actual=" << actual << " expected=" << expected
              << " tolerance=" << tolerance << '\n';
    throw std::runtime_error(message);
  }
}

uint16_t read_le_u16(const std::array<uint8_t, 8>& frame, std::size_t offset) {
  return static_cast<uint16_t>(frame[offset]) |
         static_cast<uint16_t>(frame[offset + 1] << 8);
}

}  // namespace

int main() {
  const damiao::Limits limits{-12.5f, 12.5f, -30.0f, 30.0f, -10.0f, 10.0f};

  const auto mit = damiao::encode_mit_command(1.5f, -2.0f, 0.8f, 50.0f, 0.5f, limits);
  const auto feedback = damiao::decode_sensor_feedback(
      std::array<uint8_t, 8>{0x11, mit[0], mit[1], mit[2], mit[3], mit[7], 55, 44},
      limits);
  require(feedback.can_id == 0x01, "feedback can_id");
  require(feedback.status_code == 0x01, "feedback status");
  require_close(feedback.pos, 1.5f, 0.05f, "feedback position");
  require_close(feedback.vel, -2.0f, 0.1f, "feedback velocity");
  require(feedback.t_mos == 55.0f, "feedback mos temp");
  require(feedback.t_rotor == 44.0f, "feedback rotor temp");

  require(damiao::encode_enable_command()[7] == 0xFC, "enable frame");
  require(damiao::encode_disable_command()[7] == 0xFD, "disable frame");
  require(damiao::encode_set_zero_command()[7] == 0xFE, "zero frame");
  require(damiao::encode_clear_error_command()[7] == 0xFB, "clear-error frame");

  const auto force_pos = damiao::encode_force_position_command(1.0f, 999.0f, 9.9f);
  require(read_le_u16(force_pos, 4) == 10000, "force-position velocity clamp");
  require(read_le_u16(force_pos, 6) == 10000, "force-position torque clamp");

  const auto read = damiao::encode_register_read_command(0x123, 10);
  require(read == (std::array<uint8_t, 8>{0x23, 0x01, 0x33, 10, 0, 0, 0, 0}),
          "register read frame");

  const auto write =
      damiao::encode_register_write_command(0x123, 10, std::array<uint8_t, 4>{1, 2, 3, 4});
  require(write == (std::array<uint8_t, 8>{0x23, 0x01, 0x55, 10, 1, 2, 3, 4}),
          "register write frame");

  const std::array<uint8_t, 8> reply{0x01, 0x01, 0x33, 10, 0x78, 0x56, 0x34, 0x12};
  const auto decoded = damiao::decode_register_value(reply);
  require(decoded.rid == 10, "register reply rid");
  require(decoded.data == (std::array<uint8_t, 4>{0x78, 0x56, 0x34, 0x12}),
          "register reply data");
  require(damiao::is_register_reply(reply), "register reply marker");
  require(!damiao::is_register_write_ack(reply), "register ack marker");

  std::cout << "damiao protocol tests passed\n";
  return 0;
}

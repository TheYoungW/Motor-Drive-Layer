#include <array>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "damiao/dm_serial_bus.hpp"

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

}  // namespace

int main() {
  const damiao::CanFrame tx{0x123, {1, 2, 3, 4, 5, 6, 7, 8}};
  const auto encoded = damiao::DmSerialCodec::encode_tx(tx);
  require(encoded.size() == 30, "tx frame length");
  require(encoded[0] == 0x55 && encoded[1] == 0xAA, "tx header");
  require(encoded[3] == 0x03, "tx command");
  require(encoded[13] == 0x23 && encoded[14] == 0x01, "tx can id little endian");
  require(encoded[18] == 8, "tx dlc");
  require(encoded[21] == 1 && encoded[28] == 8, "tx payload");

  damiao::DmSerialCodec codec;
  const std::array<uint8_t, 16> raw_rx{
      0xAA, 0x11, 0x08, 0x11, 0x00, 0x00, 0x00, 9, 8, 7, 6, 5, 4, 3, 2, 0x55};
  std::vector<uint8_t> stream{0x00, 0x7E};
  stream.insert(stream.end(), raw_rx.begin(), raw_rx.end());
  codec.push_bytes(stream.data(), stream.size());
  const auto parsed = codec.try_parse_rx();
  require(parsed.has_value(), "rx frame parsed");
  require(parsed->id == 0x11, "rx id");
  require(parsed->data == (std::array<uint8_t, 8>{9, 8, 7, 6, 5, 4, 3, 2}),
          "rx payload");
  require(!codec.try_parse_rx().has_value(), "rx buffer drained");

  std::cout << "dm serial codec tests passed\n";
  return 0;
}

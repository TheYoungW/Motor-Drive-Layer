#include <array>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

#include <unistd.h>

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

  damiao::CanFrame short_ext{0x1ABCDE, {9, 8, 7, 0, 0, 0, 0, 0}};
  short_ext.dlc = 3;
  short_ext.is_extended = true;
  const auto encoded_short_ext = damiao::DmSerialCodec::encode_tx(short_ext);
  require(encoded_short_ext[12] == 1, "tx extended id flag");
  require(encoded_short_ext[18] == 3, "tx preserves dlc");

  bool invalid_standard_id_rejected = false;
  try {
    damiao::CanFrame invalid_id{0x800, {}};
    damiao::DmSerialCodec::encode_tx(invalid_id);
  } catch (const std::invalid_argument&) {
    invalid_standard_id_rejected = true;
  }
  require(invalid_standard_id_rejected, "tx rejects oversized standard id");

  bool invalid_dlc_rejected = false;
  try {
    damiao::CanFrame invalid_dlc{0x123, {}};
    invalid_dlc.dlc = 9;
    damiao::DmSerialCodec::encode_tx(invalid_dlc);
  } catch (const std::invalid_argument&) {
    invalid_dlc_rejected = true;
  }
  require(invalid_dlc_rejected, "tx rejects invalid dlc");

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

  damiao::DmSerialCodec ext_codec;
  const std::array<uint8_t, 16> raw_ext_rx{
      0xAA, 0x11, 0x43, 0xDE, 0xBC, 0x1A, 0x00, 1, 2, 3, 0, 0, 0, 0, 0, 0x55};
  ext_codec.push_bytes(raw_ext_rx.data(), raw_ext_rx.size());
  const auto parsed_ext = ext_codec.try_parse_rx();
  require(parsed_ext.has_value(), "extended rx frame parsed");
  require(parsed_ext->is_extended, "rx preserves extended id flag");
  require(parsed_ext->dlc == 3, "rx preserves dlc");

  int pipe_fds[2] = {-1, -1};
  require(::pipe(pipe_fds) == 0, "create fragmented-rx pipe");
  damiao::DmSerialBus fragmented_bus(pipe_fds[0], "test-pipe");
  std::thread fragmented_writer([&] {
    ::write(pipe_fds[1], raw_rx.data(), 8);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ::write(pipe_fds[1], raw_rx.data() + 8, 8);
    ::close(pipe_fds[1]);
  });
  const auto fragmented = fragmented_bus.receive_for(std::chrono::milliseconds(50));
  fragmented_writer.join();
  require(fragmented.has_value(), "receive_for assembles fragmented serial frames");
  require(fragmented->id == 0x11, "fragmented serial frame id");
  fragmented_bus.shutdown();

  std::cout << "dm serial codec tests passed\n";
  return 0;
}

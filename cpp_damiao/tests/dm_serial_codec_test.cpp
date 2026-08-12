#include <array>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

#if !defined(_WIN32)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

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

  damiao::DmSerialCodec resync_codec;
  const std::array<uint8_t, 8> corrupt_prefix{0xAA, 1, 2, 3, 4, 5, 6, 7};
  std::vector<uint8_t> corrupt_then_valid(corrupt_prefix.begin(), corrupt_prefix.end());
  corrupt_then_valid.insert(corrupt_then_valid.end(), raw_rx.begin(), raw_rx.end());
  resync_codec.push_bytes(corrupt_then_valid.data(), corrupt_then_valid.size());
  const auto resynced = resync_codec.try_parse_rx();
  require(resynced.has_value(), "rx resynchronizes without consuming a nested valid frame");
  require(resynced->id == 0x11, "resynchronized frame id");

  std::vector<uint8_t> burst;
  for (uint8_t motor_id = 1; motor_id <= 8; ++motor_id) {
    auto frame = raw_rx;
    frame[3] = static_cast<uint8_t>(0x10 + motor_id);
    frame[7] = motor_id;
    burst.insert(burst.end(), frame.begin(), frame.end());
  }
  damiao::DmSerialCodec burst_codec;
  for (std::size_t batch = 0; batch < 1001; ++batch) {
    // Vary the split on every batch, including the 64-byte USB bulk-packet boundary.
    const std::size_t split = batch % (burst.size() + 1);
    burst_codec.push_bytes(burst.data(), split);
    burst_codec.push_bytes(burst.data() + split, burst.size() - split);
    for (uint8_t motor_id = 1; motor_id <= 8; ++motor_id) {
      const auto frame = burst_codec.try_parse_rx();
      require(frame.has_value(), "all eight fragmented rx frames are parsed");
      require(frame->id == static_cast<uint32_t>(0x10 + motor_id),
              "fragmented rx frame order");
    }
    require(!burst_codec.try_parse_rx().has_value(), "eight-frame rx burst is drained");
  }

#if !defined(_WIN32)
  int pipe_fds[2] = {-1, -1};
  require(::pipe(pipe_fds) == 0, "create fragmented-rx pipe");
  damiao::DmSerialBus fragmented_bus(pipe_fds[0], "test-pipe");
  std::thread fragmented_writer([&] {
    ::write(pipe_fds[1], raw_rx.data(), 8);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ::write(pipe_fds[1], raw_rx.data() + 8, 8);
    ::close(pipe_fds[1]);
  });
  const auto fragmented = fragmented_bus.receive_for(std::chrono::milliseconds(250));
  fragmented_writer.join();
  require(fragmented.has_value(), "receive_for assembles fragmented serial frames");
  require(fragmented->id == 0x11, "fragmented serial frame id");
  fragmented_bus.shutdown();

#ifdef B1000000
  const int pty_master = ::posix_openpt(O_RDWR | O_NOCTTY);
  require(pty_master >= 0, "open pseudo-terminal master");
  require(::grantpt(pty_master) == 0, "grant pseudo-terminal");
  require(::unlockpt(pty_master) == 0, "unlock pseudo-terminal");
  const char* pty_slave = ::ptsname(pty_master);
  require(pty_slave != nullptr, "resolve pseudo-terminal slave");
  auto one_megabaud_bus = damiao::DmSerialBus::open(pty_slave, 1000000);
  one_megabaud_bus->shutdown();
  ::close(pty_master);
#endif
#endif

  std::cout << "dm serial codec tests passed\n";
  return 0;
}

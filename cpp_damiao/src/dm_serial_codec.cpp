#include "damiao/dm_serial_bus.hpp"

#include <array>
#include <stdexcept>

namespace damiao {
namespace {

constexpr std::size_t kTxFrameLen = 30;
constexpr std::size_t kRxFrameLen = 16;

}  // namespace

std::vector<uint8_t> DmSerialCodec::encode_tx(const CanFrame& frame) {
  if (frame.dlc > 8) {
    throw std::invalid_argument("invalid DLC, expected <= 8");
  }
  if (!frame.is_extended && frame.id > 0x7FFU) {
    throw std::invalid_argument("invalid standard CAN id");
  }
  if (frame.is_extended && frame.id > 0x1FFFFFFFU) {
    throw std::invalid_argument("invalid extended CAN id");
  }
  std::vector<uint8_t> out(kTxFrameLen, 0);
  out[0] = 0x55;
  out[1] = 0xAA;
  out[2] = 0x1E;
  out[3] = 0x03;

  out[4] = 1;
  out[8] = 10;

  out[12] = static_cast<uint8_t>(frame.is_extended);
  out[13] = static_cast<uint8_t>(frame.id & 0xFF);
  out[14] = static_cast<uint8_t>((frame.id >> 8) & 0xFF);
  out[15] = static_cast<uint8_t>((frame.id >> 16) & 0xFF);
  out[16] = static_cast<uint8_t>((frame.id >> 24) & 0xFF);
  out[17] = 0;
  out[18] = frame.dlc;
  for (std::size_t i = 0; i < 8; ++i) {
    out[21 + i] = frame.data[i];
  }
  return out;
}

void DmSerialCodec::push_bytes(const uint8_t* data, std::size_t size) {
  for (std::size_t i = 0; i < size; ++i) {
    rx_buf_.push_back(data[i]);
  }
}

std::optional<CanFrame> DmSerialCodec::try_parse_rx() {
  while (!rx_buf_.empty() && rx_buf_.front() != 0xAA) {
    rx_buf_.pop_front();
  }

  while (rx_buf_.size() >= kRxFrameLen) {
    if (rx_buf_.front() != 0xAA) {
      rx_buf_.pop_front();
      continue;
    }

    std::array<uint8_t, kRxFrameLen> raw{};
    for (std::size_t i = 0; i < raw.size(); ++i) {
      raw[i] = rx_buf_[i];
    }
    for (std::size_t i = 0; i < raw.size(); ++i) {
      rx_buf_.pop_front();
    }

    if (raw[15] != 0x55 || raw[1] != 0x11) {
      continue;
    }
    const uint8_t flags = raw[2];
    const uint8_t dlc = flags & 0x3F;
    const bool is_rtr = (flags & 0x80) != 0;
    if (is_rtr || dlc > 8) {
      continue;
    }

    CanFrame frame{};
    frame.id = static_cast<uint32_t>(raw[3]) | (static_cast<uint32_t>(raw[4]) << 8) |
               (static_cast<uint32_t>(raw[5]) << 16) |
               (static_cast<uint32_t>(raw[6]) << 24);
    frame.dlc = dlc;
    frame.is_extended = (flags & 0x40) != 0;
    for (std::size_t i = 0; i < 8; ++i) {
      frame.data[i] = raw[7 + i];
    }
    return frame;
  }

  return std::nullopt;
}

}  // namespace damiao

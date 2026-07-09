#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "damiao/can_bus.hpp"

namespace damiao {

class DmSerialCodec {
 public:
  static std::vector<uint8_t> encode_tx(const CanFrame& frame);

  void push_bytes(const uint8_t* data, std::size_t size);
  std::optional<CanFrame> try_parse_rx();

 private:
  std::deque<uint8_t> rx_buf_;
};

class DmSerialBus final : public CanBus {
 public:
  static std::shared_ptr<DmSerialBus> open(const std::string& port, uint32_t baud);
  DmSerialBus(int fd, std::string port);
  ~DmSerialBus() override;

  DmSerialBus(const DmSerialBus&) = delete;
  DmSerialBus& operator=(const DmSerialBus&) = delete;

  void send(const CanFrame& frame) override;
  std::optional<CanFrame> receive_for(std::chrono::milliseconds timeout) override;
  void shutdown() override;

 private:
  int fd_;
  std::string port_;
  DmSerialCodec codec_;
  std::mutex mutex_;
};

}  // namespace damiao

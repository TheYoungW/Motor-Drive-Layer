#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "damiao/can_bus.hpp"

namespace damiao {

enum class DmDeviceType {
  Usb2CanFd = 0,
  Usb2CanFdDual = 1,
  LinkX4C = 2,
};

DmDeviceType parse_dm_device_type(const std::string& raw);
uint8_t parse_dm_channel(DmDeviceType device_type, const std::string& raw);
std::string resolve_dm_device_library_path();

class DmDeviceBus final : public CanBus {
 public:
  static std::shared_ptr<DmDeviceBus> open(DmDeviceType device_type,
                                           const std::string& dm_channel);
  ~DmDeviceBus() override;

  DmDeviceBus(const DmDeviceBus&) = delete;
  DmDeviceBus& operator=(const DmDeviceBus&) = delete;

  void send(const CanFrame& frame) override;
  std::optional<CanFrame> receive_for(std::chrono::milliseconds timeout) override;
  void shutdown() override;

 private:
  DmDeviceBus(void* handle, uint8_t channel);

  void* handle_;
  uint8_t channel_;
  std::mutex mutex_;
};

}  // namespace damiao

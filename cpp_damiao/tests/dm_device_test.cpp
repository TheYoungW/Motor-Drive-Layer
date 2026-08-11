#include <iostream>
#include <stdexcept>
#include <string>

#include "damiao/dm_device_bus.hpp"
#include "dm_device_shim.h"

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

template <typename Fn>
void require_throws(const char* label, const Fn& fn) {
  try {
    fn();
  } catch (const std::exception&) {
    return;
  }
  throw std::runtime_error(label);
}

void test_vendor_abi(const char* path) {
  char error[512]{};
  void* channel0 = nullptr;
  void* channel1 = nullptr;
  require(mb_dm_open(path, 1, 0, 1'000'000, 5'000'000, &channel0, error,
                     sizeof(error)) == 0,
          error);
  require(mb_dm_open(path, 1, 1, 1'000'000, 5'000'000, &channel1, error,
                     sizeof(error)) == 0,
          error);

  const uint8_t payload[] = {1, 2, 3, 4};
  require(mb_dm_send(channel0, 0x101, 0, sizeof(payload), payload, error,
                     sizeof(error)) == 0,
          error);
  require(mb_dm_send(channel1, 0x202, 0, sizeof(payload), payload, error,
                     sizeof(error)) == 0,
          error);
  mb_dm_frame frame{};
  require(mb_dm_recv(channel0, &frame, 10, error, sizeof(error)) == 1,
          "channel 0 receives its frame");
  require(frame.channel == 0 && frame.can_id == 0x101 && frame.data[3] == 4,
          "channel 0 frame contents");
  require(mb_dm_recv(channel1, &frame, 10, error, sizeof(error)) == 1,
          "channel 1 receives its frame");
  require(frame.channel == 1 && frame.can_id == 0x202 && frame.data[3] == 4,
          "channel 1 frame contents");

  require(mb_dm_shutdown(channel0, error, sizeof(error)) == 0, error);
  require(mb_dm_send(channel1, 0x203, 0, sizeof(payload), payload, error,
                     sizeof(error)) == 0,
          "channel 1 remains usable after closing channel 0");
  require(mb_dm_shutdown(channel1, error, sizeof(error)) == 0, error);

  // The last shutdown must release the vendor context/device so a new session
  // can enumerate, open, configure and close cleanly.
  channel0 = nullptr;
  require(mb_dm_open(path, 1, 0, 500'000, 2'000'000, &channel0, error,
                     sizeof(error)) == 0,
          error);
  require(mb_dm_shutdown(channel0, error, sizeof(error)) == 0, error);
}

}  // namespace

int main() {
  require(damiao::parse_dm_device_type("usb2canfd") == damiao::DmDeviceType::Usb2CanFd,
          "usb2canfd parse");
  require(damiao::parse_dm_device_type("usb2canfd-dual") ==
              damiao::DmDeviceType::Usb2CanFdDual,
          "dual parse");
  require(damiao::parse_dm_device_type("linkx4c") == damiao::DmDeviceType::LinkX4C,
          "linkx4c parse");

  require(damiao::parse_dm_channel(damiao::DmDeviceType::Usb2CanFd, "0") == 0,
          "single channel 0");
  require(damiao::parse_dm_channel(damiao::DmDeviceType::Usb2CanFdDual, "canfd2") == 1,
          "dual channel alias");
  require(damiao::parse_dm_channel(damiao::DmDeviceType::LinkX4C, "ch3") == 3,
          "linkx4c channel alias");

  require_throws("single channel rejects 1", [] {
    (void)damiao::parse_dm_channel(damiao::DmDeviceType::Usb2CanFd, "1");
  });
  require_throws("dual rejects 2", [] {
    (void)damiao::parse_dm_channel(damiao::DmDeviceType::Usb2CanFdDual, "2");
  });

  const auto path = damiao::resolve_dm_device_library_path();
  require(!path.empty(), "dm device library path is resolved");

  test_vendor_abi(MOCK_DM_DEVICE_V11_PATH);
  test_vendor_abi(MOCK_DM_DEVICE_V10_PATH);

  std::cout << "dm device tests passed\n";
  return 0;
}

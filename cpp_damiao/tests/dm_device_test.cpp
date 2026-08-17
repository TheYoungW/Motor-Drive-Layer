#include <cstdlib>
#include <array>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include "damiao/dm_device_bus.hpp"
#include "dm_device_shim.h"

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void set_test_env(const char* name, const char* value) {
#if defined(_WIN32)
  _putenv_s(name, value);
#else
  ::setenv(name, value, 1);
#endif
}

void unset_test_env(const char* name) {
#if defined(_WIN32)
  _putenv_s(name, "");
#else
  ::unsetenv(name);
#endif
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

void test_vendor_abi(const char* path, uint8_t expected_abi_generation,
                     bool expected_process_session_reuse) {
  char error[512]{};
  void* channel0 = nullptr;
  void* channel1 = nullptr;
  require(mb_dm_open(path, 1, 0, 1'000'000, 5'000'000, &channel0, error,
                     sizeof(error)) == 0,
          error);
  require(mb_dm_open(path, 1, 1, 1'000'000, 5'000'000, &channel1, error,
                     sizeof(error)) == 0,
          error);
  mb_dm_runtime_info runtime_info{};
  require(mb_dm_get_runtime_info(channel0, &runtime_info) == 0,
          "runtime info query succeeds");
  require(runtime_info.abi_generation == expected_abi_generation &&
              (runtime_info.process_session_reuse != 0) ==
                  expected_process_session_reuse,
          "runtime info identifies vendor ABI and process-session behavior");

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
  require(frame.channel == 0 && frame.can_id == 0x101 && frame.data[3] == 4 &&
              frame.canfd == 1 && frame.brs == 1,
          "channel 0 frame contents use CAN-FD with bitrate switching");
  require(mb_dm_recv(channel1, &frame, 10, error, sizeof(error)) == 1,
          "channel 1 receives its frame");
  require(frame.channel == 1 && frame.can_id == 0x202 && frame.data[3] == 4 &&
              frame.canfd == 1 && frame.brs == 1,
          "channel 1 frame contents use CAN-FD with bitrate switching");

  constexpr uint32_t kConcurrentFrames = 64;
  std::thread channel0_sender([&] {
    for (uint32_t sequence = 0; sequence < kConcurrentFrames; ++sequence) {
      const std::array<uint8_t, 8> data{
          0x04, static_cast<uint8_t>(sequence), 0xA0, 0xA1,
          0xA2, 0xA3, 0xA4, 0xA5};
      require(mb_dm_send(channel0, 0x204, 0, data.size(), data.data(), error,
                         sizeof(error)) == 0,
              "concurrent CH0 send succeeds");
    }
  });
  std::thread channel1_sender([&] {
    for (uint32_t sequence = 0; sequence < kConcurrentFrames; ++sequence) {
      const std::array<uint8_t, 8> data{
          0x04, static_cast<uint8_t>(sequence), 0xB0, 0xB1,
          0xB2, 0xB3, 0xB4, 0xB5};
      require(mb_dm_send(channel1, 0x204, 0, data.size(), data.data(), error,
                         sizeof(error)) == 0,
              "concurrent CH1 send succeeds");
    }
  });
  channel0_sender.join();
  channel1_sender.join();
  for (uint32_t sequence = 0; sequence < kConcurrentFrames; ++sequence) {
    require(mb_dm_recv(channel0, &frame, 100, error, sizeof(error)) == 1 &&
                frame.channel == 0 && frame.can_id == 0x204 &&
                frame.data[1] == static_cast<uint8_t>(sequence) &&
                frame.data[2] == 0xA0,
            "same-ID CH0 payload remains isolated during concurrent callbacks");
    require(mb_dm_recv(channel1, &frame, 100, error, sizeof(error)) == 1 &&
                frame.channel == 1 && frame.can_id == 0x204 &&
                frame.data[1] == static_cast<uint8_t>(sequence) &&
                frame.data[2] == 0xB0,
            "same-ID CH1 payload remains isolated during concurrent callbacks");
  }

  require(mb_dm_shutdown(channel0, error, sizeof(error)) == 0, error);
  require(mb_dm_send(channel1, 0x203, 0, sizeof(payload), payload, error,
                     sizeof(error)) == 0,
          "channel 1 remains usable after closing channel 0");
  require(mb_dm_shutdown(channel1, error, sizeof(error)) == 0, error);

  // Reconnect in the same process. v1.1 tears down and opens a new session;
  // v1.0 must reuse its process-level legacy context/device because the real
  // Linux runtime cannot device_open(index=0) after a complete close.
  channel0 = nullptr;
  require(mb_dm_open(path, 1, 0, 500'000, 2'000'000, &channel0, error,
                     sizeof(error)) == 0,
          error);
  require(mb_dm_send(channel0, 0x304, 0, sizeof(payload), payload, error,
                     sizeof(error)) == 0,
          "reconnected channel can send");
  require(mb_dm_recv(channel0, &frame, 10, error, sizeof(error)) == 1 &&
              frame.can_id == 0x304,
          "reconnected channel receives through the retained callbacks");
  require(mb_dm_shutdown(channel0, error, sizeof(error)) == 0, error);

  // Equal arbitration/data rates explicitly retain classic-CAN framing.
  channel0 = nullptr;
  require(mb_dm_open(path, 1, 0, 1'000'000, 1'000'000, &channel0, error,
                     sizeof(error)) == 0,
          error);
  require(mb_dm_send(channel0, 0x305, 0, sizeof(payload), payload, error,
                     sizeof(error)) == 0,
          error);
  require(mb_dm_recv(channel0, &frame, 10, error, sizeof(error)) == 1 &&
              frame.can_id == 0x305 && frame.canfd == 0 && frame.brs == 0,
          "equal bitrates select classic CAN framing");
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

  test_vendor_abi(MOCK_DM_DEVICE_V11_PATH, 11, false);
  test_vendor_abi(MOCK_DM_DEVICE_V10_PATH, 10, true);

  set_test_env("MOTOR_DM_DEVICE_LIB", MOCK_DM_DEVICE_V10_PATH);
  auto bus = damiao::DmDeviceBus::open(damiao::DmDeviceType::Usb2CanFdDual, "1",
                                       1'000'000, 5'000'000);
  const auto capabilities = bus->capabilities();
  require(capabilities.transport == "dm-device" && capabilities.can_fd &&
              capabilities.channel_count == 2 && capabilities.parallel_batches &&
              capabilities.reconnect && capabilities.process_session_reuse &&
              !capabilities.hardware_rx_timestamps && capabilities.can_fd_brs,
          "DM_Device bus reports instance capabilities from the v1.0 runtime");
  bus->shutdown();
  unset_test_env("MOTOR_DM_DEVICE_LIB");

  std::cout << "dm device tests passed\n";
  return 0;
}

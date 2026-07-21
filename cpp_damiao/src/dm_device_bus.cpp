#include "damiao/dm_device_bus.hpp"

#include "dm_device_shim.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <vector>

namespace damiao {
namespace {

std::string lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

std::string error_message(const std::array<char, 512>& err, const char* fallback) {
  const auto len = std::find(err.begin(), err.end(), '\0') - err.begin();
  if (len == 0) {
    return fallback;
  }
  return std::string(err.data(), static_cast<std::size_t>(len));
}

std::string platform_relative_path() {
#if defined(__APPLE__) && defined(__aarch64__)
  return "macos/arm64/libdm_device.dylib";
#elif defined(__APPLE__) && defined(__x86_64__)
  return "macos/x86_64/libdm_device.dylib";
#elif defined(__linux__) && defined(__x86_64__)
  return "linux/x86_64/libdm_device.so";
#elif defined(__linux__) && defined(__aarch64__)
  return "linux/arm64/libdm_device.so";
#elif defined(_WIN32)
  return "windows/msvc/dm_device.dll";
#else
  return "";
#endif
}

const char* library_basename() {
#if defined(_WIN32)
  return "dm_device.dll";
#elif defined(__APPLE__)
  return "libdm_device.dylib";
#else
  return "libdm_device.so";
#endif
}

}  // namespace

DmDeviceType parse_dm_device_type(const std::string& raw) {
  const auto value = lower(raw);
  if (value == "usb2canfd") return DmDeviceType::Usb2CanFd;
  if (value == "usb2canfd-dual" || value == "usb2canfd_dual" || value == "dual") {
    return DmDeviceType::Usb2CanFdDual;
  }
  if (value == "linkx4c") return DmDeviceType::LinkX4C;
  throw std::invalid_argument("unknown dm-device type: " + raw);
}

uint8_t parse_dm_channel(DmDeviceType device_type, const std::string& raw) {
  const auto value = lower(raw);
  if (value == "0" || value == "canfd1" || value == "can1" || value == "ch0" ||
      value == "channel0") {
    return 0;
  }
  if (value == "1" || value == "canfd2" || value == "can2" || value == "ch1" ||
      value == "channel1") {
    if (device_type == DmDeviceType::Usb2CanFd) {
      throw std::invalid_argument("usb2canfd has one physical channel; use dm_channel 0");
    }
    return 1;
  }
  if (value == "2" || value == "canfd3" || value == "can3" || value == "ch2" ||
      value == "channel2") {
    if (device_type != DmDeviceType::LinkX4C) {
      throw std::invalid_argument("selected dm-device type does not support channel 2");
    }
    return 2;
  }
  if (value == "3" || value == "canfd4" || value == "can4" || value == "ch3" ||
      value == "channel3") {
    if (device_type != DmDeviceType::LinkX4C) {
      throw std::invalid_argument("selected dm-device type does not support channel 3");
    }
    return 3;
  }
  throw std::invalid_argument("unknown dm-channel: " + raw);
}

std::string resolve_dm_device_library_path() {
  if (const char* env = std::getenv("MOTOR_DM_DEVICE_LIB")) {
    if (std::filesystem::exists(env)) {
      return env;
    }
    throw std::runtime_error(std::string("MOTOR_DM_DEVICE_LIB points to missing file: ") + env);
  }

  const auto rel = platform_relative_path();
  std::vector<std::filesystem::path> candidates;
  if (!rel.empty()) {
    candidates.push_back(std::filesystem::current_path() / "third_party/dm_device/v1.1.0" / rel);
    candidates.push_back(std::filesystem::current_path().parent_path() /
                         "third_party/dm_device/v1.1.0" / rel);
    candidates.push_back(std::filesystem::path(__FILE__).parent_path().parent_path() /
                         "third_party/dm_device/v1.1.0" / rel);
  }
  candidates.push_back(library_basename());

  for (const auto& candidate : candidates) {
    if (std::filesystem::exists(candidate)) {
      return candidate.string();
    }
  }
  return candidates.back().string();
}

std::shared_ptr<DmDeviceBus> DmDeviceBus::open(DmDeviceType device_type,
                                               const std::string& dm_channel) {
  const auto channel = parse_dm_channel(device_type, dm_channel);
  const auto library_path = resolve_dm_device_library_path();
  std::array<char, 512> err{};
  void* raw = nullptr;
  const int rc = mb_dm_open(library_path.c_str(), static_cast<int>(device_type), channel,
                            1000000, 5000000, &raw, err.data(), err.size());
  if (rc != 0 || raw == nullptr) {
    throw std::runtime_error(error_message(err, "mb_dm_open failed"));
  }
  return std::shared_ptr<DmDeviceBus>(new DmDeviceBus(raw, channel));
}

DmDeviceBus::DmDeviceBus(void* handle, uint8_t channel) : handle_(handle), channel_(channel) {}

DmDeviceBus::~DmDeviceBus() {
  try {
    shutdown();
  } catch (...) {
  }
}

void DmDeviceBus::send(const CanFrame& frame) {
  if (frame.dlc > 8) {
    throw std::invalid_argument("invalid DLC, expected <= 8");
  }
  std::array<char, 512> err{};
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ == nullptr) {
    throw std::runtime_error("dm-device handle already closed");
  }
  const int rc = mb_dm_send(handle_, frame.id, static_cast<uint8_t>(frame.is_extended),
                            frame.dlc, frame.data.data(), err.data(), err.size());
  if (rc != 0) {
    throw std::runtime_error(error_message(err, "mb_dm_send failed"));
  }
}

std::optional<CanFrame> DmDeviceBus::receive_for(std::chrono::milliseconds timeout) {
  std::array<char, 512> err{};
  mb_dm_frame raw{};
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ == nullptr) {
    throw std::runtime_error("dm-device handle already closed");
  }
  const auto timeout_ms = static_cast<uint32_t>(
      std::min<int64_t>(timeout.count(), static_cast<int64_t>(UINT32_MAX)));
  const int rc = mb_dm_recv(handle_, &raw, timeout_ms, err.data(), err.size());
  if (rc < 0) {
    throw std::runtime_error(error_message(err, "mb_dm_recv failed"));
  }
  if (rc == 0) {
    return std::nullopt;
  }
  CanFrame frame;
  frame.id = raw.can_id;
  frame.dlc = std::min<uint8_t>(raw.dlc, 8);
  frame.is_extended = raw.ext != 0;
  std::copy(raw.data, raw.data + 8, frame.data.begin());
  return frame;
}

void DmDeviceBus::shutdown() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ != nullptr) {
    mb_dm_shutdown(handle_);
    handle_ = nullptr;
  }
}

}  // namespace damiao

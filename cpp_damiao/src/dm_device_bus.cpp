#include "damiao/dm_device_bus.hpp"

#include "dm_device_shim.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <vector>

#if defined(_WIN32)
#include <windows.h>
#else
#include <dlfcn.h>
#endif

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

std::string legacy_platform_relative_path() {
#if defined(__linux__) && defined(__x86_64__)
  return "linux/libdm_device.so";
#elif defined(__linux__) && defined(__aarch64__)
  return "aarch64/libdm_device.so";
#elif defined(_WIN32)
  return "msvc/dm_device.dll";
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

std::filesystem::path native_library_directory() {
#if defined(_WIN32)
  HMODULE module = nullptr;
  if (!GetModuleHandleExA(
          GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
              GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
          reinterpret_cast<LPCSTR>(&native_library_directory), &module)) {
    return {};
  }
  std::array<char, 4096> path{};
  const auto length = GetModuleFileNameA(module, path.data(), path.size());
  if (length == 0 || length >= path.size()) return {};
  return std::filesystem::path(std::string(path.data(), length)).parent_path();
#else
  Dl_info info{};
  if (dladdr(reinterpret_cast<const void*>(&native_library_directory), &info) == 0 ||
      !info.dli_fname) {
    return {};
  }
  return std::filesystem::path(info.dli_fname).parent_path();
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
  const auto legacy_rel = legacy_platform_relative_path();
  std::vector<std::filesystem::path> candidates;
  const auto native_dir = native_library_directory();
  if (!native_dir.empty()) {
    if (!legacy_rel.empty()) {
      candidates.push_back(native_dir / "dm_device/v1.0.0" /
                           std::filesystem::path(legacy_rel).filename());
    }
    if (!rel.empty()) {
      candidates.push_back(native_dir / "dm_device/v1.1.0" /
                           std::filesystem::path(rel).filename());
    }
  }
  if (!legacy_rel.empty()) {
    candidates.push_back(std::filesystem::current_path() / "third_party/dm_device/v1.0.0" /
                         legacy_rel);
    candidates.push_back(std::filesystem::current_path().parent_path() /
                         "third_party/dm_device/v1.0.0" / legacy_rel);
    candidates.push_back(std::filesystem::path(__FILE__).parent_path().parent_path() /
                         "third_party/dm_device/v1.0.0" / legacy_rel);
  }
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
                                               const std::string& dm_channel,
                                               uint32_t bitrate,
                                               uint32_t data_bitrate) {
  if (bitrate == 0 || data_bitrate == 0) {
    throw std::invalid_argument("dm-device bitrate and data_bitrate must be positive");
  }
  const auto channel = parse_dm_channel(device_type, dm_channel);
  const auto library_path = resolve_dm_device_library_path();
  std::array<char, 512> err{};
  void* raw = nullptr;
  const int rc = mb_dm_open(library_path.c_str(), static_cast<int>(device_type), channel,
                            bitrate, data_bitrate, &raw, err.data(), err.size());
  if (rc != 0 || raw == nullptr) {
    throw std::runtime_error(error_message(err, "mb_dm_open failed"));
  }
  mb_dm_runtime_info runtime_info{};
  if (mb_dm_get_runtime_info(raw, &runtime_info) != 0) {
    mb_dm_shutdown(raw, err.data(), err.size());
    throw std::runtime_error("failed to query DM_Device runtime capabilities");
  }
  return std::shared_ptr<DmDeviceBus>(new DmDeviceBus(
      raw, channel, device_type, runtime_info.process_session_reuse != 0));
}

DmDeviceBus::DmDeviceBus(void* handle, uint8_t channel, DmDeviceType device_type,
                         bool process_session_reuse)
    : handle_(handle),
      channel_(channel),
      device_type_(device_type),
      process_session_reuse_(process_session_reuse) {}

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
  if (raw.channel != channel_) {
    throw std::runtime_error(
        "dm-device receive queue returned a frame for channel " +
        std::to_string(raw.channel) + " to channel " +
        std::to_string(channel_));
  }
  CanFrame frame;
  frame.id = raw.can_id;
  frame.dlc = std::min<uint8_t>(raw.dlc, 8);
  frame.is_extended = raw.ext != 0;
  frame.channel = raw.channel;
  std::copy(raw.data, raw.data + 8, frame.data.begin());
  return frame;
}

void DmDeviceBus::shutdown() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ != nullptr) {
    std::array<char, 512> err{};
    const int rc = mb_dm_shutdown(handle_, err.data(), err.size());
    handle_ = nullptr;
    if (rc != 0) {
      throw std::runtime_error(error_message(err, "mb_dm_shutdown failed"));
    }
  }
}

TransportCapabilities DmDeviceBus::capabilities() const {
  uint32_t channels = 1;
  if (device_type_ == DmDeviceType::Usb2CanFdDual) channels = 2;
  if (device_type_ == DmDeviceType::LinkX4C) channels = 4;
  return TransportCapabilities{"dm-device", 8, channels, true, true, false, true,
                               process_session_reuse_};
}

}  // namespace damiao

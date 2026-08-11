#include "dmcan.h"

#include "dm_device_shim.h"

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#if defined(_WIN32)
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace {

enum class VendorAbi { V10, V11 };

struct legacy_root;
struct legacy_device;

struct LegacyBaudInfo {
  int can_baudrate;
  int canfd_baudrate;
  float can_sp;
  float canfd_sp;
};

using modern_context_create_fn = void (*)(dmcan_context**);
using modern_context_destroy_fn = void (*)(dmcan_context*);
using modern_find_devices_with_type_fn = int (*)(dmcan_context*, dmcan_device_type_t);
using modern_device_get_fn = bool (*)(dmcan_context*, dmcan_device_handle**, int);
using modern_device_open_fn = bool (*)(dmcan_device_handle*);
using modern_device_close_fn = void (*)(dmcan_device_handle*);
using modern_device_enable_channel_fn = bool (*)(dmcan_device_handle*, uint8_t);
using modern_device_disable_channel_fn = bool (*)(dmcan_device_handle*, uint8_t);
using modern_device_get_channel_baudrate_fn =
    bool (*)(dmcan_device_handle*, uint8_t, dmcan_channel_can_info_t*);
using modern_device_set_channel_baudrate_fn =
    bool (*)(dmcan_device_handle*, uint8_t, dmcan_channel_can_info_t);
using modern_device_hook_recv_callback_fn = void (*)(dmcan_device_handle*, dev_recv_callback);
using modern_device_hook_err_callback_fn = void (*)(dmcan_device_handle*, dev_err_callback);
using modern_device_send_can_fn =
    bool (*)(dmcan_device_handle*, uint8_t, uint32_t, bool, bool, bool, bool, uint8_t,
             uint8_t*);

using legacy_root_create_fn = legacy_root* (*)(int);
using legacy_root_destroy_fn = void (*)(legacy_root*);
using legacy_find_devices_fn = int (*)(legacy_root*);
using legacy_get_devices_fn = void (*)(legacy_root*, legacy_device**, int*);
using legacy_device_open_fn = bool (*)(legacy_device*);
using legacy_device_close_fn = bool (*)(legacy_device*);
using legacy_device_open_channel_fn = bool (*)(legacy_device*, uint8_t);
using legacy_device_close_channel_fn = bool (*)(legacy_device*, uint8_t);
using legacy_device_get_baudrate_fn = bool (*)(legacy_device*, uint8_t, LegacyBaudInfo*);
using legacy_device_set_baud_fn =
    bool (*)(legacy_device*, uint8_t, bool, int, int, float, float);
using legacy_recv_callback = void (*)(usb_rx_frame_t*);
using legacy_err_callback = void (*)(usb_rx_frame_t*);
using legacy_device_hook_recv_fn = void (*)(legacy_device*, legacy_recv_callback);
using legacy_device_hook_err_fn = void (*)(legacy_device*, legacy_err_callback);
using legacy_device_send_fn =
    void (*)(legacy_device*, uint8_t, uint32_t, int32_t, bool, bool, bool, uint8_t,
             uint8_t*);

struct Api {
  void* lib = nullptr;
  void* usb_dependency = nullptr;
  VendorAbi abi = VendorAbi::V11;

  modern_context_create_fn modern_context_create = nullptr;
  modern_context_destroy_fn modern_context_destroy = nullptr;
  modern_find_devices_with_type_fn modern_find_devices_with_type = nullptr;
  modern_device_get_fn modern_device_get = nullptr;
  modern_device_open_fn modern_device_open = nullptr;
  modern_device_close_fn modern_device_close = nullptr;
  modern_device_enable_channel_fn modern_device_enable_channel = nullptr;
  modern_device_disable_channel_fn modern_device_disable_channel = nullptr;
  modern_device_get_channel_baudrate_fn modern_device_get_channel_baudrate = nullptr;
  modern_device_set_channel_baudrate_fn modern_device_set_channel_baudrate = nullptr;
  modern_device_hook_recv_callback_fn modern_device_hook_recv = nullptr;
  modern_device_hook_err_callback_fn modern_device_hook_err = nullptr;
  modern_device_send_can_fn modern_device_send = nullptr;

  legacy_root_create_fn legacy_root_create = nullptr;
  legacy_root_destroy_fn legacy_root_destroy = nullptr;
  legacy_find_devices_fn legacy_find_devices = nullptr;
  legacy_get_devices_fn legacy_get_devices = nullptr;
  legacy_device_open_fn legacy_device_open = nullptr;
  legacy_device_close_fn legacy_device_close = nullptr;
  legacy_device_open_channel_fn legacy_device_open_channel = nullptr;
  legacy_device_close_channel_fn legacy_device_close_channel = nullptr;
  legacy_device_get_baudrate_fn legacy_device_get_baudrate = nullptr;
  legacy_device_set_baud_fn legacy_device_set_baud = nullptr;
  legacy_device_hook_recv_fn legacy_device_hook_recv = nullptr;
  legacy_device_hook_err_fn legacy_device_hook_err = nullptr;
  legacy_device_send_fn legacy_device_send = nullptr;
};

struct ChannelConfig {
  uint32_t bitrate = 0;
  uint32_t data_bitrate = 0;

  bool operator==(const ChannelConfig& other) const {
    return bitrate == other.bitrate && data_bitrate == other.data_bitrate;
  }
};

struct Session {
  Api api;
  std::string library_path;
  int device_type = -1;
  dmcan_context* modern_context = nullptr;
  dmcan_device_handle* modern_device = nullptr;
  legacy_root* legacy_context = nullptr;
  legacy_device* legacy_dev = nullptr;
  std::mutex vendor_mutex;
  std::array<std::optional<ChannelConfig>, 4> channel_configs;
  std::array<std::size_t, 4> channel_refs{};
};

struct mb_dm_handle {
  std::shared_ptr<Session> session;
  uint8_t selected_channel = 0;
  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  std::deque<mb_dm_frame> queue;
  std::string pending_error;
  bool stopped = false;
};

std::mutex g_lifecycle_mutex;
std::mutex g_registry_mutex;
std::shared_ptr<Session> g_session;
std::vector<mb_dm_handle*> g_clients;

struct ProcessSessionCleanup {
  ~ProcessSessionCleanup();
};

ProcessSessionCleanup g_process_session_cleanup;

void set_err(char* err_buf, size_t err_len, const std::string& msg) {
  if (!err_buf || err_len == 0) return;
  const size_t n = msg.size() < err_len - 1 ? msg.size() : err_len - 1;
  std::memcpy(err_buf, msg.data(), n);
  err_buf[n] = '\0';
}

#if defined(_WIN32)
std::string loader_error() {
  const DWORD code = GetLastError();
  char* raw = nullptr;
  const DWORD len = FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                                       FORMAT_MESSAGE_IGNORE_INSERTS,
                                   nullptr, code, 0, reinterpret_cast<char*>(&raw), 0, nullptr);
  std::string message = len && raw ? std::string(raw, len) : "Windows loader error " +
                                                               std::to_string(code);
  if (raw) LocalFree(raw);
  return message;
}

void* open_library(const char* path) { return reinterpret_cast<void*>(LoadLibraryA(path)); }
void close_library(void* lib) {
  if (lib) FreeLibrary(reinterpret_cast<HMODULE>(lib));
}
void* load_symbol(void* lib, const char* name) {
  return reinterpret_cast<void*>(GetProcAddress(reinterpret_cast<HMODULE>(lib), name));
}
void* open_usb_dependency() { return nullptr; }
#else
std::string loader_error() {
  const char* raw = dlerror();
  return raw ? raw : "unknown dynamic-loader error";
}

void* open_library(const char* path) {
  dlerror();
  return dlopen(path, RTLD_NOW | RTLD_LOCAL);
}
void close_library(void* lib) {
  if (lib) dlclose(lib);
}
void* load_symbol(void* lib, const char* name) {
  dlerror();
  return dlsym(lib, name);
}
void* open_usb_dependency() {
#if defined(__APPLE__)
  return dlopen("libusb-1.0.dylib", RTLD_NOW | RTLD_GLOBAL);
#else
  return dlopen("libusb-1.0.so.0", RTLD_NOW | RTLD_GLOBAL);
#endif
}
#endif

void unload_api(Api& api) {
  close_library(api.lib);
  api.lib = nullptr;
  close_library(api.usb_dependency);
  api.usb_dependency = nullptr;
}

template <typename T>
bool load_required(Api& api, T& dst, const char* name, const char* abi_name, char* err_buf,
                   size_t err_len) {
  void* sym = load_symbol(api.lib, name);
  if (!sym) {
    set_err(err_buf, err_len,
            std::string("DM_Device ") + abi_name + " runtime is missing required symbol " +
                name);
    return false;
  }
  dst = reinterpret_cast<T>(sym);
  return true;
}

bool load_modern_api(Api& api, char* err_buf, size_t err_len) {
  api.abi = VendorAbi::V11;
  return load_required(api, api.modern_context_create, "dmcan_context_create", "v1.1", err_buf,
                       err_len) &&
         load_required(api, api.modern_context_destroy, "dmcan_context_destroy", "v1.1",
                       err_buf, err_len) &&
         load_required(api, api.modern_find_devices_with_type,
                       "dmcan_find_devices_with_type", "v1.1", err_buf, err_len) &&
         load_required(api, api.modern_device_get, "dmcan_device_get", "v1.1", err_buf,
                       err_len) &&
         load_required(api, api.modern_device_open, "dmcan_device_open", "v1.1", err_buf,
                       err_len) &&
         load_required(api, api.modern_device_close, "dmcan_device_close", "v1.1", err_buf,
                       err_len) &&
         load_required(api, api.modern_device_enable_channel, "dmcan_device_enable_channel",
                       "v1.1", err_buf, err_len) &&
         load_required(api, api.modern_device_disable_channel, "dmcan_device_disable_channel",
                       "v1.1", err_buf, err_len) &&
         load_required(api, api.modern_device_get_channel_baudrate,
                       "dmcan_device_get_channel_baudrate", "v1.1", err_buf, err_len) &&
         load_required(api, api.modern_device_set_channel_baudrate,
                       "dmcan_device_set_channel_baudrate", "v1.1", err_buf, err_len) &&
         load_required(api, api.modern_device_hook_recv, "dmcan_device_hook_recv_callback",
                       "v1.1", err_buf, err_len) &&
         load_required(api, api.modern_device_hook_err, "dmcan_device_hook_err_callback", "v1.1",
                       err_buf, err_len) &&
         load_required(api, api.modern_device_send, "dmcan_device_send_can", "v1.1", err_buf,
                       err_len);
}

bool load_legacy_api(Api& api, char* err_buf, size_t err_len) {
  api.abi = VendorAbi::V10;
  return load_required(api, api.legacy_root_create, "damiao_handle_create", "v1.0", err_buf,
                       err_len) &&
         load_required(api, api.legacy_root_destroy, "damiao_handle_destroy", "v1.0", err_buf,
                       err_len) &&
         load_required(api, api.legacy_find_devices, "damiao_handle_find_devices", "v1.0",
                       err_buf, err_len) &&
         load_required(api, api.legacy_get_devices, "damiao_handle_get_devices", "v1.0", err_buf,
                       err_len) &&
         load_required(api, api.legacy_device_open, "device_open", "v1.0", err_buf, err_len) &&
         load_required(api, api.legacy_device_close, "device_close", "v1.0", err_buf, err_len) &&
         load_required(api, api.legacy_device_open_channel, "device_open_channel", "v1.0",
                       err_buf, err_len) &&
         load_required(api, api.legacy_device_close_channel, "device_close_channel", "v1.0",
                       err_buf, err_len) &&
         load_required(api, api.legacy_device_get_baudrate, "device_channel_get_baudrate", "v1.0",
                       err_buf, err_len) &&
         load_required(api, api.legacy_device_set_baud, "device_channel_set_baud_with_sp",
                       "v1.0", err_buf, err_len) &&
         load_required(api, api.legacy_device_hook_recv, "device_hook_to_rec", "v1.0", err_buf,
                       err_len) &&
         load_required(api, api.legacy_device_hook_err, "device_hook_to_err", "v1.0", err_buf,
                       err_len) &&
         load_required(api, api.legacy_device_send, "device_channel_send_fast", "v1.0", err_buf,
                       err_len);
}

bool load_api(Api& api, const char* library_path, char* err_buf, size_t err_len) {
  api.lib = open_library(library_path);
  std::string load_error;
  if (!api.lib) {
    load_error = loader_error();
    // The official Linux v1.0 binary references libusb symbols without a
    // DT_NEEDED entry.  Load the system libusb globally and retry so users do
    // not have to discover an LD_PRELOAD workaround.
    api.usb_dependency = open_usb_dependency();
    if (api.usb_dependency) {
      api.lib = open_library(library_path);
      if (!api.lib) load_error += "; after loading libusb: " + loader_error();
    }
  }
  if (!api.lib) {
    set_err(err_buf, err_len,
            std::string("failed to load DM_Device runtime '") + library_path + "': " +
                load_error +
                ". Install a compatible vendor runtime or set MOTOR_DM_DEVICE_LIB");
    return false;
  }

  if (load_symbol(api.lib, "dmcan_context_create")) {
    if (load_modern_api(api, err_buf, err_len)) return true;
  } else if (load_symbol(api.lib, "damiao_handle_create")) {
    if (load_legacy_api(api, err_buf, err_len)) return true;
  } else {
    set_err(err_buf, err_len,
            std::string("unsupported DM_Device runtime ABI in '") + library_path +
                "': neither v1.1 symbol dmcan_context_create nor v1.0 symbol "
                "damiao_handle_create was found");
  }
  unload_api(api);
  return false;
}

std::string channel_error(const char* operation, uint8_t channel) {
  return std::string(operation) + "(channel=" + std::to_string(channel) + ") failed";
}

bool retry_bool(const std::function<bool()>& fn) {
  for (int attempt = 0; attempt < 5; ++attempt) {
    if (fn()) return true;
    if (attempt + 1 < 5) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return false;
}

void route_frame(Session* expected, usb_rx_frame_t* frame) {
  if (!expected || !frame || frame->head.rtr) return;
  mb_dm_frame out{};
  out.can_id = frame->head.can_id;
  out.dlc = static_cast<uint8_t>(frame->head.dlc > 8 ? 8 : frame->head.dlc);
  out.channel = frame->head.channel;
  out.ext = static_cast<uint8_t>(frame->head.ext ? 1 : 0);
  out.canfd = static_cast<uint8_t>(frame->head.canfd ? 1 : 0);
  if (out.dlc > 0) std::memcpy(out.data, frame->payload, out.dlc);

  std::lock_guard<std::mutex> registry_lock(g_registry_mutex);
  for (auto* client : g_clients) {
    if (!client || client->session.get() != expected ||
        client->selected_channel != out.channel) {
      continue;
    }
    {
      std::lock_guard<std::mutex> queue_lock(client->queue_mutex);
      if (client->stopped) continue;
      if (client->queue.size() >= 512) client->queue.pop_front();
      client->queue.push_back(out);
    }
    client->queue_cv.notify_one();
  }
}

void route_error(Session* expected, usb_rx_frame_t* frame) {
  if (!expected || !frame) return;
  std::ostringstream message;
  message << "DM_Device asynchronous CAN error on channel "
          << static_cast<unsigned>(frame->head.channel) << " (id=0x" << std::hex
          << frame->head.can_id << ")";
  std::lock_guard<std::mutex> registry_lock(g_registry_mutex);
  for (auto* client : g_clients) {
    if (!client || client->session.get() != expected ||
        client->selected_channel != frame->head.channel) {
      continue;
    }
    {
      std::lock_guard<std::mutex> queue_lock(client->queue_mutex);
      if (client->stopped) continue;
      client->pending_error = message.str();
    }
    client->queue_cv.notify_one();
  }
}

std::shared_ptr<Session> modern_session(dmcan_device_handle* device) {
  std::lock_guard<std::mutex> lock(g_registry_mutex);
  if (!g_session || g_session->modern_device != device) return {};
  return g_session;
}

std::shared_ptr<Session> legacy_session() {
  std::lock_guard<std::mutex> lock(g_registry_mutex);
  if (!g_session || g_session->api.abi != VendorAbi::V10) return {};
  return g_session;
}

void modern_recv_callback(dmcan_device_handle* device, usb_rx_frame_t* frame) {
  const auto session = modern_session(device);
  route_frame(session.get(), frame);
}
void modern_err_callback(dmcan_device_handle* device, usb_rx_frame_t* frame) {
  const auto session = modern_session(device);
  route_error(session.get(), frame);
}
void legacy_recv_callback_fn(usb_rx_frame_t* frame) {
  const auto session = legacy_session();
  route_frame(session.get(), frame);
}
void legacy_err_callback_fn(usb_rx_frame_t* frame) {
  const auto session = legacy_session();
  route_error(session.get(), frame);
}

bool install_callbacks(const std::shared_ptr<Session>& session) {
  std::lock_guard<std::mutex> vendor_lock(session->vendor_mutex);
  if (session->api.abi == VendorAbi::V11) {
    session->api.modern_device_hook_recv(session->modern_device, modern_recv_callback);
    session->api.modern_device_hook_err(session->modern_device, modern_err_callback);
  } else {
    session->api.legacy_device_hook_recv(session->legacy_dev, legacy_recv_callback_fn);
    session->api.legacy_device_hook_err(session->legacy_dev, legacy_err_callback_fn);
  }
  return true;
}

std::shared_ptr<Session> create_session(const char* library_path, int device_type, char* err_buf,
                                        size_t err_len) {
  auto session = std::make_shared<Session>();
  session->library_path = library_path;
  session->device_type = device_type;
  if (!load_api(session->api, library_path, err_buf, err_len)) return nullptr;

  if (session->api.abi == VendorAbi::V11) {
    session->api.modern_context_create(&session->modern_context);
    if (!session->modern_context) {
      set_err(err_buf, err_len, "DM_Device v1.1 dmcan_context_create returned null");
      unload_api(session->api);
      return nullptr;
    }
    int count = 0;
    for (int attempt = 0; attempt < 5; ++attempt) {
      count = session->api.modern_find_devices_with_type(
          session->modern_context, static_cast<dmcan_device_type_t>(device_type));
      if (count > 0) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (count <= 0 || !session->api.modern_device_get(session->modern_context,
                                                       &session->modern_device, 0) ||
        !session->modern_device) {
      set_err(err_buf, err_len,
              "DM_Device v1.1 found no matching device (device index 0 unavailable)");
      session->api.modern_context_destroy(session->modern_context);
      session->modern_context = nullptr;
      unload_api(session->api);
      return nullptr;
    }
    if (!retry_bool([&] { return session->api.modern_device_open(session->modern_device); })) {
      set_err(err_buf, err_len, "DM_Device v1.1 dmcan_device_open(index=0) failed");
      session->api.modern_context_destroy(session->modern_context);
      session->modern_context = nullptr;
      unload_api(session->api);
      return nullptr;
    }
  } else {
    if (device_type == 2) {
      set_err(err_buf, err_len, "DM_Device v1.0 runtime does not support linkx4c");
      unload_api(session->api);
      return nullptr;
    }
    session->legacy_context = session->api.legacy_root_create(device_type);
    if (!session->legacy_context) {
      set_err(err_buf, err_len, "DM_Device v1.0 damiao_handle_create returned null");
      unload_api(session->api);
      return nullptr;
    }
    int count = 0;
    for (int attempt = 0; attempt < 5; ++attempt) {
      count = session->api.legacy_find_devices(session->legacy_context);
      if (count > 0) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::array<legacy_device*, 16> devices{};
    int returned_count = 0;
    if (count > 0) {
      session->api.legacy_get_devices(session->legacy_context, devices.data(), &returned_count);
    }
    if (count <= 0 || returned_count <= 0 || !devices[0]) {
      set_err(err_buf, err_len,
              "DM_Device v1.0 found no matching device (device index 0 unavailable)");
      session->api.legacy_root_destroy(session->legacy_context);
      session->legacy_context = nullptr;
      unload_api(session->api);
      return nullptr;
    }
    session->legacy_dev = devices[0];
    if (!retry_bool([&] { return session->api.legacy_device_open(session->legacy_dev); })) {
      set_err(err_buf, err_len, "DM_Device v1.0 device_open(index=0) failed");
      session->api.legacy_root_destroy(session->legacy_context);
      session->legacy_context = nullptr;
      unload_api(session->api);
      return nullptr;
    }
  }
  return session;
}

bool configure_channel(Session& session, uint8_t channel, ChannelConfig config, char* err_buf,
                       size_t err_len) {
  if (channel >= session.channel_refs.size()) {
    set_err(err_buf, err_len, "DM_Device channel is out of range");
    return false;
  }
  std::lock_guard<std::mutex> vendor_lock(session.vendor_mutex);
  if (session.channel_refs[channel] > 0) {
    if (!session.channel_configs[channel].has_value() ||
        !(*session.channel_configs[channel] == config)) {
      set_err(err_buf, err_len,
              "DM_Device channel " + std::to_string(channel) +
                  " is already open with different bitrate settings");
      return false;
    }
    ++session.channel_refs[channel];
    return true;
  }

  bool configured = false;
  if (session.api.abi == VendorAbi::V11) {
    if (!retry_bool(
            [&] { return session.api.modern_device_enable_channel(session.modern_device, channel); })) {
      set_err(err_buf, err_len, channel_error("dmcan_device_enable_channel", channel));
      return false;
    }
    dmcan_channel_can_info_t info{};
    if (!session.api.modern_device_get_channel_baudrate(session.modern_device, channel, &info)) {
      set_err(err_buf, err_len, channel_error("dmcan_device_get_channel_baudrate", channel));
      session.api.modern_device_disable_channel(session.modern_device, channel);
      return false;
    }
    info.channel = channel;
    info.canfd = true;
    info.can_baudrate = config.bitrate;
    info.canfd_baudrate = config.data_bitrate;
    info.can_sp = 0.75f;
    info.canfd_sp = 0.75f;
    configured =
        session.api.modern_device_set_channel_baudrate(session.modern_device, channel, info);
    if (!configured) {
      set_err(err_buf, err_len, channel_error("dmcan_device_set_channel_baudrate", channel));
      session.api.modern_device_disable_channel(session.modern_device, channel);
      return false;
    }
  } else {
    configured = session.api.legacy_device_set_baud(
        session.legacy_dev, channel, true, static_cast<int>(config.bitrate),
        static_cast<int>(config.data_bitrate), 0.75f, 0.75f);
    if (!configured) {
      set_err(err_buf, err_len, channel_error("device_channel_set_baud_with_sp", channel));
      return false;
    }
    if (!retry_bool(
            [&] { return session.api.legacy_device_open_channel(session.legacy_dev, channel); })) {
      set_err(err_buf, err_len, channel_error("device_open_channel", channel));
      return false;
    }
  }
  session.channel_configs[channel] = config;
  session.channel_refs[channel] = 1;
  return true;
}

bool release_channel(Session& session, uint8_t channel, std::string& error) {
  if (channel >= session.channel_refs.size()) return true;
  std::lock_guard<std::mutex> vendor_lock(session.vendor_mutex);
  if (session.channel_refs[channel] == 0) return true;
  --session.channel_refs[channel];
  if (session.channel_refs[channel] > 0) return true;
  bool ok = true;
  if (session.api.abi == VendorAbi::V11) {
    // v1.1 owns channel transfer objects at device scope.  Disabling a single
    // channel while the device is shared can race its libusb event thread;
    // dmcan_device_close releases every enabled channel on final shutdown.
    ok = true;
  } else {
    ok = session.api.legacy_device_close_channel(session.legacy_dev, channel);
  }
  session.channel_configs[channel].reset();
  if (!ok) error = channel_error(session.api.abi == VendorAbi::V11
                                      ? "dmcan_device_disable_channel"
                                      : "device_close_channel",
                                  channel);
  return ok;
}

bool close_session(Session& session, std::string& error) {
  bool ok = true;
  {
    std::lock_guard<std::mutex> vendor_lock(session.vendor_mutex);
    if (session.api.abi == VendorAbi::V11) {
      session.modern_device = nullptr;
      // v1.1 documents context destruction as closing every device.  Using
      // that single owner-level teardown avoids double-cancelling libusb
      // transfers via dmcan_device_close followed by context destruction.
      session.api.modern_context_destroy(session.modern_context);
      session.modern_context = nullptr;
    } else {
      if (!session.api.legacy_device_close(session.legacy_dev)) {
        ok = false;
        error = "DM_Device v1.0 device_close failed";
      }
      session.legacy_dev = nullptr;
      session.api.legacy_root_destroy(session.legacy_context);
      session.legacy_context = nullptr;
    }
  }
  unload_api(session.api);
  return ok;
}

bool retains_process_session(const Session& session) {
  // The official Linux v1.0 runtime cannot reliably open device index 0 again
  // after device_close/damiao_handle_destroy in the same process. Keep its
  // device, root context and loaded library alive, while individual clients
  // still close their channels and release all motor-layer resources.
  return session.api.abi == VendorAbi::V10;
}

ProcessSessionCleanup::~ProcessSessionCleanup() {
  std::lock_guard<std::mutex> lifecycle_lock(g_lifecycle_mutex);
  std::shared_ptr<Session> session;
  {
    std::lock_guard<std::mutex> registry_lock(g_registry_mutex);
    // Live clients indicate an application-level lifetime bug. Avoid tearing
    // their callbacks out from under static destructors in another module.
    if (!g_clients.empty()) return;
    session = std::move(g_session);
  }
  if (session) {
    // Linux v1.0 is intentionally retained for the entire process because
    // the vendor runtime cannot reliably reopen after complete teardown.
    // Destroying that retained context from a static destructor also races
    // libusb/Python thread teardown and can corrupt the process heap. At
    // process exit the operating system safely reclaims these vendor-owned
    // resources, so only explicitly tear down non-retained sessions here.
    if (retains_process_session(*session)) return;
    std::string ignored;
    close_session(*session, ignored);
  }
}

}  // namespace

extern "C" int mb_dm_open(const char* library_path, int device_type, uint8_t selected_channel,
                          uint32_t can_baudrate, uint32_t canfd_baudrate, void** out,
                          char* err_buf, size_t err_len) {
  if (!library_path || !out || can_baudrate == 0 || canfd_baudrate == 0) {
    set_err(err_buf, err_len, "invalid mb_dm_open argument or zero bitrate");
    return -1;
  }
  *out = nullptr;
  std::lock_guard<std::mutex> lifecycle_lock(g_lifecycle_mutex);
  try {
    std::shared_ptr<Session> session;
    {
      std::lock_guard<std::mutex> registry_lock(g_registry_mutex);
      session = g_session;
    }
    bool created = false;
    if (session) {
      if (session->library_path != library_path || session->device_type != device_type) {
        set_err(err_buf, err_len,
                "another DM_Device runtime/device type is already active in this process");
        return -1;
      }
    } else {
      session = create_session(library_path, device_type, err_buf, err_len);
      if (!session) return -1;
      created = true;
      {
        std::lock_guard<std::mutex> registry_lock(g_registry_mutex);
        g_session = session;
      }
      install_callbacks(session);
    }

    if (!configure_channel(*session, selected_channel,
                           ChannelConfig{can_baudrate, canfd_baudrate}, err_buf, err_len)) {
      if (created && !retains_process_session(*session)) {
        {
          std::lock_guard<std::mutex> registry_lock(g_registry_mutex);
          g_session.reset();
        }
        std::string ignored;
        close_session(*session, ignored);
      }
      return -1;
    }

    auto* client = new mb_dm_handle();
    client->session = session;
    client->selected_channel = selected_channel;
    {
      std::lock_guard<std::mutex> registry_lock(g_registry_mutex);
      g_clients.push_back(client);
    }
    *out = client;
    return 0;
  } catch (const std::exception& exc) {
    set_err(err_buf, err_len, std::string("DM_Device open failed: ") + exc.what());
    return -1;
  } catch (...) {
    set_err(err_buf, err_len, "DM_Device open failed with unknown exception");
    return -1;
  }
}

extern "C" int mb_dm_send(void* opaque_handle, uint32_t can_id, uint8_t ext, uint8_t dlc,
                          const uint8_t* data, char* err_buf, size_t err_len) {
  auto* handle = static_cast<mb_dm_handle*>(opaque_handle);
  if (!handle || !data || dlc > 8) {
    set_err(err_buf, err_len, "invalid mb_dm_send argument");
    return -1;
  }
  uint8_t payload[8]{};
  std::memcpy(payload, data, dlc);
  std::lock_guard<std::mutex> vendor_lock(handle->session->vendor_mutex);
  if (handle->session->api.abi == VendorAbi::V11) {
    if (!handle->session->api.modern_device_send(
            handle->session->modern_device, handle->selected_channel, can_id, false, ext != 0,
            false, false, dlc, payload)) {
      set_err(err_buf, err_len,
              channel_error("dmcan_device_send_can", handle->selected_channel));
      return -1;
    }
  } else {
    handle->session->api.legacy_device_send(handle->session->legacy_dev,
                                            handle->selected_channel, can_id, 1, ext != 0, false,
                                            false, dlc, payload);
  }
  return 0;
}

extern "C" int mb_dm_recv(void* opaque_handle, mb_dm_frame* out, uint32_t timeout_ms,
                          char* err_buf, size_t err_len) {
  auto* handle = static_cast<mb_dm_handle*>(opaque_handle);
  if (!handle || !out) {
    set_err(err_buf, err_len, "invalid mb_dm_recv argument");
    return -1;
  }
  std::unique_lock<std::mutex> lock(handle->queue_mutex);
  if (handle->queue.empty() && handle->pending_error.empty() && timeout_ms > 0) {
    handle->queue_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&] {
      return handle->stopped || !handle->pending_error.empty() || !handle->queue.empty();
    });
  }
  if (!handle->pending_error.empty()) {
    set_err(err_buf, err_len, handle->pending_error);
    handle->pending_error.clear();
    return -1;
  }
  if (handle->queue.empty()) return 0;
  *out = handle->queue.front();
  handle->queue.pop_front();
  return 1;
}

extern "C" int mb_dm_shutdown(void* opaque_handle, char* err_buf, size_t err_len) {
  auto* handle = static_cast<mb_dm_handle*>(opaque_handle);
  if (!handle) return 0;
  std::lock_guard<std::mutex> lifecycle_lock(g_lifecycle_mutex);
  auto session = handle->session;
  bool last_client = false;
  {
    std::lock_guard<std::mutex> registry_lock(g_registry_mutex);
    for (auto it = g_clients.begin(); it != g_clients.end(); ++it) {
      if (*it == handle) {
        g_clients.erase(it);
        break;
      }
    }
    last_client = true;
    for (const auto* client : g_clients) {
      if (client && client->session == session) {
        last_client = false;
        break;
      }
    }
    if (last_client && g_session == session && !retains_process_session(*session)) {
      g_session.reset();
    }
  }
  {
    std::lock_guard<std::mutex> queue_lock(handle->queue_mutex);
    handle->stopped = true;
  }
  handle->queue_cv.notify_all();

  std::string error;
  bool ok = release_channel(*session, handle->selected_channel, error);
  if (last_client && !retains_process_session(*session) &&
      !close_session(*session, error)) {
    ok = false;
  }
  delete handle;
  if (!ok) {
    set_err(err_buf, err_len, error.empty() ? "DM_Device shutdown failed" : error);
    return -1;
  }
  return 0;
}

extern "C" int mb_dm_get_runtime_info(void* opaque_handle, mb_dm_runtime_info* out) {
  auto* handle = static_cast<mb_dm_handle*>(opaque_handle);
  if (!handle || !out) return -1;
  out->abi_generation = handle->session->api.abi == VendorAbi::V10 ? 10 : 11;
  out->process_session_reuse =
      static_cast<uint8_t>(retains_process_session(*handle->session) ? 1 : 0);
  return 0;
}

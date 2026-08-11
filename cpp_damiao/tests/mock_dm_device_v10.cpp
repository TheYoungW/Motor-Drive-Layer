#include "dmcan.h"

#include <array>
#include <cstdlib>
#include <cstring>

struct legacy_device {
  bool open = false;
  std::array<bool, 4> channel_open{};
  dev_recv_callback unused_modern_callback = nullptr;
};
struct legacy_root {
  legacy_device device;
};

namespace {
using legacy_recv_callback = void (*)(usb_rx_frame_t*);
using legacy_err_callback = void (*)(usb_rx_frame_t*);
legacy_recv_callback recv_callback = nullptr;
legacy_err_callback err_callback = nullptr;
bool device_was_closed = false;
}

extern "C" {
legacy_root* damiao_handle_create(int) { return new legacy_root(); }
void damiao_handle_destroy(legacy_root*) {
  // The Linux v1.0 session is retained until process exit. Calling into the
  // real vendor destructor from static teardown can corrupt libusb state, so
  // make the test process fail if the shim regresses to that behavior.
  std::abort();
}
int damiao_handle_find_devices(legacy_root*) { return 1; }
void damiao_handle_get_devices(legacy_root* root, legacy_device** list, int* count) {
  list[0] = &root->device;
  *count = 1;
}
bool device_open(legacy_device* dev) {
  // Match the observed vendor v1.0 behavior: once the process closes the
  // physical device, a newly-created legacy context cannot reopen index 0.
  if (dev->open || device_was_closed) return false;
  dev->open = true;
  return true;
}
bool device_close(legacy_device* dev) {
  dev->open = false;
  device_was_closed = true;
  return true;
}
bool device_open_channel(legacy_device* dev, uint8_t channel) {
  if (!dev->open || channel >= dev->channel_open.size()) return false;
  dev->channel_open[channel] = true;
  return true;
}
bool device_close_channel(legacy_device* dev, uint8_t channel) {
  if (channel >= dev->channel_open.size()) return false;
  dev->channel_open[channel] = false;
  return true;
}
bool device_channel_get_baudrate(legacy_device*, uint8_t, void*) { return true; }
bool device_channel_set_baud_with_sp(legacy_device*, uint8_t channel, bool, int bitrate,
                                     int data_bitrate, float, float) {
  return channel < 4 && bitrate > 0 && data_bitrate > 0;
}
void device_hook_to_rec(legacy_device*, legacy_recv_callback callback) {
  recv_callback = callback;
}
void device_hook_to_err(legacy_device*, legacy_err_callback callback) {
  err_callback = callback;
}
void device_channel_send_fast(legacy_device* dev, uint8_t channel, uint32_t can_id, int32_t,
                              bool ext, bool canfd, bool brs, uint8_t len, uint8_t* payload) {
  if (!dev->open || channel >= dev->channel_open.size() || !dev->channel_open[channel]) return;
  usb_rx_frame_t frame{};
  frame.head.can_id = can_id;
  frame.head.channel = channel;
  frame.head.ext = ext;
  frame.head.canfd = canfd;
  frame.head.brs = brs;
  frame.head.dlc = len;
  if (payload && len) std::memcpy(frame.payload, payload, len);
  if (recv_callback) recv_callback(&frame);
}
}

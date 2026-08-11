#include "dmcan.h"

#include <array>
#include <cstring>

struct dmcan_context {};
struct dmcan_device_handle {
  bool open = false;
  std::array<bool, 4> enabled{};
  std::array<dmcan_channel_can_info_t, 4> baud{};
  dev_recv_callback recv = nullptr;
  dev_err_callback err = nullptr;
};

namespace {
dmcan_device_handle device;
}

extern "C" {
void dmcan_context_create(dmcan_context** ctx) { *ctx = new dmcan_context(); }
void dmcan_context_destroy(dmcan_context* ctx) {
  device.open = false;
  delete ctx;
}
int dmcan_find_devices_with_type(dmcan_context*, dmcan_device_type_t) { return 1; }
bool dmcan_device_get(dmcan_context*, dmcan_device_handle** out, int index) {
  if (index != 0) return false;
  *out = &device;
  return true;
}
bool dmcan_device_open(dmcan_device_handle* dev) {
  if (dev->open) return false;
  dev->open = true;
  return true;
}
void dmcan_device_close(dmcan_device_handle* dev) { dev->open = false; }
bool dmcan_device_enable_channel(dmcan_device_handle* dev, uint8_t channel) {
  if (!dev->open || channel >= dev->enabled.size()) return false;
  dev->enabled[channel] = true;
  return true;
}
bool dmcan_device_disable_channel(dmcan_device_handle* dev, uint8_t channel) {
  if (channel >= dev->enabled.size()) return false;
  dev->enabled[channel] = false;
  return true;
}
bool dmcan_device_get_channel_baudrate(dmcan_device_handle* dev, uint8_t channel,
                                       dmcan_channel_can_info_t* info) {
  if (channel >= dev->baud.size() || !info) return false;
  *info = dev->baud[channel];
  return true;
}
bool dmcan_device_set_channel_baudrate(dmcan_device_handle* dev, uint8_t channel,
                                       dmcan_channel_can_info_t info) {
  if (channel >= dev->baud.size()) return false;
  dev->baud[channel] = info;
  return true;
}
void dmcan_device_hook_recv_callback(dmcan_device_handle* dev, dev_recv_callback callback) {
  dev->recv = callback;
}
void dmcan_device_hook_err_callback(dmcan_device_handle* dev, dev_err_callback callback) {
  dev->err = callback;
}
bool dmcan_device_send_can(dmcan_device_handle* dev, uint8_t channel, uint32_t can_id,
                           bool canfd, bool ext, bool rtr, bool brs, uint8_t dlen,
                           uint8_t* payload) {
  if (!dev->open || channel >= dev->enabled.size() || !dev->enabled[channel]) return false;
  usb_rx_frame_t frame{};
  frame.head.can_id = can_id;
  frame.head.channel = channel;
  frame.head.canfd = canfd;
  frame.head.ext = ext;
  frame.head.rtr = rtr;
  frame.head.brs = brs;
  frame.head.dlc = dlen;
  if (payload && dlen) std::memcpy(frame.payload, payload, dlen);
  if (dev->recv) dev->recv(dev, &frame);
  return true;
}
}

#include "damiao/socketcan_bus.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>

#if defined(__linux__)
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace damiao {
namespace {

void validate_frame(const CanFrame& frame) {
  if (frame.dlc > 8) {
    throw std::invalid_argument("invalid DLC, expected <= 8");
  }
  if (!frame.is_extended && frame.id > SocketCanCodec::kCanSffMask) {
    throw std::invalid_argument("invalid standard CAN id");
  }
  if (frame.is_extended && frame.id > SocketCanCodec::kCanEffMask) {
    throw std::invalid_argument("invalid extended CAN id");
  }
}

#if defined(__linux__)
std::runtime_error os_error(const std::string& prefix) {
  return std::runtime_error(prefix + ": " + std::strerror(errno));
}

int open_bound_socket(const std::string& interface, bool canfd) {
  const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    throw os_error("socket(PF_CAN, SOCK_RAW, CAN_RAW) failed");
  }

  if (canfd) {
    int enable = 1;
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) < 0) {
      const auto err = os_error("setsockopt(CAN_RAW_FD_FRAMES) failed");
      ::close(fd);
      throw err;
    }
  }

  ifreq ifr{};
  std::snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", interface.c_str());
  if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    const auto err = os_error("ioctl(SIOCGIFINDEX) failed for " + interface);
    ::close(fd);
    throw err;
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    const auto err = os_error("bind failed for " + interface);
    ::close(fd);
    throw err;
  }
  return fd;
}
#endif

}  // namespace

SocketCanRawFrame SocketCanCodec::encode_classic(const CanFrame& frame) {
  validate_frame(frame);
  SocketCanRawFrame raw;
  raw.can_id = frame.is_extended ? (frame.id | kCanEffFlag) : frame.id;
  raw.can_dlc = frame.dlc;
  raw.data = frame.data;
  return raw;
}

CanFrame SocketCanCodec::decode_classic(const SocketCanRawFrame& raw) {
  CanFrame frame;
  frame.is_extended = (raw.can_id & kCanEffFlag) != 0;
  frame.id = frame.is_extended ? (raw.can_id & kCanEffMask) : (raw.can_id & kCanSffMask);
  frame.dlc = std::min<uint8_t>(raw.can_dlc, 8);
  frame.data = raw.data;
  return frame;
}

SocketCanFdRawFrame SocketCanCodec::encode_fd(const CanFrame& frame, bool enable_brs) {
  validate_frame(frame);
  SocketCanFdRawFrame raw;
  raw.can_id = frame.is_extended ? (frame.id | kCanEffFlag) : frame.id;
  raw.len = frame.dlc;
  raw.flags = enable_brs ? kCanFdBrs : 0;
  std::copy(frame.data.begin(), frame.data.end(), raw.data.begin());
  return raw;
}

CanFrame SocketCanCodec::decode_fd(const SocketCanFdRawFrame& raw) {
  CanFrame frame;
  frame.is_extended = (raw.can_id & kCanEffFlag) != 0;
  frame.id = frame.is_extended ? (raw.can_id & kCanEffMask) : (raw.can_id & kCanSffMask);
  frame.dlc = std::min<uint8_t>(raw.len, 8);
  std::copy_n(raw.data.begin(), 8, frame.data.begin());
  return frame;
}

std::shared_ptr<SocketCanBus> SocketCanBus::open(const std::string& interface) {
#if defined(__linux__)
  return std::shared_ptr<SocketCanBus>(new SocketCanBus(open_bound_socket(interface, false),
                                                        interface));
#else
  (void)interface;
  throw std::runtime_error("socketcan transport is only available on Linux");
#endif
}

SocketCanBus::SocketCanBus(int fd, std::string interface)
    : fd_(fd), interface_(std::move(interface)) {}

SocketCanBus::~SocketCanBus() {
  try {
    shutdown();
  } catch (...) {
  }
}

void SocketCanBus::send(const CanFrame& frame) {
#if defined(__linux__)
  const auto raw_portable = SocketCanCodec::encode_classic(frame);
  can_frame raw{};
  raw.can_id = raw_portable.can_id;
  raw.can_dlc = raw_portable.can_dlc;
  std::copy(raw_portable.data.begin(), raw_portable.data.end(), raw.data);
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) throw std::runtime_error("socketcan fd already closed");
  if (::write(fd_, &raw, sizeof(raw)) != static_cast<ssize_t>(sizeof(raw))) {
    throw os_error("socketcan write failed");
  }
#else
  (void)frame;
  throw std::runtime_error("socketcan transport is only available on Linux");
#endif
}

std::optional<CanFrame> SocketCanBus::receive_for(std::chrono::milliseconds timeout) {
#if defined(__linux__)
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) throw std::runtime_error("socketcan fd already closed");
  pollfd pfd{fd_, POLLIN, 0};
  const int rc = ::poll(&pfd, 1, static_cast<int>(timeout.count()));
  if (rc < 0) throw os_error("socketcan poll failed");
  if (rc == 0) return std::nullopt;
  can_frame raw{};
  if (::read(fd_, &raw, sizeof(raw)) != static_cast<ssize_t>(sizeof(raw))) {
    throw os_error("socketcan read failed");
  }
  SocketCanRawFrame portable;
  portable.can_id = raw.can_id;
  portable.can_dlc = raw.can_dlc;
  std::copy(raw.data, raw.data + 8, portable.data.begin());
  return SocketCanCodec::decode_classic(portable);
#else
  (void)timeout;
  throw std::runtime_error("socketcan transport is only available on Linux");
#endif
}

void SocketCanBus::shutdown() {
#if defined(__linux__)
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
#endif
}

std::shared_ptr<SocketCanFdBus> SocketCanFdBus::open(const std::string& interface,
                                                     bool enable_brs) {
#if defined(__linux__)
  return std::shared_ptr<SocketCanFdBus>(new SocketCanFdBus(open_bound_socket(interface, true),
                                                            interface, enable_brs));
#else
  (void)interface;
  (void)enable_brs;
  throw std::runtime_error("socketcanfd transport is only available on Linux");
#endif
}

SocketCanFdBus::SocketCanFdBus(int fd, std::string interface, bool enable_brs)
    : fd_(fd), interface_(std::move(interface)), enable_brs_(enable_brs) {}

SocketCanFdBus::~SocketCanFdBus() {
  try {
    shutdown();
  } catch (...) {
  }
}

void SocketCanFdBus::send(const CanFrame& frame) {
#if defined(__linux__)
  const auto raw_portable = SocketCanCodec::encode_fd(frame, enable_brs_);
  canfd_frame raw{};
  raw.can_id = raw_portable.can_id;
  raw.len = raw_portable.len;
  raw.flags = raw_portable.flags;
  std::copy(raw_portable.data.begin(), raw_portable.data.end(), raw.data);
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) throw std::runtime_error("socketcanfd fd already closed");
  if (::write(fd_, &raw, sizeof(raw)) != static_cast<ssize_t>(sizeof(raw))) {
    throw os_error("socketcanfd write failed");
  }
#else
  (void)frame;
  throw std::runtime_error("socketcanfd transport is only available on Linux");
#endif
}

std::optional<CanFrame> SocketCanFdBus::receive_for(std::chrono::milliseconds timeout) {
#if defined(__linux__)
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) throw std::runtime_error("socketcanfd fd already closed");
  pollfd pfd{fd_, POLLIN, 0};
  const int rc = ::poll(&pfd, 1, static_cast<int>(timeout.count()));
  if (rc < 0) throw os_error("socketcanfd poll failed");
  if (rc == 0) return std::nullopt;
  canfd_frame raw{};
  if (::read(fd_, &raw, sizeof(raw)) != static_cast<ssize_t>(sizeof(raw))) {
    throw os_error("socketcanfd read failed");
  }
  SocketCanFdRawFrame portable;
  portable.can_id = raw.can_id;
  portable.len = raw.len;
  portable.flags = raw.flags;
  std::copy(raw.data, raw.data + 64, portable.data.begin());
  return SocketCanCodec::decode_fd(portable);
#else
  (void)timeout;
  throw std::runtime_error("socketcanfd transport is only available on Linux");
#endif
}

void SocketCanFdBus::shutdown() {
#if defined(__linux__)
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
#endif
}

}  // namespace damiao

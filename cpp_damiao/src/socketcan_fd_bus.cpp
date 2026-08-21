#include "damiao/socketcan_fd_bus.hpp"

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <cstring>
#include <cstdlib>
#include <limits>
#include <stdexcept>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

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

constexpr auto kDefaultSendTimeout = std::chrono::milliseconds(20);
constexpr uint64_t kMaximumSendTimeoutMs = 60000;

std::runtime_error os_error(const std::string& prefix) {
  return std::runtime_error(prefix + ": " + std::strerror(errno));
}

std::chrono::milliseconds configured_send_timeout() {
  const char* raw = std::getenv("MOTOR_DRIVE_LAYER_SOCKETCAN_SEND_TIMEOUT_MS");
  if (raw == nullptr) return kDefaultSendTimeout;

  uint64_t timeout_ms = 0;
  const auto* end = raw + std::strlen(raw);
  const auto parsed = std::from_chars(raw, end, timeout_ms);
  if (parsed.ec != std::errc{} || parsed.ptr != end || timeout_ms == 0 ||
      timeout_ms > kMaximumSendTimeoutMs) {
    return kDefaultSendTimeout;
  }
  return std::chrono::milliseconds(static_cast<int64_t>(timeout_ms));
}

void write_frame_with_timeout(int fd, const void* frame, size_t frame_size,
                              std::chrono::milliseconds timeout,
                              const std::string& endpoint) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  int queue_error = EAGAIN;

  while (true) {
    const ssize_t written = ::write(fd, frame, frame_size);
    if (written == static_cast<ssize_t>(frame_size)) return;
    if (written >= 0) {
      throw std::runtime_error(endpoint + " send failed: short frame write (" +
                               std::to_string(written) + "/" +
                               std::to_string(frame_size) + " bytes)");
    }

    const int error = errno;
    if (error == EINTR) {
      if (std::chrono::steady_clock::now() >= deadline) break;
      continue;
    }
    if (error != EAGAIN && error != EWOULDBLOCK && error != ENOBUFS) {
      throw std::runtime_error(endpoint + " send failed: " + std::strerror(error));
    }
    queue_error = error;

    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) break;
    const auto remaining = deadline - now;
    auto poll_ms = std::chrono::ceil<std::chrono::milliseconds>(remaining).count();
    poll_ms = std::min<int64_t>(poll_ms, std::numeric_limits<int>::max());

    pollfd pfd{fd, POLLOUT, 0};
    const int rc = ::poll(&pfd, 1, static_cast<int>(poll_ms));
    if (rc < 0) {
      if (errno == EINTR) {
        if (std::chrono::steady_clock::now() >= deadline) break;
        continue;
      }
      throw os_error(endpoint + " send poll failed");
    }
    if (rc == 0) break;
    if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      throw std::runtime_error(endpoint + " send poll reported socket error (revents=" +
                               std::to_string(pfd.revents) + ")");
    }
  }

  throw std::runtime_error(
      endpoint + " send timed out after " + std::to_string(timeout.count()) +
      " ms: transmit queue remained full (" + std::strerror(queue_error) + ")");
}

int open_bound_socket(const std::string& interface) {
  const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    throw os_error("socket(PF_CAN, SOCK_RAW, CAN_RAW) failed");
  }

  const int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    const auto err = os_error("failed to make SocketCAN socket non-blocking");
    ::close(fd);
    throw err;
  }

  int enable = 1;
  if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) < 0) {
    const auto err = os_error("setsockopt(CAN_RAW_FD_FRAMES) failed");
    ::close(fd);
    throw err;
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
}  // namespace

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

std::shared_ptr<SocketCanFdBus> SocketCanFdBus::open(const std::string& interface,
                                                     bool enable_brs) {
  return std::shared_ptr<SocketCanFdBus>(new SocketCanFdBus(open_bound_socket(interface),
                                                            interface, enable_brs,
                                                            configured_send_timeout()));
}

SocketCanFdBus::SocketCanFdBus(int fd, std::string interface, bool enable_brs,
                               std::chrono::milliseconds send_timeout)
    : fd_(fd),
      interface_(std::move(interface)),
      enable_brs_(enable_brs),
      send_timeout_(send_timeout) {}

SocketCanFdBus::~SocketCanFdBus() {
  try {
    shutdown();
  } catch (...) {
  }
}

void SocketCanFdBus::send(const CanFrame& frame) {
  const auto raw_portable = SocketCanCodec::encode_fd(frame, enable_brs_);
  canfd_frame raw{};
  raw.can_id = raw_portable.can_id;
  raw.len = raw_portable.len;
  raw.flags = raw_portable.flags;
  std::copy(raw_portable.data.begin(), raw_portable.data.end(), raw.data);
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) throw std::runtime_error("socketcanfd fd already closed");
  write_frame_with_timeout(fd_, &raw, sizeof(raw), send_timeout_,
                           "socketcanfd " + interface_);
}

std::optional<CanFrame> SocketCanFdBus::receive_for(std::chrono::milliseconds timeout) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) throw std::runtime_error("socketcanfd fd already closed");
  pollfd pfd{fd_, POLLIN, 0};
  const int rc = ::poll(&pfd, 1, static_cast<int>(timeout.count()));
  if (rc < 0) {
    if (errno == EINTR) return std::nullopt;
    throw os_error("socketcanfd poll failed");
  }
  if (rc == 0) return std::nullopt;
  canfd_frame raw{};
  const ssize_t received = ::read(fd_, &raw, sizeof(raw));
  if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
    return std::nullopt;
  }
  if (received != static_cast<ssize_t>(sizeof(raw))) {
    throw os_error("socketcanfd read failed");
  }
  SocketCanFdRawFrame portable;
  portable.can_id = raw.can_id;
  portable.len = raw.len;
  portable.flags = raw.flags;
  std::copy(raw.data, raw.data + 64, portable.data.begin());
  return SocketCanCodec::decode_fd(portable);
}

void SocketCanFdBus::shutdown() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

TransportCapabilities SocketCanFdBus::capabilities() const {
  // Motor-Drive-Layer's canonical Damiao frame is currently limited to the
  // protocol's eight-byte payload even though the transport uses CAN-FD.
  return TransportCapabilities{"socketcanfd", 8, 1, true, true, false, true, false,
                               enable_brs_};
}

}  // namespace damiao

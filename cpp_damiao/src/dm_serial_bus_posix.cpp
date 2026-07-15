#include "damiao/dm_serial_bus.hpp"

#include <algorithm>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstring>
#include <stdexcept>

#if defined(__APPLE__)
#include <IOKit/serial/ioss.h>
#include <sys/ioctl.h>
#endif

namespace damiao {
namespace {

speed_t baud_to_constant(uint32_t baud) {
  switch (baud) {
    case 9600:
      return B9600;
    case 115200:
      return B115200;
#ifdef B230400
    case 230400:
      return B230400;
#endif
#ifdef B460800
    case 460800:
      return B460800;
#endif
#ifdef B921600
    case 921600:
      return B921600;
#endif
#ifdef B1000000
    case 1000000:
      return B1000000;
#elif defined(__APPLE__)
    case 1000000:
      // macOS applies the actual non-standard rate with IOSSIOSPEED below.
      return B9600;
#endif
    default:
      throw std::invalid_argument("unsupported serial baud: " + std::to_string(baud));
  }
}

std::runtime_error errno_error(const std::string& prefix) {
  return std::runtime_error(prefix + ": " + std::strerror(errno));
}

}  // namespace

std::shared_ptr<DmSerialBus> DmSerialBus::open(const std::string& port, uint32_t baud) {
  const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    throw errno_error("open serial port " + port + " failed");
  }

  termios tio{};
  if (tcgetattr(fd, &tio) != 0) {
    const auto err = errno_error("tcgetattr failed");
    ::close(fd);
    throw err;
  }

  cfmakeraw(&tio);
  const speed_t speed = baud_to_constant(baud);
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);
  tio.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
#ifdef CRTSCTS
  tio.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#elif defined(__APPLE__)
  tio.c_cflag &= static_cast<tcflag_t>(~(CCTS_OFLOW | CRTS_IFLOW));
#endif
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 1;

  if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    const auto err = errno_error("tcsetattr failed");
    ::close(fd);
    throw err;
  }

#if defined(__APPLE__)
  if (baud == 1000000) {
    speed_t custom_speed = static_cast<speed_t>(baud);
    if (::ioctl(fd, IOSSIOSPEED, &custom_speed) != 0) {
      const auto err = errno_error("IOSSIOSPEED failed");
      ::close(fd);
      throw err;
    }
  }
#endif

  return std::make_shared<DmSerialBus>(fd, port);
}

DmSerialBus::DmSerialBus(int fd, std::string port) : fd_(fd), port_(std::move(port)) {}

DmSerialBus::~DmSerialBus() {
  try {
    shutdown();
  } catch (...) {
  }
}

void DmSerialBus::send(const CanFrame& frame) {
  const auto raw = DmSerialCodec::encode_tx(frame);
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) {
    throw std::runtime_error("serial port already closed");
  }
  std::size_t written = 0;
  while (written < raw.size()) {
    const auto n = ::write(fd_, raw.data() + written, raw.size() - written);
    if (n > 0) {
      written += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && errno == EINTR) {
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      pollfd pfd{fd_, POLLOUT, 0};
      const int rc = ::poll(&pfd, 1, 100);
      if (rc > 0) continue;
      if (rc < 0 && errno == EINTR) continue;
      if (rc == 0) throw std::runtime_error("dm-serial write timed out");
    }
    throw errno_error("dm-serial write failed");
  }
}

std::optional<CanFrame> DmSerialBus::receive_for(std::chrono::milliseconds timeout) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) {
    throw std::runtime_error("serial port already closed");
  }

  const bool wait_for_data = timeout > std::chrono::milliseconds::zero();
  const auto deadline = std::chrono::steady_clock::now() + std::max(timeout, std::chrono::milliseconds(0));
  for (;;) {
    if (const auto frame = codec_.try_parse_rx()) {
      return frame;
    }

    int timeout_ms = 0;
    if (wait_for_data) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) return std::nullopt;
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
      timeout_ms = static_cast<int>(std::max<int64_t>(remaining.count(), 1));
    }

    pollfd pfd{fd_, POLLIN, 0};
    const int rc = ::poll(&pfd, 1, timeout_ms);
    if (rc < 0) {
      if (errno == EINTR) continue;
      throw errno_error("dm-serial poll failed");
    }
    if (rc == 0) {
      return std::nullopt;
    }

    std::array<uint8_t, 256> tmp{};
    const auto n = ::read(fd_, tmp.data(), tmp.size());
    if (n < 0) {
      if (errno == EINTR) continue;
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        if (wait_for_data) continue;
        return std::nullopt;
      }
      throw errno_error("dm-serial read failed");
    }
    if (n == 0) return std::nullopt;
    codec_.push_bytes(tmp.data(), static_cast<std::size_t>(n));
  }
}

void DmSerialBus::shutdown() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) {
    tcdrain(fd_);
    ::close(fd_);
    fd_ = -1;
  }
}

}  // namespace damiao

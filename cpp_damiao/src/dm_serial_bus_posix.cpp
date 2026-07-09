#include "damiao/dm_serial_bus.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace damiao {
namespace {

constexpr std::size_t kTxFrameLen = 30;
constexpr std::size_t kRxFrameLen = 16;

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
    default:
      throw std::invalid_argument("unsupported serial baud: " + std::to_string(baud));
  }
}

std::runtime_error errno_error(const std::string& prefix) {
  return std::runtime_error(prefix + ": " + std::strerror(errno));
}

}  // namespace

std::vector<uint8_t> DmSerialCodec::encode_tx(const CanFrame& frame) {
  std::vector<uint8_t> out(kTxFrameLen, 0);
  out[0] = 0x55;
  out[1] = 0xAA;
  out[2] = 0x1E;
  out[3] = 0x03;

  out[4] = 1;
  out[8] = 10;

  out[12] = 0;
  out[13] = static_cast<uint8_t>(frame.id & 0xFF);
  out[14] = static_cast<uint8_t>((frame.id >> 8) & 0xFF);
  out[15] = static_cast<uint8_t>((frame.id >> 16) & 0xFF);
  out[16] = static_cast<uint8_t>((frame.id >> 24) & 0xFF);
  out[17] = 0;
  out[18] = 8;
  for (std::size_t i = 0; i < 8; ++i) {
    out[21 + i] = frame.data[i];
  }
  return out;
}

void DmSerialCodec::push_bytes(const uint8_t* data, std::size_t size) {
  for (std::size_t i = 0; i < size; ++i) {
    rx_buf_.push_back(data[i]);
  }
}

std::optional<CanFrame> DmSerialCodec::try_parse_rx() {
  while (!rx_buf_.empty() && rx_buf_.front() != 0xAA) {
    rx_buf_.pop_front();
  }

  while (rx_buf_.size() >= kRxFrameLen) {
    if (rx_buf_.front() != 0xAA) {
      rx_buf_.pop_front();
      continue;
    }

    std::array<uint8_t, kRxFrameLen> raw{};
    for (std::size_t i = 0; i < raw.size(); ++i) {
      raw[i] = rx_buf_[i];
    }
    for (std::size_t i = 0; i < raw.size(); ++i) {
      rx_buf_.pop_front();
    }

    if (raw[15] != 0x55 || raw[1] != 0x11) {
      continue;
    }
    const uint8_t flags = raw[2];
    const uint8_t dlc = flags & 0x3F;
    const bool is_rtr = (flags & 0x80) != 0;
    if (is_rtr || dlc > 8) {
      continue;
    }

    CanFrame frame{};
    frame.id = static_cast<uint32_t>(raw[3]) | (static_cast<uint32_t>(raw[4]) << 8) |
               (static_cast<uint32_t>(raw[5]) << 16) |
               (static_cast<uint32_t>(raw[6]) << 24);
    for (std::size_t i = 0; i < 8; ++i) {
      frame.data[i] = raw[7 + i];
    }
    return frame;
  }

  return std::nullopt;
}

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
  tio.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 1;

  if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    const auto err = errno_error("tcsetattr failed");
    ::close(fd);
    throw err;
  }

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
  const auto n = ::write(fd_, raw.data(), raw.size());
  if (n != static_cast<ssize_t>(raw.size())) {
    throw errno_error("dm-serial write failed");
  }
}

std::optional<CanFrame> DmSerialBus::receive_for(std::chrono::milliseconds timeout) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) {
    throw std::runtime_error("serial port already closed");
  }

  if (const auto frame = codec_.try_parse_rx()) {
    return frame;
  }

  pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLIN;
  const int timeout_ms = static_cast<int>(timeout.count());
  const int rc = ::poll(&pfd, 1, timeout_ms);
  if (rc < 0) {
    throw errno_error("dm-serial poll failed");
  }
  if (rc == 0) {
    return std::nullopt;
  }

  std::array<uint8_t, 256> tmp{};
  const auto n = ::read(fd_, tmp.data(), tmp.size());
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return std::nullopt;
    }
    throw errno_error("dm-serial read failed");
  }
  if (n > 0) {
    codec_.push_bytes(tmp.data(), static_cast<std::size_t>(n));
  }
  return codec_.try_parse_rx();
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

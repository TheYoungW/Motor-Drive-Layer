#include "damiao/dm_serial_bus.hpp"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace damiao {
namespace {

std::runtime_error windows_error(const std::string& prefix) {
  const DWORD code = GetLastError();
  char* message = nullptr;
  const DWORD size = FormatMessageA(
      FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
          FORMAT_MESSAGE_IGNORE_INSERTS,
      nullptr, code, 0, reinterpret_cast<char*>(&message), 0, nullptr);
  std::string detail = size > 0 && message != nullptr
                           ? std::string(message, static_cast<std::size_t>(size))
                           : "Windows error " + std::to_string(code);
  if (message != nullptr) {
    LocalFree(message);
  }
  while (!detail.empty() && (detail.back() == '\r' || detail.back() == '\n')) {
    detail.pop_back();
  }
  return std::runtime_error(prefix + ": " + detail);
}

std::string normalize_port(const std::string& port) {
  if (port.rfind("\\\\.\\", 0) == 0) {
    return port;
  }
  return "\\\\.\\" + port;
}

HANDLE as_handle(void* handle) {
  return static_cast<HANDLE>(handle);
}

}  // namespace

std::shared_ptr<DmSerialBus> DmSerialBus::open(const std::string& port, uint32_t baud) {
  const auto native_port = normalize_port(port);
  HANDLE handle = CreateFileA(native_port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                              OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (handle == INVALID_HANDLE_VALUE) {
    throw windows_error("open serial port " + port + " failed");
  }

  auto close_on_error = [&]() { CloseHandle(handle); };

  DCB dcb{};
  dcb.DCBlength = sizeof(dcb);
  if (!GetCommState(handle, &dcb)) {
    const auto err = windows_error("GetCommState failed");
    close_on_error();
    throw err;
  }
  dcb.BaudRate = baud;
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fBinary = TRUE;
  dcb.fParity = FALSE;
  dcb.fOutxCtsFlow = FALSE;
  dcb.fOutxDsrFlow = FALSE;
  dcb.fDtrControl = DTR_CONTROL_DISABLE;
  dcb.fDsrSensitivity = FALSE;
  dcb.fOutX = FALSE;
  dcb.fInX = FALSE;
  dcb.fRtsControl = RTS_CONTROL_DISABLE;
  if (!SetCommState(handle, &dcb)) {
    const auto err = windows_error("SetCommState failed");
    close_on_error();
    throw err;
  }

  COMMTIMEOUTS timeouts{};
  timeouts.ReadIntervalTimeout = MAXDWORD;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  timeouts.ReadTotalTimeoutConstant = 0;
  timeouts.WriteTotalTimeoutMultiplier = 0;
  timeouts.WriteTotalTimeoutConstant = 100;
  if (!SetCommTimeouts(handle, &timeouts)) {
    const auto err = windows_error("SetCommTimeouts failed");
    close_on_error();
    throw err;
  }

  SetupComm(handle, 4096, 4096);
  PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
  return std::make_shared<DmSerialBus>(static_cast<void*>(handle), port);
}

DmSerialBus::DmSerialBus(void* handle, std::string port)
    : handle_(handle), port_(std::move(port)) {}

DmSerialBus::~DmSerialBus() {
  try {
    shutdown();
  } catch (...) {
  }
}

void DmSerialBus::send(const CanFrame& frame) {
  const auto raw = DmSerialCodec::encode_tx(frame);
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ == nullptr) {
    throw std::runtime_error("serial port already closed");
  }

  std::size_t written = 0;
  while (written < raw.size()) {
    DWORD count = 0;
    const auto remaining = static_cast<DWORD>(raw.size() - written);
    if (!WriteFile(as_handle(handle_), raw.data() + written, remaining, &count, nullptr)) {
      throw windows_error("dm-serial write failed");
    }
    if (count == 0) {
      throw std::runtime_error("dm-serial write timed out");
    }
    written += static_cast<std::size_t>(count);
  }
}

std::optional<CanFrame> DmSerialBus::receive_for(std::chrono::milliseconds timeout) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ == nullptr) {
    throw std::runtime_error("serial port already closed");
  }

  const auto bounded_timeout = std::max(timeout, std::chrono::milliseconds::zero());
  const auto deadline = std::chrono::steady_clock::now() + bounded_timeout;
  for (;;) {
    if (const auto frame = codec_.try_parse_rx()) {
      return frame;
    }

    DWORD errors = 0;
    COMSTAT status{};
    if (!ClearCommError(as_handle(handle_), &errors, &status)) {
      throw windows_error("ClearCommError failed");
    }
    if (status.cbInQue > 0) {
      std::array<uint8_t, 256> buffer{};
      const DWORD requested = std::min<DWORD>(status.cbInQue, buffer.size());
      DWORD count = 0;
      if (!ReadFile(as_handle(handle_), buffer.data(), requested, &count, nullptr)) {
        throw windows_error("dm-serial read failed");
      }
      if (count > 0) {
        codec_.push_bytes(buffer.data(), static_cast<std::size_t>(count));
        continue;
      }
    }

    if (bounded_timeout == std::chrono::milliseconds::zero() ||
        std::chrono::steady_clock::now() >= deadline) {
      return std::nullopt;
    }
    Sleep(1);
  }
}

void DmSerialBus::shutdown() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ != nullptr) {
    FlushFileBuffers(as_handle(handle_));
    PurgeComm(as_handle(handle_), PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR);
    CloseHandle(as_handle(handle_));
    handle_ = nullptr;
  }
}

}  // namespace damiao

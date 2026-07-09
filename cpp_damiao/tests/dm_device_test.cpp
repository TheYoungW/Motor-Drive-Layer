#include <iostream>
#include <stdexcept>

#include "damiao/dm_device_bus.hpp"

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

template <typename Fn>
void require_throws(const char* label, const Fn& fn) {
  try {
    fn();
  } catch (const std::exception&) {
    return;
  }
  throw std::runtime_error(label);
}

}  // namespace

int main() {
  require(damiao::parse_dm_device_type("usb2canfd") == damiao::DmDeviceType::Usb2CanFd,
          "usb2canfd parse");
  require(damiao::parse_dm_device_type("usb2canfd-dual") ==
              damiao::DmDeviceType::Usb2CanFdDual,
          "dual parse");
  require(damiao::parse_dm_device_type("linkx4c") == damiao::DmDeviceType::LinkX4C,
          "linkx4c parse");

  require(damiao::parse_dm_channel(damiao::DmDeviceType::Usb2CanFd, "0") == 0,
          "single channel 0");
  require(damiao::parse_dm_channel(damiao::DmDeviceType::Usb2CanFdDual, "canfd2") == 1,
          "dual channel alias");
  require(damiao::parse_dm_channel(damiao::DmDeviceType::LinkX4C, "ch3") == 3,
          "linkx4c channel alias");

  require_throws("single channel rejects 1", [] {
    (void)damiao::parse_dm_channel(damiao::DmDeviceType::Usb2CanFd, "1");
  });
  require_throws("dual rejects 2", [] {
    (void)damiao::parse_dm_channel(damiao::DmDeviceType::Usb2CanFdDual, "2");
  });

  const auto path = damiao::resolve_dm_device_library_path();
  require(!path.empty(), "dm device library path is resolved");

  std::cout << "dm device tests passed\n";
  return 0;
}

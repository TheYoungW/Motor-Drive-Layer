# cpp_damiao

`cpp_damiao` is the configuration-agnostic C++17 Damiao driver and C ABI used by the Python package.

## Scope

- Damiao protocol frame packing and decoding.
- Generic `CanBus` abstraction.
- Runtime with background RX polling, per-motor state cache, feedback counters, configurable TX pacing, register acknowledgements, and lifecycle cleanup.
- Linux Damiao serial, SocketCAN, SocketCAN-FD, and optional DM_Device transports.
- Shared C ABI library for Python and other language bindings.
- Unit, codec, runtime, and ABI smoke tests without hardware.

The C++ library does not parse Python YAML and does not contain robot-specific ports, joint lists, IDs, models, or loop rates. C++ callers pass those values directly through constructors and methods.

## Build and test

```bash
cmake -S cpp_damiao -B cpp_damiao/build
cmake --build cpp_damiao/build -j
ctest --test-dir cpp_damiao/build --output-on-failure
```

The build produces `libmotor_abi.so` and `libdamiao_runtime.a` on Linux.

## Minimal C++ usage

```cpp
#include "damiao/dm_serial_bus.hpp"
#include "damiao/runtime.hpp"

int main() {
  auto bus = damiao::DmSerialBus::open("/dev/ttyACM0", 1000000);
  damiao::Controller controller(bus);
  controller.set_tx_gap(std::chrono::microseconds(120));

  auto motor = controller.add_damiao_motor(0x01, 0x201, "4340P");
  motor->request_feedback();
  controller.shutdown();
}
```

The values above are examples supplied by the caller. They are not defaults compiled into the driver.

## Generic feedback statistics

`MotorHandle::feedback_stats()` returns whether sensor feedback has been observed, the number of decoded sensor frames, and the age of the latest frame. Register replies and write acknowledgements do not increment the sensor counter.

The C ABI exposes the same information through `motor_handle_get_feedback_stats` and accepts caller-provided pacing through `motor_controller_set_tx_gap_us`.

Default C++ tests never open a real motor device.

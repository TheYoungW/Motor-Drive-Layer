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
  const auto state = motor->request_fresh_state(std::chrono::milliseconds(50));
  controller.shutdown();
}
```

The port, baud rate, IDs, and model above are examples supplied by the caller. The one-motor
example sets a 120 µs TX interval explicitly. Without an explicit or environment override, a
controller starts with no artificial TX delay and automatically applies a 120 µs minimum interval
between all outgoing frames when its second motor is added. Call `Controller::set_tx_gap()` after
adding motors to change the current value, or set `MOTOR_DRIVE_LAYER_TX_GAP_US` before constructing
the controller to override the automatic default. `enable_all()` and `disable_all()` also use a
separate 2 ms inter-motor delay by default, configurable through
`MOTOR_DRIVE_LAYER_BULK_OP_GAP_MS`.

`MotorHandle::request_feedback()` only transmits an asynchronous request, and
`Controller::poll_feedback_once()` only drains frames that have already arrived. Use
`MotorHandle::request_fresh_state(timeout)` when the caller needs one newer sample before
continuing. For multiple motors, `Controller::request_feedback_all(timeout)` sends every request
with the configured TX pacing and waits against one shared deadline; a timeout error lists every
motor ID that did not provide a fresh sample.

## Generic feedback statistics

`MotorHandle::feedback_stats()` returns whether sensor feedback has been observed, the number of decoded sensor frames, and the age of the latest frame. Register replies and write acknowledgements do not increment the sensor counter.

The C ABI exposes the same information through `motor_handle_get_feedback_stats` and accepts caller-provided pacing through `motor_controller_set_tx_gap_us`.

Default C++ tests never open a real motor device.

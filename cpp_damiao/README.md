# cpp_damiao

`cpp_damiao` is the internal layered C++17 Motor core used directly by the Yunyi product Runtime.

## Scope

- Damiao protocol frame packing and decoding.
- Small `CanBus` seam for deterministic unit tests and the production SocketCAN-FD backend.
- Runtime with background RX polling, per-motor state cache, feedback counters, configurable TX pacing, register acknowledgements, and lifecycle cleanup.
- Persistent multi-controller workers for parallel MIT and POS_VEL batch dispatch.
- Linux SocketCAN-FD+BRS transport only.
- Unit, codec and Runtime tests without hardware.

The C++ library does not parse product configuration. `articore_runtime/` supplies Yunyi's fixed
channels, joint list, IDs, models and policies through direct C++ construction.

## Build and test

```bash
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

The unified build produces only the public
`articore_runtime/libarticore_runtime.so` library on Linux. The static Motor
and Runtime-core libraries are internal implementation units.

## Minimal C++ usage

```cpp
#include "damiao/socketcan_fd_bus.hpp"
#include "damiao/runtime.hpp"

int main() {
  auto bus = damiao::SocketCanFdBus::open("can-left", true);
  damiao::Controller controller(bus);
  controller.set_tx_gap(std::chrono::microseconds(200));

  auto motor = controller.add_damiao_motor(0x01, 0x11, "4340P");
  const auto state = motor->request_fresh_state(std::chrono::milliseconds(50));
  controller.shutdown();
}
```

The port, baud rate, IDs, and model above are examples supplied by the caller. The one-motor
example sets a 200 µs TX interval explicitly. Without an explicit or environment override, a
controller starts with no artificial TX delay and automatically applies a 200 µs minimum interval
between all outgoing frames when its second motor is added. Call `Controller::set_tx_gap()` after
adding motors to change the current value, or set `MOTOR_DRIVE_LAYER_TX_GAP_US` before constructing
the controller to override the automatic default. `enable_all()` and `disable_all()` also use a
separate 2 ms inter-motor delay by default, configurable through
`MOTOR_DRIVE_LAYER_BULK_OP_GAP_MS`.

Linux SocketCAN-FD sockets use non-blocking writes with a 20 ms bounded wait when
the kernel transmit queue is full. A timeout raises a transport error, updates
`TransportHealth::send_errors`/`last_error`, and allows controller shutdown to continue. Set
`MOTOR_DRIVE_LAYER_SOCKETCAN_SEND_TIMEOUT_MS` before opening the bus to configure 1--60000 ms.

`MotorHandle::request_feedback()` only transmits an asynchronous request, and
`Controller::poll_feedback_once()` only drains frames that have already arrived. Use
`MotorHandle::request_fresh_state(timeout)` when the caller needs one newer sample before
continuing. For multiple motors, `Controller::request_feedback_all(timeout)` sends every request
with the configured TX pacing and waits against one shared deadline; a timeout error lists every
motor ID that did not provide a fresh sample.

## Generic feedback statistics

`MotorHandle::feedback_stats()` returns whether sensor feedback has been observed, the number of decoded sensor frames, and the age of the latest frame. Register replies and write acknowledgements do not increment the sensor counter.

The product Runtime reads the same cache directly without crossing another ABI.

## Parallel controller batches

Construct `damiao::ControllerGroup` with the Controllers that should participate. Its persistent
workers start every dispatch from one generation, retain command order within each Controller, and
wait for all Controllers before returning:

```cpp
damiao::ControllerGroup group({&ch0, &ch1});
group.send_pos_vel({
    {motor_ch0, 0.5f, 2.0f},
    {motor_ch1, -0.5f, 2.0f},
});
```

Controllers and motor handles must outlive the group. On failure, the call waits for all workers
and reports the controller index, endpoint, motor ID, and underlying error. Per-controller
SocketCAN-FD pacing can overlap.

Every `CanBus` exposes `TransportCapabilities`; `Controller::transport_capabilities()` returns the
active instance's transport name, canonical payload size, physical channel count,
CAN-FD, active CAN-FD BRS, parallel-batch, reconnect, process-session reuse, and hardware RX
timestamp flags. The legacy capability struct remains ABI-stable; new callers use the
size-versioned V2 query for `can_fd_brs`.
The pacing wrapper also records `TransportHealth`; `Controller::transport_health()` exposes live
connection state, TX/RX counters, activity ages, and transport errors.

Default C++ tests never open a real motor device.

# Motor-Drive-Layer

English | [简体中文](README.zh-CN.md)

Motor-Drive-Layer is an open-source Damiao motor driver for C++ and Python. The native C++ runtime owns protocol encoding, serial/CAN I/O, TX pacing, background feedback reception, and state caching. Python exposes the same driver through a small C ABI.

## Features

- Damiao MIT, position/velocity, velocity, and force/position control modes.
- Linux SocketCAN and SocketCAN-FD, cross-platform Damiao serial bridge, and optional DM_Device SDK transports.
- Damiao serial rates through 1,000,000 baud where supported by the host.
- Background feedback reception and per-motor state cache.
- Multi-motor controllers default to a configurable 120 µs minimum interval between outgoing frames.
- Register read/write helpers with acknowledgement and timeout handling.
- C ABI shared library and Python 3.10+ bindings.

## Architecture

```text
User Python / application code
        │
        │ editable values: port, baud, IDs, model, TX gap
        ▼
Python motor-drive-layer API ── ctypes call ── C ABI
                                      │
                                      ▼
                              C++ Damiao runtime
                                      │
               POSIX or Windows serial / SocketCAN / DM_Device
                                      │
                                      ▼
                              adapter and motors
```

C++ does not contain robot-specific ports, joint names, motor IDs, feedback IDs, or control frequencies.

## Safety

Motor control can cause unexpected motion and injury. Support the mechanism, keep an independent emergency stop available, begin with conservative limits, and verify IDs and control modes before enabling a motor.

## Build the C++ library

Requirements: a C++17 compiler and CMake 3.16+. SocketCAN additionally requires Linux development
headers.

```bash
cmake -S cpp_damiao -B cpp_damiao/build
cmake --build cpp_damiao/build -j
ctest --test-dir cpp_damiao/build --output-on-failure
```

The build produces `libmotor_abi.so` on Linux, `libmotor_abi.dylib` on macOS, or
`motor_abi.dll` on Windows, together with the static C++ runtime library.

## Supported platforms

PyPI wheels are built by GitHub Actions for Linux x86_64 and ARM64, macOS Intel and Apple Silicon,
and Windows x64. The Damiao serial transport is available on all of those platforms. SocketCAN and
SocketCAN-FD are Linux-only. The optional direct `dm-device` transport also requires the matching
vendor runtime and USB driver for the host platform.

Typical serial device names are `/dev/ttyACM0` on Linux, `/dev/cu.usbmodem*` on macOS, and `COM3`
on Windows.

## Install the Python package from source

Build the C++ ABI first, then install the Python package:

```bash
python3 -m pip install --upgrade pip
python3 -m pip install -e ./bindings/python
```

Install test dependencies with:

```bash
python3 -m pip install -e './bindings/python[test]'
```

Minimal Python usage:

```python
from motor_drive_layer import Controller

with Controller.from_dm_serial("/dev/ttyACM0", 1_000_000) as controller:
    motor = controller.add_damiao_motor(
        motor_id=0x01,
        feedback_id=0x201,
        model="4340P",
    )
    motor.request_feedback()
```

All values are supplied by the caller; the C++ driver does not assume these example IDs.

## TX pacing

A controller starts without an artificial TX delay while it has one motor. When a second motor
is added, the runtime applies a minimum 120 µs interval between all outgoing frames. Configure a
different value after adding the motors with `Controller.set_tx_gap_us()` in Python or
`Controller::set_tx_gap()` in C++; zero disables the delay. Setting
`MOTOR_DRIVE_LAYER_TX_GAP_US` before creating the controller overrides the automatic multi-motor
default.

`enable_all()` and `disable_all()` additionally wait 2 ms between motors by default. Set
`MOTOR_DRIVE_LAYER_BULK_OP_GAP_MS` before creating the controller to change that bulk-operation
interval. These are host-side minimum submission intervals, not hard real-time guarantees for
physical CAN bus timing.

## Fresh feedback

`Motor.request_feedback()` is asynchronous, `Motor.get_state()` reads the current cache, and
`Controller.poll_feedback_once()` only drains frames that have already arrived. None of those
methods waits for a newly requested frame. Use the synchronous helper when fresh data is required:

```python
state = motor.request_fresh_state(timeout_ms=50)
```

For multiple motors, request all feedback first and wait against one shared deadline:

```python
controller.request_feedback_all(timeout_ms=50)
states = [motor.get_state() for motor in motors]
```

The batch call records each motor's feedback counter, sends every request with the configured TX
pacing, and returns as soon as every counter advances. On timeout it raises `CallError` whose
message lists the missing motor IDs; it does not apply a separate full timeout to every motor.

## Python API reference

The wheel includes `py.typed` and complete `.pyi` declarations, so VS Code/Pylance, Pyright, and
Mypy can expose signatures, return types, and completion. Import the public objects from the
top-level `motor_drive_layer` package.

### Controller

| API | Behavior |
| --- | --- |
| `Controller(channel="can0")` | Open classic Linux SocketCAN. |
| `Controller.from_socketcanfd(channel="can0")` | Open Linux SocketCAN-FD. |
| `Controller.from_dm_serial(serial_port="/dev/ttyACM0", baud=1_000_000)` | Open a Damiao serial bridge. |
| `Controller.from_dm_device(dm_device_type="usb2canfd-dual", dm_channel="0")` | Open the optional DM_Device transport. |
| `add_damiao_motor(motor_id, feedback_id, model)` | Register a motor on the bus and return `Motor`. |
| `enable_all()` / `disable_all()` | Enable or disable every registered motor; these send hardware commands. |
| `request_feedback_all(timeout_ms=50)` | Request and wait for one fresh frame per motor against one shared timeout. |
| `poll_feedback_once()` | Non-blocking drain of frames that have already arrived. |
| `set_tx_gap_us(gap_us)` | Configure the minimum host-side interval between outgoing frames. |
| `shutdown()` | Attempt to disable every motor, stop polling, and close the bus. |
| `close_bus()` | Stop polling and close the bus without sending disable commands. |
| `close()` / `closed` | Free the native Controller handle; `close()` does not actively send disable commands. |

### Motor

| API | Behavior |
| --- | --- |
| `enable()` / `disable()` | Enable or disable this motor. |
| `clear_error()` | Send the clear-error command. |
| `set_zero_position()` | Set zero while the SDK believes the motor is disabled. |
| `ensure_mode(mode, timeout_ms=1000)` | Check, switch if needed, and verify the control mode. |
| `send_mit(pos, vel, kp, kd, tau)` | Send an MIT command. |
| `send_pos_vel(pos, vlim)` | Send a position/velocity command. |
| `send_vel(vel)` | Send a velocity command. |
| `send_force_pos(pos, vlim, ratio)` | Send a force/position command. |
| `request_feedback()` | Send a feedback request without waiting. |
| `request_fresh_state(timeout_ms=50)` | Request and wait for a fresh state from this motor. |
| `get_state()` | Read the C++ cache, returning `None` before the first feedback. |
| `get_feedback_stats()` | Return availability, update count, and cached-sample age. |
| `set_can_timeout_ms(timeout_ms)` | Write the Damiao CAN-timeout register. |
| `get_register_f32/u32(rid, timeout_ms=1000)` | Read a register using its declared type. |
| `write_register_f32/u32(rid, value)` | Write a register; the canonical C++ table rejects read-only or wrong-type operations. |
| `damiao_get_param_f32/u32(...)` / `damiao_write_param_f32/u32(...)` | Compatibility aliases using parameter-ID terminology. |
| `store_parameters()` | Persist parameters to the motor and potentially disable it first. |
| `close()` / `closed` | Free the native Motor handle without sending a disable command. |

Position, velocity, and torque use rad, rad/s, and Nm. `MotorState`, `FeedbackStats`, `Mode`,
`CallError`, and the register constants are also exported at package level.

### Lifetime

A `Motor` is a logical child of the `Controller` that created it and keeps that Python Controller
alive. Motor operations raise `CallError("motor controller is closed")` after the parent closes;
`motor.close()` remains available to free the handle. Prefer nested context managers:

```python
from motor_drive_layer import Controller

with Controller.from_dm_serial("/dev/ttyACM0", 1_000_000) as controller:
    with controller.add_damiao_motor(0x01, 0x201, "4340P") as motor:
        state = motor.request_fresh_state(timeout_ms=50)
```

Leaving the Motor context only frees its handle and does not disable hardware. Leaving the
Controller context calls `shutdown()`, which attempts to disable every motor before closing the bus.

## Python examples

The focused examples in `bindings/python/examples/` cover the common workflows:

| File | Purpose |
| --- | --- |
| `connection_test.py` | Disable one motor and verify fresh feedback over any supported transport. |
| `socketcan_control.py` | Control one motor over Linux SocketCAN in MIT mode. |
| `dm_serial_control.py` | Control one motor through a Damiao serial bridge in any supported control mode. |
| `dm_serial_pos_vel.py` | Send periodic position-velocity (PV) frames to seven motors through a Damiao serial bridge. |
| `multi_motor_control.py` | Control multiple motors over Linux SocketCAN. |
| `maintenance.py` | Clear errors, set the CAN timeout, optionally set zero, and read state. |
| `register_access.py` | Read registers; writes and persistent storage occur only when explicitly requested. |

Install the project first, then inspect a command before running it:

```bash
python3 bindings/python/examples/connection_test.py --help
python3 bindings/python/examples/socketcan_control.py --help
python3 bindings/python/examples/dm_serial_control.py --help
python3 bindings/python/examples/dm_serial_pos_vel.py --help
```

Motor control can cause sudden motion. Support the mechanism, prepare an independent emergency stop, and verify the channel, IDs, model, mode, and targets before enabling a motor. Maintenance and register writes can permanently change device settings; stay in read-only mode unless you know the register semantics.

## Linux SocketCAN setup

Source checkouts include three optional helpers. They configure Linux CAN network interfaces
and never enable or control a motor:

```bash
scripts/can_restart.sh can0        # classic CAN
scripts/canfd_restart.sh can0      # CAN-FD
scripts/canable_restart.sh can0    # CANable/candleLight (gs_usb)
```

They are not needed for `dm-serial` or `dm-device`. Pip-installed users can follow the
self-contained `ip link` commands printed by the CLI when an interface is not ready.

## Tests

No-hardware tests:

```bash
cmake --build cpp_damiao/build -j
ctest --test-dir cpp_damiao/build --output-on-failure
PYTHONPATH=bindings/python/src python3 -m pytest -q bindings/python/tests
```

Default CI does not open serial devices or enable motors.

## Repository layout

```text
cpp_damiao/                 C++ protocol, runtime, transports, C ABI, tests
bindings/python/            Python package, tests, and examples
third_party/dm_device/      Optional vendor runtime headers/libraries
scripts/                    Linux SocketCAN/CAN-FD interface setup helpers
.github/                    CI and issue templates
```

## Performance scope

The current hardware has demonstrated complete feedback counts at 500 Hz per motor on seven-motor serial buses. That establishes throughput, not a hard real-time deadline. USB scheduling, the host kernel, adapter firmware, and application scheduling can still produce millisecond-scale latency outliers.

## Contributing and security

See [CONTRIBUTING.md](CONTRIBUTING.md) before submitting changes. Report safety or security-sensitive motor-control issues according to [SECURITY.md](SECURITY.md) rather than publishing exploit details first.

## License

MIT. See [LICENSE](LICENSE).

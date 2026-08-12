# Motor-Drive-Layer

English | [简体中文](README.zh-CN.md)

Motor-Drive-Layer is the native C++ control foundation shared by Python, C++ and ROS 2 SDKs. Its generic motor layer owns Damiao protocol and transport behavior; the separately built Articore product runtime owns watchdog, safe-hold, fault and gripper policy without contaminating the generic API.

## Features

- Damiao MIT, position/velocity, velocity, and force/position control modes.
- Linux SocketCAN and SocketCAN-FD, cross-platform Damiao serial bridge, and optional DM_Device SDK transports.
- Damiao serial rates through 1,000,000 baud where supported by the host.
- Background feedback reception and per-motor state cache.
- Multi-motor controllers default to a configurable 120 µs minimum interval between outgoing frames.
- Register read/write helpers with acknowledgement and timeout handling.
- C ABI shared library and Python 3.10+ bindings.
- A separate Articore runtime ABI with a persistent safety and gripper worker.
- Current-position safe hold: fresh cached arm feedback is latched on entry; stale or faulted
  feedback escalates to `FAULT` instead of replaying a previous user target.

## Architecture

```text
Python SDK / C++ SDK / ROS 2
              │
              ▼
    libarticore_runtime
    product watchdog, safety and gripper policy
              │ stable function-table ABI
              ▼
         libmotor_abi
    generic controller and motor API
              │
              ▼
     C++ transport/protocol core
              │
              ▼
serial / SocketCAN / DM_Device / motors
```

The generic `motor/` layer does not contain robot-specific policy. Product concepts remain isolated
under `articore_runtime/`, with a one-way dependency on the stable motor ABI.

## Safety

Motor control can cause unexpected motion and injury. Support the mechanism, keep an independent emergency stop available, begin with conservative limits, and verify IDs and control modes before enabling a motor.

## Build the C++ library

Requirements: a C++17 compiler and CMake 3.16+. SocketCAN additionally requires Linux development
headers.

```bash
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

The build produces two public shared libraries: `libmotor_abi` for generic motor communication and
`libarticore_runtime` for product safety policy. Static core targets are internal build details.
Python exposes separate `abi_capabilities()` and `articore_runtime_capabilities()` queries so the
generic and product ABI surfaces cannot be confused.

## Supported platforms

PyPI wheels are built by GitHub Actions for Linux x86_64 and ARM64, macOS Intel and Apple Silicon,
and Windows x64. The Damiao serial transport is available on all of those platforms. SocketCAN and
SocketCAN-FD are Linux-only. The direct `dm-device` transport includes the matching redistributable
vendor runtime in every platform wheel. A host USB driver may still be required where the
operating system does not already provide one. DaMiao's current official macOS v1.1 runtime
declares macOS 26 as its minimum deployment target, so wheels containing that runtime carry the
matching `macosx_26_0` tag.

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

Models with replaceable end effectors can discover one fixed motor set per connection:

```python
from motor_drive_layer import Controller, MotorCandidate, PresencePolicy

controller = Controller.from_dm_serial("/dev/ttyACM0", 1_000_000)
try:
    discovered = controller.discover_damiao_motors(
        (
            MotorCandidate("joint1", 0x09, 0x19, "4310"),
            MotorCandidate(
                "gripper", 0x01, 0x11, "4340P", PresencePolicy.OPTIONAL
            ),
        ),
        timeout_ms=50,
        retries=1,
    )
finally:
    controller.close()  # close without sending enable, motion, or disable frames
```

`REQUIRED` absence fails with endpoint, role and CAN IDs. `OPTIONAL` absence returns
`NOT_INSTALLED`; `DISABLED` is not registered or probed. Only identity-matched fresh feedback can
produce `PRESENT`. A successful discovery freezes the active motor set until the Controller is
closed, so a motor that later drops offline cannot silently become `NOT_INSTALLED`.

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

## Parallel controller batches

`ControllerGroup` keeps one native worker per Controller and reuses those workers on every
dispatch. One call wakes all member controllers from the same dispatch generation, preserves the
command order within each controller, waits for every controller, and then returns. This avoids
creating or scheduling Python threads in a control loop and lets independent per-controller TX
pacing waits overlap.

```python
from motor_drive_layer import Controller, ControllerGroup

ch0 = Controller.from_dm_device(device="usb2canfd-dual", channel=0)
ch1 = Controller.from_dm_device(device="usb2canfd-dual", channel=1)
motors_ch0 = [ch0.add_damiao_motor(i, 0x200 + i, "4340P") for i in range(1, 8)]
motors_ch1 = [ch1.add_damiao_motor(i, 0x200 + i, "4340P") for i in range(9, 16)]
targets = [0.0] * (len(motors_ch0) + len(motors_ch1))
velocity_limit = 2.0

with ControllerGroup([ch0, ch1]) as group:
    batch = group.prepare_pos_vel(motors_ch0 + motors_ch1)
    batch.send(targets, velocity_limit)
```

`send_mit()` accepts `MitCommand` in the same way. A failed dispatch still waits for all member
controllers and raises `CallError` with the controller index, endpoint/channel, motor ID, and
underlying send error. Controllers and motors must remain open until the group is closed. Do not
mix individual motor sends with a group dispatch concurrently because their relative frame order
is unspecified. The shared DM_Device vendor call remains mutex-protected; only independent pacing
waits and other per-controller work overlap, so this is synchronized host dispatch rather than a
hard real-time simultaneous bus transmission guarantee.

For a fixed motor layout in a high-rate loop, `prepare_pos_vel()` and `prepare_mit()` retain the
validated motor pointers and reuse one preallocated Python/ctypes command array. This removes
per-cycle command dataclass and ctypes-array construction. The native ABI still validates and
partitions each call, so this is a reduced-allocation path rather than a zero-allocation or
hard-real-time guarantee.

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
| `Controller.from_dm_device(device="usb2canfd-dual", channel=0, bitrate=1_000_000, data_bitrate=5_000_000)` | Open original DM-USB2FDCAN firmware through the vendor DM_Device runtime; CH0 and CH1 are supported. Legacy positional/keyword arguments remain valid. |
| `add_damiao_motor(motor_id, feedback_id, model)` | Register a motor on the bus and return `Motor`. |
| `discover_damiao_motors(candidates, timeout_ms=50, retries=1)` | Probe Required/Optional/Disabled model candidates without enabling or moving them and freeze the present motor set for this connection. |
| `enable_all()` / `disable_all()` | Enable or disable every registered motor; these send hardware commands. |
| `request_feedback_all(timeout_ms=50)` | Request and wait for one fresh frame per motor against one shared timeout. Python internally uses the structured ABI and raises typed errors carrying the stable code, counts, and missing motor IDs. |
| `poll_feedback_once()` | Non-blocking drain of frames that have already arrived. |
| `set_tx_gap_us(gap_us)` | Configure the minimum host-side interval between outgoing frames. |
| `transport_capabilities()` | Query the active transport's CAN-FD, channel-count, parallel-batch, reconnect, process-session reuse, and hardware RX timestamp capabilities. |
| `transport_health()` | Read live connection/health flags, TX/RX frame and error counters, last activity ages, and the latest transport error. |
| `shutdown()` | Attempt to disable every motor, stop polling, and close the bus. |
| `close_bus()` | Stop polling and close the bus without sending disable commands. |
| `close()` / `closed` | Free the native Controller handle; `close()` does not actively send disable commands. |

Native callers that need stable feedback failure classification should use
`motor_controller_request_feedback_all_ex()`. It returns `MOTOR_OK`,
`MOTOR_ERROR_FEEDBACK_TIMEOUT`, `MOTOR_ERROR_FEEDBACK_INCOMPLETE`,
`MOTOR_ERROR_TRANSPORT`, or `MOTOR_ERROR_INVALID_ARGUMENT` and fills
`MotorFeedbackReport` plus a caller-owned missing-motor-ID buffer. A zero-capacity null ID buffer
is valid for count-only queries; `missing_count` always reports the complete number of missing
motors. The legacy `motor_controller_request_feedback_all()` remains 0/-1 compatible, and
`motor_last_error_message()` remains diagnostic text rather than a classification API. Check the
`structured_feedback_report` capability before resolving the additive entry point from an older
shared library.

Python intentionally exposes only `Controller.request_feedback_all()`, not an `_ex` method. It
uses the structured entry point internally and maps its result to `FeedbackTimeoutError`,
`IncompleteFeedbackError`, `FeedbackTransportError`, or `FeedbackMotorFaultError`. These remain
`CallError` subclasses for compatibility and expose `error_code`, `missing_motor_ids`, and the full
`FeedbackReport`. The Articore runtime callback uses the same structured signature, so safety
decisions never depend on parsing `motor_last_error_message()`.

The DM_Device backend does not require flashing SocketCAN firmware. Platform wheels include the
matching vendor runtime, so installing `motor-drive-layer` is sufficient. `MOTOR_DM_DEVICE_LIB`
can still override the packaged library for development, and the motor-layer installer/downloader
remains a recovery fallback. The loader probes both the
v1.0 (`damiao_*`/`device_*`) and v1.1 (`dmcan_*`) ABIs and reports missing symbols and dynamic
loader dependency errors explicitly. Linux x86_64 uses v1.0 with an isolated compatible C++
runtime; Linux ARM64 uses v1.1 with a private libusb 1.0.27 runtime because Ubuntu 22.04's
libusb 1.0.25 does not export `libusb_init_context`; Windows and macOS use v1.1. Each Controller
owns only its selected channel. v1.1 releases the shared physical USB handle after the last
Controller closes. Because the
official Linux v1.0 runtime cannot reliably reopen device index 0 in the same process after a full
device teardown, v1.0 instead closes each channel and all motor-layer threads/clients but retains
its legacy context, device handle, callbacks, and loaded library for reuse until process exit. The
operating system reclaims those retained vendor objects; calling the vendor destructor during
static process teardown can race libusb thread cleanup.

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

### ControllerGroup

| API | Behavior |
| --- | --- |
| `ControllerGroup(controllers)` | Create and retain one persistent native send worker per Controller. |
| `send_pos_vel(commands)` | Dispatch `PosVelCommand` values across their owning controllers and wait for all. |
| `send_mit(commands)` | Dispatch `MitCommand` values across their owning controllers and wait for all. |
| `prepare_pos_vel(motors)` | Create a reusable fixed-layout POS_VEL batch; scalar velocity limits are broadcast. |
| `prepare_mit(motors)` | Create a reusable fixed-layout MIT batch; all fields except position accept scalars or vectors. |
| `close()` / `closed` | Stop and join the native workers; does not close member controllers. |

### Feedback/reconnect stress diagnostic

`motor-drive-layer-stress` repeatedly opens the requested DM_Device channels, requests feedback,
closes them, and reopens them in the same process. It records latency, failures, file-descriptor
counts, and thread counts. The tool never enables a motor and never sends a control command:

```bash
motor-drive-layer-stress \
  --motor 0:0x09:0x19:4310 \
  --motor 1:0x0f:0x1f:4310 \
  --iterations 1000 --reconnect-cycles 10 --output stress.json
```

Repeat `--motor` for the real channel/motor/feedback/model mapping. The example IDs are illustrative.

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

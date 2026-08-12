# motor-drive-layer

Python bindings for the Motor-Drive-Layer native C++ Damiao motor driver.

The package loads the two bundled native ABI libraries through Python `ctypes` and exposes
SocketCAN, SocketCAN-FD, Damiao serial bridge, and optional DM_Device transports.
`libmotor_abi` is the generic motor layer; `libarticore_runtime` is the separately versioned
product safety runtime consumed by Articore SDKs.
Use `articore_runtime_abi_version()` and `articore_runtime_capabilities()` to inspect that product
runtime independently from `abi_version()` and `abi_capabilities()`. Runtime ABI 1.5 adds explicit
`STREAMING` and `HOLD_UNTIL_REPLACED` direct-command lifetimes so physical motion duration is not
confused with caller update cadence. Runtime ABI 1.4 adds native
atomic enable with parallel CH0/CH1 activation, immediate current-position hold, parallel feedback
confirmation, and all-motor rollback. Runtime ABI 1.3 exposes the
native latest-value joint mailbox and single active trajectory engine in addition to
`current_position_hold`. Runtime ABI 1.5 also advertises `protective_fault_hold`: a transient
feedback miss keeps the current output, while persistent loss stops trajectories and holds every
still-controllable motor/channel without automatically torque-disabling unrelated hardware.
SDK bindings use `articore_runtime_create_ex()` and pass the motor enable callbacks explicitly;
the two packaged native libraries remain independently loadable on Linux, Windows, and macOS.
Published wheels cover Linux x86_64/ARM64, macOS Intel/Apple Silicon, and Windows x64. The serial
transport is cross-platform; SocketCAN transports remain Linux-only.

DM-USB2FDCAN Dual works with its original `dual_app` firmware through
`Controller.from_dm_device(device="usb2canfd-dual", channel=0, bitrate=1_000_000,
data_bitrate=5_000_000)`. CH0 and CH1 are independently selectable, old positional arguments remain
compatible, and the loader probes both vendor v1.0 and v1.1 ABIs. The matching vendor runtime is
included in every platform wheel; no separate download command is required. Set
`MOTOR_DM_DEVICE_LIB` only when intentionally overriding the packaged runtime.
The current official macOS v1.1 dylib requires macOS 26, and the packaged wheel is tagged
accordingly rather than claiming compatibility with an older system.
Linux ARM64 wheels include a private libusb 1.0.27 build so the vendor v1.1 runtime can use
`libusb_init_context` on Ubuntu 22.04 without replacing the host's libusb 1.0.25 package.
Linux x86_64 wheels contain both v1.0 and a complete v1.1 environment using the same private
libusb together with a private compatible libstdc++. v1.0 remains the default on x86_64 because
the official v1.1 runtime batches receive callbacks at roughly 100 ms with `dual_app v1.0.0.3`;
set `MOTOR_DM_DEVICE_ABI=v1.1` to opt in without installing any host runtime.
On final Controller close, v1.1 fully tears down its device session. The Linux v1.0 runtime instead
closes the selected channels and motor-layer resources but retains its legacy context/device until
process exit, allowing a later `Controller.from_dm_device(...)` call to reconnect reliably in the
same Python process. The operating system reclaims those retained vendor objects at exit because
calling the vendor destructor during static teardown can race libusb thread cleanup.

Use `Motor.request_fresh_state(timeout_ms=50)` when the caller must wait for a newly requested
feedback frame. For multiple motors, `Controller.request_feedback_all(timeout_ms=50)` sends every
request with the configured TX pacing and waits against one shared deadline; a timeout reports the
missing motor IDs. The public method internally uses the structured native entry point and raises
typed `FeedbackTimeoutError`, `IncompleteFeedbackError`, `FeedbackTransportError`, or
`FeedbackMotorFaultError` exceptions. Each remains a `CallError` subclass and carries a stable
`error_code` plus a `FeedbackReport` containing timeout, expected/received/missing counts, and
missing motor IDs; no caller needs to use an `_ex` Python method. The lower-level
`request_feedback()`, `get_state()`, and
`poll_feedback_once()` methods remain non-blocking asynchronous/cache operations.

For synchronized multi-channel control, `ControllerGroup([ch0, ch1])` creates one persistent
native worker per Controller. `send_pos_vel([...])` and `send_mit([...])` dispatch typed
`PosVelCommand` or `MitCommand` values across their owning controllers, overlap independent TX
pacing waits, and return after every controller finishes. Errors retain the controller/channel,
motor ID, and native reason.
For fixed high-rate layouts, `prepare_pos_vel(motors)` and `prepare_mit(motors)` reuse one validated
ctypes command array across calls. `Controller.transport_capabilities()` reports the active
transport's CAN-FD, channel, parallel-send, reconnect, session-reuse, and timestamp capabilities.
`Controller.transport_health()` reports live connection state, TX/RX counters, errors, and last
activity ages without adding robot-product policy to the transport layer.

The `motor-drive-layer-stress` command provides a feedback-only DM_Device load/reconnect test and
reports latency plus Linux file-descriptor/thread counts. It never enables or commands motors.
The CLI `scan` command similarly uses one Controller and one batch
`discover_damiao_motors()` call for its entire candidate range, then uses
`close_bus()` plus `close()` so scanning never emits disable frames. The repository's
`scripts/test_dm_device_scan_lifecycle.py` runs 100 scan-process/CH0+CH1-reader-process cycles.

The wheel includes `py.typed` and `.pyi` declarations for editor completion and static type
checking. The main public APIs are:

- `Controller(...)`, `from_socketcanfd(...)`, `from_dm_serial(...)`, and `from_dm_device(...)`.
- `Controller.add_damiao_motor(...)`, `enable_all()`, `disable_all()`,
  `request_feedback_all()`, `set_tx_gap_us()`, `transport_capabilities()`, `shutdown()`, and
  `close_bus()`.
- `ControllerGroup(...)`, `send_pos_vel(...)`, and `send_mit(...)` for persistent native
  multi-controller dispatch, plus reusable prepared batches for fixed layouts.
- `Motor.enable()`, `disable()`, `ensure_mode()`, all four control-mode send methods,
  fresh/cached feedback methods, typed register access, parameter aliases, and
  `store_parameters()`.
- `MitCommand`, `PosVelCommand`, `MotorState`, `FeedbackStats`, `Mode`, register
  metadata/constants, and SDK exception classes.

`Motor` is a logical child of its creating `Controller`. After the parent closes, motor operations
raise `CallError`; `motor.close()` can still release the handle. Both classes support context
managers. Leaving a Motor context only frees its handle, while leaving a Controller context attempts
to disable all motors before closing the bus.

See the [project README](https://github.com/TheYoungW/Motor-Drive-Layer)
for build instructions, architecture, configuration, safety guidance, and examples.

This package is distributed under the MIT License. The optional DM_Device vendor
runtime is distributed separately and may have its own license terms.

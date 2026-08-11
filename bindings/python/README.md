# motor-drive-layer

Python bindings for the Motor-Drive-Layer native C++ Damiao motor driver.

The package loads the bundled C ABI library through Python `ctypes` and exposes
SocketCAN, SocketCAN-FD, Damiao serial bridge, and optional DM_Device transports.
Published wheels cover Linux x86_64/ARM64, macOS Intel/Apple Silicon, and Windows x64. The serial
transport is cross-platform; SocketCAN transports remain Linux-only.

DM-USB2FDCAN Dual works with its original `dual_app` firmware through
`Controller.from_dm_device(device="usb2canfd-dual", channel=0, bitrate=1_000_000,
data_bitrate=5_000_000)`. CH0 and CH1 are independently selectable, old positional arguments remain
compatible, and the loader probes both vendor v1.0 and v1.1 ABIs. Install the separate vendor
runtime with `motor-drive-layer-install-dm-device --download` or set `MOTOR_DM_DEVICE_LIB`.
On final Controller close, v1.1 fully tears down its device session. The Linux v1.0 runtime instead
closes the selected channels and motor-layer resources but retains its legacy context/device until
process exit, allowing a later `Controller.from_dm_device(...)` call to reconnect reliably in the
same Python process. A process-scope cleanup closes and destroys those retained vendor objects at
normal process exit.

Use `Motor.request_fresh_state(timeout_ms=50)` when the caller must wait for a newly requested
feedback frame. For multiple motors, `Controller.request_feedback_all(timeout_ms=50)` sends every
request with the configured TX pacing and waits against one shared deadline; a timeout reports the
missing motor IDs. The lower-level `request_feedback()`, `get_state()`, and
`poll_feedback_once()` methods remain non-blocking asynchronous/cache operations.

For synchronized multi-channel control, `ControllerGroup([ch0, ch1])` creates one persistent
native worker per Controller. `send_pos_vel([...])` and `send_mit([...])` dispatch typed
`PosVelCommand` or `MitCommand` values across their owning controllers, overlap independent TX
pacing waits, and return after every controller finishes. Errors retain the controller/channel,
motor ID, and native reason.
For fixed high-rate layouts, `prepare_pos_vel(motors)` and `prepare_mit(motors)` reuse one validated
ctypes command array across calls. `Controller.transport_capabilities()` reports the active
transport's CAN-FD, channel, parallel-send, reconnect, session-reuse, and timestamp capabilities.

The `motor-drive-layer-stress` command provides a feedback-only DM_Device load/reconnect test and
reports latency plus Linux file-descriptor/thread counts. It never enables or commands motors.

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

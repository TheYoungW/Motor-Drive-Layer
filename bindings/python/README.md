# motor-drive-layer

Python bindings for the Motor-Drive-Layer native C++ Damiao motor driver.

The package loads the bundled C ABI library through Python `ctypes` and exposes
SocketCAN, SocketCAN-FD, Damiao serial bridge, and optional DM_Device transports.
Published wheels cover Linux x86_64/ARM64, macOS Intel/Apple Silicon, and Windows x64. The serial
transport is cross-platform; SocketCAN transports remain Linux-only.

Use `Motor.request_fresh_state(timeout_ms=50)` when the caller must wait for a newly requested
feedback frame. For multiple motors, `Controller.request_feedback_all(timeout_ms=50)` sends every
request with the configured TX pacing and waits against one shared deadline; a timeout reports the
missing motor IDs. The lower-level `request_feedback()`, `get_state()`, and
`poll_feedback_once()` methods remain non-blocking asynchronous/cache operations.

The wheel includes `py.typed` and `.pyi` declarations for editor completion and static type
checking. The main public APIs are:

- `Controller(...)`, `from_socketcanfd(...)`, `from_dm_serial(...)`, and `from_dm_device(...)`.
- `Controller.add_damiao_motor(...)`, `enable_all()`, `disable_all()`,
  `request_feedback_all()`, `set_tx_gap_us()`, `shutdown()`, and `close_bus()`.
- `Motor.enable()`, `disable()`, `ensure_mode()`, all four control-mode send methods,
  fresh/cached feedback methods, typed register access, parameter aliases, and
  `store_parameters()`.
- `MotorState`, `FeedbackStats`, `Mode`, register metadata/constants, and SDK exception classes.

`Motor` is a logical child of its creating `Controller`. After the parent closes, motor operations
raise `CallError`; `motor.close()` can still release the handle. Both classes support context
managers. Leaving a Motor context only frees its handle, while leaving a Controller context attempts
to disable all motors before closing the bus.

See the [project README](https://github.com/TheYoungW/Motor-Drive-Layer)
for build instructions, architecture, configuration, safety guidance, and examples.

This package is distributed under the MIT License. The optional DM_Device vendor
runtime is distributed separately and may have its own license terms.

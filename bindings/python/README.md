# motor-drive-layer

Python bindings for the Motor-Drive-Layer native C++ Damiao motor driver.

The package loads the bundled C ABI library through Python `ctypes` and exposes
SocketCAN, SocketCAN-FD, Damiao serial bridge, and optional DM_Device transports.

Use `Motor.request_fresh_state(timeout_ms=50)` when the caller must wait for a newly requested
feedback frame. For multiple motors, `Controller.request_feedback_all(timeout_ms=50)` sends every
request with the configured TX pacing and waits against one shared deadline; a timeout reports the
missing motor IDs. The lower-level `request_feedback()`, `get_state()`, and
`poll_feedback_once()` methods remain non-blocking asynchronous/cache operations.

See the [project README](https://github.com/TheYoungW/Motor-Drive-Layer)
for build instructions, architecture, configuration, safety guidance, and examples.

This package is distributed under the MIT License. The optional DM_Device vendor
runtime is distributed separately and may have its own license terms.

# motorbridge

Python bindings for the Motor-Drive-Layer native C++ Damiao motor driver.

The package loads the bundled C ABI library through Python `ctypes` and exposes
SocketCAN, SocketCAN-FD, Damiao serial bridge, and optional DM_Device transports.
The base API has no YAML dependency; install the `hardware` extra only when using
the YAML-configured hardware benchmark.

See the [project README](https://github.com/TheYoungW/Motor-Drive-Layer),
[architecture](https://github.com/TheYoungW/Motor-Drive-Layer/blob/main/docs/architecture.md),
[configuration](https://github.com/TheYoungW/Motor-Drive-Layer/blob/main/docs/configuration.md),
and [hardware testing](https://github.com/TheYoungW/Motor-Drive-Layer/blob/main/docs/hardware-testing.md)
for build instructions, safety guidance, and examples.

This package is distributed under the MIT License. The optional DM_Device vendor
runtime is distributed separately and may have its own license terms.

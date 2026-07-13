# Motor-Drive-Layer

English | [简体中文](README.zh-CN.md)

Motor-Drive-Layer is an open-source Damiao motor driver for C++ and Python. The native C++ runtime owns protocol encoding, serial/CAN I/O, TX pacing, background feedback reception, and state caching. Python exposes the same driver through a small C ABI and adds optional user-facing YAML configuration and safe hardware diagnostics.

## Features

- Damiao MIT, position/velocity, velocity, and force/position control modes.
- Linux SocketCAN, SocketCAN-FD, Damiao serial bridge, and optional DM_Device SDK transports.
- Damiao serial rates through 1,000,000 baud where supported by the host.
- Background feedback reception and per-motor state cache.
- Configurable multi-motor TX pacing; 120 us is the provided hardware example, not a compiled constant.
- Register read/write helpers with acknowledgement and timeout handling.
- C ABI shared library and Python 3.10+ bindings.
- Pure-Python, YAML-configured multi-controller hardware benchmark.

## Architecture

```text
User Python / application code
        │
        │ editable values: port, baud, IDs, model, TX gap
        ▼
Python motorbridge API ── ctypes call ── C ABI
                                      │
                                      ▼
                              C++ Damiao runtime
                                      │
                   POSIX serial / SocketCAN / DM_Device
                                      │
                                      ▼
                              adapter and motors
```

YAML belongs only to the optional Python hardware tool. C++ neither parses YAML nor contains robot-specific ports, joint names, motor IDs, feedback IDs, or control frequencies.

See [architecture](docs/architecture.md), [configuration](docs/configuration.md), and [hardware testing](docs/hardware-testing.md).

## Safety

Motor control can cause unexpected motion and injury. Support the mechanism, keep an independent emergency stop available, begin with conservative limits, and verify IDs and control modes before enabling a motor.

The included hardware benchmark never enables motors. It disables every configured motor, verifies disabled feedback, sends zero-position/zero-velocity-limit control frames only while disabled, and disables again during cleanup. Review the script and your device behavior before using it on other hardware or firmware.

## Build the C++ library

Requirements: a C++17 compiler, CMake 3.16+, and Linux development headers.

```bash
cmake -S cpp_damiao -B cpp_damiao/build
cmake --build cpp_damiao/build -j
ctest --test-dir cpp_damiao/build --output-on-failure
```

The build produces `cpp_damiao/build/libmotor_abi.so` and the static C++ runtime library.

## Install the Python package from source

Build the C++ ABI first, then install the Python package:

```bash
python3 -m pip install --upgrade pip
python3 -m pip install -e ./bindings/python
```

Install optional YAML hardware tooling and test dependencies with:

```bash
python3 -m pip install -e './bindings/python[hardware,test]'
```

Minimal Python usage does not require YAML:

```python
from motorbridge import Controller

with Controller.from_dm_serial("/dev/ttyACM0", 1_000_000) as controller:
    motor = controller.add_damiao_motor(
        motor_id=0x01,
        feedback_id=0x201,
        model="4340P",
    )
    motor.request_feedback()
```

All values are supplied by the caller; the C++ driver does not assume these example IDs.

## Python examples

The focused examples in `bindings/python/examples/` cover the common workflows:

| File | Purpose |
| --- | --- |
| `socketcan_control.py` | Control one motor over Linux SocketCAN in MIT mode. |
| `dm_serial_control.py` | Control one motor through a Damiao serial bridge in any supported control mode. |
| `multi_motor_control.py` | Control multiple motors over Linux SocketCAN. |
| `maintenance.py` | Clear errors, set the CAN timeout, optionally set zero, and read state. |
| `register_access.py` | Read registers; writes and persistent storage occur only when explicitly requested. |

Install the project first, then inspect a command before running it:

```bash
python3 bindings/python/examples/socketcan_control.py --help
python3 bindings/python/examples/dm_serial_control.py --help
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

## Python hardware configuration

Copy the editable example before changing it:

```bash
motorbridge-hardware-test --write-example my_robot.yaml
```

The example uses `/dev/ttyACM0`, `/dev/ttyACM5`, 1,000,000 baud, a 120 us TX gap, and feedback IDs `0x201` through `0x207`. Change them to match the actual device configuration.

Validate YAML without opening any hardware:

```bash
motorbridge-hardware-test --config my_robot.yaml --validate-only
```

Run the safe disabled communication benchmark:

```bash
motorbridge-hardware-test --config my_robot.yaml
```

The result reports Python loop timing, command submission timing, exact feedback counter deltas, current feedback age, motor status, and temperatures. It does not claim that scheduler timing is physical CAN round-trip latency.

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
bindings/python/            Python package, tests, examples, YAML hardware tool
third_party/dm_device/      Optional vendor runtime headers/libraries
scripts/                    Linux SocketCAN/CAN-FD interface setup helpers
docs/                       Architecture, configuration, hardware-test documentation
.github/                    CI and issue templates
```

## Performance scope

The current hardware has demonstrated complete feedback counts at 500 Hz per motor on seven-motor serial buses. That establishes throughput, not a hard real-time deadline. USB scheduling, the host kernel, adapter firmware, and application scheduling can still produce millisecond-scale latency outliers.

## Contributing and security

See [CONTRIBUTING.md](CONTRIBUTING.md) before submitting changes. Report safety or security-sensitive motor-control issues according to [SECURITY.md](SECURITY.md) rather than publishing exploit details first.

## License

MIT. See [LICENSE](LICENSE).

# Motor-Drive-Layer

Motor-Drive-Layer is a Damiao-focused motor control layer for C++ and Python
projects. It keeps the Damiao motor control path, removes unused motor vendor
implementations, and adds a native C++ ABI path that can replace the original
Rust shared library for downstream applications.

## What remains

- `motor_core`: shared bus, transport, model, and error abstractions.
- `motor_vendors/damiao`: Damiao protocol, controller, motor model support, and
  register access.
- `motor_cli`: Damiao-only Rust CLI.
- `motor_abi`: Damiao-only C ABI used by the Python binding.
- `bindings/python`: Python SDK and Damiao-only CLI wrapper.
- `cpp_damiao`: native C++ Damiao SDK components with no Rust dependency.

If your application is C++ and you do not want Rust in the product, start with
[`cpp_damiao`](cpp_damiao/README.md).

## Quick commands

```bash
cargo run -p motor_cli -- --vendor damiao --mode scan --start-id 1 --end-id 16
cargo run -p motor_cli -- --vendor damiao --channel can0 --model 4340P \
  --motor-id 0x01 --feedback-id 0x11 --mode mit \
  --pos 0 --vel 0 --kp 2 --kd 1 --tau 0 --loop 200 --dt-ms 20
```

Python:

```bash
PYTHONPATH=bindings/python/src python3 -m motorbridge.cli --help
```

## Transports

- `socketcan` / `auto`: classic SocketCAN.
- `socketcanfd`: CAN-FD transport path.
- `dm-serial`: Damiao serial bridge.
- `dm-device`: Damiao DM_Device SDK adapters (`usb2canfd`,
  `usb2canfd-dual`, `linkx4c`).

## License

MIT. Keep the original copyright and license notice when reusing this code.

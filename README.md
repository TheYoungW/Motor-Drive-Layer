# Motor-Drive-Layer

Motor-Drive-Layer is a Damiao-focused motor control layer for C++ and Python
projects. It keeps the Damiao motor control path, removes unused motor vendor
implementations, and uses a native C++ ABI shared library for downstream
applications.

## What remains

- `cpp_damiao`: native C++ Damiao protocol, runtime, transports, tests, and C ABI.
- `bindings/python`: Python SDK that keeps the existing motorbridge API shape
  while loading the C++ ABI shared library.
- `third_party/dm_device`: optional DaMiao DM_Device runtime libraries.

This repository is C++ and Python only.

## Quick commands

```bash
cmake -S cpp_damiao -B cpp_damiao/build
cmake --build cpp_damiao/build
ctest --test-dir cpp_damiao/build --output-on-failure
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

# Contributing

Contributions are welcome for the Damiao C++ core, C ABI, Python binding, tests, and documentation.

## Development setup

```bash
cmake -S cpp_damiao -B cpp_damiao/build
cmake --build cpp_damiao/build -j
python3 -m pip install -e './bindings/python[hardware,test]'
```

## Required checks

```bash
ctest --test-dir cpp_damiao/build --output-on-failure
PYTHONPATH=bindings/python/src python3 -m pytest -q bindings/python/tests
python3 -m motorbridge.hardware_test \
  --config bindings/python/src/motorbridge/configs/damiao_dual_arm.yaml \
  --validate-only
```

Default automated tests must not require external repositories, attached hardware, root privileges, or motor power.

Changes to the C ABI must be additive unless a breaking release is explicitly planned. Update `bindings/api_surface.json`, C++ tests, Python ctypes declarations, Python tests, and user documentation together.

## Hardware changes

Describe the adapter, motor model, firmware, baud/CAN rate, IDs, power state, and safety setup used for validation. Never add a CI test that enables physical motors. Redact device serial numbers and private site information from logs.

## Pull requests

Keep changes focused, include regression tests, explain observable behavior, and identify any hardware behavior that remains unverified.

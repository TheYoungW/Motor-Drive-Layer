# Contributing

Contributions are welcome for the Damiao C++ core, product Runtime, native ABIs, tests, packaging,
and documentation. Python product bindings belong in Articore-SDK, not this repository.

## Development setup

```bash
cmake -S . -B build
cmake --build build -j
```

## Required checks

```bash
ctest --test-dir build --output-on-failure
```

Default automated tests must not require external repositories, attached hardware, root privileges, or motor power.

Changes to a C ABI must be additive unless a breaking release is explicitly planned. Treat the
installed ABI headers, exported capability bits and native ABI tests as the authoritative contract;
update them and the documentation together, then coordinate the matching ctypes declaration and
tests in Articore-SDK.

## Hardware changes

Describe the adapter, motor model, firmware, baud/CAN rate, IDs, power state, and safety setup used for validation. Never add a CI test that enables physical motors. Redact device serial numbers and private site information from logs.

## Pull requests

Keep changes focused, include regression tests, explain observable behavior, and identify any hardware behavior that remains unverified.

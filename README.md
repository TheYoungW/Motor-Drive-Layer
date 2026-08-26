# Motor Drive Layer

Native C++ control stack for the Yunyi dual-arm product.

The `main` branch supports one production path: Linux SocketCAN-FD+BRS on
`can-left` and `can-right`. The earlier multi-transport/Damiao-SDK architecture
is retained separately on the `legacy-multi-can-damiao-sdk` branch.

## Architecture

```text
Articore SDK
    -> Yunyi Runtime C ABI
        -> 500 Hz product Runtime and safety state
            -> native C++ Motor core
                -> left/right Controller workers
                    -> SocketCAN-FD+BRS frames

can-left/right receive threads
    -> Motor feedback cache
        -> Runtime state/health snapshot
            -> Articore SDK
```

The public wheel contains only `libarticore_runtime.so`. Runtime calls the Motor
core directly in the same C++ process; there is no intermediate Motor C ABI and
no Python implementation in this repository.

## Current contract

- package version: `0.15.0`
- Runtime ABI: `6.0` / `0x00060000`
- ABI matching: exact
- product: `yunyi_v1_0`
- transports: `can-left`, `can-right`
- arm order: left J1..J7, right J1..J7
- optional paired grippers selected at Runtime creation

The SDK creates the product with
`articore_runtime_create_yunyi(mode, with_grippers, &runtime)`. Runtime owns the
Motor mapping, controllers, limits, models, TCP offsets, workers and resource
lifetime.

See [the Runtime reference](articore_runtime/README.md) for the current API.

## Build and test

```bash
cmake -S . -B builds/dev -DCMAKE_BUILD_TYPE=Release
cmake --build builds/dev -j
ctest --test-dir builds/dev --output-on-failure
```

Hardware diagnostics are built explicitly and are never registered in CTest.
They may enable or move the robot and must only be run with a guarded workspace.

## Packaging

```bash
python -m build packaging/pypi --wheel
```

Publishing is a separate release action. Building and testing local changes do
not require uploading to PyPI.

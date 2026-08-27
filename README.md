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

- package version: `0.21.0`
- Runtime ABI: `11.2` / `0x000B0002`

Runtime health includes a product-order snapshot for every installed Motor,
with role, CAN ID, feedback age, status, issue bits, and a scope that separates
isolated Motor feedback loss from left/right/both-channel receive stalls.
- ABI matching: exact
- product: `yunyi_v1_0`
- transports: `can-left`, `can-right`
- arm order: left J1..J7, right J1..J7
- optional paired grippers selected at Runtime creation

The SDK creates the product with
`articore_runtime_create_yunyi(mode, with_grippers, &runtime)`. Runtime owns the
Motor mapping, controllers, limits, models, TCP offsets, workers and resource
lifetime.

Ordinary PV and `set_pose()` share one reference generator. The 500 Hz Runtime
loop is the safety/feedback and active-command refresh clock, not a
point-to-point planning frequency. Each command maps `speed=0..100` directly
onto `0..2 rad/s`; there is no second
persistent maximum-speed cap. Persistent maximum acceleration uses physical
units with `0.01` resolution, accepts `0.01..8.00 rad/s^2`, and defaults to
`6.00 rad/s^2`. It does not affect MIT. Linear/Circular first solve a sparse
5 mm / 0.035 rad Cartesian IK path, then apply one global quintic time law to
generate fixed 2 ms ordinary-PV references. Thus `duration_s=3` produces 1500
segments and 1501 points over about three reference seconds; physical arrival
may be later due to PV limits and feedback stability. Intermediate points are
continuous tracking references and only the final endpoint commands braking.
The 500 Hz loop advances exactly one point per cycle, never in bulk or at
variable planned intervals. PV still uses speed 50 and the configured maximum
acceleration; numerical waypoint derivatives do not reject the plan.
Automatic approach also uses ordinary PV. Linear, Circular and `set_pose()` require PV product mode;
ordinary MIT and MIT joint trajectories are unchanged.

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

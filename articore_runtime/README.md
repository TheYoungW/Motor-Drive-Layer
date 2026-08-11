# Articore native safety runtime

This directory contains single- and dual-arm Articore product policy. It is a separate native
library in the Motor-Drive-Layer repository and depends on the generic motor C ABI without adding
Yunyi or Articore concepts to `libmotor_abi`.

The runtime owns one persistent worker thread. A complete PV or MIT command for all active channels
is validated and sent through `ControllerGroup`; only a successful full-group send refreshes the
native `steady_clock` watchdog. The worker independently performs command timeout handling,
feedback and transport-health checks, safe-hold transmission, fault latching, linked disable, and
disable confirmation while Python is blocked or has stopped running.

PV safe hold preserves the last successful target with a dedicated low velocity limit. MIT safe
hold preserves position, zeros velocity and feedforward torque, and substitutes product safety
Kp/Kd. The same persistent worker owns each configured product gripper's
`IDLE -> MOVING -> CONTACT -> HOLDING -> OVERLOAD_RETREAT` state machine. It maps public 0..1000
opening targets to motor position, ramps closing motion with the normal MIT gains, detects contact
from torque plus a position-motion window and target error, then switches to a low-gain hold with
zero feedforward torque. Sustained overload produces a rate-limited bounded retreat. No Python
gripper control loop is involved in the native dual-arm path.

In `FAULT`, arms are always linked-disabled while the product setting chooses whether grippers
keep the last safe target or are disabled. A failed gripper hold falls back to individual motor
disable attempts. Recovery first disables every held gripper and confirms fresh disabled feedback
before returning to `READY`. If no complete arm safety target exists, command failure enters
`FAULT` instead of sending an empty arm hold.

`runtime_abi.h` is the stable boundary used by `arx_d_can.sdk.native_safety`. The optional generic
motor-drive-layer transport-health callback is used when available; the wrapper remains compatible
with the motor function-table ABI and falls back to motor feedback health when that callback is not
present. ABI version and capability functions make mismatches fail during runtime creation rather
than during motion.

Build and run the native tests with:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

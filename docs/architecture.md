# Architecture

Motor-Drive-Layer separates mechanism from user configuration.

## C++ core

The C++ layer implements Damiao frame packing, transport I/O, receive parsing, TX pacing, state caching, register transactions, and lifecycle safety. Public constructors and methods accept values from callers; the core contains no robot-specific serial ports, joint lists, motor IDs, feedback IDs, or loop rates.

The C ABI is an additive boundary used by Python and other languages. Calls on the same handle are serialized. The C++ background receiver runs independently of Python code and updates a per-motor state cache and feedback counter.

## Python binding

The base Python package is a typed wrapper around the C ABI. It does not require YAML and can be configured directly from Python values.

The optional hardware tool adds YAML as a user interface. Its loader validates input and passes every value through the normal public Python API. Removing the YAML tool would not change the C++ library.

## Data path

```text
Python target values
  -> C ABI call
  -> C++ protocol encoder
  -> paced transport write
  -> USB/CAN adapter
  -> Damiao motor
  -> motor feedback
  -> C++ background receiver
  -> decoded state/cache/counter
  -> Python get_state/get_feedback_stats
```

The feedback counter measures received sensor frames. Register replies and register-write acknowledgements do not increment it.

## Timing terminology

- `tx_gap_us` is the minimum delay requested between adjacent command submissions on one controller.
- Python tick lateness measures when the Python worker began relative to its schedule.
- Cycle submission time measures how long Python took to submit all configured motor commands.
- Feedback age measures how long ago the latest sensor frame was received.

None of these values alone is physical CAN round-trip latency. Hard real-time claims require a real-time kernel or a dedicated real-time controller plus end-to-end validation.

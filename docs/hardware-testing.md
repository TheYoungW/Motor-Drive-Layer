# Safe Python Hardware Testing

The hardware benchmark exercises the installed Python API and native C++ driver.

## Safety behavior

The tool:

1. opens each configured controller;
2. applies the YAML-provided TX gap;
3. registers YAML-provided motors;
4. sends disable commands;
5. requests initial feedback and aborts unless every state is disabled;
6. starts one Python worker per controller;
7. sends zero-position, zero-velocity-limit position/velocity frames while disabled;
8. waits for feedback to drain;
9. captures feedback counters, state, and temperatures;
10. disables and closes every controller in cleanup, including error paths.

It never calls `enable()` or `enable_all()`.

## Run

```bash
python3 -m pip install -e './bindings/python[hardware]'
motorbridge-hardware-test --write-example my_robot.yaml
motorbridge-hardware-test --config my_robot.yaml --validate-only
motorbridge-hardware-test --config my_robot.yaml
```

Use a stable `/dev/serial/by-id/` or udev-created device name for deployment instead of relying permanently on `ttyACM` enumeration order.

## Output

Each controller reports:

- completed tick count;
- ticks that began more than 100 us late;
- maximum Python tick lateness;
- maximum time to submit one controller's motor commands.

Each motor reports:

- configured motor and feedback IDs;
- received versus expected feedback counter delta;
- feedback percentage;
- latest feedback age;
- status, MOS temperature, and rotor temperature.

A non-zero exit status is returned if configuration/hardware setup fails or any motor receives fewer feedback frames than expected.

## Interpretation

Complete feedback counts demonstrate that the software/adapter/motor pipeline handled the requested load. They do not prove every frame met a fixed deadline. Python scheduler timing and latest-feedback age are diagnostic measurements, not exact per-frame command-to-feedback round-trip latency.

For motion testing, create a separate application with explicit limits, watchdogs, workspace restraints, and operator approval. This benchmark intentionally contains no enable option.

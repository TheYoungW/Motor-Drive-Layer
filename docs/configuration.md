# Python Hardware Configuration

YAML is optional and is read only by `motorbridge.hardware_config`. C++ does not link a YAML library and does not know the YAML schema.

Create an editable copy of the packaged example:

```bash
motorbridge-hardware-test --write-example my_robot.yaml
```

## Schema

```yaml
schema_version: 1
benchmark:
  frequency_hz: 500
  duration_s: 10
  feedback_settle_ms: 300
controllers:
  - name: arm_a
    port: /dev/ttyACM0
    baud: 1000000
    tx_gap_us: 120
    joints:
      - name: joint1
        motor_id: 0x01
        feedback_id: 0x201
        model: 4340P
```

All values are user-editable. In particular, `0x201–0x207` are verified example feedback IDs, not driver requirements. A device configured for `0x11–0x17` can use those values in its own YAML.

IDs may be YAML integers or quoted hexadecimal strings:

```yaml
feedback_id: 0x201
feedback_id: "0x201"
```

Quote numeric-only model names such as `"4310"` for clarity; the loader also accepts an unquoted numeric model and converts it to text.

## Validation

The loader rejects:

- unsupported schema versions;
- missing or empty required values;
- duplicate controller names or serial ports;
- duplicate joint names, motor IDs, or feedback IDs within one controller;
- IDs outside the standard 11-bit CAN range;
- non-positive baud, frequency, or duration;
- negative TX gaps or settling times.

Run validation without loading the native ABI or opening hardware:

```bash
python3 -m motorbridge.hardware_test \
  --config bindings/python/src/motorbridge/configs/damiao_dual_arm.yaml \
  --validate-only
```

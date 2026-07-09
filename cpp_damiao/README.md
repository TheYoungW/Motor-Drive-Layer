# cpp_damiao

Native C++ Damiao SDK components with no Rust dependency.

This directory is separate from `bindings/cpp`, which wraps the Rust C ABI.
Use this directory when your C++ project must not link Rust-built libraries.

## Current Scope

- Header-only Damiao protocol frame packing and decoding.
- Header-only `CanBus` abstraction.
- Header-only `Motor` helper that sends Damiao frames through any `CanBus`
  implementation.
- C++ `motor_abi` shared library compatible with the Python `motorbridge`
  `ctypes` loader.
- C++ runtime with background RX polling, per-motor state cache, TX pacing,
  register ack/cache, and bulk operation pacing.
- Concrete transports: `dm-serial`, Linux `socketcan`, Linux `socketcanfd`,
  and DaMiao `dm-device` via the vendored SDK shim.
- Unit and smoke tests with fake buses and codec-level coverage.

## Build Tests

```bash
cmake -S cpp_damiao -B cpp_damiao/build
cmake --build cpp_damiao/build
ctest --test-dir cpp_damiao/build --output-on-failure
```

The build produces `cpp_damiao/build/libmotor_abi.dylib` on macOS or
`cpp_damiao/build/libmotor_abi.so` on Linux. The Python loader checks this path
before Rust `target/release`, so local Python tests use the C++ ABI.

## Python/reBot Compatibility

No-hardware compatibility check:

```bash
PYTHONPATH=bindings/python/src \
python3 cpp_damiao/tools/verify_python_compat.py \
  --rebot-root /Users/young/works/reBotArm_control_py \
  --skip-hardware-probes
```

Dry-run reBot hardware smoke:

```bash
PYTHONPATH=bindings/python/src \
python3 cpp_damiao/tools/rebot_hardware_smoke.py \
  --rebot-config /Users/young/works/reBotArm_control_py/config/rebotarm_dm.yaml \
  --dry-run
```

Real hardware smoke over the Damiao serial bridge:

```bash
PYTHONPATH=bindings/python/src \
python3 cpp_damiao/tools/rebot_hardware_smoke.py \
  --rebot-config /Users/young/works/reBotArm_control_py/config/rebotarm_dm.yaml \
  --transport dm-serial \
  --channel /dev/ttyACM0 \
  --run-hardware
```

Real hardware smoke over DM_Device:

```bash
PYTHONPATH=bindings/python/src \
python3 cpp_damiao/tools/rebot_hardware_smoke.py \
  --rebot-config /Users/young/works/reBotArm_control_py/config/rebotarm_dm.yaml \
  --transport dm-device \
  --dm-device-type usb2canfd-dual \
  --dm-channel 0 \
  --run-hardware
```

The smoke script does not enable motors unless `--enable` is passed. By
default it registers motors, requests feedback, polls cached state, and exits.

## Minimal Usage

```cpp
#include "damiao/motor.hpp"

class MyBus : public damiao::CanBus {
 public:
  void send(const damiao::CanFrame& frame) override {
    // Send frame.id and frame.data through your CAN adapter.
  }

  std::optional<damiao::CanFrame> receive_for(std::chrono::milliseconds timeout) override {
    // Read one CAN frame from your adapter.
    return std::nullopt;
  }
};

int main() {
  MyBus bus;
  damiao::Motor motor(bus, 0x01, 0x11, damiao::model_limits("4340P"));
  motor.enable();
  motor.send_mit(0.0f, 0.0f, 2.0f, 1.0f, 0.0f);
}
```

## Remaining Hardware Work

The C++ ABI transport paths build and pass no-hardware tests locally. Final
confidence still requires running the smoke commands on the target machine with
the actual adapter and reBot arm connected.

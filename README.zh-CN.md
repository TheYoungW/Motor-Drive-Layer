# Motor-Drive-Layer

Motor-Drive-Layer 是面向达妙电机的 C++ / Python 电机控制层。当前项目只保留达妙电机控制链路，移除了不需要的其他电机厂商实现，并加入了可替代原 Rust 动态库的原生 C++ ABI 路径。

## 保留内容

- `motor_core`：共享总线、传输、模型和错误抽象。
- `motor_vendors/damiao`：达妙协议、控制器、电机模型和寄存器访问。
- `motor_cli`：只支持达妙的 Rust CLI。
- `motor_abi`：只支持达妙的 C ABI，供 Python binding 使用。
- `bindings/python`：Python SDK 和达妙专用 CLI 包装。
- `cpp_damiao`：不依赖 Rust 的原生 C++ 达妙 SDK 组件。

如果你的最终项目是 C++，并且产品里不想带 Rust，从
[`cpp_damiao`](cpp_damiao/README.md) 开始。

## 常用命令

```bash
cargo run -p motor_cli -- --vendor damiao --mode scan --start-id 1 --end-id 16
cargo run -p motor_cli -- --vendor damiao --channel can0 --model 4340P \
  --motor-id 0x01 --feedback-id 0x11 --mode mit \
  --pos 0 --vel 0 --kp 2 --kd 1 --tau 0 --loop 200 --dt-ms 20
```

Python：

```bash
PYTHONPATH=bindings/python/src python3 -m motorbridge.cli --help
```

## 传输方式

- `socketcan` / `auto`：经典 SocketCAN。
- `socketcanfd`：CAN-FD 传输路径。
- `dm-serial`：达妙串口桥。
- `dm-device`：达妙 DM_Device SDK 适配器，包括 `usb2canfd`、
  `usb2canfd-dual`、`linkx4c`。

## 许可证

MIT。复用代码时保留原始版权和许可证声明。

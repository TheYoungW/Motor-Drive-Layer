# Motor-Drive-Layer

Motor-Drive-Layer 是面向达妙电机的 C++ / Python 电机控制层。当前项目只保留达妙电机控制链路，移除了不需要的其他电机厂商实现，并使用原生 C++ ABI 动态库作为 Python 和下游项目的底层实现。

## 保留内容

- `cpp_damiao`：原生 C++ 达妙协议、运行时、传输层、测试和 C ABI。
- `bindings/python`：保持现有 motorbridge Python API 形态，并加载 C++ ABI 动态库。
- `third_party/dm_device`：可选的达妙 DM_Device 运行库。

这个仓库现在只保留 C++ 和 Python。

## 常用命令

```bash
cmake -S cpp_damiao -B cpp_damiao/build
cmake --build cpp_damiao/build
ctest --test-dir cpp_damiao/build --output-on-failure
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

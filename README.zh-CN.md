# Motor-Drive-Layer

[English](README.md) | 简体中文

Motor-Drive-Layer 是面向达妙电机的开源 C++ / Python 驱动。原生 C++ 运行时负责协议编码、串口/CAN 收发、发送节流、后台反馈接收和状态缓存；Python 通过稳定的 C ABI 调用同一套驱动。

## 功能

- 达妙 MIT、位置速度、速度和力位混合控制模式。
- Linux SocketCAN、SocketCAN-FD、达妙串口桥和可选 DM_Device SDK。
- 主机支持时，达妙串口支持最高 1,000,000 波特率。
- 后台反馈接收和每电机状态缓存。
- 可配置的多电机发送节流；示例使用 120 us，但它不是 C++ 硬编码常量。
- 带 ACK、重试和超时的寄存器读写。
- C ABI 动态库和 Python 3.10+ 接口。

## 架构边界

```text
用户 Python / 应用代码
        │
        │ 可编辑参数：串口、波特率、ID、型号、发送间隔
        ▼
Python motor-drive-layer API ── ctypes 调用 ── C ABI
                                      │
                                      ▼
                               C++ 达妙运行时
                                      │
                    POSIX 串口 / SocketCAN / DM_Device
                                      │
                                      ▼
                                适配器与电机
```

C++ 不保存机器人专用的串口、关节名、电机 ID、反馈 ID 或控制频率。

## 安全说明

电机控制可能造成意外运动和人身伤害。测试前应支撑机械结构、准备独立急停、使用保守限制，并在使能前确认 ID 和控制模式。

## 构建 C++

需要 C++17 编译器、CMake 3.16+ 和 Linux 开发环境：

```bash
cmake -S cpp_damiao -B cpp_damiao/build
cmake --build cpp_damiao/build -j
ctest --test-dir cpp_damiao/build --output-on-failure
```

构建结果包含 `cpp_damiao/build/libmotor_abi.so` 和静态 C++ 运行库。

## 从源码安装 Python

先构建 C++ ABI，然后安装 Python 包：

```bash
python3 -m pip install --upgrade pip
python3 -m pip install -e ./bindings/python
```

需要测试依赖时：

```bash
python3 -m pip install -e './bindings/python[test]'
```

最小 Python 使用：

```python
from motor_drive_layer import Controller

with Controller.from_dm_serial("/dev/ttyACM0", 1_000_000) as controller:
    motor = controller.add_damiao_motor(
        motor_id=0x01,
        feedback_id=0x201,
        model="4340P",
    )
    motor.request_feedback()
```

这些值全部由调用者提供；C++ 不会假设示例 ID。

## Python 示例

`bindings/python/examples/` 中保留了六个用途明确的示例：

| 文件 | 用途 |
| --- | --- |
| `connection_test.py` | 失能一台电机，并通过任一受支持传输验证反馈是否正常。 |
| `socketcan_control.py` | 通过 Linux SocketCAN 控制一台电机，演示 MIT 模式。 |
| `dm_serial_control.py` | 通过达妙串口桥控制一台电机，支持 MIT、位置速度、速度和力位混合模式。 |
| `multi_motor_control.py` | 通过 Linux SocketCAN 控制多台电机。 |
| `maintenance.py` | 清除错误、设置 CAN 超时、可选设置零位并读取状态。 |
| `register_access.py` | 读取寄存器；只有明确传入写参数时才会写入或保存。 |

先安装项目，再使用 `--help` 查看参数：

```bash
python3 bindings/python/examples/connection_test.py --help
python3 bindings/python/examples/socketcan_control.py --help
python3 bindings/python/examples/dm_serial_control.py --help
```

电机控制可能造成突然运动。运行控制示例前，请支撑机械结构、准备独立急停，并核对通道、电机 ID、反馈 ID、型号、控制模式和目标值。维护及寄存器写入可能永久改变设备参数；不确定寄存器含义时请只读，不要使用写入或保存参数。

## Linux SocketCAN 配置

源码仓库提供三个可选辅助脚本，它们只配置 Linux CAN 网络接口，不会使能或控制电机：

```bash
scripts/can_restart.sh can0        # 经典 CAN
scripts/canfd_restart.sh can0      # CAN-FD
scripts/canable_restart.sh can0    # CANable/candleLight（gs_usb）
```

使用 `dm-serial` 或 `dm-device` 时不需要这些脚本。通过 pip 安装的用户可以按照 CLI 错误提示中的完整 `ip link` 命令配置接口。

## 测试

无硬件测试：

```bash
cmake --build cpp_damiao/build -j
ctest --test-dir cpp_damiao/build --output-on-failure
PYTHONPATH=bindings/python/src python3 -m pytest -q bindings/python/tests
```

默认 CI 不会打开串口，也不会使能真实电机。

## 项目结构

```text
cpp_damiao/                 C++协议、运行时、传输层、C ABI和测试
bindings/python/            Python包、测试和示例
third_party/dm_device/      可选厂商运行库
scripts/                    Linux SocketCAN/CAN-FD接口配置工具
.github/                    CI和Issue模板
```

## 性能边界

当前硬件已验证七电机串口在每电机500 Hz下反馈计数完整。这证明的是吞吐能力，不代表硬实时保证。USB调度、普通Linux内核、适配器固件和应用调度仍可能产生毫秒级长尾延迟。

## 贡献与安全

提交修改前请阅读[CONTRIBUTING.md](CONTRIBUTING.md)。涉及机械臂安全或通信漏洞的问题请按照[SECURITY.md](SECURITY.md)私下报告，不要先公开可直接复现危险动作的细节。

## 许可证

MIT，见[LICENSE](LICENSE)。

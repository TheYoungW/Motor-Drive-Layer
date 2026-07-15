# Motor-Drive-Layer

[English](README.md) | 简体中文

Motor-Drive-Layer 是面向达妙电机的开源 C++ / Python 驱动。原生 C++ 运行时负责协议编码、串口/CAN 收发、发送节流、后台反馈接收和状态缓存；Python 通过稳定的 C ABI 调用同一套驱动。

## 功能

- 达妙 MIT、位置速度、速度和力位混合控制模式。
- Linux SocketCAN、SocketCAN-FD、达妙串口桥和可选 DM_Device SDK。
- 主机支持时，达妙串口支持最高 1,000,000 波特率。
- 后台反馈接收和每电机状态缓存。
- 多电机 Controller 默认在输出帧之间保持可配置的最小 120 µs 间隔。
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

## 发送间隔

Controller 只有一台电机时，运行时不额外延迟发送。添加第二台电机后，运行时会在所有输出帧之间保持最小
120 µs 间隔。添加完电机后，可以通过 Python 的 `Controller.set_tx_gap_us()` 或 C++ 的
`Controller::set_tx_gap()` 修改该值；设为零可关闭延迟。在创建 Controller 前设置
`MOTOR_DRIVE_LAYER_TX_GAP_US` 可覆盖自动的多电机默认值。

`enable_all()` 和 `disable_all()` 还会默认在电机之间额外等待 2 ms。在创建 Controller 前设置
`MOTOR_DRIVE_LAYER_BULK_OP_GAP_MS` 可修改这个批量操作间隔。这些数值是主机侧的最小提交间隔，不是对 CAN 总线物理时序的硬实时保证。

## 新鲜反馈

`Motor.request_feedback()` 只异步发送请求，`Motor.get_state()` 只读取当前缓存，
`Controller.poll_feedback_once()` 也只会排空已经到达的帧；这三个方法都不会等待刚请求的反馈。需要新数据时请使用同步接口：

```python
state = motor.request_fresh_state(timeout_ms=50)
```

多电机场景使用一个共享的截止时间请求全部反馈：

```python
controller.request_feedback_all(timeout_ms=50)
states = [motor.get_state() for motor in motors]
```

批量接口会先记录每台电机的反馈计数，再按已配置的发送间隔发出全部请求。所有计数都增加后立即返回；超时时会抛出
`CallError` 并在消息中列出缺失的电机 ID，不会对每台电机重复等待一个完整超时。

## Python API 参考

安装包包含 `py.typed` 和完整 `.pyi` 声明；VS Code/Pylance、Pyright 和 Mypy 可以直接显示参数、返回值和自动补全。公开对象应从 `motor_drive_layer` 顶层导入。

### Controller

| 接口 | 作用 |
| --- | --- |
| `Controller(channel="can0")` | 打开经典 Linux SocketCAN。 |
| `Controller.from_socketcanfd(channel="can0")` | 打开 Linux SocketCAN-FD。 |
| `Controller.from_dm_serial(serial_port="/dev/ttyACM0", baud=1_000_000)` | 打开达妙串口桥。 |
| `Controller.from_dm_device(dm_device_type="usb2canfd-dual", dm_channel="0")` | 打开可选 DM_Device 传输。 |
| `add_damiao_motor(motor_id, feedback_id, model)` | 在总线上注册电机并返回 `Motor`。 |
| `enable_all()` / `disable_all()` | 依次使能或失能所有已注册电机；会发送硬件命令。 |
| `request_feedback_all(timeout_ms=50)` | 请求并等待所有电机各收到一帧新反馈，共享一个总超时。 |
| `poll_feedback_once()` | 非阻塞排空当前已经到达的帧。 |
| `set_tx_gap_us(gap_us)` | 设置相邻输出帧的最小主机提交间隔。 |
| `shutdown()` | 先尝试失能全部电机，再停止接收线程并关闭总线。 |
| `close_bus()` | 不发送失能命令，直接停止接收并关闭总线。 |
| `close()` / `closed` | 释放原生 Controller 句柄；`close()` 不主动发送失能命令。 |

### Motor

| 接口 | 作用 |
| --- | --- |
| `enable()` / `disable()` | 使能或失能这一台电机。 |
| `clear_error()` | 发送清除错误命令。 |
| `set_zero_position()` | 在 SDK 认为电机已失能时设置零位。 |
| `ensure_mode(mode, timeout_ms=1000)` | 检查并在需要时切换控制模式，然后验证结果。 |
| `send_mit(pos, vel, kp, kd, tau)` | 发送 MIT 控制命令。 |
| `send_pos_vel(pos, vlim)` | 发送位置/速度命令。 |
| `send_vel(vel)` | 发送速度命令。 |
| `send_force_pos(pos, vlim, ratio)` | 发送力位混合命令。 |
| `request_feedback()` | 只发送反馈请求，不等待返回。 |
| `request_fresh_state(timeout_ms=50)` | 请求并等待这一台电机的新反馈，返回 `MotorState`。 |
| `get_state()` | 读取 C++ 当前缓存；没有反馈时返回 `None`。 |
| `get_feedback_stats()` | 返回是否收到过反馈、更新计数和缓存年龄。 |
| `set_can_timeout_ms(timeout_ms)` | 写入达妙 CAN 超时寄存器。 |
| `get_register_f32/u32(rid, timeout_ms=1000)` | 按声明的数据类型读取寄存器。 |
| `write_register_f32/u32(rid, value)` | 写寄存器；C++ 权限表会拒绝只读或类型错误的操作。 |
| `damiao_get_param_f32/u32(...)` / `damiao_write_param_f32/u32(...)` | 兼容参数 ID 命名的寄存器访问接口。 |
| `store_parameters()` | 将参数持久保存到电机，可能先发送失能命令。 |
| `close()` / `closed` | 释放原生 Motor 句柄；不发送电机失能命令。 |

位置、速度和力矩统一使用 rad、rad/s 和 Nm。`MotorState`、`FeedbackStats`、`Mode`、`CallError` 以及寄存器常量也从包顶层公开。

### 生命周期

`Motor` 是创建它的 `Controller` 的逻辑子对象，并在 Python 中持有父 Controller 引用。Controller 关闭后，Motor 的硬件操作会抛出 `CallError("motor controller is closed")`；此时仍可调用 `motor.close()` 释放句柄。推荐使用嵌套上下文管理器：

```python
from motor_drive_layer import Controller

with Controller.from_dm_serial("/dev/ttyACM0", 1_000_000) as controller:
    with controller.add_damiao_motor(0x01, 0x201, "4340P") as motor:
        state = motor.request_fresh_state(timeout_ms=50)
```

退出 Motor 上下文只释放句柄，不会失能电机；退出 Controller 上下文会调用 `shutdown()`，尝试失能全部电机后关闭总线。

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

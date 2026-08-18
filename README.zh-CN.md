# Motor-Drive-Layer

[English](README.md) | 简体中文

Motor-Drive-Layer 是供 Python、C++ 和 ROS 2 SDK 共用的原生 C++ 控制底座。通用电机层负责达妙协议与 Transport；独立构建的 Articore 产品运行时负责看门狗、安全保持、故障和夹爪策略，不污染通用接口。

## 功能

- 达妙 MIT、位置速度、速度和力位混合控制模式。
- Linux SocketCAN、SocketCAN-FD、跨平台达妙串口桥和可选 DM_Device SDK。
- 主机支持时，达妙串口支持最高 1,000,000 波特率。
- 后台反馈接收和每电机状态缓存。
- 多电机 Controller 默认在输出帧之间保持可配置的最小 200 µs 间隔。
- 带 ACK、重试和超时的寄存器读写。
- C ABI 动态库和 Python 3.10+ 接口。
- 独立的 Articore runtime ABI 和常驻安全/夹爪工作线程。
- 同一原生 Runtime 的正式薄绑定：强类型 Python `ArticoreRuntime`、可安装的 C++17 RAII API，
  以及供 ROS 2 和其他语言使用的稳定 C ABI；绑定层不重新实现控制和安全逻辑。
- 显式区分需持续续期的流式命令和保持到被替换的单次目标，慢速运动不会被误判为上层失联。
- 原生 Runtime 对输出命令独立执行机械硬限位、产品软限位和逐关节动态制动区。
- 按次夹爪命令只包含开合度、归一化速度和标定力矩等级；Runtime 对张开与闭合统一生成速度斜坡，
  堵转窗口、回退距离和持续时间仍是不可由普通用户覆盖的产品安全参数。
- Runtime ABI 2.2 内置 `yunyi_gripper_v1` 产品 profile；SDK 只需在 connect 前绑定 profile ID，
  不再在 Python 配置中复制夹爪映射、增益、阈值、时序、回退、十档力控表和故障策略。
- Runtime ABI 2.4 将 `connect()` 定义为结构化反馈屏障：成功返回时，每个已配置关节和已安装夹爪
  都已有新鲜缓存；失败时返回稳定分类、逐通道统计和逐电机配置 CAN ID，不再依赖解析日志字符串。
- 普通 PV/MIT 位置命令使用容量为一的最新值槽位，由原生固定频率线程限步推进；新目标原子替换
  旧终点，不建立 FIFO 队列。
- 保护性故障保持：单次反馈缺失时机械臂和夹爪继续当前输出并计数；连续缺失会停止普通运动，
  但健康电机和可用通道继续保持，不会被底层自动联动失能。
- 确定性的 Runtime 失能/关闭事务：先排空旧控制流量，再并行失能并确认双通道，只有未确认
  的电机才会定向重发一次失能命令。

## 架构边界

```text
Python SDK / C++ SDK / ROS 2
              │
              ▼
    libarticore_runtime
    产品看门狗、安全和夹爪策略
              │ 稳定函数表 ABI
              ▼
         libmotor_abi
    通用 Controller 与 Motor API
              │
              ▼
      C++ Transport/协议核心
              │
              ▼
串口 / SocketCAN / DM_Device / 电机
```

通用 `motor/` 层不包含机器人产品策略；产品概念隔离在 `articore_runtime/`，并且只能
单向依赖稳定的 motor ABI。

原生动态库是唯一行为来源。产品 SDK 只定义机器人名称、电机/通道映射、方向、限位、URDF、
IK 和动力学，不再复制 ctypes 声明、看门狗、安全状态机、固定频率控制循环、夹爪状态机或
原生句柄生命周期规则。

### 正式语言绑定

Python 直接从 `motor_drive_layer` 导入 `ArticoreRuntime` 和强类型配置/报告对象。私有
`_runtime_abi` 模块统一管理 ctypes，公开封装会保证 Runtime 停止和释放前
ControllerGroup、Controller、Motor 句柄始终有效。

C++17 使用标准 CMake package：

```cmake
find_package(MotorDriveLayer CONFIG REQUIRED)
target_link_libraries(robot_driver PRIVATE motorbridge::articore_runtime_cpp)
```

包含 `<articore/runtime.hpp>` 并使用不可复制、可移动的 `articore::Runtime` RAII 对象；
`articore/runtime_abi.h` 继续作为 ROS 2 和其他语言的稳定中立边界。

RK3588 可运行 `scripts/build_aarch64_runtime.sh`，直接生成并安装 aarch64 `.so`、头文件和
CMake package；使用板端 sysroot 时设置 `MOTOR_AARCH64_SYSROOT`。发布 CI 同时生成原生
Linux aarch64 artifact，并可提供 aarch64 Python wheel。
产品适配边界和生命周期顺序见 [Native Runtime binding contract](docs/language-bindings.md)。

## 安全说明

电机控制可能造成意外运动和人身伤害。测试前应支撑机械结构、准备独立急停、使用保守限制，并在使能前确认 ID 和控制模式。

## 构建 C++

需要 C++17 编译器和 CMake 3.16+；SocketCAN 还需要 Linux 开发头文件：

```bash
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

构建结果包含两个正式动态库：通用通信使用 `libmotor_abi`，产品安全策略使用
`libarticore_runtime`。静态 core target 只是内部构建细节。

Python 可分别调用 `abi_capabilities()` 与 `articore_runtime_capabilities()` 查询通用电机层
和产品运行时能力，避免把两套 ABI 的职责混在同一份能力列表中。

## 支持平台

PyPI 正式 wheel 仅通过 GitHub Actions 为 Linux x86_64/ARM64 构建。SocketCAN 和
SocketCAN-FD 仅支持 Linux；可选的 `dm-device` 直连方式在每个 Linux wheel 中包含匹配的
厂商运行库。其他平台仍可尝试从源码构建，但不属于正式 PyPI wheel 矩阵。

Linux 常见串口名称为 `/dev/ttyACM0`。

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

可更换末端的产品模型可以在每次连接时探测并冻结本次活动电机集合：

```python
from motor_drive_layer import Controller, MotorCandidate, PresencePolicy

controller = Controller.from_dm_serial("/dev/ttyACM0", 1_000_000)
try:
    discovered = controller.discover_damiao_motors(
        (
            MotorCandidate("joint1", 0x09, 0x19, "4310"),
            MotorCandidate(
                "gripper", 0x01, 0x11, "4340P", PresencePolicy.OPTIONAL
            ),
        ),
        timeout_ms=50,
        retries=1,
    )
finally:
    controller.close()  # 不发送使能、运动或失能帧，直接释放连接
```

`REQUIRED` 缺失会报告端点、角色和 CAN ID；`OPTIONAL` 缺失返回 `NOT_INSTALLED`；
`DISABLED` 不注册也不探测。只有身份匹配且校验有效的新鲜反馈才能确认 `PRESENT`。
探测成功后，本次 Controller 的活动电机集合不再变化，因此运行中掉线不会被静默降级成
`NOT_INSTALLED`。

## 发送间隔

Controller 只有一台电机时，运行时不额外延迟发送。添加第二台电机后，运行时会在所有输出帧之间保持最小
200 µs 间隔。添加完电机后，可以通过 Python 的 `Controller.set_tx_gap_us()` 或 C++ 的
`Controller::set_tx_gap()` 修改该值；设为零可关闭延迟。在创建 Controller 前设置
`MOTOR_DRIVE_LAYER_TX_GAP_US` 可覆盖自动的多电机默认值。

`enable_all()` 和 `disable_all()` 还会默认在电机之间额外等待 2 ms。在创建 Controller 前设置
`MOTOR_DRIVE_LAYER_BULK_OP_GAP_MS` 可修改这个批量操作间隔。这些数值是主机侧的最小提交间隔，不是对 CAN 总线物理时序的硬实时保证。

## 多 Controller 并行批量发送

`ControllerGroup` 为每个 Controller 常驻一个原生工作线程，并在每次发送时复用。一次调用会用同一批次代数唤醒全部 Controller，保持每个 Controller 内部的命令顺序，等待所有 Controller 完成后统一返回。控制循环不再每周期创建或调度 Python 线程，不同 Controller 的发送间隔等待可以重叠。

```python
from motor_drive_layer import Controller, ControllerGroup

ch0 = Controller.from_dm_device(device="usb2canfd-dual", channel=0)
ch1 = Controller.from_dm_device(device="usb2canfd-dual", channel=1)
motors_ch0 = [ch0.add_damiao_motor(i, 0x200 + i, "4340P") for i in range(1, 8)]
motors_ch1 = [ch1.add_damiao_motor(i, 0x200 + i, "4340P") for i in range(9, 16)]
targets = [0.0] * (len(motors_ch0) + len(motors_ch1))
velocity_limit = 2.0

with ControllerGroup([ch0, ch1]) as group:
    batch = group.prepare_pos_vel(motors_ch0 + motors_ch1)
    batch.send(targets, velocity_limit)
```

`send_mit()` 同样接收 `MitCommand`。某一路失败时，批次仍会等待全部 Controller 结束，然后通过 `CallError` 返回 Controller 索引、端点/通道、电机 ID 和底层错误。Controller 和 Motor 必须保持打开，直到 Group 关闭；不要把单电机发送与 Group 发送并发混用，因为两者之间的帧顺序没有定义。DM_Device 厂商调用仍由共享互斥锁保护，重叠的是各通道独立的发送间隔等待及其他通道内工作，因此这是主机侧同步调度，不承诺两条物理总线硬实时同时发帧。

固定电机布局的高频循环可使用 `prepare_pos_vel()` 和 `prepare_mit()`：它们会保留已校验的
Motor 指针并复用同一块 Python/ctypes 命令数组，避免每周期创建命令 dataclass 和 ctypes
数组。原生 ABI 每次仍会做校验和按 Controller 分组，因此这是“减少分配”的路径，不承诺
零分配或硬实时。

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
| `Controller.from_socketcanfd(channel="can0", enable_brs=True)` | 打开 Linux SocketCAN-FD；默认在发送帧上设置 `CANFD_BRS`。仅对数据段不切换速率的兼容设备显式传入 `False`。 |
| `Controller.from_dm_serial(serial_port="/dev/ttyACM0", baud=1_000_000)` | 打开达妙串口桥。 |
| `Controller.from_dm_device(device="usb2canfd-dual", channel=0, bitrate=1_000_000, data_bitrate=5_000_000)` | 通过厂商 DM_Device 运行库打开原厂固件，支持 CH0/CH1。默认发送 CAN-FD+BRS 帧（1 Mbps 仲裁、5 Mbps 数据段）；电机必须已配置为对应 CAN-FD 波特率。显式把两个速率设为相同值可回退经典 CAN。 |
| `add_damiao_motor(motor_id, feedback_id, model)` | 在总线上注册电机并返回 `Motor`。 |
| `discover_damiao_motors(candidates, timeout_ms=50, retries=1)` | 不使能、不运动地探测 Required/Optional/Disabled 候选电机，并冻结本次连接中实际存在的电机集合。 |
| `enable_all()` / `disable_all()` | 依次使能或失能所有已注册电机；会发送硬件命令。 |
| `request_feedback_all(timeout_ms=50)` | 请求并等待所有电机各收到一帧新反馈，共享一个总超时。Python 内部使用结构化 ABI，并通过分类异常携带稳定错误码、反馈统计和缺失电机 ID。 |
| `poll_feedback_once()` | 非阻塞排空当前已经到达的帧。 |
| `set_tx_gap_us(gap_us)` | 设置相邻输出帧的最小主机提交间隔。 |
| `transport_capabilities()` | 查询当前 Transport 实例是否支持 CAN-FD、是否实际启用 BRS、物理通道数、并行批量、重连、进程会话复用及硬件接收时间戳。 |
| `transport_health()` | 查询实时连接/健康标志、收发帧与错误计数、最近收发时间以及最后一个 Transport 错误。 |
| `shutdown()` | 先尝试失能全部电机，再停止接收线程并关闭总线。 |
| `close_bus()` | 不发送失能命令，直接停止接收并关闭总线。 |
| `close()` / `closed` | 释放原生 Controller 句柄；`close()` 不主动发送失能命令。 |

需要稳定区分反馈故障的原生调用方应使用
`motor_controller_request_feedback_all_ex()`。该接口返回 `MOTOR_OK`、
`MOTOR_ERROR_FEEDBACK_TIMEOUT`、`MOTOR_ERROR_FEEDBACK_INCOMPLETE`、
`MOTOR_ERROR_TRANSPORT` 或 `MOTOR_ERROR_INVALID_ARGUMENT`，并填写
`MotorFeedbackReport` 以及由调用方提供的缺失电机 ID 缓冲区。允许用空指针和零容量只查询
数量；`missing_count` 始终返回完整缺失数量。旧的
`motor_controller_request_feedback_all()` 继续保持 0/-1 语义，
`motor_last_error_message()` 仅用于日志，不再承担错误分类。加载旧版动态库时，应先检查
`structured_feedback_report` capability 再解析新增符号。

Python 对外仍然只提供 `Controller.request_feedback_all()`，不会暴露 `_ex` 方法。内部根据
结构化返回值抛出 `FeedbackTimeoutError`、`IncompleteFeedbackError`、
`FeedbackTransportError` 或 `FeedbackMotorFaultError`；它们继续继承 `CallError`，并提供
`error_code`、`missing_motor_ids` 和完整 `FeedbackReport`。Articore runtime 的反馈回调也
使用相同的结构化签名，安全逻辑不再解析 `motor_last_error_message()` 字符串。

DM-USB2FDCAN Dual 的回调帧会在进入任何共享注册表操作前立即复制为 motor 自有的不可变
帧，并保留物理 channel。MotorHandle 同时严格校验协议定义的仲裁 ID 和 payload 解码出的
电机 ID。Linux DM_Device v1.0 仍可能偶发交付“帧头与 payload 来自不同通道”的帧，因此与
时间间隔、反馈速度和电机模型速度明显不相容的单次位置跳变会在写缓存前被丢弃，只保留上一份
有效缓存；该处理只记录 `FeedbackIntegrityStats`，不会进入全局 FAULT 或失能。可使用
`Motor.get_feedback_integrity_stats()` 读取身份不匹配、短帧和异常跳变统计。

DM_Device 后端不需要刷写 SocketCAN 固件。可执行一次
`motor-drive-layer-install-dm-device --download` 安装对应厂商运行库，也可通过
`MOTOR_DM_DEVICE_LIB` 指定官方 `libdm_device`/`dm_device.dll`。加载器会探测 v1.0
（`damiao_*`/`device_*`）和 v1.1（`dmcan_*`）ABI，并明确报告缺失符号或动态库依赖错误。
Linux 优先使用兼容范围更广的 v1.0，并支持回退到 v1.1；Windows/macOS 使用相同 Python
接口。每个 Controller 只管理所选通道。v1.1 会在最后一个 Controller 关闭后释放共享物理
USB 句柄。由于官方 Linux v1.0 运行库在同一进程内完整销毁后无法可靠地再次打开设备索引
0，v1.0 会关闭各通道并释放 motor 层线程、client 和队列，但保留 legacy context、device
句柄、回调及动态库到进程退出，供后续 Controller 重连复用。进程退出时由操作系统回收这些
保留的厂商对象，避免在静态析构阶段调用厂商销毁函数并与 libusb 线程清理产生竞态。

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

### ControllerGroup

| 接口 | 作用 |
| --- | --- |
| `ControllerGroup(controllers)` | 为每个 Controller 创建并持有一个常驻原生发送线程。 |
| `send_pos_vel(commands)` | 按 Motor 所属 Controller 并行调度 `PosVelCommand`，并等待全部完成。 |
| `send_mit(commands)` | 按 Motor 所属 Controller 并行调度 `MitCommand`，并等待全部完成。 |
| `prepare_pos_vel(motors)` | 创建固定电机布局且可复用的 POS_VEL 批次；速度限制可用一个标量广播。 |
| `prepare_mit(motors)` | 创建固定电机布局且可复用的 MIT 批次；除位置外各字段可传标量或向量。 |
| `close()` / `closed` | 停止并回收工作线程；不会关闭成员 Controller。 |

### 反馈与重连压力诊断

`motor-drive-layer-stress` 会在同一进程内重复打开指定 DM_Device 通道、请求反馈、关闭并重连，
同时记录延迟、失败、文件描述符数量和线程数量。该工具不会使能电机，也不会发送控制命令：

```bash
motor-drive-layer-stress \
  --motor 0:0x09:0x19:4310 \
  --motor 1:0x0f:0x1f:4310 \
  --iterations 1000 --reconnect-cycles 10 --output stress.json
```

请按实际通道、电机 ID、反馈 ID 和型号重复填写 `--motor`；示例 ID 仅用于说明格式。

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

`bindings/python/examples/` 中保留了七个用途明确的示例：

| 文件 | 用途 |
| --- | --- |
| `connection_test.py` | 失能一台电机，并通过任一受支持传输验证反馈是否正常。 |
| `socketcan_control.py` | 通过 Linux SocketCAN 控制一台电机，演示 MIT 模式。 |
| `dm_serial_control.py` | 通过达妙串口桥控制一台电机，支持 MIT、位置速度、速度和力位混合模式。 |
| `dm_serial_pos_vel.py` | 通过达妙串口桥向七台电机周期发送位置速度（PV）帧。 |
| `multi_motor_control.py` | 通过 Linux SocketCAN 控制多台电机。 |
| `maintenance.py` | 清除错误、设置 CAN 超时、可选设置零位并读取状态。 |
| `register_access.py` | 读取寄存器；只有明确传入写参数时才会写入或保存。 |

先安装项目，再使用 `--help` 查看参数：

```bash
python3 bindings/python/examples/connection_test.py --help
python3 bindings/python/examples/socketcan_control.py --help
python3 bindings/python/examples/dm_serial_control.py --help
python3 bindings/python/examples/dm_serial_pos_vel.py --help
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

达妙电机设置为 `CAN_BR=9` 时，需要同时配置 Linux 的仲裁速率和数据速率，并保持 BRS 开启：

```bash
ip link set can0 type can bitrate 1000000 sample-point 0.75 \
  dbitrate 5000000 dsample-point 0.875 fd on
ip link set can0 up
```

仅设置 `dbitrate` 只是让接口具备数据段速率；发送帧的 `canfd_frame.flags` 仍必须包含
`CANFD_BRS`。`Controller.from_socketcanfd("can0")` 现在默认设置该标志，
`transport_capabilities().can_fd_brs` 可诊断当前实例是否实际启用。新增 C 接口为
`motor_controller_new_socketcanfd_ex(channel, enable_brs)`，旧的单参数接口也改为默认 BRS。

## 测试

无硬件测试：

```bash
cmake --build cpp_damiao/build -j
ctest --test-dir cpp_damiao/build --output-on-failure
PYTHONPATH=bindings/python/src python3 -m pytest -q bindings/python/tests
```

默认 CI 不会打开串口，也不会使能真实电机。

显式启用的真机验收脚本会检查两路 SocketCAN-FD、每路8台电机、400 Hz 初始位置MIT保持
（前馈力矩为零）、逐电机反馈频率、Linux CAN 错误计数以及测试后16/16失能反馈。必须提供16个真实
`--motor` 映射并显式传入 `--i-understand-motors-will-be-enabled`：

```bash
python3 scripts/test_socketcanfd_brs_dual_channel.py --help
```

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

DM_Device 默认的 1 Mbps/5 Mbps 配置会同时配置通道并以 CAN-FD+BRS 发送帧，5 Mbps
数据段使用 87.5% 采样点。实机已验证单通道 8 台电机在 500 Hz 下反馈完整；双通道
各 8 台电机的纯 C++ Runtime 也完成了 30 秒 500 Hz streaming raw MIT 真机测试，并在
每周期读取全部缓存状态时保持 498.53～498.93 Hz 反馈。两条接口始终 ERROR-ACTIVE、
错误计数为零，结束后 16 台电机全部正常失能。因此 Runtime ABI 2.5 在左右两侧均报告
SocketCAN-FD+BRS 时允许最高 500 Hz；DM Device 和旧调用路径仍限制为 400 Hz。容量为一
的非阻塞 raw mailbox 与内部同步的缓存读取也已通过 Articore SDK 公开 raw MIT 路径验收：
30 秒提交 500.02 Hz，16 台反馈 497.36～499.36 Hz，原生 transport 错误为零，最终确认
16/16 失能。实际生效频率通过 `articore_runtime_get_control_hz()` 暴露。

Runtime ABI 2.6 将完整 MIT 合成力矩保护下沉到原生 worker。每个实际 MIT 发送周期
——包括重复发送 latest-mailbox 目标的周期——都使用最新原生 q/dq 反馈重新计算 P+D+
前馈，并将每个关节限制在配置力矩上限的 80%。超限时 Kp、Kd 和前馈力矩按同一比例
缩放；任一必需反馈缺失、过期或非有限时，整个双臂批次不发送并进入现有保护性故障保持路径。
诊断统计可以低频读取，Python raw MIT 提交热路径不再需要读反馈。该修改已通过模拟原生
测试和绑定层测试，本轮按要求未连接真机。

## 贡献与安全

提交修改前请阅读[CONTRIBUTING.md](CONTRIBUTING.md)。涉及机械臂安全或通信漏洞的问题请按照[SECURITY.md](SECURITY.md)私下报告，不要先公开可直接复现危险动作的细节。

## 许可证

MIT，见[LICENSE](LICENSE)。

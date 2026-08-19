# Motor-Drive-Layer

[English](README.md) | 简体中文

Motor-Drive-Layer 是达妙电机与 Articore 产品 Runtime 的原生 C++ 控制底座。所有运行时行为
都在 C++ 中实现，并通过稳定的 C ABI 对外提供。本仓库不发布 Python 模块，也不使用 Python
实现控制、安全、机器人动力学或重力补偿。

## 架构边界

```text
Articore-SDK / C++ SDK / ROS 2
          │ 稳定 C ABI
          ▼
  libarticore_runtime.so
  看门狗、安全、机器人模型、
  夹爪与重力补偿
          │ 稳定电机 ABI
          ▼
      libmotor_abi.so
  Controller、Motor 与通信接口
          │
          ▼
     C++ 协议与通信核心
          │
          ▼
SocketCAN / SocketCAN-FD / 串口 / DM Device
```

职责划分如下：

- `cpp_damiao/` 只负责通用通信、协议、Controller 和 Motor。
- `articore_runtime/` 负责产品 Runtime、机器人模型和重力补偿。
- Articore-SDK 自己维护 Python `ctypes` 声明、值类型和用户接口。
- PyPI 的 `motor-drive-layer` wheel 只分发二进制文件，不包含 `.py` 或 `.pyi`，也不能
  `import motor_drive_layer`。

公开的原生产物包括：

- 通用电机 ABI：`libmotor_abi.so`。
- 产品 Runtime ABI：`libarticore_runtime.so`，声明位于
  [`articore/runtime_abi.h`](articore_runtime/include/articore/runtime_abi.h)。
- C++17 RAII 目标：`motorbridge::articore_runtime_cpp`。

## 原生能力

- 达妙 MIT、位置速度、速度和力位混合模式。
- Linux SocketCAN、SocketCAN-FD+BRS、达妙串口桥和 DM Device。
- 后台反馈接收、反馈一致性检查与电机状态缓存。
- 结构化反馈、通信和 Runtime 故障。
- 有限超时的非阻塞 SocketCAN 发送，内核队列堵塞不会永久卡死关闭流程。
- Runtime 原子使能/失能、看门狗、安全保持和确定性故障处理。
- 产品夹爪策略、关节限制和逐周期 MIT 合力矩保护。
- 七轴原生机器人模型：FK、IK、Jacobian、重力、质量/科氏项、RNEA 和 ABA。
- 原生重力补偿拖动模式。

## Pinocchio 与 ROS 2 隔离

Pinocchio 只作为机器人模型的 C++ 构建依赖。需要的模板实现直接编译进
`libarticore_runtime.so`，并使用隐藏符号。安装后的 Runtime 不动态依赖
`libpinocchio_default.so` 或 Boost.Serialization。

因此 ROS 2 的 `LD_LIBRARY_PATH=/opt/ros/...` 无法替换 Runtime 内部使用的机器人模型实现。
CI 会检查 ELF 依赖表，并在故意注入错误 Pinocchio 库的环境中创建机器人模型。

## 构建

需要 CMake 3.16+、C++17 编译器和 Pinocchio C++ 开发头文件。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

安装原生 SDK 与 CMake 包：

```bash
cmake --install build --prefix /desired/prefix
```

C++ 项目使用：

```cmake
find_package(MotorDriveLayer CONFIG REQUIRED)
target_link_libraries(robot_driver PRIVATE motorbridge::articore_runtime_cpp)
```

RK3588 可使用 `scripts/build_aarch64_runtime.sh` 交叉编译。需要板端 sysroot 时设置
`MOTOR_AARCH64_SYSROOT`。

## PyPI 二进制载荷

PyPI wheel 从 `packaging/pypi/` 构建，仅用于分发平台动态库：

```text
motor_drive_layer_native/
└── lib/
    ├── libmotor_abi.so
    ├── libarticore_runtime.so
    └── dm_device/
```

wheel 不提供 Python import 接口。Articore-SDK 通过发行包元数据定位动态库，并自行维护
ABI 声明。这样底层保持纯 C/C++，上层 SDK 的 `robot.connect()`、`enable()` 和重力补偿
交互方式不变。

编译原生库后可构建本地 wheel：

```bash
python3 -m build --wheel packaging/pypi
```

这里使用 Python 是因为 PyPI wheel 构建工具本身基于 Python；最终 wheel 不安装任何 Python
运行时代码。

## 通信行为

单电机 Controller 默认不增加发送延迟；添加第二个电机后，默认启用 200 µs 的最小帧间隔。
可通过 `MOTOR_DRIVE_LAYER_TX_GAP_US` 或原生配置接口修改。

Linux SocketCAN 与 SocketCAN-FD socket 使用非阻塞模式。内核发送队列持续满时，默认 20 ms
后返回错误，写入通信健康状态并传递给 Runtime 故障处理。可用
`MOTOR_DRIVE_LAYER_SOCKETCAN_SEND_TIMEOUT_MS` 设置 1 到 60000 ms 的超时。

`motor_controller_request_feedback_all_ex()` 返回稳定的反馈错误码与缺失电机报告。Runtime
集成必须使用结构化结果，不能解析错误字符串做安全决策。

## Runtime 与重力补偿

`libarticore_runtime.so` 独占固定频率 worker、最新命令 mailbox、电机 lease、看门狗、
使能/失能事务、故障保持、夹爪策略、力矩限制和重力补偿状态机。语言绑定只负责提交配置和
命令，不得重新实现这些算法。

Runtime ABI 2.8 提供第一版重力补偿模式。SDK 将已安装的七轴侧绑定到 `yunyi_v1_0`，以
MIT 模式使能后启动重力补偿。原生 worker 平滑移除刚度和阻尼，同时逐步加入随姿态变化的
重力力矩；停止时反向过渡到当前位置 MIT 保持。第一版暂不包含摩擦和科氏补偿。

Runtime ABI 2.9 将整机维护操作收归 C++ Runtime。`configure_mode`、`clear_faults` 和
`set_zero` 复用 Runtime 已持有的 Motor lease，在同一 worker/发送屏障内并行处理左右通道，
不会释放 ControllerGroup、关闭 Runtime 或重建资源。调零固定检查 READY、双 transport、
反馈新鲜、物理失能和静止速度，操作后逐电机验证失能、零位和零速；任何部分失败都以稳定
错误码写入统一 `SafetyHealthV2`，不会伪装为成功。产品工厂
`articore_runtime_create_product("yunyi_v1_0", ...)` 内部拥有 can-left/can-right、
SocketCAN-FD+BRS、14 个关节和两个夹爪。协议仍是逐电机写入，真正 all-or-none 需要固件增加
prepare/commit。

Runtime ABI 2.10 将 `yunyi_v1_0` 从“由语言绑定组装的通用容器”升级为完整产品
Runtime。产品工厂固定拥有双通道、16 个 Motor、方向/量程/限位、默认 MIT 参数、夹爪与
重力模型；新增固定 14 关节 MIT/PV/普通位置帧、双夹爪帧和一次性整机状态快照。SDK 只传
逻辑关节数组，不再传 Controller、ControllerGroup、Motor 或产品配置。产品 Runtime 对象
保持存在，`disconnect()` 只把状态切回 DISCONNECTED，不释放 lease 或重建 worker。普通
位置命令的速度传 0 时，由原生产品配置选择当前模式默认值。

Runtime ABI 2.11 为整机工厂增加固定拓扑参数 `with_grippers`。值为 true 时创建并验证
14 个关节和左右两个夹爪；值为 false 时只创建 14 个关节 Motor，不发送夹爪反馈请求，
也不把缺少夹爪写入 health。新的 `articore_runtime_set_grippers()` 只接收左右 0～1000
开合度和 1～5 力度等级；无夹爪产品调用它会安全返回成功。整机状态只公开夹爪是否可用、
开合度和力度等级，不再向普通 SDK 暴露夹爪 Motor 坐标、速度、力矩或句柄。

Runtime ABI 2.12 将通信质量与电机硬故障分离。少于
`feedback_failure_threshold` 的偶发反馈缺口只计数；持续延迟进入 `DEGRADED`，原生层将
速度参考和 MIT 力矩上限缩放到 25%；延迟达到三倍阈值后进入 `SAFE_STOP`，停止接受新轨迹
并持续发送当前位置保护保持，不主动失能，也不写入 `fault_reason`。只有确认的电机故障码、
意外失能、非有限反馈或 transport 断开才进入 `FAULT`。通信恢复不会自动重放旧目标；调用
`recover()` 后 Runtime 重新读取全部电机、同步当前位置，并回到 `ENABLED` 等待新命令。

Runtime ABI 2.13 将普通 MIT/PV 位置接口的统一速度参数改为 `0～100`。`0` 暂停位置
reference 推进，`100` 对应产品为当前模式配置的最大普通速度；百分比到 rad/s、逐周期
步长和关节绝对速度上限均在 C++ Runtime 内处理。Raw MIT/PV 帧继续使用物理量，不受影响。

Runtime ABI 2.14 增加 Runtime 所有的整机/单电机使能与失能接口。稳定角色名采用
`left/joint1`～`left/joint7`、`right/joint1`～`right/joint7`，夹爪为
`left/gripper`、`right/gripper`；空角色表示全部已安装电机。设置操作必须通过新反馈确认，
查询返回 `DISABLED`、`ENABLED`、`MIXED` 或 `UNKNOWN`。单电机切换仅允许在非运动状态，
并进入不可下发运动命令的 `PARTIALLY_ENABLED`；只有整机原子 `enable()` 建立当前位置保持后
才进入正常 `ENABLED`。

Runtime ABI 2.15 增加 `articore_runtime_get_pose()`。接口从原生反馈缓存读取指定左/右臂
完整七关节快照，并使用底层内置 Pinocchio 产品模型计算法兰位姿，固定输出
`[x, y, z, roll, pitch, yaw]`（米、弧度）以及参与计算的最旧反馈时间戳和序列号。
调用本身不发送 CAN 帧；当前产品没有 TCP 偏移配置，因此不提供虚构的 TCP 位姿，也不增加
驱动无法可靠提供的电压/相电流字段。

## 安全

电机控制可能造成意外运动和人身伤害。测试时必须支撑机构、准备独立急停、确认通道/ID/型号/
模式，并从保守限制开始。寄存器写入可能永久改变电机配置。

## 测试

默认测试不会使能真实电机：

```bash
cmake --build build -j
ctest --test-dir build --output-on-failure
```

CI 还会验证 wheel 不含 Python 源码、两个原生 ABI 库可加载、机器人模型不依赖 Pinocchio
动态库，以及各发布架构上的 DM Device 依赖可正确解析。

真机验收脚本位于 `scripts/`，必须先检查脚本，并显式提供电机映射和确认参数。

## 仓库结构

```text
cpp_damiao/              通用 C++ 协议、通信与电机 C ABI
articore_runtime/         原生产品 Runtime、机器人模型与 C/C++ ABI
packaging/pypi/           仅组装二进制 wheel；不含 Python 运行时模块
third_party/dm_device/    可选厂商头文件和可再分发动态库
scripts/                  构建、诊断与真机验收工具
tests/                    原生 CMake 包消费测试
```

## 许可证

Motor-Drive-Layer 使用 MIT 许可证。随 wheel 分发的 DM Device、libusb 和 libstdc++ 组件保留
其各自许可证说明，位于 `packaging/pypi/LICENSES/`。

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

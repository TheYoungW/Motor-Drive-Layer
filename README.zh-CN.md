# Motor Drive Layer

Yunyi 双臂产品的原生 C++ 控制底层。

`main` 只保留 Linux 下的双 SocketCAN-FD+BRS 产品路径：`can-left` 与
`can-right`。原来的多种 CAN、Damiao SDK 架构单独保存在
`legacy-multi-can-damiao-sdk` 分支。

## 架构

```text
Articore SDK
    -> Yunyi Runtime C ABI
        -> 500 Hz 整机 Runtime 与安全状态机
            -> 原生 C++ Motor 核心
                -> 左右 Controller 发送 worker
                    -> SocketCAN-FD+BRS 帧

can-left/right 接收线程
    -> Motor 反馈缓存
        -> Runtime 整机状态与 health 快照
            -> Articore SDK
```

公开 wheel 只包含 `libarticore_runtime.so`。Runtime 在同一 C++ 进程内直接
调用 Motor 核心，不再经过 Motor C ABI；本仓库也不包含 Python 控制实现。

## 当前接口契约

- 包版本：`0.21.0`
- Runtime ABI：`11.2` / `0x000B0002`

Runtime health 现在按产品顺序返回每个已安装电机的角色名、CAN ID、反馈年龄、
状态码和问题位，并明确区分单电机反馈异常、多电机异常、左/右通道停收和双通道停收。
- ABI 校验：必须完全相等
- 产品：`yunyi_v1_0`
- 关节顺序：左 J1～J7、右 J1～J7
- 夹爪：创建 Runtime 时选择是否安装左右成对夹爪

SDK 通过 `articore_runtime_create_yunyi(mode, with_grippers, &runtime)`
创建完整产品。Motor 映射、Controller、限位、模型、TCP 偏移、worker 和
资源生命周期都由 Runtime 管理。公开 ABI 不包含 Motor 指针、通用组装接口、
能力位或带版本后缀的重复函数。

普通 PV 与 `set_pose()` 共用同一套参考生成器。500 Hz 是 Runtime 的安全监控和
驱动命令刷新周期，不是点到点规划频率。每条命令的 `speed`
直接把 `0～100` 映射为 `0～2 rad/s`，不存在额外的持久化最大速度上限。
最大加速度使用真实物理单位和 `0.01` 设置精度，范围 `0.01～8.00 rad/s²`、
默认 `6.00 rad/s²`；它不影响 MIT。关节轨迹、Linear/Circular 在 PV 模式下
也复用这套普通 PV 参考限制。

`set_joint_pv(left, right, speed)` 是关节点到点：输入关节角度并提交完整双臂
PV 目标。`set_pose(left_pose, right_pose, speed)` 是末端位姿到关节目标的便捷
命令：只额外在 C++ 中对两侧 Pose 各求一次终点 IK，然后仍提交同一种普通 PV
关节目标；它不属于 Linear/Circular 路径规划。两侧共用同一份规划参考快照，
全部在 `1e-4` 精度和 8 ms 软预算内成功后才原子安装 14 关节 PV 目标；任一侧
失败或超时都不改变当前目标。

关节轨迹、Linear 和 Circular 共用一个 Motion ID、FIFO、状态查询和取消接口。
Linear/Circular 先按 5 mm / 0.035 rad 求稀疏 IK 几何路径，再用一条全局五次
时间律生成固定 2 ms 间隔的普通 PV 参考点。`duration_s=3` 对应 1500 段、
1501 点；参考序列约运行 3 秒，真实到位可能因 PV 限速、加速度限制和反馈稳定
确认而更晚。中间点使用连续速度跟踪，不再把每一点当作刹停终点；只有单个 motion
的最终点制动。连续的 Linear/Circular FIFO 在公共端点且跟踪误差不超过 0.04 rad
时按计划时刻直接交接，不再为每段额外等待 200 ms 稳定窗口；超出门槛时仍等待
真实跟踪恢复。500 Hz 循环每周期只推进一个点，不会批量发送或使用可变间隔。普通 PV
仍使用默认 speed 50 和当前最大加速度；规划点的数值导数不用于超限拒绝。
同一个 `move_linear_trajectory()` 也可一次提交 2～64 个 Pose：两个 Pose 保持
普通直线语义，三个及以上 Pose 对每个有效内部角点默认使用 10 mm 笛卡尔圆角，
短线段自动缩小圆角以避免相邻过渡重叠。整条折线只生成一个 Motion ID，并使用
一条全局五次时间律；`duration_s` 仍表示每条原始线段的参考时间。
自动接近显式起点也使用普通 PV。Linear、Circular 和
`set_pose()` 均要求 PV 产品模式；普通 MIT 和 MIT 关节轨迹保持不变。
`set_pose()` 仍是普通 PV 点目标，不加入 Motion FIFO。

当前 API 见 [Runtime 说明](articore_runtime/README.md)。

## 构建与测试

```bash
cmake -S . -B builds/dev -DCMAKE_BUILD_TYPE=Release
cmake --build builds/dev -j
ctest --test-dir builds/dev --output-on-failure
```

真机诊断程序不会加入 CTest，需要单独构建。它们可能使能或移动机械臂，必须
确认现场安全后运行。本地开发、编译和测试不需要先发布 PyPI。

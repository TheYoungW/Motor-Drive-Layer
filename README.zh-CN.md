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

- 包版本：`0.22.0`
- Runtime ABI：`11.3` / `0x000B0003`

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

普通关节 PV 的 14 个关节共享
`s(u)=35u⁴-84u⁵+70u⁶-20u⁷` 这一条静止到静止的七次进度曲线。参考点更新和
物理 POS_VEL 报文都按 100 Hz 执行，每个点只发送一次；独立的 500 Hz Runtime
调度继续负责安全监控、反馈检查和命令 watchdog。每条命令的 `speed`
直接把 `0～100` 映射为 `0～2 rad/s`，不存在额外的持久化最大速度上限。
最大加速度使用真实物理单位和 `0.01` 设置精度，范围 `0.01～8.00 rad/s²`、
默认 `6.00 rad/s²`。Runtime 根据速度、加速度和内建 jerk 平滑约束取共同总时长，
并向上对齐到完整的 10 ms 采样。当前 ABI 没有独立 jerk 设置，因此内建策略使用
`j_max = a_max / 0.10 s`（默认加速度下为 `60 rad/s³`）；它不影响 MIT。
关节 PTP 是唯一的点到点规划器。Pose 调用方先通过纯计算接口
`solve_ik(left_pose, right_pose)` 获得 14 个关节角，再调用 `set_joint_pv()`。
旧 `set_pose()` 符号仅作为兼容快捷入口保留，内部执行相同的 IK→关节七次 PTP 链路；
Linear/Circular 继续使用各自现有的笛卡尔路径与五次时间律。

`set_joint_pv(left, right, speed)` 是同步关节点到点：输入关节角度并提交完整双臂
PV 目标；各关节位移不同，但任意时刻完成比例相同，且同时启动、同时停止、同时到达。
`solve_ik(left_pose, right_pose)` 只求解、不运动：两侧共用同一份规划参考快照；
若尚未使能，则使用已连接 Runtime 的新鲜完整反馈，
以当前 TCP、产品限位和最近关节分支求解固定顺序的 14 个关节角。它不会使能 Motor、
发送控制帧或改变运动队列。兼容的 `set_pose()` 只有在两侧都于 `1e-4` 精度和
8 ms 软预算内成功后，才原子安装同一个同步关节 PTP 目标；任一侧失败或超时都不改变
当前目标。它不属于 Linear/Circular 路径规划。

关节轨迹、Linear 和 Circular 共用一个 Motion ID、FIFO、状态查询和取消接口。
Linear 从当前/指定起点的 FK Pose 出发，在笛卡尔空间对 XYZ 做直线插值、对姿态做
四元数最短路真 SLERP，并以上一个关节解作为下一个 Pose 的 IK 初值。几何路径按
2 mm / 0.1 rad 或更细离散，再用一条全局五次时间律生成固定 10 ms 间隔的普通 PV
参考点；`duration_s=3` 通常对应 300 段、301 点。每个 Linear 参考点通过 Runtime
内部实时 PV 以 100 Hz 原样发送一次，不再叠加执行层插值、步进或重复报文；独立的
500 Hz Runtime 调度继续负责安全和反馈。Circular 由 start/via/end 三点确定有向圆弧，
位置按 2 mm 或更细离散，姿态以最短路真 SLERP 经过 via 姿态；随后使用同一套全局五次
时间律和 10 ms 实时 PV 参考。真实到位可能因
PV 限速、加速度限制和反馈稳定确认而更晚。连续的 Linear/Circular FIFO 在公共端点
且跟踪误差不超过 0.04 rad
时按计划时刻直接交接，不再为每段额外等待 200 ms 稳定窗口；超出门槛时仍等待
真实跟踪恢复。Linear 和 Circular 都会按 10 ms 相邻关节参考检查默认 speed 50 与当前最大加速度；
若给定时长太短，会自动向上拉长到完整 10 ms，而不是向 PV 驱动提交过大的阶跃。
同一个 `move_linear_trajectory()` 也可一次提交 2～64 个 Pose：两个 Pose 保持
普通直线语义，三个及以上 Pose 对每个有效内部角点默认使用 10 mm 笛卡尔圆角，
短线段自动缩小圆角以避免相邻过渡重叠。整条折线只生成一个 Motion ID，并使用
一条全局五次时间律；`duration_s` 仍表示每条原始线段的参考时间。
公开的 `set_joint_pv()` 是用户使用的普通 PV（步进/点到点）接口；实时 PV 只允许由
Runtime 内部已经完成校验的有限 Joint/Linear/Circular 轨迹选择，不导出 Raw PV 或
流式 PV 普通接口。自动接近显式起点也使用普通 PV。Linear、Circular 和
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

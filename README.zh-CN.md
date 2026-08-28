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

- 包版本：`0.22.4`
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

普通关节 PV 是给用户开发使用的步进模式。一次
`set_joint_pv(left, right, speed)` 提交完整的 14 关节目标；下一次调用原子覆盖上一目标，
Runtime 不生成有限的七次/五次曲线，也不创建 Motion ID。Runtime 在每个 500 Hz
控制周期在线推进 POS_VEL 的 `P` 参考值，使其趋向最新终点。`speed=0～100` 控制
`P` 的参考速度比例（`0～2 rad/s`）；电机 POS_VEL 的 `V` 上限与它独立，当前产品
上限为 `3 rad/s`。

最大加速度仍使用真实物理单位和 `0.01` 设置精度，范围 `0.01～8.00 rad/s²`、
默认 `6.00 rad/s²`。它只约束普通 PV 在线步进的加速、刹车和反向。
Joint/Linear/Circular 使用完全独立的轨迹速度、加速度、jerk、时间和同步约束。
用户只填写位置/路径和时间，轨迹加速度与 jerk 是 Runtime 内部策略，不作为接口暴露。
Pose 调用方通过纯计算接口 `solve_ik(left_pose, right_pose)` 获得 14 个关节角，
再选择普通 PV 或 MIT 执行；`set_pose()` 只求解一次 IK，然后按 Runtime 当前选择的
普通 PV/MIT 模式原子安装关节终点，不是轨迹规划器。

`solve_ik(left_pose, right_pose)` 只求解、不运动：两侧共用同一份规划参考快照；
若尚未使能，则使用已连接 Runtime 的新鲜完整反馈，
以当前 TCP、产品限位和最近关节分支求解固定顺序的 14 个关节角。它不会使能 Motor、
发送控制帧或改变运动队列。兼容的 `set_pose()` 只有在两侧都于 `1e-4` 精度和
8 ms 软预算内成功后，才按当前普通 PV/MIT 模式原子安装求解出的关节目标；任一侧失败或超时都不改变
当前目标。它不属于 Linear/Circular 路径规划，也不创建关节轨迹。

关节轨迹、Linear 和 Circular 共用一个 Motion ID、FIFO、状态查询和取消接口。
Linear 从当前/指定起点的 FK Pose 出发，在笛卡尔空间对 XYZ 做直线插值、对姿态做
四元数最短路真 SLERP。第一个路径 Pose 只使用当前规划关节角作为 IK seed，后续每个
Pose 只使用上一个 IK 解；Linear/Circular 路径 IK 不使用备用或随机 seed，当前局部分支
同时使用零空间姿态约束保持靠近上一个解，并根据上一段关节步长预测下一次向前半步的
偏好姿态，让冗余关节尽量保持速度方向连续、减少 `+/-` IK 抖动。这是优化目标，不是
关节换向拒绝规则；笛卡尔路径确实需要换向时仍允许通过。只有局部分支无法收敛或单个
采样关节跳变超过 0.35 rad 时才拒绝路径。几何路径按
2 mm / 0.1 rad 或更细离散，再用一条全局五次时间律生成固定 10 ms 间隔的内部实时
PV 关键点；`duration_s=3` 通常对应 300 段、301 点。Runtime 在相邻关键点之间按时间
线性重采样，并以 500 Hz 发送所得参考，不叠加普通 PV 的终点步进器。Circular 由
start/via/end 三点确定有向圆弧，
位置按 2 mm 或更细离散，姿态以最短路真 SLERP 经过 via 姿态；随后使用同一套全局五次
时间律和 10 ms 实时 PV 参考。真实到位可能因
PV 限速、加速度限制和反馈稳定确认而更晚。连续的 Linear/Circular FIFO 在公共端点
且跟踪误差不超过 0.04 rad
时按计划时刻直接交接，不再为每段额外等待 200 ms 稳定窗口；超出门槛时仍等待
真实跟踪恢复。Linear 和 Circular 都会按 10 ms 相邻关节参考检查默认 speed 50 与轨迹自身的加速度限制；
若给定时长太短，会自动向上拉长到完整 10 ms，而不是向 PV 驱动提交过大的阶跃。
同一个 `move_linear_trajectory()` 也可一次提交 2～64 个 Pose：两个 Pose 保持
普通直线语义，三个及以上 Pose 对每个有效内部角点默认使用 10 mm 笛卡尔圆角，
短线段自动缩小圆角以避免相邻过渡重叠。整条折线只生成一个 Motion ID，并使用
一条全局五次时间律；`duration_s` 仍表示每条原始线段的参考时间。
公开的 `set_joint_pv()` 是用户使用的普通 PV 步进接口；实时 PV 只允许由
Runtime 内部已经完成校验的有限 Joint/Linear/Circular 轨迹选择，不导出 Raw PV 或
流式 PV 普通接口。自动接近属于同一个内部轨迹执行链。Linear、Circular 要求 PV
产品模式；`set_pose()` 支持当前普通 PV 或 MIT 模式，且不加入 Motion FIFO。

当前 API 见 [Runtime 说明](articore_runtime/README.md)。

## 构建与测试

```bash
cmake -S . -B builds/dev -DCMAKE_BUILD_TYPE=Release
cmake --build builds/dev -j
ctest --test-dir builds/dev --output-on-failure
```

真机诊断程序不会加入 CTest，需要单独构建。它们可能使能或移动机械臂，必须
确认现场安全后运行。本地开发、编译和测试不需要先发布 PyPI。

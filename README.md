# ROS2 四旋翼无人机仿真系统

## 目录

- [项目简介](#项目简介)
- [当前阶段](#当前阶段)
- [总体方案](#总体方案)
- [已实现功能](#已实现功能)
- [已验证场景](#已验证场景)
- [待完成场景](#待完成场景)
- [项目结构](#项目结构)
- [环境与依赖](#环境与依赖)
- [编译与运行](#编译与运行)
- [人工验收流程](#人工验收流程)
- [目标发布](#目标发布)
- [安全保护与参数基线](#安全保护与参数基线)
- [当前限制](#当前限制)
- [文档说明](#文档说明)

## 项目简介

本项目是一个基于 Ubuntu 22.04 和 ROS2 Humble 的小型四旋翼无人机仿真系统。系统以四个电机目标 RPM 为动力学输入，计算无人机的位置、速度、姿态和角速度，并通过 ROS2 Topic、TF 和 RViz2 形成可观察的闭环仿真环境。

核心算法与 ROS2 通信层分离，动力学、控制器、Motor Mixer、多目标点任务管理、分段五次轨迹、静态环境碰撞检查、三维栅格 A*、视线简化、安全规划轨迹生成、静态避障执行与有序多目标静态避障均可独立测试。

## 当前阶段

动力学、高度/yaw、单目标三维位置闭环、第一版多目标点顺序飞行、连续轨迹生成与跟踪、静态三维 AABB 环境、统一碰撞查询、三维 26 邻域 A*、确定性视线简化、安全规划轨迹生成、单目标静态避障和有序多目标静态避障均已完成。新任务节点从实际地面 Odom 起飞，在空中导航地板以上逐段规划，稳定到达当前目标后才切换下一目标。三个独立静态避障场景仍保留可复现评测、结构化指标和数据曲线。

## 总体方案

### 当前已实现运行链路

```text
mission.yaml → waypoint_manager_node
  ├─→ /drone/mission/current_waypoint_index
  ├─→ /drone/mission/complete
  └─→ /drone/goal
  ↓
position_controller_node
  ↓
PositionController
  ├─→ HorizontalPositionController
  └─→ HoverController
       ├─→ AltitudeController
       ├─→ AttitudeController
       └─→ MotorMixer
  ↓
/drone/motor_rpm_cmd
  ↓
quadrotor_dynamics_node
  ↓
QuadrotorModel
  ↓
/drone/odom、/drone/imu、/drone/path、map -> base_link TF
  ↓
控制反馈 + RViz2 可视化
```

连续轨迹是独立的第二条上游链路：

```text
trajectory.yaml → trajectory_mission_node ← /drone/odom
  ├─→ /drone/trajectory_setpoint (position/velocity/acceleration/yaw)
  ├─→ /drone/trajectory/current_segment
  ├─→ /drone/trajectory/complete
  └─→ /drone/reference_path
  ↓
position_controller_node (setpoint_source=trajectory)
  ↓
同一 PositionController、HoverController、Mixer 和动力学闭环
```

静态环境是与控制链路解耦的监测链路：

```text
environment.yaml → static_environment_node
  ├─→ /drone/environment/markers
  └─← /drone/odom → /drone/environment/in_collision

StaticEnvironment + CollisionChecker
  ├─→ 安全半径收缩后的工作空间
  ├─→ 安全半径膨胀后的 AABB 障碍物
  └─→ 点与线段碰撞查询
```

规划轨迹生成与执行链路：

```text
/drone/planned_path（A* 原始栅格路径）
  ↓ PathSimplifier（0.35 m 有效规划半径）
/drone/simplified_path（视线简化折线）
  ↓ PlannedTrajectoryBuilder + PiecewiseQuinticTrajectory
  ↓ 多候选中间速度比例、动态约束和密集碰撞验证
/drone/reference_path（连续参考轨迹）
  ↓ planned_trajectory_node（准备起点、稳态时间采样、Odom 失效时暂停）
/drone/trajectory_setpoint
  ↓ position_controller_node → /drone/motor_rpm_cmd → quadrotor_dynamics_node
```

`execution_enabled=false` 时链路止于 `/drone/reference_path`，保持只显示行为；只有静态避障 Launch 显式设为 `true` 时才发布执行 setpoint。

有序多目标静态避障使用一条独占执行链路：

```text
首次有效 /drone/odom
  → 原地垂直起飞到 z=1.5 m（使用原环境与 0.35 m 有效半径检查）
  → 从稳定后的实际 Odom 规划当前目标
  → AStarPlanner → PathSimplifier → PlannedTrajectoryBuilder
  → /drone/trajectory_setpoint → PositionController → 动力学
  → 目标处停稳 1.0 s → 从新的实际 Odom 规划下一目标
```

`multi_goal_static_avoidance_node` 直接组合已有三个纯算法类，不同时启动 `astar_planner_node`、`planned_trajectory_node`、固定轨迹任务或 waypoint 任务。地面只用于动力学接触与起飞；A* 空中导航工作空间的安全最低高度为 `0.50 m`，不会从地面状态直接开始规划。

节点将 Odom 的完整机体系线速度旋转到世界系后用于 x/y/z 反馈。目标 roll/pitch 仍被忽略，只使用目标四元数中的 yaw；期望 roll/pitch 由水平位置控制器生成。

### 最终目标链路

```text
三维目标点或多目标点
  ↓
地图、路径规划与避障
  ↓
安全轨迹或 Waypoint
  ↓
x/y/z 位置控制 + 姿态控制
  ↓
Motor Mixer 与四电机 RPM
  ↓
四旋翼动力学
  ↓
状态反馈、RViz2 可视化与实验评测
```

## 已实现功能

- 六个 `ament_cmake` package：`drone_msgs`、`drone_dynamics`、`drone_controller`、`drone_mission`、`drone_planning`、`drone_bringup`；
- 自定义 `drone_msgs/msg/MotorRPM` 消息；
- 自定义 `drone_msgs/msg/TrajectorySetpoint` 消息，包含世界系位置、速度、加速度和 yaw；
- 四旋翼刚体动力学、电机一阶响应、RPM 限幅、X 型推力与力矩模型；
- 可配置的简化水平地面约束；
- `/drone/odom`、`/drone/imu`、`/drone/path` 和 `map -> base_link` TF；
- 与动力学符号一致的 Motor Mixer、姿态/角速度控制器和高度控制器；
- 高度控制器、姿态控制器和 Mixer 组成的 `HoverController`，并已接入 ROS2 控制节点；
- 世界系 x/y 位置和速度反馈的 `HorizontalPositionController`；
- 组合水平位置与现有 Hover 链路的 `PositionController`；
- 水平与高度控制链路支持三维期望加速度前馈，默认 `pose_goal` 模式保持原有零速度、零加速度目标语义；
- 使用真实控制器、Mixer、电机一阶响应和刚体动力学的 20 秒姿态闭环稳定性测试；
- 通过真实 ROS2 Launch、节点和 Topic 的可重复单目标三维端到端 smoke test；
- 与 ROS2 无关的顺序 `WaypointManager`、ROS2 任务节点、任务状态 Topic 和参数化五点任务；
- 通过真实 ROS2 节点和 Topic 的可重复多目标点端到端回归测试；
- 与 ROS2 无关的 C² 分段五次轨迹、稳态时间驱动的轨迹任务节点，以及核对连续性、跟踪误差和完成保持的真实 ROS2 端到端回归测试；
- 与 ROS2 无关的有限三维工作空间、静态 AABB 障碍物和保守球形无人机碰撞模型，支持点与三维线段查询；
- 静态环境 ROS2 节点、瞬态本地 MarkerArray、实时碰撞状态监测和独立 Domain 96 集成测试；
- 与 ROS2 无关的三维 26 邻域 A*，对候选节点和每条邻接边复用 `CollisionChecker`，并提供独立 Domain 97 规划集成测试；
- 与 ROS2 无关的确定性最远可见点 `PathSimplifier`，严格验证输入点、原始边和所有简化边；
- `PiecewiseQuinticTrajectory` 支持兼容式中间速度缩放，默认 `1.0` 保持旧行为，`0.0` 提供逐段直线且 waypoint 停速的确定性保底；
- 与 ROS2 无关的 `PlannedTrajectoryBuilder`：先保持原视线简化快速路径；若连续曲线在拐角切入障碍物，则按原始 A* 索引在局部确定性补点，并在纯动态约束失败时依次放大分段时间；每个候选仍完整检查有限性、速度、加速度、采样点和采样连线碰撞；
- `/drone/simplified_path`、安全 `/drone/reference_path` 和轨迹生成指标的 transient-local 发布，以及独立 Domain 98 集成测试；
- `planned_trajectory_node` 的起点准备、稳态时钟执行、Odom 超时暂停、结束保持和段/完成状态发布；
- `static_avoidance_sim.launch.py` 的唯一 A*→规划轨迹→控制器→动力学链路，以及独立 Domain 99 真实端到端安全回归；
- 三个独立规划配置和 `tools/evaluate_static_avoidance.py` 顺序评测工具；每个场景使用独立 ROS Domain，保存 JSON、CSV、XY 路径、位置跟踪、跟踪误差和净空曲线，不加入默认 `colcon test`；
- `multi_goal_static_avoidance_node` 的首次 Odom 起飞检查、导航地板、有序逐段规划、目标停稳切换、Odom 超时暂停和最终持续保持；
- `multi_goal_static_avoidance_sim.launch.py` 的唯一多目标规划执行链路，以及独立 Domain 113 远程首目标与偏置返航真实闭环安全回归；
- RViz 三维多目标编辑器：一个世界坐标固定的 XY 移动平面和 Z 轴箭头、最多 8 个有序目标、快速几何拒绝、异步完整连续轨迹验证、独立预览 Path 和可复制 YAML；
- `tools/evaluate_multi_goal_mission.py` 的独立 Domain 114 完整任务评测，记录三段路径、实际/参考状态、净空、四电机 RPM、目标接受时刻和最终悬停，并生成结构化数据与报告图；
- MotorRPM 命令超时保护；
- 默认关闭的 `/drone/external_wrench` 世界系外力输入，具备 frame、有限性、模长限制和 `0.20 s` 超时清零保护，并发布实际应用状态；
- `disturbance_hover_sim.launch.py`、周期外力工具和 Domain 119 悬停抗扰自动评测；
- Xacro 四旋翼模型、robot_state_publisher 和 RViz2 基础可视化；
- `basic_sim.launch.py` 一键启动动力学、控制器、机器人模型发布和 RViz2；`mission_sim.launch.py` 启动离散顺序任务，`trajectory_sim.launch.py` 启动连续轨迹任务，`environment_sim.launch.py` 启动静态环境监测，`planning_sim.launch.py` 再增加一次性 A* 规划与路径显示。

## 已验证场景

- 零 RPM 自由落体，以及正常 Launch 下零 RPM 保持在简化地面；
- 对称推力和独立 roll、pitch、yaw 力矩方向；
- 从地面自动起飞到 `1.5 m` 并稳定悬停；
- 高度在 `0 → 1.5 m`、`1.5 → 2.0 m`、`2.0 → 1.5 m` 间自动升降；
- yaw 转向能够快速接近目标，用户在 RViz2 中确认基本无超调；
- `0.02 rad` 正负 roll/pitch 固定命令和水平姿态均通过 20 秒完整闭环测试；
- 原地 `1.5 m` 悬停、`(0.5,0,1.5)` 小目标和 `(2,1,1.5)` 单目标三维飞行均通过真实 ROS2 数值验收；
- `(2,1,1.5)` 连续观察 27 秒后的三维误差约 `1.37e-7 m`，水平速度约 `7.84e-8 m/s`，无非有限值、姿态发散、RPM 边界值或日志饱和；
- 可重复的 ROS2 单目标端到端 smoke test 已验证 `(2,1,1.5)` 连续满足位置和速度条件 `2.0 s` 后继续观测 `3.0 s`，且没有离开稳定区域；
- 五点任务 `(0,0,1.5,0) → (2,0,1.5,0) → (2,1.5,2.0,π/2) → (0,1.5,1.5,π) → (0,0,1.5,0)` 已按索引 `0→1→2→3→4` 完成；自动回归中最终误差 `0.034256 m`、线速度 `0.021543 m/s`，完成后仍持续发布最终目标；
- 同一五点的连续轨迹回归中，准备悬停后约 `3.576 s` 开始轨迹，约 `27.576 s` 完成；参考最大速度 `0.558524 m/s`、最大加速度 `0.271903 m/s²`，采样最大跟踪误差 `0.030507 m`，最终误差 `0.002669 m`、速度 `0.001033 m/s`；四段边界均通过位置、速度和加速度连续性检查，三个中间点的参考速度均非零；
- 静态环境碰撞检查已验证安全点、膨胀障碍物、收缩工作空间、非有限输入，以及线段穿越、相切、角点、平行、端点命中、零长度和极短线段；ROS2 集成测试验证了 Marker 分类以及安全、碰撞、恢复和非法 Odom 抑制发布；
- 最终验收环境为 `15.5 × 9.5 × 5.5 m`，包含三个在安全膨胀后连接边界的高墙和三个内部独立高障碍物；O5/O6 形成原始 `1.9 m`、有效规划宽度 `1.2 m` 的挑战通道，不能从同一外围一次绕过全部障碍物；
- O5/O6 已从 `x=10.7 m` 整体右移到 `x=11.4 m`，O4 与末端障碍物组的 `0.35 m` 规划膨胀后 x 间距由 `0.30 m` 增加到 `1.00 m`，同时保持原有 S 型路线和挑战通道宽度；
- 默认场景的三维 26 邻域 A* 使用 `0.25 m` 基础安全半径和 `0.10 m` 规划裕量，按 `0.35 m` 有效半径检查节点和边；默认单目标为 `(0,0,1.5) → (13.2,5.5,1.5)`；
- 远端高 y 目标 `(13.2,5.5,1.5)` 的正式评测得到 72 个原始路径点、`21.013672 m`，扩展 18562 个节点；初始视线简化为 9 点，安全细化后为 18 点、`20.181489 m`，连续轨迹 `62.768339 s`，所有采样点与相邻线段均通过 `0.35 m` 有效规划碰撞检查；
- 用户已在 RViz2 中人工确认黄色 A* 原始栅格路径、粉色视线简化折线和蓝色连续参考轨迹能够同时显示；蓝色连续参考轨迹整体平滑，符合当前路径轨迹化预期。视觉观察不用于精确测量安全距离，几何安全仍以自动碰撞验证为依据；
- 远端目标闭环评测于 `68.757 s` 完成，最大跟踪误差 `0.030305 m`、对基础膨胀障碍物的最小净空 `0.089736 m`、最终误差 `0.006014 m`、最终速度 `0.001466 m/s`；全程无碰撞、非有限值或控制器饱和；
- 评测场景 B `(12.1,1.1,2.5)` 与 C `(7.0,5.0,4.0)` 也完成闭环；B/C 最大跟踪误差为 `0.030705/0.030305 m`、最小净空为 `0.170595/0.168025 m`，均无碰撞、非有限值或饱和；
- 用户已在 RViz2 中观察静态避障完整执行，整体运动和绕障效果符合预期。精确碰撞净空、跟踪误差和最终误差仍以自动回归指标为准；
- 1.5 m 间距的三层地图可达性扫描共检查 231 个规则采样点，其中 51 个不在规划安全 workspace、48 个位于规划膨胀障碍物内；其余 132 个安全目标全部可达，不可达目标为 0，可达率 `100%`；
- 默认单目标闭环由 72 个原始路径点经 3 次局部细化形成 18 点、17 段，总轨迹 `62.768339 s`，任务于 `68.757 s` 完成；最大跟踪误差 `0.030305 m`、最小基础净空 `0.089736 m`、最终误差 `0.006014 m`，无碰撞或饱和；
- 最新 Domain 114 正式多目标评测从实际地面 Odom 原地起飞，依次执行 P1、P2 和 P3；Launch 后 `142.388 s` 完成，导航执行 `136.762 s`，三段从轨迹启动到目标接受分别为 `63.749/39.298/32.196 s`。最大跟踪误差 `0.027957 m`，最小基础净空 `0.089363 m`；起飞瞬态实际速度峰值 `1.016178 m/s`，三段导航实际速度峰值为 `0.559882/0.636064/0.506316 m/s`，参考速度/加速度峰值为 `0.633128 m/s`、`0.344052 m/s²`；四电机范围 `0.0–13067.5 RPM`，完成后保持 `10811.1–10827.2 RPM` 悬停；最终误差 `0.004183 m`、最终速度 `0.001692 m/s`，无碰撞、非有限值或饱和；
- 当前远程首目标与两段偏置返航任务已由 Domain 114 正式评测确认严格按序执行；任务结束时绿色实际 Path 仍保留从起飞点开始的 `1451` 个历史点，当前六障碍地图的定量验收以自动测试和 `metrics.json` 为准；
- 用户已在 RViz2 中人工确认自定义目标选择、完整预览、Execute、严格顺序导航和最终稳定悬停能够完成；
- RViz2 的 Orbit 焦点为 `(6.75,2.25,1.5)`、观察距离 `17.5 m`，可同时显示扩展后的完整工作空间、六个原始障碍物、透明基础安全膨胀区、无人机及四类路径；
- RViz2 显示无人机模型、TF、历史 Path 和目标 Pose；
- 控制器退出后约 `0.30 s` 触发 MotorRPM watchdog，目标转速归零；控制器重启并重新发送目标后闭环恢复；
- 悬停抗扰最终采用 `+x 0.30 N × 2.0 s`：最大位置误差/水平偏移 `0.350191 m`，最大速度 `0.188198 m/s`，最大 roll/pitch `0/0.033841 rad`，最大 RPM `10856.7`，恢复时间 `5.700 s`，最终误差 `0.008458 m`、最终速度 `0.005322 m/s`，无非有限值、姿态发散或控制器饱和；
- 当前完整测试结果为 `259 tests, 0 errors, 0 failures, 0 skipped`；`drone_bringup` 的 13 个 Launch 测试全部通过。交互执行 E2E 的三目标任务用时 `49.961 s`，最大导航跟踪误差 `0.021084 m`，最小基础净空 `0.241174 m`，最大电机转速 `13067.5 RPM`，最终误差 `0.007045 m`、最终速度 `0.002704 m/s`，无碰撞、非有限值或饱和。预检无路径测试在失败后继续观察 `3 s`：trajectory setpoint 数为 `0`，`404` 条 RPM 命令的最大绝对值为 `0`，最大高度和水平位移均为 `0 m`。

## 下一阶段

### 物理模型与控制飞行包线审计（2026-07）

当前刚体模型原本已经包含重力、四旋翼总推力、roll/pitch/yaw 力矩、刚体角速度耦合、电机一阶响应和独立外部集中力。本轮在机体系增加逐轴集总平动阻力，再旋转到世界系参与平动；同时增加机体系角阻尼力矩。正式系数为：

| 参数 | x / roll | y / pitch | z / yaw | 单位 |
|---|---:|---:|---:|---|
| 线性阻力 | `0.01` | `0.01` | `0.01` | N/(m/s) |
| 二次阻力 | `0.0` | `0.0` | `0.0` | N/(m/s)² |
| 角阻尼 | `0.01` | `0.01` | `0.02` | N·m/(rad/s) |

这些是适用于当前中低速仿真的集总系数，不是特定真实机型的风洞或飞行数据辨识结果，不能据此声称模型已经达到高保真空气动力学精度。扫描过的 `0.10–0.30 N/(m/s)` 粗粒度平动阻力虽然能产生明显自由衰减，但 `0.20` 在默认三目标任务中造成 `0.2500 m` 最大跟踪误差；在不增加积分、不修改轨迹和不放宽安全阈值的约束下，该候选被否决，随后细化低阻力区并选择 `0.01`。

执行能力审计得到：质量 `1.0 kg`，最大总推力 `33.5128 N`，推重比 `3.4174`，悬停 `10818.9 RPM`，为 `20000 RPM` 上限的 `54.10%`。`0.15 rad` 倾角保持高度时可实现 `1.4821 m/s²` 水平加速度，因此正式 `0.8 m/s²` 限制仍保留 `0.6821 m/s²`（约 `46%`）能力余量。roll/pitch/yaw 控制力矩上限分别为物理估算极限的 `42.2%/42.2%/8.8%`。动力学与控制器的质量、机臂、旋翼系数和 RPM 范围一致；Xacro 的 `0.20 m` 机臂几何一致，但它没有 inertial、RPM 或旋翼系数，当前只承担可视模型角色。

| 方案 | 最大水平加速度 | 最大倾角 | 选择结论 |
|---|---:|---:|---|
| 旧保守基线 | `0.4 m/s²` | `0.08 rad` | 2 m 阶跃和 `0.8 N` 扰动长期触发水平限幅 |
| 推荐平衡方案 | `0.8 m/s²` | `0.15 rad` | 首个消除水平阶跃限幅且保留充分倾角余量的候选 |
| 激进未采用 | `1.5 m/s²` | `0.20 rad` | 阶跃收益不可测，`0.8 N` 偏移仅再减少约 `0.021 m` |

推荐方案的 2 m 水平阶跃上升/稳定时间为 `4.885/8.993 s`，无超调，最大倾角 `0.10566 rad`，最大 RPM `12016.9`，水平/高度/姿态/Mixer 饱和为 `0/0/0/0`。`0.30 N × 2 s` 最大偏移 `0.3486 m`、恢复约 `4.655 s`；`0.80 N × 2 s` 最大偏移从旧基线 `2.0913 m` 降至 `0.9502 m`，恢复从 `9.410 s` 缩短至 `6.415 s`，100 Hz 水平限幅样本从 `465` 降至 `71`，且没有高度、姿态或 Mixer 饱和。`0.30 N × 10 s` 的最大/近稳态偏差约 `0.744 m`，符合无积分 PD 在恒力下保留稳态误差的预期。

正式三目标最终评测得到：完成时间 `139.309 s`，最大跟踪误差 `0.02633 m`，最小基础净空 `0.09474 m`，最大倾角 `0.03923 rad`，最大 RPM `13067.5`，控制饱和 `0`，最终误差 `0.00559 m`、最终速度 `0.00242 m/s`。地图、目标、A*、路径简化、连续轨迹、`0.35 m/s` 名义速度和 `0.35 m/s²` 最大参考加速度均未改变。

限制层级现在明确为：物理电机/旋翼能力是最外层，控制器推力、力矩、倾角和水平加速度是中间硬限制，轨迹的速度和参考加速度是正常任务的内层限制。控制器通过 `/drone/controller/diagnostics` 输出请求水平加速度、总推力、三轴力矩、裁剪前/后 RPM 和四级饱和标志。Mixer 仍使用简单逐电机裁剪；推荐工况没有 Mixer 饱和，因此本轮未引入复杂 desaturation。

上述物理审计阶段没有混入积分调参；随后独立完成了下面的水平积分升级。模型仍未包含旋翼陀螺效应、桨叶挥舞、地面效应、电池电压下降、电机非对称、传感器噪声、复杂阵风和空气密度随高度变化。

### 水平位置积分与 anti-windup（2026-07）

水平位置环现在是 PID 类控制：P/D/轨迹加速度前馈保持原结构，新增的积分状态直接存储世界系水平加速度贡献 `integral_acceleration_world`，单位为 `m/s²`，而不是累计米数。正式参数为 `Ki_x=Ki_y=0.15 s⁻²`、积分加速度向量上限 `0.35 m/s²`、`Kaw=2.0`、捕获半径 `0.50 m`、大目标跳变 reset 距离 `1.0 m`；`Kp=0.4`、`Kd=1.2`、最大水平加速度 `0.8 m/s²` 和最大倾角 `0.15 rad` 未改变。

常规 anti-windup 使用 back-calculation：`integral_dot = Ki·position_error + Kaw·(a_saturated-a_raw)`，水平输出饱和时冻结误差积分但允许 residual 回算；积分向量另有独立限幅。节点在启动、无有效目标、地面待机、大 pose goal 跳变、无效状态和控制失效时 reset；Odom 短时超时、捕获半径外、水平饱和及快速轨迹阶段 freeze。连续 trajectory 采样不会被当成目标跳变。撤力后的主动卸载只依据控制器内部的确定性条件 `position_error·integral_acceleration < 0`，以独立的 `horizontal_integrator_unload_gain=2.0` 向零衰减，不依赖外力 Topic、Odom 二阶差分或固定时间窗。诊断分别发布 `horizontal_saturation_backcalc_active` 与 `horizontal_integrator_unloading_active`；旧 `horizontal_anti_windup_active` 仅作为两者逻辑或的兼容汇总字段。

本轮确定性卸载回归：`0.30 N × 2 s` 最大偏移/恢复为 `0.33145 m / 8.066 s`；`0.80 N × 2 s` 为 `0.93701 m / 7.536 s`，水平饱和和 saturation back-calculation 各 `154` 个采样，高度/姿态/Mixer 均无饱和。三次独立 `0.30 N × 10 s` 撤力实验的反向超调为 `0.1077668/0.1077669/0.1077631 m`，恢复时间为 `4.60074/4.60105/4.60058 s`，极差分别约 `0.0000039 m / 0.00047 s`；三次最终误差约 `0.00273 m`、最终速度约 `0.00165 m/s`。

默认无扰动三目标仍只作为最终回归：完成 `139.204 s`，最大跟踪误差 `0.028843 m`，最小基础净空 `0.094310 m`，最终误差/速度 `0.001536 m / 0.004355 m/s`，控制饱和为零。动力学质量改为 `1.10/1.20 kg` 而控制器仍按 `1.00 kg` 时，水平保持不漂移，但高度分别稳定在 `1.1731/0.8462 m`；水平积分没有解决高度质量失配。高度环和姿态环仍是 PD，不能把整个飞控称为完整 PID。外力仍是集中等效力，不是障碍地图中的完整风场。

最终全量回归为 `282 tests, 0 errors, 0 failures, 0 skipped`；交互导航 E2E 与预检失败测试均由本轮真实 xUnit XML 确认通过。结构化汇总记录了所有数据来源路径、生成时间和 Git revision；不存在来源的指标不会自动伪造为通过。

- 若需要进一步缩短任务时间，评估转角/净空相关的分段速度规划；
- 航迹跟踪抗扰和更长时间极限工况仍属于后续可选验证；
- 局部规划、动态障碍与在线重规划属于可选扩展，不作为当前主线必做内容。

`/drone/path` 是动力学实际状态的历史位姿；`/drone/planned_path` 是 A* 原始栅格路径；`/drone/simplified_path` 是视线简化折线；`/drone/reference_path` 是连续参考轨迹。四者用途不同；只显示 Launch 不驱动无人机，静态避障 Launch 才启用规划轨迹执行。

## 项目结构

```text
ros2_drone_sim/
├── README.md
├── docs/
│   ├── AI_CONTEXT.md
│   └── ai_usage.md
├── src/
│   ├── drone_msgs/
│   ├── drone_dynamics/
│   ├── drone_controller/
│   ├── drone_mission/
│   ├── drone_planning/
│   └── drone_bringup/
├── tools/
├── results/
└── report/
```

- `drone_msgs`：项目自定义 ROS2 消息；
- `drone_dynamics`：纯动力学模型、ROS2 动力学节点及单元测试；
- `drone_controller`：高度、姿态、Mixer、HoverController 和 ROS2 控制节点；
- `drone_mission`：与 ROS2 无关的顺序 WaypointManager、C² 分段五次轨迹、对应任务节点及单元测试；
- `drone_planning`：静态 AABB 环境、点/线段碰撞检查、A*、视线简化、安全轨迹组合层及对应 ROS2 节点；
- `drone_bringup`：参数、Launch、Xacro 和 RViz2 配置；
- `tools`：辅助测试工具；
- `results`、`report`：实验结果和报告预留目录。

## 环境与依赖

已验证环境：

- Ubuntu 22.04.5；
- ROS2 Humble；
- g++ 11.4.0，C++17；
- CMake 3.22.1；
- Eigen3 3.4.0；
- colcon、rosdep、ament_cmake、tf2、RViz2 和常用 ROS2 消息包。

## 编译与运行

```bash
cd ~/ros2_drone_sim
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

source install/setup.bash
ros2 launch drone_bringup basic_sim.launch.py
```

默认 Launch 会启动：

- `/quadrotor_dynamics_node`；
- `/position_controller_node`；
- `/robot_state_publisher`；
- `/rviz2`。

无图形界面启动：

```bash
ros2 launch drone_bringup basic_sim.launch.py use_rviz:=false
```

启动默认五点顺序任务（默认包含 RViz2）：

```bash
ros2 launch drone_bringup mission_sim.launch.py
```

任务配置位于 `src/drone_bringup/config/mission.yaml`，无图形界面时可增加 `use_rviz:=false`。

启动默认五点连续轨迹（默认包含 RViz2）：

```bash
ros2 launch drone_bringup trajectory_sim.launch.py
```

轨迹配置位于 `src/drone_bringup/config/trajectory.yaml`，默认四段各 `6.0 s`。节点先在 P0 连续稳定 `1.0 s`，再按稳态时钟推进轨迹；Odom 超时或无效时暂停轨迹时间。无图形界面时同样可增加 `use_rviz:=false`。

启动静态三维环境（不会自动启动 waypoint 或轨迹任务）：

```bash
ros2 launch drone_bringup environment_sim.launch.py
```

环境配置位于 `src/drone_bringup/config/environment.yaml`。默认 workspace 为 `[-1,14.5] × [-2.5,7.0] × [-0.5,5] m`，三个在安全膨胀后连接边界的高墙与三个内部独立高障碍物形成水平蛇形路线和局部挑战通道；默认 A* 契约为 `(0,0,1.5) → (13.2,5.5,1.5)`，分辨率保持 `0.25 m`。`/drone/environment/markers` 显示工作空间、原始障碍物和透明基础安全膨胀区；`/drone/environment/in_collision` 只在收到有限且未超时的 Odom 时发布。无图形界面时可增加 `use_rviz:=false`。

启动默认三维 A* 规划与显示：

```bash
ros2 launch drone_bringup planning_sim.launch.py
```

规划参数位于 `src/drone_bringup/config/astar.yaml`；环境几何仍只来自共用的 `environment.yaml`。静态环境 Marker 和碰撞状态使用 `0.25 m` 基础无人机安全半径，A* 通过 `planning_margin=0.10 m` 使用 `0.35 m` 有效规划半径，为后续轨迹跟踪和执行预留额外净空。节点启动时规划一次，并以 transient-local QoS 发布 `/drone/planned_path`、`/drone/planning/success` 和 `/drone/planning/expanded_nodes`。当前发布的是未经简化或平滑的原始栅格路径，无人机不会自动沿该路径飞行。

可重复扫描地图自由空间可达性：

```bash
ros2 run drone_planning map_reachability_check --ros-args \
  --params-file src/drone_bringup/config/environment.yaml \
  --params-file src/drone_bringup/config/astar.yaml
```

该工具默认以 `1.5 m` 网格在 `z=1.5/2.5/4.0 m` 三层采样，区分安全 workspace 外、规划膨胀障碍物内、安全可达和安全不可达目标；它只调用现有 A* 做离线检查，不接入飞控链路。

启动原始路径、简化折线和经过验证的连续参考轨迹显示：

```bash
ros2 launch drone_bringup planned_trajectory_sim.launch.py
```

规划轨迹配置位于 `src/drone_bringup/config/planned_trajectory.yaml`，环境几何与安全参数继续来自 `environment.yaml`，规划裕量继续来自 `astar.yaml`，没有在新配置中复制。节点只处理收到的第一条合法原始路径；默认以 `0.35 m/s` 名义速度生成各段时间，依次尝试 `[1.0, 0.75, 0.5, 0.25, 0.0]`，并以 `0.02 s` 周期验证速度、加速度和 `0.35 m` 有效半径下的碰撞安全。若初始简化曲线发生拐角切入，构建器最多进行 8 轮、每轮最多补入 3 个原始 A* 局部点；若仅动态约束失败，则尝试分段时间比例 `[1.0,1.05,1.10,1.15,1.20,1.25,1.5,2.0,3.0,4.0]`。这些保底均不修改 A*、安全半径或碰撞条件。默认 `execution_enabled=false`，因此该命令不发布 `/drone/trajectory_setpoint`。

启动完整静态避障执行（默认包含 RViz2）：

```bash
ros2 launch drone_bringup static_avoidance_sim.launch.py
```

该 Launch 不包含固定五点的 `trajectory_sim.launch.py`，而是直接启动动力学、轨迹模式控制器、robot_state_publisher、可选 RViz2、静态环境、A* 和启用执行的 `planned_trajectory_node`。无人机先持续跟踪规划起点；位置误差 `<0.20 m`、线速度 `<0.15 m/s` 连续保持 `1.0 s` 后才启动轨迹时钟。Odom 无效或超过 `0.25 s` 时轨迹时间暂停，结束后持续发布最终位置与零速度、零加速度。执行状态通过 `/drone/planned_trajectory/current_segment` 和 `/drone/planned_trajectory/complete` 发布。

`static_avoidance_sim.launch.py` 的可选 `astar_config` 参数默认仍指向 `config/astar.yaml`，因此原启动命令不变。三个评测场景分别使用 `astar_evaluation_scenario_a.yaml`、`astar_evaluation_scenario_b.yaml` 和 `astar_evaluation_scenario_c.yaml`，避免运行时改写默认配置。顺序执行完整评测：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/evaluate_static_avoidance.py
```

评测工具为三个场景分别使用 Domain `110/111/112`，逐一启动并关闭完整 Launch，在 `results/static_avoidance/<scenario_name>/` 保存 `metrics.json`、`trajectory.csv`、`xy_path.png`、`position_tracking.png`、`tracking_error.png` 和 `clearance.png`。它是较长的实验工作流，不属于默认 `colcon test`；Domain 99 端到端测试仍是快速、确定性的核心安全回归。

启动有序多目标静态避障（默认包含 RViz2）：

```bash
ros2 launch drone_bringup multi_goal_static_avoidance_sim.launch.py
```

默认任务配置 `multi_goal_mission.yaml` 依次包含 `(13.2,5.5,1.5)`、`(7.0,5.0,4.0)` 和 `(0.8,0.7,2.0)` 三个零 yaw 目标：P1 一次完整穿越地图到右上远端，P2 返回中部上侧高点，P3 偏置返航到起飞区附近。目标数量没有写死为三个；配置支持任意非空数量的 `[x,y,z,yaw]` 分组，当前节点显式要求 yaw 为零。任意数量的合法目标都可以配置并按序尝试规划；每个目标仍必须满足几何安全、A* 可达和连续轨迹验证。可用 `mission_config:=<绝对路径>` 载入独立任务 YAML。节点启动时一次性读取目标列表，以首次有效 Odom 的 x/y 为起飞锚点，先发布 `z=1.5 m` 的静止 setpoint；位置与速度连续稳定 `1.0 s` 后，才从当时实际 Odom 规划 P1。每个目标同样采用“到达、停稳、切换”语义，Odom 无效或超时会暂停当前轨迹时钟，最终持续保持最后目标。任务状态可通过 `/drone/multi_goal/current_goal_index`、`current_segment`、`complete`、`success` 和 `visited_goals` 五个 Topic 观察。

绿色 `/drone/path` 实际轨迹默认以 `10 Hz` 采样并最多保留 `6000` 点，可覆盖约 10 分钟飞行；多目标 E2E 会验证长任务结束后首个起飞轨迹点仍然存在，防止历史缓存配置回退。

RViz 默认开启绿色实际轨迹、蓝色连续参考轨迹、全部多目标 Marker 和当前目标 Pose；黄色 A* 原始路径与粉色简化折线保留为可选显示但默认关闭，避免考核演示时遮挡绿色运行轨迹。`/drone/multi_goal/goal_markers` 将未访问目标显示为黄色、当前目标显示为放大的橙红色、已完成目标显示为绿色，并附带 Pn 状态标签和任务状态文字；`/drone/multi_goal/current_goal_pose` 只表示当前多目标任务目标。两者都采用 Reliable、Transient Local、Depth 1 QoS。状态文字以 `5 Hz` 更新：`Actual` 是最新有效 Odom 的三维实际速度模长，`Reference` 是当前连续轨迹采样的参考速度模长，`Nominal` 是轨迹分段时间计算使用的配置基线；`nominal_speed=0.35 m/s` 不表示飞行器全程恒速 `0.35 m/s`。Odom 无效或超时时 `Actual` 显示 `--`，非执行状态的 `Reference` 显示 `0.00 m/s`。进入最终完成态时，节点一次性向 `/drone/planned_path`、`/drone/simplified_path` 和 `/drone/reference_path` 发布瞬态保留的空 Path，清除规划辅助线；绿色 `/drone/path` 实际历史轨迹不被清除。需要分析规划细节时可在 Displays 面板手动勾选。

完整评测默认任务：

```bash
python3 tools/evaluate_multi_goal_mission.py --timeout 200
```

工具直接读取正式 `multi_goal_mission.yaml`、`environment.yaml` 和 `dynamics.yaml`，使用 ROS Domain 114。结果保存在 `results/multi_goal_evaluation/default_mission/`：`launch.log` 是完整节点日志，`metrics.json` 是任务级和逐段指标，`trajectory.csv` 是 Odom、参考、误差、净空、状态与四电机对齐时间序列；`xy_path.png`、`position_tracking.png`、`speed_tracking.png`、`tracking_error.png`、`clearance.png`、`motor_rpm.png` 和 `mission_summary.png` 分别用于路径、位置、速度、误差、安全净空、电机和报告综合展示。

导航地板 `0.50 m` 是规划阶段的安全球心最低高度，与动力学地面接触不是同一概念：起飞竖直段单独使用原始环境检查，空中各段使用原始 workspace 最低 z 加 `0.35 m` 有效半径得到的安全地板。多目标任务名义速度已由 `0.25 m/s` 保守提高到 `0.35 m/s`；同时将公共确定性时间比例候选从 `[1.0,1.25,1.5,2.0,3.0,4.0]` 细化为 `[1.0,1.05,1.10,1.15,1.20,1.25,1.5,2.0,3.0,4.0]`。最终三段选择的 duration scale 为 `1.00/1.00/1.10`，轨迹时间为 `62.754/38.316/31.213 s`；旧基线本轮复测 `183.894 s`，最终全量回归 `142.175 s`，缩短 `41.719 s`（`22.69%`）。地图、目标、`0.25 m` 基础安全半径、`0.10 m` 规划裕量、`0.35 m/s²` 最大参考加速度、控制器能力和 A* 均未修改。本轮属于统一名义速度与时间比例的保守参数优化；完整扫描摘要保存在 `results/speed_optimization/`。

### 外部力扰动与悬停抗扰

外部扰动默认关闭，现有 Launch 的数值语义不变。动力学节点可选订阅 `/drone/external_wrench`（`geometry_msgs/msg/WrenchStamped`）：第一版只接受 `header.frame_id=map`，`force` 作为作用在质心上的世界坐标系外力，方向按 map 正方向加入平动方程；`torque` 必须全零。输入必须有限，外力模长不得超过 `max_external_force=2.0 N`，超限、NaN/Inf、非法 frame 或非零 torque 会整条拒绝并节流警告。合法非零输入超过 `external_wrench_timeout=0.20 s` 未刷新后自动归零，不会永久保留。

`/drone/external_wrench/active`（`std_msgs/Bool`）和 `/drone/external_wrench/applied`（`WrenchStamped`）使用 Reliable、Transient Local、Depth 1，表示当前未超时的扰动状态和实际通过检查的 map 系外力；超时或明确零输入后发布 inactive 和零力。悬停演示启动：

```bash
ros2 launch drone_bringup disturbance_hover_sim.launch.py
```

该 Launch 只在本实验中设置 `enable_external_wrench=true`，复用现有 `/drone/goal` 位置控制链自动起飞到 `(0,0,1.5)`。稳定后可在另一终端周期施力，工具在结束或 Ctrl+C 时额外发布零 Wrench：

```bash
source install/setup.bash
python3 tools/apply_external_wrench.py \
  --force-x 0.30 --force-y 0.0 --force-z 0.0 \
  --duration 2.0 --rate 20
```

自动评测使用独立 Domain 119，等待稳定悬停、记录基线、施力、撤销、判定连续 `1.0 s` 的恢复并关闭 Launch：

```bash
source install/setup.bash
python3 tools/evaluate_hover_disturbance.py --timeout 45
```

最终默认选择 `0.30 N × 2.0 s`。初始候选 `0.80 N × 2.0 s` 会超过现有约 `0.4 m/s²` 水平控制能力，实测水平偏移 `2.087771 m`、恢复 `10.679879 s` 并出现 5 条饱和日志，因此保留在 `results/hover_disturbance/candidates/force_0p8_duration_2p0/`，没有通过修改控制器迎合测试。最终通过结果位于 `results/hover_disturbance/default/`，包含 `launch.log`、`metrics.json`、`trajectory.csv`、`position_error.png`、`position_xyz.png`、`velocity.png`、`attitude.png`、`motor_rpm.png`、`external_force.png` 和 `disturbance_summary.png`。

本功能仅模拟集中作用于质心的可控外力，不等同于真实空气动力学风场；当前没有空间变化风场、随机阵风模型或非零外力矩支持。动力学另有独立的集总平动空气阻力和角阻尼模型。

带完整 RViz 标记和阶段状态的一键入口为：

```bash
ros2 launch drone_bringup disturbance_visual_demo.launch.py profile:=short_gust
```

`short_gust` 默认施加 `+X 0.30 N × 2 s` 并恢复观察 `10 s`；`persistent_release` 默认施加 `+X 0.30 N × 10 s` 并在撤力后观察 `10 s`。两者均复用正式 `dynamics.yaml` 和 `controller.yaml`；正常基础、任务和避障 Launch 继续默认关闭外力，只有专用扰动实验入口显式覆盖 `enable_external_wrench=true`。节点先持续发布 `(0,0,1.5)` 悬停目标，收到有效 Odom 且位置误差、速度分别连续满足 `<0.05 m`、`<0.05 m/s` 至少 `1 s` 后才开始倒计时。阶段依次为 `WAIT_FOR_ODOM → TAKEOFF_AND_SETTLE → COUNTDOWN → DISTURBANCE_ACTIVE → RECOVERY → COMPLETE`；完成后仍持续发布零外力。

专用 RViz 配置预先打开 RobotModel、TF、Odom、绿色实际轨迹、悬停目标和 `/drone/disturbance/markers`。红色箭头是作用于质心的 **Equivalent External Force**，长度与 N 制等效集中外力成比例；蓝色箭头仅是水平位置积分产生的加速度补偿，单位语义为 `m/s²`，不是实际作用力。黄色细线连接实际位置和目标；上方文字显示阶段、等效力、剩余扰动时间、水平误差、水平积分加速度及 horizontal/altitude/attitude/mixer 四级饱和。零外力和近零积分时相应箭头会删除，不会构造零向量方向。

### RViz 三维多目标编辑、预览与执行

有障碍地图中的目标可以用独立编辑器直观选择：

```bash
ros2 launch drone_bringup interactive_goal_editor_sim.launch.py
```

该 Launch 只启动静态环境、`interactive_goal_editor_node` 和 RViz2，不启动控制器、动力学、默认 P1/P2/P3 任务，也不发布 `/drone/trajectory_setpoint` 或 `/drone/motor_rpm_cmd`。它显式使用 `execution_enabled=false`，不创建执行服务客户端，右键菜单不显示 Execute，启动日志显示 `preview only`。第一版固定从参数 `planning_start=[0,0,1.5]` 预览，只编辑、验证和显示，不执行飞行。无障碍位置控制实验仍使用终端向 `/drone/goal` 发布目标。

RViz 中用水平控制面调整世界坐标 x/y，用竖直箭头调整 z；释放鼠标后坐标按 `0.05 m` 吸附。两个模式都有 `Add Goal`、`Undo Last Goal`、`Clear All Goals`、`Set Height`（`1.5/2.5/4.0 m`）、`Validate & Preview` 和 `Print Mission YAML`；只有执行模式显示 `Execute Validated Mission`。中间目标按 Add 顺序成为 P1、P2……；移动到最后一个目标后可直接选择 `Validate & Preview`，编辑器会先把与上一目标不同的当前候选自动加入列表，再验证完整序列。如果最后一点已经执行过 Add，则不会重复添加。默认最多 8 个且逻辑不依赖目标数为 3。绿色候选表示几何合法，红色表示非法，黄色表示拖动编辑中，蓝色表示完整验证中；READY 后固定目标全部为绿色，失败段目标为红色。

快速检查在释放鼠标、设置高度和 Add 时执行，只检查有限坐标、`0.50 m` 导航地板、`0.35 m` 规划安全 workspace 与规划膨胀障碍物。`Validate & Preview` 则在后台依次验证 `planning_start→P1→P2→...` 的 A*、路径简化、连续轨迹生成、速度/加速度限制以及轨迹点和相邻采样线段碰撞。几何合法不保证完整序列一定可规划；只有完整验证通过才进入 READY。移动候选、添加、撤销或清空目标会立即令旧预览失效，之后必须重新点击 `Validate & Preview`。

编辑器状态均使用 Reliable、Transient Local、Depth 1：

- `/drone/interactive_goals/selected_goals`：按 Pn 顺序的 `PoseArray`，零 yaw；
- `/drone/interactive_goals/goal_markers`：固定目标和 Pn 标签；
- `/drone/interactive_goals/preview_path`：成功验证后的完整多段连续轨迹预览；
- `/drone/interactive_goals/status`：EDITING、VALIDATING、READY 或带具体段/原因的 REJECTED；
- `/drone/interactive_goals/ready`：当前草稿是否已完整验证；
- `/drone/interactive_goals/count`：已确认目标数量。

Interactive Marker update Topic 为 `/drone/interactive_goals/goal_editor/update`。完整验证成功后选择 `Print Mission YAML`，日志会打印可复制的 `goals: [...]` 平铺列表；它不会直接修改正式任务配置。任何目标变化后，必须重新完整验证，之后才能再次打印 YAML。

需要从同一个 RViz 流程实际飞行时启动：

```bash
ros2 launch drone_bringup interactive_goal_navigation_sim.launch.py
```

无人机在收到执行请求前保持地面且电机命令为零。对中间目标依次执行 Add，移动到最后一个目标后直接选择 `Validate & Preview`；确认目标数量和 READY 状态后再选择 `Execute Validated Mission`。编辑器通过 `/drone/interactive_goals/execute` 提交包含完整 `PoseArray` 和 `draft_revision` 的不可变快照；执行节点不会直接播放编辑器的预览线，而是等待最新实际 Odom，从实际地面 x/y 的起飞锚点对整个序列再次异步执行 A* 和连续轨迹预检。全部段预检成功后才起飞，正式执行每一段仍从该段开始时的实际 Odom 重新规划。

执行 Launch 显式使用 `execution_enabled=true`，会创建执行客户端、显示 Execute 菜单，并在启动日志显示 `preview and execution enabled`。服务返回 `accepted=true` 只代表不可变快照已接收并进入异步预检。若地面预检失败，执行节点不会发布任何 trajectory setpoint，控制器持续发布零 RPM，无人机保持地面；若飞行开始后失败，则任务停止并持续向最近的有限安全 Odom 位置发布零速度、零加速度 hold。两类失败都发布 `active=false`、`success=false`、`complete=false`，并清空 planned、simplified 和 reference 辅助规划线。

请求接受后编辑器隐藏候选、固定目标和预览线，并锁定拖动、Add、Undo、Clear、Validate 与再次 Execute；执行目标由已有多目标 Marker 独占显示，Print Mission YAML 仍可使用。任务结束后持续悬停，绿色实际历史轨迹保留，辅助规划线清空。当前第一版每次 Launch 只接受一份任务；若目标非法、Odom 超时或完整序列无路径，不会开始部分飞行，应重启该 Launch 后重新编辑。不要把 READY 理解为对任意实际起点都可执行的保证。

执行状态使用 Reliable、Transient Local、Depth 1 Topic：

- `/drone/interactive_mission/active`：预检和执行期间为 true，完成或失败后为 false；
- `/drone/interactive_mission/status`：等待、预检、起飞、规划、执行、保持、完成或具体失败原因；
- `/drone/interactive_mission/draft_revision`：执行节点实际冻结的草稿 revision。

人工演示时应确认无目标阶段保持地面、未 READY 的 Execute 被拒绝、READY 后预检通过才起飞、Pn 严格按序变绿、绿色实际轨迹持续增长、完成后蓝色参考线消失并在末目标稳定悬停。服务和状态可辅助检查：

```bash
ros2 service list | grep interactive
ros2 topic echo /drone/interactive_mission/status
ros2 topic echo /drone/multi_goal/visited_goals
```

常用检查：

```bash
ros2 node list
ros2 topic list
ros2 topic echo /drone/odom --once
ros2 run tf2_ros tf2_echo map base_link
```

## 目标发布

`/drone/goal` 使用 `geometry_msgs/msg/PoseStamped`，属于旧 waypoint/直接位置目标链路，不用于多目标列表显示，也不驱动 `multi_goal_static_avoidance_node`。多目标显示使用独立的 `/drone/multi_goal/goal_markers` 和 `/drone/multi_goal/current_goal_pose`。例如向直接位置目标链路发布 `1.5 m` 高度、零 yaw 目标：

```bash
ros2 topic pub --once /drone/goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}"
```

当前控制器使用目标 x/y/z 和四元数中的 yaw。目标 frame 支持空字符串或 `map`。

例如发布已验收的单目标三维位置：

```bash
ros2 topic pub --once /drone/goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0, z: 1.5}, orientation: {w: 1.0}}}"
```

RViz2 的 SetGoal 工具也发布到 `/drone/goal`，但 2D 操作不便于精确指定高度，因此高度验收更适合使用终端命令。

上述 `/drone/goal` 方式继续用于无障碍、直接位置控制实验；六障碍地图中的一个或多个候选目标建议先使用 RViz 三维编辑器做完整路径预览。预览成功并不自动接入飞行执行。

`trajectory_sim.launch.py` 将控制器切换为 `setpoint_source=trajectory`，订阅 `/drone/trajectory_setpoint`。该模式使用消息中的 position、velocity、acceleration 和 yaw；轨迹消息无效或超过 `0.20 s` 未更新时发布零 RPM，不回退到 `/drone/goal`。

## 安全保护与参数基线

正常 Launch 的主要参数位于：

- `drone_bringup/config/dynamics.yaml`；
- `drone_bringup/config/controller.yaml`。

当前稳定控制基线：

- 控制频率：`100 Hz`；
- 动力学频率：`200 Hz`；
- 高度：`Kp=3.0`、`Kd=3.5`；
- 水平位置：`Kp=0.4`、速度 `Kd=1.2`、`Ki=0.15`、积分加速度上限 `0.35 m/s²`、最大加速度 `0.8 m/s²`、最大倾角 `0.15 rad`；
- roll/pitch 姿态：`Kp=4.0`、角速度 `Kd=0.35`、最大力矩 `1.0 N·m`；
- yaw：`Kp=1.0`、`Kd=0.40`、最大力矩 `0.20 N·m`；
- 名义稳态悬停转速：约 `10818.9 RPM/电机`；
- 地面约束：`enable_ground_contact=true`、`ground_z=0.0 m`；
- MotorRPM watchdog：`enable_motor_command_timeout=true`、`motor_command_timeout=0.30 s`。

静态避障安全模型明确分为三层：无人机基础安全半径 `0.25 m`、规划额外裕量 `0.10 m`、A* 与连续轨迹共用的有效规划半径 `0.35 m`。URDF 中电机中心距机心 `0.20 m`、旋翼半径 `0.065 m`，因此 `0.25 m` 球形模型接近实际水平包络且不是明显过度保守；运行时检查确认规划裕量只相加一次。RViz 透明膨胀区和实时碰撞 Topic 显示基础 `0.25 m`，规划路径与轨迹验证使用 `0.35 m`。

配置目标位于有效膨胀障碍物内或安全 workspace 外时会在启动/规划阶段拒绝；目标本身安全但不存在路径时，A* 发布失败且执行端保持当前安全位置。当前尚未实现动态障碍物、局部规划或在线重规划。

控制器在没有有效目标、Odom 缺失或超时、frame/四元数非法、算法结果无效时主动发布零 RPM。动力学 watchdog 在已收到过命令但后续超时后，只把目标 RPM 设为零；电机实际转速继续按既有一阶模型自然衰减。

## 人工验收流程

按 A→H 顺序人工确认；全部通过后只执行一次 I。每个 Launch 结束时在对应终端按 `Ctrl+C`，不要让两个仿真 Launch 同时运行在同一 ROS Domain。

### A. 编译与环境

测试目的：确认工作区可构建且六个本地 ROS 包可被发现。

终端 1：

```bash
cd /home/peter/ros2_drone_sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | grep '^drone_'
```

终端 2：不需要。RViz：不需要。终端应看到六个包构建成功，列表包含 `drone_bringup`、`drone_controller`、`drone_dynamics`、`drone_mission`、`drone_msgs`、`drone_planning`。通过标准：构建退出码为 0 且六包齐全。停止方式：命令自行结束。

### B. 基础悬停

测试目的：确认正式基础 Launch 的无扰动位置闭环保持原行为。

终端 1：

```bash
cd /home/peter/ros2_drone_sim
source install/setup.bash
ros2 launch drone_bringup basic_sim.launch.py use_rviz:=true
```

终端 2：

```bash
source /home/peter/ros2_drone_sim/install/setup.bash
ros2 topic pub --once /drone/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}"
ros2 topic echo /drone/motor_rpm_cmd --once
ros2 param get /quadrotor_dynamics_node enable_external_wrench
```

RViz 应观察到：无人机从地面起飞至 `z=1.5 m`，水平位置稳定、无明显振荡；正式基础配置中没有扰力 Marker 显示。终端应观察到：四个 RPM 有限且悬停时不触及上下限，参数值为 `False`。通过标准：稳定悬停、RPM 有限、无外力箭头且外力输入关闭。停止方式：终端 1 `Ctrl+C`。

### C. 短时扰动可视化

测试目的：验证短阵风、Marker 语义和撤力恢复。

终端 1：

```bash
cd /home/peter/ros2_drone_sim
source install/setup.bash
ros2 launch drone_bringup disturbance_visual_demo.launch.py \
  profile:=short_gust force_x:=0.30 disturbance_duration:=2.0 recovery_duration:=10.0
```

终端 2：

```bash
source /home/peter/ros2_drone_sim/install/setup.bash
ros2 topic echo /drone/external_wrench/applied
```

RViz 应观察到：稳定后倒计时；扰动期间红色 Equivalent External Force 箭头沿 `+X` 出现，无人机同向偏移，蓝色水平积分加速度箭头逐渐出现；文字为 `ACTIVE`、`[0.30, 0.00, 0.00] N`；撤力后为 `RECOVERY`，红箭头删除并最终回到目标附近。终端应观察到：阶段只在切换时打印，外力从零变为 `0.30 N` 再连续归零，无 Mixer 饱和日志。通过标准：阶段、箭头和 Topic 同步，恢复完成、无残余红箭头、无 NaN/Inf、`Mix=0`。停止方式：两个终端均 `Ctrl+C`。

### D. 持续扰动与撤力恢复

测试目的：观察水平积分在恒定扰力下建立并在撤力后卸载。

终端 1：

```bash
cd /home/peter/ros2_drone_sim
source install/setup.bash
ros2 launch drone_bringup disturbance_visual_demo.launch.py \
  profile:=persistent_release force_x:=0.30 disturbance_duration:=10.0 recovery_duration:=10.0
```

终端 2：

```bash
source /home/peter/ros2_drone_sim/install/setup.bash
ros2 topic echo /drone/controller/diagnostics
```

RViz 应观察到：红色外力箭头保持 10 s，蓝色积分补偿总体指向扰力反方向；持续扰动时偏差逐渐减小，撤力后蓝色箭头逐渐缩短，无严重反向冲击，最后接近目标。终端应观察到：`horizontal_i_acceleration_x` 建立后卸载，四类饱和尤其 `mixer_saturated` 保持 false。通过标准：补偿方向正确、撤力恢复平顺、最终误差收敛且无饱和/非有限值。停止方式：两个终端均 `Ctrl+C`。

### E. 静态多目标避障

测试目的：确认正式六障碍地图和三目标规划执行链不受扰动演示影响。

终端 1：

```bash
cd /home/peter/ros2_drone_sim
source install/setup.bash
ros2 launch drone_bringup multi_goal_static_avoidance_sim.launch.py use_rviz:=true
```

终端 2（先持续观察 `complete`；任务完成后用 `Ctrl+C`，再执行后两条）：

```bash
source /home/peter/ros2_drone_sim/install/setup.bash
ros2 topic echo /drone/multi_goal/complete
ros2 topic echo /drone/environment/in_collision --once
ros2 param get /quadrotor_dynamics_node enable_external_wrench
```

RViz 应观察到：地图、原始障碍物与透明膨胀区、目标、蓝色参考路径和绿色实际轨迹；无人机按 P1→P2→P3 飞行且不穿障。终端应观察到：任务最终 `complete: true`、碰撞始终 false、外力参数为 `False`，没有 `DISTURBANCE_ACTIVE`。通过标准：全部目标完成、无碰撞、无主动外力。停止方式：终端 1 `Ctrl+C`；终端 2 的连续 echo 用 `Ctrl+C`。

### F. RViz 交互目标导航

测试目的：验证三维目标编辑、完整预览、预检门控与安全执行。

终端 1：

```bash
cd /home/peter/ros2_drone_sim
source install/setup.bash
ros2 launch drone_bringup interactive_goal_navigation_sim.launch.py use_rviz:=true
```

终端 2（两条 `echo` 分别观察；用 `Ctrl+C` 结束第一条后再执行第二条）：

```bash
source /home/peter/ros2_drone_sim/install/setup.bash
ros2 topic echo /drone/interactive_goals/status
ros2 topic echo /drone/interactive_mission/status
```

RViz 操作：选择工具栏 `Interact`；拖动 `goal_candidate` 的水平面和竖直箭头指定 x/y/z；右键依次选 `Add Goal`，最后选 `Validate & Preview`；状态进入 READY 且蓝色预览完整后，右键选 `Execute Validated Mission`。安全候选为绿色；障碍物内部（例如 O1 中心附近）和 workspace 外候选为红色并拒绝；执行中观察蓝色参考路径、绿色实际轨迹和 Pn 状态。终端应观察到：安全序列进入预检并执行，非法目标给出具体拒绝原因；预检或路径失败时不启动飞行，飞行后失败则保持最近安全位置。通过标准：至少一组安全目标完成；障碍物内部与 workspace 外目标均拒绝；无路径时保持安全。停止方式：两个终端均 `Ctrl+C`。

### G. 预检失败安全测试

测试目的：用现有闭合墙测试场景复现“端点合法但无路径”的地面预检失败，并验证零 setpoint/零 RPM。

终端 1：

```bash
cd /home/peter/ros2_drone_sim
source install/setup.bash
colcon test --packages-select drone_bringup \
  --ctest-args -R test_interactive_preflight_failure --output-on-failure
colcon test-result --verbose
```

终端 2：不需要。RViz：该自动化安全入口不启动 RViz。终端应观察到：测试 PASS；节点日志明确报告 preflight 无路径失败。测试断言会继续观察失败后状态，确认有效 trajectory setpoint 数为 0、RPM 始终为 0、无人机高度和水平位置均不变化。通过标准：该测试无 failure/error，且日志/断言满足上述四项。停止方式：正常时自动结束；异常卡住时 `Ctrl+C`。

### H. Topic 和诊断检查

测试目的：核对扰动链及规划链真实 Topic、频率、状态和动力学外力开关。

先检查扰动链。终端 1：

```bash
cd /home/peter/ros2_drone_sim
source install/setup.bash
ros2 launch drone_bringup disturbance_visual_demo.launch.py use_rviz:=false profile:=short_gust
```

终端 2（以下命令逐条执行；`hz` 用 `Ctrl+C` 结束后再执行下一条）：

```bash
source /home/peter/ros2_drone_sim/install/setup.bash
ros2 topic list
ros2 topic echo /drone/odom --once
ros2 topic echo /drone/motor_rpm_cmd --once
ros2 topic echo /drone/controller/diagnostics --once
ros2 topic echo /drone/external_wrench/active --once
ros2 topic echo /drone/external_wrench/applied --once
ros2 topic echo /drone/disturbance/markers --once
ros2 param get /quadrotor_dynamics_node enable_external_wrench
ros2 topic hz /drone/odom
ros2 topic hz /drone/external_wrench
```

用 `Ctrl+C` 停止两个终端后，再检查规划链。终端 1：

```bash
source /home/peter/ros2_drone_sim/install/setup.bash
ros2 launch drone_bringup multi_goal_static_avoidance_sim.launch.py use_rviz:=false
```

终端 2：

```bash
source /home/peter/ros2_drone_sim/install/setup.bash
ros2 topic echo /drone/environment/in_collision --once
ros2 topic echo /drone/multi_goal/current_goal_index --once
ros2 topic echo /drone/multi_goal/success --once
ros2 topic echo /drone/multi_goal/complete --once
ros2 param get /quadrotor_dynamics_node enable_external_wrench
```

RViz：本项关闭 RViz，只核对通信。终端应观察到：消息字段有限；Odom 约 200 Hz、外力命令约 25 Hz；演示链参数为 `True`，规划链为 `False`；碰撞 false，任务状态 Topic 存在且更新。通过标准：所有命令使用的 Topic 均存在、消息可读、频率合理、两个 Launch 的开关隔离正确。停止方式：`ros2 topic hz/echo` 和 Launch 均用 `Ctrl+C`。

### I. 最终自动全量测试

测试目的：仅在 A～H 全部通过后执行合并 `main` 前的最终回归。

终端 1：

```bash
cd /home/peter/ros2_drone_sim
source install/setup.bash
colcon test
colcon test-result --verbose
```

终端 2：不需要。RViz：不需要。终端应观察到所有包测试结束并输出汇总。通过标准：`failures=0`、`errors=0`，并把 tests、failures、errors、skipped 四个数字记录在下表；不需要在每个手工测试后重复全量回归。停止方式：正常时自动结束，异常卡住时 `Ctrl+C`。

| 项目 | 结果 | 备注 |
|---|---|---|
| 编译 | PASS/FAIL | |
| 基础悬停 | PASS/FAIL | |
| 短时扰动 | PASS/FAIL | |
| 持续扰动与撤力 | PASS/FAIL | |
| 多目标静态避障 | PASS/FAIL | |
| RViz交互导航 | PASS/FAIL | |
| 预检失败安全 | PASS/FAIL | |
| Topic与诊断 | PASS/FAIL | |
| 全量测试 | PASS/FAIL | tests=；failures=；errors=；skipped= |

## 当前限制

- `WaypointManager` 多目标模式仍采用“到点并稳定后离散切换”的 stop-settle-switch 策略；连续飞行由独立的 `trajectory_sim.launch.py` 提供，不改变该语义；
- 地面模型只有质心 z 方向的无反弹、无摩擦刚性约束；
- 动力学包含集总平动空气阻力和角阻尼，但暂不包含旋翼陀螺效应和传感器噪声；
- 水平位置环已有带 back-calculation 和独立限幅的积分补偿；高度与姿态环仍无积分；
- 用户已在 RViz2 中人工确认五点任务能够依次到达，各 waypoint 附近停稳后再切换；当前仍是 stop-settle-switch，不是平滑轨迹；
- 用户已在 RViz2 中人工确认无人机能够沿连续参考轨迹完成五点飞行，中间 waypoint 不再停稳后切换，整体运动连续；
- 静态环境碰撞状态仍只报告几何结果，不会阻止电机或产生物理撞击响应；已验证的静态避障依靠规划轨迹与跟踪控制保持安全；
- 尚未实现局部规划、在线重规划或动态障碍物；
- 更大倾角、外部扰动、高角速度和最大 RPM 极限工况仍需继续验证。

## 文档说明

本 README 只记录稳定方案、已验证能力和可复现运行方式。供后续 AI 快速接手的当前技术上下文位于 `docs/AI_CONTEXT.md`，AI 使用记录位于 `docs/ai_usage.md`。

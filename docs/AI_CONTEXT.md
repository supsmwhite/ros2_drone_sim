# ROS2 四旋翼仿真当前上下文

本文只记录当前有效实现与最近验证结果。历史场景、旧坐标和开发过程请查 Git 历史。

## 仓库状态

- 仓库：`/home/peter/ros2_drone_sim`
- 当前开发分支：`feature/five-obstacle-scene`
- ROS2：Humble
- 下一阶段：如需继续提速，评估转角/净空相关的分段速度规划
- 当前不包含：动态障碍、局部规划、在线重规划

## 当前系统链路

项目已实现四旋翼刚体动力学、电机一阶响应、位置/姿态/高度控制、四电机混控、三维 A*、障碍物膨胀、连续线段碰撞检查、路径简化、分段五次连续轨迹、安全验证、单目标和多目标静态避障、RViz2 与自动化测试。

关键 Launch：

- `environment_sim.launch.py`：动力学、控制器、模型、RViz 和静态环境
- `planning_sim.launch.py`：静态环境与一次性 A*
- `planned_trajectory_sim.launch.py`：A*、简化路径和连续参考轨迹，不执行飞行
- `static_avoidance_sim.launch.py`：单目标闭环静态避障
- `multi_goal_static_avoidance_sim.launch.py`：三目标闭环静态避障
- `interactive_goal_editor_sim.launch.py`：RViz 三维有序目标编辑和完整轨迹预览；不启动飞控或动力学
- `interactive_goal_navigation_sim.launch.py`：RViz 编辑、实际 Odom 全序列预检和交互式顺序飞行
- `disturbance_hover_sim.launch.py`：启用可选外力输入、自动起飞至 `(0,0,1.5)` 并悬停

所有规划/避障 Launch 均从 `src/drone_bringup/config/environment.yaml` 读取同一地图，没有复制运行时障碍物配置。

## 外部力扰动与悬停抗扰

`QuadrotorModel::set_external_wrench()` 保持纯模型接口：force 在 map/world 中表达并加入世界系平动力，torque 在 body 中表达。ROS 第一版只允许 `frame_id=map`、有限且模长不超过 `2.0 N` 的 force，并要求 torque 全零；非法 frame、NaN/Inf、超限 force 和非零 torque 会拒绝整条消息。`enable_external_wrench` 默认 `false`，现有 Launch 不创建订阅且旧动力学数值不变；`disturbance_hover_sim.launch.py` 单独覆盖为 true。合法非零输入超过 `external_wrench_timeout=0.20 s` 后模型和状态 Topic 自动归零。

输入 Topic 为 `/drone/external_wrench`（`geometry_msgs/msg/WrenchStamped`）。状态 `/drone/external_wrench/active` 与 `/drone/external_wrench/applied` 使用 Reliable、Transient Local、Depth 1。周期人工工具：

```bash
python3 tools/apply_external_wrench.py --force-x 0.30 --duration 2.0 --rate 20
```

自动评测命令：

```bash
python3 tools/evaluate_hover_disturbance.py --timeout 45
```

Domain 119 最终实验使用 `+x 0.30 N × 2.0 s`，基线误差 `0.001424 m`，最大位置误差和水平偏移 `0.350191 m`，最大竖直误差 `0.000291 m`，最大速度 `0.188198 m/s`，最大 roll/pitch `0/0.033841 rad`，RPM 命令范围 `10784.1–10856.7`，饱和日志 `0`，恢复时间 `5.699643 s`，最终误差 `0.008458 m`、最终速度 `0.005322 m/s`，无 NaN/Inf 或姿态发散，验收通过。失败候选 `0.80 N × 2.0 s` 的最大水平偏移 `2.087771 m`、恢复时间 `10.679879 s`、饱和日志 `5`；候选结果被保留，控制器参数未修改。

最终文件位于 `results/hover_disturbance/default/`：`launch.log`、`metrics.json`、`trajectory.csv` 和七张要求图像；失败候选位于 `results/hover_disturbance/candidates/force_0p8_duration_2p0/`。该功能是集中外力注入，不是完整空气动力学；未实现空间变化风场、随机阵风、气动阻力和非零 torque。

## RViz 三维目标编辑器

`interactive_goal_editor_node` 负责编辑、预览和提交任务，不直接控制无人机。只读预览启动命令：

```bash
ros2 launch drone_bringup interactive_goal_editor_sim.launch.py
```

该 Launch 只包含静态环境、编辑器和 RViz，不包含控制器、动力学或默认多目标任务；节点不创建 `/drone/trajectory_setpoint` 和 `/drone/motor_rpm_cmd` Publisher。它显式设置 `execution_enabled=false`，因此不创建执行客户端、不显示 Execute 菜单，日志显示 `preview only`。第一版预览起点固定为 `planning_start=[0.0,0.0,1.5]`，所有目标 yaw 为零。无障碍直接位置实验继续从终端发布 `/drone/goal`。

活动 Interactive Marker 名为 `goal_candidate`，server update Topic 是 `/drone/interactive_goals/goal_editor/update`。它只有世界坐标固定的 `MOVE_PLANE` XY 控制面、世界 z 方向 `MOVE_AXIS` 箭头和右键菜单，没有旋转、`MOVE_3D` 或复合 3D 控件。释放鼠标时以 `0.05 m` 吸附；候选拖动中为黄色，快速几何合法为绿色，非法为红色，完整验证中为蓝色。两个模式都有 Add、Undo、Clear、高度 `1.5/2.5/4.0 m`、Validate & Preview 和 Print Mission YAML；仅 `execution_enabled=true` 显示 Execute Validated Mission。中间目标按 Add 顺序定义 P1、P2……；Validate 会把与最后一个已确认目标不同的当前合法候选先加入为末目标，再启动完整验证，已 Add 的末目标不会重复。配置上限默认 8，逻辑和测试覆盖 5 个目标而不依赖三目标硬编码。

快速检查复用 `environment.yaml` 的 `StaticEnvironment`、`CollisionChecker`、`safety_radius+planning_margin=0.35 m` 和 `minimum_navigation_altitude=0.50 m`，分别报告非有限坐标、导航地板、safe workspace 或规划膨胀障碍物错误。完整验证在异步任务中从固定起点逐段调用 `AStarPlanner` 和内部复用 `PathSimplifier` 的 `PlannedTrajectoryBuilder`；构建器继续验证速度、加速度、轨迹采样点和相邻采样线段。草稿 revision 防止旧异步结果覆盖已修改列表。READY 才允许打印 YAML；任何候选或列表变化会立即清空预览、令 ready=false，并要求重新验证。不能声称任意几何合法序列都可达。

编辑器的 Reliable、Transient Local、Depth 1 Topic：

- `/drone/interactive_goals/goal_markers`：固定目标；
- `/drone/interactive_goals/selected_goals`：按序零 yaw `PoseArray`；
- `/drone/interactive_goals/preview_path`：独立的完整连续轨迹预览，不覆盖 `/drone/reference_path`；
- `/drone/interactive_goals/status`、`ready`、`count`：明确状态、完整验证标志与数量。

交互式执行启动命令：

```bash
ros2 launch drone_bringup interactive_goal_navigation_sim.launch.py
```

该 Launch 仅建立一条执行链路：动力学、trajectory 模式控制器、模型、静态环境、`execution_enabled=true` 的编辑器、`goal_source=interactive` 的现有 `multi_goal_static_avoidance_node` 和 RViz。编辑器创建执行客户端、显示 Execute 菜单，日志显示 `preview and execution enabled`。没有 Execute 请求时执行节点状态为 `WAITING FOR VALIDATED MISSION`，无人机保持地面且不产生非零 RPM。READY 后编辑器通过 `/drone/interactive_goals/execute`（`drone_msgs/srv/ExecuteGoalSequence`）提交完整 `PoseArray goals` 和 `uint64 draft_revision` 快照；响应仅表示异步预检是否被接受。

执行节点独立复核 frame、数量、有限值、零 yaw、导航地板、安全 workspace 和规划膨胀障碍物。随后等待新鲜实际 Odom：地面状态以 `(actual_x,actual_y,takeoff_height)` 为预检起点并单独检查垂直起飞段，空中状态直接使用实际位置；再对 `START→P1→...→PN` 的每段异步运行 A* 和 `PlannedTrajectoryBuilder`。任一段失败时整体拒绝且不起飞。全部预检通过后才复用原状态机起飞，并继续从每段开始时的实际 Odom 重新规划；编辑器预览 Path 从不作为控制参考。

失败控制显式区分 `flight_started=false/true`，且安全 hold 使用 `std::optional<Eigen::Vector3d>`，不再把默认零向量视为有效位置。地面预检失败后 Failed 状态发布零 trajectory setpoint，控制器因没有有效 setpoint 持续输出零 RPM；已开始飞行后失败则持续发布最近有限安全 Odom 位置的零速度、零加速度 hold。若已飞行却缺少安全位置，节点记录 fatal 并拒绝发布默认原点目标。两类失败都为 `active=false`、`success=false`、`complete=false`，`fail()` 清空 planned、simplified 和 reference Path。

请求快照接受后，编辑器在本次 Launch 生命周期内保持锁定，隐藏候选、编辑目标 Marker 和预览 Path；拖动、Add、Undo、Clear、Validate 和重复 Execute 均不改变快照，Print YAML 保留。执行节点发布并独占显示已有多目标 Marker。完成后保留全部绿色目标和绿色实际轨迹，清空 planned/simplified/reference Path，并持续保持末目标。第一版不支持同一次 Launch 的第二份任务、任务替换或抢占。

交互式任务状态 Topic 均为 Reliable、Transient Local、Depth 1：

- `/drone/interactive_mission/active`；
- `/drone/interactive_mission/status`；
- `/drone/interactive_mission/draft_revision`。

自动集成 E2E 使用 P1 `(3.5,1.0,2.5)`、P2 `(5.5,1.0,4.0)`、P3 `(7.0,5.0,4.0)`，覆盖未 READY 门控、编辑器锁定、实际 Odom 预检、三目标顺序执行、完成清线和持续实际轨迹。全量回归中的任务用时 `50.799 s`，最大导航跟踪误差 `0.024151 m`，最小基础净空 `0.242058 m`，最大电机转速 `13067.5 RPM`，最终误差 `0.004832 m`、最终速度 `0.001565 m/s`，无碰撞、非有限值或饱和。另有测试专用闭合墙场景验证“端点几何合法但全序列无路径”时任务整体失败且零 setpoint。该测试环境不修改正式六障碍地图。

## 当前地图

工作空间：

```text
x: [-1.0, 14.5]
y: [-2.5, 7.0]
z: [-0.5, 5.0]
```

六个障碍物均为 AABB，格式为中心与尺寸：

| ID | 中心 `(x,y,z)` m | 尺寸 `(x,y,z)` m | 作用 |
|---|---|---|---|
| O1 | `(2.6,-0.5,2.35)` | `(0.8,4.0,4.7)` | 下侧边界墙，迫使前段上绕 |
| O2 | `(4.6,4.15,2.35)` | `(0.8,4.7,4.7)` | 上侧安全边界墙，迫使下绕 |
| O3 | `(6.7,0.8,2.35)` | `(0.8,3.2,4.7)` | 内部中段障碍物 |
| O4 | `(8.9,3.75,2.35)` | `(0.8,5.5,4.7)` | 上侧安全边界墙，形成后段转向 |
| O5 | `(11.4,-0.85,2.35)` | `(0.8,1.3,4.7)` | 末端下侧内部障碍物 |
| O6 | `(11.4,2.95,2.35)` | `(0.8,2.5,4.7)` | 末端上侧内部障碍物 |

“安全边界墙”表示障碍物经过对应安全半径膨胀后与收缩 workspace 相接，因此规划器不能从边界缝隙绕过。O5/O6 保持原始 `1.9 m` 通道；按 `0.35 m` 有效规划半径从两侧膨胀后，有效宽度为 `1.2 m`。

本轮将 O5/O6 从 `x=10.7 m` 整体右移 `0.7 m`。O4 原始 x 上界为 `9.3 m`，O5/O6 新原始 x 下界为 `11.0 m`：

```text
原始 x 间距：11.0 - 9.3 = 1.7 m
基础 0.25 m 膨胀后间距：10.75 - 9.55 = 1.20 m
规划 0.35 m 膨胀后间距：10.65 - 9.65 = 1.00 m
```

旧规划膨胀后 x 间距仅 `0.30 m`。新间距允许飞行器在 O4 后、O6 前从低 y 区域安全转向高 y 区域，同时不改变 O5/O6 挑战通道宽度。

## 安全模型

- 基础安全半径：`safety_radius=0.25 m`
- 规划额外裕量：`planning_margin=0.10 m`
- A*、路径简化和连续轨迹验证的有效规划半径：`0.35 m`
- 多目标导航地板：球心 `z=0.50 m`

URDF 与动力学参数中的机臂/电机中心半径为 `0.20 m`，URDF 旋翼半径为 `0.065 m`，水平可视包络最远约 `0.265 m`。`0.25 m` 球形模型接近实际几何包络，属于合理简化，不应为通过测试而降低。

代码审查确认没有重复添加规划裕量：

- `StaticEnvironment` 只保存原始 workspace 和原始障碍物；
- `CollisionChecker` 按构造时给定半径收缩 workspace、膨胀每个障碍物一次；
- `static_environment_node` 以 `0.25 m` 构造检查器，因此 RViz 透明区和实时碰撞 Topic 表示基础安全范围；
- `astar_planner_node`、`planned_trajectory_node`、`multi_goal_static_avoidance_node` 各自计算一次 `0.25+0.10=0.35 m`，再构造一个规划检查器；
- 路径简化和轨迹构建复用该 `0.35 m` 检查器，不再次膨胀。

目标位于安全 workspace 外或有效膨胀障碍物内时被拒绝；目标安全但无路径时规划失败，执行端保持当前安全位置。碰撞状态是监测结果，不模拟物理撞击响应。

## 当前目标

默认单目标：

```text
start = (0.0, 0.0, 1.5)
goal  = (13.2, 5.5, 1.5)
```

默认多目标任务：

| 目标 | 坐标 `(x,y,z,yaw)` | 作用 |
|---|---|---|
| P1 | `(13.2,5.5,1.5,0.0)` | 一次完整穿越到地图右上远端 |
| P2 | `(7.0,5.0,4.0,0.0)` | 高空返回中部上侧 |
| P3 | `(0.8,0.7,2.0,0.0)` | 偏置返航到起飞区附近 |

P1、P2、P3 以及评测目标 B `(12.1,1.1,2.5)` 均在 `0.35 m` 模型下安全且 A* 可达。P1 已同时作为默认单目标和默认多目标首段完成闭环验证。

默认任务仍为上述三个目标，但配置解析没有写死数量：支持任意非空数量的 `[x,y,z,yaw]` 分组，且当前版本要求所有 yaw 为零。任意数量的合法目标都可以配置并按序尝试规划；每个目标仍必须满足几何安全、A* 可达和连续轨迹验证。自动测试覆盖 1、3、5 个目标以及空列表、非 4 倍数、NaN/Inf 和非零 yaw 拒绝。

多目标轨迹名义速度为 `0.35 m/s`，由本轮完整速度扫描从旧 `0.25 m/s` 基线保守提高；单目标轨迹名义速度仍为 `0.35 m/s`。控制器增益、最大倾角、最大水平加速度、最大参考加速度、最大 RPM、地图、安全半径、规划裕量、A* 分辨率和核心算法均未改变。

## 安全轨迹细化回退

原问题是：A* 折线及视线简化折线本身均无碰撞，但五次多项式在尖锐拐角附近可能切入规划膨胀障碍物。`PathSimplifier` 现在额外返回严格递增的原始路径索引，旧 `simplify()` 接口与首次最远可见点行为保持不变。

`PlannedTrajectoryBuilder` 的首个候选仍使用原简化路径、原分段时长和原 `velocity_scale` 候选。仅当完整采样验证报告点或线段碰撞时，才在失败轨迹段附近按原始 A* 索引确定性恢复局部点；最多细化 8 轮、每轮最多插入 3 点。若几何已安全但仅速度或加速度超限，则依次尝试 `duration_scale=[1.0,1.05,1.10,1.15,1.20,1.25,1.5,2.0,3.0,4.0]`。所有候选均重新验证有限值、端点、速度、加速度、采样点与相邻采样线段；实现不包含目标坐标或障碍物编号硬编码。

远端 P1 最终使用 3 轮细化、18 个 waypoint、`velocity_scale=1.0`、`duration_scale=1.0`。参考轨迹对 `0.35 m` 规划膨胀模型的采样最小净空为 `0.050000 m`。本轮提速没有改变安全半径、规划裕量、地图或 A* 核心算法。

## 远端 P1 验证

使用 `astar_evaluation_scenario_a.yaml` 和完整单目标闭环链路：

| 指标 | 结果 |
|---|---:|
| A* 成功 | 是 |
| 扩展节点 | `18562` |
| 原始路径 | `72` 点 / `21.013672 m` |
| 初始 / 最终路径 | `9 / 18` 点，局部细化 `3` 轮 |
| 最终路径长度 | `20.181489 m` |
| 轨迹时间 | `62.768339 s` |
| 速度比例 | `1.00` |
| 最大参考速度 | `0.552505 m/s` |
| 最大参考加速度 | `0.343554 m/s²` |
| 闭环任务时间 | `68.757455 s` |
| 最大跟踪误差 | `0.030305 m` |
| 对基础膨胀障碍物最小净空 | `0.089736 m` |
| 最终位置误差 | `0.006014 m` |
| 最终速度 | `0.001466 m/s` |
| 碰撞 / NaN/Inf / 饱和 | 无 / 无 / 无 |

连续轨迹的采样点和相邻采样线段均通过 `0.35 m` 有效规划半径检查；实际 Odom 点和相邻 Odom 线段均未进入基础 `0.25 m` 膨胀障碍物。

其余静态避障评测配置也已闭环通过：

| 场景 | 目标 | 原始路径 | 简化路径 | 轨迹时间 | 最大跟踪误差 | 最小基础净空 | 最终误差 |
|---|---|---:|---:|---:|---:|---:|---:|
| B 水平高度 | `(12.1,1.1,2.5)` | 55 点 / `16.490537 m` | 7 点 / `15.358924 m` | `44.285448 s` | `0.030705 m` | `0.170595 m` | `0.002636 m` |
| C 三维路线 | `(7.0,5.0,4.0)` | 41 点 / `12.297858 m` | 7 点 / `11.044104 m` | `39.756184 s` | `0.030305 m` | `0.168025 m` | `0.003158 m` |

B/C 均无碰撞、非有限值或控制器饱和；C 使用与 P2 相同的中部上侧高点，覆盖与 B 不同的三维路线。

## 默认任务闭环验证

默认单目标 `(13.2,5.5,1.5)`：

- A*：72 点、`21.013672 m`、扩展 18562 节点；
- 初始简化 9 点，经 3 轮局部细化得到 18 点、`20.181489 m`；
- 连续轨迹：17 段、`62.768339 s`、速度比例 `1.00`、时间比例 `1.00`；
- 闭环：`68.757 s` 完成，最大跟踪误差 `0.030305 m`，最小基础净空 `0.089736 m`，最终误差 `0.006014 m`、最终速度 `0.001466 m/s`；
- 无碰撞、非有限值或控制器饱和。

默认多目标：

- 名义速度：`0.35 m/s`（旧基线 `0.25 m/s`）；
- 最新 Domain 114 正式评测 Launch 到完成：`142.388 s`；导航执行：`136.762 s`；
- 三段从轨迹启动到目标接受：`63.749/39.298/32.196 s`；
- 三段轨迹时间：`62.754/38.316/31.213 s`；
- 三段 velocity scale：`1.00/0.25/1.00`；duration scale：`1.00/1.00/1.10`；
- 三段最大参考速度：`0.552488/0.633128/0.497278 m/s`；
- 三段最大参考加速度：`0.343598/0.344052/0.325660 m/s²`；
- 三段最大实际速度：`0.559882/0.636064/0.506316 m/s`；包含起飞时的全任务实际速度峰值为 `1.016178 m/s`；
- 三段最大跟踪误差：`0.022007/0.027957/0.018635 m`；
- 三段最小基础净空：`0.089363/0.138852/0.201013 m`，全局最小 `0.089363 m`；
- 四电机全程范围：`0.0–13067.5 RPM`，完成后悬停范围 `10811.1–10827.2 RPM`，未触及 `20000 RPM`；
- 最终误差：`0.004183 m`；最终速度：`0.001692 m/s`；
- 实际轨迹历史：1451 点，任务结束时仍包含起飞初始点；
- 无碰撞、非有限值或控制器饱和。

速度扫描结果保存在 `results/speed_optimization/multi_goal_speed_sweep.csv` 和 `.json`。`0.30 m/s` 在细化候选下用时 `154.963 s`；`0.35 m/s` 扫描用时 `142.076 s`；`0.40 m/s` 探索运行因三段均选择粗粒度 `duration_scale=1.25`，用时反而回升到 `149.706 s`，因此未采用。细粒度候选使 `0.35 m/s` 的 P2→P3 从 `1.25` 降为 `1.10`，证明缩短了实际任务而不只是增加搜索成本。

## 地图可达性检查

`map_reachability_check` 是只读离线工具，直接读取运行时 `environment.yaml`，调用现有 A*，不接入控制链路。默认以 `1.5 m` 间距在 `z=1.5/2.5/4.0 m` 三层采样。

```bash
ros2 run drone_planning map_reachability_check --ros-args \
  --params-file src/drone_bringup/config/environment.yaml \
  --params-file src/drone_bringup/config/astar.yaml
```

结果：

| 高度 | 采样 | 安全 workspace 外 | 规划膨胀障碍物内 | 安全目标 | 可达 | 不可达 |
|---:|---:|---:|---:|---:|---:|---:|
| `1.5 m` | 77 | 17 | 16 | 44 | 44 | 0 |
| `2.5 m` | 77 | 17 | 16 | 44 | 44 | 0 |
| `4.0 m` | 77 | 17 | 16 | 44 | 44 | 0 |
| 合计 | 231 | 51 | 48 | 132 | 132 | 0 |

安全候选目标可达率为 `100%`，未发现孤立安全采样点。

## RViz 与实际轨迹

RViz Orbit 默认焦点为 `(6.75,2.25,1.5)`，距离 `17.5 m`。默认视野覆盖完整 workspace、六个障碍物、基础膨胀区、无人机和路径。

路径 Topic：

- 绿色 `/drone/path`：实际 Odom 历史轨迹，10 Hz，最多 6000 点，约保留 10 分钟，默认开启
- 蓝色 `/drone/reference_path`：连续参考轨迹，默认开启
- 黄色 `/drone/planned_path`：A* 原始路径，保留但默认关闭
- 粉色 `/drone/simplified_path`：简化折线，保留但默认关闭

多目标显示 Topic：

- `/drone/multi_goal/goal_markers`：全部目标主体、Pn 状态标签和任务状态文字；未访问为黄色，当前目标为放大的橙红色，已完成为绿色
- `/drone/multi_goal/current_goal_pose`：当前多目标任务目标 Pose，任务完成后保留最后目标

两个多目标显示 Publisher 均使用 Reliable、Transient Local、Depth 1 QoS，RViz 晚启动也能得到最新状态。旧 `/drone/goal` 属于 waypoint/直接位置目标链路，多目标节点不复用它。RViz 默认开启静态环境、RobotModel、多目标 Marker 和当前多目标 Pose，旧 Goal Pose 默认关闭。人工分析规划细节时，可在 Displays 中重新勾选黄色和粉色路径。

任务状态 Marker 默认以 `5 Hz` 刷新。`Actual` 来自最新且未超时 Odom 的三维线速度模长，无有效 Odom 时显示 `--`；`Reference` 来自当前 `PiecewiseQuinticTrajectory` 采样速度模长，起飞、规划、保持、完成和失败状态为 `0.00 m/s`；`Nominal` 是轨迹分段时间计算的配置基线。`nominal_speed=0.35 m/s` 不代表实际速度或参考速度全程恒定为 `0.35 m/s`。

首次进入 `MissionComplete` 时，多目标节点一次性向 `/drone/planned_path`、`/drone/simplified_path` 和 `/drone/reference_path` 发布带 `map` frame 和当前时间戳的空 Path；这些 Path 同样为 transient-local，晚订阅者不会看到最后一段残留。绿色 `/drone/path` 不会被清除，多目标 E2E 会验证任务结束时实际轨迹仍保留起飞初始点。`fail()` 同样清空三类辅助规划线：飞行前失败保持地面且不发布 setpoint，飞行后失败在最近安全位置悬停，并显示 `MISSION FAILED`。超过约 10 分钟的永久记录应使用 rosbag，而不是无限增长 RViz Path 消息。

## 构建、测试与人工运行

构建与完整测试：

```bash
cd /home/peter/ros2_drone_sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

阶段 A 实际执行 `colcon build --symlink-install`、`source install/setup.bash && colcon test --event-handlers console_cohesion+` 和 `colcon test-result --verbose`；6 个包构建成功，结果为 `246 tests, 0 errors, 0 failures, 0 skipped`，其中 `drone_bringup` 的 12 个 Launch 测试全部通过。默认参数三目标于 `142.199 s` 完成，最小基础净空 `0.089752 m`、最大跟踪误差 `0.027339 m`、最终误差 `0.004245 m`，无碰撞、非有限值或控制器饱和日志。交互三目标 E2E 于 `50.721 s` 完成，最小基础净空 `0.242045 m`、最大跟踪误差 `0.024197 m`、最终误差 `0.004826 m`，无碰撞、非有限值或饱和。预检无路径测试在失败后持续观察 `3 s`，总 setpoint 数 `0`，收到 `412` 条 RPM 命令且最大绝对值 `0`，最大 `|z|=0 m`、最大水平位移 `0 m`；新增纯状态测试还覆盖飞行后失败保持最近非原点安全位置且速度、加速度为零，以及缺失/非有限安全位置时拒绝发布。用户已经在 RViz2 中人工确认自定义目标选择、完整预览、Execute、顺序导航和最终悬停成功；阶段 A 没有要求用户重复 GUI 人工操作。

阶段 B 完整回归再次执行相同构建和全量测试命令，结果为 `252 tests, 0 errors, 0 failures, 0 skipped`，`drone_bringup` 的 13 个 Launch 测试全部通过。默认参数三目标于 `142.143 s` 完成，最小基础净空 `0.089742 m`、最大跟踪误差 `0.029898 m`、最终误差 `0.004216 m`，无饱和；交互三目标于 `50.722 s` 完成，最小基础净空 `0.242077 m`、最大跟踪误差 `0.023958 m`、最终误差 `0.004834 m`，无饱和。阶段 A 预检失败回归仍为 setpoint `0`、最大 RPM 命令 `0`、最大 `|z|=0 m`、最大水平位移 `0 m`。新增测试覆盖外力纯模型方向/零输入兼容/非有限拒绝，以及 ROS 默认关闭、合法应用、非法 frame、非零 torque、超限/非有限拒绝和超时归零。

三组单目标闭环评测：

```bash
python3 tools/evaluate_static_avoidance.py --timeout 100
```

默认完整多目标评测：

```bash
python3 tools/evaluate_multi_goal_mission.py --timeout 200
```

工具使用独立 Domain 114，直接读取正式任务、环境和动力学 YAML。输出目录为 `results/multi_goal_evaluation/default_mission/`：`launch.log` 保存完整日志；`metrics.json` 保存任务级、逐段指标和验收结论；`trajectory.csv` 保存完整对齐时间序列；`xy_path.png`、`position_tracking.png`、`speed_tracking.png`、`tracking_error.png`、`clearance.png`、`motor_rpm.png` 与 `mission_summary.png` 用于报告绘图。最近正式实跑生成 `29026` 行 CSV，所有验收条件通过。

默认多目标 RViz 演示：

```bash
ros2 launch drone_bringup multi_goal_static_avoidance_sim.launch.py use_rviz:=true
```

状态检查：

```bash
ros2 topic echo /drone/multi_goal/current_goal_index
ros2 topic echo /drone/multi_goal/complete
ros2 topic echo /drone/environment/in_collision
ros2 topic hz /drone/path
```

## 2026-07 物理真实性审计与控制包线重标定

`QuadrotorModel` 在原有重力、旋翼 Wrench、`omega × I omega`、电机一阶响应、外力和地面约束之外，新增了一个可关闭的集总空气动力学层：

```text
v_body = q_body_to_world^-1 * v_world
F_drag_body = -c_linear .* v_body - c_quadratic .* abs(v_body) .* v_body
F_drag_world = q_body_to_world * F_drag_body
tau_damping_body = -c_angular .* omega_body
```

正式 `dynamics.yaml` 使用 `linear_drag=[0.01,0.01,0.01] N/(m/s)`、`quadratic_drag=[0,0,0] N/(m/s)^2`、`angular_damping=[0.01,0.01,0.02] N*m/(rad/s)`，开关开启。系数有限且非负；关闭开关严格走旧行为。阻力与 `/drone/external_wrench` 是两个独立物理量。极端有限速度的二次项使用长双精度计算和有限输出保护；姿态每步归一化。积分器仍是 `200 Hz`、`dt=0.005 s` 固定步长半隐式 Euler，没有切换 RK4，因此精度仍是一阶，简化地面仍只约束世界 z。

这些系数是低速集总阻尼，不是具体机型辨识。分阶段扫描先覆盖 xy/z 线性 `0/0.1/0.2/0.3`，再覆盖 roll/pitch `0/0.005/0.01/0.02` 和 yaw `0/0.01/0.02/0.04`，最后只验证少量组合。粗候选 `linear_xy=0.20` 的自由衰减合理，但默认三目标最大跟踪误差达到 `0.2500 m`；无积分 PD 必须靠位置偏差平衡沿轨迹速度产生的阻力，因此继续细化 `0.01/0.02/0.03/0.05`，最终 `0.01` 在三目标任务中通过 `0.04 m` 跟踪门槛。没有通过偷偷增加积分或放宽安全净空解决该冲突。

能力审计脚本 `tools/audit_vehicle_capability.py` 直接读取正式动力学/控制 YAML，并核对 Xacro。物理结果为最大单电机/总推力 `8.3782/33.5128 N`、推重比 `3.4174`、悬停 `10818.9 RPM`（上限的 `54.10%`）。0.08/0.12/0.15/0.20/0.25 rad 保持高度时的水平能力分别为 `0.7862/1.1825/1.4821/1.9879/2.5040 m/s²`，均不接近 RPM 上限。roll/pitch/yaw 最大力矩估算为 `2.3697/2.3697/2.2810 N*m`，控制限制 `1.0/1.0/0.2 N*m` 均低于物理能力。Xacro 只提供一致的 `0.20 m` 几何机臂，没有参与纯动力学的质量、惯量、RPM 或旋翼系数。

物理审计阶段正式 `controller.yaml` 从 `max_horizontal_acceleration=0.4`、`max_tilt_angle=0.08` 更新为 `0.8 m/s²` 和 `0.15 rad`；当时 Kp/Kd 保持 `0.4/1.2` 且未混入积分。`g*tan(0.15)=1.4821 m/s²`，比控制限制多 `0.6821 m/s²`。候选扫描覆盖所有要求的加速度值 `0.4/0.6/0.8/1.0/1.2/1.5` 和倾角值 `0.08/0.12/0.15/0.18/0.20`，只组合有理论余量的代表点。`0.8/0.15` 是首个水平阶跃零限幅候选；`1.5/0.20` 对阶跃没有收益，故未采用。

最终对比：推荐 2 m 阶跃上升 `4.885 s`、稳定 `8.993 s`、超调 `0`、最大倾角 `0.10566 rad`、最大 RPM `12016.9`、四级饱和全零。`0.3 N × 2 s` 最大偏移/恢复 `0.3486 m/4.655 s`；`0.8 N × 2 s` 从旧 `2.0913 m/9.410 s/465` 个水平限幅样本改善为 `0.9502 m/6.415 s/71`，高度/姿态/Mixer 始终无饱和。`0.3 N × 10 s` 约 `0.744 m` 偏差展示了当前 PD 的恒力稳态误差。

正式默认三目标最终评测结果为完成 `139.309 s`、最大跟踪误差 `0.026328 m`、最小基础净空 `0.094738 m`、最大倾角 `0.039230 rad`、最大 RPM `13067.5`、饱和 `0`、最终误差 `0.005590 m`、最终速度 `0.002421 m/s`。轨迹和规划参数完全未变。`ControllerDiagnostics.msg` 与 `/drone/controller/diagnostics` 提供水平加速度请求、总推力、三轴力矩、裁剪前带符号等效 RPM、裁剪后 RPM 和 horizontal/altitude/attitude/mixer 四级饱和状态。当前 Mixer 不是瓶颈，仍保留简单裁剪。

结构化结果位于 `results/vehicle_capability_audit/baseline/` 和 `results/vehicle_model_upgrade/{baseline,drag_sweep,controller_envelope_sweep,selected}/`。该阶段把恒力稳态误差明确留给后续独立的 anti-windup 水平积分研究；研究结果记录在下一节，未扩展 Mixer、动态障碍、在线重规划或传感器噪声。

## 2026-07 水平积分补偿升级

水平控制器已从无状态 PD 升级为 PID 类控制；高度与姿态仍是 PD。算法状态为世界系 `Eigen::Vector2d integral_acceleration_world_`，单位 `m/s²`。正式值：`Ki=[0.15,0.15] s^-2`、积分向量上限 `0.35 m/s²`、`Kaw=2.0`、capture `0.50 m`、pose reset distance `1.0 m`；P/D、`0.8 m/s²` 水平限制和 `0.15 rad` 倾角限制未改。禁用积分或 Ki=0 时严格退化为旧 PD 输出。

误差积分与输出饱和回算为 `Ki·e + Kaw·(a_sat-a_raw)`；饱和时冻结 `Ki·e`、保留 back-calculation，并单独约束积分向量。节点 reset 覆盖启动、无目标、地面、大 pose 跳变、非法状态和停止输出；Odom 短超时、capture 外、快速 trajectory 和饱和阶段使用 freeze。轨迹采样只在时间回退/源切换语义下 reset，不会逐样本清零。连续 Odom 还用于检测等效水平加速度阶跃：已有积分储量的偏置撤销后，进入 `8 s` 显式 Kaw 去积分窗口；控制器不订阅 `/drone/external_wrench`。诊断区分 P/D/I/FF、raw/saturated、enabled/frozen/reset/anti-windup。

分阶段扫描严格为 Ki `0.05/0.10/0.15/0.20`、Kaw `0.5/1/2`、积分上限 `0.25/0.35/0.45`，最终选择 `0.15/2.0/0.35`。PD→最终：`0.3 N×2 s` 偏移/恢复 `0.34862/5.725→0.33352/5.500`；`0.8 N×2 s` 为 `0.95020/7.543→0.93680/7.536`。持续 `0.3 N×15 s` 最后 3 秒误差 `0.74934→0.08199 m`，最终方案均速 `0.01058 m/s`，无持续水平饱和。`0.3 N×10 s` 撤力后反向超调 `0.15857 m`，最终误差/速度 `0.00309/0.00373`。

最终无外力三目标回归为完成 `139.204 s`、最大跟踪误差 `0.026826 m`、最小净空 `0.094343 m`、最终误差/速度 `0.004221/0.005364`、饱和 `0`。质量失配 `1.10/1.20 kg` 时水平仍为零，但高度稳定在 `1.1731/0.8462 m`；这明确保留为高度积分/模型自适应的后续问题。外力输入仍是集中等效力，不是空间风场。结果位于 `results/horizontal_integral_upgrade/`。

最终全量回归 `276/276` 通过，错误/失败/跳过均为 `0`，bringup Launch 测试为 `14` 个。交互 E2E：`49.960 s`、最大跟踪误差 `0.019958 m`、最小净空 `0.241833 m`、最终误差/速度 `0.003702/0.004641`、饱和 `0`。预检失败仍为 setpoint `0`、RPM 消息 `404` 且最大命令 `0`、高度/水平位移 `0`。

正式配置全量回归为 `259 tests, 0 errors, 0 failures, 0 skipped`，13 个 `drone_bringup` Launch 测试全部通过。回归内默认三目标于 `138.942 s` 完成，最大跟踪误差 `0.031196 m`、最小净空 `0.094690 m`、最终误差 `0.005580 m`、最终速度 `0.002418 m/s`、饱和 `0`。交互三目标用时 `49.961 s`，最大跟踪误差 `0.021084 m`、最小净空 `0.241174 m`、最大 RPM `13067.5`、最终误差 `0.007045 m`、最终速度 `0.002704 m/s`、饱和 `0`。预检无路径仍保持 setpoint `0`、404 条 RPM 命令最大值 `0`、高度和水平位移 `0`。

## 当前风险与后续约束

- 最窄挑战通道有效规划宽度仍为 `1.2 m`，后续提速必须重新验证连续轨迹和实际 Odom 净空，不能只看 A* 点路径。
- 远端 P1 需要 3 轮局部路径细化，最新多目标实跑最小基础净空 `0.089363 m`，已接近本轮 `0.085 m` 选择下限；不建议继续统一提高名义速度。
- 完整多目标任务约 `142 s`，人工演示建议预留约 3 分钟并保持绿色实际轨迹显示开启。
- 地图可达性扫描是规则网格抽样，不是对连续自由空间的数学完备证明，但三层 132 个安全候选点全部可达。
- 测试代码仍有用于独立几何断言的显式 AABB；运行时配置和评测工具读取统一 `environment.yaml`。修改地图时必须同步这些测试常量。
- 本轮统一速度优化已完成；若继续提速，建议下一阶段研究转角/净空相关的分段速度规划。动态目标、RPM 曲线扩展和局部重规划不在当前阶段。

# AI Context：ROS2 四旋翼仿真

## 稳定里程碑与开发边界

当前阶段：assessment scope consolidation（面向考核收束与结构优化）

工作分支：`refactor/assessment-scope-consolidation`

禁止事项：不得删除旧节点、Launch 或测试，不得增加与考核无关的新功能

当前稳定版本是覆盖静态避障、交互导航、路径切线 yaw、目标 yaw 完成门控、外部扰动与水平积分的功能完整基线。当前阶段不得继续扩展新功能。

本阶段从冻结 `main` 建立独立分支，只增加统一入口、依赖审计和轻量结构测试。

默认不要加入高度积分、姿态积分、障碍地图风扰、动态障碍或 MPC。只有新任务明确要求，或出现可复现且需要对应机制解决的物理问题时，才重新评估这些方向。

## 环境与包

- 目标环境：Ubuntu 22.04、ROS2 Humble、C++17。
- 本机默认工作区：`/home/peter/ros2_drone_sim`；公开命令不得依赖该绝对路径。
- `drone_msgs`：`MotorRPM`、`TrajectorySetpoint`、`ControllerDiagnostics`、`ExecuteGoalSequence`。
- `drone_dynamics`：四旋翼刚体、电机响应、地面约束、集中外力输入、Odom/IMU/TF。
- `drone_controller`：位置、高度、姿态控制和 X 构型 Mixer。
- `drone_mission`：离散 waypoint 和分段五次无障碍轨迹任务。
- `drone_planning`：静态环境、碰撞检查、3D A*、路径简化、安全连续轨迹、多目标与交互导航。
- `drone_bringup`：配置、Launch、RViz、扰动演示节点和端到端测试。

## 当前运行链路

```text
任务/目标/规划
→ position_controller_node
→ 高度与姿态控制
→ MotorMixer
→ /drone/motor_rpm_cmd
→ quadrotor_dynamics_node
→ /drone/odom、/drone/imu、/drone/path、TF
```

```text
StaticEnvironment
→ CollisionChecker
→ AStarPlanner
→ PathSimplifier
→ PlannedTrajectoryBuilder / PiecewiseQuinticTrajectory
→ /drone/trajectory_setpoint
→ 跟踪执行
```

### 控制基线

- 水平位置控制：`P + D + 受限 I + 期望加速度前馈`。
- 高度控制：`PD + 重力/期望加速度前馈/倾角补偿`。
- 姿态控制：姿态 `P + 角速度 D`。
- Mixer：X 构型，输出受 `min_rpm/max_rpm` 限制。

不要把整个飞控称作完整 PID。只有水平 x/y 位置环有积分；高度和姿态环没有积分。

## 主要节点

| 节点 | 作用 |
|---|---|
| `quadrotor_dynamics_node` | 接收四电机 RPM，推进刚体并发布状态 |
| `position_controller_node` | 接收 Pose 目标或轨迹 setpoint，发布 RPM 和诊断 |
| `waypoint_manager_node` | YAML 或 Service 输入的无障碍离散多目标任务 |
| `goal_visualizer_node` | 无障碍单/多目标统一 Marker 显示 |
| `trajectory_mission_node` | 无障碍分段五次轨迹任务 |
| `static_environment_node` | 地图 Marker 和实时碰撞状态 |
| `astar_planner_node` | 单次 3D A* 原始路径规划 |
| `planned_trajectory_node` | 路径简化、安全连续轨迹和可选执行 |
| `multi_goal_static_avoidance_node` | 起飞、逐段规划、执行、停稳切换和失败保持 |
| `interactive_goal_editor_node` | RViz 目标编辑、预览、READY 门控和执行请求 |
| `disturbance_demo_node` | 悬停目标、外力时序和扰动 Marker 编排 |

`robot_state_publisher` 和 `rviz2` 由相应 Launch 启动。

## 关键 Topic 与 Service

### 控制与动力学

- `/drone/goal`：普通 Pose 目标。
- `/drone/trajectory_setpoint`：位置、速度、加速度和 yaw 参考。
- `/drone/motor_rpm_cmd`：四电机命令。
- `/drone/controller/diagnostics`：限幅、积分和控制诊断。
- `/drone/odom`、`/drone/imu`、`/drone/path`：状态和实际轨迹。
- `/drone/external_wrench`：`map` 系质心集中等效外力输入。
- `/drone/external_wrench/active`、`/drone/external_wrench/applied`：实际应用状态。
- `/drone/disturbance/markers`：专用扰动可视化。

### 规划与任务

- `/drone/planned_path`、`/drone/simplified_path`、`/drone/reference_path`。
- `/drone/planning/success`、`/drone/planning/expanded_nodes`。
- `/drone/environment/markers`、`/drone/environment/in_collision`。
- `/drone/multi_goal/current_goal_index`、`visited_goals`、`complete`、`success`。
- `/drone/multi_goal/goal_markers`、`/drone/multi_goal/current_goal_pose`。
- `/drone/mission/goals`、`/drone/mission/current_waypoint_index`、`/drone/mission/complete`。
- `/drone/mission/execute`：复用 `ExecuteGoalSequence` 的运行时任务接口；执行中拒绝抢占。
- `/drone/mission/goal_markers`：无障碍单目标与多目标统一 MarkerArray。

### 交互导航

- `/drone/interactive_goals/goal_editor/update`：Interactive Marker update。
- `/drone/interactive_goals/selected_goals`、`preview_path`、`status`、`ready`、`count`。
- `/drone/interactive_goals/execute`：`ExecuteGoalSequence` Service。
- `/drone/interactive_mission/active`、`status`、`draft_revision`。

交互编辑器只在 READY 后允许执行。候选和已添加目标统一保存 `{position, yaw}`；RViz 候选使用大半径世界 XY 平移外圈、独立的小半径世界 Z yaw 内圈和 Z 平移箭头，两圈具有显式且互不重叠的鼠标命中区域，`Set Yaw` 菜单提供 `0°、±45°、±90°、±135°、180°`。改变 yaw 与改变位置一样增加 draft revision 并使 READY 失效。`selected_goals`、执行 Service 请求和 YAML 均保留各目标 yaw。执行节点从新鲜实际 Odom 对完整目标序列重新预检；地面预检失败时保持零 setpoint/零 RPM，飞行开始后的失败则保持最近有限安全位置。

## Launch 入口

| Launch | 用途 |
|---|---|
| `assessment_basic_sim.launch.py` | 正式基础入口；等待运行时 single/multi，不自动执行 YAML |
| `assessment_navigation_sim.launch.py` | 正式规划避障入口；默认 `path_tangent` |
| `assessment_disturbance_sim.launch.py` | 正式抗扰入口；转发两个 profile |
| `basic_sim.launch.py` | 基础动力学、控制器、模型和 RViz；等待 `/drone/goal` |
| `mission_sim.launch.py` | 无障碍离散多目标任务 |
| `trajectory_sim.launch.py` | 无障碍连续轨迹任务 |
| `environment_sim.launch.py` | 静态环境显示与碰撞监测 |
| `planning_sim.launch.py` | 静态环境和一次性 A* 显示 |
| `planned_trajectory_sim.launch.py` | 路径简化和安全连续轨迹显示/执行入口 |
| `static_avoidance_sim.launch.py` | 单目标静态避障闭环 |
| `multi_goal_static_avoidance_sim.launch.py` | 正式多目标静态避障闭环 |
| `interactive_goal_editor_sim.launch.py` | 只读预览，不启动飞行执行链 |
| `interactive_goal_navigation_sim.launch.py` | RViz 编辑、预检和实际导航 |
| `disturbance_hover_sim.launch.py` | 自动悬停目标和外力输入能力，不主动发布扰力 |
| `disturbance_visual_demo.launch.py` | `short_gust` / `persistent_release` 扰动演示 |

`simulation_core.launch.py` 是内部公共入口，集中创建动力学、控制器、模型和
RViz；公开 Launch 文件名和默认行为保持不变。`basic_sim` 与 `mission_sim` 自动
启动 `goal_visualizer_node`。`mission_sim` 默认加载 YAML，也可设置
`start_with_configured_waypoints:=false` 等待 `goal_cli multi`。

`goal_cli` 的纯数字 yaw 保持弧度兼容，同时支持 `yaw=30`、`yaw=60`、
`yaw=90` 这类角度输入。单目标 Marker 仅显示权威目标内容并保持
`GOAL CURRENT`；多目标的 CURRENT/DONE 来自 waypoint manager 任务状态。

## 正式配置与参数

### 控制器 `controller.yaml`

- 控制频率 `100 Hz`，Odom/轨迹超时均 `0.2 s`。
- 水平：`kp_x=kp_y=0.4`，`kd_x=kd_y=1.2`。
- 水平积分：启用，`ki_x=ki_y=0.15`，加速度限制 `0.35 m/s²`。
- anti-windup gain `2.0`，卸载 gain `2.0`，捕获半径 `0.50 m`，目标跃迁重置距离 `1.0 m`。
- 最大水平加速度 `0.8 m/s²`，最大倾角 `0.15 rad`。
- 高度：`kp=3.0`，垂向速度 `kd=3.5`。
- 姿态 P：roll/pitch/yaw 为 `4.0/4.0/1.0`；角速度 D 为 `0.35/0.35/0.40`。
- Mixer：臂长 `0.20 m`，`kF=1.91e-6`，`kM=2.60e-7`，RPM 范围 `0–20000`。

参数真实名称以 `src/drone_bringup/config/controller.yaml` 为准，不要在文档或新 Launch 中发明别名。

### 动力学 `dynamics.yaml`

- 质量 `1.0 kg`，惯量 `0.02/0.02/0.04 kg·m²`，仿真频率 `200 Hz`。
- 电机一阶时间常数 `0.05 s`，命令超时 `0.30 s`。
- 外力超时 `0.20 s`，最大外力 `2.0 N`，非零外力矩不支持。
- 已启用集总低速平动阻力和角阻尼；这不是特定机体的风洞辨识模型。
- 简化刚性水平地面启用。静态障碍只参与碰撞查询，不产生物理反作用。
- 正常工作流不启用外力订阅；只有专用扰动 Launch 覆盖 `enable_external_wrench=true`。

### 地图与规划

- 工作空间：`x [-1.0,14.5]`、`y [-2.5,7.0]`、`z [-0.5,5.0]`。
- 正式地图为六个静态 AABB 障碍物，配置见 `environment.yaml`。
- 基础安全半径 `0.25 m`，规划额外裕量 `0.10 m`，有效规划半径 `0.35 m`。
- 多目标导航最低球心高度 `0.50 m`。
- 默认多目标：P1 `(13.2,5.5,1.5,0)`、P2 `(7.0,5.0,4.0,0)`、P3 `(0.8,0.7,2.0,0)`。
- 静态避障 yaw 默认 `fixed`；可选 `path_tangent` 使用轨迹水平速度切线，低速保持
  最近有效 yaw，在目标前 `0.80 m` 内混合至当前 waypoint yaw，并以 `0.30 s`
  一阶滤波和 `0.80 rad/s` 限速。阈值为 `0.10 m/s`。

## 当前正式结果

### 多目标静态避障

数据源：`results/horizontal_integral_upgrade/selected/multi_goal/metrics.json`。

- 任务成功，3 个目标全部访问，Launch 到完成 `139.203712 s`。
- 最大跟踪误差 `0.028843 m`。
- 对基础膨胀障碍物最小净空 `0.094310 m`。
- 最终误差 `0.001536 m`，最终速度 `0.004355 m/s`。
- 无点/线段碰撞、非有限值或控制器饱和；饱和计数 `0`。

### 静态避障路径切线 yaw 验证

数据源：`results/static_avoidance_yaw/scenario_metrics.json`。这些场景来自提交前工作树，
结果文件明确保留该来源，不伪造提交 SHA。

- 单目标 fixed 对照：任务成功，最大跟踪误差 `0.030812 m`，最小净空
  `0.094306 m`，最大相邻 yaw 跳变与最终 yaw 误差均为 `0`。
- 单目标 path-tangent：任务成功，最大跟踪误差 `0.031013 m`，最小净空
  `0.094028 m`，最大相邻 yaw 跳变 `0.016161 rad`，最大参考变化率
  `0.800119 rad/s`，平均切线误差 `0.052637 rad`，最终 yaw 误差 `0`。
- 三目标 path-tangent：依序访问 P1/P2/P3，最大跟踪误差 `0.039032 m`，最小净空
  `0.094029 m`，最大相邻 yaw 跳变 `0.016328 rad`，最大参考变化率
  `0.800272 rad/s`，平均切线误差 `0.130313 rad`，最终 yaw 误差 `0`；无碰撞、
  非有限值或控制器饱和。

### 交互终端 yaw 验证

`results/interactive_goal_yaw/path_tangent_e2e.json` 记录提交
`06013d454a1287427b61ce9c52374ff1a03fc3fe` 上的三目标 `90°、180°、-90°`
自动闭环场景。任务依序成功，目标切换同时要求位置、线速度、最短角 yaw 误差和角速度
在完整保持时间内满足阈值；无碰撞、饱和或非有限值。本次最终完整回归构建 6 个
package，并通过 `342 tests, 0 errors, 0 failures, 0 skipped`。后续仅包含验证记录和文档
的提交不声明经过这次测试。

### 外力与水平积分

数据源：`results/horizontal_integral_upgrade/selected/regression_summary.json` 和 `repeat_results.json`。

- **PD baseline**：水平积分关闭；持续 `0.30 N` 下末 3 秒平均误差 `0.749340 m`。
- **当前水平 PD+I+FF 正式基线**：`ki=0.15`、积分加速度限制 `0.35 m/s²`；同场景末 3 秒平均误差 `0.081989 m`。
- 当前基线三次独立 `0.30 N × 10 s` 撤力实验：恢复时间 `4.600580–4.601050 s`，反向超调 `0.107763–0.107767 m`，均无控制器饱和。

旧 PD 数据只作为对照，不能称为当前最终控制结果；其他候选和扫描位于 `results/`，不应逐项复制到用户文档。

### 自动测试

yaw 完成门控修复后的最终记录来自提交
`06013d454a1287427b61ce9c52374ff1a03fc3fe`：构建 6 个 package 成功，
`342 tests, 0 errors, 0 failures, 0 skipped`。三个交互目标被接受时的 yaw 误差为
`0.005191925 rad`、`0.004745509 rad`、`0.004973346 rad`。实际命令、工作树状态和
统计见 `results/interactive_goal_yaw/full_regression.json` 与
`results/interactive_goal_yaw/path_tangent_e2e.json`。

`results/` 中既有量化指标属于开发阶段实验数据，必须保留其原始 `git_commit` 等来源字段；提交收尾不得手工改写旧 SHA，也不得声称所有长时间实验已在提交候选上重跑。核心演示的人工视觉验收由用户在提交收尾前完成，与自动回归分开记录。

## 许可证与参考项目

- 本项目所有者确认当前仓库为独立实现，并选择 `Apache-2.0`；根目录 `LICENSE` 和六个 `package.xml` 是许可证权威入口。
- 任务指定参考项目为 `https://gitee.com/potato77/pengyu_sim` 和 `https://github.com/hku-mars/MARSIM`。
- 两个参考项目只用于理解仿真、动力学、控制和系统组织思路；本仓库没有复制或改编其代码、模型、地图、配置、图片或其他资源，不是二者的移植版本。

## 当前限制

- 高度环、姿态环仍为 PD；只有水平位置环具有受限积分。
- 外力是质心处集中等效力，不是完整空间风场。
- 静态环境没有物理碰撞反作用。
- 没有动态障碍、局部规划和在线重规划。
- 静态避障已有可选的路径水平切线 yaw 参考，但默认仍为 fixed；它不包含 yaw
  加速度规划、完整姿态规划或最优 yaw 求解。
- 无障碍运行时任务首版不支持抢占，执行中请求会被拒绝。
- 交互执行首版不支持同一 Launch 内提交第二份任务、替换或抢占。
- 交互目标只支持 Undo 最后一个目标后重建，不支持直接修改任意历史目标；终端 yaw 编辑不是完整姿态规划。

## 考核收束审计

### 审计结论与正式依赖

三个新入口都只声明考核参数并 Include 现有 Launch，不直接创建 ROS 节点：

```text
assessment_basic_sim
└─ mission_sim(start_with_configured_waypoints=false)
   ├─ basic_sim → simulation_core
   ├─ goal_visualizer_node
   └─ waypoint_manager_node

assessment_navigation_sim
└─ interactive_goal_navigation_sim(yaw_mode=path_tangent)
   ├─ simulation_core(setpoint_source=trajectory)
   ├─ static_environment_node
   ├─ interactive_goal_editor_node
   └─ multi_goal_static_avoidance_node

assessment_disturbance_sim
└─ disturbance_visual_demo(profile=...)
   ├─ quadrotor_dynamics_node(enable_external_wrench=true)
   ├─ position_controller_node
   ├─ disturbance_demo_node
   └─ robot_state_publisher / RViz
```

基础入口中的 single 直接向 `/drone/goal` 发布，multi 通过
`/drone/mission/execute` 交给 `waypoint_manager_node`，两者共享同一权威目标 Marker
节点。入口只启动一套动力学、控制器和任务执行器；不应在任务执行中尝试抢占，但顺序
运行 single 与 multi 没有节点名、执行器或 Topic 冲突。

`obstacle_field` 使用原有 `environment.yaml` 六障碍综合地图，不修改其几何或历史结果
来源。`narrow_passage` 使用独立 `environment_narrow_passage.yaml`：5 个 AABB 组成两道
错位墙和 S 形强制路线，两处开口原始宽度均为 `1.8 m`；保持基础安全半径 `0.25 m`
和规划裕量 `0.10 m` 后，有效开口宽度为 `1.1 m`。起点 `(0,0,1.5)` 至测试目标
`(8.5,0,1.5)` 的直线被第一道墙阻挡。

`assessment_navigation_sim` 解析场景后只向内部 Launch 转发一个
`environment_config`；`interactive_goal_navigation_sim` 再把同一个 LaunchConfiguration
传给 `static_environment_node`、`interactive_goal_editor_node` 和
`multi_goal_static_avoidance_node`，因此显示、预检和执行不可能各自回退到不同地图。

### 正式入口自动闭环证据

- 基础 single：同一正式仿真依次完成 `(0,0,1.5)` 和 `(2,1,1.5)`；最终误差分别
  `0.008872 m`、`0.038596 m`，速度分别 `0.011517 m/s`、`0.029225 m/s`，Marker、
  Path、有限 RPM 和无持续饱和均通过。
- 基础 multi：Service 接受 `P1→P2→P3→P4` 方形任务，索引严格为
  `[0,1,2,3]`，四个 yaw 原样保留；最终误差 `0.031921 m`，速度
  `0.027204 m/s`。最长连续瞬时限幅 `0.51 s`，完成后已退出限幅。
- obstacle_field：正式 assessment wrapper 完成三目标 RViz 反馈、预览、READY、锁定
  与执行；任务 `54.682 s`，最大跟踪误差 `0.022500 m`，最小净空
  `0.239976 m`，最终误差 `0.000859 m`，零碰撞、零非有限值、零饱和。
- narrow_passage：正式 wrapper 加载 11 个环境 Marker（工作空间 + 5 组原始/膨胀
  Marker），直线被阻挡；A* 原始路径 `10.029160 m`，直线 `8.500000 m`，最大横向
  偏离 `0.850000 m`。原始、简化和连续参考路径均通过碰撞检查，实际轨迹通过两处
  指定通道，最小基础膨胀净空 `0.229716 m`，最终误差 `0.006103 m`，零碰撞、零饱和。
- disturbance：正式入口默认 short_gust，确认只在该入口启用 external wrench；实际
  `+X` 外力、红色箭头、负 X 水平积分及蓝色箭头一致，撤力后最终水平误差
  `0.124823 m`、速度 `0.002655 m/s`，零持续饱和。旧 Launch 测试继续验证
  `persistent_release` 的 `10 s` profile 参数。
- 本轮完整回归：6 packages，`364 tests, 0 errors, 0 failures, 0 skipped`。

### 替代测试与删除依据状态

| 候选旧项 | 替代测试状态 | 正式入口 E2E | 是否已具备删除依据 | 尚缺少的人工验收 |
|---|---|---|---|---|
| `test_single_goal_e2e.py` | 悬停与单目标已由新测试在同一正式入口覆盖 | `test_assessment_basic_single_e2e.py` | 具备下一轮删除审查条件 | RViz 单目标 Marker、轨迹与 RPM 观感 |
| `test_waypoint_mission_e2e.py` | 运行时四目标、顺序、yaw、误差和持续饱和已覆盖 | `test_assessment_basic_multi_e2e.py` | 具备下一轮删除审查条件 | 四目标 Marker 的 CURRENT/DONE 视觉状态 |
| `basic_sim` | 正式基础入口覆盖其 Pose 控制链，但仍是 `mission_sim` 内部依赖 | basic single/multi | 暂不可删除 | 先重构 Include 依赖，不得复制节点 |
| `mission_sim` | 正式基础入口直接 Include | basic multi | 暂不可删除 | 它仍是正式入口内部实现 |
| `static_avoidance_sim` / `test_static_avoidance_e2e.py` | 新通道单目标覆盖安全规划与执行，但执行节点不同 | narrow passage E2E | 暂不可删除 | 旧 `planned_trajectory_node` 是否仍需独立展示 |
| `multi_goal_static_avoidance_sim` | 正式 obstacle_field 使用相同多目标执行器并覆盖完整闭环 | obstacle_field E2E | 具备下一轮删除审查条件 | 对照旧 YAML 正式三目标结果图和 RViz 展示 |
| `test_multi_goal_static_avoidance_e2e.py` | 正式交互三目标覆盖顺序、净空、误差和安全 | obstacle_field E2E | 具备下一轮删除审查条件 | 确认旧长测量化字段是否均已迁移 |
| `interactive_goal_editor_sim` / editor E2E | 正式导航覆盖编辑、预览和锁定，但不覆盖禁飞只读模式 | obstacle_field E2E | 暂不可删除 | 只读预览模式是否仍需答辩诊断 |
| `disturbance_hover_sim` | 正式抗扰入口覆盖悬停、外力启用和恢复 | disturbance E2E | 具备下一轮删除审查条件 | 手工外力注入调试入口是否仍需保留 |
| `test_disturbance_visual_launch.py` | 正式默认 short_gust 有完整闭环；旧测试仍独立保护 persistent_release | disturbance E2E | 暂不可删除 | persistent_release 完整 RViz 长时恢复 |
| `trajectory_sim` / trajectory E2E | 尚无正式入口对等覆盖旧无障碍连续轨迹节点 | 无 | 暂不可删除 | 无障碍连续轨迹是否仍为答辩材料 |
| `environment_sim` / environment E2E | 正式导航覆盖地图 Marker 和碰撞，但旧测试有精确边界断言 | 两个导航 E2E | 暂不可删除 | 环境独立诊断入口必要性 |
| `planning_sim` / A* E2E | 正式通道验证 A* 路径，但不替代独立 planner ROS 适配 | narrow passage E2E | 暂不可删除 | 独立 A* Topic 展示必要性 |
| `planned_trajectory_sim` / planned E2E | 正式通道覆盖三类安全路径，但节点适配层不同 | narrow passage E2E | 暂不可删除 | 独立 raw/simplified/reference 展示必要性 |

### 节点与实现审计

| 功能或文件 | 当前作用 | 对应哪项考核 | 是否被最终入口使用 | 是否与其他实现重叠 | 建议 | 删除风险 | 替代测试 |
|---|---|---|---|---|---|---|---|
| `waypoint_manager_node` | 离散 waypoint、运行时多目标 Service、停稳切换 | 基础 single/multi 中的 multi | 是，基础入口 | 与规划包多目标执行器在“顺序任务”层重叠，但无规划职责 | 保留 | 删除会破坏无障碍运行时 multi | `test_waypoint_service`、`test_waypoint_mission_e2e`，后续基础入口 multi E2E |
| `trajectory_mission_node` | YAML 驱动的无障碍分段五次轨迹 | 非正式考核入口 | 否 | 连续轨迹能力与规划包 builder/executor 重叠 | 内部化，远期待删除 | 删除会失去无障碍轨迹算法的独立 ROS 适配回归 | `test_piecewise_quintic_trajectory`；先补 builder 对等回归 |
| `goal_visualizer_node` | single/multi 权威目标 Marker | 基础三项 | 是，基础入口 | 与规划多目标 Marker Topic 分离，语义不同 | 保留 | 删除会使基础展示缺目标状态 | `test_goal_visualizer_node`、基础入口 Marker E2E |
| `astar_planner_node` | 一次性起终点 3D A* 与原始路径发布 | 算法说明/内部诊断 | 否 | 规划逻辑已内嵌于 planned 与 multi-goal 执行链 | 内部化，远期合并 | 直接删除会失去独立 A* ROS 层诊断 | `test_astar_planner`、`test_astar_planner_e2e` |
| `planned_trajectory_node` | 单目标路径简化、连续轨迹和可选执行 | 历史单目标避障 | 否 | 与 `multi_goal_static_avoidance_node` 的逐段 builder/执行重叠 | 内部化，远期合并 | 删除会丢失单段轨迹 ROS 集成覆盖 | builder 单元测试、`test_planned_trajectory_e2e` |
| `multi_goal_static_avoidance_node` | 起飞、逐段 A*、简化、轨迹、yaw 门控、失败保持 | 多障碍、多目标、通道 | 是，导航入口 | 覆盖旧单目标规划执行链，但支持交互任务 | 保留 | 删除会破坏正式导航执行器 | multi-goal、interactive、preflight E2E |
| `interactive_goal_editor_node` | RViz 目标编辑、预览、预检、READY 和执行请求 | 正式规划避障交互 | 是，导航入口 | 与只读 editor Launch 共用同一节点，通过参数区分 | 保留 | 删除会破坏 RViz 正式工作流 | editor、navigation、preflight E2E |
| `static_environment_node` | 障碍 Marker 与实时碰撞状态 | 避障、距离与视觉结果 | 是，导航入口 | 被多个历史 Launch 重复组合，但节点实现唯一 | 保留 | 删除会失去地图显示与碰撞状态 | `test_static_environment_e2e` |
| `disturbance_demo_node` | 悬停目标、扰力时序、Marker 和状态编排 | 独立抗扰加分 | 是，抗扰入口 | `disturbance_hover_sim` 只提供底层接口，不复制该节点 | 保留 | 删除会破坏两个 profile 演示 | disturbance node/visual Launch E2E |

### Launch 审计

| 功能或文件 | 当前作用 | 对应哪项考核 | 是否被最终入口使用 | 是否与其他实现重叠 | 建议 | 删除风险 | 替代测试 |
|---|---|---|---|---|---|---|---|
| `assessment_basic_sim` | 正式 single/multi 统一入口，禁止 YAML 自启动 | 悬停、单目标、3～4 目标 | 是 | 薄封装，无节点复制 | 保留 | 无 | 结构 + basic single/multi 正式 E2E |
| `assessment_navigation_sim` | 正式交互规划入口与真实场景选择 | 多障碍、绕行/通道 | 是 | 薄封装，无节点复制 | 保留 | 无 | 结构 + obstacle_field/narrow_passage 正式 E2E |
| `assessment_disturbance_sim` | 正式 profile 入口 | 抗扰加分 | 是 | 薄封装，无节点复制 | 保留 | 无 | 结构 + short_gust 正式 E2E + persistent 参数测试 |
| `simulation_core` | 唯一公共动力学/控制/模型/RViz 组合 | 所有飞行 | 间接用于基础和导航 | 多个历史入口均 Include | 内部化并保留 | 删除会同时破坏多条链 | launch 结构 + 各真实 E2E |
| `basic_sim` | Pose 目标基础仿真和 Marker | 基础控制内部链 | 间接用于基础 | 被 mission/environment/trajectory Include | 内部化 | 删除需先改依赖树 | single E2E + 基础正式 E2E |
| `mission_sim` | waypoint YAML/Service 任务 | 基础 multi 内部链 | 是，基础直接复用 | 与 basic 组合 | 内部化并保留 | 删除会破坏正式基础入口 | waypoint Service/E2E |
| `interactive_goal_navigation_sim` | 完整交互规划执行链 | 正式导航内部链 | 是，导航直接复用 | 与 editor-only、multi-goal Launch 部分重叠 | 内部化并保留 | 删除会破坏正式导航入口 | interactive navigation/preflight E2E |
| `disturbance_visual_demo` | 完整抗扰演示链 | 正式抗扰内部链 | 是，抗扰直接复用 | 与 hover Launch 的动力学/控制组合重叠 | 内部化并保留 | 删除会破坏正式抗扰入口 | disturbance visual E2E |
| `trajectory_sim` | YAML 无障碍连续轨迹 | 非正式 | 否 | 与规划连续轨迹链重叠 | 待删除 | 历史轨迹 ROS 回归丢失 | piecewise 单元 + builder E2E 后再删 |
| `environment_sim` | basic + 静态环境 | 内部地图诊断 | 否 | 组合被 planning 链包含 | 待删除 | 独立碰撞显示入口消失 | static environment E2E 可迁移到导航 |
| `planning_sim` | environment + A* | 内部 A* 诊断 | 否 | 被 planned trajectory 包含 | 待删除 | A* ROS 层诊断入口消失 | A* 单元/E2E |
| `planned_trajectory_sim` | planning + 单段安全轨迹 | 历史轨迹诊断 | 否 | 与 static avoidance 重叠 | 待删除 | 展示/执行边界诊断减少 | planned trajectory E2E |
| `static_avoidance_sim` | 单目标避障闭环 | 可由正式导航单目标覆盖 | 否 | multi-goal 执行链的子集 | 待删除 | 固定基准场景和旧结果复现入口消失 | 导航单目标 E2E + 保留配置 |
| `multi_goal_static_avoidance_sim` | YAML 正式三目标旧入口 | 可由正式导航多目标覆盖 | 否 | 正式导航复用同一执行节点 | 待删除 | 自动长任务与正式指标复现入口消失 | obstacle_field 正式 E2E 已具备，下一轮审查指标迁移 |
| `interactive_goal_editor_sim` | 禁飞的只读编辑/预览 | 内部 UI 诊断 | 否 | navigation 中 editor 的子集 | 待删除 | 隔离 UI 与执行器的诊断能力下降 | editor E2E 使用直接节点 fixture |
| `disturbance_hover_sim` | 自动悬停、启用外力但不施扰 | 内部外力诊断 | 否 | 与正式 disturbance 链重叠 | 待删除 | 手工外力注入入口消失 | 正式 disturbance E2E 已具备，下一轮审查手工诊断需求 |

### 端到端与 Launch 测试审计

| 功能或文件 | 当前作用 | 对应哪项考核 | 是否被最终入口使用 | 是否与其他实现重叠 | 建议 | 删除风险 | 替代测试 |
|---|---|---|---|---|---|---|---|
| `test_assessment_launch_structure.py` | 三入口加载、默认值、Include 复用、无节点/Topic 复制 | 全部入口结构 | 是 | 与旧 launch structure 部分重叠 | 保留 | 无 | 无 |
| `test_launch_structure.py` | 公共 core 复用 | 内部架构 | 间接 | 被新结构测试扩展 | 合并候选 | 过早删除会丢旧入口约束 | 合并断言到新结构测试 |
| `test_single_goal_e2e.py` | 基础闭环稳定悬停 | 悬停/单目标 | 间接 | 正式 basic single 已覆盖并扩展为两目标 | 删除审查候选 | 删除会失去旧单目标独立基准 | `test_assessment_basic_single_e2e.py` 已通过 |
| `test_waypoint_mission_e2e.py` | YAML waypoint 顺序完成 | 3～4 目标 | 间接 | 正式 basic multi 已覆盖运行时四目标 | 删除审查候选 | 删除会失去旧 YAML 任务基准 | `test_assessment_basic_multi_e2e.py` 已通过 |
| `test_trajectory_mission_e2e.py` | 无障碍连续轨迹跟踪 | 非正式 | 否 | planned builder E2E 重叠 | 待删除 | 旧 trajectory node 无 ROS 回归 | builder 对等集成测试 |
| `test_static_environment_e2e.py` | Marker、碰撞状态 | 避障展示 | 间接 | 导航长测也观察环境 | 内部化 | 删除会失去边界/Marker 精确断言 | 导航环境断言 |
| `test_astar_planner_e2e.py` | A* ROS 输出独立安全 | 避障算法 | 否 | planner 单元和更高层 E2E 重叠 | 待删除 | ROS 参数/Topic 适配缺覆盖 | 导航路径安全断言 |
| `test_planned_trajectory_e2e.py` | 原始/简化/参考路径安全 | 轨迹安全 | 否 | static avoidance 覆盖执行 | 内部化 | 删除会失去三类路径逐项检查 | 导航预览路径断言 |
| `test_static_avoidance_e2e.py` | 单目标闭环避障 | 多障碍 | 间接 | multi-goal/interactive 重叠 | 待删除 | 单目标固定基准丢失 | assessment navigation 单目标 E2E |
| `test_multi_goal_static_avoidance_e2e.py` | 正式三目标、净空、顺序与无冲突节点 | 多目标避障 | 间接 | obstacle_field 正式 E2E 已覆盖相同执行器 | 删除审查候选 | 当前历史量化结果主要保护网 | 正式 obstacle_field E2E 已通过；先审查指标迁移 |
| `test_interactive_goal_editor_e2e.py` | 只读编辑、预览与 RViz namespace | 导航 UI | 间接 | navigation E2E 覆盖部分行为 | 内部化 | 删除会失去编辑器隔离测试 | 将交互断言合入正式导航 E2E |
| `test_interactive_mission_service.py` | 请求校验、等待和拒绝重复任务 | 多目标执行安全 | 间接 | navigation E2E 部分覆盖 | 保留 | 删除会弱化任务接口边界 | 正式导航 Service 边界测试 |
| `test_interactive_goal_navigation_e2e.py` | READY 快照、锁定和完整执行 | 正式导航 | 是，现已从 assessment wrapper 启动 | 与 editor/multi-goal E2E 互补 | 保留 | 删除会失去正式交互闭环 | 本身即 obstacle_field 正式 E2E |
| `test_interactive_preflight_failure.py` | 无路时零 setpoint/零 RPM/不起飞 | 规划安全 | 是，内部链相同 | 独特失败路径 | 保留 | 高：可能放行不安全任务 | 无等价替代 |
| `test_external_wrench.py` | 外力校验、应用、超时和默认关闭 | 抗扰底层安全 | 间接 | disturbance demo 覆盖正常路径 | 保留 | 删除会失去默认禁用与超时边界 | 无等价替代 |
| `test_horizontal_integral_node.py` | 积分生命周期、禁用和安全输出 | 抗扰控制 | 间接 | 扰动 E2E 只覆盖系统结果 | 保留 | 删除会弱化 anti-windup 回归 | 无等价替代 |
| `test_disturbance_demo_node.py` | 阶段、扰力和 Marker | 抗扰演示 | 是，节点相同 | visual Launch 测启动结构 | 保留 | 删除会失去时序语义覆盖 | assessment disturbance E2E |
| `test_disturbance_visual_launch.py` | persistent_release 参数与 external-wrench opt-in | 抗扰入口补充 | 间接 | 正式 E2E 覆盖默认 short_gust | 内部化，暂保留 | 删除会失去 persistent_release 自动覆盖 | 正式 short_gust E2E 已通过，persistent 长闭环尚缺 |
| `test_physical_parameter_consistency.py` | 动力学与控制器物理参数一致 | 全部飞行 | 是，配置相同 | 无 | 保留 | 高：参数漂移可破坏所有场景 | 无等价替代 |

本轮以上“待删除”全部只记为候选，不实际删除。删除前必须先让替代测试通过正式入口，
并确认历史量化结果仍可追溯；安全、碰撞、yaw 和完成门控不得放宽。

## 构建与测试

```bash
cd /home/peter/ros2_drone_sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

常用定向入口：

```bash
colcon test --packages-select drone_bringup \
  --ctest-args -R test_interactive_preflight_failure --output-on-failure
```

失败日志通常位于 `build/<package>/Testing/Temporary/LastTest.log` 和 `build/<package>/test_results/`。不同真实 Launch/E2E 不要并行复用同一 ROS Domain。

上面的绝对路径只描述本机默认工作区；README 和其他公开使用命令应使用 `~/ros2_drone_sim` 或当前工作目录。

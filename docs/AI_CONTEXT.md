# AI Context：ROS2 四旋翼仿真

## 稳定里程碑与开发边界

当前稳定里程碑：静态避障、交互导航、外部扰动与水平积分阶段。

后续开发应从最新 `main` 新建独立功能分支。本文只记录当前有效实现、正式参数和正式结果，不作为历史开发日志。

默认不要加入高度积分、姿态积分、障碍地图风扰、动态障碍或 MPC。只有新任务明确要求，或出现可复现且需要对应机制解决的物理问题时，才重新评估这些方向。

## 环境与包

- 目标环境：Ubuntu 22.04、ROS2 Humble、C++17。
- 工作区：`/home/peter/ros2_drone_sim`。
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
| `waypoint_manager_node` | 无障碍离散多目标任务 |
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
- `/drone/mission/current_waypoint_index`、`/drone/mission/complete`。

### 交互导航

- `/drone/interactive_goals/goal_editor/update`：Interactive Marker update。
- `/drone/interactive_goals/selected_goals`、`preview_path`、`status`、`ready`、`count`。
- `/drone/interactive_goals/execute`：`ExecuteGoalSequence` Service。
- `/drone/interactive_mission/active`、`status`、`draft_revision`。

交互编辑器只在 READY 后允许执行。执行节点从新鲜实际 Odom 对完整目标序列重新预检；地面预检失败时保持零 setpoint/零 RPM，飞行开始后的失败则保持最近有限安全位置。

## Launch 入口

| Launch | 用途 |
|---|---|
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
- 当前静态避障任务要求或生成零 yaw；未按路径切线规划机头方向。

## 当前正式结果

### 多目标静态避障

数据源：`results/horizontal_integral_upgrade/selected/multi_goal/metrics.json`。

- 任务成功，3 个目标全部访问，Launch 到完成 `139.203712 s`。
- 最大跟踪误差 `0.028843 m`。
- 对基础膨胀障碍物最小净空 `0.094310 m`。
- 最终误差 `0.001536 m`，最终速度 `0.004355 m/s`。
- 无点/线段碰撞、非有限值或控制器饱和；饱和计数 `0`。

### 外力与水平积分

数据源：`results/horizontal_integral_upgrade/selected/regression_summary.json` 和 `repeat_results.json`。

- **PD baseline**：水平积分关闭；持续 `0.30 N` 下末 3 秒平均误差 `0.749340 m`。
- **当前水平 PD+I+FF 正式基线**：`ki=0.15`、积分加速度限制 `0.35 m/s²`；同场景末 3 秒平均误差 `0.081989 m`。
- 当前基线三次独立 `0.30 N × 10 s` 撤力实验：恢复时间 `4.600580–4.601050 s`，反向超调 `0.107763–0.107767 m`，均无控制器饱和。

旧 PD 数据只作为对照，不能称为当前最终控制结果；其他候选和扫描位于 `results/`，不应逐项复制到用户文档。

### 自动测试

本轮最终全量回归：构建 6 个 package 成功；`288 tests, 0 errors, 0 failures, 0 skipped`。

## 当前限制

- 高度环、姿态环仍为 PD；只有水平位置环具有受限积分。
- 外力是质心处集中等效力，不是完整空间风场。
- 静态环境没有物理碰撞反作用。
- 没有动态障碍、局部规划和在线重规划。
- 静态避障 yaw 为零或未结合路径方向。
- 普通无障碍单目标只能通过 `/drone/goal` Topic 输入，缺少一键交互入口。
- 普通无障碍位置实验没有独立目标 Marker。
- 交互执行首版不支持同一 Launch 内提交第二份任务、替换或抢占。

## 下一阶段优先级

1. 普通单目标增加一键输入入口和目标 Marker。
2. 静态避障增加基于路径切线或目标要求的 yaw 行为。
3. 整理整体报告与答辩材料。

速度优化不再是默认主线。高度积分、姿态积分、动态障碍、完整风场、状态估计等均为有明确需求时再评估的可选扩展。

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

# AI Context：ROS2 四旋翼仿真

## 稳定里程碑与开发边界

当前稳定里程碑：静态避障、交互导航、外部扰动与水平积分阶段。

后续开发应从最新 `main` 新建独立功能分支。本文只记录当前有效实现、正式参数和正式结果，不作为历史开发日志。

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

`results/interactive_goal_yaw/path_tangent_e2e.json` 记录未提交工作树上的三目标
`90°、180°、-90°` 自动闭环场景。任务依序成功，最大 yaw 参考跳变
`0.016216 rad`、最大参考变化率 `0.800737 rad/s`、最大位置跟踪误差
`0.020015 m`、最小净空 `0.240015 m`，无碰撞、饱和或非有限值。既有到点门控会在
中间目标 yaw 完全稳定前切段，目标接受瞬间机体 yaw 误差为
`0.735928/0.531219/0.035898 rad`；该现象不属于本轮交互输入链路允许修改的范围。
最终完整回归构建 6 个 package，并通过 `334` 项测试；记录明确标注测试对象为未提交
工作树，不将基线 SHA 冒充为被测提交。

### 外力与水平积分

数据源：`results/horizontal_integral_upgrade/selected/regression_summary.json` 和 `repeat_results.json`。

- **PD baseline**：水平积分关闭；持续 `0.30 N` 下末 3 秒平均误差 `0.749340 m`。
- **当前水平 PD+I+FF 正式基线**：`ki=0.15`、积分加速度限制 `0.35 m/s²`；同场景末 3 秒平均误差 `0.081989 m`。
- 当前基线三次独立 `0.30 N × 10 s` 撤力实验：恢复时间 `4.600580–4.601050 s`，反向超调 `0.107763–0.107767 m`，均无控制器饱和。

旧 PD 数据只作为对照，不能称为当前最终控制结果；其他候选和扫描位于 `results/`，不应逐项复制到用户文档。

### 自动测试

路径切线 yaw 提交候选已在
`558e30fbad58eb18d8ae0764c9d60ed60e42b76f` 上完成最终回归：构建 6 个
package 成功；`330 tests, 0 errors, 0 failures, 0 skipped`。实际命令、时间与统计记录在
`results/static_avoidance_yaw/full_regression.json`；其中 `tested_commit` 指向被测试提交，
不是后续验证记录提交。

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

## 下一阶段优先级

1. 整理整体报告与答辩材料。

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

上面的绝对路径只描述本机默认工作区；README 和其他公开使用命令应使用 `~/ros2_drone_sim` 或当前工作目录。

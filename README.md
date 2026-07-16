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
- [目标发布](#目标发布)
- [安全保护与参数基线](#安全保护与参数基线)
- [当前限制](#当前限制)
- [文档说明](#文档说明)

## 项目简介

本项目是一个基于 Ubuntu 22.04 和 ROS2 Humble 的小型四旋翼无人机仿真系统。系统以四个电机目标 RPM 为动力学输入，计算无人机的位置、速度、姿态和角速度，并通过 ROS2 Topic、TF 和 RViz2 形成可观察的闭环仿真环境。

核心算法与 ROS2 通信层分离，动力学、控制器、Motor Mixer、多目标点任务管理、分段五次轨迹、静态环境碰撞检查和三维栅格 A* 均可独立测试。项目后续目标包括路径简化、规划结果轨迹化和静态避障执行。

## 当前阶段

动力学、高度/yaw、单目标三维位置闭环、第一版多目标点顺序飞行、连续轨迹生成与跟踪、静态三维 AABB 环境、统一碰撞查询和三维 26 邻域 A* 均已完成。`planning_sim.launch.py` 可同时显示环境和 `/drone/planned_path` 原始栅格路径；规划结果目前尚未接入轨迹执行，也不会介入动力学或电机控制。

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
- MotorRPM 命令超时保护；
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
- 默认规划演示环境约为 `10 × 10 × 5.5 m`，包含两个错开的墙状障碍物并保持 `0.25 m` 安全半径；标准起点 `(0,0,1.5)` 和终点 `(8,5,1.5)` 均安全，起终点直线发生碰撞，同时已用一条墙顶上方的已知安全折线证明场景可行；该折线不是规划结果；
- 默认场景的三维 26 邻域 A* 已找到安全原始栅格路径；候选点和相邻边都经过碰撞检查，确定性结果为 `37` 个路径点、长度 `11.485281 m`、扩展 `5911` 个节点，当前路径从墙体侧面绕行且最大高度 `1.5 m`；
- RViz2 人工运行已确认三维工作空间边界、两个原始障碍物和透明安全膨胀区域可见，无人机模型与环境处于同一 `map` 坐标系；初始位置的碰撞状态为 `false`；
- RViz2 显示无人机模型、TF、历史 Path 和目标 Pose；
- 控制器退出后约 `0.30 s` 触发 MotorRPM watchdog，目标转速归零；控制器重启并重新发送目标后闭环恢复；
- 当前工作区最近一次完整测试结果为 `181 tests, 0 errors, 0 failures, 0 skipped`。

## 待完成场景

- 原始栅格路径的视线简化与规划结果轨迹化；
- 基于规划结果的自动飞行与静态避障；
- 长时间、扰动和极限工况稳定性验证；
- 飞行数据曲线、指标分析和最终实验整理。

`/drone/path` 是动力学实际状态的历史位姿；`/drone/reference_path` 是当前连续轨迹的参考路径；`/drone/planned_path` 是 A* 输出的原始栅格路径。三者用途不同，规划路径目前不会驱动无人机。

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
- `drone_planning`：与 ROS2 无关的静态 AABB 环境、点/线段碰撞检查，以及环境可视化与监测节点；
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

环境配置位于 `src/drone_bringup/config/environment.yaml`。默认 workspace 为 `[-1,9] × [-2.5,7.5] × [-0.5,5] m`，两个错开的墙状障碍物为 `(0,0,1.5) → (8,5,1.5)`、`0.25 m` 栅格分辨率的 A* 场景预留周围和上方通道；起终点和分辨率只配置在 `astar.yaml`，没有复制进环境参数。`/drone/environment/markers` 显示工作空间、原始障碍物和透明的安全膨胀区；`/drone/environment/in_collision` 只在收到有限且未超时的 Odom 时发布。无图形界面时可增加 `use_rviz:=false`。

启动默认三维 A* 规划与显示：

```bash
ros2 launch drone_bringup planning_sim.launch.py
```

规划参数位于 `src/drone_bringup/config/astar.yaml`；环境几何仍只来自共用的 `environment.yaml`。节点启动时规划一次，并以 transient-local QoS 发布 `/drone/planned_path`、`/drone/planning/success` 和 `/drone/planning/expanded_nodes`。当前发布的是未经简化或平滑的原始栅格路径，无人机不会自动沿该路径飞行。

常用检查：

```bash
ros2 node list
ros2 topic list
ros2 topic echo /drone/odom --once
ros2 run tf2_ros tf2_echo map base_link
```

## 目标发布

`/drone/goal` 使用 `geometry_msgs/msg/PoseStamped`。例如发布 `1.5 m` 高度、零 yaw 目标：

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

`trajectory_sim.launch.py` 将控制器切换为 `setpoint_source=trajectory`，订阅 `/drone/trajectory_setpoint`。该模式使用消息中的 position、velocity、acceleration 和 yaw；轨迹消息无效或超过 `0.20 s` 未更新时发布零 RPM，不回退到 `/drone/goal`。

## 安全保护与参数基线

正常 Launch 的主要参数位于：

- `drone_bringup/config/dynamics.yaml`；
- `drone_bringup/config/controller.yaml`。

当前稳定控制基线：

- 控制频率：`100 Hz`；
- 动力学频率：`200 Hz`；
- 高度：`Kp=3.0`、`Kd=3.5`；
- 水平位置：`Kp=0.4`、速度 `Kd=1.2`、最大加速度 `0.4 m/s²`、最大倾角 `0.08 rad`；
- roll/pitch 姿态：`Kp=4.0`、角速度 `Kd=0.35`、最大力矩 `1.0 N·m`；
- yaw：`Kp=1.0`、`Kd=0.40`、最大力矩 `0.20 N·m`；
- 名义稳态悬停转速：约 `10818.9 RPM/电机`；
- 地面约束：`enable_ground_contact=true`、`ground_z=0.0 m`；
- MotorRPM watchdog：`enable_motor_command_timeout=true`、`motor_command_timeout=0.30 s`。

控制器在没有有效目标、Odom 缺失或超时、frame/四元数非法、算法结果无效时主动发布零 RPM。动力学 watchdog 在已收到过命令但后续超时后，只把目标 RPM 设为零；电机实际转速继续按既有一阶模型自然衰减。

## 当前限制

- `WaypointManager` 多目标模式仍采用“到点并稳定后离散切换”的 stop-settle-switch 策略；连续飞行由独立的 `trajectory_sim.launch.py` 提供，不改变该语义；
- 地面模型只有质心 z 方向的无反弹、无摩擦刚性约束；
- 动力学暂不包含空气阻力、旋翼陀螺效应和传感器噪声；
- 当前控制器没有位置积分环和复杂反饱和机制；
- 用户已在 RViz2 中人工确认五点任务能够依次到达，各 waypoint 附近停稳后再切换；当前仍是 stop-settle-switch，不是平滑轨迹；
- 用户已在 RViz2 中人工确认无人机能够沿连续参考轨迹完成五点飞行，中间 waypoint 不再停稳后切换，整体运动连续；
- 当前静态环境和 A* 只提供几何查询、全局路径与可视化，不会阻止电机、改变动力学或产生物理撞击响应；
- 尚未实现路径简化、规划结果轨迹化、自动避障、在线重规划或动态障碍物；
- 更大倾角、外部扰动、高角速度和最大 RPM 极限工况仍需继续验证。

## 文档说明

本 README 只记录稳定方案、已验证能力和可复现运行方式。供后续 AI 快速接手的当前技术上下文位于 `docs/AI_CONTEXT.md`，AI 使用记录位于 `docs/ai_usage.md`。

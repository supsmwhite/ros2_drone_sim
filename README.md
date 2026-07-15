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

核心算法与 ROS2 通信层分离，动力学、控制器和 Motor Mixer 均可独立测试。项目最终目标包括三维位置控制、多目标点飞行、地图、路径规划和静态避障。

## 当前阶段

动力学、高度/yaw 闭环和单目标三维位置闭环均已完成并通过真实运行验收。当前控制器使用 `/drone/goal` 的 x、y、z 和 yaw，能够自动起飞、飞向单个三维目标并重新稳定。

## 总体方案

### 当前已实现运行链路

```text
/drone/goal
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

- 四个 `ament_cmake` package：`drone_msgs`、`drone_dynamics`、`drone_controller`、`drone_bringup`；
- 自定义 `drone_msgs/msg/MotorRPM` 消息；
- 四旋翼刚体动力学、电机一阶响应、RPM 限幅、X 型推力与力矩模型；
- 可配置的简化水平地面约束；
- `/drone/odom`、`/drone/imu`、`/drone/path` 和 `map -> base_link` TF；
- 与动力学符号一致的 Motor Mixer、姿态/角速度控制器和高度控制器；
- 高度控制器、姿态控制器和 Mixer 组成的 `HoverController`，并已接入 ROS2 控制节点；
- 世界系 x/y 位置和速度反馈的 `HorizontalPositionController`；
- 组合水平位置与现有 Hover 链路的 `PositionController`；
- 使用真实控制器、Mixer、电机一阶响应和刚体动力学的 20 秒姿态闭环稳定性测试；
- 通过真实 ROS2 Launch、节点和 Topic 的可重复单目标三维端到端 smoke test；
- MotorRPM 命令超时保护；
- Xacro 四旋翼模型、robot_state_publisher 和 RViz2 基础可视化；
- `basic_sim.launch.py` 一键启动动力学、控制器、机器人模型发布和 RViz2。

## 已验证场景

- 零 RPM 自由落体，以及正常 Launch 下零 RPM 保持在简化地面；
- 对称推力和独立 roll、pitch、yaw 力矩方向；
- 从地面自动起飞到 `1.5 m` 并稳定悬停；
- 高度在 `0 → 1.5 m`、`1.5 → 2.0 m`、`2.0 → 1.5 m` 间自动升降；
- yaw 转向能够快速接近目标，用户在 RViz2 中确认基本无超调；
- `0.02 rad` 正负 roll/pitch 固定命令和水平姿态均通过 20 秒完整闭环测试；
- 原地 `1.5 m` 悬停、`(0.5,0,1.5)` 小目标和 `(2,1,1.5)` 单目标三维飞行均通过真实 ROS2 数值验收；
- `(2,1,1.5)` 连续观察 27 秒后的三维误差约 `1.37e-7 m`，水平速度约 `7.84e-8 m/s`，无非有限值、姿态发散、RPM 边界值或日志饱和；
- 可重复的 ROS2 单目标端到端 smoke test 已验证 `(2,1,1.5)` 到达后连续稳定悬停至少 `2.0 s`；
- RViz2 显示无人机模型、TF、历史 Path 和目标 Pose；
- 控制器退出后约 `0.30 s` 触发 MotorRPM watchdog，目标转速归零；控制器重启并重新发送目标后闭环恢复；
- 当前工作区最近一次完整测试结果为 `106 tests, 0 errors, 0 failures, 0 skipped`。

## 待完成场景

- 轨迹生成、轨迹跟踪和多目标点飞行；
- 障碍物地图；
- 路径规划和静态避障；
- 长时间、扰动和极限工况稳定性验证；
- 飞行数据曲线、指标分析和最终实验整理。

`/drone/path` 当前只是动力学状态的历史位姿显示，不代表轨迹生成或轨迹跟踪已经实现。

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
│   └── drone_bringup/
├── tools/
├── results/
└── report/
```

- `drone_msgs`：项目自定义 ROS2 消息；
- `drone_dynamics`：纯动力学模型、ROS2 动力学节点及单元测试；
- `drone_controller`：高度、姿态、Mixer、HoverController 和 ROS2 控制节点；
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

- 尚未实现轨迹生成与跟踪、多目标点、地图、规划和避障；
- 地面模型只有质心 z 方向的无反弹、无摩擦刚性约束；
- 动力学暂不包含空气阻力、旋翼陀螺效应和传感器噪声；
- 当前控制器没有位置积分环和复杂反饱和机制；
- 本轮三维目标验收为无 RViz 的数值观测；新的三维飞行动画效果尚未单独进行人工确认；
- 更大倾角、外部扰动、高角速度和最大 RPM 极限工况仍需继续验证。

## 文档说明

本 README 只记录稳定方案、已验证能力和可复现运行方式。供后续 AI 快速接手的当前技术上下文位于 `docs/AI_CONTEXT.md`，AI 使用记录位于 `docs/ai_usage.md`。

# ROS2 四旋翼无人机仿真系统

## 项目简介

本项目是一个基于 ROS2 Humble 开发的小型四旋翼无人机仿真系统，用于完成无人机动力学、位置与姿态控制、地图构建、路径规划及避障等功能。

项目以四个电机转速 RPM 作为动力学输入，通过四旋翼动力学模型计算无人机的位置、速度、姿态和角速度；控制器根据目标点与当前状态，计算四个电机的目标转速，从而形成完整的闭环仿真系统。

项目最终将在 RViz2 中完成无人机状态、飞行轨迹、目标点、障碍物地图及规划路径的可视化。

## 项目目标

项目计划实现以下功能：

1. 四旋翼六自由度动力学仿真；
2. 电机一阶响应、推力和力矩模型；
3. 位置与姿态串级控制；
4. 悬停、单目标点和多目标点飞行；
5. RViz2 无人机模型、轨迹和目标点可视化；
6. 静态障碍物地图生成与显示；
7. 路径规划和静态避障；
8. 飞行数据记录、曲线绘制和指标分析；
9. 参数文件配置与一键启动。

## 总体方案

系统的基本数据流如下：

```text
目标点
  ↓
路径规划与避障模块
  ↓
安全目标点或 Waypoint
  ↓
位置与姿态控制器
  ↓
4 个电机目标 RPM
  ↓
四旋翼动力学模型
  ↓
位置、速度、姿态和角速度
  ↓
状态反馈与 RViz2 可视化
```

项目采用模块化 ROS2 节点设计，核心算法与 ROS2 通信接口分离，便于独立测试和后续扩展。

## 当前完成情况

项目目前处于初始化阶段。

已完成并验证的工程初始化内容：

* 建立 ROS2 工作空间和 Git 仓库；
* 创建 `drone_msgs`、`drone_dynamics`、`drone_controller` 和 `drone_bringup`；
* 生成四电机 RPM 自定义消息；
* 实现并独立验证四旋翼刚体动力学、电机一阶响应和 X 型力矩分配；
* 动力学节点可发布 Odom、IMU、Path 和 `map -> base_link` TF；
* 正常 Launch 启用可配置的简化水平地面，启动时不会继续落入负 z；
* 实现并独立验证与动力学电机布局和力矩符号一致的 Motor Mixer；
* 实现并独立验证姿态/角速度控制器和带倾斜补偿的高度控制器纯算法；
* 已将高度、姿态和 Mixer 组合接入控制节点，可根据 `map` 高度目标自动起飞并悬停；
* 创建与 X 型布局一致的基础四旋翼 Xacro 模型；
* RViz2 可显示无人机模型、TF、轨迹、目标点、坐标轴和网格；
* 使用基础 Launch 文件启动动力学、高度/姿态闭环控制节点、robot_state_publisher 和 RViz2。

动力学模块、Motor Mixer、姿态/角速度控制器、高度控制器和基础 RViz2 可视化已完成独立测试。高度/姿态闭环已实际从地面起飞至 `1.5 m` 并稳定悬停；调参后的高度响应在实际复测中平稳收敛且未观察到回弹，yaw 转向也经用户在 RViz2 中确认能够快速到达且基本无超调。当前目标的 x/y 会被忽略，水平位置控制尚未实现。

当前地面模型只约束世界系竖直位置和向下速度，不包含反弹、摩擦、起落架弹性、姿态约束或复杂碰撞。

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

各模块的计划职责：

* `drone_msgs`：项目自定义 ROS2 消息；
* `drone_dynamics`：四旋翼动力学模型与状态更新；
* `drone_controller`：位置环、姿态环和电机 Mixer；
* `drone_map`：障碍物地图生成、加载和发布；
* `drone_planner`：碰撞检测、路径规划和安全目标点生成；
* `drone_bringup`：参数、Launch、URDF 和 RViz2 配置；
* `tools`：目标发布、数据处理和自动评测脚本；
* `results`：实验数据、曲线和截图；
* `report`：最终报告及相关图片；
* `docs`：AI 工作上下文和 AI 使用说明。

## 环境与依赖

已验证的基础开发环境：

* Ubuntu 22.04.5；
* ROS2 Humble；
* g++ 11.4.0，使用 C++17；
* CMake 3.22.1；
* Eigen3 3.4.0；
* colcon、rosdep 和 ament_cmake；
* tf2；
* RViz2；
* ROS2 常用消息包。

最终使用的完整依赖和安装方式将在项目实现过程中补充。

## 编译与运行

当前四个基础 package 已使用以下命令完成编译验证：

```bash
cd ~/ros2_drone_sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

基础系统已使用以下命令完成启动验证：

```bash
ros2 launch drone_bringup basic_sim.launch.py
```

该 Launch 启动 `quadrotor_dynamics_node` 和 altitude-hover 模式的 `position_controller_node`。发送目标前控制器持续发布零 RPM；收到有效 `map` 目标后只控制目标 z 和 yaw。

正常 Launch 默认设置 `enable_ground_contact=true`、`ground_z=0.0`：零 RPM 时模型停在地面，推力超过重力后可以正常离地。纯 `QuadrotorModel` 的地面约束默认关闭，以保留自由落体测试行为。

默认同时启动 robot_state_publisher 和 RViz2。无界面运行可使用：

```bash
ros2 launch drone_bringup basic_sim.launch.py use_rviz:=false
```

例如发布 `1.5 m` 高度、零 yaw 目标：

```bash
ros2 topic pub --once /drone/goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {z: 1.5}, orientation: {w: 1.0}}}"
```

当前模式忽略目标 x/y。控制器正常运行时会对缺少目标、里程计超时和非法输入主动发布零 RPM；控制器进程完全退出后的动力学命令超时保护尚未实现。

当前已验证的控制参数基线位于 `drone_bringup/config/controller.yaml`。高度 PD 使用 `Kp=3.0`、`Kd=3.5`；yaw 使用 `Kp=1.0`、`Kd=0.40`、最大力矩 `0.20 N·m`。

完整地图与避障系统计划通过以下命令启动：

```bash
ros2 launch drone_bringup full_sim.launch.py
```

完整地图与避障 Launch 尚未实现，不能运行。

## 计划验收场景

项目将依次完成以下实验：

1. 动力学基础响应测试；
2. 在 `(0, 0, 1.5)` 附近稳定悬停；
3. 飞往目标点 `(2, 1, 1.5)` 并稳定悬停；
4. 按顺序飞行多个目标点；
5. 在静态障碍物地图中完成绕行；
6. 完成狭窄通道或明显绕行场景；
7. 输出位置误差、RPM、轨迹和最小障碍物距离曲线。

目标点悬停误差应收敛至 `0.3 m` 以内，避障实验中的最小障碍物距离应大于设定安全距离。

## 验收方式

每个模块必须经过独立测试和系统联合测试。

功能只有在满足以下条件后，才视为已经完成：

* 可以正常编译；
* 节点可以稳定运行；
* Topic 和 TF 数据正确；
* 实际运行结果符合预期；
* 有明确的测试步骤；
* 重要结果有日志、曲线或截图记录。

## 项目状态说明

本 README 仅记录已经确定的系统方案和稳定实现结果。

实时开发进度、当前问题、技术决策及下一步任务记录在：

```text
docs/AI_CONTEXT.md
```

AI 辅助编程使用情况记录在：

```text
docs/ai_usage.md
```

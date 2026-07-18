# ROS2 四旋翼无人机仿真系统

## 项目简介

这是一个运行于 Ubuntu 22.04、ROS2 Humble 的四旋翼闭环仿真项目。系统使用四电机目标 RPM 驱动刚体动力学，覆盖位置、高度和姿态控制，并通过 Odom、IMU、TF 与 RViz2 展示飞行状态。

项目已形成从任务输入、三维静态规划、安全连续轨迹到跟踪执行的完整链路，也支持多目标任务、RViz 三维目标编辑，以及质心处外部集中扰力和抗扰恢复实验。算法核心与 ROS2 节点适配层分离，关键模块具有单元、集成和真实 Launch 端到端测试。

## 当前里程碑

当前阶段已完成：

- 基础悬停与单目标三维位置控制；
- 无障碍多目标顺序飞行；
- 分段五次连续轨迹生成与跟踪；
- 有限三维静态环境和统一碰撞检查；
- 三维 26 邻域 A*；
- 路径简化、安全连续轨迹生成及动态约束验证；
- 单目标和多目标静态避障；
- RViz 三维目标编辑、预览、预检和执行；
- 质心处外部集中等效力注入与可视化；
- 水平位置受限积分、积分卸载和 anti-windup；
- 自动测试、可复现实验工具和结构化结果记录。

## 快速开始

```bash
cd ~/ros2_drone_sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

常用一键入口：

```bash
ros2 launch drone_bringup basic_sim.launch.py
```

基础动力学、位置控制器、模型和 RViz2；等待 `/drone/goal`。

```bash
ros2 launch drone_bringup mission_sim.launch.py
```

运行无障碍离散多目标顺序任务。

```bash
ros2 launch drone_bringup trajectory_sim.launch.py
```

运行无障碍分段五次连续轨迹任务。

```bash
ros2 launch drone_bringup static_avoidance_sim.launch.py
```

运行单目标三维静态避障。

```bash
ros2 launch drone_bringup multi_goal_static_avoidance_sim.launch.py
```

运行正式三目标静态避障任务。

```bash
ros2 launch drone_bringup interactive_goal_navigation_sim.launch.py
```

在 RViz 中自行选择多个目标，预览后执行导航。

```bash
ros2 launch drone_bringup disturbance_visual_demo.launch.py profile:=short_gust
```

运行带外力、误差和水平积分箭头的短时扰动演示。

不同仿真 Launch 不要同时运行在同一个 ROS Domain。

## 典型演示

### 基础悬停

`disturbance_hover_sim.launch.py` 会自动发布 `(0,0,1.5)` 悬停目标；除非另有节点向外力 Topic 发布消息，否则不会主动施加外力。

```bash
ros2 launch drone_bringup disturbance_hover_sim.launch.py
```

### 单目标 `(2,1,1.5)`

终端 1：

```bash
ros2 launch drone_bringup basic_sim.launch.py
```

终端 2：

```bash
source /home/peter/ros2_drone_sim/install/setup.bash

ros2 topic pub --once /drone/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0, z: 1.5}, orientation: {w: 1.0}}}"
```

这是当前核心演示中唯一必须使用第二个终端输入目标的基础实验。

### 任务、轨迹与避障

- 无障碍多目标任务：`ros2 launch drone_bringup mission_sim.launch.py`
- 无障碍连续轨迹：`ros2 launch drone_bringup trajectory_sim.launch.py`
- 单目标静态避障：`ros2 launch drone_bringup static_avoidance_sim.launch.py`
- 多目标静态避障：`ros2 launch drone_bringup multi_goal_static_avoidance_sim.launch.py`

### RViz 交互导航

```bash
ros2 launch drone_bringup interactive_goal_navigation_sim.launch.py
```

在 RViz 工具栏选择 `Interact`，拖动 `goal_candidate` 的水平控制面和竖直箭头。右键依次使用 `Add Goal` 添加多个目标，选择 `Validate & Preview`；状态为 READY 且蓝色预览完整后，选择 `Execute Validated Mission`。执行前会从实际 Odom 再次预检完整序列，任一段无安全路径时均不起飞。

### 扰动实验

短时 `+X 0.30 N × 2 s`：

```bash
ros2 launch drone_bringup disturbance_visual_demo.launch.py profile:=short_gust
```

持续 `+X 0.30 N × 10 s` 并撤力恢复：

```bash
ros2 launch drone_bringup disturbance_visual_demo.launch.py profile:=persistent_release
```

红色箭头表示质心处集中等效外力，蓝色箭头表示水平积分产生的世界系加速度补偿；两者都不表示真实风速或完整风场。

## 系统架构

控制与动力学链：

```text
任务/目标/规划
→ 位置控制器
→ 高度与姿态控制器
→ Motor Mixer
→ 四电机 RPM
→ 四旋翼动力学
→ Odom/IMU/TF/RViz
```

静态规划链：

```text
静态环境
→ CollisionChecker
→ 3D A*
→ PathSimplifier
→ Piecewise Quintic Trajectory
→ 跟踪执行
```

六个 ROS2 package 为 `drone_msgs`、`drone_dynamics`、`drone_controller`、`drone_mission`、`drone_planning` 和 `drone_bringup`。控制器输入可为 `/drone/goal` 或 `/drone/trajectory_setpoint`，输出 `/drone/motor_rpm_cmd`；动力学发布 `/drone/odom`、`/drone/imu`、`/drone/path` 和 `map → base_link` TF。

控制律应准确理解为：水平位置 `P + D + 受限 I + 期望加速度前馈`；高度 `PD + 重力/前馈/倾角补偿`；姿态 `P + 角速度 D`；Mixer 为带 RPM 限制的 X 构型。整个飞控不是完整 PID。

## 核心能力与验证结果

正式多目标静态避障任务依次访问 `(13.2,5.5,1.5)`、`(7.0,5.0,4.0)`、`(0.8,0.7,2.0)`：

| 指标 | 正式结果 |
|---|---:|
| Launch 到任务完成 | `139.203712 s` |
| 最大跟踪误差 | `0.028843 m` |
| 对基础膨胀障碍物最小净空 | `0.094310 m` |
| 最终位置误差 | `0.001536 m` |
| 最终速度 | `0.004355 m/s` |
| 碰撞 / 控制器饱和 | `无 / 0` |

持续 `0.30 N` 外力下，以末 3 秒平均误差作为稳态指标：

| 控制基线 | 稳态误差 |
|---|---:|
| PD baseline（水平积分关闭） | `0.749340 m` |
| 当前水平 PD+I+FF 正式基线 | `0.081989 m` |

当前基线的三次独立 `0.30 N × 10 s` 撤力重复实验：恢复时间 `4.600580–4.601050 s`，反向超调 `0.107763–0.107767 m`；三次均无控制器饱和。

本轮最终全量回归：`288 tests, 0 errors, 0 failures, 0 skipped`。

正式数据见：

- `results/horizontal_integral_upgrade/selected/`
- `results/horizontal_integral_upgrade/repeat_results.json`

历史候选和参数扫描保留在 `results/` 对应子目录，仅用于追溯，不作为当前控制基线。

## 人工验收 A～J

正常人工演示只运行“主命令”；Topic、频率和参数命令属于可选诊断。A～I 完成后只执行一次 J。

### A. 编译与环境

```bash
cd /home/peter/ros2_drone_sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | grep '^drone_'
```

通过标准：构建成功，并发现六个项目 package。

### B. 基础悬停

```bash
ros2 launch drone_bringup disturbance_hover_sim.launch.py
```

通过标准：自动起飞至 `z=1.5 m` 并稳定，无非有限值、明显姿态发散或持续饱和。

### C. 单目标三维位置控制

按“典型演示”的两终端步骤发布 `(2,1,1.5)`。通过标准：到达目标附近并稳定悬停。

### D. 无障碍多目标顺序飞行

```bash
ros2 launch drone_bringup mission_sim.launch.py
```

通过标准：严格按序访问任务点，每点停稳后切换，最终完成并保持。

### E. 短时扰动

```bash
ros2 launch drone_bringup disturbance_visual_demo.launch.py profile:=short_gust
```

通过标准：阶段、红色外力箭头、蓝色积分箭头和运动一致，撤力后完成恢复。

### F. 持续扰动与撤力恢复

```bash
ros2 launch drone_bringup disturbance_visual_demo.launch.py profile:=persistent_release
```

通过标准：积分补偿方向正确，持续扰动偏差减小，撤力恢复平顺且最终收敛。

### G. 静态多目标避障

```bash
ros2 launch drone_bringup multi_goal_static_avoidance_sim.launch.py
```

通过标准：P1→P2→P3 全部完成，无碰撞、主动外力、非有限值或持续饱和。

### H. RViz 交互目标导航

```bash
ros2 launch drone_bringup interactive_goal_navigation_sim.launch.py
```

使用 `Interact` 拖动候选，依次 `Add Goal`，再 `Validate & Preview` 和 `Execute Validated Mission`。通过标准：至少一组自行选择的多目标序列安全完成，非法目标被拒绝。

### I. 预检失败安全

```bash
colcon test --packages-select drone_bringup \
  --ctest-args -R test_interactive_preflight_failure --output-on-failure
colcon test-result --verbose
```

通过标准：闭合墙场景报告无路径，测试无 failure/error，有效 setpoint 数为 0、RPM 为 0、无人机不离地。

### J. 最终自动全量测试

```bash
colcon test
colcon test-result --verbose
```

通过标准：`failures=0`、`errors=0`。失败时查看 `build/<package>/Testing/Temporary/LastTest.log` 和对应 `test_results/`。

### 可选诊断

```bash
ros2 topic hz /drone/odom
ros2 topic echo /drone/controller/diagnostics --once
ros2 topic echo /drone/environment/in_collision --once
ros2 topic echo /drone/interactive_goals/status
ros2 topic echo /drone/interactive_mission/status
ros2 topic echo /drone/external_wrench/applied
ros2 param get /quadrotor_dynamics_node enable_external_wrench
```

## 当前限制

- 高度环和姿态环仍为 PD；只有水平位置环具有受限积分。
- 外力是作用于质心的集中等效力，不是空间变化的完整风场。
- 静态环境提供碰撞检查和状态监测，不模拟物理碰撞反作用。
- 尚无动态障碍、局部规划和在线重规划。
- 静态避障任务当前 yaw 为零或未结合路径方向规划。
- 普通无障碍单目标通过 `/drone/goal` Topic 输入，缺少一键参数或交互入口。
- 普通无障碍位置实验尚未单独显示目标 Marker。

后三项主要影响展示和交互完整性，不影响当前已验证的规划、控制与安全链路。

## 后续优化

近期展示与交互优化：

1. 为普通单目标实验增加一键参数或交互输入，并显示目标点 Marker；
2. 静态避障根据路径切线或目标要求设置 yaw；
3. 整理整体报告和答辩材料。

可选扩展：只有出现明确物理需求时再评估垂向持续扰动与高度积分；其后可研究动态障碍和在线重规划、更完整的时空风场、传感器噪声与状态估计。这些不是当前里程碑缺陷或默认必做项。

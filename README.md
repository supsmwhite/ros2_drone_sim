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

### 静态避障路径切线 yaw

三个静态避障入口默认使用兼容模式 `yaw_mode:=fixed`，保持原有固定 yaw。显式选择
`path_tangent` 后，yaw 参考跟随五次轨迹的水平速度切线；水平速度低于 `0.10 m/s`
时保持最近有效方向，通过最短角误差避免 `±π` 跳变，并在目标前 `0.80 m` 内平滑
混合到当前目标指定 yaw。最终参考还经过 `0.30 s` 一阶滤波和 `0.80 rad/s`
速率限制。

```bash
ros2 launch drone_bringup static_avoidance_sim.launch.py yaw_mode:=fixed
ros2 launch drone_bringup static_avoidance_sim.launch.py yaw_mode:=path_tangent
ros2 launch drone_bringup multi_goal_static_avoidance_sim.launch.py yaw_mode:=path_tangent
ros2 launch drone_bringup interactive_goal_navigation_sim.launch.py yaw_mode:=path_tangent
```

可覆盖参数为 `fixed_yaw`、`tangent_speed_threshold`、
`terminal_blend_distance`、`yaw_filter_time_constant` 和 `max_yaw_rate`。该能力只是
基于轨迹水平切线的平滑 yaw 参考生成，不改变 A*、路径简化、位置轨迹、碰撞检查或
控制器，也不是完整姿态规划或最优 yaw 规划。

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
source ~/ros2_drone_sim/install/setup.bash

ros2 run drone_mission goal_cli single 2.0 1.0 1.5 0.0
```

工具会检查输入和工作空间，等待 `/drone/goal` 订阅者并发布 `map` 系目标。
原有 `ros2 topic pub` 方式仍然兼容。
单目标 Marker 始终标记为 `GOAL CURRENT`；可视化节点不根据 Odom 推断任务完成。

最后一项可继续使用弧度，也可用更直观的角度格式。例如朝向 90°：

```bash
ros2 run drone_mission goal_cli single 2.0 1.0 1.5 yaw=90
```

### 命令行多目标

终端 1 启动等待运行时任务的无障碍仿真：

```bash
ros2 launch drone_bringup mission_sim.launch.py start_with_configured_waypoints:=false
```

终端 2 提交任意数量的 `[x y z yaw]` 目标组：

```bash
ros2 run drone_mission goal_cli multi \
  0.0 0.0 1.5 yaw=0 \
  2.0 0.0 1.5 yaw=0 \
  2.0 1.5 1.8 yaw=90
```

任务通过 `/drone/mission/execute` 提交。任务执行中不允许抢占，新请求会返回
拒绝原因。默认 `mission_sim.launch.py` 仍从 `config/mission.yaml` 自启动，也可用
`mission_config:=<yaml>` 选择任务文件。

### 任务、轨迹与避障

- 无障碍多目标任务：`ros2 launch drone_bringup mission_sim.launch.py`
- 无障碍连续轨迹：`ros2 launch drone_bringup trajectory_sim.launch.py`
- 单目标静态避障：`ros2 launch drone_bringup static_avoidance_sim.launch.py`
- 多目标静态避障：`ros2 launch drone_bringup multi_goal_static_avoidance_sim.launch.py`

### RViz 交互导航

```bash
ros2 launch drone_bringup interactive_goal_navigation_sim.launch.py
```

在 RViz 工具栏选择 `Interact`，拖动 `goal_candidate` 的水平控制面设置 x/y、竖直箭头设置高度，并用水平旋转环自由设置世界 Z 轴 yaw。右键 `Set Yaw` 可精确选择 `0°、±45°、±90°、±135°、180°`。右键依次使用 `Add Goal` 添加多个目标，选择 `Validate & Preview`；状态为 READY 且蓝色预览完整后，选择 `Execute Validated Mission`。执行前会从实际 Odom 再次预检完整序列，任一段无安全路径时均不起飞。

候选与每个已添加目标都独立保存位置和终端 yaw，方向箭头及 `P<n> yaw=<角度>` 标签显示保存结果。改变位置、高度或 yaw 都会使旧预览失效，需重新验证。当前不支持直接编辑任意历史目标；请用 `Undo Last Goal` 恢复最后一个目标为候选，修改后重新添加。`Print Mission YAML` 输出可复制的 `[x,y,z,yaw]` 弧度格式，并保留 6 位小数。

默认 `yaw_mode:=fixed` 仍让整段任务使用全局 `fixed_yaw`；目标 yaw 会被保存和传递，但不改变兼容行为。`yaw_mode:=path_tangent` 飞行时跟随路径切线，并在每个终点平滑过渡到该交互目标的 yaw。这项能力准确称为“交互式多目标终端 yaw 编辑与路径切线 yaw 执行”，不是完整姿态规划。

三目标终端 yaw 演示：

```bash
ros2 launch drone_bringup interactive_goal_navigation_sim.launch.py \
  yaw_mode:=path_tangent
```

依次设置 `P1=90°、P2=180°、P3=-90°`，再执行 `Validate & Preview` 和 `Execute Validated Mission`。默认 fixed 对照仍使用不带参数的同一 Launch 命令。

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

路径切线 yaw 定向验证中，单目标任务最大相邻 yaw 跳变 `0.016161 rad`、最大 yaw
参考变化率 `0.800119 rad/s`、最终 yaw 误差 `0 rad`；三目标任务对应为
`0.016328 rad`、`0.800272 rad/s`、`0 rad`，目标顺序、轨迹安全和完成状态保持正常。
完整 fixed 对照与 path-tangent 指标见 `results/static_avoidance_yaw/`。

交互终端 yaw 自动闭环场景使用 `90°、180°、-90°`，三目标依序完成；最大相邻
yaw 参考跳变 `0.016216 rad`、最大参考变化率 `0.800737 rad/s`、最大位置跟踪误差
`0.020015 m`、最小膨胀障碍净空 `0.240015 m`，无碰撞、饱和或非有限值。既有
位置/速度到点门控会在中间目标 yaw 完全稳定前切换下一段，目标接受瞬间的机体 yaw
误差为 `0.735928/0.531219/0.035898 rad`；本轮按边界只记录该现象，未修改 yaw
算法、控制器或保持参数。工作树来源和完整指标见 `results/interactive_goal_yaw/`。
同一提交候选工作树的最终完整回归构建 6 个 package，`334 tests, 0 errors,
0 failures, 0 skipped`；来源命令和工作树状态见该目录的 `full_regression.json`。

持续 `0.30 N` 外力下，以末 3 秒平均误差作为稳态指标：

| 控制基线 | 稳态误差 |
|---|---:|
| PD baseline（水平积分关闭） | `0.749340 m` |
| 当前水平 PD+I+FF 正式基线 | `0.081989 m` |

当前基线的三次独立 `0.30 N × 10 s` 撤力重复实验：恢复时间 `4.600580–4.601050 s`，反向超调 `0.107763–0.107767 m`；三次均无控制器饱和。

路径切线 yaw 提交候选已在 `558e30fbad58eb18d8ae0764c9d60ed60e42b76f`
上完成最终回归：构建 6 个 package 成功，
`330 tests, 0 errors, 0 failures, 0 skipped`；真实命令和统计见
`results/static_avoidance_yaw/full_regression.json`。提交前场景指标继续保留其工作树来源，
没有改写为该 SHA，也不表示人工 RViz 视觉验收已由自动测试替代。

正式数据见：

- `results/horizontal_integral_upgrade/selected/`
- `results/horizontal_integral_upgrade/repeat_results.json`

历史候选和参数扫描保留在 `results/` 对应子目录，仅用于追溯，不作为当前控制基线。

## 人工验收 A～J

正常人工演示只运行“主命令”；Topic、频率和参数命令属于可选诊断。A～I 完成后只执行一次 J。

### A. 编译与环境

```bash
cd ~/ros2_drone_sim
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
- 无障碍命令行任务不支持抢占；执行中提交的新任务会被明确拒绝。

## 后续优化

近期展示与交互优化：

1. 静态避障根据路径切线或目标要求设置 yaw；
2. 整理整体报告和答辩材料。

可选扩展：只有出现明确物理需求时再评估垂向持续扰动与高度积分；其后可研究动态障碍和在线重规划、更完整的时空风场、传感器噪声与状态估计。这些不是当前里程碑缺陷或默认必做项。

## References and acknowledgments

任务指定参考项目：

- [pengyu_sim](https://gitee.com/potato77/pengyu_sim)
- [MARSIM](https://github.com/hku-mars/MARSIM)

这些项目仅用于理解无人机仿真、动力学、控制和系统组织思路。本仓库不是二者的代码移植版本，也未复制或改编其源代码、模型、地图、配置、图片或其他资源；ROS2 包结构、节点、算法、配置、测试和文档均在本项目中独立完成。

## License

本项目采用 Apache License 2.0，完整条款见根目录 `LICENSE`。

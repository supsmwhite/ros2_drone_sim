# AI 上下文：ROS2 四旋翼考核项目

本文供后续 AI 或开发者快速接管项目，记录稳定的产品边界、依赖关系、正式协议与验证
状态，不作为开发日志或实验复盘。

## 1. 当前状态

项目基于 Ubuntu 22.04、ROS2 Humble、C++17 和 Eigen3。功能与正式考核协议已经收束：
公开三个仿真入口，使用一张 `environment.yaml` 静态地图，保留五组基础/导航场景和
两组独立抗扰场景。

七组正式结果已经完成自动分析、人工验收和 finalize，所有正式 manifest 条目均为
`report_eligible=true`。04 登记 1 张 RViz 截图，05 登记 3 张 RViz 截图，Reviewer
为 `Peter`。

## 2. 产品边界

系统提供四电机目标 RPM 驱动的刚体动力学，位置、速度与姿态闭环控制，单目标与
多目标顺序任务，三维 A*、路径简化、安全连续轨迹、`path_tangent` yaw、静态碰撞与
安全净空监测、RViz 交互目标，以及短时/持续外力抗扰演示。

水平位置环为 `P + D + 受限 I + 期望加速度前馈`，高度和姿态环为 PD。静态障碍只用于
规划和碰撞监测，不产生物理接触；外力是质心处集中等效力。系统不包含动态障碍、局部
重规划、完整风场、MPC 或完整姿态规划。

## 3. 公开入口

| 入口 | 默认参数 | 用途 |
|---|---|---|
| `assessment_basic_sim.launch.py` | `use_rviz:=true` | 等待 `goal_cli single` 或 `goal_cli multi`，不自动执行 YAML waypoint |
| `assessment_navigation_sim.launch.py` | `yaw_mode:=path_tangent`、`use_rviz:=true` | RViz/Service 目标、预检、三维规划与执行 |
| `assessment_disturbance_sim.launch.py` | `profile:=short_gust`、`use_rviz:=true` | 短时外力演示；另支持 `profile:=persistent_release` |

基础正式 multi 在 Recorder 启动前完成 `(0,0,1.5,0°)` 预备悬停；正式记录只包含
`(3,0,1.5,0°)` → `(3,3,1.5,90°)` → `(0,3,1.5,180°)` →
`(0,0,1.5,-90°)` 四个目标。

## 4. 包与节点职责

| 包 | 关键节点/接口 | 职责 |
|---|---|---|
| `drone_msgs` | 自定义 msg/srv | 任务、状态与执行接口 |
| `drone_dynamics` | `quadrotor_dynamics_node` | 四电机 RPM 驱动刚体动力学与可选外力输入 |
| `drone_controller` | `position_controller_node` | 位置、速度、姿态和电机混控闭环 |
| `drone_mission` | `waypoint_manager_node`、`goal_visualizer_node`、`goal_cli` | 运行时任务、顺序执行、基础目标 Marker 与命令行提交 |
| `drone_planning` | `static_environment_node` | 工作空间、障碍 Marker、碰撞与净空统计 |
| `drone_planning` | `interactive_goal_editor_node` | RViz 目标编辑、预览、预检和执行服务入口 |
| `drone_planning` | `multi_goal_static_avoidance_node` | 多目标 A*、简化、连续轨迹、yaw 与完成门控 |
| `drone_bringup` | `disturbance_demo_node` | 扰动时序、外力和积分补偿可视化 |

`robot_state_publisher` 与可选 `rviz2` 属于公共可视化基础设施。

## 5. Launch 依赖

```text
assessment_basic_sim
└── mission_sim
    └── basic_sim
        └── simulation_core

assessment_navigation_sim
└── interactive_goal_navigation_sim
    └── simulation_core

assessment_disturbance_sim
└── disturbance_visual_demo
```

内部 Launch 是三个公开入口的实现组件，不应与公开入口并列宣传。

## 6. 数据流

基础任务：

```text
goal_cli single/multi → waypoint_manager_node → /drone/goal
→ position_controller_node → /drone/motor_rpm_cmd → quadrotor_dynamics_node
```

交互导航：

```text
RViz/Service 目标 → 交互编辑、预览与预检 → 3D A* → 路径简化
→ 安全连续轨迹 → path_tangent/终端 yaw → /drone/trajectory_setpoint
→ position_controller_node → quadrotor_dynamics_node
```

抗扰演示：

```text
disturbance_demo_node → 目标与外力时序
→ quadrotor_dynamics_node（启用 external wrench）→ position_controller_node
→ 外力/积分补偿 Marker 与恢复状态
```

输出主要包括 Odometry、IMU、控制诊断、任务/导航状态、Marker，以及 Recorder 生成的
CSV、路径、事件、summary 和图表。

## 7. 配置边界

| 配置 | 职责 |
|---|---|
| `dynamics.yaml` | 动力学与电机参数 |
| `controller.yaml` | 闭环控制参数 |
| `mission.yaml` | 基础任务参数；assessment 入口禁用自动 waypoint |
| `environment.yaml` | 唯一正式工作空间与六障碍物地图 |
| `astar.yaml` | 规划离散化与安全余量 |
| `planned_trajectory.yaml` | 路径简化与连续轨迹参数 |
| `interactive_goal_editor.yaml` | RViz 编辑、预览与预检参数 |
| `interactive_goal_executor.yaml` | 多目标执行与完成门控参数 |

环境节点、预检和执行器必须加载同一份 `environment.yaml`。正式导航参数为
`nominal_speed=0.50 m/s`、`max_reference_speed=0.90 m/s`、
`max_reference_acceleration=0.60 m/s²`。该组合经三次完整地图验证，相比旧保守参数
平均缩短约 13.9%，且未引入碰撞、控制饱和、非有限值或明显安全净空下降。

## 8. 正式实验协议

```text
01 Hover
02 Single Goal
03 Basic Multi-goal Mission
04 Full-map Static Avoidance
05 Multi-goal 3D Navigation
06 Disturbance
   ├── Short Gust
   └── Persistent Release
```

- 03：无障碍基础环境中的四目标顺序任务与终端 yaw。
- 04：目标 `(13.2,5.5,1.5)` 的单目标全地图静态避障。
- 05：在障碍环境中一次提交 P1→P2→P3→P4，验证四段规划、高度变化与各 Pose 的终端 yaw。
- 06：`+X 0.30 N × 2 s` short gust 与 `+X 0.30 N × 10 s` persistent release 两组独立加分实验。

05 的固定目标为 `(13.15,5.80,3.40,0°)`、`(9.70,-1.20,1.20,177°)`、
`(6.30,5.55,2.35,-112°)`、`(0.45,5.70,1.00,-97°)`。原 narrow-corridor 协议已被
该综合场景取代。

正式运行由 `scripts/run_final_assessment.sh` 创建。临时 smoke 输出统一写入
`/tmp/ros2_drone_assessment_smoke/`，不得作为报告证据。完整目录结构、schema、指标
定义与 finalize 规则见 `results/README.md`。

## 9. 最终验证状态

七组 Final 的 analyzer 均通过，人工验收均完成，受保护证据与参数校验完整，根
manifest 全部标记为可用于报告。截图策略为：hover、single、basic multi 与两组
disturbance 可不登记截图；04 已登记 1 张；05 已登记 3 张。

指标语义以正式结果文件为准。导航报告使用第一条有效正式路径段之后的 navigation
tracking max/RMS，不使用包含起飞的 full-mission tracking。扰动报告使用撤力后持续
满足保持条件的 `recovery_confirmed_time_s`；`recovery_time_s` 仅是首次进入门限的
兼容别名。已有 Final 不得重新分析或 finalize。

## 10. 测试策略

```bash
bash scripts/test_fast.sh
bash scripts/test_assessment.sh
bash scripts/test_full.sh
```

- 普通修改：fast；
- 正式入口修改：fast + assessment；
- 阶段收尾：full；
- 纯文档修改：不重跑飞行仿真。

最终完整回归记录：fast 为 30 CTest / 313 内部用例，assessment 为 5 CTest / 16
内部用例，full 为 34 CTest / 326 内部用例；均为 0 errors、0 failures、0 skipped。
自动回归不能替代 RViz 目标/yaw、障碍与轨迹 Marker、绕行效果、扰动箭头和撤力恢复的
人工检查。

## 11. 关键约束与禁止误述

- 不把水平 `P + D + 受限 I + 前馈`、高度/姿态 PD 统称为完整 PID。
- 不声称静态障碍具有接触物理，不把集中等效外力描述成完整风场。
- 不声称实现动态障碍、局部重规划、MPC 或完整姿态规划。
- 不把 smoke/trial、全任务通用指标或旧 narrow-corridor 结果当作正式报告证据。
- 不将 full-mission tracking 与 navigation tracking 混用。
- 不修改或重新生成七组已 finalize 的正式证据；Reviewer 保持为 `Peter`。

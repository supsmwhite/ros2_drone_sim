# AI 上下文：考核收束后的真实结构

本文记录 `refactor/assessment-scope-consolidation` 分支的最终产品边界、依赖关系与
回归策略。项目只公开三个考核入口，使用一张 `environment.yaml` 静态地图；本阶段
不增加功能，也不改变动力学、控制律、规划安全半径、yaw 算法或完成门控。

## 产品边界

公开能力集中为：悬停、单目标、3～4 目标顺序飞行、多障碍物静态避障、明显绕行或
地图可用通道、误差/RPM/轨迹/障碍距离结果，以及独立抗扰演示。导航目标的位置、
高度和 yaw 由用户在 RViz 中选择。

系统边界保持不变：水平位置环为 `P + D + 受限 I + 期望加速度前馈`，高度和姿态
环为 PD；静态障碍物用于规划和碰撞监测，不产生物理接触；外力是质心处集中等效
力。系统不包含动态障碍、局部重规划、完整风场、MPC 或完整姿态规划。

## 三个公开入口

| 入口 | 默认参数 | 实现方式 |
|---|---|---|
| `assessment_basic_sim.launch.py` | `use_rviz:=true` | 薄封装 `mission_sim.launch.py`，强制 `start_with_configured_waypoints:=false`，等待 `goal_cli single` 或 `goal_cli multi` |
| `assessment_navigation_sim.launch.py` | `yaw_mode:=path_tangent`、`use_rviz:=true` | 薄封装 `interactive_goal_navigation_sim.launch.py`，复用预览、预检、3D A*、简化、连续轨迹和执行链 |
| `assessment_disturbance_sim.launch.py` | `profile:=short_gust`、`use_rviz:=true` | 薄封装 `disturbance_visual_demo.launch.py`；另支持 `profile:=persistent_release` |

基础入口不会自动读取 YAML waypoint。`single` 与 `multi` 共用同一个运行时任务接口，
由任务管理节点串行接收，不创建第二套执行器或重名 Marker 发布者。

## Launch 依赖图

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

内部保留的五个 Launch 仅为
`simulation_core.launch.py`、`basic_sim.launch.py`、`mission_sim.launch.py`、
`interactive_goal_navigation_sim.launch.py` 和 `disturbance_visual_demo.launch.py`。

## 保留的运行节点和工具

| 包 | ROS 节点/命令 | 作用 |
|---|---|---|
| `drone_dynamics` | `quadrotor_dynamics_node` | 四电机 RPM 驱动的刚体动力学，可选外力输入 |
| `drone_controller` | `position_controller_node` | 位置、速度、姿态和电机混控闭环 |
| `drone_mission` | `waypoint_manager_node` | 运行时单目标/多目标任务、顺序执行与完成状态 |
| `drone_mission` | `goal_visualizer_node` | 基础任务目标 Marker |
| `drone_planning` | `static_environment_node` | 唯一正式地图的工作空间、原始/膨胀障碍 Marker 与碰撞统计 |
| `drone_planning` | `interactive_goal_editor_node` | RViz 目标编辑、预览、预检和执行服务入口 |
| `drone_planning` | `multi_goal_static_avoidance_node` | 多目标 A*、简化、连续轨迹、yaw 和完成门控 |
| `drone_bringup` | `disturbance_demo_node` | 扰动时序、外力与补偿可视化 |
| `drone_mission` | `goal_cli` | 向基础入口提交 `single` 或 `multi` 任务 |
| `drone_planning` | `map_reachability_check` | 离线检查正式地图可达性 |

`robot_state_publisher` 与可选 `rviz2` 属于公共可视化基础设施。保留的核心库包括
waypoint、目标解析/可视化、分段五次轨迹、静态环境、碰撞检查、A*、路径简化、
连续轨迹构建、yaw 参考、目标完成门控、多目标可视化、失败安全和交互目标编辑。

## 配置边界

正式配置为：

- `dynamics.yaml`：动力学和电机参数；
- `controller.yaml`：闭环控制参数；
- `mission.yaml`：任务管理默认参数，assessment 基础入口禁用其中的自动 waypoint；
- `environment.yaml`：唯一正式工作空间和六障碍物地图；
- `astar.yaml`：正式规划离散化和安全余量；
- `planned_trajectory.yaml`：简化与连续轨迹参数；
- `interactive_goal_editor.yaml`：RViz 编辑、预览和预检参数；
- `interactive_goal_executor.yaml`：正式多目标执行与完成门控参数。

环境、预检和执行器均加载同一个 `environment.yaml`。本轮没有另建未经验证的
“窄通道地图”；明显绕行或通道展示通过正式六障碍物地图上的人工目标选择验收。

## 正式工作流

基础任务：

```text
goal_cli single/multi
→ waypoint_manager_node
→ /drone/goal
→ position_controller_node
→ /drone/motor_rpm_cmd
→ quadrotor_dynamics_node
```

交互导航：

```text
RViz 目标位置与 yaw
→ interactive_goal_editor_node 预览和预检
→ 3D A* → 路径简化 → 安全连续轨迹
→ multi_goal_static_avoidance_node
→ path_tangent yaw → /drone/trajectory_setpoint
→ position_controller_node → quadrotor_dynamics_node
```

抗扰演示：

```text
disturbance_demo_node
→ 目标与外力时序
→ quadrotor_dynamics_node（仅此入口启用 external wrench）
→ position_controller_node
→ 外力/积分补偿 Marker 与恢复状态
```

## 测试收束与断言迁移

底层算法和物理测试全部保留。`drone_bringup` 最终固定为 11 个 CTest 目标：2 个结构/
物理一致性测试、4 个正式入口测试、5 个关键安全与边界测试。

| 原重复覆盖中的有效断言 | 迁入位置 |
|---|---|
| 公共 core 复用、内部 Launch 依赖、无重复节点 | `test_assessment_launch_structure.py` |
| 单目标和多目标正式运行时任务 | `test_assessment_basic_single_e2e.py`、`test_assessment_basic_multi_e2e.py` |
| 目标顺序、READY、raw/simplified/reference 路径安全、净空、终态、碰撞、非有限值、饱和、单发布者 | `test_interactive_goal_navigation_e2e.py` |
| 六个原始/膨胀障碍 Marker、工作空间 Marker 语义 | `test_interactive_goal_navigation_e2e.py` |
| 碰撞边界和工作空间边界 | `test_collision_checker.cpp` |
| `short_gust`、`persistent_release=10 s`、进程集合、外力/Marker/恢复 | `test_assessment_disturbance_launch.py` 与节点级安全测试 |

收束前基线为 45 个 CTest 目标，其中 `drone_bringup` 22 个，Launch/E2E 21 个，
GTest/Pytest 24 个，内部测试用例 361 个。收束后的静态目标为 34 个 CTest，其中
`drone_bringup` 11 个、Launch/E2E 11 个、GTest/Pytest 23 个；最终内部用例数以本轮
唯一一次完整回归实测为 326 个。

## 三档回归

```bash
bash scripts/test_fast.sh
bash scripts/test_assessment.sh
bash scripts/test_full.sh
```

- 普通修改：`test_fast.sh`；
- 正式入口修改：`test_fast.sh` 后再运行 `test_assessment.sh`；
- 阶段收尾或合并 `main` 前：仅在前两档通过后运行 `test_full.sh`；
- 单独文档修改：不重跑完整仿真。

快速档运行全部底层测试和轻量安全测试，不运行三个长时间飞行任务。考核档只运行
三个正式入口、基础入口的 single/multi 两种任务以及预检失败边界。完整档构建并测试
整个工作区。三档通过标准均为 `errors=0`、`failures=0`。

## 已验证结果与人工验收边界

历史量化结果保存在 `results/`，本轮不删除也不改写。yaw 完成门控修复后的证据为
`results/interactive_goal_yaw/full_regression.json` 和
`results/interactive_goal_yaw/path_tangent_e2e.json`：测试提交
`06013d454a1287427b61ce9c52374ff1a03fc3fe`，完整回归
`342 tests, 0 errors, 0 failures, 0 skipped`，三个目标被接受时 yaw 误差约为
`0.005192 rad`、`0.004746 rad`、`0.004973 rad`。

自动回归证明执行链和安全条件，但不能替代以下人工检查：RViz 目标位置/yaw 编辑、
原始与膨胀障碍 Marker、轨迹 Marker、明显绕行或通道视觉效果，以及抗扰箭头和撤力
恢复过程。

本轮分级回归记录：

| 档位 | CTest / 内部用例 | 结果 | 总耗时 |
|---|---:|---|---:|
| `test_fast.sh` | 30 / 313 | `0 errors, 0 failures, 0 skipped` | `24.39 s` |
| `test_assessment.sh` | 5 / 16 | `0 errors, 0 failures, 0 skipped` | `120.90 s` |
| `test_full.sh`（本轮唯一一次） | 34 / 326 | `0 errors, 0 failures, 0 skipped` | `136.20 s` |

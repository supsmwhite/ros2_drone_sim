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

## 最终实验结果与人工验收边界

开发阶段的旧量化结果已从当前收束分支的 `results/` 移除；它们仍由 `main`、历史提交
和 `assessment-feature-complete-v1` 标签完整保留。新 `results/` 只服务最终报告实验，
临时调参与独立开发评测不得写入。标记为 `smoke` 的记录只验证统一记录/分析流程，
不作为最终报告数据。

后续批准的实验按 `01_hover`、`02_single_goal`、`03_multi_goal`、`04_navigation`、
`05_disturbance`、`06_failure_case` 顺序生成。正式交互导航目标点与路线必须由用户最终
选择，工具和文档不得代为决定或预填结果。

统一评测工具对六类实验使用独立停止状态机。基础 multi 等待真实 mission complete，
导航等待 interactive active 与 navigation complete/success，扰动等待外力开始、撤销和
恢复，失败案例等待明确拒绝并完成地面安全观察。`goal_position_error` 表示实际位置到
当前任务目标的距离；`tracking_error` 表示实际位置到 trajectory setpoint 的距离。
`paths.json` 保存各目标的 planned/simplified/reference 唯一路径段，空 Path 只记录清除
事件，不能覆盖历史。

最终指标语义冻结在 schema 3。full-mission tracking 包含起飞；navigation tracking 从
第一条带有效 goal index 的 reference 路径段开始，缺失时依次回退 planned、simplified
和 navigation goal activation，报告优先采用 navigation max/RMS。路径段同时记录
recording 和 mission 时间，任务前 transient-local 路径的 mission 时间为 null。

多目标 activation 是目标正式成为当前目标的时间，非最终 arrival 等于下一目标
activation，最终 arrival 等于 complete，duration 为 arrival 减 activation。扰动反向
超调使用有效阶段平均水平外力方向的有符号位移，仅统计撤力后穿越目标到反方向的距离；
它与无方向峰值水平偏差、沿力方向正向峰值是三个独立指标。

唯一允许提交的 smoke 是 `results/01_hover/smoke`。single、multi、navigation、
disturbance 和 failure_case 的工具验证写入 `/tmp/ros2_drone_assessment_smoke/`，不作为
报告数据。参数表将 ROS 基础/导航完成门控与评测分析阈值分开记录；正式结果仍必须等待
用户批准目标、路线、阈值和报告图表。

自动回归证明执行链和安全条件，但不能替代以下人工检查：RViz 目标位置/yaw 编辑、
原始与膨胀障碍 Marker、轨迹 Marker、明显绕行或通道视觉效果，以及抗扰箭头和撤力
恢复过程。

项目负责人已完成人工 RViz 验收：hover、single goal、multi goal、interactive
navigation、full-map avoidance、failure rejection、short gust 和 persistent
disturbance/release 均通过；单目标 Marker 已确认能够从 `GOAL CURRENT` 切换为
`GOAL DONE`。导航速度与路径净空属于后续 `experiment/navigation-performance` 分支的
性能探索，不是当前稳定基线的阻塞问题。

本轮分级回归记录：

| 档位 | CTest / 内部用例 | 结果 | 总耗时 |
|---|---:|---|---:|
| `test_fast.sh` | 30 / 313 | `0 errors, 0 failures, 0 skipped` | `24.39 s` |
| `test_assessment.sh` | 5 / 16 | `0 errors, 0 failures, 0 skipped` | `120.90 s` |
| `test_full.sh`（本轮唯一一次） | 34 / 326 | `0 errors, 0 failures, 0 skipped` | `136.20 s` |

## 导航速度实验结论（2026-07-20）

`experiment/navigation-performance` 从 `08aa10e`（与 `origin/main`、
`assessment-stable-v1` 相同代码树）开始。实验基础设施提交 `f87fee1` 增加候选参数透传和
`tools/run_navigation_speed_sweep.py`；后续两个小修复分别处理任务前空事件时间和区分
navigation/full-mission 速度峰值。默认 YAML、控制器、A*、地图、安全半径和路径算法均
未修改。所有原始扫描数据只位于 `/tmp/ros2_drone_navigation_performance/`。

baseline 完整地图 `(0,0,1.5) → (13.2,5.5,1.5)` 三次均成功，任务本体时间为
`67.440 / 67.421 / 67.401 s`，平均 `67.421 s`、总体标准差 `0.016 s`。平均 navigation
tracking max/RMS 为 `0.0242 / 0.0081 m`，实际导航峰值 `0.5575 m/s`，最小安全净空
`0.1502 m`。稳定三目标路线 baseline 也成功，任务时间 `54.680 s`。

S1～S4 完整地图快速筛选全部成功且无碰撞、非有限值或饱和；任务时间依次为
`67.001 / 64.625 / 62.904 / 61.808 s`，selected duration scale 依次为
`1.10 / 1.15 / 1.20 / 1.25`，velocity scale 均为 `1.00`。最快 S4 的实际/参考峰值为
`0.706 / 0.695 m/s`，tracking max/RMS 为 `0.0265 / 0.0096 m`，净空 `0.1500 m`，但
相对 baseline 只缩短 `5.612 s`（`8.324%`），没有达到 `10%` 或 `10 s` 收益门槛。
因此未进入候选三次重复，也不修改正式默认参数；最终结论为 **C：放弃本轮提速，保留
稳定 baseline**。自动 duration scaling 是收益被压缩的主要环节，不进入控制器或路径
净空调参阶段。

在保持第一轮结论不变的前提下，最后一轮只验证 `0.50～0.65 m/s²` 参考加速度余量，
用于区分总加速度中的水平/垂直分量；上限严格低于控制器 `0.8 m/s²` 水平能力，不借此
修改控制器、路径或安全边界。

第二轮加速度余量扫描在 `228a7c8` 的加速度分量诊断工具上完成。补跑 baseline 的参考
水平/垂直/总加速度峰值为 `0.3436 / 0.0024 / 0.3436 m/s²`；峰值位于 mission
`45.086 s`、trajectory segment 9、位置 `(10.530,1.258,1.500)`，证实它来自水平拐角，
不是起飞或高度变化。实际中心差分水平/垂直/总峰值为
`0.3845 / 0.1206 / 0.3849 m/s²`。

A1 (`0.45/0.85/0.50`) 完整地图为 `62.041 s`，未达到收益门槛。A2
(`0.50/0.90/0.60`) 和 A3 (`0.55/1.00/0.65`) 均完成完整地图 3/3：A2 时间
mean/min/max/std 为 `58.041/58.021/58.061/0.016 s`，相对第二轮 baseline 改善
`9.360 s / 13.887%`；A3 为 `57.288/57.281/57.301/0.009 s`，改善
`10.114 s / 15.005%`。两者无碰撞、非有限值或饱和，净空约 `0.1502/0.1501 m`。

最终建议采用更稳健的 **A2** 作为候选，但仍等待项目负责人确认且不修改正式 YAML。
A3 虽再快 `0.753 s`，三目标实际水平加速度达到 `0.733 m/s²`、tracking max 增至
`0.0486 m`；A2 保留更明确的 `0.8 m/s²` 控制反馈余量，同时已满足收益门槛。因此本轮
结论为 **A：建议采用 A2 候选**，无需进入路径净空优化；若未来继续追求 A3 以上性能，
应优先研究分段时间分配而不是继续放宽控制或安全边界。

## 导航四层几何诊断（2026-07-20）

诊断分支 `experiment/navigation-clearance-smoothing` 从性能分支 `848cd21` 创建。三组
candidate 记录使用工具提交 `207ae2f` 且 `git_dirty=false`：完整地图 baseline
`(0,0,1.5) → (13.2,5.5,1.5)` 成功、`67.405 s`；同路线 A2 成功、`58.055 s`；A2
三目标 `(3.5,1,2.5) → (5.5,1,4) → (7,5,4)` 成功、`44.688 s`。原始 CSV、JSON 和
PNG 仅位于 `/tmp/ros2_drone_navigation_geometry/`。离线分析工具最终提交为 `6f3f5a4`。

完整地图统一按 `0.02 m` 弧长采样。baseline 的 planned/simplified/reference/actual
最小 safety clearance 分别为 `0.1500 / 0.1500 / 0.1500 / 0.1502 m`；A2 为
`0.1500 / 0.1500 / 0.1500 / 0.1502 m`。A2 最小点分别位于约
`(11.002,4.600,1.500)`、`(10.600,1.718,1.500)`、`(10.600,1.712,1.500)` 和
`(10.600,2.876,1.500)`。因此全局 planned→simplified、simplified→reference、
reference→actual 净空损失均约为零；全图最低净空从 **P（A*）层**已经出现。

但全局最小值会掩盖局部问题。A2 共识别 8 个真实 simplified 拐角。按实际净空和局部
动态，最重要的三个为：

| 拐角位置 / 转角 | planned | simplified | reference | actual | 主要证据 |
|---|---:|---:|---:|---:|---|
| `(10.60,1.35,1.50)` / `45.0°` | 0.1500 | 0.1500 | 0.1500 | 0.1545 | P 层贴墙贯穿到 reference；ref speed `0.076–0.410 m/s` |
| `(3.60,1.85,1.50)` / `55.3°` | 0.3157 | 0.1813 | 0.2115 | 0.2061 | S 层损失 `0.1344 m`；A2 cross-track max `0.0158 m` |
| `(9.10,0.35,1.50)` / `26.6°` | 0.1549 | 0.3034 | 0.2713 | 0.2647 | P 层先贴墙，S 层移开；T/C 再损失 `0.0321/0.0066 m` |

另一个明显的 S 层局部损失位于 `(7.60,-1.15,1.50)`：`0.3508 → 0.1775 m`。
`(9.85,0.60,1.50)` 的短段区域 reference speed 为 `0.097–0.412 m/s`，reference/
actual jerk 峰值为 `1.13/55.65 m/s³`，是最明显的视觉僵硬候选；无任何控制饱和。
三目标路线排除停稳后重新规划接缝后只有 2 个真实拐角，最小四层净空为
`0.2000/0.1813/0.2510/0.2354 m`，未复现完整地图的 `0.150 m` 瓶颈。

baseline 与 A2 的 planned 和 simplified 都不是逐位完全相同，因为规划起点取实际 Odom，
两次起飞稳定高度相差 `1.03 mm`；除第一个点外，其余 planned 和 simplified 点完全相同，
因此离散几何可视为一致。reference 因时间参数改变而发生真实空间变化，双向 Hausdorff
距离 `0.0300 m`、baseline→A2 RMS `0.00688 m`。actual 两次空间曲线的双向 Hausdorff
距离 `0.0314 m`。A2 相比 baseline 的 temporal tracking max/RMS 从
`0.0261/0.00913` 增至 `0.0294/0.01064 m`，spatial cross-track max/RMS 从
`0.01245/0.00408` 增至 `0.01790/0.00579 m`；外切在 baseline 已存在，A2 将其放大。
全局最大绝对 roll/pitch 从 `0.0327/0.0382` 增至 `0.0362/0.0624 rad`，实际最大 yaw
rate 基本不变（`0.7636 → 0.7642 rad/s`），两者均无饱和。

最终分类为 **M（混合问题）**：全局贴墙主要始于 P；S 在若干局部产生显著净空下降和
短尖折线；T 在 `(9.1,0.35)` 附近进一步降低净空，并通过短段速度谷值/jerk 形成僵硬；
C 的局部净空附加损失均小于 `0.01 m`，但 A2 明确增大 tracking/cross-track，因此是外切
放大项而非主贴墙来源。下一步最高优先级应单独研究 S 层的低净空 shortcut/急转弯
waypoint 保留；第二优先级才是 T 层的曲率感知局部 duration scale，直线段保留 A2。
当前证据不支持统一增大障碍膨胀、修改安全半径、调控制增益或继续提高全局速度。本轮未
修改 A*、simplifier、quintic、控制器、动力学、地图、安全参数或正式 YAML，也未采用 A2。

## 连续轨迹局部时间分配实验（2026-07-20）

`experiment/trajectory-curvature-timing` 从
`experiment/navigation-clearance-smoothing@a774283` 创建，未携带失败的
`shortcut_preferred_clearance`、additional-clearance 或 preferred/fallback shortcut
产品逻辑。核心实现提交为 `57b1cf6`，实验工具提交为 `651b812`；正式快速筛选从干净的
`651b812` 运行，原始 CSV/JSON/PNG 只位于
`/tmp/ros2_drone_trajectory_curvature_timing/`。

时间链路审计确认：每段距离基础时长为
`max(segment_length / nominal_speed, min_segment_duration)`；quintic 接收独立的每段
duration。首尾 waypoint velocity 为零，内部 velocity 是相邻两段
`displacement / duration` 的平均再乘现有 velocity scale，因此共享 waypoint 导数同时受
左右 duration 影响。局部时间倍率先作用于距离基础时长，现有 global duration candidate
再统一相乘；搜索顺序仍为 global duration 外层、velocity scale 内层。动态限制失败继续
尝试候选，首次碰撞失败会停止继续增大全局时长并按原逻辑插入 raw path 点重新 refinement。
所以只修改 duration 虽不改 waypoint 坐标，仍可能通过共享导数改变 quintic 段内空间曲线；
本轮保留原碰撞采样和 refinement 正是为此。

新增参数默认值为 `corner_timing_enabled=false`、start/full angle
`25/70 deg`、max duration scale `1.0`，预览、预检和执行均由两个正式 Launch 入口透传。
默认关闭时每个 segment multiplier 严格为 `1.0`，旧 duration 逐项不变。内部点转角使用
world-space 3D 向量的 `acos(clamp(dot(normalize(Pi-Pi-1),
normalize(Pi+1-Pi)),-1,1))`，短段和非有限点拒绝。区间内使用
`u²(3-2u)` smoothstep；每段取左右端点 corner scale 的 `max`，不相乘。

固定 A2 (`0.50/0.90/0.60`) 完整地图快速筛选结果如下。任务时间相对稳定 baseline
`67.42 s` 的改善分别为 `13.884% / 9.701% / 21.982%`：

| 候选 | mission / reference duration (s) | global / local max | ref jerk max / 三个最严重角点平均 | tracking max/RMS (m) | cross-track max/RMS (m) | ref/actual clearance (m) |
|---|---:|---:|---:|---:|---:|---:|
| T0 | 58.059 / 53.385 | 1.10 / 1.000 | 6.665 / 5.386 | 0.0280 / 0.0117 | 0.0183 / 0.00581 | 0.1500 / 0.1502 |
| T10 | 60.880 / 56.221 | 1.10 / 1.0995 | 6.994 / 4.656 | 0.0257 / 0.0109 | 0.0190 / 0.00526 | 0.1500 / 0.1502 |
| T20 | 52.600 / 47.941 | 1.00 / 1.2000 | 7.484 / 4.963 | 0.0364 / 0.0141 | 0.0151 / 0.00527 | 0.1025 / 0.1097 |

T0/T10 的 planned 和 simplified 几何一致；T10 reference Hausdorff 为 `0.0050 m`。
T10 的最严重三个角点 jerk 平均下降 `13.6%`，tracking RMS 和 cross-track RMS 分别改善
`6.8%/9.6%`，但全局 reference jerk 增加 `4.9%`、cross-track max 增加 `4.2%`，且
`60.880 s > 60.68 s`，失去相对稳定 baseline 至少 10% 的时间优势。

T20 使 global scale 从 `1.10` 降到 `1.00`，但同时把 velocity scale 从 `1.00` 改选为
`0.25`，初始 9 点 simplified trajectory 不再触发 T0 的三轮 collision refinement，最终
simplified 点数由 18 变为 9。它相对 T0 的 reference Hausdorff 为 `0.2966 m`，reference/
actual clearance 分别下降约 `0.0475/0.0404 m`，tracking max 增加 `30.2%`，全局
reference jerk 增加 `12.3%`；因此它的时间和 cross-track max 改善不能作为有效收益。
三候选全局 actual jerk max 为 `10.520/10.247/26.734 m/s³`，仅作辅助。

五个重点区域的 reference jerk max（T0/T10/T20）依次为：`(3.60,1.85)`
`6.516/4.966/7.484`，`(7.60,-1.15)` `3.474/3.356/6.059`，`(9.10,0.35)`
`3.917/3.694/0.559`，`(9.85,0.60)` `4.770/4.352/0.540`，`(10.60,1.35)`
`4.871/4.649/1.344 m/s³`。T20 后三处的大幅下降来自 simplified 几何改变，不能归因于
同一路径的局部时间优化。每处的速度、加速度、actual jerk、tracking、cross-track 和
reference/actual clearance 明细保存在 `timing_sweep.json` 的 `local_corners`。

T0/T10/T20 均 navigation success、collision=false、saturation=0、nonfinite=0；最大绝对
roll 为 `0.0362/0.0286/0.0383 rad`，pitch 为 `0.0624/0.0638/0.0591 rad`，yaw rate 为
`0.7644/0.7654/0.7360 rad/s`，实际水平加速度为 `0.6475/0.6700/0.6258 m/s²`。
T20 已满足几何改变和净空退化停止条件，因此按协议未运行 T30、三目标、候选三次重复或
人工 RViz 候选对比；没有合格候选可供人工比较。

最终分类为 **C：局部 duration scale 无效，保留 T0**。T10 只是变慢且未达到动态改善
门槛，T20 的 global-scale 跳变、velocity-scale 改选和 collision refinement 差异使实验
不再保持相同 simplified 几何。当前证据说明仅修改相邻段 duration 不足；若继续研究拐角
速度谷值和恢复突变，应单独进入 waypoint velocity 边界条件研究，但不得把本轮任何候选
设为正式默认值。

本轮 `colcon build --symlink-install` 通过；角度/smoothstep/segment 映射/builder 共 18 个
定向 GTest 全通过；`tools/tests` 为 `81 passed`；`test_fast.sh` 为 322 个内部用例、
0 errors/failures；`test_assessment.sh` 为 16 个内部用例、0 errors/failures。本轮按约定未运行
full，也未修改 A*、simplifier、控制器、地图、正式 YAML、正式 `results/` 或 main。

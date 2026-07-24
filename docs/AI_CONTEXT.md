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

环境节点、预检和执行器必须加载同一份 `environment.yaml`。当前合并候选默认参数为
Candidate H：`nominal_speed=0.70 m/s`、`max_reference_speed=1.28 m/s`、
`max_reference_acceleration=0.88 m/s²`、`max_horizontal_acceleration=1.12 m/s²`、
`min_segment_duration=2.0 s`、`max_tilt_angle=0.15 rad`。参考加速度低于控制限幅，
控制限幅低于 `g*tan(0.15)=1.482 m/s²`；控制 P/D/I 参数未改。

`turn_aware_speed_limiting` 默认启用。进入中间目标的转角小于 `30°`、
位于 `[30°,60°)`、不小于 `60°` 时，分别使用 `1.0/0.9/0.8`，并同步缩放
`nominal_speed`、`max_reference_speed` 和 `max_reference_acceleration`。预检与实际
规划调用同一策略；单目标任务和最终目标段保持 `1.0`。复杂轨迹仍从严格升序的
`duration_scale_candidates` 中选择第一个满足安全和动态约束的比例。

同代码固定四目标配对基线 `0.50/0.90/0.60/0.80` 的总任务/导航时间为
`130.789/127.149 s`；H 加转弯限速的两次平均为 `112.695/109.055 s`，导航时间缩短
`14.23%`。H 的平均跟踪最大/p95/RMS 为 `0.03870/0.02302/0.01256 m`，平均最小净空
`0.17498 m`，RPM 使用率 `57.45%`，超过 `5 cm` 的样本、碰撞、饱和和非有限值均为
零。这些仍是临时 Trial，不能作为 finalized 正式证据；合并前人工 RViz 验收待开发者
执行。

收束后又以公开默认入口、无速度覆盖参数运行一次固定四目标 Trial，实际读取到 H 和
`turn_aware_speed_limiting=true`；总任务/导航时间 `112.704/109.064 s`，
tracking 最大/p95/RMS `0.04004/0.02299/0.01237 m`，最小净空 `0.17495 m`，
碰撞、饱和、非有限值均为零。该结果同样只位于 `/tmp`。

旧的 `0.50/0.90/0.60/0.80` 仍是七组 finalized 结果的参数快照，不得回写或重新
分析。详细历史候选、门槛和复现方法见 `docs/navigation_speed_validation.md`。

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
bash scripts/test_navigation_speed_smoke.sh all --candidate local_check
bash scripts/test_navigation_speed_smoke.sh formal_four_goal \
  --candidate h_default_release_candidate
```

- 普通修改：fast；
- 正式入口修改：fast + assessment；
- 阶段收尾：full；
- 导航参数或转弯策略修改：三场景 smoke；发布候选默认值再运行显式四目标 Trial；
- 纯文档修改：不重跑飞行仿真。

当前合并候选回归记录：fast 为 31 CTest / 321 内部用例（另有 tools Python
`151 passed`），assessment 为 5 CTest / 16 内部用例，full 为 35 CTest / 334
内部用例；均为 0 errors、0 failures、0 skipped。
自动回归不能替代 RViz 目标/yaw、障碍与轨迹 Marker、绕行效果、扰动箭头和撤力恢复的
人工检查。

性能回归应先检查每段日志中的 `turn_speed_scale` 与 `duration_scale`，再检查 tracking
p95/RMS、连续超过 `5 cm` 的时间、planned/simplified/reference/actual 四类路径碰撞
以及控制饱和。smoke 和 Trial 只写入 `/tmp/ros2_drone_assessment_smoke/`。

## 11. 关键约束与禁止误述

- 不把水平 `P + D + 受限 I + 前馈`、高度/姿态 PD 统称为完整 PID。
- 不声称静态障碍具有接触物理，不把集中等效外力描述成完整风场。
- 不声称实现动态障碍、局部重规划、MPC 或完整姿态规划。
- 不把 smoke/trial、全任务通用指标或旧 narrow-corridor 结果当作正式报告证据。
- 不将 full-mission tracking 与 navigation tracking 混用。
- 不修改或重新生成七组已 finalize 的正式证据；Reviewer 保持为 `Peter`。
- 不把当前性能 Trial 的 `112.695 s` 误述为新的 finalized 正式结果或系统最大速度。
- 不声称合并候选已完成人工 RViz 验收；该步骤仍由开发者执行。

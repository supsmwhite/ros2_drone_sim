# ROS2 四旋翼无人机仿真系统

这是一个面向课程考核的 ROS2 Humble 四旋翼闭环仿真项目。系统以四电机目标
RPM 驱动刚体动力学，提供位置控制、多目标任务、三维静态避障、RViz 交互导航、
轨迹/误差/RPM/障碍物距离结果，以及独立抗扰演示。

当前 `main` 是功能完整冻结基线；考核入口收束在独立分支完成，不删除历史实现，
也不增加与考核无关的功能。

## 构建

```bash
cd ~/ros2_drone_sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

不同仿真 Launch 不要同时运行在同一个 ROS Domain。

## 三个正式考核入口

### 1. 基础实验

```bash
ros2 launch drone_bringup assessment_basic_sim.launch.py
```

Launch 启动动力学、控制器、模型、RViz、运行时任务接口和目标 Marker，但不会自动
执行旧 YAML 任务。另开终端并加载工作区后，按考核项目提交任务：

```bash
ros2 run drone_mission goal_cli single 0 0 1.5 yaw=0
```

```bash
ros2 run drone_mission goal_cli single 2 1 1.5 yaw=0
```

```bash
ros2 run drone_mission goal_cli multi \
  0 0 1.5 yaw=0 \
  2 0 1.5 yaw=90 \
  2 2 1.5 yaw=180 \
  0 2 1.5 yaw=-90
```

纯数字 yaw 仍按弧度解析，`yaw=<degrees>` 按角度解析。任务执行中不允许抢占；请在
前一任务完成后再提交下一任务。

### 2. 规划避障

```bash
ros2 launch drone_bringup assessment_navigation_sim.launch.py
```

默认 `yaw_mode:=path_tangent`。在 RViz 工具栏选择 `Interact`，拖动候选目标的位置、
高度和 yaw；依次使用 `Add Goal`、`Validate & Preview`，确认状态为 READY 且蓝色
预览完整后选择 `Execute Validated Mission`。执行链为：

```text
RViz 目标位置与 yaw
→ 预览和预检
→ 3D A*
→ 路径简化
→ 安全连续轨迹
→ path_tangent yaw
→ 多目标执行
```

项目只保留一张正式静态障碍地图 `environment.yaml`。用户在 RViz 中自行添加多个
目标点，并可为每个目标设置 yaw。同一个导航入口可根据目标选择展示多障碍物静态
避障、明显绕行、地图可用区域中的通道穿越以及多目标顺序导航。公开参数为
`yaw_mode:=path_tangent|fixed`（默认 `path_tangent`）和 `use_rviz:=true|false`
（默认 `true`）。

### 3. 抗扰加分演示

```bash
ros2 launch drone_bringup assessment_disturbance_sim.launch.py \
  profile:=short_gust
```

可选 `profile:=persistent_release`。短时配置施加 `+X 0.30 N × 2 s`；持续配置施加
`+X 0.30 N × 10 s` 后撤力恢复。红色箭头表示质心处集中等效外力，蓝色箭头表示
水平积分补偿；它们不代表完整风场。

## 考核要求—运行方式—验收指标

| 考核要求 | 运行方式 | 主要验收指标 |
|---|---|---|
| 悬停 `(0,0,1.5)` | 基础入口 + `goal_cli single 0 0 1.5 yaw=0` | 位置误差收敛、姿态稳定、RPM 有限且无持续饱和 |
| 单目标 `(2,1,1.5)` | 基础入口 + `goal_cli single 2 1 1.5 yaw=0` | 到达并稳定保持，轨迹与目标 Marker 正确 |
| 3～4 目标顺序飞行 | 基础入口 + `goal_cli multi ...` | 严格按序访问，每点停稳后切换并最终完成 |
| 多障碍物静态避障 | 正式导航入口，在 RViz 中自行选取多个目标 | 预检成功、无碰撞、障碍物净空为正、任务完成 |
| 狭窄通道或明显绕行 | 同一正式导航入口，选择能形成明显绕行的目标点 | 规划路径与直连路径有清晰差异，并在地图可用区域内保持安全净空 |
| 位置误差、RPM、轨迹和障碍距离 | RViz、控制诊断、`results/` 正式图表/指标 | 无非有限值；误差、RPM、轨迹、最小净空可追溯 |
| 独立抗扰加分 | 抗扰入口，两个 `profile` | 扰动阶段正确、补偿方向正确、撤力后稳定恢复 |

## 已验证结果

正式三目标静态避障任务依序访问 `(13.2,5.5,1.5)`、`(7.0,5.0,4.0)`、
`(0.8,0.7,2.0)`：

| 指标 | 结果 |
|---|---:|
| Launch 到任务完成 | `139.203712 s` |
| 最大跟踪误差 | `0.028843 m` |
| 对基础膨胀障碍物最小净空 | `0.094310 m` |
| 最终位置误差 | `0.001536 m` |
| 最终速度 | `0.004355 m/s` |
| 碰撞 / 控制器饱和 | `无 / 0` |

交互 `path_tangent` 三目标 `90°、180°、-90°` 场景在提交
`06013d454a1287427b61ce9c52374ff1a03fc3fe` 上依序完成。三个目标被接受时的 yaw
误差约为 `0.005192 rad`、`0.004746 rad`、`0.004973 rad`；完成门控同时检查位置、
线速度、最短角 yaw 误差和角速度，并保持完整规定时间。对应完整回归为
`342 tests, 0 errors, 0 failures, 0 skipped`。数据源：
`results/interactive_goal_yaw/path_tangent_e2e.json` 与 `full_regression.json`。

持续 `0.30 N` 外力下，水平积分关闭的 PD 对照末 3 秒平均误差为 `0.749340 m`，
当前水平 `PD+I+FF` 基线为 `0.081989 m`。三次独立撤力实验恢复时间为
`4.600580–4.601050 s`，均无控制器饱和。正式图表和原始数据位于
`results/horizontal_integral_upgrade/selected/`。

三个正式入口已有真实自动闭环验证。唯一正式地图上的交互导航代表性三目标任务
完成时间为 `54.663 s`，最大跟踪误差 `0.019330 m`，最小障碍净空
`0.239986 m`，最终误差 `0.000898 m`，无碰撞、非有限值或控制器饱和。自动结果
证明统一导航链可安全执行任务，但不替代对目标选择、Marker、明显绕行和可用通道的
RViz 人工视觉验收。

## 系统边界

水平位置控制为 `P + D + 受限 I + 期望加速度前馈`，高度和姿态环为 PD，不能把
整套飞控称作完整 PID。静态障碍只用于规划和碰撞监测，不产生物理接触反作用；外力
是作用于质心的集中等效力。系统不包含动态障碍、局部重规划、完整风场、MPC 或完整
姿态规划。

完整参数、Topic、节点关系、结构审计和结果来源见 `docs/AI_CONTEXT.md`。

## 正式入口的内部依赖

以下 Launch 只作为三个正式入口的实现组件或内部诊断入口，不与公开入口并列：

| Launch | 内部用途 |
|---|---|
| `simulation_core.launch.py` | 公共动力学、控制器、模型和 RViz 组件 |
| `basic_sim.launch.py` | 仅等待 Pose 目标的基础仿真 |
| `mission_sim.launch.py` | 离散 waypoint/YAML/Service 任务 |
| `interactive_goal_navigation_sim.launch.py` | 正式导航入口复用的内部完整链 |
| `disturbance_visual_demo.launch.py` | 正式抗扰入口复用的内部演示链 |

## 测试

普通代码修改运行快速档：

```bash
bash scripts/test_fast.sh
```

正式入口修改还需运行考核档：

```bash
bash scripts/test_assessment.sh
```

阶段收尾或合并 `main` 前运行完整档：

```bash
bash scripts/test_full.sh
```

执行纪律为“普通修改 → fast”“正式入口修改 → fast + assessment”“阶段收尾或合并
`main` 前 → full”；仅修改文档不重跑完整仿真。通过标准为 `failures=0`、
`errors=0`。人工视觉验收与自动回归分开记录，不能用测试结果替代 RViz 操作、
Marker、明显绕行和扰动箭头的人工确认。

## References and acknowledgments

任务指定参考项目为 [pengyu_sim](https://gitee.com/potato77/pengyu_sim) 和
[MARSIM](https://github.com/hku-mars/MARSIM)。它们仅用于理解系统组织思路；本仓库
不是二者的移植版本，也未复制或改编其代码、模型、地图、配置或图片。

## License

本项目采用 Apache License 2.0，完整条款见根目录 `LICENSE`。

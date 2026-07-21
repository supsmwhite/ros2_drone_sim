# ROS2 四旋翼无人机仿真系统

## 项目简介

本项目是面向课程考核与实验复现的 ROS2 Humble 四旋翼仿真系统。系统以四电机目标
RPM 驱动刚体动力学，实现位置、速度与姿态闭环控制，支持单目标和多目标任务、三维
静态避障、RViz 交互导航，以及独立外力抗扰演示，并提供完整的结果记录、自动分析与
证据校验流程。

## 核心能力

| 能力 | 实现范围 |
|---|---|
| 动力学与电机响应 | 四旋翼刚体动力学、四电机目标 RPM 输入 |
| 闭环飞行控制 | 位置、速度与姿态闭环控制 |
| 基础任务 | 单目标与多目标顺序飞行 |
| 三维路径规划 | 三维 A*、路径简化与安全连续轨迹 |
| 航向参考 | 导航阶段 `path_tangent` yaw 与目标终端 yaw |
| 复杂导航 | 障碍环境中的多目标三维避障与高度变化 |
| 安全监测 | 静态碰撞检测与安全净空统计 |
| 抗扰控制 | 外力扰动与受限积分补偿 |
| 实验工作流 | 统一记录、分析、人工验收与证据校验 |

## 系统架构

```text
任务层 / 交互目标
        ↓
任务管理 / 预检 / A*
        ↓
路径简化 / 连续轨迹 / yaw 参考
        ↓
位置与姿态控制器
        ↓
电机 RPM 命令
        ↓
四旋翼刚体动力学
        ↓
Odometry / IMU / RViz / Results
```

| ROS 包 | 主要职责 |
|---|---|
| `drone_msgs` | 自定义消息与任务执行服务接口 |
| `drone_dynamics` | 四旋翼刚体动力学、电机响应与外力输入 |
| `drone_controller` | 位置、速度、姿态控制与电机混控 |
| `drone_mission` | 单目标/多目标任务管理与基础目标可视化 |
| `drone_planning` | 静态环境、三维规划、轨迹生成、交互目标与碰撞监测 |
| `drone_bringup` | 正式 Launch 入口、配置装配与抗扰演示 |

## 正式考核场景

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

| 场景 | 验证目的 |
|---|---|
| 01 Hover | 定点悬停的位置误差、速度、姿态稳定性与电机状态 |
| 02 Single Goal | 无障碍基础环境中的单目标到达、稳定保持与 yaw 控制 |
| 03 Basic Multi-goal Mission | 无障碍基础环境中的四目标顺序任务与终端 yaw |
| 04 Full-map Static Avoidance | 单目标全地图静态避障、轨迹跟踪、碰撞与安全净空 |
| 05 Multi-goal 3D Navigation | 障碍环境中的四目标顺序任务、四段规划、高度变化与终端 yaw |
| 06 Short Gust | 短时外力下的瞬态抑制与撤力恢复 |
| 06 Persistent Release | 持续外力下的积分补偿与撤力恢复 |

原单目标 narrow-corridor 协议已由更综合的四目标三维导航场景取代。

## 最终实验结果

以下数值来自各正式运行的 `summary.json` 和抗扰 `report_metrics.json`。

### 基础任务

| 场景 | 最终结果 |
|---|---|
| Hover | 最终位置误差 `0.00033 m`；最终速度 `0.00044 m/s`；姿态稳定、无发散；饱和样本 `0`，结束时未饱和 |
| Single Goal | 最终位置误差 `0.01121 m`；到达时间 `7.19 s`；最终 yaw 误差 `0.00000061 rad`；结束时未饱和（瞬态饱和样本 `21`） |
| Basic Multi-goal | 完成 `4/4` 目标；访问顺序 `[0,1,2,3]`；总任务时间 `33.22 s`；最终误差 `0.00695 m` |

### 静态避障

| 指标 | 结果 |
|---|---:|
| 最终位置误差 | `0.00386 m` |
| 导航跟踪最大误差 | `0.02342 m` |
| 导航跟踪 RMS | `0.00965 m` |
| 最小安全净空 | `0.15018 m` |
| 实际/参考路径比 | `1.0766` |
| 碰撞 | 无 |
| 饱和样本 | `0` |

### 四目标三维导航

| 指标 | 结果 |
|---|---:|
| 目标完成 | `4/4` |
| 目标顺序 | `[0,1,2,3]` |
| 四段规划 | 完整 |
| 总任务时间 | `133.87 s` |
| 实际总路径 | `51.13 m` |
| 导航跟踪最大误差 | `0.03263 m` |
| 导航跟踪 RMS | `0.01039 m` |
| 最小安全净空 | `0.18152 m` |
| 最终位置误差 | `0.00666 m` |
| 最终速度 | `0.00019 m/s` |
| 碰撞 | 无 |
| 饱和样本 | `0` |

### 抗扰

| 指标 | Short Gust | Persistent Release |
|---|---:|---:|
| 外力 | `0.30 N` | `0.30 N` |
| 持续时间 | `2.00 s` | `10.04 s` |
| 峰值水平偏差 | `0.3321 m` | `0.4850 m` |
| 撤力后正式确认恢复时间 | `7.29 s` | `1.005 s` |
| 最终误差 | `0.0162 m` | `0.0598 m` |
| 饱和样本 | `0` | `0` |

恢复时间均从外力撤除时刻开始计时。Persistent Release 的约 `1.005 s` 表示撤力时系统
已经进入位置与速度恢复门限，随后持续满足约 `1 s` 的保持条件，Recorder 才正式确认
恢复。报告正文继续优先采用 `recovery_confirmed_time_s`。

七组正式结果均已完成自动分析、人工验收和证据 finalize，`results/manifest.json` 中
均为 `report_eligible=true`，Reviewer 为 `Peter`。

## 构建与运行

```bash
cd ~/ros2_drone_sim
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

已安装全部依赖时，rosdep 会直接跳过已满足的包。

不同仿真 Launch 不应同时运行在同一个 ROS Domain。

### 基础任务

```bash
ros2 launch drone_bringup assessment_basic_sim.launch.py
```

另开已加载工作区的终端提交目标：

```bash
ros2 run drone_mission goal_cli single 0 0 1.5 yaw=0
ros2 run drone_mission goal_cli single 2 1 1.5 yaw=0
ros2 run drone_mission goal_cli multi \
  3 0 1.5 yaw=0 \
  3 3 1.5 yaw=90 \
  0 3 1.5 yaw=180 \
  0 0 1.5 yaw=-90
```

`yaw=<degrees>` 按角度解析；纯数字 yaw 按弧度解析。任务串行执行，请在前一任务完成后
再提交下一任务。

### 静态避障与三维导航

```bash
ros2 launch drone_bringup assessment_navigation_sim.launch.py
```

RViz 可用于添加、预检、预览和执行交互目标。单目标静态避障也可通过 Service 提交：

```bash
ros2 service call /drone/interactive_goals/execute \
  drone_msgs/srv/ExecuteGoalSequence \
  "{goals: {header: {frame_id: map}, poses: [{position: {x: 13.2, y: 5.5, z: 1.5}, orientation: {w: 1.0}}]}, draft_revision: 1}"
```

四目标正式 Service 请求及完整协议见 `scripts/run_final_assessment.sh` 和
`results/README.md`。

### 抗扰演示

```bash
ros2 launch drone_bringup assessment_disturbance_sim.launch.py \
  profile:=short_gust
```

持续外力场景使用 `profile:=persistent_release`。

正式实验协议、Recorder、Analyzer、manifest、指标语义和证据校验规则见
`results/README.md`。

## 测试

```bash
bash scripts/test_fast.sh
bash scripts/test_assessment.sh
bash scripts/test_full.sh
```

普通代码修改运行 fast；正式入口修改追加 assessment；阶段收尾运行 full。人工 RViz
验收与自动回归分别记录，自动测试不能替代目标编辑、Marker、绕行轨迹和扰动箭头检查。

## 系统边界

水平位置控制为 `P + D + 受限 I + 期望加速度前馈`，高度和姿态环为 PD，因此不能把
整套飞控称作完整 PID。静态障碍用于规划和碰撞监测，不产生物理接触反作用；外力是
作用于质心的集中等效力。系统不包含动态障碍、局部重规划、完整风场、MPC 或完整姿态
规划。更完整的节点、数据流、配置与测试上下文见 `docs/AI_CONTEXT.md`。

## References and acknowledgments

任务指定参考项目为 [pengyu_sim](https://gitee.com/potato77/pengyu_sim) 和
[MARSIM](https://github.com/hku-mars/MARSIM)。它们仅用于理解系统组织思路；本仓库
不是二者的移植版本，也未复制或改编其代码、模型、地图、配置或图片。

## License

本项目采用 Apache License 2.0，完整条款见根目录 `LICENSE`。

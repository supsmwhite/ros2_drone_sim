# AI_CONTEXT

本文只保存当前有效的工程事实、接口、约束和验收基线，供新的 AI 在较短上下文内接手开发。历史开发过程不在此累积。

## 1. 项目定位

本项目是基于 Ubuntu 22.04 和 ROS2 Humble 的小型四旋翼无人机闭环仿真系统。动力学与控制器使用 C++17 和 Eigen3，RViz2 负责显示，colcon + ament_cmake 负责构建。

项目采用“纯算法类与 ROS2 节点分离”的结构。动力学、三维控制、顺序多目标点、C² 连续轨迹、静态三维 AABB 环境、统一碰撞查询、三维 26 邻域 A*、确定性视线简化、安全规划轨迹生成和静态避障执行均已实现，并有可重复的 ROS2 集成回归。

## 2. 当前阶段和下一任务

### 当前阶段

动力学、三维控制、顺序多目标点任务、连续轨迹跟踪、静态环境碰撞检查、三维栅格 A*、路径简化、规划轨迹验证和静态避障执行均已完成。`planned_trajectory_node` 默认只发布可视化与验证结果；静态避障 Launch 显式启用执行，并独占发布规划参考和轨迹 setpoint。

当前接入代码：

- 使用 `/drone/goal` 的 x、y、z 和 yaw；
- 将 Odom 的完整机体系线速度旋转到世界系后使用；
- 由水平控制器生成期望 body-to-world 姿态，再复用现有高度、姿态和 Mixer 链路；
- 原地起飞、悬停和 yaw 的既有能力没有回归；原地 `1.5 m`、`(0.5,0,1.5)` 和 `(2,1,1.5)` 均已再次实跑通过。
- 多目标任务严格按索引顺序切换，不跳点；最终点验收后发布完成状态并持续保持最终目标。
- 连续轨迹节点先在 P0 连续稳定 `1.0 s`，再按 `steady_clock` 推进四段各 `6.0 s` 的五次轨迹；Odom 异常时暂停轨迹时间，完成后持续发布最终零速度、零加速度参考。
- 控制器的 `trajectory` 模式使用期望速度和三维加速度前馈；消息无效或超时 `0.20 s` 时发布零 RPM，不回退到 Pose 目标。
- 静态环境用 `0.25 m` 安全球保守近似无人机：障碍物向外膨胀、工作空间向内收缩，点和线段触碰安全边界均算碰撞。
- 默认演示 workspace 为 `[-1,9] × [-2.5,7.5] × [-0.5,5] m`，包含中心为 `(2.5,1.0,1.5)` 和 `(6.0,4.0,1.5)`、尺寸均为 `(0.8,3.0,3.0) m` 的两个错开墙体；安全半径保持 `0.25 m`。
- 静态避障场景契约是 start `(0,0,1.5)`、goal `(8,5,1.5)`、栅格 `0.25 m`：起终点安全、直线受阻；真实闭环执行已通过 Domain 99 验收。
- `/drone/environment/in_collision` 只在 Odom 有限且未超时时发布；它是几何状态报告，不会改变 RPM 或模拟物理撞击。
- A* 使用安全 workspace 最小角为栅格原点和 `0.25 m` 分辨率，搜索 26 邻域；欧氏移动代价与欧氏启发函数一致，每个候选点及相邻线段均由 `CollisionChecker` 验证，避免对角切过障碍物边角。
- `0.25 m` 是静态环境节点和 RViz 透明区域使用的基础无人机安全半径；规划节点额外读取 `planning_margin=0.10 m`，以 `0.35 m` 有效规划半径为后续轨迹跟踪与执行预留净空。零裕量合法，负数和非有限裕量在节点构造阶段拒绝。
- 默认 `(0,0,1.5) → (8,5,1.5)` 原始规划结果通过 `/drone/planned_path` transient-local 发布；`planned_trajectory_node` 以相同 `0.35 m` 有效规划半径生成简化路径和连续参考。默认 `execution_enabled=false` 不发布 setpoint；启用后先准备起点，再通过 `/drone/trajectory_setpoint` 驱动真实闭环，Odom 无效或超时时暂停轨迹时钟，完成后保持终点。

完整动态闭环测试复现了旧 `Kd=0.20` 的自激，并证明 `Kd=0.35` 在正负 roll/pitch 固定小倾角和水平姿态下均有稳定裕度。修正后 `(2,1,1.5)` 连续 27 秒无姿态发散、RPM 边界值或日志饱和，十几秒后的自激问题已解决。

### 下一任务

静态避障执行阶段已经完成。后续若开展局部规划、动态障碍或在线重规划，应作为独立阶段，不改变当前静态全局轨迹执行基线。

## 3. 当前架构与数据流

### 当前接入链路

```text
mission.yaml → waypoint_manager_node ← /drone/odom
  ├─→ /drone/mission/current_waypoint_index
  ├─→ /drone/mission/complete
  └─→ /drone/goal (PoseStamped；使用 x、y、z、yaw)
  ↓
position_controller_node
  ↓
PositionController
  ├─→ HorizontalPositionController → 期望水平加速度和姿态
  └─→ HoverController → AltitudeController → AttitudeController → MotorMixer
  ↓
/drone/motor_rpm_cmd (MotorRPM)
  ↓
quadrotor_dynamics_node → QuadrotorModel
  ↓
/drone/odom、/drone/imu、/drone/path、map -> base_link
  ├─→ position_controller_node 状态反馈
  └─→ robot_state_publisher + RViz2 可视化
```

连续轨迹链路：

```text
trajectory.yaml → trajectory_mission_node ← /drone/odom
  ├─→ /drone/trajectory_setpoint (TrajectorySetpoint)
  ├─→ /drone/trajectory/current_segment
  ├─→ /drone/trajectory/complete
  └─→ /drone/reference_path
  ↓
position_controller_node (setpoint_source=trajectory)
  ↓
同一 PositionController → HoverController → Mixer → 动力学闭环
```

规划轨迹显示与可选执行链路：

```text
/drone/planned_path（A* 原始栅格路径）
  ↓ PathSimplifier（确定性最远可见点贪心）
/drone/simplified_path（0.35 m 模型验证的折线）
  ↓ PlannedTrajectoryBuilder + PiecewiseQuinticTrajectory
  ↓ velocity scale 候选、动态约束和 0.02 s 密集碰撞验证
/drone/reference_path（经过验证的连续参考轨迹）
  ↓ execution_enabled=true：起点准备 + steady_clock 采样
/drone/trajectory_setpoint → PositionController → Mixer → 动力学
```

默认 `execution_enabled=false` 时只显示和报告结果；`static_avoidance_sim.launch.py` 显式启用执行。

静态环境监测链路：

```text
environment.yaml → static_environment_node
  ├─→ /drone/environment/markers (transient-local MarkerArray)
  └─← /drone/odom → /drone/environment/in_collision

StaticEnvironment → CollisionChecker
  ├─→ 收缩后的安全工作空间
  ├─→ 膨胀后的静态 AABB
  └─→ point_in_collision / segment_in_collision
```

`HoverController` 是不依赖 ROS2 的组合层，按高度控制器、姿态控制器、Mixer 顺序调用；任何一级无效时最终返回四电机零 RPM，各级饱和状态向上传递。

`PositionController` 与其子控制器均不依赖 ROS2。节点只做消息检查、目标 yaw 提取、Odom 机体系线速度到世界系的完整旋转、输入组装和 RPM 发布，不复制水平 PD、姿态构造或 Mixer 公式。

`WaypointManager` 同样不依赖 ROS2。位置误差、世界系线速度模长、最短 yaw 误差和机体系角速度模长必须分别严格小于配置阈值，并连续保持 `hold_duration`，才接受当前点。离开验收区域会清零连续计时；非法配置直接拒绝，非有限状态不会推进任务。

### TF 所有权

- `map -> base_link`：只由 `quadrotor_dynamics_node` 动态发布；
- `base_link ->` 机臂、电机、旋翼和机头标记：由 robot_state_publisher 根据固定关节发布；
- robot_state_publisher 不发布 `map -> base_link`，避免 TF 冲突。

### 最终目标链路

```text
三维目标/多目标点
→ 地图与规划
→ 安全轨迹或 Waypoint
→ x/y/z 位置与姿态控制
→ Mixer/RPM
→ 动力学
→ 状态反馈、可视化与评测
```

## 4. 已完成模块汇总

### drone_msgs

- `MotorRPM.msg` 已生成并使用；
- `TrajectorySetpoint.msg` 包含 `Header`、世界系 position/velocity/acceleration 和 yaw；
- 四个字段固定对应 M1～M4，字段名同时包含位置和旋转方向。

### drone_dynamics

- `QuadrotorModel` 保存世界系位置/速度、body-to-world 姿态四元数、机体系角速度和四电机实际转速；
- 已实现 RPM 限幅与单位转换、电机一阶响应、推力、反扭矩、X 型三轴力矩、刚体平动/转动、四元数积分与归一化；
- 已实现可配置的简化水平地面；核心模型默认关闭，正常 Launch 默认开启；
- 地面静止时世界系实际加速度为 0，理想 IMU 比力约为机体系 `+g`；
- 动力学节点发布 Odom、IMU、Path 和 TF；
- 已实现 MotorRPM 命令 watchdog。超时只把目标 RPM 设为零，实际电机转速仍按一阶模型衰减。

### drone_controller

- `MotorMixer`：总推力和机体系三轴力矩到 `[M1,M2,M3,M4]` RPM；包含输入、中间溢出、逐电机限幅和安全返回；
- `AttitudeController`：body-to-world 四元数误差和机体系角速度阻尼，输出 roll/pitch/yaw 力矩；包含最短路径和精确 180° 的确定性符号规则；
- `AltitudeController`：世界系 z/vz PD、加速度前馈、重力补偿、倾斜补偿和安全限幅；
- `HorizontalPositionController`：世界系 x/y 位置与速度 PD 加期望加速度前馈，向量模长/最大倾角联合限幅，并用期望推力方向和 yaw 航向参考构造 body-to-world 期望姿态；
- `HoverController`：组合高度、姿态和 Mixer；
- `PositionController`：组合水平位置控制和 `HoverController`，传播中间输出及各级 valid/saturated 状态，任一级无效时安全返回零 RPM；
- `position_controller_node` 支持互斥的 `pose_goal` 与 `trajectory` 来源；前者保持原有零期望速度/加速度语义，后者订阅完整轨迹参考并实施独立超时保护。

### drone_mission

- `WaypointManager`：保存有序 `(x,y,z,yaw)` 列表，以严格阈值和连续稳定时间逐点验收；
- `waypoint_manager_node`：订阅 `/drone/odom`，持续发布 `/drone/goal`、当前索引和完成状态；
- Odom 超时或消息无效时重置当前 waypoint 的连续验收计时并停止推进，但继续发布当前目标和状态；最终点完成后继续保持最终目标；
- 第一版为 stop-settle-switch，不生成平滑轨迹，也不修改底层控制器。
- `PiecewiseQuinticTrajectory`：纯 C++ 分段五次轨迹，位置/速度/加速度 C² 连续；中间速度取相邻割线速度平均并乘可配置的 `[0,1]` 比例，默认 `1.0` 与旧行为完全一致，`0.0` 时各段沿安全直线且在 waypoint 停速；各点加速度为零，yaw 仍按最短角确定性解包再插值；
- `trajectory_mission_node`：发布连续 setpoint、当前段、完成状态和瞬态本地参考 Path；Odom 超时或无效时暂停稳态时间进度。

### drone_planning

- `AxisAlignedBox`：`map/ENU` 下的三维 `min_corner/max_corner`；所有轴必须有限且严格 `min < max`；
- `StaticEnvironment`：保存一个有限工作空间和任意数量静态 AABB，构造时拒绝非有限或非正尺寸几何；
- `CollisionChecker`：保存环境和有限非负安全半径，预计算收缩工作空间与膨胀障碍物；点查询采用闭障碍物/开安全工作空间语义，线段查询采用闭区间三维 slab 算法；
- 非有限点或线段按碰撞处理；相切、角点、障碍物边界和收缩工作空间边界均算碰撞；零长度线段退化为点查询；
- `static_environment_node`：发布环境 MarkerArray，订阅 Odom 并报告几何碰撞状态；Odom 非有限、缺失或超时期间不发布“安全”结果。
- `PathSimplifier`：先严格验证原始点和相邻边，再从终点向前寻找锚点的最远可见后继；精确保留首尾点、保持顺序且结果完全确定。
- `PlannedTrajectoryBuilder`：简化原始路径，按 `max(length/nominal_speed, min_segment_duration)` 分配时间，依次尝试 `[1.0,0.75,0.5,0.25,0.0]`，只接受有限且满足速度、加速度、采样点和采样连线碰撞约束的轨迹。
- `planned_trajectory_node`：首次收到合法 `/drone/planned_path` 后生成一次并保存轨迹、总时长、简化路径、velocity scale 和执行状态；默认只发布显示与指标，启用执行后发布 `/drone/trajectory_setpoint`、段索引和完成状态。

### drone_bringup 与可视化

- `basic_sim.launch.py` 加载动力学和控制器参数并启动两个节点、robot_state_publisher 和可选 RViz2；
- `mission_sim.launch.py` 复用基础仿真并加载 `mission.yaml` 启动任务节点；
- `trajectory_sim.launch.py` 复用基础仿真、令控制器使用轨迹来源，并加载 `trajectory.yaml` 启动轨迹节点；
- `environment_sim.launch.py` 复用基础仿真并加载 `environment.yaml` 启动静态环境节点，不启动 waypoint 或轨迹任务；
- `planning_sim.launch.py` 在环境仿真上启动一次性 A*；`planned_trajectory_sim.launch.py` 再启动规划轨迹节点，并同时显示原始、简化和连续三种 Path；
- `static_avoidance_sim.launch.py` 不复用固定五点轨迹 Launch，直接启动动力学、轨迹模式控制器、robot_state_publisher、可选 RViz2、环境、A* 和执行模式规划轨迹节点；
- Xacro 模型采用 0.20 m X 型布局，红色机头标记用于辨认 `+x`；
- RViz2 配置包含 Grid、RobotModel、TF、map/base_link Axes、实际/原始规划/简化/参考 Path、Goal，以及工作空间、原始障碍物和透明安全膨胀区 MarkerArray；
- SetGoal 工具发布 `/drone/goal`；终端更适合精确设置目标高度。

## 5. 关键文件入口

```text
README.md
docs/AI_CONTEXT.md

src/drone_msgs/msg/MotorRPM.msg
src/drone_msgs/msg/TrajectorySetpoint.msg

src/drone_dynamics/include/drone_dynamics/quadrotor_model.hpp
src/drone_dynamics/src/quadrotor_model.cpp
src/drone_dynamics/src/quadrotor_dynamics_node.cpp
src/drone_dynamics/test/test_quadrotor_model.cpp

src/drone_controller/include/drone_controller/mixer/motor_mixer.hpp
src/drone_controller/include/drone_controller/attitude/attitude_controller.hpp
src/drone_controller/include/drone_controller/altitude/altitude_controller.hpp
src/drone_controller/include/drone_controller/position/horizontal_position_controller.hpp
src/drone_controller/include/drone_controller/position/position_controller.hpp
src/drone_controller/include/drone_controller/hover/hover_controller.hpp
src/drone_controller/src/motor_mixer.cpp
src/drone_controller/src/attitude_controller.cpp
src/drone_controller/src/altitude_controller.cpp
src/drone_controller/src/horizontal_position_controller.cpp
src/drone_controller/src/position_controller.cpp
src/drone_controller/src/hover_controller.cpp
src/drone_controller/src/position_controller_node.cpp
src/drone_controller/test/

src/drone_mission/include/drone_mission/waypoint_manager.hpp
src/drone_mission/include/drone_mission/piecewise_quintic_trajectory.hpp
src/drone_mission/src/waypoint_manager.cpp
src/drone_mission/src/waypoint_manager_node.cpp
src/drone_mission/src/piecewise_quintic_trajectory.cpp
src/drone_mission/src/trajectory_mission_node.cpp
src/drone_mission/test/test_waypoint_manager.cpp
src/drone_mission/test/test_piecewise_quintic_trajectory.cpp

src/drone_planning/include/drone_planning/axis_aligned_box.hpp
src/drone_planning/include/drone_planning/static_environment.hpp
src/drone_planning/include/drone_planning/collision_checker.hpp
src/drone_planning/include/drone_planning/path_simplifier.hpp
src/drone_planning/include/drone_planning/planned_trajectory_builder.hpp
src/drone_planning/src/static_environment.cpp
src/drone_planning/src/collision_checker.cpp
src/drone_planning/src/static_environment_node.cpp
src/drone_planning/src/planned_trajectory_node.cpp
src/drone_planning/test/test_collision_checker.cpp
src/drone_planning/test/test_path_simplifier.cpp
src/drone_planning/test/test_planned_trajectory_builder.cpp

src/drone_bringup/config/dynamics.yaml
src/drone_bringup/config/controller.yaml
src/drone_bringup/config/mission.yaml
src/drone_bringup/config/trajectory.yaml
src/drone_bringup/config/environment.yaml
src/drone_bringup/config/astar.yaml
src/drone_bringup/config/planned_trajectory.yaml
src/drone_bringup/launch/basic_sim.launch.py
src/drone_bringup/launch/mission_sim.launch.py
src/drone_bringup/launch/trajectory_sim.launch.py
src/drone_bringup/launch/environment_sim.launch.py
src/drone_bringup/launch/planning_sim.launch.py
src/drone_bringup/launch/planned_trajectory_sim.launch.py
src/drone_bringup/test/test_single_goal_e2e.py
src/drone_bringup/test/test_waypoint_mission_e2e.py
src/drone_bringup/test/test_trajectory_mission_e2e.py
src/drone_bringup/test/test_static_environment_e2e.py
src/drone_bringup/test/test_astar_planner_e2e.py
src/drone_bringup/test/test_planned_trajectory_e2e.py
src/drone_bringup/urdf/drone.urdf.xacro
src/drone_bringup/rviz/drone_sim.rviz
```

参数和算法默认值可能同时存在于头文件与 YAML。运行基线以 `drone_bringup/config/*.yaml` 为准；物理参数还必须在动力学和 Mixer 两侧保持一致。

## 6. 坐标系、电机编号和单位

### 坐标系

- 世界系 `map`：ENU，x 前、y 左、z 上；
- 机体系 `base_link`：FLU，x 前、y 左、z 上；
- 重力：世界系负 z；
- `orientation_body_to_world`：将 `base_link` 向量旋转到 `map`；
- 四元数单位化后使用，动力学姿态增量因角速度在机体系表达而右乘。

### X 型电机布局

```text
                +x（前）
        M1 前左 CCW    M4 前右 CW

        M2 后左 CW     M3 后右 CCW
                -x（后）

          +y（左）      -y（右）
```

固定顺序：

1. M1：前左，CCW；
2. M2：后左，CW；
3. M3：后右，CCW；
4. M4：前右，CW。

外部 ROS2 接口使用 RPM，动力学内部使用 rad/s；其余物理量使用 SI 单位。不得单独改变动力学、Mixer、消息或 URDF 中任一处的编号和符号。

## 7. 节点、Topic 和消息类型

### `/quadrotor_dynamics_node`

订阅：

```text
/drone/motor_rpm_cmd  drone_msgs/msg/MotorRPM
```

发布：

```text
/drone/odom  nav_msgs/msg/Odometry
/drone/imu   sensor_msgs/msg/Imu
/drone/path  nav_msgs/msg/Path
/tf          map -> base_link
```

Odom 约定：

- `header.frame_id=map`；
- `child_frame_id=base_link`；
- pose 在 `map` 中；
- `twist.linear` 和 `twist.angular` 在 `base_link` 中。

因此控制器需要世界系速度时，必须计算：

```text
velocity_world = orientation_body_to_world * velocity_body
```

不能把 `odom.twist.twist.linear.z` 无条件当作世界系 `vz`。

### `/position_controller_node`

根据 `setpoint_source` 只订阅一种目标来源，并始终订阅 Odom：

```text
/drone/goal                 geometry_msgs/msg/PoseStamped       (pose_goal)
/drone/trajectory_setpoint  drone_msgs/msg/TrajectorySetpoint   (trajectory)
/drone/odom                 nav_msgs/msg/Odometry
```

发布：

```text
/drone/motor_rpm_cmd  drone_msgs/msg/MotorRPM
```

目标仅接受空 frame（按 `map` 处理）或 `map`。`pose_goal` 读取 position.x/y/z 和 orientation 中的 yaw，期望速度与加速度为零；`trajectory` 使用消息中的 position/velocity/acceleration/yaw，消息非法或接收超时 `0.20 s` 时安全发布零 RPM且不回退。Odom 姿态用于把完整 `twist.linear` 从 `base_link` 旋转到 `map`，其 x/y/z 分量分别作为水平和垂直速度反馈；`twist.angular` 保持机体系表达直接传入姿态控制器。

### `/waypoint_manager_node`

订阅 `/drone/odom`，发布：

```text
/drone/goal                            geometry_msgs/msg/PoseStamped
/drone/mission/current_waypoint_index std_msgs/msg/UInt32
/drone/mission/complete               std_msgs/msg/Bool
```

默认以 `20 Hz` 更新。五点列表按扁平 `[x,y,z,yaw]` 参数提供；默认阈值为位置 `0.20 m`、线速度 `0.15 m/s`、yaw `0.10 rad`、角速度 `0.20 rad/s`，连续保持 `1.0 s`。Odom 超时阈值为 `0.25 s`。

### `/trajectory_mission_node`

订阅 `/drone/odom`，发布：

```text
/drone/trajectory_setpoint         drone_msgs/msg/TrajectorySetpoint
/drone/trajectory/current_segment std_msgs/msg/UInt32
/drone/trajectory/complete        std_msgs/msg/Bool
/drone/reference_path             nav_msgs/msg/Path
```

默认以 `50 Hz` 发布。节点先静态发布 P0，位置误差 `<0.20 m` 且速度 `<0.15 m/s` 连续保持 `1.0 s` 后启动轨迹；使用稳态时钟累计有效 Odom 时间，Odom 超时或无效时暂停而不跳时。参考 Path 使用 transient-local QoS。

### `/static_environment_node`

订阅：

```text
/drone/odom  nav_msgs/msg/Odometry
```

发布：

```text
/drone/environment/markers       visualization_msgs/msg/MarkerArray
/drone/environment/in_collision std_msgs/msg/Bool
```

MarkerArray 使用 transient-local + reliable QoS，包含 `workspace`、`obstacles` 和 `inflated_obstacles` 三类 namespace。碰撞状态只在收到有限且未超过 `0.25 s` 的 Odom 时发布；首次状态不刷日志，之后只在安全/碰撞转换时记录日志。

### `/planned_trajectory_node`

以 transient-local + reliable QoS 订阅：

```text
/drone/planned_path  nav_msgs/msg/Path
```

首次收到合法 `map` 原始路径后生成一次并发布：

```text
/drone/simplified_path                                  nav_msgs/msg/Path
/drone/reference_path                                   nav_msgs/msg/Path
/drone/trajectory_generation/success                    std_msgs/msg/Bool
/drone/trajectory_generation/simplified_waypoints       std_msgs/msg/UInt32
/drone/trajectory_generation/selected_velocity_scale    std_msgs/msg/Float64
/drone/trajectory_generation/duration                   std_msgs/msg/Float64
```

所有生成结果均为 transient-local + reliable、`frame_id=map`。节点从 `environment.yaml` 读取 workspace/obstacles/base radius，从 `astar.yaml` 读取共享 `planning_margin`，从 `planned_trajectory.yaml` 读取轨迹参数。默认 `nominal_speed=0.35 m/s`、`min_segment_duration=2.0 s`、验证周期 `0.02 s`、参考 Path 周期 `0.05 s`、速度上限 `0.70 m/s`、加速度上限 `0.35 m/s²`、fixed yaw `0.0`。`0.45 m/s` 按明确时间公式在默认简化路径上产生约 `0.573 m/s²` 峰值加速度，与 `0.35 m/s²` 上限不相容，因此保持名义速度 `0.35 m/s`。执行参数默认为 `execution_enabled=false`、`publish_frequency=50 Hz`、`odometry_timeout=0.25 s`、准备位置/速度容差 `0.20 m / 0.15 m/s`、连续保持 `1.0 s`。启用时另发布：

```text
/drone/trajectory_setpoint                       drone_msgs/msg/TrajectorySetpoint
/drone/planned_trajectory/current_segment        std_msgs/msg/UInt32
/drone/planned_trajectory/complete               std_msgs/msg/Bool
```

### 可视化节点

- `/robot_state_publisher` 发布 `base_link` 下的固定子链接 TF；
- `/rviz2` 默认启动，可用 `use_rviz:=false` 关闭；
- `/drone/path` 是状态历史显示，`/drone/planned_path` 是 A* 原始栅格路径，`/drone/simplified_path` 是视线简化折线，`/drone/reference_path` 是连续轨迹参考；
- `/drone/environment/markers` 是静态环境几何显示，不是规划路径；
- `/drone/goal` 的 Pose 显示与控制器订阅使用同一消息类型。

## 8. 核心动力学与控制公式

### 电机、推力与力矩

```text
omega_cmd = clamp(RPM, RPM_min, RPM_max) * 2*pi/60
omega_next = omega + (1-exp(-dt/tau_m)) * (omega_cmd-omega)
F_i = k_F * omega_i^2
Q_i = k_M * omega_i^2
```

令 `a=arm_length/sqrt(2)`：

```text
T     = F1 + F2 + F3 + F4
tau_x = a * ( F1 + F2 - F3 - F4)
tau_y = a * (-F1 + F2 + F3 - F4)
tau_z =     (-Q1 + Q2 - Q3 + Q4)
```

M1/M3 为 CCW，对机体产生负 yaw 反扭矩；M2/M4 为 CW，产生正 yaw 反扭矩。

### 刚体运动

```text
p_dot = v
v_dot = R(q) * [0,0,T]^T / m + [0,0,-g]^T
I * omega_dot = tau - omega × (I*omega)
```

速度、位置和角速度使用固定步长半隐式 Euler；姿态使用机体系角速度形成增量四元数并归一化。

地面开启时只夹紧 `position_world.z >= ground_z`，并只清除接触时的负 `velocity_world.z`；不会清零 x/y 速度，也不会阻止正向离地运动。

理想 IMU 比力：

```text
specific_force_body = q.conjugate() * (linear_acceleration_world - gravity_world)
```

### 水平位置控制

输入位置、速度和输出加速度均在 `map/ENU` 世界系中表达：

```text
e_p = p_xy_desired - p_xy_current
e_v = v_xy_desired - v_xy_current
a_xy_raw = Kp .* e_p + Kd .* e_v + a_xy_feedforward
a_limit = min(max_horizontal_acceleration, gravity * tan(max_tilt_angle))
```

当 `|a_xy_raw| > a_limit` 时按向量模长等比例缩放，保持加速度方向不变并设置 `saturated=true`。反馈与前馈组合后再经过原有限幅；中间量使用扩展精度和稳定模长计算，极大有限输入在限幅前不会因 double 平方溢出而污染输出。

期望姿态采用几何构造：

```text
b3_des = normalize([ax, ay, gravity])
b1_heading = [cos(yaw), sin(yaw), 0]
b2_des = normalize(b3_des × b1_heading)
b1_des = b2_des × b3_des
R_des = [b1_des, b2_des, b3_des]
```

`R_des` 的列是期望机体系轴在世界系中的方向，随后转换并归一化为 body-to-world 四元数。yaw 是水平航向参考。ENU/FLU 且 yaw=0 时：`+x → 正 pitch`、`-x → 负 pitch`、`+y → 负 roll`、`-y → 正 roll`；yaw 非零时 roll/pitch 映射随航向旋转。

### 姿态控制

```text
q_error = q_current.conjugate() * q_desired
attitude_error = 2 * q_error.vec()
torque = Kp .* attitude_error + Kd .* (omega_desired-omega_current)
```

`q_error` 选择最短旋转路径；精确 180° 时用误差向量绝对值最大分量确定统一符号。输出力矩在 `base_link` 中按 `[roll,pitch,yaw]` 排列并逐轴限幅。

### 高度控制

```text
e_z  = z_desired - z_current
e_vz = vz_desired - vz_current
az_command = Kp*e_z + Kd*e_vz + az_feedforward
vertical_force = mass * (gravity + az_command)
cos_tilt = (orientation_body_to_world * UnitZ).z()
collective_thrust = vertical_force / cos_tilt
```

`az_command`、倾角余弦和总推力均有限幅/非法输入保护；`cos_tilt<=0` 返回无效零推力，过小正值使用 `min_tilt_cosine`。

### Mixer

令 `b=k_M/k_F`，控制器请求为 `[T,tx,ty,tz]`：

```text
F1 = (T + tx/a - ty/a - tz/b) / 4
F2 = (T + tx/a + ty/a + tz/b) / 4
F3 = (T - tx/a + ty/a - tz/b) / 4
F4 = (T - tx/a - ty/a + tz/b) / 4
```

随后由 `omega=sqrt(F/k_F)` 转成 RPM。逐电机限幅后的实际 Wrench 可能不同于请求值，调用者必须关注 `saturated`。

## 9. 当前参数基线

### 动力学与安全参数

| 参数 | 当前值 |
|---|---:|
| 质量 | `1.0 kg` |
| 惯量 | `Ixx=0.02, Iyy=0.02, Izz=0.04 kg·m²` |
| 机臂长度 | `0.20 m` |
| 推力系数 | `1.91e-6 N/(rad/s)²` |
| 反扭矩系数 | `2.60e-7 N·m/(rad/s)²` |
| 电机时间常数 | `0.05 s` |
| RPM 范围 | `0～20000 RPM` |
| 重力 | `9.80665 m/s²` |
| 动力学频率 | `200 Hz`，固定 `dt=0.005 s` |
| Path | 名义 `20 Hz`，最多 `2000` 点 |
| 正常 Launch 地面 | 开启，`ground_z=0.0 m` |
| MotorRPM watchdog | 开启，`0.30 s` |

名义稳态悬停转速约 `10818.9 RPM/电机`。

### 控制参数

| 参数 | 当前值 |
|---|---:|
| 控制频率 | `100 Hz` |
| Odom 超时 | `0.20 s` |
| 高度 Kp / Kd | `3.0 / 3.5` |
| 上升/下降加速度限制 | `5.0 / 5.0 m/s²` |
| 总推力范围 | `0～30 N` |
| 最小倾角余弦 | `0.5` |
| roll/pitch 姿态 Kp | `4.0 / 4.0` |
| yaw 姿态 Kp | `1.0` |
| roll/pitch 角速度 Kd | `0.35 / 0.35` |
| yaw 角速度 Kd | `0.40` |
| roll/pitch 最大力矩 | `1.0 / 1.0 N·m` |
| yaw 最大力矩 | `0.20 N·m` |

高度 `Kp=3.0,Kd=3.5`、roll/pitch `Kp=4.0,Kd=0.35,max=1.0 N·m` 和 yaw `Kp=1.0,Kd=0.40,max=0.20 N·m` 是当前实际验收后的基线，未经新测试不要随意改动。

### 当前水平参数（已通过单目标系统验收）

| 参数 | `controller.yaml` 当前值 |
|---|---:|
| x/y 位置 Kp | `0.40 / 0.40` |
| x/y 速度 Kd | `1.20 / 1.20` |
| 重力 | `9.80665 m/s²` |
| 最大水平加速度 | `0.40 m/s²` |
| 最大倾角 | `0.08 rad` |

这组中等保守参数在姿态阻尼修正后通过了小目标和完整三维目标验收。此前同一组水平参数在旧姿态 `Kd=0.20` 下约 5～6 秒后自激，说明修复来自姿态内环稳定裕度，而不是继续削弱水平指令。

## 10. 当前限制和风险

- 旧 roll/pitch `Kd=0.20` 在完整闭环中缺乏稳定裕度，不能恢复为运行基线；
- 当前固定小倾角测试覆盖 `0.02 rad` 和 20 秒，真实三维目标最大倾角约 `0.039 rad`；更大倾角、外部扰动和极限工况仍未系统验证；
- `/drone/path` 只是实际历史位姿；多目标 WaypointManager 仍是 stop-settle-switch，连续轨迹由独立节点和 Launch 提供；
- 静态 AABB 环境、A*、视线简化、安全连续参考和规划轨迹执行已经实现；局部规划、在线重规划和动态障碍仍未实现；
- 环境碰撞状态只报告几何结果，不阻止电机工作，也没有障碍物物理碰撞响应；
- 简化地面没有反弹、摩擦、起落架弹性、姿态约束或碰撞几何；
- 动力学没有空气阻力、旋翼陀螺效应、传感器噪声或偏置；
- 控制器没有位置积分环和复杂反饱和，持续扰动下可能有稳态误差；
- 高度控制必须使用世界系 `vz`，Odom `twist.linear` 的机体系约定是后续接入最容易犯错的地方；
- Mixer 与动力学的 `arm_length`、`k_F`、`k_M` 和 RPM 范围在两个 YAML/参数结构中重复，修改时必须同步；
- watchdog 只在“至少收到过一次命令”后检查超时；启动后从未收到命令时模型保持原有零目标 RPM；
- watchdog 没有瞬间清零实际电机转速；这是保持电机一阶响应连续性的有意设计；
- 长时间、高角速度、最大 RPM 和外部扰动稳定性仍未系统验证；
- package 的许可证字段仍为 `TODO`。

## 11. 最新构建、测试和运行验证汇总

### 仓库状态

当前稳定代码基线已经包含：

- 四旋翼动力学与简化地面；
- Motor Mixer；
- 姿态/角速度控制器；
- 高度控制器；
- 水平位置控制纯算法；
- HoverController；
- PositionController、完整 Odom 速度转换和 ROS2 x/y/z/yaw 闭环；
- roll/pitch `Kd=0.35` 姿态稳定性修复；
- 使用真实控制器、Mixer、电机响应和刚体动力学的完整动态闭环测试；
- 已通过真实运行验收的单目标三维位置控制参数；
- 顺序 WaypointManager、任务节点和五点任务配置；
- 多目标点真实 ROS2 端到端回归；
- C² 分段五次轨迹、轨迹任务节点、参考 Path 和连续轨迹真实 ROS2 端到端回归；
- 静态 AABB 环境、安全半径、点/线段碰撞查询、环境节点和 Domain 96 ROS2 集成回归；
- 静态避障执行 Launch、规划轨迹执行状态机和 Domain 99 真实闭环安全回归；
- MotorRPM watchdog；
- 当前文档整理结果。

`main` 已包含三维 A*、路径简化和只显示/验证的安全规划轨迹。本分支增加静态避障执行，但不实现局部规划、动态障碍或在线重规划。Git 当前状态始终以实际命令为准。

开始任何新任务前，应重新执行：

```bash
git status --short --branch
git log -3 --oneline
```

Git 当前状态始终以实际命令输出为准，不在本文件中长期保存固定提交 SHA。

### 构建与测试

最近一次代码验证结论：

- 六个 package 完整 `colcon build --symlink-install` 成功；
- 当前工作区汇总为 `203 tests, 0 errors, 0 failures, 0 skipped`；
- 动力学测试覆盖自由落体、地面、起飞/落地、推力/力矩方向和电机响应；
- 控制器测试覆盖 Mixer 11 项、姿态 11 项、高度 13 项、Hover/转换 18 项；
- 水平位置控制器共 17 项测试，并覆盖加速度前馈叠加、限幅和非法前馈安全返回；
- `PositionController` 共 10 项测试，并覆盖三维加速度前馈贯通现有水平和高度链路；
- 完整速度转换新增 4 项测试，覆盖单位姿态、pitch 产生 world z、yaw 旋转 world x/y 和非法四元数；
- 姿态完整动态闭环新增 6 项测试，真实复用 `AttitudeController`、`MotorMixer`、电机一阶响应和 `QuadrotorModel`，按 100 Hz 控制/200 Hz 动力学运行；其中 1 项固定记录旧 `Kd=0.20` 不满足稳定裕度，5 项验证 `Kd=0.35` 的正负 roll/pitch 和水平姿态 20 秒稳定性；
- `drone_bringup/test/test_single_goal_e2e.py` 复用 `basic_sim.launch.py use_rviz:=false`，通过真实 `/drone/goal → position_controller_node → /drone/motor_rpm_cmd → quadrotor_dynamics_node → /drone/odom` 链路验收 `(2,1,1.5)`；三维误差 `<0.30 m`、水平速度 `<0.15 m/s`、世界系 `|vz|<0.10 m/s` 必须连续保持至少 `2.0 s`，随后继续观测 `3.0 s` 且不得离开稳定区域；观测结束还要求误差 `<0.10 m`、水平速度 `<0.08 m/s`、世界系 `|vz|<0.05 m/s`，并检查有限值、全程与启动宽限期后 RPM 范围、消息活性、节点存活和进程退出码；
- `WaypointManager` 有 15 项纯算法测试，覆盖阈值严格性、连续保持、显式重置、顺序推进、最终完成、yaw 环绕、线/角速度、非有限输入和非法配置；
- `test_waypoint_mission_e2e.py` 使用真实任务、控制和动力学节点，独立验证索引严格按 `0→1→2→3→4` 以及五个 `/drone/goal` 的完整 x/y/z/yaw 序列，并检查完成状态不回退、最终目标持续发布、数值有限、节点存活和进程正常退出；
- `PiecewiseQuinticTrajectory` 有 10 项纯算法测试，新增覆盖默认 scale 与旧行为精确一致、scale `0.0` 的中间停速和逐段直线性、所有候选比例的 C² 连续性，以及非法比例拒绝；
- `test_trajectory_mission_e2e.py` 使用真实轨迹、控制和动力学节点，独立检查段序、参考位置/速度/加速度边界连续性、中间非零速度、跟踪误差、完成后保持、参考 Path、有限值、节点存活和进程退出；
- `CollisionChecker` 有 22 项确定性纯算法测试，覆盖环境/半径合法性、工作空间和障碍物边界、安全点、膨胀区、非有限输入，三维线段穿越、相切、角点、平行、端点、零长度和极短线段，以及默认规划演示场景的直线受阻与已知安全折线契约；
- `test_static_environment_e2e.py` 使用独立 `ROS_DOMAIN_ID=96`，只启动环境节点并人工发布 Odom，检查 Marker 数量、frame 和 namespace，以及安全、原障碍物、膨胀区、工作空间外、恢复安全和非法 Odom 抑制发布；
- `AStarPlanner` 有 12 项纯算法测试，覆盖非法资源参数、起终点碰撞、空环境三维对角、默认场景安全路径、精确端点、重复确定性、不可穿墙、非对齐端点吸附和对角边角穿越抑制；
- `test_astar_planner_e2e.py` 使用独立 `ROS_DOMAIN_ID=97`，启动真实环境和规划节点，并用测试内独立 slab 几何复核完整路径不穿过膨胀障碍物；
- `PathSimplifier` 有 6 项纯算法测试，覆盖点数、有限性、碰撞点/边拒绝、无障碍直达、障碍转折、安全输出、精确端点和确定性；
- `PlannedTrajectoryBuilder` 有 4 项纯算法测试，覆盖默认 A* 场景、安全与动态约束、精确首尾、确定性、完整速度曲线碰撞后的候选下降与 `scale=0` 保底，以及非法参数；
- `test_planned_trajectory_e2e.py` 使用独立 `ROS_DOMAIN_ID=98`，只启动环境、A* 和规划轨迹节点，并以测试内独立 slab 几何复核简化折线和参考 Path 的每条边都不穿越 `0.35 m` 膨胀障碍；同时确认没有 `/drone/trajectory_setpoint` Topic；
- `test_static_avoidance_e2e.py` 使用独立 `ROS_DOMAIN_ID=99`，启动完整 A*、轨迹、控制与动力学链路，检查起点准备、段序、完成保持、有限性、消息/节点活性、唯一参考与 setpoint 发布者、环境碰撞状态，并以独立基础 `0.25 m` 膨胀 AABB 检查实际 Odom；
- 精确 180° 四元数 q/-q 确定性和极大有限 Mixer 输入均已覆盖。

以上结果来自本阶段实际执行的局部和完整构建/测试。用户已在 RViz2 中人工确认离散五点任务能够依次到达并在各点附近停稳后切换；连续轨迹是另一条运行链路。

### 已实际运行验证

- 自动从地面起飞至 `1.5 m` 并稳定悬停；
- `0 → 1.5 m`、`1.5 → 2.0 m`、`2.0 → 1.5 m` 的自动升降均由用户实际确认；
- 调参后 yaw 转向由用户在 RViz2 中确认快速接近目标且基本无超调；
- Odom 名义约 `200 Hz`，控制器命令名义 `100 Hz`；
- RobotModel、TF、状态历史 Path 和 Goal Pose 已显示；
- 正常控制器持续发布时 watchdog 不误触发；控制器结束后约 `0.304 s` 触发超时并使目标 RPM 归零，警告不按 200 Hz 刷屏；
- 控制器重启并重新发送目标后，watchdog 和高度闭环能够恢复。
- ROS2 单目标端到端完整回归实跑中，首次进入位置容差为 `5.466 s`、连续稳定开始于 `5.671 s`、`7.676 s` 完成首次验收、`10.677 s` 完成额外观测，总连续稳定 `5.006 s`；观测结束位置 `(1.989402,0.994701,1.499997) m`、三维误差 `0.011848 m`、世界系速度 `(0.007201,0.003601,0.000004) m/s`，命令 RPM 全程范围 `0～13756.945`、启动宽限期后范围 `10598.006～10818.975`，共采集 `2159` 条 Odom 和 `1080` 条 MotorRPM；
- ROS2 五点任务完整回归中，索引首次观测/切换时间为 `0.034/3.586/10.536/16.786/23.737 s`，最终点于 `30.088 s` 完成；发布目标依次精确匹配五个预设 x/y/z/yaw；结束位置 `(-0.001908,0.034044,1.499991) m`、误差 `0.034097 m`、线速度 `0.021450 m/s`，完成后继续收到 `41` 个目标样本，共采集 `6418` 条 Odom 和 `647` 条 Goal，所有进程正常退出；
- ROS2 连续五点轨迹完整回归中，轨迹于 `3.576 s` 启动，段索引首次观测/切换时间为 `0.025/9.556/15.576/21.576 s`，于 `27.576 s` 完成；最大参考速度 `0.558524 m/s`、最大参考加速度 `0.271903 m/s²`、采样最大跟踪误差 `0.030507 m`，相邻参考最大位置/速度/加速度步长为 `0.011213 m`、`0.005455 m/s`、`0.008975 m/s²`，三个中间点附近参考速度为 `0.215906/0.211353/0.215706 m/s`；结束位置 `(-0.000039,0.002669,1.499998) m`、误差 `0.002669 m`、速度 `0.001033 m/s`，完成后继续收到 `201` 个 setpoint，共采集 `6311` 条 Odom 和 `1578` 条 setpoint；
- 用户已在 RViz2 中人工确认无人机能够沿连续参考轨迹完成五点飞行，中间 waypoint 不再停稳后切换，整体运动连续；
- 静态环境集成运行已确认 `workspace/obstacles/inflated_obstacles` Marker 分类、碰撞 true/false 转换，以及非法或超时 Odom 不发布状态；`environment_sim.launch.py` 的 RViz2 画面已确认工作空间线框、两个原始障碍物、透明安全膨胀区域和无人机模型处于同一 `map` 场景，初始碰撞状态为 `false`；
- 默认 A* ROS2 集成使用 `0.35 m` 有效规划半径，输出 `40` 个路径点、总长 `11.978138 m`、扩展 `6013` 个节点、最大高度 `1.6 m`；结果安全绕过墙体且尚未驱动无人机运动，规划耗时在本机约 `0.4～0.45 s`；
- 默认规划轨迹集成将 `40` 个原始点简化为 `5` 个点，折线长度由 `11.978138 m` 降为 `11.175430 m`；选择 `velocity_scale=1.00`，总时长 `31.929800 s`，采样最大速度 `0.534070 m/s`、最大加速度 `0.346694 m/s²`、验证采样 `1598` 个，独立几何复核无轨迹碰撞；参考 Path 为 `640` 个点；
- `planned_trajectory_sim.launch.py` 已以默认 `use_rviz=true` 实际启动；用户已在 RViz2 中人工确认黄色 A* 原始栅格路径、粉色视线简化折线和蓝色连续参考轨迹能够同时显示，蓝色连续参考轨迹整体平滑，符合当前路径轨迹化预期。运行期不存在 `/drone/trajectory_setpoint`，Odom 持续保持 `(0,0,0)`，确认本阶段没有驱动飞行。人工观察不用于精确测量安全距离，几何净空由 Domain 98 独立 slab 检查确认。
- Domain 99 静态避障完整回归中，起点准备和轨迹开始均为 `3.936 s`，段索引首次观测/切换时间为 `0.375/14.240/27.424/31.765 s`，任务于 `35.847 s` 完成；最大同期采样跟踪误差 `0.030705 m`，实际 Odom 对基础 `0.25 m` 膨胀障碍物的最小采样净空 `0.168625 m`，控制器日志 `saturated=true` 为 `0` 次；最终位置误差 `0.005969 m`、速度 `0.001466 m/s`，完成后继续收到 `600/150/300` 个 Odom/setpoint/RPM 样本，总样本数 `7762/1923/3880`，全程碰撞状态未出现 true，节点和消息流持续存活且无 NaN/Inf；
- 用户已在 RViz2 中观察静态避障完整执行，整体运动和绕障效果符合预期。精确碰撞净空、跟踪误差和最终误差仍以自动回归指标为准；
- 用户已人工验证多个较远目标能够到达并持续悬停；未为这些人工场景记录或推断精确误差、速度与采样数。

姿态阻尼扫描使用真实 `AttitudeController → MotorMixer → 电机一阶响应 → QuadrotorModel → 姿态反馈`，固定 `+0.02 rad roll` 运行 20 秒：

| roll/pitch Kd | 最大姿态 | 最大角速度 | 最后 5 秒最大误差 | 结论 |
|---:|---:|---:|---:|---|
| `0.20` | `0.118969 rad` | `1.45628 rad/s` | `0.100986 rad` | 不通过 |
| `0.25` | `0.032947 rad` | `0.22175 rad/s` | `2.60e-7 rad` | 通过 |
| `0.30` | `0.029915 rad` | `0.20590 rad/s` | `7.04e-13 rad` | 通过 |
| `0.35` | `0.027314 rad` | `0.19215 rad/s` | `5.62e-14 rad` | 通过并选用 |
| `0.40` | `0.025062 rad` | `0.18012 rad/s` | `2.97e-14 rad` | 通过 |
| `0.50` | `0.021384 rad` | `0.16014 rad/s` | `1.38e-14 rad` | 通过 |

选用 `0.35`，因为它比 `Kd > tau_m*Kp = 0.20` 的近似边界有明显裕度，响应和稳态均满足标准，同时不使用更大的 `0.40/0.50`。`±roll`、`±pitch` 四组在 `Kd=0.35` 下指标对称：最大姿态约 `0.027314 rad`、最大角速度约 `0.19215 rad/s`、最后 5 秒误差和角速度接近数值零，命令 RPM `10502.3～11126.6`，实际 RPM `10634.1～11002.5`，Attitude/Mixer 均无饱和。水平姿态 20 秒保持零姿态/零角速度和 `10818.9 RPM`。

本阶段 ROS2 实跑结果：

- 原地 `(0,0,1.5)` 观察 12 秒：最终 `(0,0,1.4999997)`，世界系 `vz=3.61e-7 m/s`，roll/pitch 与角速度为 0，RPM 命令 `10426.1～11374.9`（包含起飞瞬态），无边界值；
- 小目标 `(0.5,0,1.5)` 观察 17 秒：最终 `(0.499990,0,1.500000)`，三维误差 `9.89e-6 m`，水平速度 `1.02e-5 m/s`，最大 pitch `0.01421 rad`、最大 pitch 角速度 `0.06679 rad/s`，RPM `10705.9～10932.3`；
- 完整目标 `(2,1,1.5)` 观察 27 秒：最终 `(2.000000114,1.000000076,1.500000000)`，三维误差 `1.37e-7 m`，水平速度 `7.84e-8 m/s`，`|vz|=1.13e-14 m/s`；最大 `|roll|=0.02611 rad`、`|pitch|=0.03913 rad`，最大 roll/pitch 角速度 `0.07517/0.11250 rad/s`，RPM `10554.9～11090.8`；
- 完整目标共采集 5401 个 Odom 和 2700 个 RPM 命令样本，无 NaN/Inf 或 0/20000 RPM；控制节点 1 Hz 日志样本全程 `saturated=false`。各级饱和没有独立 ROS Topic，因此运行期“零饱和持续时间”限于日志采样分辨率；完整闭环自动测试则连续累计并确认 Attitude/Mixer 饱和时间均为 0。

以上结果确认旧参数下十几秒后的姿态自激已经消除，并确认多目标顺序飞行、连续轨迹跟踪、静态环境碰撞监测、三维 A*、路径简化、安全规划轨迹生成和静态避障执行均可重复完成。

## 12. x/y 接入状态与系统验收标准

### 设计约束

1. 保持 `map` ENU、`base_link` FLU、body-to-world 四元数和现有电机编号不变；
2. 不破坏已调好的高度/yaw 参数及 100 Hz/200 Hz 基线；
3. x/y 控制算法已作为与 ROS2 无关的独立类实现和测试，接入时不得把算法复制进节点；
4. 输入位置和水平速度必须明确位于世界系；从 Odom 读取速度时先将完整 `twist.linear` 从 `base_link` 旋转到 `map`；
5. 世界系水平加速度/误差到期望 roll/pitch 的符号必须结合 ENU/FLU、重力和推力方向推导并用单轴测试确认；
6. 期望倾角、水平加速度和输出必须有限且限幅；非法输入应安全返回无效结果；
7. 节点已使用目标 x/y，README 中的三维位置能力必须继续以真实运行回归为依据；
8. 不在 x/y 任务中顺带实现轨迹、地图、规划或避障。

### 已完成的算法和接入级验收

- 零水平误差输出水平姿态；
- 正/负 x 和 y 误差分别产生符合约定的期望 pitch/roll；
- 速度阻尼方向正确；
- 倾角限制、NaN/Inf、非法参数和饱和状态均有单元测试；
- `PositionController` 的状态传播、失败零 RPM 和 Hover 行为保持已有单元测试；
- 完整 Odom 线速度机体系到世界系转换已有独立测试；
- 原有测试和新增测试全部保持通过，当前完整汇总为 199 项。

### 系统级验收

- 原地 `1.5 m` 悬停和 yaw 转向不得回归；
- 非零 x/y 目标能够产生对应倾斜和水平运动；
- 能到达单个三维目标并稳定，位置、姿态、RPM 无 NaN/Inf 或持续发散；
- `/drone/odom`、TF、RobotModel 和 `/drone/path` 状态一致；
- 明确记录输入、预期、实际数据和人工 RViz2 观察边界。

达到这些条件代表单目标三维位置控制完成；多目标顺序任务和连续轨迹另由各自纯算法测试与真实 ROS2 端到端测试验收，但仍不代表地图、碰撞检查、规划或避障完成。

当前系统级数值验收全部通过：非零 x/y 产生正确倾斜和运动，接近目标后恢复水平，位置、速度、姿态和 RPM 均满足标准。用户已在 RViz2 中确认五点任务按序到达，RobotModel/TF/Path 运行正常。

## 13. AI 工作规则

开始任务前：

1. 阅读 `README.md` 和本文件；
2. 检查 Git 状态、目录、相关源码、参数、测试和 Launch；
3. 明确本次范围、禁止修改项和验收条件；
4. 先核对当前代码事实，不沿用历史印象。

实施时：

1. 只修改当前任务需要的文件，保留用户已有改动；
2. 算法与 ROS2 通信分离，参数不得无说明散落硬编码；
3. 不擅自改变坐标系、电机编号、Topic、消息类型和已验证参数；
4. 关键公式先检查坐标表达、四元数方向、单位和符号；
5. 编译成功不等于功能完成，节点启动也不等于算法通过；
6. “已完成并验证”必须有实际测试或运行证据；RViz2 人工效果未经观察不得写成已确认；
7. 不通过删除有效功能或放宽测试来规避问题；
8. 不安装、删除系统软件或执行破坏性 Git 操作，除非用户明确授权。

完成任务后：

1. 报告修改文件、公式/接口决策、构建测试结果、运行结果和未验证边界；
2. 更新本文件中的当前结论，直接替换过时描述，不追加开发日报；
3. 只有稳定能力、环境或使用方式变化时才更新 README；
4. 保持测试数量只记录当前最新汇总，不保留过时累计数量；
5. 不把 `/drone/path` 历史显示写成轨迹规划或跟踪功能。

# AI_CONTEXT

本文只保存当前有效的工程事实、接口、约束和验收基线，供新的 AI 在较短上下文内接手开发。历史开发过程不在此累积。

## 1. 项目定位

本项目是基于 Ubuntu 22.04 和 ROS2 Humble 的小型四旋翼无人机闭环仿真系统。动力学与控制器使用 C++17 和 Eigen3，RViz2 负责显示，colcon + ament_cmake 负责构建。

项目采用“纯算法类与 ROS2 节点分离”的结构。当前动力学、高度/yaw 闭环和基础可视化已经运行；最终目标是增加 x/y 位置控制、轨迹、多目标点、地图、规划和静态避障。

## 2. 当前阶段和下一任务

### 当前阶段

动力学和高度/yaw 闭环已完成，正在进行 x/y 位置控制前的整理和稳定性验证。

当前控制器：

- 使用 `/drone/goal` 的 z 和 yaw；
- 将期望 roll/pitch 固定为 0；
- 明确忽略目标 x/y；
- 已能够自动起飞、升降、悬停和 yaw 转向。

### 下一任务

在不破坏现有高度/yaw 链路的前提下，设计并独立测试 x/y 位置控制器，使世界系水平位置/速度误差生成合理的期望 roll/pitch。完成算法级符号、限幅和安全测试后，才接入 ROS2 节点。

当前不进入轨迹生成、多目标点、地图、规划或避障开发。

## 3. 当前架构与数据流

### 已实现闭环

```text
/drone/goal (PoseStamped；当前使用 z、yaw)
  ↓
position_controller_node
  ↓
AltitudeController → AttitudeController → MotorMixer
  ↓
/drone/motor_rpm_cmd (MotorRPM)
  ↓
quadrotor_dynamics_node → QuadrotorModel
  ↓
/drone/odom、/drone/imu、/drone/path、map -> base_link
  ├─→ position_controller_node 状态反馈
  └─→ robot_state_publisher + RViz2 可视化
```

`HoverController` 是不依赖 ROS2 的组合层，按高度控制器、姿态控制器、Mixer 顺序调用；任何一级无效时最终返回四电机零 RPM，各级饱和状态向上传递。

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
- `HoverController`：组合高度、姿态和 Mixer；
- `position_controller_node` 已实际调用 `HoverController`，不是仅保存消息的骨架。

### drone_bringup 与可视化

- `basic_sim.launch.py` 加载动力学和控制器参数并启动两个节点、robot_state_publisher 和可选 RViz2；
- Xacro 模型采用 0.20 m X 型布局，红色机头标记用于辨认 `+x`；
- RViz2 配置包含 Grid、RobotModel、TF、map/base_link Axes、`/drone/path` 和 `/drone/goal`；
- SetGoal 工具发布 `/drone/goal`；终端更适合精确设置目标高度。

## 5. 关键文件入口

```text
README.md
docs/AI_CONTEXT.md

src/drone_msgs/msg/MotorRPM.msg

src/drone_dynamics/include/drone_dynamics/quadrotor_model.hpp
src/drone_dynamics/src/quadrotor_model.cpp
src/drone_dynamics/src/quadrotor_dynamics_node.cpp
src/drone_dynamics/test/test_quadrotor_model.cpp

src/drone_controller/include/drone_controller/mixer/motor_mixer.hpp
src/drone_controller/include/drone_controller/attitude/attitude_controller.hpp
src/drone_controller/include/drone_controller/altitude/altitude_controller.hpp
src/drone_controller/include/drone_controller/hover/hover_controller.hpp
src/drone_controller/src/motor_mixer.cpp
src/drone_controller/src/attitude_controller.cpp
src/drone_controller/src/altitude_controller.cpp
src/drone_controller/src/hover_controller.cpp
src/drone_controller/src/position_controller_node.cpp
src/drone_controller/test/

src/drone_bringup/config/dynamics.yaml
src/drone_bringup/config/controller.yaml
src/drone_bringup/launch/basic_sim.launch.py
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

订阅：

```text
/drone/goal  geometry_msgs/msg/PoseStamped
/drone/odom  nav_msgs/msg/Odometry
```

发布：

```text
/drone/motor_rpm_cmd  drone_msgs/msg/MotorRPM
```

目标仅接受空 frame（按 `map` 处理）或 `map`。当前只读取 position.z 和 orientation 中的 yaw，忽略 position.x/y，并移除目标 roll/pitch。

### 可视化节点

- `/robot_state_publisher` 发布 `base_link` 下的固定子链接 TF；
- `/rviz2` 默认启动，可用 `use_rviz:=false` 关闭；
- `/drone/path` 是状态历史显示，不是规划轨迹；
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
| roll/pitch 角速度 Kd | `0.20 / 0.20` |
| yaw 角速度 Kd | `0.40` |
| roll/pitch 最大力矩 | `1.0 / 1.0 N·m` |
| yaw 最大力矩 | `0.20 N·m` |

高度 `Kp=3.0,Kd=3.5` 和 yaw `Kp=1.0,Kd=0.40,max=0.20 N·m` 是当前实际验收后的基线，未经新测试不要随意改动。

## 10. 当前限制和风险

- x/y 目标被忽略，三维目标点飞行尚未实现；
- `/drone/path` 只是历史位姿；轨迹生成、跟踪和多目标点尚未实现；
- 地图、规划和避障尚未实现，也没有相应 package；
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

文档整理开始前，Git 位于 `main`，与 `origin/main` 同步且工作树干净。最新提交：

```text
23487fbaf9d586290c6d69bad1430fefdc68cdee
feat: add motor command timeout protection
2026-07-13 17:25:23 +08:00
```

### 构建与测试

最近一次代码验证结论：

- 四个 package 完整 `colcon build --symlink-install` 成功；
- 当前工作区汇总为 `65 tests, 0 errors, 0 failures, 0 skipped`；
- 动力学测试覆盖自由落体、地面、起飞/落地、推力/力矩方向和电机响应；
- 控制器测试覆盖 Mixer 11 项、姿态 11 项、高度 13 项、Hover/转换 14 项；
- 精确 180° 四元数 q/-q 确定性和极大有限 Mixer 输入均已覆盖。

本次文档整理没有重新构建代码；以上为当前提交已有的最近验证结果。

### 已实际运行验证

- 自动从地面起飞至 `1.5 m` 并稳定悬停；
- `0 → 1.5 m`、`1.5 → 2.0 m`、`2.0 → 1.5 m` 的自动升降均由用户实际确认；
- 调参后 yaw 转向由用户在 RViz2 中确认快速接近目标且基本无超调；
- Odom 名义约 `200 Hz`，控制器命令名义 `100 Hz`；
- RobotModel、TF、状态历史 Path 和 Goal Pose 已显示；
- 正常控制器持续发布时 watchdog 不误触发；控制器结束后约 `0.304 s` 触发超时并使目标 RPM 归零，警告不按 200 Hz 刷屏；
- 控制器重启并重新发送目标后，watchdog 和高度闭环能够恢复。

未把尚未执行的 x/y、轨迹跟踪、地图或避障写为已验证。

## 12. x/y 控制前的约束与验收标准

### 设计约束

1. 保持 `map` ENU、`base_link` FLU、body-to-world 四元数和现有电机编号不变；
2. 不破坏已调好的高度/yaw 参数及 100 Hz/200 Hz 基线；
3. x/y 控制算法先作为与 ROS2 无关的独立类实现和测试；
4. 输入位置和水平速度必须明确位于世界系；从 Odom 读取速度时先将完整 `twist.linear` 从 `base_link` 旋转到 `map`；
5. 世界系水平加速度/误差到期望 roll/pitch 的符号必须结合 ENU/FLU、重力和推力方向推导并用单轴测试确认；
6. 期望倾角、水平加速度和输出必须有限且限幅；非法输入应安全返回无效结果；
7. 接入节点后才能开始使用目标 x/y，接入前继续明确忽略；
8. 不在 x/y 任务中顺带实现轨迹、地图、规划或避障。

### 进入 ROS2 接入前

- 零水平误差输出水平姿态；
- 正/负 x 和 y 误差分别产生符合约定的期望 pitch/roll；
- 速度阻尼方向正确；
- 倾角限制、NaN/Inf、非法参数和饱和状态均有单元测试；
- 现有 65 项测试保持通过。

### 系统级验收

- 原地 `1.5 m` 悬停和 yaw 转向不得回归；
- 非零 x/y 目标能够产生对应倾斜和水平运动；
- 能到达单个三维目标并稳定，位置、姿态、RPM 无 NaN/Inf 或持续发散；
- `/drone/odom`、TF、RobotModel 和 `/drone/path` 状态一致；
- 明确记录输入、预期、实际数据和人工 RViz2 观察边界。

达到这些条件只代表单目标三维位置控制完成，不代表轨迹、多目标点、地图或避障完成。

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

# AI_CONTEXT

## 1. 项目定位

本项目是一个基于 Ubuntu 22.04 和 ROS2 Humble 的小型四旋翼无人机仿真系统。

最终目标是实现以下完整闭环：

```text
目标点
→ 路径规划与避障
→ 安全参考目标
→ 位置与姿态控制器
→ 4 个电机 RPM
→ 四旋翼动力学
→ 无人机状态
→ 控制反馈与 RViz2 可视化
```

项目不是对参考仓库的简单复制或包装，核心动力学、控制器和 ROS2 系统结构需要重新实现。

## 2. 总体推进原则

项目不按照固定天数推进，而按照可验收的功能阶段推进。

计划顺序：

1. 工程初始化与基础通信；
2. 动力学模型；
3. 姿态与高度控制；
4. 三维位置控制；
5. RViz2 与多目标点；
6. 障碍物地图；
7. 路径规划与避障；
8. 稳定性测试与加分功能；
9. 实验整理、报告和演示。

不得在前一阶段未稳定时，大规模推进依赖它的后续模块。

## 3. 当前阶段目标

### 当前阶段

动力学模块第一阶段已验证，正在进行控制器纯算法组件的独立实现与验证。

### 当前任务

1. 四旋翼刚体动力学、电机一阶响应和固定步长积分已实现；
2. 零 RPM、对称推力、roll、pitch、yaw 和电机限幅测试已通过；
3. Odom、IMU、Path 和 `map -> base_link` TF 已接入并完成运行检查；
4. 基础 URDF、robot_state_publisher 和 RViz2 显示已实现并完成运行检查；
5. 可配置的简化水平地面约束已实现并完成单元/运行验证；
6. 与当前动力学符号严格一致的 Motor Mixer 已实现并通过独立单元测试，尚未接入 ROS2 节点；
7. 与动力学四元数和力矩符号一致的姿态/角速度控制器已实现并通过独立单元测试，尚未接入 ROS2 节点；
8. 下一步独立设计高度控制器，在各算法稳定后再接入控制器节点。

### 当前阶段完成标准

满足以下条件后，才能进入控制器开发：

* 工作空间可以通过 `colcon build` 编译；
* 动力学节点可以正常启动；
* 可以接收四个电机 RPM；
* 可以发布 `/drone/odom`；
* 可以发布 `map -> base_link` TF；
* RPM 为零时，无人机受到重力并下落；
* 四电机同速时，总推力方向和大小正确；
* 可以分别验证滚转、俯仰和偏航力矩方向；
* RViz2 中能够看到无人机位置或简化模型变化。

## 4. 已完成并验证

四旋翼动力学模块已完成第一阶段实现和独立验证。

已完成的工程基础验证：

* 已建立 ROS2 工作空间目录、Git 仓库和基础 `.gitignore`；
* Ubuntu 22.04、ROS2 Humble、C++17/CMake、Eigen3 和计划使用的基础 ROS2 依赖已通过检查；
* 已创建 `drone_msgs`、`drone_dynamics`、`drone_controller` 和 `drone_bringup`，四个 package 均通过 `colcon build --symlink-install`；
* `drone_msgs/msg/MotorRPM` 已成功生成，字段明确对应 M1～M4、电机位置和旋转方向；
* 动力学与控制器采用算法类和 ROS2 节点分离结构；
* `basic_sim.launch.py` 已实际启动动力学节点和控制器骨架，节点名、订阅、发布和 Topic 类型均与当前接口一致；
* Launch 经 Ctrl-C 停止后，两个节点均正常退出。
* VS Code C/C++ 索引已配置为读取控制器和动力学 package 的真实 CMake 编译数据库；两个 C++ package 会在普通 colcon 构建时自动导出 `compile_commands.json`。
* `QuadrotorModel` 已实现位置、世界系速度、姿态四元数、机体系角速度和四电机实际转速状态；
* 已实现 RPM 限幅与转换、电机一阶响应、单桨推力与反扭矩、X 型三轴力矩、平动方程、刚体转动方程、姿态积分和归一化；
* 六项 GTest 全部通过，覆盖要求的五类动力学场景以及电机限幅/一阶响应；
* 动力学节点已实际发布 `/drone/odom`、`/drone/imu`、`/drone/path` 和 `map -> base_link` TF；
* Odom 实测约 200 Hz，Path 实测约 20 Hz；Topic 输入的零 RPM、对称推力、roll、pitch 和 yaw 响应方向均通过检查。
* 已创建与 0.20 m X 型机臂约定一致的基础 Xacro 模型，并由 robot_state_publisher 发布 `base_link` 到固定子链接；
* RViz2 已实际启动并显示 RobotModel、TF、Path、Pose、map/base_link Axes 和 Grid；
* 12000 RPM 竖直运动时，模型状态与 Path 由同一 `map -> base_link`/Odom 状态驱动；短时 roll 输入后四元数和 base_link 坐标轴均发生对应变化。
* `QuadrotorModel` 已实现可选的简化水平地面约束；正常 Launch 在 `ground_z=0` 启用，零 RPM 保持地面静止，高推力可正常离地；
* 地面关闭自由落体、地面静止、起飞、空中落地及水平速度不受摩擦影响的测试均已通过；地面静止时世界系实际加速度为 0，IMU 机体系比力为 +g。
* `MotorMixer` 已作为与 ROS2 无关的纯算法类实现，可将总推力和机体系 roll、pitch、yaw 力矩反解为固定顺序 `[M1,M2,M3,M4]` 的目标 RPM；
* Mixer 的零输入、悬停推力、三轴单独力矩、混合指令往返、饱和、非有限/极大有限输入和非法参数等 11 项 GTest 已全部通过；完整工作空间重新构建通过。
* `AttitudeController` 已作为与 ROS2 无关的纯算法类实现，输入期望/当前 body-to-world 四元数及机体系角速度，输出 base_link 中的 roll、pitch、yaw 力矩；
* 姿态误差符号、角速度阻尼、四元数最短路径与归一化、逐轴力矩限幅、非法输入和非法参数等 10 项 GTest 已全部通过；Mixer 极大有限输入补丁测试也已通过。

详细命令、结果和验证边界见“验证记录”。以上工程初始化结果不代表动力学、控制器或可视化功能已经实现。

只有同时满足以下条件的内容才能写入本节：

1. 已经完成代码；
2. 已经成功编译；
3. 已经实际运行；
4. 已按照明确场景进行验证；
5. 结果符合预期。

不得将“代码已经生成”写成“功能已经完成”。

## 5. 当前问题

当前没有发现阻塞后续开发的环境问题。

尚未确认的事项：

* 基础 RViz2 显示已验证；不同屏幕尺寸下的默认视角、模型大小和配色仍建议由用户人工确认并按演示需要微调；
* Mixer 和姿态/角速度控制器均尚未接入 `position_controller_node`；高度、位置控制律及控制器电机指令发布仍未实现；
* 当前只有质心 z 方向的简化刚性地面约束，没有反弹、摩擦、起落架弹性、姿态约束或复杂碰撞形状；
* 当前模型没有空气阻力、旋翼陀螺效应或传感器噪声，持续不对称力矩会使角速度不断增加；
* 已验证短时固定步长响应，尚未验证长时间、高角速度或最大 RPM 下的数值稳定性；
* 节点保持最后一次 RPM 命令，尚未实现命令超时归零机制；
* 项目许可证尚未确定，因此四个 package 的 `<license>` 当前保留为 `TODO`；
* `.idea/` 当前没有忽略规则；只有实际使用 JetBrains IDE 并产生该目录时才需要决定是否补充。

记录问题时应包含：

```text
问题现象：
复现步骤：
相关节点或文件：
报错信息：
已经尝试的方法：
当前判断：
下一步检查：
```

问题解决后，应删除过时描述，或将关键结论移入“技术决策”与“验证记录”。

## 6. 关键技术决策

### 开发环境

* 操作系统：Ubuntu 22.04；
* ROS2：Humble；
* 核心动力学和控制器：C++17；
* 数学运算：Eigen3；
* 可视化：RViz2；
* 构建系统：colcon + ament_cmake。

### 工程设计

* 算法类与 ROS2 节点分离；
* 参数统一通过 YAML 文件配置；
* Launch、RViz 和 URDF 集中放在 `drone_bringup`；
* 地图和规划分别使用独立功能包；
* 不使用 Gazebo 作为动力学计算核心；
* RViz2 仅负责显示，实际运动由动力学节点计算。
* `map -> base_link` 只由 `quadrotor_dynamics_node` 动态发布；robot_state_publisher 的 URDF 根链接是 `base_link`，只发布到固定子链接的静态 TF；
* `/drone/goal` 统一使用 `geometry_msgs/msg/PoseStamped`，供控制器骨架和 RViz Pose 显示共同订阅。
* 地面约束集中在 `QuadrotorModel`，ROS2 消息发布层不修改模型位置；核心模型默认关闭地面，正常 Launch 通过 YAML 默认开启。
* Motor Mixer 是 `drone_controller` 中不依赖 ROS2 的纯算法组件；其参数独立保存，不让 `drone_controller` 依赖 `drone_dynamics`，但 `arm_length`、`k_F`、`k_M` 和 RPM 范围必须与动力学配置保持一致；
* Mixer 先在单电机推力空间反解，再转换为 rad/s 和 RPM。当前采用逐电机限幅，饱和后的实际 Wrench 可能不再等于请求值，后续控制器必须处理 `saturated` 标志。
* 姿态四元数继续使用 `orientation_body_to_world`（base_link 向量旋转到 map）；姿态误差为 `q_current.conjugate()*q_desired`，若误差四元数 `w<0` 则整体反号以选择最短路径；
* 姿态力矩使用 `Kp .* (2*q_error.vec) + Kd .* (omega_desired-omega_current)`。该阻尼写法保证当前正角速度在期望角速度为零时产生负力矩，与动力学 base_link 三轴正力矩约定一致。

### 动力学参数基线

当前参数全部由 `drone_bringup/config/dynamics.yaml` 提供，采用 SI 单位：

* 质量：`1.0 kg`；
* 主惯量：`Ixx=0.02`、`Iyy=0.02`、`Izz=0.04 kg·m²`；
* 机臂长度（质心到电机）：`0.20 m`；
* 推力系数：`k_F=1.91e-6 N/(rad/s)²`；
* 反扭矩系数：`k_M=2.60e-7 N·m/(rad/s)²`；
* 电机时间常数：`0.05 s`；
* RPM 范围：`0～20000 RPM`；
* 重力加速度：`9.80665 m/s²`；
* 核心模型地面约束默认值：`enable_ground_contact=false`、`ground_z=0.0 m`；
* 正常 Launch 参数：`enable_ground_contact=true`、`ground_z=0.0 m`；
* 仿真频率：`200 Hz`，固定步长 `dt=0.005 s`；
* Path 每 10 个仿真步发布一次，即名义 `20 Hz`，最多保留 2000 个点。

按以上参数计算的稳态名义悬停转速约为 `10818.9 RPM/电机`。这是开环稳态推力平衡点，不会自动消除电机启动阶段已经产生的速度误差。

### 固定坐标、电机和单位约定

世界坐标系采用 ENU：

* `x`：前方；
* `y`：左方；
* `z`：上方。

机体系 `base_link` 采用 FLU：

* `x`：前方；
* `y`：左方；
* `z`：上方。

重力沿世界坐标系负 `z` 方向。无人机采用 X 型四旋翼布局，俯视无人机时电机约定为：

```text
              +x（前）
       M1（前左）   M4（前右）
          CCW          CW

       M2（后左）   M3（后右）
           CW         CCW
              -x（后）

     +y（左）          -y（右）
```

固定电机编号和旋转方向：

* M1：前左，CCW；
* M2：后左，CW；
* M3：后右，CCW；
* M4：前右，CW。

ROS2 外部接口使用 RPM 表示电机转速，动力学内部统一转换为 rad/s。所有物理量统一使用 SI 单位制。

以上约定是后续动力学、控制器、Mixer、URDF 和可视化共同遵循的固定基础，不得擅自修改。Mixer 已按动力学正向矩阵的严格逆运算实现并通过往返验证。

如修改坐标或电机约定，必须同步更新：

* 动力学模型；
* 控制器；
* Mixer；
* URDF；
* RViz2；
* README；
* 本文件。

## 7. 当前系统接口

以下动力学接口已经实现并经过运行检查。

### 动力学节点

节点名称：

```text
quadrotor_dynamics_node
```

订阅：

```text
/drone/motor_rpm_cmd
```

发布：

```text
/drone/odom  (nav_msgs/msg/Odometry)
/drone/imu   (sensor_msgs/msg/Imu)
/drone/path  (nav_msgs/msg/Path)
/tf          (map -> base_link)
```

Odom 的位姿位于 `map`，`child_frame_id` 为 `base_link`；twist 使用机体系线速度和角速度。IMU orientation 为 `base_link` 相对 `map` 的姿态，linear acceleration 发布机体系比力，因此零 RPM 自由落体时为零。

### 控制器节点

节点名称：

```text
position_controller_node
```

订阅：

```text
/drone/goal  (geometry_msgs/msg/PoseStamped)
/drone/odom
```

发布：

```text
/drone/motor_rpm_cmd
```

当前仅创建电机 RPM 发布器，尚未发布控制指令。

### 可视化节点

`basic_sim.launch.py` 默认启动：

```text
/robot_state_publisher
/rviz2
```

robot_state_publisher 从 `drone.urdf.xacro` 生成的 robot_description 发布 `base_link` 到机臂、电机、旋翼和机头标记的固定 TF。RViz Fixed Frame 为 `map`，订阅 `/robot_description`、`/drone/path` 和 `/drone/goal`。可通过 `use_rviz:=false` 关闭图形界面。

### 规划节点

后续计划订阅：

```text
/drone/goal
/drone/odom
/map/obstacles
```

后续计划发布：

```text
/drone/reference
/drone/planned_path
```

地图和规划接口仍是后续规划，当前没有创建对应 package 或节点。

## 8. 核心物理关系

动力学主链路：

```text
目标 RPM
→ 电机一阶响应
→ 实际角速度
→ 单电机推力与反扭矩
→ 总推力与三轴力矩
→ 线加速度与角加速度
→ 速度、位置、姿态和角速度
```

核心推力模型：

```text
F_i = k_F * omega_i^2
```

电机指令和一阶响应：

```text
omega_cmd = clamp(RPM, RPM_min, RPM_max) * 2*pi/60
omega_next = omega + (1 - exp(-dt/tau_m)) * (omega_cmd - omega)
Q_i = k_M * omega_i^2
```

设 `a = arm_length/sqrt(2)`，按照 M1 前左、M2 后左、M3 后右、M4 前右，有：

```text
T     = F1 + F2 + F3 + F4
tau_x = a * ( F1 + F2 - F3 - F4)
tau_y = a * (-F1 + F2 + F3 - F4)
tau_z =     (-Q1 + Q2 - Q3 + Q4)
```

其中 `tau_x`、`tau_y` 直接由各电机位置的 `r_i × [0,0,F_i]` 得到。M1/M3 为 CCW，旋翼对机体的反作用力矩为负 z；M2/M4 为 CW，反作用力矩为正 z。

平动方程：

```text
p_dot = v
v_dot = R(q) * [0, 0, T]^T / m + [0, 0, -g]^T
```

核心转动方程：

```text
I * omega_dot = tau - omega × (I * omega)
```

速度、位置和机体系角速度采用固定步长半隐式 Euler。姿态使用机体系角速度构造增量旋转四元数，右乘到 `base_link -> map` 四元数并在每步归一化。

所有角速度单位、RPM 与 rad/s 的换算、力矩方向和电机旋转方向必须经过人工确认。

## 9. 验证记录

### 2026-07-10：开发环境与空工作空间检查

测试目标：确认当前 Ubuntu 22.04 + ROS2 Humble 环境是否满足项目初始化和后续 C++ ROS2 开发需要，不创建功能包、不安装或删除软件。

主要检查命令：

```bash
. /etc/os-release
command -v ros2 colcon rosdep g++ cmake
source /opt/ros/humble/setup.bash
ros2 pkg prefix ament_cmake
ros2 pkg prefix rviz2
ros2 pkg prefix tf2
ros2 pkg prefix tf2_ros
ros2 pkg prefix std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs
ros2 pkg executables rviz2
ros2 pkg executables tf2_ros
rosdep --version
rosdep db
rosdep check --from-paths src --ignore-src
g++ --version
cmake --version
dpkg-query -W libeigen3-dev ros-humble-desktop ros-humble-ament-cmake
timeout 4s ros2 run demo_nodes_cpp talker
colcon list
colcon build --symlink-install
git status --short --branch
git check-ignore -v build install log results report .vscode .idea
```

另外建立了一个临时的非 ROS2 功能包 CMake 检查工程，使用以下依赖完成配置、C++17 编译、链接和运行；检查后已删除临时源码和生成物：

```cmake
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
```

已确认正常：

* 操作系统为 Ubuntu 22.04.5 LTS（Jammy，x86_64）；
* `/opt/ros/humble/setup.bash` 存在，加载后 `ROS_DISTRO=humble`、`ROS_VERSION=2`、`ROS_PYTHON_VERSION=3`；
* ROS2 C++ 示例 `demo_nodes_cpp talker` 能启动并连续发布消息，4 秒后由 `timeout` 主动结束；
* `colcon-core 0.21.0`、`colcon-common-extensions 0.3.0`、`rosdep 0.26.0` 可用；`rosdep db` 能读取 Ubuntu Jammy 规则库；
* `ament_cmake 1.3.14` 能被标准 CMake 工程发现并参与构建；
* GNU g++ 11.4.0 和 CMake 3.22.1 可用，临时工程以 C++17 成功编译、链接并运行；
* Eigen3 开发包 3.4.0 已安装，实际 Eigen 向量程序编译和运行成功；
* RViz2 11.2.27 已安装，`rviz2` 可执行文件存在，`rviz2 --help` 正常返回；
* tf2、tf2_ros、rclcpp、std_msgs、geometry_msgs、sensor_msgs、nav_msgs 和 visualization_msgs 均能被 ROS2/CMake 找到并完成临时工程构建；
* `src/` 当前包含 0 个 `package.xml`，`colcon list` 返回 0 个功能包；空工作空间构建成功，结果为 `Summary: 0 packages finished`；
* Git 工作树有效，分支为 `main`；`.gitignore` 已正确忽略 `build/`、`install/`、`log/`、`.vscode/`、Python 缓存、临时文件和 ROS bag 常见文件；
* `results/` 与 `report/` 未被忽略，与保存实验结果和报告的项目规划一致，当前 `.gitignore` 对初始化阶段基本合理。

尚未确认：

* 未实际启动 RViz2 图形窗口，因此未验证图形渲染、项目 RViz 配置或显示插件；
* `rosdep check` 返回依赖满足，但当前没有功能包和 `package.xml`，该结果只确认命令及本地规则库可用，不能验证项目依赖声明；
* 没有功能包可供编译和测试，未验证任何项目节点、Topic、TF、Launch 或运行行为；
* Git 仓库尚无提交，README、文档和 `.gitignore` 都处于未跟踪状态。

检查结论：开发环境满足开始创建 ROS2 C++ 功能包的基础条件；本次未创建功能包，未安装或删除软件。

### 2026-07-10：ROS2 工程初始化检查

测试目标：创建四个基础 package、自定义电机 RPM 消息、动力学与控制器节点骨架和基础 Launch，并验证构建及 ROS 图；不实现完整算法。

实际执行命令：

```bash
source /opt/ros/humble/setup.bash
colcon list
rosdep check --from-paths src --ignore-src
colcon build --symlink-install
source install/setup.bash
ros2 interface show drone_msgs/msg/MotorRPM
ros2 launch drone_bringup basic_sim.launch.py
ros2 node list
ros2 topic list
ros2 node info /quadrotor_dynamics_node
ros2 node info /position_controller_node
ros2 topic type /drone/goal
ros2 topic type /drone/odom
ros2 topic type /drone/motor_rpm_cmd
```

实际结果：

* `colcon list` 发现 `drone_msgs`、`drone_dynamics`、`drone_controller` 和 `drone_bringup` 四个 `ament_cmake` package；
* `rosdep check --from-paths src --ignore-src` 返回 `All system dependencies have been satisfied`；
* `colcon build --symlink-install` 成功，结果为 `Summary: 4 packages finished`；
* `ros2 interface show drone_msgs/msg/MotorRPM` 成功，生成了以下四个 `float64` 字段：

```text
m1_front_left_ccw_rpm
m2_rear_left_cw_rpm
m3_rear_right_ccw_rpm
m4_front_right_cw_rpm
```

* `basic_sim.launch.py` 成功启动 `/quadrotor_dynamics_node` 和 `/position_controller_node`；
* `ros2 topic list` 包含 `/drone/goal`、`/drone/odom` 和 `/drone/motor_rpm_cmd`；
* `/drone/goal` 类型为 `geometry_msgs/msg/PointStamped`；
* `/drone/odom` 类型为 `nav_msgs/msg/Odometry`；
* `/drone/motor_rpm_cmd` 类型为 `drone_msgs/msg/MotorRPM`；
* 动力学节点订阅 `/drone/motor_rpm_cmd` 并注册 `/drone/odom` 发布器；
* 控制器节点订阅 `/drone/goal`、`/drone/odom` 并注册 `/drone/motor_rpm_cmd` 发布器；
* Ctrl-C 停止 Launch 后，两个节点进程均正常退出；
* 未发现阻塞编译或启动的依赖、CMake 或 package 结构问题。

验证边界：

* `QuadrotorDynamics` 当前只负责将四个外部 RPM 输入转换为内部 rad/s，没有状态量、受力、力矩或积分公式；
* `PositionController` 当前只接收并保存目标点和里程计输入；姿态控制器与 Mixer 只保留独立算法位置；
* 两个节点只注册规定的 ROS2 接口，不发布伪造的里程计或电机指令；
* 尚未实现 TF、IMU、Path、参数 YAML、URDF、RViz 配置、动力学测试或控制器测试；
* package 许可证字段仍为 `TODO`，需要项目维护者确定许可证后统一修改。

是否通过：工程初始化、消息生成、编译、基础 Launch、节点与 Topic 图检查通过；动力学和控制算法不在本次验收范围内，仍未实现。

### 2026-07-10：VS Code C++ 索引修复

问题现象：`position_controller_node.cpp` 中项目头文件、自定义消息和 ROS2 头文件被 IDE 标红，但 `colcon build` 实际成功。

原因：VS Code C/C++ 扩展没有编译数据库，也没有项目级索引配置，因此不知道 ament 为每个编译目标注入的 include 路径和编译参数。

处理：

* 在 `drone_controller` 和 `drone_dynamics` 的 CMake 配置中启用 `CMAKE_EXPORT_COMPILE_COMMANDS`；
* 新增 `.vscode/settings.json`，同时引用两个 package 的编译数据库；
* 指定 `/usr/bin/g++` 和 C++17；
* 调整 `.gitignore`，保留共享的 `.vscode/settings.json`，继续忽略其他 VS Code 本地状态。

验证结果：

* `build/drone_controller/compile_commands.json` 包含控制器算法和节点两个源文件；
* `build/drone_dynamics/compile_commands.json` 包含动力学算法和节点两个源文件；
* `position_controller_node.cpp` 的真实编译命令包含项目 include 目录、生成的 `drone_msgs` include 目录、ROS2 include 目录和 `-std=c++17`；
* 工作区配置 JSON 语法检查通过；
* 完整工作空间仍可通过 `colcon build --symlink-install` 构建。

使用提示：配置变更后，在 VS Code 命令面板执行 `C/C++: Reset IntelliSense Database` 和 `Developer: Reload Window`，使当前编辑器进程立即重建索引。

### 2026-07-10：四旋翼动力学第一阶段验证

测试目标：验证四电机 RPM 驱动的六自由度刚体模型、电机响应、X 型力矩符号、ROS2 状态输出和 TF，不使用控制器生成 RPM。

构建与算法测试命令：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
colcon test --packages-select drone_dynamics --event-handlers console_direct+
colcon test-result --verbose
```

结果：四个 package 构建成功；`drone_dynamics` 的 6 个 GTest 全部通过，`colcon test-result` 为 0 errors、0 failures。

算法级场景结果：

1. 零 RPM，仿真 1.0 s：`z=-4.95236 m`、`vz=-9.80665 m/s`、角速度范数 0、四元数范数 1，通过；
2. 四电机对称：名义悬停转速 `10818.9 RPM`；0.8 倍悬停转速时 `az=-3.53039 m/s²`，悬停转速时 `az≈1.78e-15 m/s²`，1.2 倍时 `az=+4.31493 m/s²`，三轴力矩范数 0，通过；
3. roll 输入 `[11000,11000,9000,9000] RPM`：`tau=[+0.236971,0,0] N·m`，只产生正 roll 响应，通过；
4. pitch 输入 `[9000,11000,11000,9000] RPM`：`tau=[0,+0.236971,0] N·m`，只产生正 pitch 响应，通过；
5. yaw 输入 `[9000,10908.7,9000,10908.7] RPM`：`tau_z=+0.216693 N·m`，相对四电机 `10000 RPM` 的总推力差为 0，通过；
6. 电机响应与限幅：M1 的负 RPM 被限制为 0；M2 的 `20000 RPM` 命令按 `15000 RPM` 上限和 `tau_m=0.05 s` 响应，一时间常数后的实际转速为 `992.933 rad/s`，通过。

ROS2 运行验证使用隔离域，避免默认 domain 中已有的旧 Launch 进程干扰：

```bash
export ROS_DOMAIN_ID=77
ros2 launch drone_bringup basic_sim.launch.py
ros2 topic echo /drone/odom --once
ros2 topic echo /drone/imu --once
ros2 topic hz /drone/odom
ros2 run tf2_ros tf2_echo map base_link
python3 tools/dynamics_probe.py M1 M2 M3 M4 --settle 0.30 --duration 0.30
```

运行级结果：

* 隔离域中只有 `/quadrotor_dynamics_node` 和仍为骨架的 `/position_controller_node`；
* `/drone/odom`、`/drone/imu`、`/drone/path` 和 `/tf` 类型分别正确；
* Odom 实测平均约 `199.96 Hz`，Path 实测约 `19.99 Hz`；
* `tf2_echo map base_link` 成功读取平移和单位四元数；
* 零 RPM 的约 0.31 s 观测窗口中 `delta_vz=-3.04006 m/s`，三轴角速度为 0，四元数范数 1；
* 低对称 RPM `8655.12` 时 `delta_vz=-1.06061 m/s`；名义悬停 RPM 时 `delta_vz=-0.00211 m/s`；高对称 RPM `12982.68` 时 `delta_vz=+1.26955 m/s`；
* 高 RPM 命令在节点启动前已持续发布时，1.0 s 稳定后 0.30 s 窗口得到 `delta_z=+1.15367 m`、`delta_vz=+1.27287 m/s`，确认上升；
* roll 场景最终主轴角速度 `wx=+0.92020 rad/s`，其他轴为 0；
* pitch 场景最终主轴角速度 `wy=+0.97369 rad/s`，非目标轴残差约 `1e-17`；
* yaw 场景最终主轴角速度 `wz=+0.42073 rad/s`，非目标轴残差约 `1e-17`；
* 所有运行场景的四元数范数均为 1。

坐标、单位和数值结论：ENU/FLU 与力矩方向没有发现冲突，所有计算使用 SI 单位。名义悬停 RPM 只保证电机稳定后的净加速度接近零；若节点先以零 RPM 自由落体，之后施加悬停 RPM 不会消除已有下降速度。短时 200 Hz 仿真稳定，长时间高角速度稳定性尚未验证。

是否通过：本阶段要求的动力学算法、五类响应、ROS2 输出和 TF 均已实际通过；控制器、地面碰撞、气动阻力、URDF/RViz 和长时间极限工况不属于本次已验证内容。

### 2026-07-10：默认 ROS domain 重复节点隐患复核

先前动力学验证时，默认 ROS domain 中存在另一个终端遗留的旧 `basic_sim.launch.py`，因此当时使用 `ROS_DOMAIN_ID=77` 隔离测试。用户关闭旧进程后完成以下复核：

* 启动前未发现残留 Launch、动力学或控制器进程；
* 默认 domain 的 `ros2 node list` 和应用 Topic 列表均为空；
* 在默认 domain 单次启动 `basic_sim.launch.py` 后，只出现一个 `/quadrotor_dynamics_node` 和一个 `/position_controller_node`，重复节点检查结果为 `none`；
* `/drone/motor_rpm_cmd` 为 1 publisher + 1 subscriber；
* `/drone/odom` 为 1 publisher + 1 subscriber；
* `/drone/goal` 为 0 publisher + 1 subscriber；
* `/drone/imu`、`/drone/path` 和 `/tf` 均为 1 publisher；
* Ctrl-C 后两个节点均正常退出，停止后默认 domain 再次为空。

结论：同名节点和重复 Topic 是旧 Launch 进程并行运行造成的外部运行状态问题，不是 package、Launch 文件或节点命名缺陷；旧进程关闭后隐患已经消除，不需要修改节点或 Topic 名称。

### 2026-07-12：基础 URDF 与 RViz2 可视化验证

测试目标：不修改动力学公式和电机约定，为现有 `map -> base_link` 状态增加简化四旋翼模型、固定子链接 TF、轨迹、目标点、坐标轴和网格显示。

实现内容：

* `drone.urdf.xacro` 使用基础几何体创建 `base_link`、红色机头标记、4 条 X 型机臂、4 个电机和 4 个旋翼；
* 电机中心到质心距离为 `0.20 m`，位置与 M1 前左、M2 后左、M3 后右、M4 前右一致；M1/M3 旋翼为蓝色，M2/M4 为橙色；
* 所有关节均为固定关节，robot_state_publisher 只发布 `base_link` 到固定子链接；
* `drone_sim.rviz` 配置 Fixed Frame=`map`，包含 Grid、RobotModel、TF、map/base_link Axes、`/drone/path` Path 和 `/drone/goal` Pose；
* `/drone/goal` 从 `PointStamped` 统一调整为 `PoseStamped`，控制器仍只有输入保存骨架，没有新增控制算法；
* `basic_sim.launch.py` 增加 robot_state_publisher、RViz2 和默认 true 的 `use_rviz` 参数。

实际命令：

```bash
source /opt/ros/humble/setup.bash
xacro src/drone_bringup/urdf/drone.urdf.xacro > /tmp/drone.urdf
check_urdf /tmp/drone.urdf
colcon build --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
ros2 launch drone_bringup basic_sim.launch.py
ros2 node list
ros2 topic list
ros2 run tf2_ros tf2_echo map base_link
```

结构与启动结果：

* Xacro 和安装后的 Xacro 均通过 `check_urdf`，根链接为 `base_link`；
* 四个 package 构建成功，已有测试结果保持 `0 errors, 0 failures`；
* `/quadrotor_dynamics_node`、`/position_controller_node`、`/robot_state_publisher` 和 `/rviz2` 均实际启动；
* RViz2 使用 OpenGL 4.6 启动，Global Status 和各显示项状态为 OK；
* `/tf` 的动态消息实测只包含 `map -> base_link`；`/tf_static` 包含 `base_link` 到机头、机臂、电机和旋翼的固定变换，没有重复发布 `map -> base_link`；
* `base_link -> m1_rotor_link` 实测平移为 `[0.141, 0.141, 0.025] m`，与 0.20 m X 型机臂一致；
* `use_rviz:=false` 可只启动动力学、控制器骨架和 robot_state_publisher。

三项可视化测试：

1. 目标点：发布 `PoseStamped` `(2.0,1.0,1.5)` 后，RViz Goal Pose 箭头在 map 网格中实际可见；RViz 和控制器骨架均以 `PoseStamped` 订阅该 Topic，通过；
2. 竖直运动：四电机持续 `12000 RPM` 时，0.30 s 运行窗口得到 `delta_vz=+0.66610 m/s`；RViz 中模型沿 z 运动并形成竖直 Path，RobotModel、TF 和 Path 无状态错误，通过；
3. 姿态变化：短时左高右低 roll 输入后，Odom 四元数变为 `x=0.92166,w=0.38799`，其他旋转分量为 0；base_link 坐标轴和模型随 TF 发生 roll 方向变化，不是只发生平移，通过。

人工确认项：不同显示器和窗口尺寸下，建议用户确认默认视距 `3.8 m`、模型配色、红色机头辨识度及长轨迹场景是否符合最终演示偏好；这些属于外观微调，不影响已验证的 TF 和消息链路。

验证边界：RViz2 只显示状态，不参与动力学；控制器、Mixer、地图、规划和避障仍未实现；持续不对称 RPM 且没有角阻尼时，模型会持续旋转并可能离开默认 map 视野，测试时应重启 Launch 或调整 RViz Target Frame。

是否通过：本阶段三个验收重点——模型显示、TF 驱动位置/姿态、轨迹与目标点显示——均已实际通过。

### 2026-07-13：简化地面接触与 SetGoal 修正

实现范围：

* `QuadrotorParameters` 新增 `enable_ground_contact` 和 `ground_z`；核心模型默认分别为 `false`、`0.0 m`，正常 Launch 的 YAML 配置为 `true`、`0.0 m`；
* 地面约束集中在 `QuadrotorModel::apply_ground_contact_constraint()`，积分后只夹紧低于地面的 `position_world.z`，只清除负的 `velocity_world.z`；
* 正向 z 速度和正向离地加速度不被清零；x/y 位置和速度不受地面约束影响；
* `reset()` 在地面启用时把初始 z 设置为 `ground_z`，否则保持世界原点；
* 约束后用实际速度变化除以 dt 回算 `linear_acceleration_world_`；IMU 比力统一由实际世界系加速度减去重力后旋回机体系计算；
* RViz SetGoal 工具 Topic 从 `/goal_pose` 修正为 `/drone/goal`，类型保持 `geometry_msgs/msg/PoseStamped`。

单元测试：

```bash
colcon test --packages-select drone_dynamics --event-handlers console_direct+
colcon test-result --verbose
```

结果为 11 个 GTest 全部通过，`0 errors, 0 failures`：

* 地面关闭、零 RPM 1 s：`z=-4.95236 m`、`vz=-9.80665 m/s`、IMU 比力 0，原自由落体行为保持；
* 地面开启、零 RPM 1 s：`z=0`、`vz=0`、`az=0`、IMU z 比力 `9.80665 m/s²`；
* 地面开启、12000 RPM 2 s：`z=3.81592 m`、`vz=4.14397 m/s`，可以离地；
* 从空中零 RPM 落到 `ground_z=0.35 m`：最终 `z=0.35`、`vz=0`、`az=0`，整个过程中未穿透；
* 带水平速度落地：水平速度在接触时和地面继续运行后都为 `0.221007 m/s`，确认没有错误清零 x/y 速度；
* 非有限 `ground_z`（NaN）会在模型参数校验阶段被拒绝；
* 原有对称推力、roll、pitch、yaw、电机响应与限幅测试继续通过。

正常 Launch 运行验证：

1. 无 RPM 等待超过 3 s：参数读取为 `enable_ground_contact=true`、`ground_z=0.0`；Odom `z=0,vz=0`，IMU z=`9.80665`；实际 RViz 中模型保持在地面，通过；
2. 持续 `10818.95 RPM` 3 s：`z=0.000171 m`、`vz=3.67e-05 m/s`、IMU z=`9.806653`；电机响应阶段未下沉，稳态保持在地面附近，通过；
3. 从地面持续 `12000 RPM`：模型正常离地，采样时 Odom `z=161.57 m,vz=29.12 m/s`，Path 和 `map -> base_link` TF 同步上升，通过。持续开环推力且没有空气阻力，因此高度和速度会继续增长；
4. 运行中的 `/rviz2` 节点实际创建 `/drone/goal` 的 `PoseStamped` publisher；控制器骨架和 RViz Goal Pose 各有一个同类型 subscriber，安装后的 RViz 配置不再包含 `/goal_pose`。

人工确认项：SetGoal 发布端点和类型已经实际确认，用户仍需在 RViz 中点击 SetGoal 并选择一个具体位置，再用 `ros2 topic echo /drone/goal --once` 确认所选坐标符合预期；本次未自动化鼠标点击，因此不把具体点击坐标写成已验证。

限制：当前是质心 z 方向的无反弹、无摩擦刚性地面，不包含碰撞几何、起落架弹性、地面姿态约束或水平阻力。倾斜机体在地面上仍可转动，符合本阶段明确的简化边界。

是否通过：地面约束、加速度/IMU 一致性、起飞、落地、不清除水平速度及 SetGoal Topic 修正均通过代码和运行检查；RViz 鼠标选择的具体目标坐标待用户人工操作确认。

### 2026-07-13：Motor Mixer 独立实现与验证

实现内容：

* 在 `drone_controller` 中实现与 ROS2 无关的 `MotorMixer`，输入为总推力 `T` 和机体系三轴力矩，输出顺序固定为 `[M1,M2,M3,M4]` 的目标 RPM；
* Mixer 参数默认使用 `arm_length=0.20 m`、`k_F=1.91e-6`、`k_M=2.60e-7`、`0～20000 RPM`，必须与动力学参数保持一致；
* 令 `a=arm_length/sqrt(2)`、`b=k_M/k_F`，反解为：
  `F1=(T+tx/a-ty/a-tz/b)/4`、`F2=(T+tx/a+ty/a+tz/b)/4`、`F3=(T-tx/a+ty/a-tz/b)/4`、`F4=(T-tx/a-ty/a+tz/b)/4`；
* 负总推力、负单电机推力及超范围 RPM 使用逐电机安全限幅并设置 `saturated=true`；非有限 Wrench 返回四电机 0 RPM、`valid=false`、`saturated=true`；
* Mixer 编译为独立的 `motor_mixer` 库，供后续控制器链接，但本阶段没有接入 `position_controller_node`。

实际命令：

```bash
colcon build --symlink-install --packages-select drone_controller \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon test --packages-select drone_controller --event-handlers console_direct+
colcon test-result --verbose
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

实际结果：Mixer 的 10 个 GTest 全部通过，覆盖零输入、纯悬停推力、正 roll/pitch/yaw、混合指令正反向往返、不可实现指令、负总推力、NaN/Inf 输入和非法参数。控制器 package 定向构建成功，随后四个 package 的完整工作空间构建成功。`colcon test-result` 显示工作区累计 `23 tests, 0 errors, 0 failures, 0 skipped`；其中本次 Mixer 测试为 10 项，其余是先前保留的动力学测试结果。

验证边界：只验证了纯算法控制分配；没有修改动力学代码，没有让 ROS2 节点调用 Mixer，也没有实现姿态 PID、高度控制、位置控制或闭环飞行。

### 2026-07-13：姿态/角速度控制器独立实现与 Mixer 健壮性补丁

实现内容：

* `AttitudeController` 编译为独立库，输入期望/当前 `orientation_body_to_world` 四元数和期望/当前机体系角速度，输出 base_link 中 `[roll,pitch,yaw]` 力矩及 `valid`、`saturated`；
* 有效非单位四元数会先稳定归一化，零范数或含 NaN/Inf 的四元数以及非有限角速度返回零力矩和 `valid=false`；
* 使用 `q_error=q_current.conjugate()*q_desired`，并在 `q_error.w()<0` 时整体反号；姿态误差为 `2*q_error.vec()`；
* 控制律为 `torque=Kp.*attitude_error+Kd.*(omega_desired-omega_current)`，逐轴限制到 `[-max_torque,+max_torque]`。这里使用加号是为了让正当前角速度产生负阻尼力矩；它等价于 `-Kd.*(omega_current-omega_desired)`；
* 默认参数为 `Kp=[4,4,2]`、`Kd=[0.2,0.2,0.1]`、`max_torque=[1,1,0.5] N*m`；增益必须有限且非负，力矩上限必须有限且为正；
* Mixer 补充 roll/pitch/yaw 中间项、四个电机推力、omega 和 RPM 的有限性检查；任何中间非有限结果安全返回四电机零 RPM、`valid=false`、`saturated=true`。

实际结果：`drone_controller` 定向构建成功；Mixer 11 个 GTest 和姿态控制器 10 个 GTest 全部通过。工作区测试汇总为 `35 tests, 0 errors, 0 failures, 0 skipped`，随后四个 package 的完整工作空间构建成功。极大有限 Wrench（`numeric_limits<double>::max()`）会被识别为中间结果溢出并返回有限的四电机零 RPM。

验证边界：姿态控制器与 Mixer 均未接入 `position_controller_node`，没有发布 RPM；没有修改 `drone_dynamics`，没有实现高度、位置或闭环飞行。

后续每次测试应使用以下格式：

```text
测试名称：
测试目标：
启动命令：
输入条件：
预期结果：
实际结果：
关键数据：
是否通过：
相关结果文件：
```

例如：

```text
测试名称：零转速自由落体测试
测试目标：验证重力方向和位置积分
输入条件：四电机 RPM 均为 0
预期结果：z 方向速度逐渐减小，无人机向下运动
是否通过：待测试
```

## 10. 常用命令

### 编译

```bash
cd ~/ros2_drone_sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 启动基础系统

```bash
ros2 launch drone_bringup basic_sim.launch.py
```

当前 Launch 启动已实现的动力学节点和控制器骨架。动力学节点会产生状态，控制器仍不产生有效 RPM 输出。

无界面启动：

```bash
ros2 launch drone_bringup basic_sim.launch.py use_rviz:=false
```

### 动力学算法测试

```bash
colcon test --packages-select drone_dynamics --event-handlers console_direct+
colcon test-result --verbose
```

### Mixer 算法测试

```bash
colcon test --packages-select drone_controller --event-handlers console_direct+
colcon test-result --verbose
```

### 查看节点

```bash
ros2 node list
```

### 查看 Topic

```bash
ros2 topic list
ros2 topic info /drone/odom
ros2 topic echo /drone/odom
ros2 topic hz /drone/odom
```

### 查看 TF

```bash
ros2 run tf2_ros tf2_echo map base_link
```

具体启动命令将在对应 Launch 文件实现后补充。

## 11. 下一步任务

当前优先级：

1. 提交已验证的动力学、简化地面、URDF、RViz、Mixer、姿态控制器和文档变更；
2. 增加长时间、高角速度和最大 RPM 数值稳定性测试；
3. 独立设计并测试高度控制器；
4. 控制算法稳定后，再将其与 Mixer 接入 ROS2 控制器节点；
5. 最后进行悬停闭环测试，暂不进入地图或规划。

在以上任务完成并验证前，不开始完整位置控制器、地图或避障模块。

## 12. AI 工作规则

AI 在开始编程前必须：

1. 阅读根目录 `README.md`；
2. 阅读 `docs/AI_CONTEXT.md`；
3. 检查当前项目文件结构；
4. 明确本次任务目标和验收条件；
5. 不得擅自大规模重构与当前任务无关的模块。

AI 编写代码时必须：

1. 只处理当前阶段相关任务；
2. 核心算法和 ROS2 通信代码分离；
3. 参数不得无说明地硬编码；
4. 不得改变既定坐标系、电机编号和 Topic 接口；
5. 如确需改变关键设计，先说明原因并记录到“技术决策”；
6. 不得以“成功编译”等同于“功能完成”；
7. 不得删除已有有效功能来规避问题。

每次任务完成后，AI 必须：

1. 执行或提供明确的编译命令；
2. 执行或提供明确的运行与验证步骤；
3. 说明修改了哪些文件；
4. 说明实际完成了什么；
5. 说明哪些内容尚未验证；
6. 更新本文件中的相关内容；
7. 仅在稳定功能、环境或使用方式发生变化时更新 `README.md`；
8. 为后续 `docs/ai_usage.md` 保留关键 Prompt、AI 错误及人工修正信息。

## 13. 文档维护规则

### README.md

只记录：

* 项目稳定目标；
* 已确定总体方案；
* 已稳定实现的功能；
* 最终项目结构；
* 可复现的编译与运行方式；
* 实验结果和使用说明。

不得记录临时报错、短期任务或未经验证的结论。

### AI_CONTEXT.md

每完成一个有实际意义的开发任务后更新，重点维护：

* 当前阶段目标；
* 已完成并验证内容；
* 当前问题；
* 技术决策；
* 实际节点和 Topic；
* 验证记录；
* 下一步任务。

更新时应覆盖失效内容，避免长期堆积已经过时的信息。

### ai_usage.md

开发过程中先保存素材，最终统一整理，内容至少包括：

* 使用的 AI 工具；
* 关键 Prompt 或交互摘要；
* AI 完成的模块；
* 人工确认和修改的公式及接口；
* AI 产生的错误；
* 错误发现与修正过程；
* 动力学、控制器和 ROS2 接口的验证方法。

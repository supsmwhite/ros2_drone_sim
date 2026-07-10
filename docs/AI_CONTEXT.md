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

工程初始化与动力学模块设计。

### 当前任务

1. 开发环境与空工作空间检查已完成，结果见“验证记录”；
2. 在确认初始化内容后建立 Git 基线提交；
3. 创建基础 ROS2 功能包；
4. 确定坐标系、电机编号和旋转方向；
5. 确定动力学状态量、输入和输出；
6. 建立动力学节点基本框架；
7. 建立最基础的 RViz2 显示和启动文件。

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

当前尚未完成任何核心功能。

已完成的工程基础验证：

* 已建立 ROS2 工作空间目录、Git 仓库和基础 `.gitignore`；
* Ubuntu 22.04、ROS2 Humble、C++17/CMake、Eigen3 和计划使用的基础 ROS2 依赖已通过检查；
* 当前空工作空间可执行 `colcon build --symlink-install`，但由于 `src/` 中没有功能包，该结果不代表任何 ROS2 功能包已经通过编译。

详细命令、结果和验证边界见“验证记录”。以上环境检查不代表动力学、控制器或可视化功能已经实现。

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

* RViz2 的可执行文件和命令行入口可用，但尚未实际验证图形窗口、OpenGL 渲染和项目显示配置；
* 当前没有 ROS2 功能包，因此尚未验证任何项目包的依赖声明、编译、测试、节点、Topic 或 TF；
* Git 仓库当前尚无提交，所有项目文件仍未纳入版本历史；
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

### 初始坐标约定

暂定采用 ROS 常用的 ENU 坐标系：

* `x`：前方；
* `y`：左方；
* `z`：上方；
* 重力方向：世界坐标系 `-z`；
* 机体推力方向：机体系 `+z`。

电机编号、旋转方向及 Mixer 矩阵必须在实现动力学前明确，并在后续保持统一。

如修改坐标或电机约定，必须同步更新：

* 动力学模型；
* 控制器；
* Mixer；
* URDF；
* RViz2；
* README；
* 本文件。

## 7. 当前系统接口

当前接口尚未最终实现，初步规划如下。

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
/drone/odom
/drone/imu
/drone/path
/tf
```

### 控制器节点

节点名称：

```text
position_controller_node
```

订阅：

```text
/drone/goal
/drone/odom
```

发布：

```text
/drone/motor_rpm_cmd
```

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

接口只有在实际实现后，才能标记为确定状态。

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

核心转动方程：

```text
I * omega_dot = tau - omega × (I * omega)
```

姿态使用四元数表示，积分后必须归一化。

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
colcon build --symlink-install
source install/setup.bash
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

1. 人工确认当前初始化文件并建立 Git 基线提交；
2. 创建 `drone_msgs`、`drone_dynamics` 和 `drone_bringup`；
3. 明确电机编号与旋转方向；
4. 明确动力学参数与状态变量；
5. 实现动力学算法类；
6. 实现动力学 ROS2 节点；
7. 完成第一组动力学独立测试。

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

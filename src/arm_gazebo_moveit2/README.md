<div align="center">

# arm_gazebo_moveit2

**Gazebo Harmonic 仿真 + ros2_control + MoveIt2 的 SO-101 抓取演示**

</div>

---

## 1. 功能简介

`arm_gazebo_moveit2` 在 Gazebo 中仿真 SO-101 机械臂，并提供 MoveIt2 规划与抓取演示：

1. `gazebo.launch.py`：启动 Gazebo 世界、机器人模型、ros2_control 控制器。
2. `demo_gazebo.launch.py`：在 Gazebo 基础上启动 `move_group`、Planning Scene 物体和 RViz。
3. `grasp_demo.launch.py`：自动执行固定九步抓取流程。
4. `grasp_demo_gui.launch.py`：用 PyQt5 界面设置木块/放置位置和速度，手动触发抓取。

本包只做仿真；真机 MoveIt 配置见 `arm_moveit_config`。

---

## 2. 安装依赖

### 2.1 需要安装的库

| 依赖 | 用途 |
| --- | --- |
| `arm_description`（当前工作空间） | 机械臂 URDF 与 STL 网格 |
| `xacro`、`robot_state_publisher` | 模型解析与 TF |
| `ros_gz_sim`、`ros_gz_bridge` | Gazebo 启动与 `/clock` 桥接 |
| `gz_ros2_control` | Gazebo 内 ros2_control |
| `controller_manager`、`joint_trajectory_controller`、`joint_state_broadcaster` | 控制器 |
| `moveit_configs_utils`、`moveit_ros_move_group` | MoveIt2 配置与规划 |
| `moveit_ros_visualization`、`rviz2` | MotionPlanning 面板 |
| `moveit_msgs`、`shape_msgs`、`control_msgs`、`trajectory_msgs` | 规划/执行消息 |
| `python3-pyqt5`、`python3-yaml` | GUI 抓取演示 |

### 2.2 安装与编译

```bash
sudo apt install ros-$ROS_DISTRO-ros-gz-sim \
                 ros-$ROS_DISTRO-ros-gz-bridge \
                 ros-$ROS_DISTRO-gz-ros2-control \
                 ros-$ROS_DISTRO-moveit \
                 ros-$ROS_DISTRO-controller-manager \
                 ros-$ROS_DISTRO-joint-trajectory-controller \
                 ros-$ROS_DISTRO-joint-state-broadcaster \
                 python3-pyqt5 \
                 python3-yaml \
                 ros-$ROS_DISTRO-xacro

cd /home/zwr185/dev_ws
colcon build --symlink-install --packages-select arm_description arm_gazebo_moveit2
source install/setup.bash
```

> 必须先编译 `arm_description`，否则 Gazebo 找不到 `package://arm_description/meshes/...`。

---

## 3. 四个 Launch 文件

### 3.1 gazebo.launch.py

**功能**：启动 Gazebo 世界（地面、木板、障碍墙、待抓取方块）、`robot_state_publisher`、模型生成、`/clock` 桥接，并依次加载 `joint_state_broadcaster`、`manipulator_controller`、`gripper_controller`。不启动 MoveIt。

**启动命令**：

```bash
cd /home/zwr185/dev_ws
source install/setup.bash

# 带 Gazebo GUI
ros2 launch arm_gazebo_moveit2 gazebo.launch.py

# 无显示环境（SSH）只启动 server
ros2 launch arm_gazebo_moveit2 gazebo.launch.py headless:=true
```

**对应的源码文件**：

| 文件 | 作用 |
| --- | --- |
| `launch/gazebo.launch.py` | Gazebo、模型生成、控制器加载 |
| `urdf/so101_new_follower.gazebo.xacro` | 仿真机器人描述（包含 `arm_description` 模型） |
| `urdf/so101_new_follower.ros2_control.xacro` | ros2_control 硬件与关节接口 |
| `worlds/so101_empty.sdf` | Gazebo 世界与场景物体 |
| `config/gazebo_colors.yaml` | link 颜色覆盖 |
| `config/ros2_controllers.yaml` | 控制器配置 |

**依赖的目录**：

| 目录 | 内容 / 作用 |
| --- | --- |
| `launch/` | 存放 launch 文件 |
| `urdf/` | Gazebo / ros2_control xacro |
| `worlds/` | `so101_empty.sdf` |
| `config/` | 颜色、控制器、初始位姿、关节限位等 |
| `/home/zwr185/dev_ws/src/arm_description/` | URDF 与 `meshes/`，通过 `GZ_SIM_RESOURCE_PATH` 解析 |
| `resource/` | ament 包索引标记 |

**调试参数**：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `headless` | `false` | `true` 只启动 `gz sim -s`，不打开 Gazebo GUI |

---

### 3.2 demo_gazebo.launch.py

**功能**：包含 `gazebo.launch.py`，再启动：

- `move_group`：MoveIt 规划、执行、Planning Scene；
- `apply_planning_scene.py`：把木板、障碍墙、可选目标方块加入 Planning Scene；
- `rviz2`：加载 `config/moveit.rviz`，提供 MotionPlanning 面板。

**启动命令**：

```bash
ros2 launch arm_gazebo_moveit2 demo_gazebo.launch.py

# 无 RViz
ros2 launch arm_gazebo_moveit2 demo_gazebo.launch.py use_rviz:=false

# 抓取调试：目标方块不作为碰撞体
ros2 launch arm_gazebo_moveit2 demo_gazebo.launch.py include_target_block:=false

# 无显示环境
ros2 launch arm_gazebo_moveit2 demo_gazebo.launch.py headless:=true use_rviz:=false
```

**对应的源码文件**：

| 文件 | 作用 |
| --- | --- |
| `launch/demo_gazebo.launch.py` | 组合 Gazebo + MoveIt + RViz |
| `launch/gazebo.launch.py` | 仿真与控制器 |
| `scripts/apply_planning_scene.py` | 把木板/障碍物写入 Planning Scene |
| `config/moveit.rviz` | RViz 配置 |
| `config/kinematics.yaml`、`ompl_planning.yaml`、`so101_new_follower.srdf` | MoveIt 配置 |

**启动参数**：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `headless` | `false` | 是否只启动 Gazebo server |
| `use_rviz` | `true` | 是否启动 RViz |
| `include_target_block` | `true` | 是否把待抓取方块加入 Planning Scene；抓取时必须为 `false` |

---

### 3.3 grasp_demo.launch.py

**功能**：在 `demo_gazebo.launch.py`（`include_target_block:=false`）基础上启动 `scripts/grasp_demo.py`，自动执行九步流程：

```text
初始位姿 → 打开夹爪 → IK 到方块 → 闭合夹爪 → 回 up
→ shoulder_pan 转负限位 → 释放 → 复位
```

**启动命令**：

```bash
ros2 launch arm_gazebo_moveit2 grasp_demo.launch.py

# 不启动 RViz，只看 Gazebo
ros2 launch arm_gazebo_moveit2 grasp_demo.launch.py use_rviz:=false
```

**对应的源码文件**：

| 文件 | 作用 |
| --- | --- |
| `scripts/grasp_demo.py` | 固定九步抓取流程 |
| `launch/grasp_demo.launch.py` | 组合仿真 + 抓取脚本 |
| `config/initial_positions.yaml` | 初始关节角 |
| `config/joint_limits.yaml` | 速度/加速度限制 |

**调试参数**（脚本内常量）：

| 常量 | 默认值 | 说明 |
| --- | --- | --- |
| `GRASP_TCP_REL_BASE` | `(0.30, 0.0, 0.0075)` | 抓取 TCP 相对底座位置（m） |
| `GRASP_LINK_QUAT` | 绕 z 轴 90° | 夹爪朝向 |
| `CLAMP_GRIPPER` | `-0.80` | 夹紧时的夹爪角度 |
| `PAN_NEGATIVE_LIMIT` | `-1.95` | 放置时 shoulder_pan 角度 |

---

### 3.4 grasp_demo_gui.launch.py

**功能**：在 `demo_gazebo.launch.py`（`include_target_block:=false`）基础上启动 PyQt5 界面，可以设置：

- 木块世界坐标 X/Y/Z；
- 抓取点 Y/Z 偏移；
- 放置点 X/Y/Z（保存到 `~/.config/arm_gazebo_moveit2/grasp_gui.yaml`）；
- 速度/加速度缩放；
- 复位、抓取、停止按钮与实时日志。

**启动命令**：

```bash
ros2 launch arm_gazebo_moveit2 grasp_demo_gui.launch.py
```

> GUI 需要显示器；SSH 无显示环境不可用。

**对应的源码文件**：

| 文件 | 作用 |
| --- | --- |
| `scripts/grasp_demo_gui.py` | PyQt5 抓取界面与九步流程 |
| `launch/grasp_demo_gui.launch.py` | 组合仿真 + GUI |
| `config/grasp_gui.yaml` | 放置点默认值（会复制到用户配置目录） |
| `config/initial_positions.yaml`、`joint_limits.yaml` | 初始位姿与运动限制 |

**GUI 调试参数**：

| 参数 | 默认值 | 范围 | 说明 |
| --- | --- | --- | --- |
| 木块 X/Y/Z | `0.30` / `0.0` / `0.115` m | 按世界范围 | 目标方块位置 |
| 抓取 Y 偏移 | 代码默认 | `-0.3 ~ 0.3` m | 调整夹爪对准点 |
| 抓取 Z 偏移 | 代码默认 | `-0.2 ~ 0.3` m | 调整抓取高度 |
| 放置 X/Y/Z | `0.06` / `0.20` / `0.115` m | Z 不低于木板顶面 0.1 | 放置位置，自动保存 |
| 速度/加速度缩放 | `0.5` / `0.5` | `0.05 ~ 1.0` | 越小越慢、越安全 |

---

## 4. 仿真配置

| 文件 | 常用调试项 |
| --- | --- |
| `config/ros2_controllers.yaml` | `update_rate: 50`、控制器名称与关节列表 |
| `config/initial_positions.yaml` | 启动时的初始关节角 |
| `config/joint_limits.yaml` | 速度/加速度缩放与各关节上限 |
| `config/kinematics.yaml` | IK 求解器与超时 |
| `config/ompl_planning.yaml` | 规划器参数 |
| `config/so101_new_follower.srdf` | 规划组、`home`/`up`/`clamp` 状态 |
| `config/gazebo_colors.yaml` | 各 link 在 Gazebo 中的颜色 |
| `worlds/so101_empty.sdf` | 木板、障碍墙、目标方块位置 |

常用调试命令：

```bash
ros2 control list_controllers
ros2 action list
ros2 topic echo /joint_states
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "{...}"
```

- 控制器没加载：确认 `gazebo.launch.py` 正常生成模型，`controller_manager` 已出现。
- MoveIt 规划失败：检查 `include_target_block`，抓取时必须为 `false`。
- 机器人掉到地面/穿模：检查 `initial_positions.yaml` 和 world 中的木板高度。
- 无显示环境：使用 `headless:=true use_rviz:=false`。

---

## 5. 常见问题

| 问题 | 处理 |
| --- | --- |
| Gazebo 找不到 mesh | 先编译 `arm_description`，并确认 `GZ_SIM_RESOURCE_PATH` 包含其 share |
| 模型生成失败 | 检查 `urdf/*.xacro` 能否被 `xacro` 解析 |
| 控制器加载失败 | 检查 `config/ros2_controllers.yaml` 名称与 launch 中 `CONTROLLERS` 一致 |
| 抓取时夹爪被挡 | 确认 `include_target_block:=false` |
| GUI 打不开 | 检查 `python3-pyqt5` 和显示器 |
| 规划很慢/失败 | 降低速度缩放、调整 `ompl_planning.yaml`、检查起始位姿碰撞 |

---

## License

BSD-3-Clause，详见 `package.xml`。

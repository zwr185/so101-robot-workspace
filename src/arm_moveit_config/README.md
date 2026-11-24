<div align="center">

# arm_moveit_config

**SO-101 真机 MoveIt2 配置、ros2_control 控制链路与真机抓取界面**

</div>

---

## 1. 功能简介

`arm_moveit_config` 提供真机机械臂的 MoveIt2 配置与抓取演示：

1. `demo.launch.py`：真机 ros2_control + `move_group` + RViz 的基础启动文件。
2. `grasp_realarm.launch.py`：在 demo 基础上启动固定姿态 IK 的抓取 GUI。
3. `1_grasp_realarm.launch.py`：在 demo 基础上启动“固定 `wrist_roll`、只解前 4 关节”的抓取 GUI。

硬件接口由 `arm_hardware` 的 `St3215SystemHardware` 提供，机器人模型来自 `arm_description`。

---

## 2. 安装依赖

### 2.1 需要安装的库

| 依赖 | 用途 |
| --- | --- |
| `arm_description`（当前工作空间） | URDF、meshes |
| `arm_hardware`（当前工作空间） | ST3215 串口 ros2_control 硬件插件 |
| `moveit_ros_move_group`、`moveit_configs_utils` | MoveIt2 规划与配置 |
| `controller_manager`、`joint_trajectory_controller`、`joint_state_broadcaster` | ros2_control |
| `robot_state_publisher`、`rviz2`、`xacro` | 模型发布与可视化 |
| `moveit_msgs`、`control_msgs`、`trajectory_msgs`、`sensor_msgs` | 规划/执行消息 |
| `python3-pyqt5`、`python3-yaml` | 抓取 GUI |

### 2.2 安装与编译

```bash
sudo apt install ros-$ROS_DISTRO-moveit \
                 ros-$ROS_DISTRO-controller-manager \
                 ros-$ROS_DISTRO-joint-trajectory-controller \
                 ros-$ROS_DISTRO-joint-state-broadcaster \
                 python3-pyqt5 \
                 python3-yaml \
                 ros-$ROS_DISTRO-xacro

cd /home/zwr185/dev_ws
colcon build --symlink-install \
  --packages-select arm_description arm_hardware arm_moveit_config
source install/setup.bash
```

### 2.3 真机串口

硬件插件默认使用 `/dev/ttyACM0`、1,000,000 波特率、舵机 ID 1–6，配置在：

```text
config/so101_new_follower.ros2_control.xacro
```

如果串口不同，修改该文件中的 `<param name="port">`。串口权限：

```bash
sudo usermod -aG dialout $USER
# 重新登录后生效
```

---

## 3. 主要 Launch 文件

### 3.1 demo.launch.py

**功能**：真机 MoveIt 基础启动，包含：

| 节点 | 作用 |
| --- | --- |
| `robot_state_publisher` | 发布 URDF 与 TF |
| `move_group` | MoveIt 规划、Planning Scene、轨迹执行 |
| `rviz2` | MotionPlanning 面板 |
| `ros2_control_node` | 加载 `arm_hardware/St3215SystemHardware` |
| `spawner` | 加载 `manipulator_controller`、`gripper_controller`、`joint_state_broadcaster` |

**启动命令**：

```bash
cd /home/zwr185/dev_ws
source install/setup.bash

# 真机模式
ros2 launch arm_moveit_config demo.launch.py

# 无硬件，使用 mock_components 测试
ros2 launch arm_moveit_config demo.launch.py use_fake_hardware:=true

# 不启动 RViz
ros2 launch arm_moveit_config demo.launch.py use_rviz:=false
```

**对应的源码文件**：

| 文件 | 作用 |
| --- | --- |
| `launch/demo.launch.py` | 真机控制链路 + MoveIt + RViz |
| `config/so101_new_follower.urdf.xacro` | 机器人描述入口，包含 `arm_description` |
| `config/so101_new_follower.ros2_control.xacro` | 串口、波特率、舵机 ID、ros2_control 接口 |
| `config/ros2_controllers.yaml` | 控制器与关节列表 |
| `config/so101_new_follower.srdf` | 规划组、`home`/`up`/`clamp` 状态 |
| `config/kinematics.yaml`、`ompl_planning.yaml` | IK 与规划参数 |

**启动参数**：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_fake_hardware` | `false` | `true` 使用 `mock_components/GenericSystem`，不连真机 |
| `use_rviz` | `true` | 是否启动 RViz |
| `db` | `false` | 是否启动 MoveIt warehouse 数据库 |

**关键调试参数**（launch 内固定）：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `trajectory_execution.allowed_start_tolerance` | `0.1` | 放宽起始状态偏差 |
| `trajectory_execution.allowed_execution_duration_scaling` | `3.0` | 允许执行时间放大 3 倍 |
| `trajectory_execution.allowed_goal_duration_margin` | `2.0` | 目标时间额外余量（秒） |
| `publish_frequency` | `15.0` | `robot_state_publisher` 发布频率 |

---

### 3.2 grasp_realarm.launch.py

**功能**：包含 `demo.launch.py`（真机模式），再启动 `grasp_realarm_gui.py`。GUI 输入木块坐标和 X/Y/Z 抓取偏移，按 8 步流程执行：

```text
home → 打开夹爪 → 到抓取点 → 闭合夹爪 → 回 up → 到放置点 → 打开夹爪 → 回 home
```

抓取使用固定姿态 IK，夹爪 TCP 朝向由代码常量决定。

**启动命令**：

```bash
ros2 launch arm_moveit_config grasp_realarm.launch.py

# 不启动 RViz
ros2 launch arm_moveit_config grasp_realarm.launch.py use_rviz:=false
```

**对应的源码文件**：

| 文件 | 作用 |
| --- | --- |
| `scripts/grasp_realarm_gui.py` | 固定姿态 IK + 8 步抓取 GUI |
| `launch/grasp_realarm.launch.py` | 组合 demo + GUI |
| `config/grasp_realarm.yaml` | 默认木块坐标与放置点 |
| `config/ros2_controllers.yaml` | 控制器 |

**依赖的目录**：

| 目录 | 内容 / 作用 |
| --- | --- |
| `launch/` | MoveIt 与抓取 launch |
| `config/` | URDF/SRDF、控制器、运动学、抓取默认坐标 |
| `scripts/` | 抓取 GUI |
| `/home/zwr185/dev_ws/src/arm_description/` | 机器人模型与 meshes |
| `/home/zwr185/dev_ws/src/arm_hardware/` | 真机 ros2_control 硬件插件 |
| `resource/` 与 `.setup_assistant` | ament / MoveIt Setup Assistant 标记 |

**调试参数**：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| 木块 X/Y/Z | `0.30` / `0.00` / `0.00` m | base_link 坐标，可在 `config/grasp_realarm.yaml` 修改 |
| 抓取 X/Y/Z 偏移 | 代码常量 | 调整 TCP 对准点 |
| 放置 X/Y/Z | `0.20` / `0.20` / `0.04` m | `config/grasp_realarm.yaml` 的 `place` 段 |
| 速度/加速度缩放 | `0.1` / `0.1` | GUI 范围 `0.05 ~ 1.0`，真机建议保守 |

---

### 3.3 1_grasp_realarm.launch.py

**功能**：与 `grasp_realarm` 相同的 8 步流程，但第 3 步固定 `wrist_roll = 0`，用数值位置 IK 求前 4 个关节，让 `gripper_frame_link` 到达 GUI 给定位置，再交给 MoveIt 规划执行。

**启动命令**：

```bash
ros2 launch arm_moveit_config 1_grasp_realarm.launch.py
```

**对应的源码文件**：

| 文件 | 作用 |
| --- | --- |
| `scripts/1_grasp_realarm_gui.py` | 固定 `wrist_roll` 的位置 IK 抓取 GUI |
| `launch/1_grasp_realarm.launch.py` | 组合 demo + GUI |
| `config/grasp_realarm.yaml` | 默认坐标 |

**额外调试常量**：

| 常量 | 默认值 | 说明 |
| --- | --- | --- |
| `GRASP_WRIST_ROLL` | `0.0` | 抓取时固定的 wrist_roll 角度（rad） |
| `GRASP_POSITION_TOLERANCE` | `0.004` | 位置 IK 收敛阈值（m） |
| `GRASP_X/Y/Z_OFFSET` | `0.05` / `0.03` / `0.04` | 抓取点偏移（m） |

---

### 3.4 其他辅助 Launch

| 文件 | 作用 |
| --- | --- |
| `rsp.launch.py` | 只启动 `robot_state_publisher` |
| `move_group.launch.py` | 只启动 `move_group` |
| `moveit_rviz.launch.py` | 只启动 MoveIt RViz |
| `spawn_controllers.launch.py` | 单独加载控制器 |
| `warehouse_db.launch.py` | MoveIt warehouse 数据库 |
| `static_virtual_joint_tfs.launch.py` | 发布虚拟关节静态 TF |
| `setup_assistant.launch.py` | MoveIt Setup Assistant |

---

## 4. 调试要点

| 项目 | 说明 |
| --- | --- |
| 串口 | 默认 `/dev/ttyACM0`，修改 `config/so101_new_follower.ros2_control.xacro` |
| 波特率 / ID | 默认 `1000000`、`1,2,3,4,5,6`，同上文件 |
| 控制器 | `manipulator_controller`（5 关节）、`gripper_controller`（夹爪）、`joint_state_broadcaster` |
| 关节限位 | `config/joint_limits.yaml` |
| 初始位姿 | `config/initial_positions.yaml` |
| 规划组 | `config/so101_new_follower.srdf` |
| IK | `config/kinematics.yaml` |
| 抓取默认坐标 | `config/grasp_realarm.yaml` |

常用调试命令：

```bash
ros2 control list_controllers
ros2 topic echo /joint_states
ros2 topic echo /planning_scene
ros2 action list
ros2 service list | grep compute_ik
```

- 真机不动：确认 `use_fake_hardware:=false`、串口正确、`/joint_states` 有数据。
- 轨迹执行报 `CONTROL_FAILED`：降低速度缩放，或适当增大 launch 中的执行容差。
- 抓取位置不准：调整 GUI 偏移和 `config/grasp_realarm.yaml`。
- GUI 打不开：检查 `python3-pyqt5` 和显示器。

---

## 5. 常见问题

| 问题 | 处理 |
| --- | --- |
| 找不到 `arm_hardware` 插件 | 编译 `arm_hardware` 后重新 `source install/setup.bash` |
| 串口打不开 | 检查 `/dev/ttyACM0`、`dialout` 权限、是否被占用 |
| 控制器未激活 | `ros2 control list_controllers`，检查 spawner 日志 |
| MoveIt 规划失败 | 检查初始位姿、碰撞、`srdf` 和 `joint_limits` |
| 轨迹执行超时 | 降低速度/加速度缩放，检查 launch 中执行容差 |
| 无硬件想测试 | `use_fake_hardware:=true` |

---

## License

BSD-3-Clause，详见 `package.xml`。

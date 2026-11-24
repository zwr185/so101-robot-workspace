<div align="center">

# arm_description

**SO-ARM101 机械臂模型、RViz 可视化、自定义关节 GUI 与 IK 服务**

</div>

---

## 1. 功能简介

`arm_description` 提供机械臂的模型与调试工具：

1. **URDF 模型**：`urdf/arm.urdf.xacro` 定义 SO-101 的 link、joint、限位和 STL 网格。
2. **RViz 显示**：加载模型、TF 和预设视角，支持滑块或自定义 GUI 控制关节。
3. **自定义关节 GUI**：`joint_state_gui` 提供角度滑块、末端坐标显示、工作空间采样和 IK 求解。
4. **IK 服务**：`ik_solver_node` 提供 `/solve_ik`（`moveit_msgs/GetPositionIK`）。
5. **真机同步**：`teleop_real.launch.py` 把 GUI 的 `/joint_states` 转发为 `/cmd_angles`，驱动真机从动臂。

本包不包含舵机串口协议；真机驱动复用 `remote_control_pkg/follower_arm_joint`。

---

## 2. 安装依赖

### 2.1 需要安装的库

| 依赖 | 用途 |
| --- | --- |
| `robot_state_publisher`、`joint_state_publisher_gui` | 发布 TF 和滑块控制 |
| `rviz2` | 模型可视化 |
| `xacro`、`urdf` | 解析模型 |
| `moveit_msgs` | `GetPositionIK` 服务类型 |
| `tf2_ros` | 末端位姿查询 |
| `rclpy`、`sensor_msgs`、`std_msgs` | ROS 2 通信 |
| `python3-numpy` | 正逆运动学计算 |
| `python3-pyqt5` | 自定义关节 GUI |
| `arm_msg`（当前工作空间） | `/cmd_angles` 消息 |
| `remote_control_pkg`（当前工作空间） | 真机从动臂驱动 |

### 2.2 安装与编译

```bash
sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui \
                 ros-$ROS_DISTRO-rviz2 \
                 ros-$ROS_DISTRO-robot-state-publisher \
                 ros-$ROS_DISTRO-moveit-msgs \
                 ros-$ROS_DISTRO-tf2-ros \
                 python3-numpy \
                 python3-pyqt5 \
                 ros-$ROS_DISTRO-xacro

cd /home/zwr185/dev_ws
colcon build --symlink-install --packages-select arm_msg remote_control_pkg arm_description
source install/setup.bash
```

### 2.3 真机串口权限

```bash
sudo usermod -aG dialout $USER
# 重新登录后生效
```

---

## 3. 三个 Launch 文件

### 3.1 display.launch.py

**功能**：最基础的模型显示。启动 `robot_state_publisher`、`joint_state_publisher_gui` 和 `rviz2`；用滑块改变关节角，RViz 中的模型同步运动。不连接真机。

**启动命令**：

```bash
cd /home/zwr185/dev_ws
source install/setup.bash
ros2 launch arm_description display.launch.py
```

**对应的源码文件**：

| 文件 | 作用 |
| --- | --- |
| `urdf/arm.urdf.xacro` | 机械臂模型、关节限位、网格引用 |
| `rviz/arm.rviz` | RViz 预设视角与显示配置 |
| `meshes/` | 模型 STL 文件 |
| `launch/display.launch.py` | 启动三个显示节点 |

**依赖的目录**：

| 目录 | 内容 / 作用 |
| --- | --- |
| `launch/` | 存放 launch 文件 |
| `urdf/` | `arm.urdf.xacro` |
| `meshes/` | `Follower/`、`Leader/`、`Individual/` 三组 STL |
| `rviz/` | `arm.rviz` |
| `scripts/` | 本 launch 不直接使用 |

**调试要点**：

- 模型不显示：确认 `xacro` 能解析 `urdf/arm.urdf.xacro`，并已 `source install/setup.bash`。
- 网格缺失：检查 `meshes/` 是否安装到 `share/arm_description/meshes/`。
- 关节滑块顺序按 URDF 关节顺序显示，适合快速检查零位和限位。

---

### 3.2 display_custom_gui.launch.py

**功能**：启动模型、自定义关节 GUI 和 IK 服务。GUI 可以拖动关节、显示末端坐标、输入目标点调用 `/solve_ik`；启动 RViz 查看结果。

**启动命令**：

```bash
cd /home/zwr185/dev_ws
source install/setup.bash

# 启动 RViz
ros2 launch arm_description display_custom_gui.launch.py

# 不启动 RViz，只开 GUI
ros2 launch arm_description display_custom_gui.launch.py use_rviz:=false
```

**对应的源码文件**：

| 文件 | 作用 |
| --- | --- |
| `scripts/joint_state_gui.py` | PyQt5 关节控制 GUI，发布 `/joint_states`，调用 IK |
| `scripts/ik_solver_node.py` | 订阅 `/robot_description`，提供 `/solve_ik` |
| `scripts/arm_kinematics.py` | URDF 正运动学、数值 IK、工作空间采样 |
| `urdf/arm.urdf.xacro` | 模型与关节限位 |
| `rviz/arm.rviz` | RViz 配置 |

**依赖的目录**：

| 目录 | 内容 / 作用 |
| --- | --- |
| `launch/` | 存放 launch 文件 |
| `scripts/` | GUI、IK、运动学模块（CMake 安装到 `lib/arm_description`） |
| `urdf/` | 模型 |
| `meshes/` | STL |
| `rviz/` | RViz 配置 |
| `resource/` | ament 包索引标记 |

**话题与服务**：

| 名称 | 类型 | 作用 |
| --- | --- | --- |
| `/robot_description` | `std_msgs/String`（TRANSIENT_LOCAL） | URDF 文本，GUI/IK 解析模型 |
| `/joint_states` | `sensor_msgs/JointState` | GUI 发布的关节角（弧度） |
| `/solve_ik` | `moveit_msgs/GetPositionIK` | 末端位置 IK 服务 |

**调试参数**：

| 节点 | 参数 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `joint_state_gui` | `base_frame` | `base_link` | 末端坐标参考系 |
| `joint_state_gui` | `tip_frame` | `gripper_frame_link` | 末端链接 |
| `joint_state_gui` | `publish_rate` | `20.0` | `/joint_states` 发布频率（Hz） |
| `joint_state_gui` | `ik_service` | `/solve_ik` | IK 服务名 |
| `ik_solver_node` | `target_link` | `gripper_frame_link` | IK 目标链接 |
| `ik_solver_node` | `tolerance_cm` | `0.5` | IK 容差（cm） |
| `ik_solver_node` | `max_iterations` | `300` | 单次迭代上限 |
| `ik_solver_node` | `restarts` | `10` | 随机重启次数 |

> 这些参数没有在 launch 中暴露，需要调整时使用 `ros2 run ... --ros-args -p 参数:=值`，或修改 launch 的 `parameters`。

**使用要点**：

- GUI 直接显示 URDF 角度；末端位置单位为 cm。
- IK 只求解 5 个臂关节，`gripper` 不参与末端位置求解。
- `ik_solver_node` 必须收到 `robot_state_publisher` 发布的 `/robot_description` 后才能求解。

---

### 3.3 teleop_real.launch.py

**功能**：在自定义 GUI 的基础上增加“仿真到真机”的转发：

```text
joint_state_gui → /joint_states → simtoreal_controll_node → /cmd_angles
→ remote_control_pkg/follower_arm_joint → 串口 → 真机从动臂
```

RViz 中的模型和真机从动臂同步运动，适合拖动 GUI 调试真机。

**启动命令**：

```bash
cd /home/zwr185/dev_ws
source install/setup.bash

# 默认 by-id 串口
ros2 launch arm_description teleop_real.launch.py

# 使用 /dev/ttyACM0，并修正 wrist_roll 方向
ros2 launch arm_description teleop_real.launch.py \
  port:=/dev/ttyACM0 \
  wrist_roll_sign:=1.0

# 只开 GUI/RViz，不驱动真机
ros2 launch arm_description teleop_real.launch.py start_driver:=false
```

**对应的源码文件**：

| 文件 | 作用 |
| --- | --- |
| `scripts/simtoreal_controll_node.py` | `/joint_states` → `/cmd_angles` 转发，处理 `wrist_roll` 方向 |
| `scripts/joint_state_gui.py` | GUI 关节控制 |
| `scripts/ik_solver_node.py` | IK 服务 |
| `scripts/arm_kinematics.py` | 运动学 |
| `remote_control_pkg/follower_arm_joint.py` | 真机从动臂串口驱动（外部包） |
| `remote_control_pkg/servo_controller.py` | 串口协议（外部包） |

**依赖的目录**：

| 目录 | 内容 / 作用 |
| --- | --- |
| `launch/`、`scripts/`、`urdf/`、`meshes/`、`rviz/` | 与 `display_custom_gui` 相同 |
| `/home/zwr185/dev_ws/src/remote_control_pkg/` | 真机从动臂驱动、串口控制 |
| `/home/zwr185/dev_ws/src/arm_msg/msg/` | `/cmd_angles` 消息 |
| `/home/zwr185/dev_ws/Seeed_RoboController/json/` | 从动臂校准文件 |

**启动参数**：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_rviz` | `true` | 是否启动 RViz |
| `start_driver` | `true` | 是否启动真机从动臂驱动 |
| `port` | by-id 从动臂串口 | 真机串口；用 `/dev/ttyACM0` 时显式传入 |
| `wrist_roll_sign` | `-1.0` | `wrist_roll` 方向修正；真机反向时改为 `1.0` |

**调试要点**：

- 真机不动：确认 `start_driver:=true`、串口正确、`/cmd_angles` 有数据。
- 真机方向相反：调整 `wrist_roll_sign`；其他关节方向由校准文件 `drive_mode` 决定。
- GUI 与真机初始姿态差太大：先手动把真机摆到接近 GUI 的零位，再启动驱动。
- 只调试模型时使用 `start_driver:=false`，避免串口被占用。

---

## 4. 常见问题

| 问题 | 处理 |
| --- | --- |
| 模型不显示 | 确认 `xacro` 可解析、`meshes/` 已安装、`source install/setup.bash` |
| `IK 服务不可用` | 确认 `ik_solver_node` 已启动，且 `/robot_description` 有数据 |
| 自定义 GUI 打不开 | 检查 `python3-pyqt5` 和显示器环境 |
| 真机不跟随 | 检查 `port`、`start_driver`、`/cmd_angles`、`/joint_states` |
| 真机方向反 | 调整 `wrist_roll_sign` 或校准文件 `drive_mode` |
| 修改脚本后不生效 | 使用 `--symlink-install` 重新编译，或重新 `colcon build` |

---

## License

本包许可证见 `package.xml` 与仓库根目录说明。

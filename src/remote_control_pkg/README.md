<div align="center">

# remote_control_pkg

**SO-ARM101 主从遥操作驱动：领导臂读取 + 从动臂实时跟随**

</div>

---

## 1. 功能简介

`remote_control_pkg` 提供双机械臂主从遥操作：

1. `leader_arm_joint`：释放领导臂舵机力矩，操作者手动拖动，节点以 50 Hz 读取角度并发布 `/leader_joint_angles`。
2. `follower_arm_joint`：订阅领导臂角度，驱动从动臂同 ID 舵机跟随。
3. `teleop_quit_key`：监听终端 `p` 键，退出整个遥操作 launch。

从动臂首帧用 1.2 秒余弦缓动对齐领导臂姿态，之后直接下发，不再平滑，保证实时跟手。

---

## 2. 安装依赖

### 2.1 需要安装的库

| 依赖 | 用途 |
| --- | --- |
| `rclpy` | ROS 2 节点 |
| `arm_msg`（当前工作空间） | `ArmJointAngles` 消息 |
| `python3-serial` | 舵机串口通信 |
| `launch`、`launch_ros` | launch 启动 |

### 2.2 安装与编译

```bash
sudo apt install python3-serial

cd /home/zwr185/dev_ws
colcon build --symlink-install --packages-select arm_msg remote_control_pkg
source install/setup.bash
```

### 2.3 串口权限

```bash
sudo usermod -aG dialout $USER
# 重新登录后生效
```

---

## 3. Launch 文件

### 3.1 teleop.launch.py

**功能**：先启动领导臂读取节点，1 秒后启动从动臂跟随节点，并启动 `p` 键退出监听。领导臂和从动臂各占用一个串口。

**启动命令**：

```bash
cd /home/zwr185/dev_ws
source install/setup.bash

# 使用默认 by-id 串口
ros2 launch remote_control_pkg teleop.launch.py

# 使用 /dev/ttyACM* 或自定义串口
ros2 launch remote_control_pkg teleop.launch.py \
  leader_port:=/dev/ttyACM0 \
  follower_port:=/dev/ttyACM1
```

按 `p` 或 `Ctrl+C` 退出。

**对应的源码文件**：

| 文件 | 作用 |
| --- | --- |
| `remote_control_pkg/leader_arm_joint.py` | 领导臂只读节点：释放力矩、50 Hz 发布角度 |
| `remote_control_pkg/follower_arm_joint.py` | 从动臂跟随节点：订阅领导臂角度并直写舵机 |
| `remote_control_pkg/servo_controller.py` | 串口协议与直写控制（无平滑线程） |
| `remote_control_pkg/arm_calibration.py` | LeRobot 校准文件加载与角度换算 |
| `remote_control_pkg/teleop_quit_key.py` | 监听 `p` 键并触发 launch 退出 |

**依赖的目录**：

| 目录 | 内容 / 作用 |
| --- | --- |
| `launch/` | 存放本 launch 文件 |
| `remote_control_pkg/` | 领导臂、从动臂、串口控制与校准模块 |
| `/home/zwr185/dev_ws/src/arm_msg/msg/` | `ArmJointAngles.msg` |
| `/home/zwr185/dev_ws/Seeed_RoboController/json/` | 领导臂 / 从动臂校准文件（默认路径） |
| `resource/` | ament 包索引标记 |

**话题**：

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/leader_joint_angles` | `arm_msg/msg/ArmJointAngles` | 领导臂 → 从动臂 | 50 Hz 领导臂角度 |
| 反馈话题（默认关闭） | `ArmJointAngles` | 从动臂 → 外部 | 设置 `feedback_topic` 后才发布 |

**调试参数**：

| 节点 | 参数 | 默认值 | 说明 |
| --- | --- | --- | --- |
| leader | `port` | by-id 领导臂串口 | launch 参数 `leader_port` 覆盖 |
| leader | `baudrate` | `1000000` | 舵机波特率 |
| leader | `output_topic` | `/leader_joint_angles` | 角度发布话题 |
| leader | `publish_rate` | `50.0` | 发布频率（Hz） |
| leader | `calibration_file` | 领导臂校准 JSON | 角度换算 |
| leader | `release_torque` | `true` | 启动时释放领导臂力矩 |
| follower | `port` | by-id 从动臂串口 | launch 参数 `follower_port` 覆盖 |
| follower | `baudrate` | `1000000` | 舵机波特率 |
| follower | `input_topic` | `/leader_joint_angles` | 领导臂输入 |
| follower | `feedback_topic` | 空 | 空表示不反馈，避免和视觉链路冲突 |
| follower | `feedback_rate` | `5.0` | 反馈频率（Hz） |
| follower | `command_timeout` | `0.5` | 领导臂断流超时（秒） |
| follower | `align_on_start` | `true` | 首帧是否平滑对齐 |
| follower | `align_duration` | `1.2` | 首帧对齐时长（秒） |

**使用要点**：

- 启动后领导臂会释放力矩，可自由拖动；从动臂也会先释放力矩，建议先把两套机械臂摆到接近姿态再操作。
- 从动臂只在首帧做 1.2 秒缓动对齐，之后直接写目标位置；如果领导臂中断超过 `command_timeout`，从动臂会告警并保持当前位置。
- `follower_arm_joint` 也被 `arm_description/launch/teleop_real.launch.py` 复用，作为真机从动臂驱动。
- 两个串口不能用同一个设备；如果 by-id 路径不存在，改用 `/dev/ttyACM0`、`/dev/ttyACM1`。

**调试命令**：

```bash
ros2 node list
ros2 topic hz /leader_joint_angles
ros2 topic echo /leader_joint_angles
```

---

## 4. 常见问题

| 问题 | 处理 |
| --- | --- |
| 找不到 `remote_control_pkg` | 编译 `arm_msg` 和本包后执行 `source install/setup.bash` |
| by-id 串口不存在 | 用 `ls /dev/serial/by-id/` 查找，或改传 `leader_port` / `follower_port` |
| 打不开串口 | 检查权限、设备是否被其他程序占用 |
| 领导臂推不动 | 确认 `release_torque:=true`，或检查串口连接 |
| 从动臂不跟随 | 检查 `/leader_joint_angles` 是否有 50 Hz 数据，确认 `input_topic` 一致 |
| 按 `p` 无反应 | 当前终端不是 TTY 时无法读取按键，改用 `Ctrl+C` |
| 从动臂方向相反 | 检查校准文件的 `drive_mode` 和机械装配方向 |

---

## License

Apache-2.0。

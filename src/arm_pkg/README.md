<div align="center">

# arm_pkg

**SO-ARM101 / STS3215 机械臂的 ROS 2 驱动、轨迹录制回放与视觉抓取功能包**

</div>

---

## 1. 功能简介

`arm_pkg` 提供三类功能：

1. **关节驱动**：把 `ArmJointAngles` 话题转换为舵机原始位置，通过 `/dev/ttyACM0` 驱动 ID 1–6 的舵机。
2. **轨迹录制/回放**：用 PyQt5 界面把手动示教的角度录成 CSV，并按原节奏回放。
3. **视觉抓取/跟踪**：订阅 `vision_pkg` 输出的箱体坐标，经 IK 解算后驱动机械臂完成抓取放置或目标跟踪。

校准文件由 `Seeed_RoboController` 工具生成，本包只读取。

---

## 2. 安装依赖

### 2.1 需要安装的库

| 依赖 | 用途 |
| --- | --- |
| `rclpy`、`launch`、`launch_ros`、`ament_index_python` | ROS 2 运行与 launch |
| `arm_msg`（当前工作空间） | `ArmJointAngles` 消息 |
| `vision_pkg`（当前工作空间） | 箱体检测与像素→坐标转换 |
| `usb_cam` | USB 相机驱动 |
| `cv_bridge`、`python3-opencv` | `vision_pkg` 图像处理 |
| `python3-numpy` | URDF 正逆运动学计算 |
| `python3-serial` | 舵机串口通信 |
| `python3-pyqt5` | 录制回放 GUI |
| `python3-xacro` | 解析 `arm.urdf.xacro` |

### 2.2 安装与编译

```bash
# 系统 / ROS 依赖（包名随 ROS 发行版可能略有差异）
sudo apt install ros-$ROS_DISTRO-usb-cam \
                 ros-$ROS_DISTRO-cv-bridge \
                 python3-opencv \
                 python3-numpy \
                 python3-serial \
                 python3-pyqt5 \
                 python3-xacro

# 编译工作空间（只做录制回放时可不编译 vision_pkg）
cd /home/zwr185/dev_ws
colcon build --symlink-install --packages-select arm_msg vision_pkg arm_pkg
source install/setup.bash
```

也可以使用 pip 安装：

```bash
pip install pyserial PyQt5 xacro numpy
```

### 2.3 串口权限

```bash
sudo usermod -aG dialout $USER
# 重新登录后生效
```

---

## 3. 三个 Launch 文件

### 3.1 arm_record_playback.launch.py

**功能**：启动串口驱动和录制回放 GUI。回放时直接写舵机目标位置，不做平滑，保持录制时的速度；驱动以 10 Hz 发布当前角度供 GUI 录制。

**启动命令**：

```bash
cd /home/zwr185/dev_ws
source install/setup.bash

# 使用默认校准文件
ros2 launch arm_pkg arm_record_playback.launch.py

# 指定主动臂或其他校准文件
ros2 launch arm_pkg arm_record_playback.launch.py \
  calibration_file:=/home/zwr185/dev_ws/Seeed_RoboController/json/my_awesome_leader_arm.json
```

**对应的源码文件**：

| 文件 | 作用 |
| --- | --- |
| `arm_pkg/arm_playback_joint_node.py` | launch 启动的串口驱动节点（回放直写） |
| `arm_pkg/gui_record_playback_node.py` | launch 启动的 PyQt5 录制回放界面 |
| `arm_pkg/arm_catch_joint_node.py` | 驱动节点的基类，提供串口、订阅、反馈 |
| `arm_pkg/arm_joint_node.py` | 提供 `ServoController`（串口协议、平滑运动） |
| `arm_pkg/arm_calibration.py` | 角度与舵机原始位置换算 |
| `arm_pkg/csv_paths.py` | 解析 CSV 文件路径 |

**依赖的目录**：

| 目录 | 内容 / 作用 |
| --- | --- |
| `launch/` | 存放本 launch 文件 |
| `arm_pkg/` | 节点源码与公共模块 |
| `csv/` | 轨迹文件默认读写目录，含两个示例轨迹 |
| `/home/zwr185/dev_ws/src/arm_msg/msg/` | `ArmJointAngles.msg` |
| `/home/zwr185/dev_ws/Seeed_RoboController/json/` | 默认校准文件目录 |
| `resource/` | ament 包索引标记（编译安装需要） |

**关键话题**：

| 话题 | 作用 |
| --- | --- |
| `/cmd_angles` | GUI 回放 → 驱动 |
| `/arm_joint_angles` | 驱动反馈 → GUI 录制 |
| `/arm_catch_angles` | 外部视觉角度（平滑控制） |
| `/leader_joint_angles` + `/teleop_enable` | 遥操作输入与使能 |

**调试参数**：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `calibration_file` | `.../my_awesome_follower_arm.json` | 校准文件路径 |
| `port` | `/dev/ttyACM0` | 串口设备，写在节点参数中，launch 未暴露 |
| `feedback_rate` | `10.0` | 录制反馈频率（Hz） |
| GUI 文件名 | `arm_trajectory.csv` | 只写文件名时保存到 `csv/` |

**使用要点**：

- 驱动启动时会失能舵机，可以手动摆臂录制；回放后舵机保持力矩，重新示教建议重启 launch。
- 关闭 GUI 窗口只会结束 GUI，串口驱动仍在运行；结束整个 launch 请在终端按 `Ctrl+C`。

---

### 3.2 arm_catch_box.launch.py

**功能**：启动完整视觉抓取流水线：相机 → 颜色检测 → 像素坐标转测量坐标 → IK → 抓取并放置。抓取流程包括悬停、下降、闭合夹爪、抬起、移动到放置点、下降、张开夹爪、抬起。

**启动命令**：

```bash
cd /home/zwr185/dev_ws
source install/setup.bash

# 默认参数
ros2 launch arm_pkg arm_catch_box.launch.py

# 指定相机、校准文件和抓取偏移
ros2 launch arm_pkg arm_catch_box.launch.py \
  video_device:=/dev/video0 \
  calibration_file:=/home/zwr185/dev_ws/Seeed_RoboController/json/my_awesome_follower_arm.json \
  input_yaw_offset:=-90.0 \
  grasp_offset_x:=-2.0 \
  grasp_offset_y:=5.0
```

**对应的源码文件**：

| 文件 | 作用 |
| --- | --- |
| `arm_pkg/arm_catch_joint_node.py` | 串口驱动：订阅 `/arm_catch_angles` 并驱动 6 个舵机 |
| `arm_pkg/arm_catch_box_node.py` | 抓取状态机：IK 解算、悬停、夹取、放置 |
| `arm_pkg/arm_joint_node.py` | 提供 `ServoController` |
| `arm_pkg/arm_calibration.py` | 角度换算、关节限位、夹爪开合角度 |
| `arm_pkg/urdf_fk.py` | 解析 URDF、正运动学 |
| `arm_pkg/urdf_ik.py` | 逆运动学求解 |
| `arm_pkg/urdf/arm.urdf.xacro` | 机械臂模型与关节限位 |

**依赖的目录**：

| 目录 | 内容 / 作用 |
| --- | --- |
| `launch/` | 存放本 launch 文件 |
| `arm_pkg/` | 抓取节点、驱动节点与公共模块 |
| `urdf/` | `arm.urdf.xacro`，IK 使用的机械臂模型 |
| `/home/zwr185/dev_ws/src/arm_msg/msg/` | `ArmJointAngles.msg` |
| `/home/zwr185/dev_ws/src/vision_pkg/config/` | `cam.yaml`、`box_detect.yaml`、`box_tf.yaml` |
| `/home/zwr185/dev_ws/src/vision_pkg/vision_pkg/` | `box_detect_node.py`、`box_tf_node.py` |
| `/home/zwr185/dev_ws/src/vision_pkg/json/` | `box_tf_homography.json` 透视变换标定文件 |
| `/home/zwr185/dev_ws/Seeed_RoboController/json/` | 默认校准文件 |
| `usb_cam` ROS 包 | 相机驱动 |
| `resource/` | ament 包索引标记 |

**话题与按键**：

| 名称 | 作用 |
| --- | --- |
| `/image_raw` | 相机图像 |
| `/box_center` | 色块中心像素坐标 |
| `/box_world_pos` | 测量平面坐标（cm） |
| `/arm_catch_angles` | 抓取节点输出的 6 个角度 |
| `/arm_current_angles` | 驱动反馈的当前角度（5 Hz） |
| `/arm_catch_done` | 抓取完成/失败信号 |
| `s` / `g` / `q` | 停止抓取 / 重新抓取 / 退出 launch |

**调试参数**：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `video_device` | 空（自动探测） | 相机设备，如 `/dev/video0` |
| `calibration_file` | 从动臂校准 JSON | 关节限位和夹爪映射 |
| `input_yaw_offset` | `-90.0` | 视觉坐标绕底座 z 轴旋转角 |
| `grasp_offset_x` / `grasp_offset_y` | `-2.0` / `5.0` | 抓取点在视觉坐标系的偏移（cm） |
| `place_x` / `place_y` / `place_z` | `-30.0` / `10.0` / `10.0` | 放置点坐标（cm） |
| `pick_z_offset` / `grasp_z_offset` | `10.0` / `3.0` | 悬停高度、抓取高度（cm） |
| `step_delay` / `rearm_delay` | `2.0` / `5.0` | 每步等待时间、完成后冷却时间（秒） |
| `ik_tolerance` | `0.1` | IK 目标容差（cm），实际最大允许误差 2 cm |

> `place_*`、`pick_z_offset` 等未在 launch 中暴露，需要调试时可在 launch 里追加 `parameters`，或直接运行节点加 `--ros-args -p 参数:=值`。

---

### 3.3 arm_track_box.launch.py

**功能**：启动相机、检测、坐标转换和 IK 跟踪。机械臂 5 个关节持续跟随目标箱体，夹爪固定输出 `0°`，不执行抓取。

**启动命令**：

```bash
cd /home/zwr185/dev_ws
source install/setup.bash

# 默认参数
ros2 launch arm_pkg arm_track_box.launch.py

# 指定相机和校准文件
ros2 launch arm_pkg arm_track_box.launch.py \
  video_device:=/dev/video0 \
  calibration_file:=/home/zwr185/dev_ws/Seeed_RoboController/json/my_awesome_follower_arm.json \
  input_yaw_offset:=-90.0
```

**对应的源码文件**：

| 文件 | 作用 |
| --- | --- |
| `arm_pkg/arm_catch_joint_node.py` | 串口驱动 |
| `arm_pkg/arm_track_box_node.py` | 跟踪节点：目标平滑、限频 IK、发布关节角 |
| `arm_pkg/arm_joint_node.py` | 提供 `ServoController` |
| `arm_pkg/arm_calibration.py` | 角度换算与关节限位 |
| `arm_pkg/urdf_fk.py` | URDF 解析与正运动学 |
| `arm_pkg/urdf_ik.py` | 逆运动学 |
| `arm_pkg/urdf/arm.urdf.xacro` | 机械臂模型 |

**依赖的目录**：

| 目录 | 内容 / 作用 |
| --- | --- |
| `launch/` | 存放本 launch 文件 |
| `arm_pkg/` | 跟踪节点、驱动节点与公共模块 |
| `urdf/` | `arm.urdf.xacro` |
| `/home/zwr185/dev_ws/src/arm_msg/msg/` | `ArmJointAngles.msg` |
| `/home/zwr185/dev_ws/src/vision_pkg/config/` | `cam.yaml`、`box_detect.yaml`、`box_tf.yaml` |
| `/home/zwr185/dev_ws/src/vision_pkg/vision_pkg/` | 检测与坐标转换节点 |
| `/home/zwr185/dev_ws/src/vision_pkg/json/` | 像素→坐标标定文件 |
| `/home/zwr185/dev_ws/Seeed_RoboController/json/` | 默认校准文件 |
| `usb_cam` ROS 包 | 相机驱动 |
| `resource/` | ament 包索引标记 |

**调试参数**：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `video_device` | 空（自动探测） | 相机设备 |
| `calibration_file` | 从动臂校准 JSON | 关节限位 |
| `input_yaw_offset` | `-90.0` | 视觉坐标 yaw 旋转 |
| `solve_interval` | `0.5` | IK 求解周期（秒） |
| `z_offset` | `20.0` | 目标 z 方向抬升（cm） |
| `smooth_alpha` | `0.3` | 目标低通系数，越小越平滑 |
| `min_target_change` | `0.5` | 目标变化小于该值不重新求解（cm） |
| `ik_restarts` / `ik_max_iterations` | `4` / `300` | IK 重启次数和最大迭代次数 |

**话题**：

| 话题 | 作用 |
| --- | --- |
| `/box_world_pos` | 目标坐标输入 |
| `/arm_catch_angles` | 跟踪节点输出（5 关节 + 夹爪 0°） |
| `/arm_current_angles` | 驱动反馈（跟踪节点不订阅） |

**停止方式**：终端按 `Ctrl+C`。跟踪节点没有 `q` 快捷键，也不会因单个节点退出自动关闭其他节点。

---

### 3.4 三者对比

| 项目 | `arm_record_playback` | `arm_catch_box` | `arm_track_box` |
| --- | --- | --- | --- |
| 用途 | 录制/回放轨迹 | 视觉抓取放置 | 视觉目标跟踪 |
| 需要相机 | 否 | 是 | 是 |
| 需要 URDF/IK | 否 | 是 | 是 |
| 需要 CSV | 是 | 否 | 否 |
| 串口驱动 | `arm_playback_joint_node` | `arm_catch_joint_node` | `arm_catch_joint_node` |
| 键盘 | 无（GUI） | `s` / `g` / `q` | 无 |
| 退出是否关闭整个 launch | 否 | 是（按 `q` 时） | 否 |

---

## 4. 视觉调试配置

视觉抓取和跟踪共用 `vision_pkg/config/` 下的三份配置：

| 文件 | 常用调试项 |
| --- | --- |
| `cam.yaml` | `video_device`、`image_width`、`image_height`、`framerate` |
| `box_detect.yaml` | `hue_min/hue_max`、`sat_min/sat_max`、`val_min/val_max`、`min_area`、`show_window` |
| `box_tf.yaml` | `mode`、`camera_x/camera_y/camera_z`、`target_z`、`cm_per_pixel`、`x_direction/y_direction` |

常用调试命令：

```bash
ros2 topic echo /box_center          # 是否检测到色块
ros2 topic echo /box_world_pos       # 坐标是否合理
ros2 topic echo /arm_catch_angles    # IK 输出角度
ros2 topic hz /image_raw             # 相机帧率
ros2 node list
```

- 检测不到箱体：调 HSV 阈值和 `min_area`，并打开 `show_window: true` 查看 Mask。
- 坐标不对：检查 `camera_x/y/z`、`target_z`、`cm_per_pixel` 和 `input_yaw_offset`。
- IK 报误差过大：调整 `place_x/y/z`、`pick_z_offset`、`grasp_z_offset`，确认目标在工作空间内。

---

## 5. 常见问题

| 问题 | 处理 |
| --- | --- |
| 找不到 `arm_pkg` / `vision_pkg` | `colcon build` 后执行 `source install/setup.bash` |
| 打不开 `/dev/ttyACM0` | 检查设备是否存在、是否被其他程序占用、用户是否在 `dialout` 组 |
| 相机无图像 | 用 `v4l2-ctl --list-devices` 查设备，启动时加 `video_device:=/dev/videoX` |
| 检测不到箱体 | 调整 `box_detect.yaml` 的 HSV 和 `min_area`，确认光照稳定 |
| 机械臂不动 | 检查串口、校准文件、`/arm_catch_angles` 是否有数据 |
| 录制 GUI 没数据 | 确认 `/arm_joint_angles` 有约 10 Hz 输出 |
| 按 `q` 后整个 launch 退出 | `arm_catch_box` 的设计如此；只想暂停按 `s` |
| 跟踪模式夹爪不动 | 跟踪只控制 5 个臂关节，夹爪固定 0°，需夹取请用 `arm_catch_box` |

---

## License

Apache-2.0，详见 [LICENSE](LICENSE)。

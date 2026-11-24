<div align="center">

# SO-101 机械臂 ROS 2 工作空间

**从舵机标定、真机驱动、视觉抓取、主从遥操作到 MoveIt2 / Gazebo 仿真与语音控制的完整机械臂项目**

</div>

---

## 项目简介

本仓库围绕 **SO-ARM101（SO-101）六自由度机械臂 + 飞特 STS3215 总线舵机**，实现了从底层舵机调试到上层智能交互的完整功能链路，适用于教学演示、视觉抓取、遥操作、MoveIt2 规划和二次开发。

主要硬件：

- SO-101 机械臂，6 个 STS3215 舵机（ID 1–6）
- USB 转 TTL 串口，默认 `/dev/ttyACM0`，波特率 1 Mbps
- USB 相机
- Ubuntu + ROS 2

---

## 已实现功能

### 1. 舵机标定与维护
- 串口自动识别、舵机 ID 扫描与修改
- 舵机中位校准、中位测试、失能
- LeRobot 格式整臂校准，生成 `homing_offset / range_min / range_max`
- 角度限位、PID 参数读写与备份
- 图形化的工厂标定工具和校准向导

### 2. 机械臂驱动与轨迹
- ROS 2 话题 `/cmd_angles` 驱动 6 个舵机
- 当前角度反馈、平滑运动、遥操作跟随
- 轨迹录制为 CSV，并按原始节奏回放
- 提供 PyQt5 录制回放界面

### 3. 视觉检测与抓取
- USB 相机图像采集
- HSV 颜色阈值检测箱体/球体
- 像素坐标 → 机械臂底座坐标的透视变换标定
- IK 解算与视觉抓取放置流程
- 视觉目标跟踪（不抓取，仅跟随）

### 4. 主从遥操作
- 领导臂读取角度（50 Hz）
- 从动臂实时跟随，首帧平滑对齐
- 支持终端 `p` 键退出

### 5. 模型、可视化与真机同步
- SO-101 URDF 模型、STL 网格、RViz 配置
- 自定义关节控制 GUI
- 正运动学、数值 IK、`/solve_ik` 服务
- GUI 关节角转发到真机 `/cmd_angles`

### 6. MoveIt2 真机控制
- ST3215 的 ros2_control 硬件插件
- MoveIt2 配置、SRDF、OMPL/KDL 参数
- 真机抓取 GUI，支持固定姿态 IK 和固定 `wrist_roll` 两种模式
- 8 步抓取/放置流程

### 7. Gazebo 仿真
- Gazebo Harmonic + ros2_control 仿真
- MoveIt2 规划与 RViz MotionPlanning
- 自动九步抓取演示
- PyQt5 抓取界面，可设置木块、放置点和速度

### 8. 语音控制
- Whisper 语音识别
- DeepSeek 大模型决策
- MCP 工具调用
- 语音触发点头、摇头、视觉抓取、遥操作
- edge-tts 语音播报

---

## 功能包说明

| 路径 | 作用 |
| --- | --- |
| `Seeed_RoboController/` | 独立 Python 舵机标定工具集，非 ROS 包，提供工厂校准、LeRobot 校准和多种 GUI/CLI |
| `src/arm_msg/` | 机械臂消息与服务定义（`ArmJointAngles`、`RecordControl`） |
| `src/arm_pkg/` | 机械臂串口驱动、轨迹录制回放、视觉抓取与目标跟踪 |
| `src/remote_control_pkg/` | 领导臂—从动臂主从遥操作 |
| `src/vision_pkg/` | 相机、HSV 检测、像素坐标到测量坐标的标定与转换 |
| `src/arm_description/` | URDF 模型、RViz、自定义关节 GUI、IK 服务、仿真到真机转发 |
| `src/arm_hardware/` | STS3215 的 ros2_control 硬件接口插件 |
| `src/arm_moveit_config/` | 真机 MoveIt2 配置、控制链路和抓取界面 |
| `src/arm_gazebo_moveit2/` | Gazebo + MoveIt2 仿真与抓取演示 |
| `src/ai_pkg/` | 语音识别、DeepSeek、MCP 工具和语音动作控制 |

---

## 典型使用流程

### 首次使用机械臂
1. 用 `Seeed_RoboController` 扫描舵机、修改 ID、做中位校准。
2. 生成或更新 LeRobot 校准文件。
3. 用 `arm_pkg` 驱动机械臂，验证角度和方向。
4. 需要时用 `remote_control_pkg` 做主从遥操作。

### 视觉抓取
1. 用 `vision_pkg` 完成相机检测和像素坐标标定。
2. 用 `arm_pkg` 的 `arm_catch_box.launch.py` 执行抓取放置。
3. 或使用 `arm_track_box.launch.py` 只做目标跟踪。

### MoveIt2 真机抓取
1. 编译并启动 `arm_moveit_config`。
2. 用移动规划面板或抓取 GUI 控制真机。
3. 根据实机调整串口、限位和抓取偏移。

### Gazebo 仿真
1. 启动 `arm_gazebo_moveit2` 的 Gazebo / MoveIt2。
2. 运行自动抓取或 GUI 抓取演示，不需要真机。

### 语音控制
1. 配置 `DEEPSEEK_API_KEY`。
2. 启动 `ai_pkg` 的 `voice_control.launch.py`。
3. 按 `r` 说指令，触发点头、摇头、抓取或遥操作。

---

## 快速开始

```bash
cd /home/zwr185/dev_ws
colcon build --symlink-install
source install/setup.bash
```

常用启动命令：

```bash
# 轨迹录制回放
ros2 launch arm_pkg arm_record_playback.launch.py

# 视觉抓取
ros2 launch arm_pkg arm_catch_box.launch.py

# 主从遥操作
ros2 launch remote_control_pkg teleop.launch.py

# MoveIt2 真机
ros2 launch arm_moveit_config demo.launch.py

# Gazebo 仿真抓取
ros2 launch arm_gazebo_moveit2 grasp_demo_gui.launch.py

# 语音控制
ros2 launch ai_pkg voice_control.launch.py
```

> 各功能包的依赖安装、参数、话题和调试方法，请查看对应目录下的 `README.md`。

---

## 说明

- `Seeed_RoboController/` 是独立 Python 工具集，根目录的 `COLCON_IGNORE` 会让 colcon 跳过它。
- 代码中的串口、校准文件、相机设备等默认路径按当前工作空间配置，迁移到其他机器时需要修改。
- 各功能包许可证以各自 `package.xml` 或 `LICENSE` 为准。

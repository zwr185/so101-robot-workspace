<div align="center">

# ai_pkg

**语音识别 + DeepSeek 决策 + MCP 工具调用的机械臂语音控制功能包**

</div>

---

## 1. 功能简介

`ai_pkg` 把语音指令转换为机械臂动作：

1. `llm_agent_node`：按 `r` 录音，Whisper 识别为文字，DeepSeek 通过 function calling 选择工具，再用 edge-tts 播报结果。
2. `mcp_server_node`：基于 FastMCP 提供 `robot_nod`、`robot_shake_head`、`robot_remote_control`、`robot_grasp_box` 四个工具。
3. `robot_action_manager_node`：按需启动/关闭点头、摇头、抓取、遥操作节点，避免动作节点常驻占用串口和 CPU。
4. `robot_catch_box_node`：按需启动相机、检测和坐标转换，把目标坐标转发给 `arm_pkg` 的抓取节点。
5. `robot_nod_node` / `robot_shake_head_node`：回放预录的点头/摇头 CSV 轨迹。
6. `robot_remote_control_node`：把语音开关转换为 `/teleop_enable`。

机械臂串口驱动复用 `arm_pkg/arm_playback_joint_node`。

---

## 2. 安装依赖

### 2.1 需要安装的库

| 依赖 | 用途 |
| --- | --- |
| `rclpy`、`std_msgs`、`geometry_msgs` | ROS 2 节点与话题 |
| `arm_msg`、`arm_pkg` | 机械臂消息、串口驱动、抓取节点 |
| `vision_pkg`、`usb_cam` | 相机、颜色检测、坐标转换 |
| `remote_control_pkg` | 遥操作领导臂节点 |
| `pyaudio`、`portaudio19-dev` | 录音 |
| `openai-whisper` | 语音识别（会自动安装 PyTorch） |
| `openai` | 调用 DeepSeek 兼容接口 |
| `fastmcp` | MCP Server / Client |
| `edge-tts` | 语音合成 |
| `ffmpeg` | edge-tts 播放/转码依赖 |

### 2.2 安装与编译

```bash
sudo apt install portaudio19-dev ffmpeg \
                 ros-$ROS_DISTRO-usb-cam \
                 ros-$ROS_DISTRO-cv-bridge \
                 python3-opencv \
                 python3-numpy

pip install pyaudio openai-whisper edge-tts fastmcp openai

cd /home/zwr185/dev_ws
colcon build --symlink-install \
  --packages-select arm_msg arm_pkg vision_pkg remote_control_pkg ai_pkg
source install/setup.bash
```

### 2.3 配置 DeepSeek API Key

```bash
export DEEPSEEK_API_KEY="sk-你的密钥"
```

建议写入 `~/.bashrc`。首次运行 Whisper 会下载模型（默认 `large-v3-turbo`），需要网络和较大磁盘空间。

### 2.4 音频设备

```bash
# 检查麦克风和扬声器
arecord -l
aplay -l
```

---

## 3. Launch 文件

### 3.1 voice_control.launch.py

**功能**：启动语音控制常驻节点：

| 节点 | 作用 |
| --- | --- |
| `arm_playback_joint_node`（arm_pkg） | 从动臂唯一串口驱动：回放、视觉抓取、遥操作共用 |
| `robot_remote_control_node` | `/robot_remote_control` → `/teleop_enable` |
| `robot_action_manager_node` | 按语音工具请求按需拉起动作节点 |
| `llm_agent_node` | 录音、Whisper、DeepSeek、MCP、TTS |

动作节点（点头、摇头、抓取、领导臂）**不常驻**，由 `robot_action_manager_node` 在需要时启动，结束后关闭。

**启动命令**：

```bash
cd /home/zwr185/dev_ws
source install/setup.bash
export DEEPSEEK_API_KEY="sk-你的密钥"

# 默认相机自动探测
ros2 launch ai_pkg voice_control.launch.py

# 指定相机、校准文件和抓取偏移
ros2 launch ai_pkg voice_control.launch.py \
  video_device:=/dev/video0 \
  calibration_file:=/home/zwr185/dev_ws/Seeed_RoboController/json/my_awesome_follower_arm.json \
  input_yaw_offset:=-90.0 \
  grasp_offset_x:=-2.0 \
  grasp_offset_y:=5.0
```

**对应的源码文件**：

| 文件 | 作用 |
| --- | --- |
| `ai_pkg/llm_agent_node.py` | 语音识别、LLM 决策、MCP 调用、TTS |
| `ai_pkg/mcp_server_node.py` | FastMCP 工具服务（被 Agent 自动拉起） |
| `ai_pkg/robot_action_manager_node.py` | 按需启动/停止动作节点 |
| `ai_pkg/robot_remote_control_node.py` | 遥操作使能开关 |
| `ai_pkg/robot_catch_box_node.py` | 按需启动视觉链路并触发抓取 |
| `ai_pkg/robot_nod_node.py` | 点头轨迹回放 |
| `ai_pkg/robot_shake_head_node.py` | 摇头轨迹回放 |
| `arm_pkg/arm_playback_joint_node.py` | 串口驱动（外部包） |
| `remote_control_pkg/leader_arm_joint.py` | 领导臂读取（外部包，按需启动） |

**依赖的目录**：

| 目录 | 内容 / 作用 |
| --- | --- |
| `launch/` | 存放本 launch 文件 |
| `ai_pkg/` | Agent、MCP、动作管理、动作节点 |
| `test/` | 录音、ASR、TTS、DeepSeek、MCP 的独立示例脚本 |
| `tmp/` | 录音 `ai_pkg_output.wav`、TTS `ai_pkg_tts.mp3` |
| `/home/zwr185/dev_ws/src/arm_pkg/csv/` | 点头/摇头轨迹 CSV |
| `/home/zwr185/dev_ws/src/arm_pkg/` | 串口驱动、抓取节点 |
| `/home/zwr185/dev_ws/src/vision_pkg/config/` | 相机、检测、坐标转换参数 |
| `/home/zwr185/dev_ws/src/remote_control_pkg/` | 领导臂节点 |
| `/home/zwr185/dev_ws/Seeed_RoboController/json/` | 校准文件 |
| `resource/` | ament 包索引标记 |

**话题**：

| 话题 | 类型 | 作用 |
| --- | --- | --- |
| `/robot_action/nod` | `std_msgs/Bool` | MCP 请求点头 |
| `/robot_action/shake_head` | `std_msgs/Bool` | MCP 请求摇头 |
| `/robot_action/grasp_box` | `std_msgs/Bool` | MCP 请求抓取 |
| `/robot_action/remote_control` | `std_msgs/Bool` | MCP 请求开/关遥操作 |
| `/robot_nod_done`、`/robot_shake_head_done`、`/robot_catch_box_done` | `std_msgs/Bool` | 动作完成信号 |
| `/teleop_enable` | `std_msgs/Bool` | 遥操作使能 |
| `/leader_joint_angles` | `ArmJointAngles` | 领导臂角度 |
| `/box_world_pos_raw` | `geometry_msgs/Point` | 视觉原始坐标 |
| `/box_world_pos` | `geometry_msgs/Point` | 转发给抓取节点的目标 |
| `/arm_catch_angles` | `ArmJointAngles` | 抓取/跟踪角度 |
| `/arm_catch_done` | `std_msgs/Bool` | 抓取结果 |

**Launch 参数**：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `video_device` | 空（自动探测） | 相机设备 |
| `vision_idle_timeout` | `30.0` | 抓取结束后视觉链路保留时间（秒） |
| `calibration_file` | 从动臂校准 JSON | 角度映射与关节限位 |
| `input_yaw_offset` | `-90.0` | 视觉坐标绕底座 z 轴旋转角 |
| `grasp_offset_x` / `grasp_offset_y` | `-2.0` / `5.0` | 抓取点偏移（cm） |

**语音与按键**：

| 操作 | 作用 |
| --- | --- |
| 按 `r` | 开始录音 |
| 按 `s` | 停止录音；抓取过程中由抓取节点处理为停止抓取 |
| 按 `g` | 抓取过程中重新抓取（由 `arm_catch_box_node` 处理） |
| 按 `q` | 退出程序 |
| 说“点个头” | 触发点头轨迹 |
| 说“摇个头” | 触发摇头轨迹 |
| 说“抓取箱子/抓取球/抓取物体” | 触发视觉抓取 |
| 说“开启遥操作/停止遥操作” | 开关主从遥操作 |

**使用流程**：

1. 启动后终端提示按键和语音指令。
2. 按 `r` 说指令，再按 `s` 结束录音。
3. Agent 识别并调用 MCP 工具；动作节点按需启动。
4. 动作完成后 Agent 播报结果。
5. 抓取结束后视觉链路在 `vision_idle_timeout` 后自动关闭。

**调试命令**：

```bash
ros2 node list
ros2 topic echo /robot_action/nod
ros2 topic echo /robot_catch_box_done
ros2 topic echo /teleop_enable
ros2 run ai_pkg mcp_server_node --ros-args -p http:=true
# 浏览器打开 http://127.0.0.1:8000/mcp 调试工具
```

---

## 4. 其他节点与调试配置

### 4.1 可单独运行的节点

| 节点 | 命令 | 说明 |
| --- | --- | --- |
| MCP Server（stdio） | `ros2 run ai_pkg mcp_server_node` | 等待 MCP 客户端连接；终端按 `q` 退出 |
| MCP Server（HTTP） | `ros2 run ai_pkg mcp_server_node --ros-args -p http:=true` | 默认 `127.0.0.1:8000/mcp` |
| 语音 Agent | `ros2 run ai_pkg llm_agent_node` | 自动拉起本地 MCP Server |
| 点头 | `ros2 run ai_pkg robot_nod_node` | 等待 `/robot_nod` 触发 |
| 摇头 | `ros2 run ai_pkg robot_shake_head_node` | 等待 `/robot_shake_head` 触发 |
| 抓取衔接 | `ros2 run ai_pkg robot_catch_box_node` | 等待 `/robot_catch_box` 触发 |
| 遥操作开关 | `ros2 run ai_pkg robot_remote_control_node` | 转发 `/teleop_enable` |

### 4.2 关键调试参数

| 节点 | 参数 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `llm_agent_node` | `whisper_model` | `large-v3-turbo` | 识别模型；显存不足可改 `small` |
| `llm_agent_node` | `llm_model` | `deepseek-chat` | DeepSeek 模型 |
| `llm_agent_node` | `base_url` | `https://api.deepseek.com` | API 地址 |
| `llm_agent_node` | `mcp_url` | 空 | 空=自动 stdio；可填 HTTP MCP 地址 |
| `llm_agent_node` | `tts_voice` | `zh-CN-XiaoyiNeural` | edge-tts 音色 |
| `action_manager` | `startup_delay` | `1.0` | 动作节点启动等待（秒） |
| `action_manager` | `grasp_startup_delay` | `2.0` | 抓取节点启动等待（秒） |
| `action_manager` | `startup_timeout` | `10.0` | 等待动作节点订阅触发话题的最长时间（秒） |
| `action_manager` | `vision_idle_timeout` | `30.0` | 视觉空闲关闭时间（秒） |
| `robot_catch_box_node` | `detect_timeout` | `15.0` | 等待检测到目标的最长时间（秒） |
| `robot_catch_box_node` | `manage_vision` | `true` | 是否由本节点启停视觉链路 |
| `robot_nod_node` | `nod_file` | `nod_trajectory.csv` | 点头轨迹文件 |
| `robot_shake_head_node` | `shake_file` | `shakehead_trajectory.csv` | 摇头轨迹文件 |
| `robot_nod_node` / `shake` | `transition_duration` | `1.5` | 从当前姿态过渡到轨迹首帧的时间（秒） |
| `robot_remote_control_node` | `leader_timeout` | `1.0` | 领导臂断流超时（秒） |
| `mcp_server_node` | `http` | `false` | 是否启用 HTTP MCP |
| `mcp_server_node` | `host` / `port` | `127.0.0.1` / `8000` | HTTP 监听地址 |

> 点头/摇头轨迹必须先用 `arm_pkg` 的录制功能生成，并放在 `/home/zwr185/dev_ws/src/arm_pkg/csv/`。

---

## 5. 常见问题

| 问题 | 处理 |
| --- | --- |
| 提示未设置 API Key | 执行 `export DEEPSEEK_API_KEY="sk-..."` 后重启 |
| 录音失败 | 检查麦克风、`pyaudio` 和 `portaudio19-dev` |
| 识别很慢 | 换更小的 `whisper_model`，或确认模型已下载 |
| 没有语音播报 | 检查 `ffmpeg`、扬声器、`aplay -l` |
| 抓取一直超时 | 检查 `/box_world_pos_raw`、相机、光照和 HSV 阈值 |
| 点头/摇头不执行 | 检查 `arm_pkg/csv/` 中轨迹文件是否存在 |
| 遥操作无法开启 | 确认 `remote_control_pkg` 已编译，领导臂串口可用 |
| MCP 工具无响应 | 检查 `/robot_action/*` 是否有消息，动作管理器是否在运行 |
| 按 `r`/`s`/`q` 无反应 | 需要可用终端；从 IDE 启动时可能不可用 |

---

## License

Apache-2.0。

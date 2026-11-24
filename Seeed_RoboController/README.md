<div align="center">

# Seeed_RoboController

**Seeed Studio SoARM 10X 系列机械臂 · FTServo 舵机校准与控制工具集**

*FTServo servo factory calibration, LeRobot calibration and motion control toolkit for Seeed Studio SoARM 10X robotic arms*

![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20Ubuntu%20%7C%20macOS-blue)
![Python](https://img.shields.io/badge/python-3.8+-green)
![GUI](https://img.shields.io/badge/GUI-PySide6-41cd52)
![Bus](https://img.shields.io/badge/bus-Feetech%20STS%2FSMS-orange)

</div>

---

## 目录

- [1. 项目概述](#1-项目概述)
  - [1.1 核心用途](#11-核心用途)
  - [1.2 主要功能特性](#12-主要功能特性)
  - [1.3 适用场景](#13-适用场景)
  - [1.4 整体定位与边界](#14-整体定位与边界)
  - [1.5 硬件与协议约定](#15-硬件与协议约定)
- [2. 完整目录结构说明](#2-完整目录结构说明)
  - [2.1 结构总览](#21-结构总览)
  - [2.2 根目录文件](#22-根目录文件)
  - [2.3 json/](#23-json)
  - [2.4 scservo_sdk/](#24-scservo_sdk)
  - [2.5 src/ 公共模块](#25-src-公共模块)
  - [2.6 src/tools/](#26-srctools)
  - [2.7 src/gui/ 概览](#27-srcgui-概览)
- [3. src/gui 目录专项说明](#3-srcgui-目录专项说明)
  - [3.1 功能定位](#31-功能定位)
  - [3.2 内部子结构](#32-内部子结构)
  - [3.3 核心实现逻辑](#33-核心实现逻辑)
  - [3.4 在功能包中的作用与依赖关系](#34-在功能包中的作用与依赖关系)
  - [3.5 使用注意事项](#35-使用注意事项)
- [4. 快速开始](#4-快速开始)
  - [4.1 环境要求](#41-环境要求)
  - [4.2 安装与自检](#42-安装与自检)
  - [4.3 常用启动命令](#43-常用启动命令)
  - [4.4 典型工作流](#44-典型工作流)
- [5. 校准文件格式](#5-校准文件格式)
- [6. 故障排查](#6-故障排查)
- [7. 开发与扩展约定](#7-开发与扩展约定)

---

## 1. 项目概述

### 1.1 核心用途

`Seeed_RoboController` 是一套面向 **Seeed Studio SoARM 10X 系列（SO-ARM100/101）六自由度机械臂** 的 Python 工具集，围绕飞特（Feetech）**STS/SMS 系列串口总线舵机**提供从设备发现、出厂标定、整臂校准到角度控制、参数维护和故障诊断的完整闭环。

工具集的核心工作是把「舵机原始编码器计数」与「机械臂关节角度」建立可靠的对应关系，并生成 **LeRobot 兼容的 JSON 校准文件**，使机械臂可以被 LeRobot 等上层框架直接使用。

### 1.2 主要功能特性

| 能力分类 | 具体功能 |
| --- | --- |
| 设备发现 | 串口自动识别与优先级排序、舵机 ID 扫描（工厂 GUI 扫描 1–20，`scan_id.py` 扫描 1–252）、单舵机 ID 修改 |
| 出厂标定 | 快速中位校准（当前位置设为 2048）、标准中位校准（含电压/温度安全告警）、中位测试、舵机失能 |
| 整臂校准 | 图形化 LeRobot 校准向导、命令行 LeRobot 校准、逐关节记录 `homing_offset` / `range_min` / `range_max` |
| 运动控制 | 按校准文件回到中位、手动输入全部关节角度、余弦缓动平滑运动、单关节滑块控制、实时角度曲线 |
| 参数维护 | 角度限位读写（EEPROM）、PID（P/D/I）读写与备份恢复、波特率修改、寄存器读写、恢复出厂设置 |
| 遥操作 | 双端口主从同步遥控（主控臂读角度 → 从控臂同 ID 跟随） |
| 集成工具 | 双串口工厂标定 GUI（三标签页：舵机标定 / 单舵机控制 / 高级工具） |
| 诊断 | 舵机扫描、寄存器诊断、夹爪专项诊断、全关节小幅运动验证 |
| 跨平台 | Windows / Ubuntu / macOS；统一浅色主题，规避 Windows 深色模式文字不可读问题 |

### 1.3 适用场景

- **产线与出厂标定**：批量舵机改 ID、设中位、写入限位、调 PID 并验证。
- **整机装配后校准**：为从动臂（follower）或主动臂（leader）生成 LeRobot 校准文件。
- **LeRobot 数据采集前准备**：校准机械臂后，将关节回到中位再开始遥操作采集。
- **故障排查**：定位「舵机不转 / 角度范围异常 / 串口无响应 / 夹爪不动」等问题。
- **教学与二次开发**：作为 STS/SMS 总线舵机 Python 控制与 GUI 开发的参考实现。

### 1.4 整体定位与边界

- 本目录是**独立的 Python 工具集，不是 ROS 2 功能包**。根目录的 `COLCON_IGNORE` 会让 `colcon` 直接跳过本目录，因此它可以安全地放置在 ROS 工作空间中而不参与构建。
- 依赖方向是单向的：**GUI / CLI 工具 → `src` 公共模块 → `scservo_sdk` → `pyserial` → USB 转 TTL 串口**。
- 与 LeRobot 的关系：校准文件的字段和目录约定与 LeRobot 一致，可直接配合 LeRobot 使用；本项目本身不包含 LeRobot 训练/推理代码。
- 根目录的 `setup.py` **不是** setuptools 打包脚本，而是环境自检脚本，必须以 `python setup.py` 方式运行。

### 1.5 硬件与协议约定

| 项目 | 约定 |
| --- | --- |
| 机械臂 | SoARM 10X / SO-ARM100 / SO-ARM101，6 自由度 |
| 舵机 | Feetech SMS/STS 系列，以 STS3215 为主，TTL 半双工总线 |
| 默认波特率 | `1000000`（1 Mbps） |
| 位置量程 | `0 – 4095`（12 bit），`2048` 为中位，`4096` 计数 = `360°` |
| 角度分辨率 | `360 / 4096 ≈ 0.0879°` / 计数 |
| 标准关节 ID | 1 `shoulder_pan`、2 `shoulder_lift`、3 `elbow_flex`、4 `wrist_flex`、5 `wrist_roll`、6 `gripper` |
| 连续旋转关节 | `wrist_roll`（ID 5），校准范围固定为 `[0, 4095]`；从动臂样例中 `drive_mode = 1` |
| 供电 | 5V 系统（4.5–5.5V）或 12V 系统（10.5–13.5V），工具按实测电压自动区分 |
| 安全温度 | 默认上限 `60°C` |

---

## 2. 完整目录结构说明

### 2.1 结构总览

```text
Seeed_RoboController/
├── README.md                        # 项目说明文档（本文件）
├── requirements.txt                 # Python 依赖清单
├── setup.py                         # 环境自检脚本（非打包脚本）
├── COLCON_IGNORE                    # 空文件，使 colcon 跳过本目录
├── .gitignore                       # Git 忽略规则
│
├── json/                            # 校准文件样例与运行时备份
│   ├── my_awesome_follower_arm.json # 从动臂校准样例
│   └── my_awesome_leader_arm.json   # 主动臂校准样例
│
├── scservo_sdk/                     # Feetech 官方 SCServo Python SDK
│   ├── __init__.py
│   ├── port_handler.py
│   ├── protocol_packet_handler.py
│   ├── group_sync_read.py
│   ├── group_sync_write.py
│   ├── sms_sts.py
│   ├── scscl.py
│   ├── hls.py
│   └── scservo_def.py
│
└── src/
    ├── port_utils.py                # 跨平台串口工具
    ├── calibration_manager.py       # LeRobot 校准文件管理
    │
    ├── tools/                       # 命令行工具集
    │   ├── scan_id.py
    │   ├── change_single_servo_id.py
    │   ├── servo_middle_calibration.py
    │   ├── servo_quick_calibration.py
    │   ├── servo_center_test.py
    │   ├── servo_disable.py
    │   ├── servo_remote_control.py
    │   ├── servo_pid_tune.py
    │   ├── servo_register_diag.py
    │   ├── lerobot_calibrate.py
    │   └── run_calibration_middle.py
    │
    └── gui/                         # 图形界面与可视化工具（详见第 3 章）
        ├── factory_calibration_tool.py
        ├── calibration_wizard.py
        ├── servo_angle_limit_set.py
        ├── servo_pid_tune_gui.py
        ├── 01_read_angle.py
        ├── 02_write_angle.py
        ├── 03_write_all_angle.py
        ├── 04_write_angle_stable.py
        ├── diag6.py
        ├── test_all_joints.py
        └── theme_utils.py
```

> 运行后 Python 会自动生成 `__pycache__/`、`*.pyc` 等缓存目录/文件，已由 `.gitignore` 忽略，不属于源码结构。

### 2.2 根目录文件

| 文件 | 作用 |
| --- | --- |
| `README.md` | 项目总览、目录说明、GUI 专项说明、使用与维护指南。 |
| `requirements.txt` | 运行依赖：`PySide6>=6.0.0`、`pyserial>=3.5`。注意 `01_read_angle.py` / `02_write_angle.py` 还需要 `numpy` 与 `matplotlib`，二者未列入该文件。 |
| `setup.py` | 环境自检脚本。检查 Python 版本、`PySide6` / `pyserial` 是否可导入、项目模块与 SDK 是否完整，并打印常用命令。必须在项目根目录执行：`python setup.py`。 |
| `COLCON_IGNORE` | 空文件。用于让 `colcon` 忽略本目录，明确本工具集不参与 ROS 2 构建。 |
| `.gitignore` | 忽略 `__pycache__`、虚拟环境、IDE 配置、系统缓存文件等。 |

### 2.3 json/

存放 LeRobot 格式的整臂校准文件，同时也会被运行时工具写入。

| 文件 / 模式 | 作用 |
| --- | --- |
| `my_awesome_follower_arm.json` | 从动臂（follower）校准样例。包含 6 个关节的 `id`、`drive_mode`、`homing_offset`、`range_min`、`range_max`；其中 `wrist_roll` 为 `drive_mode = 1`，范围为 `0–4095`。 |
| `my_awesome_leader_arm.json` | 主动臂（leader）校准样例，字段结构相同。`lerobot_calibrate.py` 校准主动臂时会额外复制一份到该路径。 |
| `servo_pid_backup_*.json` | **运行时生成**。`servo_pid_tune_gui.py` 写入 PID 前会自动备份原值到本目录，用于一键恢复。 |

### 2.4 scservo_sdk/

Feetech 官方 SCServo Python SDK，封装串口通信、协议打包解析和舵机寄存器操作，是本项目所有舵机控制能力的底座。

| 文件 | 作用 |
| --- | --- |
| `__init__.py` | 汇总导出各子模块，支持 `from scservo_sdk import ...`。 |
| `port_handler.py` | 串口句柄管理：打开/关闭端口、设置波特率、超时与延迟控制。默认波特率 `1000000`。 |
| `protocol_packet_handler.py` | SCS 协议核心：报文头/长度/校验和、指令打包与解析，`ping`、`read`、`write`、`syncRead`、`syncWrite` 等，以及电压/过热/过载等错误位定义。 |
| `group_sync_read.py` | 多舵机同步读：一次事务读取多个 ID 的连续寄存器。 |
| `group_sync_write.py` | 多舵机同步写：一次事务向多个 ID 下发相同寄存器区间。 |
| `sms_sts.py` | SMS/STS 系列高层 API：`WritePosEx`（位置+速度+加速度）、`ReadPos`、力矩开关、PID/限位/ID/波特率读写、EEPROM 解锁/加锁等。 |
| `scscl.py` | SCS 系列舵机高层 API（保留兼容）。 |
| `hls.py` | HLS 系列舵机高层 API（保留兼容）。 |
| `scservo_def.py` | 协议常量：广播 ID `0xFE`、最大 ID `0xFC`、指令码（`INST_PING` / `INST_READ` / `INST_WRITE` / `INST_SYNC_READ` / `INST_SYNC_WRITE` / `INST_RESET` 等）、通信结果码。 |

### 2.5 src/ 公共模块

| 文件 | 作用 |
| --- | --- |
| `src/port_utils.py` | 跨平台串口工具。`get_available_ports()` 按平台优先级排序并过滤虚拟串口（Linux 过滤 `ttyS*`，macOS 过滤 debug/蓝牙设备，Windows 按 COM 编号排序）；`get_default_port(index)` 获取默认端口；`list_ports_for_user()` 输出可读列表；`find_port_by_description()` 按描述查找；`select_port_interactive()` 提供命令行交互式选口；`get_platform_info()` 返回平台信息。 |
| `src/calibration_manager.py` | LeRobot 校准文件管理器。默认根目录 `~/.cache/huggingface/lerobot/calibration/`，分别管理 `robots/`（从动臂）与 `teleoperators/`（主动臂）。提供校准文件枚举、读取、保存、按关节名生成可读摘要，以及中英关节名映射 `JOINT_NAME_MAP`。 |

> `src/` 下没有 `__init__.py`，依赖 Python 3 的命名空间包机制。因此**所有 `python -m ...` 命令都必须在项目根目录执行**，以保证 `src` 与 `scservo_sdk` 可被导入。

### 2.6 src/tools/

命令行工具集，既可单独使用，也会被 GUI 以子进程方式调用。

| 文件 | 功能 | 常用命令 |
| --- | --- | --- |
| `scan_id.py` | 扫描指定端口上的所有舵机（ID 1–252），打印型号；无参数时交互式选口。 | `python -m src.tools.scan_id`<br>`python -m src.tools.scan_id --list` |
| `change_single_servo_id.py` | 扫描总线上的单个舵机并修改其 ID：解锁 EEPROM → 写地址 `5` → 校验新 ID。 | `python -m src.tools.change_single_servo_id` |
| `servo_middle_calibration.py` | 标准中位校准（交互式/自动两种模式）：扫描舵机、读取电压/温度并给出安全告警，将当前位置标定为中位 `2048`。向 GUI 导出 `get_voltage_range()` 与 `SAFE_TEMPERATURE_MAX`。 | `python -m src.tools.servo_middle_calibration` |
| `servo_quick_calibration.py` | 快速中位校准：失能全部舵机后直接把当前位置设为 `2048`，适合产线快速作业。工厂 GUI 的「中位校准」优先调用此脚本。 | `python -m src.tools.servo_quick_calibration /dev/ttyACM0` |
| `servo_center_test.py` | 中位测试：使能力矩并把舵机移动到 `2048`，用于验证中位校准结果。 | `python -m src.tools.servo_center_test` |
| `servo_disable.py` | 失能全部舵机（力矩置 0），便于手动摆臂与示教。 | `python -m src.tools.servo_disable` |
| `servo_remote_control.py` | 双端口主从同步遥控：从主控端口读取角度，实时写给从控端口相同 ID 的舵机，循环周期约 10ms。 | `python -m src.tools.servo_remote_control --read-port /dev/ttyACM0 --control-port /dev/ttyACM1` |
| `servo_pid_tune.py` | 命令行 PID 读写：寄存器 `0x15/0x16/0x17`（21/22/23），写入前备份原值到 `json/`。 | `python -m src.tools.servo_pid_tune /dev/ttyACM0` |
| `servo_register_diag.py` | 寄存器诊断：检查 `current`、`moving` 等寄存器是否可读、数值是否随运动变化。 | `python -m src.tools.servo_register_diag` |
| `lerobot_calibrate.py` | 命令行 LeRobot 校准：逐关节记录中位 `homing_offset`，再记录运动范围 `range_min/range_max`，保存到 LeRobot 缓存目录；主动臂额外复制到 `json/my_awesome_leader_arm.json`。 | `python -m src.tools.lerobot_calibrate` |
| `run_calibration_middle.py` | 按校准文件把所有关节驱动到中位。`--mode zero` 使用 `homing_offset`，`--mode range` 使用范围中点。 | `python -m src.tools.run_calibration_middle json/my_awesome_follower_arm.json /dev/ttyACM0 --mode zero` |

### 2.7 src/gui/ 概览

| 文件 | 类型 | 一句话说明 |
| --- | --- | --- |
| `factory_calibration_tool.py` | PySide6 集成 GUI | 双串口工厂标定主工具，含舵机标定、单舵机控制、高级工具三个标签页。 |
| `calibration_wizard.py` | PySide6 对话框 | 图形化 LeRobot 整臂校准向导，逐关节记录中位与范围。 |
| `servo_angle_limit_set.py` | PySide6 GUI | 实时采集舵机位置并写入角度限位（EEPROM）。 |
| `servo_pid_tune_gui.py` | PySide6 GUI | PID 可视化调参，支持备份与恢复。 |
| `01_read_angle.py` | matplotlib | 实时柱状图显示 6 个舵机角度。 |
| `02_write_angle.py` | matplotlib | 滑块控制全部关节角度（基于校准文件）。 |
| `03_write_all_angle.py` | PySide6 GUI | 手动输入全部关节角度并一次下发。 |
| `04_write_angle_stable.py` | PySide6 GUI | 在 03 基础上增加余弦缓动平滑轨迹。 |
| `diag6.py` | 一次性脚本 | 夹爪（ID 6）专项诊断。 |
| `test_all_joints.py` | 一次性脚本 | 逐个验证 1–5 号关节小幅运动并回位。 |
| `theme_utils.py` | 公共模块 | 强制 Fusion 浅色主题。 |

---

## 3. src/gui 目录专项说明

### 3.1 功能定位

`src/gui` 是整个功能包的**人机交互层与现场操作入口**，面向产线标定人员、装配调试人员和开发者，把底层的串口/寄存器操作封装为可视化界面。它承担以下职责：

1. **设备连接与状态可视化**：串口选择、舵机在线扫描、位置/电压/温度/负载/电流/运动状态实时显示。
2. **标定作业**：中位校准、ID 修改、整臂 LeRobot 校准、角度限位写入、PID 调参。
3. **运动验证**：单关节控制、全部关节角度下发、平滑轨迹运动、中位测试。
4. **系统集成**：双串口工厂工具将多个 CLI 工具编排为一个统一界面，并处理串口独占与进程生命周期。
5. **诊断入口**：提供一次性脚本快速验证夹爪与各关节是否正常。

该目录只负责「交互与流程编排」，不重新实现协议栈：除 `01–04` 为演示/教学目的使用 `pyserial` 自行组包外，其余 GUI 均通过 `scservo_sdk` 访问舵机。

### 3.2 内部子结构

按职责可划分为六组：

| 分组 | 文件 | 说明 |
| --- | --- | --- |
| 集成式工厂工具 | `factory_calibration_tool.py` | 唯一支持双端口并行操作的主界面，内部包含扫描线程、面板、对话框和子进程编排。 |
| 整臂校准 | `calibration_wizard.py` | 独立 QDialog，引导式完成 6 关节校准并保存 JSON。 |
| 参数维护 | `servo_angle_limit_set.py`、`servo_pid_tune_gui.py` | 分别维护机械行程限位与闭环 PID 参数，均涉及 EEPROM 写入。 |
| 角度控制 | `01_read_angle.py`、`02_write_angle.py`、`03_write_all_angle.py`、`04_write_angle_stable.py` | 从「只读可视化」到「直接下发」再到「平滑轨迹」的递进式控制工具。 |
| 现场诊断 | `diag6.py`、`test_all_joints.py` | 硬编码端口的极简脚本，用于快速定位单点故障。 |
| 公共支撑 | `theme_utils.py` | 统一浅色主题，被多个 PySide6 界面复用。 |

### 3.3 核心实现逻辑

#### 3.3.1 分层与依赖

```text
┌────────────────────────────── src/gui ──────────────────────────────┐
│  PySide6 窗口 / 对话框            matplotlib 可视化（01、02）        │
│  QThread / QObject Worker + Signal/Slot 异步通信                     │
│  subprocess 调用 src/tools 下的 CLI（隔离串口占用与阻塞操作）        │
└───────────────┬───────────────────────────────┬─────────────────────┘
                │                               │
     scservo_sdk（PortHandler + sms_sts）   pyserial 自实现 SCS 组包
                │                         （01/02/03/04 使用）
                ▼
        USB-TTL 串口 → STS/SMS 总线舵机（ID 1–6）
```

- **线程模型**：耗时的串口扫描、寄存器读写放在 `QThread` / 工作对象中，通过 `Signal` 把结果推回 UI，避免界面卡死。
- **数据模型**：关节角度与原始位置之间的换算统一由校准文件的 `homing_offset`、`drive_mode`、`range_min`、`range_max` 决定（见第 5 章）。
- **进程模型**：工厂工具需要调用外部 CLI 时使用 `subprocess`，因为同一个串口不能被两个进程同时打开；工具通过停止扫描线程、终止旧子进程、代次号校验来保证串口一定被释放。

#### 3.3.2 双串口工厂标定工具 `factory_calibration_tool.py`

这是功能包中最核心、体量最大的 GUI（约 3900 行），围绕「两个独立串口 + 两个独立舵机面板」构建。

**主要类与职责**

| 类 | 职责 |
| --- | --- |
| `RemoteControlWorker` | 以子进程方式启动/停止 `src.tools.servo_remote_control`，监控输出并把日志转发到界面。 |
| `IDChangeDialog` | 修改舵机 ID：暂停扫描 → 解锁 EEPROM → 写地址 `5` → 校验 → 恢复扫描。 |
| `ServoWorker` | 单个端口的后台工作对象：连接串口、扫描 ID 1–20、周期读取状态、执行力矩/中位/限位/目标位置等命令。 |
| `ServoPanel` | 单个端口的 UI 面板：舵机卡片（`Pos`/`V`/`T`/`Spd`/`Load`/`Cur`/`Mov`/`Model`/`Status`）、力矩开关、中位、清限位、目标位置滑条、ID 标定入口。 |
| `AdvancedToolsPanel` | 「高级工具」页：指定 ID 的寄存器读取/写入（1/2 字节）、波特率修改、恢复出厂设置。 |
| `EZToolUI` | 主窗口：串口选择、双面板布局、标签页、遥控按钮、状态栏与操作日志。 |

**三个标签页**

1. **舵机标定**：左右两个 `ServoPanel`，分别绑定串口 1 / 串口 2；每个面板独立扫描与刷新。
2. **单舵机控制**：对选中 ID 做精细操作（力矩、中位、限位、目标位置）。
3. **高级工具**：寄存器级读写、波特率修改、恢复出厂。

**关键实现细节**

- **自动扫描与热插拔**：扫描范围固定为 ID `1–20`；每隔 2 秒刷新串口列表；连续 5 次扫描为空时判定串口断开并尝试重连。
- **串口独占处理**：执行「中位校准 / 中位测试 / 失能」前先停止对应面板的扫描线程，等待端口释放；通过 `_tool_gen` 代次号判断操作是否已被更新的操作取代，避免旧线程错误地重启扫描。
- **外部工具编排**：按顺序尝试 `servo_quick_calibration.py`，缺失时回退到 `servo_middle_calibration.py`；中位测试调用 `servo_center_test.py`；失能调用 `servo_disable.py`；遥控调用 `servo_remote_control.py`，均以 `python -m src.tools.xxx` 方式启动并实时收集输出。
- **中位校准原理**：向力矩/校准地址 `40` 写入命令值 `128`，舵机把当前位置记为 `2048`；「中位测试」则使能力矩并移动到 `2048`。
- **ID 修改**：写 ID 前必须 `unLockEprom()`，写入后校验新 ID 是否可 `ping` 通，再 `lockEprom()`。
- **日志与可见性**：子进程输出按关键词过滤后写入操作日志；Windows 下强制 `PYTHONIOENCODING=utf-8` 避免 emoji 输出导致子进程异常。

#### 3.3.3 整臂校准向导 `calibration_wizard.py`

`CalibrationWizard` 是一个 `QDialog`，把 LeRobot 校准流程图形化：

1. 选择臂类型（`follower` / `leader`），默认端口 `/dev/ttyACM0`。
2. 连接串口，扫描 ID，仅保留标准关节 `1–6`；缺失关节会提示并标记为未连接。
3. **失能全部舵机**，启动位置读取线程，实时刷新每个关节的当前位置。
4. 逐关节操作：
   - 手动把关节摆到机械中位，点击「记录中位值」→ 记录 `homing_offset`；
   - 点击「开始记录范围」，手动在全行程内来回摆动 → 持续更新 `range_min` / `range_max`；点击「停止记录范围」结束；
   - `wrist_roll` 为连续旋转关节，不记录范围，固定为 `[0, 4095]`。
5. 点击「保存校准文件」，生成 JSON 并写入项目 `json/` 目录；未完成关节使用默认值（中位 `2048`、范围 `[0, 4095]`）。
6. 支持「上一个/下一个关节」导航、预加载已有校准文件、断开连接。

> 注意：该向导保存路径为绝对路径 `/home/zwr185/dev_ws/Seeed_RoboController/json`，迁移到其他机器时需修改源码中的保存目录。

#### 3.3.4 关节角度读取与写入 `01–04`

这四个脚本都使用 `pyserial` 直接构造 SCS 协议报文（不依赖 `scservo_sdk`），适合作为协议教学示例或最小依赖调试工具。

| 脚本 | 核心逻辑 |
| --- | --- |
| `01_read_angle.py` | 读取寄存器 `56`（当前位置），按固定映射 `1024→-90°、2048→0°、3072→+90°` 换算并裁剪；matplotlib 水平柱状图每 50ms 刷新；默认端口 `COM5`。该脚本**不使用校准文件**，仅用于快速观察原始角度趋势。 |
| `02_write_angle.py` | 读取 LeRobot 校准文件，为每个关节创建 matplotlib 滑块；角度经 `homing_offset` / `drive_mode` 换算为原始位置并裁剪到校准范围；启动时使能力矩，退出时失能；默认端口 `/dev/ttyACM0`。 |
| `03_write_all_angle.py` | PySide6 界面，为 6 个关节提供角度输入框，支持「快速输入」逗号分隔角度（如 `45,-50,-80,-45,50,0`）后一次下发；「运动」写入目标位置，「复位」回到校准零点（0°）；所有目标都裁剪到 `range_min–range_max` 与 `0–4095`。 |
| `04_write_angle_stable.py` | 在 03 基础上改为平滑运动：以当前位置为起点，使用余弦缓动在固定 `1.2s` 内插值，每 `30ms` 下发一次中间位置；通过 `WritePosEx` 设置速度/加速度上限；提供「停止」按钮，避免大角度跳变冲击。 |

角度换算关系：

```text
position = homing_offset + sign(drive_mode) × angle × (4096 / 360)
angle    = sign(drive_mode) × (position − homing_offset) × (360 / 4096)
```

其中 `sign(drive_mode) = -1`（`drive_mode != 0`）或 `+1`，结果最终裁剪到 `[range_min, range_max]` 与 `[0, 4095]`。

#### 3.3.5 位置限位与 PID 调参

**`servo_angle_limit_set.py`（角度限位设置）**

- 界面为 6 行 × 4 列表格：`ID / MIN / POS / MAX`，固定对应 ID 1–6。
- 后台监控线程周期读取当前位置（寄存器 `56`），实时刷新 `POS`，并自动累积本次采集到的 `MIN` / `MAX`。
- 「重置」清空采集值；「读取」从舵机读出已写入的限位；「写入EEPROM」先解锁 EEPROM，再写寄存器 `9–12`（最小/最大角度限位低/高字节）。
- 写入限位会影响舵机可运动范围，错误限位可能导致关节无法动作，需谨慎使用。

**`servo_pid_tune_gui.py`（PID 调参）**

- 复用 `src/tools/servo_pid_tune.py` 的寄存器读写逻辑，维护寄存器 `21 (P)`、`22 (D)`、`23 (I)`。
- 支持读取全部舵机、应用到选中 ID、应用到全部；写入前自动备份原值到 `json/servo_pid_backup_*.json`。
- 支持「一键恢复备份」和手动选择备份文件；调参建议：末端发软先加 P，I 一般保持 0。

#### 3.3.6 辅助诊断与主题

| 文件 | 逻辑 |
| --- | --- |
| `diag6.py` | 针对夹爪 ID 6：使能力矩 → 读限位 `9/11` → 读当前位置 → 下发 `当前位置−300` → 读回目标寄存器 `42` → 读取负载 `60`、电压 `62`、温度 `63`、运动标志 `66` → 回原位。用于判断「夹爪不转」是限位、力矩还是机械卡滞问题。 |
| `test_all_joints.py` | 对 ID 1–5 逐个使能力矩，读取限位与当前位置，向 `+120` 计数（约 `10.5°`）方向小幅运动并回位；超出限位时反向。用于快速验证各关节机械与电气通路。 |
| `theme_utils.py` | `setup_light_theme(app)`：设置 `Fusion` 风格并应用浅色调色板（窗口 `#f8f9fa`、文本 `#212529` 等），避免 Windows 深色模式下硬编码白底控件出现「白底白字」。 |

### 3.4 在功能包中的作用与依赖关系

`src/gui` 位于依赖链最上层，是用户实际操作的入口；它向下依赖公共模块和 SDK，向内依赖项目数据文件。

| GUI 文件 | scservo_sdk | src.port_utils | src.calibration_manager | src.tools（子进程） | json/ 校准文件 | matplotlib |
| --- | :---: | :---: | :---: | :---: | :---: | :---: |
| `factory_calibration_tool.py` | ✓ | ✓ | — | ✓ | — | — |
| `calibration_wizard.py` | ✓ | — | ✓（关节名映射） | — | 写入 | — |
| `servo_angle_limit_set.py` | ✓ | —（自带端口枚举） | — | — | — | — |
| `servo_pid_tune_gui.py` | ✓（经 tools 逻辑） | ✓ | ✓（关节名映射） | ✓（复用逻辑） | 备份写入 | — |
| `03_write_all_angle.py` | —（自组包） | — | ✓（关节名映射） | — | 读取 | — |
| `04_write_angle_stable.py` | —（自组包） | — | ✓（关节名映射） | — | 读取 | — |
| `02_write_angle.py` | —（自组包） | — | — | — | 读取 | ✓ |
| `01_read_angle.py` | —（自组包） | — | — | — | — | ✓ |
| `diag6.py` / `test_all_joints.py` | ✓ | — | — | — | — | — |
| `theme_utils.py` | — | — | — | — | — | — |

具体协作关系：

- `factory_calibration_tool.py` 是**编排者**：自身负责界面与扫描，把中位校准、中位测试、失能、遥控委托给 `src/tools` 中的 CLI，并复用 `servo_middle_calibration.py` 的电压/温度阈值。
- `calibration_wizard.py` 与 `lerobot_calibrate.py` 是**两条等价的校准路径**（GUI / CLI），产出同一套 JSON 结构。
- `03/04` 与 `run_calibration_middle.py` 是**校准文件的下游消费者**，负责把角度/中位转换为原始位置并下发。
- `json/` 是 GUI 与 CLI 之间的**数据契约**：校准向导、CLI 校准写入，角度控制与中位运行读取。

### 3.5 使用注意事项

1. **运行方式**
   - 工厂工具、校准向导建议在项目根目录以模块方式运行，确保 `src` 与 `scservo_sdk` 可导入：
     `python -m src.gui.factory_calibration_tool`
   - `03/04` 已把项目根目录插入 `sys.path`，可直接运行：
     `python src/gui/03_write_all_angle.py json/my_awesome_follower_arm.json --port /dev/ttyACM0`
   - `calibration_wizard.py` 源码中默认端口为 `/dev/ttyACM0`，且通过相对路径补充 `sys.path`，建议从项目根目录运行或先修改默认端口。
2. **串口独占**：同一串口同一时刻只能被一个进程打开。使用工厂工具时不要再启动其他 GUI/CLI；若出现 `PermissionError` 或「打开串口失败」，先关闭占用程序。工厂工具内部虽会自动终止旧的工具子进程，但无法控制外部程序。
3. **硬编码路径与端口**：`01` 默认 `COM5`，`02/03/04` 默认 `/dev/ttyACM0`，`diag6.py` / `test_all_joints.py` 硬编码 `/dev/ttyACM0`，`03/04` 默认校准文件为绝对路径。迁移到其他机器时需通过命令行参数或修改源码调整。
4. **安全操作**
   - 手动摆臂（校准、示教）前必须失能力矩，避免舵机对抗或发热。
   - 首次运行或更换机械结构后，先用小幅角度验证方向，再扩大范围。
   - 大角度变化优先使用 `04_write_angle_stable.py`，避免冲击和过流。
   - 出现异味、异响、过热时立即断电检查。
5. **EEPROM 写入风险**：ID、角度限位、PID、波特率均写入 EEPROM（掉电保存）。错误限位可能让关节无法运动；错误波特率会导致后续无法通信；修改前建议记录原值，PID 界面已提供自动备份。
6. **连续旋转关节**：`wrist_roll` 不记录行程范围，校准文件固定 `[0, 4095]`；不要按普通绝对关节的方式理解其角度。
7. **Linux 串口权限**：当前用户需属于 `dialout` 组：`sudo usermod -aG dialout $USER`，重新登录后生效。
8. **可选依赖**：`01/02` 依赖 `numpy` 与 `matplotlib`，但未写入 `requirements.txt`；使用前需额外安装。
9. **电源匹配**：工具会按实测电压区分 5V/12V 系统并给出安全范围；电压异常时先检查电源与接线，不要强行校准。
10. **界面显示**：若 Windows 深色模式下出现文字不可读，确认 `theme_utils.py` 可正常导入；该模块会强制浅色 palette。

---

## 4. 快速开始

### 4.1 环境要求

- Python >= 3.8
- `PySide6 >= 6.0`
- `pyserial >= 3.5`
- 可选：`numpy`、`matplotlib`（仅 `01_read_angle.py` / `02_write_angle.py` 需要）

### 4.2 安装与自检

```bash
cd /home/zwr185/dev_ws/Seeed_RoboController

# 建议使用独立虚拟环境，避免污染系统 Python
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate

pip install -r requirements.txt
pip install numpy matplotlib       # 可选，仅 01/02 需要

# 环境自检（必须在项目根目录执行）
python setup.py
```

自检脚本会检查 Python 版本、依赖导入、项目模块、SDK 模块和关键文件完整性，并打印常用命令。

### 4.3 常用启动命令

| 用途 | 命令 |
| --- | --- |
| 双串口工厂标定（推荐入口） | `python -m src.gui.factory_calibration_tool` |
| 手动指定两个串口 | `python -m src.gui.factory_calibration_tool --port1 /dev/ttyACM0 --port2 /dev/ttyACM1` |
| 列出可用串口 | `python -m src.gui.factory_calibration_tool --list-ports` |
| 扫描舵机 ID | `python -m src.tools.scan_id` |
| 图形化 LeRobot 校准 | `python -m src.gui.calibration_wizard --arm-type follower` |
| 命令行 LeRobot 校准 | `python -m src.tools.lerobot_calibrate` |
| 按校准文件回中位 | `python -m src.tools.run_calibration_middle json/my_awesome_follower_arm.json /dev/ttyACM0` |
| 手动角度控制 | `python src/gui/03_write_all_angle.py json/my_awesome_follower_arm.json --port /dev/ttyACM0` |
| 平滑角度控制 | `python src/gui/04_write_angle_stable.py json/my_awesome_follower_arm.json --port /dev/ttyACM0` |
| 滑块角度控制 | `python src/gui/02_write_angle.py json/my_awesome_follower_arm.json` |
| 实时角度曲线 | `python src/gui/01_read_angle.py`（端口硬编码为 `COM5`，需先修改源码） |
| 角度限位设置 | `python -m src.gui.servo_angle_limit_set` |
| PID 调参 | `python -m src.gui.servo_pid_tune_gui` |
| 双端口同步遥控 | `python -m src.tools.servo_remote_control --read-port /dev/ttyACM0 --control-port /dev/ttyACM1` |
| 失能全部舵机 | `python -m src.tools.servo_disable` |

> 说明：`python -m src.gui.xxx` 需要项目根目录作为当前工作目录；`03/04` 采用直接脚本路径运行。

### 4.4 典型工作流

**A. 产线/出厂标定**

```text
扫描 ID → 修改为新 ID → 快速中位校准 → （可选）写入角度限位 → （可选）PID 调参 → 中位测试验证
```

对应工具：`scan_id.py` / 工厂 GUI → `change_single_servo_id.py` 或 ID 标定对话框 → 工厂 GUI「中位校准」→ `servo_angle_limit_set.py` → `servo_pid_tune_gui.py` → 工厂 GUI「中位测试」。

**B. LeRobot 整臂校准**

```text
连接机械臂 → 失能力矩 → 逐关节记录中位 → 逐关节记录范围 → 保存 JSON → 回中位验证
```

对应工具：`calibration_wizard.py`（或 `lerobot_calibrate.py`）→ `run_calibration_middle.py`。

**C. 日常调试**

```text
01 观察角度趋势 → 02 滑块逐关节验证 → 03 小幅全关节运动 → 04 平滑运动
```

**D. 主从遥操作**

```text
主动臂接主控端口（只读） → 从动臂接从控端口（控制） → 启动同步遥控
```

对应工具：`servo_remote_control.py`，或工厂 GUI 顶部「遥控」按钮。

---

## 5. 校准文件格式

校准文件为 JSON，每个关节一个对象；下面是从动臂校准文件的结构节选（完整文件包含 6 个关节）：

```json
{
    "shoulder_pan": {
        "id": 1,
        "drive_mode": 0,
        "homing_offset": 2050,
        "range_min": 776,
        "range_max": 3498
    },
    "wrist_roll": {
        "id": 5,
        "drive_mode": 1,
        "homing_offset": 1017,
        "range_min": 0,
        "range_max": 4095
    }
}
```

| 字段 | 含义 |
| --- | --- |
| 关节名（键） | `shoulder_pan`、`shoulder_lift`、`elbow_flex`、`wrist_flex`、`wrist_roll`、`gripper`。 |
| `id` | 舵机总线 ID，标准配置为 1–6。 |
| `drive_mode` | 方向模式。`0` 表示正向，非 `0` 表示角度方向取反。 |
| `homing_offset` | 关节角度零点对应的原始编码器计数（机械中位）。 |
| `range_min` / `range_max` | 该关节允许的原始计数范围，用于角度裁剪与安全限位。 |

**文件位置约定**

| 场景 | 路径 |
| --- | --- |
| 项目内样例 / 向导保存 | `Seeed_RoboController/json/*.json` |
| LeRobot 从动臂 | `~/.cache/huggingface/lerobot/calibration/robots/so_follower/*.json` |
| LeRobot 主动臂 | `~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/*.json` |
| PID 自动备份 | `Seeed_RoboController/json/servo_pid_backup_*.json` |

---

## 6. 故障排查

| 现象 | 可能原因 | 处理方式 |
| --- | --- | --- |
| 列不出串口 / 无可用端口 | USB 转 TTL 未连接、驱动缺失、线缆故障 | 检查连接与驱动；Linux 用 `ls /dev/ttyUSB* /dev/ttyACM*` 确认设备。 |
| `Permission denied` / 打不开串口 | 当前用户无串口权限 | 加入 `dialout` 组并重新登录。 |
| 串口被占用 | 另一个 GUI/CLI 正在使用同一端口 | 关闭占用程序；工厂工具会自动终止自己启动的旧子进程。 |
| 扫描不到舵机 | 未供电、波特率不匹配、总线接反、ID 冲突 | 检查电源与接线；确认波特率 `1000000`；逐个排查 ID。 |
| 舵机使能但不转 | 力矩未开、限位过窄、过温/过压/过载保护 | 用 `diag6.py` 或寄存器诊断查看限位、温度、电压、负载与错误位。 |
| 角度输入后不动 | 校准范围裁剪、目标等于当前位置、串口未连接 | 检查校准文件 `range_min/range_max` 与日志输出的目标计数。 |
| 校准向导中舵机可手动转动 | 向导为保证安全会主动失能力矩 | 属正常行为；记录中位与范围时手动摆臂即可。 |
| 保存的校准文件无效 | 关节未全部完成、保存路径无权限 | 补全未完成关节；检查 `json/` 目录权限。 |
| 界面白底白字/文字不可见 | Windows 深色主题与硬编码样式冲突 | 确认 `theme_utils.setup_light_theme()` 生效；必要时切换系统主题。 |
| `ModuleNotFoundError: matplotlib` | 未安装可选依赖 | `pip install numpy matplotlib`。 |
| `ModuleNotFoundError: src` | 未在项目根目录运行 | 切换到 `Seeed_RoboController` 根目录后使用 `python -m ...`。 |
| 子进程输出乱码/emoji 报错 | Windows 控制台编码不是 UTF-8 | 使用 UTF-8 终端；工厂工具已为子进程设置 `PYTHONIOENCODING=utf-8`。 |
| PID/限位写入后行为异常 | EEPROM 参数不合适 | 用 PID 备份恢复；限位可重新读取/清除或恢复出厂设置。 |

---

## 7. 开发与扩展约定

1. **新增 CLI 工具**：放在 `src/tools/`，复用 `src/port_utils.py` 的端口选择与排序逻辑；保证可通过 `python -m src.tools.<name>` 从项目根目录运行。
2. **新增 GUI**：放在 `src/gui/`，耗时操作放入 `QThread` / `QObject` Worker，通过 `Signal` 更新界面；启动时调用 `theme_utils.setup_light_theme()` 保持风格一致。
3. **串口访问**：不要跨进程共享串口；需要在 GUI 中调用外部脚本时，采用「停止本地扫描 → 子进程执行 → 结束后恢复扫描」的模式。
4. **EEPROM 操作**：写 ID、限位、PID、波特率前先 `unLockEprom()`，写完 `lockEprom()`，并做好原值备份与结果校验。
5. **路径处理**：现有部分脚本存在绝对路径硬编码，新代码应基于 `PROJECT_ROOT` 或命令行参数构造路径，避免迁移后失效。
6. **依赖管理**：新增第三方依赖时同步更新 `requirements.txt`；`numpy` / `matplotlib` 目前为可选依赖，如改为默认使用应一并列入。
7. **ROS 集成**：本目录当前通过 `COLCON_IGNORE` 排除在 colcon 构建之外；若要转换为 ROS 2 功能包，需移除该文件并补充 `package.xml`、`CMakeLists.txt` 或 `setup.py` 的 setuptools 配置（现有 `setup.py` 仅用于环境自检，不能直接用于打包）。

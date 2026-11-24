<div align="center">

# vision_pkg

**USB 相机颜色检测与像素坐标 → 机械臂底座坐标转换功能包**

</div>

---

## 1. 功能简介

`vision_pkg` 负责视觉链路的前半段：

1. 启动 `usb_cam` 获取 `/image_raw` 图像。
2. 用 HSV 颜色阈值检测画面中最大的色块，发布像素中心 `/box_center`。
3. 用透视变换（或线性近似）把像素坐标转换为机械臂底座平面坐标 `/box_world_pos`，单位 cm。
4. 提供交互式标定工具 `box_tf_calibrate`，采集至少 4 组“像素—实际坐标”点，生成 `box_tf_homography.json`。

抓取和 IK 由 `arm_pkg` / `ai_pkg` 完成；本包只输出目标坐标。

---

## 2. 安装依赖

### 2.1 需要安装的库

| 依赖 | 用途 |
| --- | --- |
| `rclpy`、`sensor_msgs`、`geometry_msgs` | ROS 2 图像与坐标消息 |
| `cv_bridge`、`python3-opencv` | 图像转换、HSV 检测 |
| `python3-numpy` | 透视变换与坐标计算 |
| `usb_cam` | USB 相机驱动 |
| `rqt_topic` | 标定/验证时查看话题 |
| `xacro`、`launch`、`launch_ros` | 系统依赖 |

### 2.2 安装与编译

```bash
sudo apt install ros-$ROS_DISTRO-usb-cam \
                 ros-$ROS_DISTRO-rqt-topic \
                 ros-$ROS_DISTRO-cv-bridge \
                 python3-opencv \
                 python3-numpy

cd /home/zwr185/dev_ws
colcon build --symlink-install --packages-select vision_pkg
source install/setup.bash
```

### 2.3 相机权限

```bash
ls -l /dev/video*
# 当前用户需要有 /dev/video* 读写权限，通常加入 video 组
sudo usermod -aG video $USER
# 重新登录后生效
```

---

## 3. 两个 Launch 文件

### 3.1 real_pos_test.launch.py

**功能**：启动相机、颜色检测、坐标转换和 `rqt_topic`，用于验证标定精度。启动后可在 `rqt_topic` 中同时查看 `/box_center`（像素）和 `/box_world_pos`（cm）。

**启动命令**：

```bash
cd /home/zwr185/dev_ws
source install/setup.bash

# 默认相机和标定文件
ros2 launch vision_pkg real_pos_test.launch.py

# 指定相机和透视变换文件
ros2 launch vision_pkg real_pos_test.launch.py \
  video_device:=/dev/video0 \
  homography_file:=/home/zwr185/dev_ws/src/vision_pkg/json/box_tf_homography.json
```

**对应的源码文件**：

| 文件 | 作用 |
| --- | --- |
| `vision_pkg/box_detect_node.py` | HSV 检测最大色块，发布 `/box_center` |
| `vision_pkg/box_tf_node.py` | 像素 → 底座平面坐标，发布 `/box_world_pos` |
| `vision_pkg/camera_native_node.py` | 可选：直接用 OpenCV 窗口显示相机画面 |
| `vision_pkg/hsv_image_node.py` | 可选：带 HSV 滑条的颜色调试窗口 |

**依赖的目录**：

| 目录 | 内容 / 作用 |
| --- | --- |
| `launch/` | 存放本 launch 文件 |
| `vision_pkg/` | 检测、坐标转换和调试节点源码 |
| `config/` | `cam.yaml`、`box_detect.yaml`、`box_tf.yaml` |
| `json/` | `box_tf_homography.json` 标定结果 |
| `resource/` | ament 包索引标记 |
| `usb_cam` ROS 包 | 相机驱动 |
| `rqt_topic` ROS 包 | 话题可视化 |

**话题**：

| 话题 | 类型 | 作用 |
| --- | --- | --- |
| `/image_raw` | `sensor_msgs/Image` | 相机原始图像 |
| `/box_center` | `geometry_msgs/PointStamped` | 色块中心像素坐标 |
| `/box_world_pos` | `geometry_msgs/Point` | 底座平面坐标（cm） |

**调试参数**：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `video_device` | 空（自动探测） | 相机设备，如 `/dev/video0` |
| `homography_file` | `.../vision_pkg/json/box_tf_homography.json` | 透视变换标定文件 |

---

### 3.2 real_pos_locate.launch.py

**功能**：启动相机、颜色检测和 `rqt_topic`，并自动打开标定程序 `box_tf_calibrate`。把球放在相对机械臂底座已知的 (x, y) 位置，输入实际坐标并采集像素点；采集至少 4 个点后输入 `q`，程序拟合透视变换并保存。

**启动命令**：

```bash
cd /home/zwr185/dev_ws
source install/setup.bash

# 默认保存到 vision_pkg/json/box_tf_homography.json
ros2 launch vision_pkg real_pos_locate.launch.py

# 指定相机、保存路径和像素话题
ros2 launch vision_pkg real_pos_locate.launch.py \
  video_device:=/dev/video0 \
  output:=/home/zwr185/dev_ws/src/vision_pkg/json/box_tf_homography.json \
  pixel_topic:=/box_center
```

**对应的源码文件**：

| 文件 | 作用 |
| --- | --- |
| `vision_pkg/box_tf_calibrate.py` | 交互式采集点、拟合透视变换、计算 RMSE 并保存 JSON |
| `vision_pkg/box_detect_node.py` | 提供 `/box_center` 像素坐标 |
| `vision_pkg/box_tf_node.py` | 使用标定文件进行坐标转换（本 launch 不启动，测试时使用） |

**依赖的目录**：

| 目录 | 内容 / 作用 |
| --- | --- |
| `launch/` | 存放本 launch 文件 |
| `vision_pkg/` | 标定、检测节点源码 |
| `config/` | `cam.yaml`、`box_detect.yaml` |
| `json/` | 标定结果默认保存目录 |
| `resource/` | ament 包索引标记 |
| `usb_cam` ROS 包 | 相机驱动 |
| `rqt_topic` ROS 包 | 查看 `/box_center` 像素 |
| `gnome-terminal`（系统命令） | 自动打开标定终端；没有时需手动执行打印的命令 |

**标定步骤**：

1. 在 `rqt_topic` 中勾选 `/box_center`，确认像素 u、v 持续刷新。
2. 把球放在相对机械臂底座已知的 (x, y) 位置，在标定终端输入 `x y` 并回车。
3. 更换至少 4 个位置，建议覆盖整个工作区域。
4. 输入 `q` 结束，生成 `box_tf_homography.json`，同时输出 RMSE 和最大误差。
5. 重新启动 `real_pos_test.launch.py` 或 `box_tf_node` 使新标定生效。

**调试参数**：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `video_device` | 空（自动探测） | 相机设备 |
| `output` | `.../vision_pkg/json/box_tf_homography.json` | 标定结果保存路径 |
| `pixel_topic` | `/box_center` | 像素坐标话题 |

> 该 launch 内部写死了 `/opt/ros/jazzy/setup.bash`。如果使用其他 ROS 发行版，需要修改 `launch/real_pos_locate.launch.py` 中的这一行；没有 `gnome-terminal` 时会打印需要手动执行的标定命令。

---

## 4. 视觉配置

| 文件 | 常用调试项 |
| --- | --- |
| `config/cam.yaml` | `video_device`、`image_width`、`image_height`、`framerate`、曝光/白平衡 |
| `config/box_detect.yaml` | HSV 阈值 `hue_min/hue_max`、`sat_min/sat_max`、`val_min/val_max`、`min_area`、`show_window` |
| `config/box_tf.yaml` | `mode`（auto/homography/linear）、`camera_x/y/z`、`target_z`、`cm_per_pixel`、`x_direction/y_direction` |

常用调试命令：

```bash
ros2 topic echo /box_center       # 是否检测到色块
ros2 topic echo /box_world_pos    # 转换后的坐标是否合理
ros2 topic hz /image_raw          # 相机帧率
ros2 run vision_pkg hsv_image_node   # 用滑条调 HSV
ros2 run vision_pkg camera_native_node  # 直接查看相机画面
```

- 检测不到：降低 `min_area`、调整 HSV，确认 `show_window: true` 时 Mask 正常。
- 坐标偏差大：重新运行 `real_pos_locate.launch.py` 标定；检查 `camera_x/y/z` 和 `mode`。
- 没有标定文件：`mode: auto` 会自动退回线性近似（`cm_per_pixel` 等参数）。

> `json/` 目录不会安装到 `install/share`，`box_tf.yaml` 中直接使用源码绝对路径，移动工作空间后需要同步修改。

---

## 5. 常见问题

| 问题 | 处理 |
| --- | --- |
| 找不到 `vision_pkg` | 编译后执行 `source install/setup.bash` |
| 相机无图像 | 检查 `/dev/video*`，启动时加 `video_device:=/dev/videoX` |
| 标定终端没有自动打开 | 手动执行 launch 日志中打印的 `ros2 run vision_pkg box_tf_calibrate ...` |
| 标定点不足 | 至少采集 4 组点，否则不会拟合 |
| 坐标误差大 | 增加标定点、覆盖工作区，检查相机是否移动 |
| 检测框不稳定 | 固定光照，调整 HSV 和形态学参数 |

---

## License

Apache-2.0，详见 [LICENSE](LICENSE)。

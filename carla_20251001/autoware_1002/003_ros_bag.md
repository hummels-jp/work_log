# Autoware Universe RosBag 回放仿真教程

> 来源：[Autoware Foundation 官方文档](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/rosbag-replay-simulation/)

## 概述

RosBag 回放仿真是 Autoware Universe 中一个重要的测试和验证工具，允许开发者使用预录制的传感器数据来测试自动驾驶算法的性能，而无需实际的车辆或实时传感器数据。

## 重要说明

### 示例数据版权
- **示例地图和 rosbag**: Copyright 2020 TIER IV, Inc.
- **隐私考虑**: 由于隐私原因，rosbag 不包含图像数据，这将导致：
  - 无法使用此示例 rosbag 测试交通灯识别功能
  - 目标检测精度会降低

## 准备步骤

### 步骤 1: 下载并解压示例地图

您可以手动下载[地图文件](https://drive.google.com/file/d/1A-8BvYRX3DhSzkAnOcGWFw5T30xTlwZI/view?usp=sharing)，或使用以下命令：

```bash
gdown -O ./autoware_map/ 'https://docs.google.com/uc?export=download&id=1A-8BvYRX3DhSzkAnOcGWFw5T30xTlwZI'
unzip -d ./autoware_map/ ./autoware_map/sample-map-rosbag.zip
```

### 步骤 2: 下载示例 rosbag 文件

您可以手动下载 [rosbag 文件](https://drive.google.com/file/d/1sU5wbxlXAfHIksuHjP3PyI2UVED8lZkP/view?usp=sharing)，或使用以下命令：

```bash
gdown -O ./autoware_map/ 'https://docs.google.com/uc?export=download&id=1sU5wbxlXAfHIksuHjP3PyI2UVED8lZkP'
unzip -d ./autoware_map/ ./autoware_map/sample-rosbag.zip
```

### 步骤 3: 检查 autoware_data 文件夹

确认您有 `~/autoware_data` 文件夹和其中的文件：

```bash
$ cd ~/autoware_data
$ ls -C -w 30
image_projection_based_fusion
lidar_apollo_instance_segmentation
lidar_centerpoint
tensorrt_yolo
tensorrt_yolox
traffic_light_classifier
traffic_light_fine_detector
traffic_light_ssd_fine_detector
yabloc_pose_initializer
```

如果没有这些文件，请按照 [手动下载构件](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/artifacts) 的说明进行操作。

## 如何运行 rosbag 回放仿真

### 使用命令行方式

#### 方法 1: 命令行启动

**步骤 1: 启动 Autoware**

```bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch logging_simulator.launch.xml \
    map_path:=$HOME/autoware_map/sample-map-rosbag \
    vehicle_model:=sample_vehicle \
    sensor_model:=sample_sensor_kit
```

> **注意**: 这里不能使用 `~` 代替 `$HOME`。

![启动后的界面](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/rosbag-replay/after-autoware-launch.png)

> ⚠️ **警告**: 在播放 rosbag 之前，您可能会在终端中遇到错误和警告消息。这是正常行为。一旦播放 rosbag 并进行适当的初始化，这些消息就会停止。

**步骤 2: 播放示例 rosbag 文件**

```bash
source ~/autoware/install/setup.bash
ros2 bag play ~/autoware_map/sample-rosbag/ -r 0.2 -s sqlite3
```

> ⚠️ **警告**: 由于 rosbag 中的时间戳与当前系统时间戳之间的差异，Autoware 可能会在终端中生成警告消息，提醒这种不匹配。这是正常行为。

![播放后的界面](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/rosbag-replay/after-rosbag-play.png)

**步骤 3: 调整 RViz 视图**

要将视图聚焦在自车上，请在 RViz 视图面板中将 `Target Frame` 从 `viewer` 更改为 `base_link`。

![更改目标帧](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/rosbag-replay/change-target-frame.png)

**步骤 4: 切换视图类型**

要切换到 `Third Person Follower` 等视图，请在 RViz 视图面板中更改 `Type`。

![第三人称跟随视图](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/rosbag-replay/third-person-follower.png)

### 参考视频教程

[参考视频教程](https://drive.google.com/file/d/12D6aSC1Y3Kf7STtEPWG5RYynxKdVcPrc/view?usp=sharing)

## 使用 Autoware Launch GUI

如果您更喜欢图形用户界面 (GUI) 而不是命令行来启动和管理仿真，请参考本文档末尾的 `使用 Autoware Launch GUI` 部分，获取分步指南。

### 开始使用 Autoware Launch GUI

**步骤 1: 安装**
确保您已安装 Autoware Launch GUI。[安装说明](https://github.com/autowarefoundation/autoware-launch-gui#installation)。

**步骤 2: 启动 GUI**
从应用程序菜单打开 Autoware Launch GUI。

![GUI 启动截图](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/rosbag-replay/launch-gui/launch_gui_main.png)

### 启动日志仿真

**步骤 1: 设置 Autoware 路径**
在 GUI 中，设置 Autoware 安装的路径。

![设置 Autoware 路径的 GUI 截图](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/rosbag-replay/launch-gui/launch_gui_setup.png)

**步骤 2: 选择启动文件**
为车道驾驶场景选择 `logging_simulator.launch.xml`。

![选择启动文件的 GUI 截图](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/rosbag-replay/launch-gui/selecting_launch_file.png)

**步骤 3: 自定义参数**
根据需要调整参数，如 `map_path`、`vehicle_model` 和 `sensor_model`。

![自定义参数的 GUI 截图 1](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/rosbag-replay/launch-gui/customizing-parameters1.png)

![自定义参数的 GUI 截图 2](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/rosbag-replay/launch-gui/customizing-parameters2.png)

**步骤 4: 启动仿真**
点击启动按钮开始仿真并访问所有日志。

![启动仿真的 GUI 截图](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/rosbag-replay/launch-gui/starting_simulation.png)

**步骤 5: 播放 Rosbag**
移动到 `Rosbag` 标签并选择您想要播放的 rosbag 文件。

![选择 rosbag 文件的 GUI 截图](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/rosbag-replay/launch-gui/selecting_rosbag_file.png)

**步骤 6: 调整播放速度**
根据需要调整播放速度和任何其他您想要自定义的参数。

![调整播放速度的 GUI 截图](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/rosbag-replay/launch-gui/adjusting_flags.png)

**步骤 7: 开始播放**
点击播放按钮开始 rosbag 播放，并可以访问如 `暂停/播放`、`停止` 和 `速度滑块` 等设置。

![开始播放的 GUI 截图](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/rosbag-replay/launch-gui/starting_playback.png)

**步骤 8: 查看仿真**
移动到 `RViz` 窗口查看仿真。

![仿真视图](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/rosbag-replay/after-rosbag-play.png)

**步骤 9: 聚焦自车视图**
要将视图聚焦在自车上，请在 RViz 视图面板中将 `Target Frame` 从 `viewer` 更改为 `base_link`。

![更改目标帧](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/rosbag-replay/change-target-frame.png)

**步骤 10: 切换视图类型**
要切换到 `Third Person Follower` 等视图，请在 RViz 视图面板中更改 `Type`。

![第三人称跟随视图](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/rosbag-replay/third-person-follower.png)

## 高级用法和技巧

### RosBag 播放参数

#### 播放速度控制
```bash
# 慢速播放 (0.2倍速)
ros2 bag play ~/autoware_map/sample-rosbag/ -r 0.2 -s sqlite3

# 正常速度播放
ros2 bag play ~/autoware_map/sample-rosbag/ -r 1.0 -s sqlite3

# 快速播放 (2倍速)
ros2 bag play ~/autoware_map/sample-rosbag/ -r 2.0 -s sqlite3
```

#### 循环播放
```bash
# 无限循环播放
ros2 bag play ~/autoware_map/sample-rosbag/ -r 0.2 -s sqlite3 --loop
```

#### 指定开始时间
```bash
# 从第10秒开始播放
ros2 bag play ~/autoware_map/sample-rosbag/ -r 0.2 -s sqlite3 --start-offset 10.0
```

#### 播放特定话题
```bash
# 只播放特定话题
ros2 bag play ~/autoware_map/sample-rosbag/ \
    --topics /sensing/lidar/top/pointcloud_raw /localization/pose_estimator/pose
```

### 常见问题和解决方案

#### 问题 1: 时间戳警告
**现象**: 终端显示时间戳不匹配警告
**解决方案**: 这是正常现象，可以忽略这些警告

#### 问题 2: 初始化错误
**现象**: 启动时出现错误和警告消息
**解决方案**: 开始播放 rosbag 后这些消息会消失

#### 问题 3: 视图不正确
**现象**: RViz 中看不到车辆或数据
**解决方案**: 
- 确保将 Target Frame 设置为 `base_link`
- 检查相关的显示项是否已启用

### 数据分析

#### 查看 RosBag 信息
```bash
# 查看 bag 文件基本信息
ros2 bag info ~/autoware_map/sample-rosbag/

# 查看详细信息
ros2 bag info ~/autoware_map/sample-rosbag/ --verbose
```

#### 话题列表
```bash
# 列出所有话题
ros2 topic list

# 查看特定话题的消息类型
ros2 topic info /sensing/lidar/top/pointcloud_raw
```

#### 实时监控
```bash
# 监控话题频率
ros2 topic hz /sensing/lidar/top/pointcloud_raw

# 查看消息内容
ros2 topic echo /localization/pose_estimator/pose
```

## 扩展应用

### 与其他仿真器集成

本教程可以与以下仿真器结合使用：
- [AWSIM 仿真器](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/digital-twin-simulation/awsim-tutorial/)
- [MORAI Sim: Drive](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/digital-twin-simulation/MORAI_Sim-tutorial/)
- [CARLA 仿真器](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/digital-twin-simulation/carla-tutorial/)

### 场景仿真

可以进一步扩展到：
- [场景仿真](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/scenario-simulation/)
- [规划仿真](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/planning-simulation/)
- [驾驶日志回放器](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/scenario-simulation/rosbag-replay-simulation/driving-log-replayer/)

## 系统要求

### 硬件要求
- **CPU**: 多核处理器 (推荐 8核以上)
- **内存**: 16GB RAM (推荐 32GB)
- **GPU**: NVIDIA GPU (支持 CUDA 12.4)
- **存储**: 至少 100GB 可用空间

### 软件要求
- **操作系统**: Ubuntu 22.04 LTS
- **ROS**: ROS2 Humble
- **CUDA**: 12.4 (如果使用 GPU 加速)

## 相关资源

### 官方文档链接
- [Autoware 文档主页](https://autowarefoundation.github.io/autoware-documentation/main/)
- [Autoware GitHub 仓库](https://github.com/autowarefoundation/autoware)
- [安装指南](https://autowarefoundation.github.io/autoware-documentation/main/installation/)
- [网络配置](https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/)

### 开发指南
- [如何指南](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/)
- [创建车辆和传感器模型](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-vehicle-and-sensor-model/)
- [传感器标定](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-vehicle-and-sensor-model/calibrating-sensors/)

### 贡献和支持
- [贡献指南](https://autowarefoundation.github.io/autoware-documentation/main/contributing/)
- [故障排除](https://autowarefoundation.github.io/autoware-documentation/main/support/troubleshooting/)
- [支持指南](https://autowarefoundation.github.io/autoware-documentation/main/support/support-guidelines/)

## 总结

RosBag 回放仿真是 Autoware Universe 开发和测试流程中的关键工具。通过本教程，您可以：

1. **理解基本概念**: 掌握 RosBag 回放仿真的原理和用途
2. **完成环境配置**: 按步骤设置必要的数据和环境
3. **掌握操作方法**: 学会使用命令行和 GUI 两种方式
4. **解决常见问题**: 了解并处理常见的配置和运行问题
5. **扩展应用**: 将技能应用到更复杂的仿真场景中

建议在实际项目中结合自己的数据和需求，进一步定制和优化仿真配置。

---

*Copyright © 2023 The Autoware Foundation*

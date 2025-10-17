# Autoware Universe RosBag 使用示例

https://blog.csdn.net/weixin_45168199/article/details/123212178

roslaunch runtime_manager runtime_manager.launch

ros2 launch runtime_manager runtime_manager.launch


## 概述

本文档演示如何在 Autoware Universe 中使用 rosbag 进行数据录制、回放和仿真。RosBag 是 ROS2 中用于记录和回放消息数据的重要工具，在自动驾驶开发中用于测试、调试和算法验证。

## 环境要求

- **Autoware Universe**: 基于 ROS2 Humble 的最新版本
- **Ubuntu 22.04 LTS**
- **ROS2 Humble**
- **CUDA 12.4** (用于 GPU 加速)
- **CARLA 0.9.15** (可选，用于仿真数据生成)

## 1. 基础概念

### RosBag 文件格式
- **SQLite3**: 默认存储格式，支持高效查询
- **MCAP**: 新型格式，支持更好的性能和兼容性
- **消息类型**: sensor_msgs, geometry_msgs, nav_msgs 等

### 关键话题 (Topics)
```bash
# 传感器数据
/sensing/lidar/top/pointcloud_raw
/sensing/camera/camera0/image_raw
/sensing/imu/tamagawa/imu_raw
/sensing/gnss/ublox/nav_sat_fix

# 定位数据
/localization/pose_estimator/pose
/localization/kinematic_state

# 感知数据
/perception/object_recognition/detection/objects
/perception/object_recognition/tracking/objects

# 规划数据
/planning/scenario_planning/trajectory
/planning/scenario_planning/lane_driving/behavior_planning/path

# 控制数据
/control/command/control_cmd
/control/command/gear_cmd
```

## 2. 安装和准备

### 下载示例数据
```bash
# 创建数据目录
mkdir -p /home/kotei/huqianqian/software/autoware.universe/demo/demo01_rosbag/autoware_map/

# 下载示例地图
gdown -O ./autoware_map/ 'https://docs.google.com/uc?export=download&id=1A-8BvYRX3DhSzkAnOcGWFw5T30xTlwZI'
unzip -d ./autoware_map/ ./autoware_map/sample-map-rosbag.zip

# 下载示例 RosBag 文件
gdown -O ./autoware_map/ 'https://docs.google.com/uc?export=download&id=1sU5wbxlXAfHIksuHjP3PyI2UVED8lZkP'
unzip -d ./autoware_map/ ./autoware_map/sample-rosbag.zip
```

### 验证 Autoware 数据文件
```bash
# 检查 autoware_data 目录
cd ./autoware_data
ls -la

# 应该包含以下模型文件：
# - image_projection_based_fusion
# - lidar_apollo_instance_segmentation
# - lidar_centerpoint
# - tensorrt_yolo
# - tensorrt_yolox
# - traffic_light_classifier
# - traffic_light_fine_detector
# - traffic_light_ssd_fine_detector
# - yabloc_pose_initializer
```

## 3. RosBag 回放仿真

### 3.1 启动 Autoware (命令行方式)

```bash
# 设置环境变量
source ~/autoware/install/setup.bash

# 启动 logging simulator
ros2 launch autoware_launch logging_simulator.launch.xml \
    map_path:=/home/kotei/huqianqian/software/autoware.universe/demo/demo01_rosbag/autoware_map/sample-map-rosbag \
    vehicle_model:=sample_vehicle \
    sensor_model:=sample_sensor_kit
```

### 3.2 回放 RosBag 文件

```bash
# 在新终端中回放数据
source ~/autoware/install/setup.bash

# 以 0.2 倍速回放 (推荐用于调试)
ros2 bag play ~/autoware_map/sample-rosbag/ -r 0.2 -s sqlite3

# 正常速度回放
ros2 bag play ~/autoware_map/sample-rosbag/ -r 1.0 -s sqlite3
ros2 bag play /home/kotei/huqianqian/software/autoware.universe/demo/demo01_rosbag/autoware_map/sample-rosbag/ -r 1.0 -s sqlite3

# 循环回放
ros2 bag play ~/autoware_map/sample-rosbag/ -r 0.2 -s sqlite3 --loop
```

### 3.3 RViz 可视化配置

```bash
# 在 RViz 中调整视图
# 1. 将 Target Frame 从 "viewer" 更改为 "base_link"
# 2. 将 Type 更改为 "Third Person Follower" 获得更好的跟随视角
# 3. 启用相关显示项：
#    - PointCloud2 (激光雷达数据)
#    - Image (相机数据)
#    - Path (规划路径)
#    - Objects (检测目标)
```

## 4. 使用 Autoware Launch GUI

### 4.1 安装 Launch GUI
```bash
# 安装 GUI 工具
pip install autoware-launch-gui

# 或从源码安装
git clone https://github.com/autowarefoundation/autoware-launch-gui.git
cd autoware-launch-gui
pip install -e .
```

### 4.2 GUI 操作步骤

1. **启动 GUI**
   ```bash
   autoware-launch-gui
   ```

2. **配置 Autoware 路径**
   - 设置 Autoware 安装路径: `~/autoware`

3. **选择启动文件**
   - 选择 `logging_simulator.launch.xml`

4. **自定义参数**
   - map_path: `$HOME/autoware_map/sample-map-rosbag`
   - vehicle_model: `sample_vehicle`
   - sensor_model: `sample_sensor_kit`

5. **启动仿真**
   - 点击 Launch 按钮

6. **RosBag 回放**
   - 切换到 Rosbag 标签
   - 选择 rosbag 文件
   - 调整回放速度
   - 点击 Play 按钮

## 5. 数据录制示例

### 5.1 录制传感器数据
```bash
# 录制所有传感器数据
ros2 bag record -a -o my_sensor_data

# 录制特定话题
ros2 bag record \
    /sensing/lidar/top/pointcloud_raw \
    /sensing/camera/camera0/image_raw \
    /sensing/imu/tamagawa/imu_raw \
    /sensing/gnss/ublox/nav_sat_fix \
    -o sensor_data_$(date +%Y%m%d_%H%M%S)

# 录制定位和感知数据
ros2 bag record \
    /localization/pose_estimator/pose \
    /perception/object_recognition/detection/objects \
    /perception/object_recognition/tracking/objects \
    -o perception_data_$(date +%Y%m%d_%H%M%S)
```

### 5.2 录制规划和控制数据
```bash
# 录制规划数据
ros2 bag record \
    /planning/scenario_planning/trajectory \
    /planning/scenario_planning/lane_driving/behavior_planning/path \
    /planning/scenario_planning/lane_driving/motion_planning/path \
    -o planning_data_$(date +%Y%m%d_%H%M%S)

# 录制控制数据
ros2 bag record \
    /control/command/control_cmd \
    /control/command/gear_cmd \
    /vehicle/status/velocity_status \
    /vehicle/status/steering_status \
    -o control_data_$(date +%Y%m%d_%H%M%S)
```

## 6. 高级使用技巧

### 6.1 RosBag 信息查看
```bash
# 查看 bag 文件信息
ros2 bag info ~/autoware_map/sample-rosbag/

# 查看话题列表
ros2 bag info ~/autoware_map/sample-rosbag/ | grep "Topic information"

# 查看消息统计
ros2 bag info ~/autoware_map/sample-rosbag/ --verbose
```

### 6.2 消息过滤和转换
```bash
# 只回放特定话题
ros2 bag play ~/autoware_map/sample-rosbag/ \
    --topics /sensing/lidar/top/pointcloud_raw \
              /localization/pose_estimator/pose

# 重新映射话题名称
ros2 bag play ~/autoware_map/sample-rosbag/ \
    --remap /old_topic:=/new_topic

# 从特定时间开始回放
ros2 bag play ~/autoware_map/sample-rosbag/ \
    --start-offset 10.0  # 从第10秒开始

# 回放指定时间段
ros2 bag play ~/autoware_map/sample-rosbag/ \
    --start-offset 10.0 --duration 30.0  # 从第10秒开始，播放30秒
```

### 6.3 性能优化
```bash
# 使用压缩存储
ros2 bag record -a --compression-mode file \
    --compression-format zstd -o compressed_data

# 设置存储格式
ros2 bag record -a --storage-id mcap -o mcap_data

# 限制文件大小
ros2 bag record -a --max-bag-size 1000000000 -o split_data  # 1GB
```

## 7. 与 CARLA 集成

### 7.1 从 CARLA 录制数据
```bash
# 启动 CARLA 服务器
./CarlaUE4.sh -world-port=2000 -resx=800 -resy=600

# 启动 CARLA-ROS Bridge
source ~/autoware/install/setup.bash
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py \
    host:=localhost port:=2000 timeout:=10

# 录制 CARLA 数据
ros2 bag record \
    /carla/ego_vehicle/lidar \
    /carla/ego_vehicle/camera/rgb/front/image_color \
    /carla/ego_vehicle/imu \
    /carla/ego_vehicle/gnss \
    -o carla_simulation_$(date +%Y%m%d_%H%M%S)
```

### 7.2 在 Autoware 中回放 CARLA 数据
```bash
# 启动 Autoware 用于 CARLA 数据回放
ros2 launch autoware_launch logging_simulator.launch.xml \
    map_path:=$HOME/autoware_map/carla_town03 \
    vehicle_model:=sample_vehicle \
    sensor_model:=carla_sensor_kit

# 回放 CARLA 数据
ros2 bag play carla_simulation_20241001_143000/ -r 0.5
```

## 8. 常见问题和解决方案

### 8.1 时间戳不匹配警告
```bash
# 问题：rosbag 时间戳与系统时间不匹配
# 解决：使用 --clock 参数
ros2 bag play ~/autoware_map/sample-rosbag/ --clock
```

### 8.2 内存不足
```bash
# 问题：大文件回放时内存不足
# 解决：限制缓冲区大小
ros2 bag play ~/autoware_map/sample-rosbag/ \
    --read-ahead-queue-size 100
```

### 8.3 话题不匹配
```bash
# 问题：话题名称不匹配
# 解决：检查话题名称并重新映射
ros2 topic list  # 查看当前话题
ros2 bag info ~/autoware_map/sample-rosbag/  # 查看bag中的话题

# 重新映射话题
ros2 bag play ~/autoware_map/sample-rosbag/ \
    --remap /old_topic:=/correct_topic
```

## 9. 数据分析工具

### 9.1 使用 PlotJuggler 分析数据
```bash
# 自动安装 PlotJuggler（无需用户确认）
sudo apt update -qq
sudo apt install -y ros-humble-plotjuggler-ros

# 启动 PlotJuggler
ros2 run plotjuggler plotjuggler

# 自动化安装脚本
install_plotjuggler() {
    echo "正在安装 PlotJuggler..."
    if sudo apt install -y ros-humble-plotjuggler-ros > /dev/null 2>&1; then
        echo "✓ PlotJuggler 安装成功"
    else
        echo "✗ PlotJuggler 安装失败"
        return 1
    fi
}

# 调用安装函数
install_plotjuggler

# 加载 rosbag 数据进行分析（命令行方式）
ros2 run plotjuggler plotjuggler --buffer_size 500000 --layout ~/autoware_map/analysis_layout.xml
```

### 9.2 Python 脚本分析
```python
#!/usr/bin/env python3
"""
RosBag 数据分析示例脚本
"""
import rclpy
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import numpy as np

def analyze_pointcloud_data(bag_path):
    """分析点云数据"""
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    topic_types = reader.get_all_topics_and_types()
    pointcloud_count = 0
    
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        
        if topic == '/sensing/lidar/top/pointcloud_raw':
            msg = deserialize_message(data, PointCloud2)
            pointcloud_count += 1
            print(f"PointCloud {pointcloud_count}: {msg.width}x{msg.height} points")
    
    print(f"Total pointclouds: {pointcloud_count}")

if __name__ == '__main__':
    bag_path = "/home/kotei/autoware_map/sample-rosbag/"
    analyze_pointcloud_data(bag_path)
```

## 10. 最佳实践

### 10.1 数据管理
- 使用描述性的文件名和时间戳
- 定期清理旧的 rosbag 文件
- 使用压缩减少存储空间
- 备份重要的测试数据

### 10.2 性能优化
- 根据需要选择合适的回放速度
- 使用话题过滤减少不必要的数据
- 配置合适的队列大小
- 监控系统资源使用情况

### 10.3 调试技巧
- 使用慢速回放进行详细分析
- 结合 RViz 进行可视化调试
- 录制多个传感器数据进行对比
- 使用 PlotJuggler 分析数值数据

## 总结

RosBag 是 Autoware Universe 开发和测试中的重要工具。通过本文档的示例，您可以：

1. **自动化网络配置**: 配置系统自动允许外部网址访问，无需手动许可
2. **数据回放**: 使用官方示例数据进行仿真测试
3. **数据录制**: 记录真实车辆或仿真环境的数据
4. **数据分析**: 使用各种工具分析传感器和算法数据
5. **CARLA 集成**: 结合 CARLA 仿真器进行数据采集和测试

### 重要提示

⚠️ **网络安全注意事项**：
- 本文档中的网络配置主要适用于**开发和测试环境**
- 在生产环境中使用时，请根据实际安全需求调整配置
- 建议定期审查网络访问权限设置
- 在企业环境中，请遵循公司的网络安全政策

✅ **自动化优势**：
- 减少手动交互，提高开发效率
- 确保一致的配置环境
- 便于批量部署和持续集成
- 降低人为配置错误的风险

建议在实际项目中根据具体需求调整配置参数，并建立完善的数据管理流程。

## 参考资料

- [Autoware Universe 官方文档](https://autowarefoundation.github.io/autoware-documentation/main/)
- [ROS2 Bag 文档](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)
- [CARLA-ROS Bridge](https://carla.readthedocs.io/projects/ros-bridge/en/latest/)
- [PlotJuggler](https://github.com/facontidavide/PlotJuggler)

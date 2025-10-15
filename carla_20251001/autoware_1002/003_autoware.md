# Autoware.universe部署03：与Carla（二进制版）联调

> 来源：[CSDN博客 - ZARD帧心](https://blog.csdn.net/zardforever123/article/details/132357436)

## 概述

本文介绍了 Autoware.universe 与 Carla 二进制版的联调过程。Autoware.universe 是一个开源的自动驾驶软件平台，而 Carla 则是一个用于自动驾驶仿真的开源工具。将这两者联调起来可以在仿真环境中测试和验证 Autoware.universe 的功能和性能，为自动驾驶系统的开发和测试提供有力支持。

其中 Autoware.universe 的安装以及高精地图的绘制参考其他相关文章：
- [Autoware.universe部署01：Ubuntu20.04安装Autoware.universe并与Awsim联调](https://blog.csdn.net/zardforever123/article/details/132029636?spm=1001.2014.3001.5501)
- [Autoware.universe部署02：高精Lanelet2地图的绘制](https://blog.csdn.net/zardforever123/article/details/132528899?spm=1001.2014.3001.5502)

## 一、Carla的使用

### 1.1 二进制Carla的安装

二进制包GitHub地址：[https://github.com/carla-simulator/carla/releases](https://github.com/carla-simulator/carla/releases)

下载第一个 `.tar.gz` 压缩包，解压后可以直接运行：

```bash
tar -zxvf CARLA_0.9.13.tar.gz
cd CARLA_0.9.13
./CarlaUE4.sh
```

出现服务器界面后，去 `/home/autoware/CARLA_0.9.13/PythonAPI/examples` 下运行 API：

```bash
python manual_control.py
```

即可开启 Carla 仿真驾驶。

#### 命令行选项

启动 Carla 时有一些可用的配置选项：

```bash
./CarlaUE4.sh -quality-level=Low
```

启动 CARLA 时的配置选项：

- `carla-rpc-port=N`：侦听端口 N 处的客户端连接。默认情况下，流式端口 Streaming port 设置为 N+1
- `carla-streaming-port=N`：指定用于传感器数据流的端口。使用 0 获取随机未使用的端口。第二个端口将自动设置为 N + 1
- `quality-level={Low,Epic}`：更改图形质量级别
- `-carla-server`：让 carla 以服务的方式运行
- `-benchmark -fps=15`：引擎以1/15秒的固定时间逐步运行

### 1.2 Carla API的使用

首先打开 carla：`./CarlaUE4.sh`

#### 1.2.1 增加行人流和车流

执行 `CARLA_0.9.13/PythonAPI/examples` 路径下的 `generate_traffic.py` 文件：

```bash
python generate_traffic.py -w 10 -n 10
```

可以看到，carla 服务器中的街道上多了很多运动的车辆和行人。后面的参数 `-n 10 -w 10` 分别代表车辆和行人的数量。

脚本参数说明：
- `--host` - 主机服务器ip
- `-p --port` - 端口
- `-n --number-of-vehicles` - 车辆数量
- `-w --number-of-walkers` - 行人数量
- `--safe` - 避免碰撞
- `--filterv` - 车辆滤波器
- `--filterw` - 行人滤波器
- `--generationv` - 限制某些车辆生成
- `--generationw` - 限制某些行人生成

#### 1.2.2 改变服务器的时间，光照，天气

执行 `CARLA_0.9.13/PythonAPI/examples` 路径下的 `dynamic_weather.py` 文件：

```bash
python dynamic_weather.py -s 5
```

参数 `-s 5` 意思是把天气变化加速了5倍。这个脚本里是按照仿真时间，修改服务器中设置天气的参数 `carla.WeatherParameters`，按照一定的顺序自动的调整系统的光照和天气。

光照条件包括：太阳高度和太阳角度
天气条件包括：云，雨，水坑，风，雾，湿度

更多天气参数的设置可以查看 `/pythonAPI/python_api.md` 中的 `carla.WeatherParameters`。

#### 1.2.3 手动控制车辆

执行 `CARLA_0.9.13/PythonAPI/examples` 路径下的 `manual_control.py` 文件：

```bash
python manual_control.py
```

键盘B按键可以切换自动控制和人工控制，pygame操作手册如下：

| 按键 | 功能 | 备注 |
|------|------|------|
| W | 前进 | ↑ 键也可以前进 |
| S | 刹车 | ↓ 键也可以刹车 |
| A/D | A左转 D 右转 | ← 也可以左转，→ 也可以右转 |
| Q | 倒车档 | Q+W可以倒车 |
| Space | 手刹 | - |
| P | 开启/关闭自动驾驶模式 | - |
| M | 自动档/手动档 | - |
| ，和 . | 加减挡 | ，减档 . 加档 |
| CTRL + W | 按一下 CTRL + W，匀速前进 | 车会一直以 60 km/h 的速度前进 |
| L | 控制车灯 | 切换雾灯、近光灯等切换 |
| SHIFT + L | 切换远光灯 | - |
| Z/X | 转向灯 | Z 左转向，X 右转向 |
| I | 车内照明灯 | - |
| TAB | 切换视角 | - |
| N | 切换不同类型的 camera 和 lidar | 每按下一次，sensor |
| [1-9] | 顺序切换切换不同类型的 camera 和 lidar | 按下数字键，可直接切换到对应sensor |
| G | 打开/关闭 毫米波雷达 | - |
| C | 切换天气 | Shift+C切换顺序和C相反 |
| Backspace | 换车型 | - |
| V | 选地图图层 | Shift+V ，切换顺序和V 相反 |
| B | 加载当前的地图图层 | Shift+B 卸载当前的地图图层 |
| O | 打开/关闭所有车门 | - |
| T | 切换到车辆的自动测量记录传导 | 在carla客户端界面显示 |
| R | 记录车辆走行情况 | CTRL + R : 切换到 R做的记录 |
| CTRL + P | 回放R的记录 | - |
| F1 | 显示/不显示左侧sensor相关信息例如加速度，陀螺仪，GNSS等 | - |
| H | 可以弹出帮助命令 | - |
| ESC | 退出pygame | - |

#### 1.2.4 车辆的自动驾驶

执行 `CARLA_0.9.13/PythonAPI/examples` 路径下的 `automatic_control.py` 文件：

```bash
python automatic_control.py 
```

会弹出一个pygame窗口，车辆正在进行自动驾驶，到达终点后自动关闭。

## 二、Universe与Carla联调

参考文章：[https://blog.csdn.net/yuteng12138/article/details/130102293](https://blog.csdn.net/yuteng12138/article/details/130102293)

### 2.1 设置OpenPlanner

#### （1）下载拓展代码

```bash
cd ~/autoware_universe
mkdir op_carla && cd op_carla/
git clone https://github.com/hatem-darweesh/op_bridge.git -b ros2
git clone https://github.com/hatem-darweesh/op_agent.git -b ros2
git clone https://github.com/hatem-darweesh/scenario_runner.git
```

#### （2）下载地图文件

从 [https://drive.google.com/drive/folders/1Or0CMS08AW8XvJtzzR8TfhqdY9MMUBpS](https://drive.google.com/drive/folders/1Or0CMS08AW8XvJtzzR8TfhqdY9MMUBpS) 下载 Town01.pcd 和 Town01.osm 地图。

创建 Town01 文件夹，将 Town01.pcd 和 Town01.osm 复制到 Town01_map 文件夹中：

- Town01.pcd 重命名为 pointcloud_map.pcd
- Town01.osm 重命名为 lanelet2_map.osm

其他地图可以在这里下载：

```bash
git clone https://bitbucket.org/carla-simulator/autoware-contents/src/master.git
```

#### （3）修改网络配置

修改文件：`op_carla/op_bridge/op_scripts/run_exploration_mode_ros2.sh`，在第8行，如果不在同一台电脑上运行仿真，需要设置运行仿真的电脑的 IP 地址：

```bash
export SIMULATOR_LOCAL_HOST="192.168.11.5"
```

如果在本机上运行仿真需要修改为：

```bash
export SIMULATOR_LOCAL_HOST="localhost"
```

#### （4）修改Autoware启动配置

修改文件：`op_carla/op_agent/start_ros2.sh`，在第15行，按照实际的安装路径修改：

```bash
source ${YouPath}/autoware_universe/autoware/install/setup.bash
```

在第18行，按照实际的安装路径修改启动Autoware的launch文件路径，按照Town01的路径修改map_path：

```bash
ros2 launch \
  ${YouPath}/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml \
  map_path:=${YouPath}/autoware_universe/autoware/src/${map_name} \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit
```

#### （5）修改Autoware主启动文件

修改文件: `src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml`

在第11行添加如下，用于将Carla点云转换为Autoware.universe使用的点云：

```xml
<arg name="launch_carla_interface" default="true" description="convert carla sensor data to autoware suitable format"/>
```

在第17行将launch_sensing_driver中的true改成false，关闭自带的传感器驱动：

```xml
<arg name="launch_sensing_driver" default="false" description="launch sensing driver"/>
```

在第23行将use_sim_time中的false改成true，使用仿真时间：

```xml
<arg name="use_sim_time" default="true" description="use_sim_time"/>
```

**注意**：这里改了之后，如果再使用非仿真测试时要改回来，否则将不能定位。另外也可以不改参数文件，在launch时添加参数：

```bash
ros2 launch \
  ${YouPath}/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml \
  map_path:=${YouPath}/${map_name} \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit \
  use_sim_time:=true
```

在第52行添加如下，将Carla点云转化为Universe可用的点云：

```xml
<!-- CARLA -->
<group if="$(var launch_carla_interface)">
  <node pkg="carla_pointcloud" exec="carla_pointcloud_node" name="carla_pointcloud_interface" output="screen"/>
</group>
```

#### （6）修改GNSS配置

修改文件: `src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/gnss.launch.xml`

在第4行将coordinate_system中的1改成2，坐标系使用MGRS：

```xml
<arg name="coordinate_system" default="2" description="0:UTM, 1:MGRS, 2:PLANE"/>
```

在第5行添加如下：

```xml
<arg name="plane_zone" default="0"/>
```

#### （7）修改传感器标定配置

修改文件：`src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_description/config/sensor_kit_calibration.yaml`

完整替换为：

```yaml
sensor_kit_base_link:
  camera0/camera_link:
    x: 0.7
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  camera1/camera_link:
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  camera2/camera_link:
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  camera3/camera_link:
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  camera4/camera_link:
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  camera5/camera_link:
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  traffic_light_right_camera/camera_link:
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  traffic_light_left_camera/camera_link:
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  velodyne_top_base_link:
    x: 0.0
    y: 0.0
    z: 0.8
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  velodyne_left_base_link:
    x: -0.5
    y: 0.0
    z: 0.8
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  velodyne_right_base_link:
    x: 0.5
    y: 0.0
    z: 0.8
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  gnss_link:
    x: 0.0
    y: 0.0
    z: 0.8
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  tamagawa/imu_link:
    x: 0.0
    y: 0.0
    z: 0.8
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
```

#### （8）修改传感器基础标定配置

修改文件：`src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_description/config/sensors_calibration.yaml`

完整替换为：

```yaml
base_link:
  sensor_kit_base_link:
    x: 0.0
    y: 0.0
    z: 1.6
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  velodyne_rear_base_link:
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
```

#### （9）修改环境变量

```bash
gedit ~/.bashrc
```

根据安装位置修改路径后，再添加到环境变量中：

```bash
export CARLA_ROOT=/yourpath/carla
export SCENARIO_RUNNER_ROOT=/home/username/autoware_universe/op_carla/scenario_runner
export LEADERBOARD_ROOT=/home/username/autoware_universe/op_carla/op_bridge
export TEAM_CODE_ROOT=/home/username/autoware_universe/op_carla/op_agent
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/util
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/agents
# 注意下面的文件名称要对，可以去Carla文件夹看看(我的是，3.7)
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg
```

### 2.2 重新编译并运行

把上面改动的几个包重新编译（ROS2需要把所有文件复制到install，因此包里任何文件修改均需重新编译）：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select autoware_launch sample_sensor_kit_description
```

安装依赖：

```bash
pip install py-trees networkx tabulate transforms3d
sudo apt-get install ros-galactic-sensor-msgs-py
```

运行Carla服务器：

```bash
cd $CARLA_ROOT
./CarlaUE4.sh -quality-level=Epic -world-port=2000 -resx=800 -resy=600
```

再打开一个终端运行Universe，同时启动Carla ros2 bridge：

```bash
cd ${YouPath}/autoware_universe/op_carla/op_bridge/op_scripts
source ~/autoware_universe/install/setup.bash
./run_exploration_mode_ros2.sh
```

成功加载地图后可以进行自动驾驶测试。

## 三、更换地图（Town10HD服务器默认地图）调试

#### （1）准备地图文件

首先创建 Town10HD 文件夹，将 Town10HD.pcd 和 Town10HD.osm 复制到文件夹中：

- Town10HD.pcd 重命名为 pointcloud_map.pcd
- Town10HD.osm 重命名为 lanelet2_map.osm

#### （2）修改运行脚本

修改 `run_exploration_mode_ros2.sh` 文件：

```bash
sudo gedit op_carla/op_bridge/op_scripts/run_exploration_mode_ros2.sh
```

在第25行，更改地图为Town10HD：

```bash
export FREE_MAP_NAME="Town10HD" 
```

#### （3）修改启动脚本

修改 `start_ros2.sh` 文件：

```bash
sudo gedit op_carla/op_agent/start_ros2.sh
```

在第18行，按照实际的安装路径修改启动Autoware的launch文件路径，按照Town10HD的路径修改map_path：

```bash
ros2 launch ${You_path}/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml map_path:=${map_path_name} vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

再次运行就可以加载Town10HD地图（服务器默认的地图），并测试Open-planner功能。

## 四、测试其他功能

### 4.1 测试倒车入库功能

由于之前下载的高精地图中只含有车道元素，因此我们按照相关方法绘制停车场与停车位进行测试。

但是无法倒车，查看源码之后发现Carla bridge并未写关于倒车的代码，无法倒车入库，因此需要修改 `op_ros2_agent.py` 文件的速度控制函数：

```python
def on_autoware_universe_vehicle_control(self, data):
    # print(' $$$$$$$$$$$$$ >>>> Steering Angle: ', data.lateral.steering_tire_angle)
    cmd = carla.VehicleControl()  
    cmd.steer = (-data.lateral.steering_tire_angle / self.max_steer_angle)*self.steering_factor
    # 源码没有考虑倒车
    # speed_diff = data.longitudinal.speed - self.speed 
    # if speed_diff > 0:            
    #     cmd.throttle = 0.75           
    #     cmd.brake = 0.0  
    # elif speed_diff < 0.0:
    #     cmd.throttle = 0.0
    #     if data.longitudinal.speed <= 0.0 :                
    #         cmd.brake = 0.75
    #     elif  speed_diff > -1:
    #         cmd.brake = 0.0
    #     else :
    #         cmd.brake = 0.01

    if data.longitudinal.speed >= 0:
        cmd.reverse = 0
        cmd.gear = 1
    else:
        cmd.reverse = 1
        cmd.gear = -1

    if data.longitudinal.acceleration > 0:            
        cmd.throttle = 0.4        
        cmd.brake = 0.0
    elif data.longitudinal.acceleration <= 0.0:
        cmd.throttle = 0.0
        if(- data.longitudinal.acceleration > 1.0):
            cmd.brake = 1.0
        else:
            cmd.brake = - data.longitudinal.acceleration//1.5

    # cmd.steer = -data.lateral.steering_tire_rotation_angle 
    self.current_control = cmd
    self.step_mode_possible = True
```

之后就可以正常倒车（但是轨迹跟踪不太好，估计是车身参数导致的，后面可以优化）。

另外，由于雷达安装位置的不同，导致车体影响了点云，因此在Carla（`src/universe/external/open_planner/op_carla_bridge/carla_pointcloud/src/carla_pointcloud/carla_pointcloud_interface_node.cpp`）中添加以下代码，用于剔除雷达一定范围内的点云：

```cpp
else
{
    // std::cout << "CARLA CARLA >>>> Received Cloud : " << scanMsg << ", Converted: " << std::endl;
    // 过滤掉距离传感器较近的点
    pcl::PointCloud<pcl::PointXYZI>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*scanMsg, *xyz_cloud);
    for (size_t i = 0; i < xyz_cloud->points.size(); ++i)
    {
        if (sqrt(xyz_cloud->points[i].x * xyz_cloud->points[i].x + xyz_cloud->points[i].y * xyz_cloud->points[i].y +
                xyz_cloud->points[i].z * xyz_cloud->points[i].z) >= 3.0 && !isnan(xyz_cloud->points[i].z))
        {
            pcl_output->points.push_back(xyz_cloud->points.at(i));
        }
    }
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*pcl_output, output);
    output.header = scanMsg->header;
    
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    if (pcl_ros::transformPointCloud(tf_output_frame_, output, transformed_cloud, *tf_buffer_))
    {
        transformed_cloud.header.stamp = scanMsg->header.stamp;
        velodyne_points_localization->publish(transformed_cloud);
        velodyne_points_perception->publish(transformed_cloud);
    }
}
```

## 总结

通过本文的详细步骤，我们成功实现了 Autoware.universe 与 Carla 二进制版的联调。主要涉及以下几个方面：

1. **Carla基础使用**：包括安装、配置和基本API的使用
2. **OpenPlanner集成**：下载相关代码包和地图文件
3. **配置文件修改**：修改网络、传感器、启动等相关配置文件
4. **环境变量设置**：配置Carla相关的Python路径
5. **编译和运行**：重新编译相关包并启动联调
6. **功能测试**：包括地图切换和倒车入库等功能测试

这套联调系统为自动驾驶算法的开发和测试提供了一个强大的仿真平台，可以在安全的虚拟环境中验证各种驾驶场景和算法性能。

## 参考资料

- [Carla官方文档](https://carla.readthedocs.io/)
- [Autoware Universe官方文档](https://autowarefoundation.github.io/autoware-documentation/)
- [OpenPlanner相关项目](https://github.com/hatem-darweesh)
- 相关博客和技术文章

---

*原文作者：ZARD帧心*  
*发布时间：2023-08-27*  
*更新时间：2023-08-27*

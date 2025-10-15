# Autoware Planning Simulation Tutorial

本文档基于Autoware官方文档，详细介绍了如何使用Planning Simulation进行自动驾驶仿真测试。

## 目录

1. [准备工作](#准备工作)
2. [基础仿真](#基础仿真)
3. [高级仿真](#高级仿真)
4. [使用Autoware Launch GUI](#使用autoware-launch-gui)
5. [自定义地图使用](#自定义地图使用)
6. [提高仿真速度](#提高仿真速度)

## 准备工作

### 下载示例地图

下载并解压示例地图文件：

```bash
gdown -O ~/autoware_map/ 'https://docs.google.com/uc?export=download&id=1499_nsbUbIeturZaDj7jhUownh5fvXHd'
unzip -d ~/autoware_map ~/autoware_map/sample-map-planning.zip
```

> **注意**：示例地图版权归TIER IV, Inc.所有，Copyright 2020。

您也可以[手动下载地图](https://drive.google.com/file/d/1499_nsbUbIeturZaDj7jhUownh5fvXHd/view?usp=sharing)。

### 检查必要文件

检查是否存在 `~/autoware_data` 文件夹及其内容：

```bash
cd ~/autoware_data
ls -C -w 30
```

应该显示以下文件：

```
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

如果没有这些文件，请参考[手动下载artifacts](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/artifacts)说明。

### 配置最大速度

默认情况下，最大速度被限制为15km/h。您可以修改[maximum velocity](https://github.com/autowarefoundation/autoware_launch/blob/c03bd4bdb70117efffc328e5fe6e57426f169b3b/autoware_launch/config/planning/scenario_planning/common/common.param.yaml#L3)参数来调整。

## 基础仿真

### 车道行驶场景

#### 1. 启动Autoware

```bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

> **警告**：注意不能使用 `~` 代替 `$HOME`，否则地图加载会失败。

启动成功后，您将看到RViz界面显示地图和车辆模型。

#### 2. 设置初始位置

![设置初始位置](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/lane-following/set-initial-pose.png)

a) 点击工具栏中的 `2D Pose estimate` 按钮，或按 `P` 键。

b) 在3D视图面板中，点击并按住左键，然后拖拽设置初始位置的方向。正确设置后会显示代表车辆的图像。

> **警告**：记住要将车辆的初始位置设置为与车道相同的方向。要确认车道方向，请查看地图上显示的箭头。

#### 3. 设置目标位置

a) 点击工具栏中的 `2D Goal Pose` 按钮，或按 `G` 键。

b) 在3D视图面板中，点击并按住左键，然后拖拽设置目标位置的方向。如果设置正确，您会看到从初始位置到目标位置的规划路径。

![设置目标位置](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/lane-following/set-goal-pose.png)

#### 4. 启动自动驾驶

现在您可以通过点击 `AutowareStatePanel` 中 `OperationMode` 的 `AUTO` 按钮来启动车辆自动驾驶。

或者，您可以通过运行以下命令手动启动车辆：

```bash
source ~/autoware/install/setup.bash
ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode {}
```

之后，您可以看到 `OperationMode` 上显示 `AUTONOMOUS` 标志，`AUTO` 按钮变为灰色。

![开始驾驶](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/lane-following/start-driving.png)

### 停车场景

1. 设置初始位置和目标位置，并启动自动驾驶。

![设置目标位置后](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/parking/after-set-goal-pose.png)

2. 当车辆接近目标时，它将从车道行驶模式切换到停车模式。

3. 之后，车辆将倒车进入目标停车位。

![停车操作](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/parking/parking-maneuver.png)

### 路边停车和驶出场景

1. 在驶出场景中，将车辆设置在路肩。

![路边停车和驶出](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/pullover-pullout/pullover-pullout.jpg)

2. 设置目标并启动车辆。

3. 在路边停车场景中，同样在车道中设置车辆，并在路肩设置目标。

### 变道场景

#### 1. 下载并解压西新宿地图

```bash
gdown -O ~/autoware_map/ 'https://github.com/tier4/AWSIM/releases/download/v1.1.0/nishishinjuku_autoware_map.zip'
unzip -d ~/autoware_map ~/autoware_map/nishishinjuku_autoware_map.zip
```

#### 2. 使用西新宿地图启动Autoware

```bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/nishishinjuku_autoware_map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

![打开西新宿地图](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/lane-change/open-nishishinjuku-map.png)

#### 3. 在相邻车道设置初始位置和目标位置

![设置位置和目标](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/lane-change/set-position-and-goal.png)

#### 4. 启动车辆

车辆将沿着规划路径进行变道。

![变道过程](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/lane-change/lane-changing.png)

### 避障场景

#### 1. 在同一车道设置初始位置和目标位置

路径将被规划出来。

![设置位置和目标](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/avoidance/set-position-and-goal.png)

#### 2. 在路边设置"2D Dummy Bus"

新的路径将被重新规划。

![设置虚拟巴士](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/avoidance/set-dummy-bus.png)

#### 3. 启动车辆

车辆将沿着新规划的路径避开障碍物。

## 高级仿真

### 放置虚拟对象

#### 1. 放置虚拟车辆或行人

1. 点击工具栏中的 `2D Dummy Car` 或 `2D Dummy Pedestrian` 按钮。
2. 通过在地图上点击和拖拽来设置虚拟对象的位置。
3. 在 `Tool Properties -> 2D Dummy Car/Pedestrian` 面板中设置对象的速度。

> **注意**：速度参数的更改只会影响更改参数后放置的对象。

![设置虚拟车辆](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/lane-following/place-dummy-car.png)

#### 2. 删除虚拟对象

点击工具栏中的 `Delete All Objects` 按钮来删除视图中放置的所有虚拟对象。

#### 3. 交互式虚拟对象

1. 点击工具栏中的 `Interactive` 按钮使虚拟对象变为交互式。

![设置交互式虚拟车辆](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/lane-following/check-interactive.png)

2. 添加交互式虚拟对象：按住 `SHIFT` 并右键点击。
3. 删除交互式虚拟对象：按住 `ALT` 并右键点击。
4. 移动交互式虚拟对象：按住右键拖拽对象。

![移动交互式虚拟车辆](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/lane-following/move-dummy-object.png)

### 交通灯识别仿真

默认情况下，地图上的所有交通灯都被视为绿灯。因此，当创建通过有交通灯的交叉口的路径时，车辆会直接通过交叉口而不停车。

以下步骤说明如何设置和重置交通灯，以测试Planning组件的响应。

#### 设置交通灯

1. 转到 `Panels -> Add new panel`，选择 `TrafficLightPublishPanel`，然后按 `OK`。

2. 在 `TrafficLightPublishPanel` 中，设置交通灯的 `ID` 和颜色。

3. 点击 `SET` 按钮。

![设置交通灯](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/traffic-light/set-traffic-light.png)

4. 最后，点击 `PUBLISH` 按钮将交通灯状态发送到仿真器。经过所选交通灯的任何规划路径都会相应改变。

![发送交通灯颜色](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/traffic-light/send-traffic-light-color.png)

#### 查看交通灯ID

默认情况下，RViz应该在地图上显示每个交通灯的ID。您可以通过放大区域或更改视图类型来更仔细地查看ID。

如果ID没有显示，请尝试以下故障排除步骤：

a) 在 `Displays` 面板中，通过切换 `Map > Lanelet2VectorMap > Namespaces` 旁边的三角形图标找到 `traffic_light_id` 主题。

b) 勾选 `traffic_light_id` 复选框。

c) 通过点击 `Map` 复选框两次重新加载主题。

![查看交通灯ID](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/traffic-light/see-traffic-light-ID.png)

#### 更新/重置交通灯

您可以通过选择下一个颜色（在图像中是 `GREEN`）并点击 `SET` 按钮来更新交通灯的颜色。在图像中，车辆前方的交通灯从 `RED` 变为 `GREEN`，车辆重新启动。

![交通灯颜色更新后](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/traffic-light/after-traffic-light-color-update.png)

要从 `TrafficLightPublishPanel` 中移除交通灯，点击 `RESET` 按钮。

[参考视频教程](https://drive.google.com/file/d/1bs_dX1JJ76qHk-SGvS6YF9gmekkN8fz7/view?usp=sharing)

## 使用Autoware Launch GUI

本节提供了使用Autoware Launch GUI进行规划仿真的分步指南，作为基础仿真部分中提供的命令行指令的替代方案。

### Autoware Launch GUI入门

#### 1. 安装

确保您已安装Autoware Launch GUI。[安装说明](https://github.com/autowarefoundation/autoware-launch-gui#installation)。

#### 2. 启动GUI

从应用程序菜单打开Autoware Launch GUI。

![GUI启动截图](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/launch-gui/launch_gui_main.png)

### 启动Planning Simulation

#### 车道行驶场景

1. **设置Autoware路径**：在GUI中，设置您的Autoware安装路径。

![GUI设置Autoware路径截图](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/launch-gui/launch_gui_setup.png)

2. **选择启动文件**：为车道行驶场景选择 `planning_simulator.launch.xml`。

![GUI选择启动文件截图](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/launch-gui/selecting_launch_file.png)

3. **自定义参数**：根据需要调整参数，如 `map_path`、`vehicle_model` 和 `sensor_model`。

![GUI自定义参数截图1](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/launch-gui/customizing-parameters1.png)

![GUI自定义参数截图2](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/launch-gui/customizing-parameters2.png)

4. **启动仿真**：点击启动按钮开始仿真。

![GUI启动仿真截图](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/launch-gui/starting_simulation.png)

5. **任何场景**：从这里，您可以按照以下场景的说明进行操作：

- 车道行驶场景
- 停车场景
- 变道场景
- 避障场景
- 高级仿真

### 监控和管理仿真

- **实时监控**：使用GUI实时监控CPU/内存使用情况和Autoware日志。
- **配置文件管理**：保存您的仿真配置文件，以便在将来的仿真中快速访问。
- **调整参数**：通过GUI轻松实时修改仿真参数。

## 自定义地图使用

上述内容描述了使用示例地图在规划仿真器中进行一些操作的过程。如果您有兴趣使用自己环境的地图运行Autoware，请访问[如何创建矢量地图](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-maps/#creating-maps)部分获取指导。

![规划仿真自定义地图](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/others/psim-custom-map.png)

## 提高仿真速度

原始的Autoware设计用于在广泛的速度范围内运行。但出于安全考虑，默认最大速度被限制为15 km/h。在这种情况下，即使您在RViz面板中将滑块拖到更高的速度，系统也不会允许。

如果您想以更高的速度运行Autoware，可以修改位于autoware_launch的 `config` 目录中的配置文件[common.param.yaml](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/scenario_planning/common/common.param.yaml)中的 `max_vel` 参数。

![common.param.yaml](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/others/common.param.yaml.png)

例如，要将最大速度设置为72 km/h，您需要将值更改为 `20.0`（因为72 km/h等于20 m/s）。

![增加最大速度](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/images/planning/others/increase-max-velocity.png)

## 总结

本教程涵盖了Autoware Planning Simulation的完整使用流程，从基础的车道行驶到复杂的避障场景，以及如何使用GUI界面和自定义配置。通过这些仿真，您可以：

1. **验证规划算法**：测试不同场景下的路径规划和行为规划
2. **调试系统行为**：观察车辆在各种情况下的响应
3. **参数调优**：通过仿真优化系统参数
4. **功能开发**：在安全的仿真环境中开发和测试新功能

Planning Simulation是开发和测试自动驾驶系统的重要工具，为实车测试提供了安全、可控的环境。

---

> **参考资料**：本文档基于[Autoware官方Planning Simulation教程](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/planning-simulation/)整理而成。

# Autoware Universe标定流程学习（一）

## 0. 引言

### 0.1 Autoware不同版本介绍

Autoware官方说明文档：[https://autowarefoundation.github.io/autoware-documentation/main](https://autowarefoundation.github.io/autoware-documentation/main)

详细的Autoware算法层面的文档：[https://autowarefoundation.github.io/autoware.universe/main/](https://autowarefoundation.github.io/autoware.universe/main/)

### 0.2 Autoware.AI

第一个基于 ROS 1 发布的 Autoware 发行版，该存储库包含各种软件包，涵盖自动驾驶技术的不同方面 - 传感、驱动、定位、映射、感知和规划。由于多种原因，很难提高 Autoware.AI 的功能，已于2022年底停止维护。想要使用autoware.ai的功能，建议切换到Autoware.core/universe。

autoware.1.10版本后，标定工具箱需要另外安装。现已推出轻量级标定工具，不依赖于autoware环境，即不需要下载安装编译整个autoware环境，可独立运行。

主要支持ubuntu14.04-indigo、Ubuntu16.04-kinetic、ubuntu18.04-melodic。

### 0.3 Autoware.Auto

基于 ROS 2 发布的第二个 Autoware 发行版。从 Autoware.AI 开发中吸取的经验教训，Autoware.Auto 采用了不同的开发过程来开发 ROS 2 版本的 Autoware，避免了简单地将 Autoware.AI 从 ROS 1 移植到 ROS 2。取而代之的是，用适当的工程实践将代码库从头开始重写的。

auto版本对新工程师的门槛太高，且进行大规模的架构更改非常困难。暂时没有找到属于auto版本的标定模块。

主要支持ubuntu18.04-dashing、ubuntu20.04-foxy、galactic。

### 0.4 Autoware.core/universe

基于ROS2，Autoware Core 继承了 Autoware.Auto 的原始策略，包含基本的运行时和技术组件，可满足自动驾驶系统所需的传感、计算和驱动等基本功能和能力。Universe 模块是 Core 模块的扩展，可由技术开发人员提供，以增强传感、计算和驱动的功能和能力。需要另外安装Calibrtion Tools标定工具，且使用标定工具时，需要编译整个autoware环境。主要支持ubuntu20.04-galactic、ubuntu22.04-humble。

这里我们来学习一下在ubuntu20.04 + ros2 galactic下如何完成学习。这个是根据版本来决定的，比如Autoware.auto就是ubuntu20.04 + ros2 foxy。

### ROS 2版本说明

#### Foxy Fitzroy

- 类型: LTS（长期支持）
- 支持时间: 3年
- 主要特性:
  - 引入了组件生命周期的支持。
  - 增加了多个质量等级1的软件包。
  - 对Windows平台的改进。
- 支持的平台:
  - Ubuntu 20.04 (Focal Fossa)
  - macOS 10.15 (Catalina)
  - Windows 10

#### Galactic Geochelone

- 类型: 非LTS
- 支持时间: 1年
- 主要特性:
  - 提供了对DDS中间件的更广泛支持，默认支持Cyclone DDS。
  - 改进了跨平台兼容性。
  - 加强了许多质量等级1的软件包。
- 支持的平台:
  - Ubuntu 20.04 (Focal Fossa)
  - macOS 10.15 (Catalina)
  - Windows 10

#### Humble Hawksbill

- 类型: LTS（长期支持）
- 支持时间: 5年
- 主要特性:
  - 更加稳定且易于维护。
  - 引入了对DDS安全插件的支持。
  - 增加了多个质量等级1的软件包。
  - 对不同平台和体系结构的改进。
- 支持的平台:
  - Ubuntu 22.04 (Jammy Jellyfish)
  - macOS 12 (Monterey)
  - Windows 11

#### Rolling Ridley

- 类型: 持续更新
- 支持时间: 持续更新，没有固定的支持时间
- 主要特性:
  - 提供最新的功能和改进。
  - 不推荐用于生产环境，因为可能包含尚未完全测试的新功能。
  - 适合希望紧跟ROS 2开发进度的开发者。

Autoware的整体框架和模块主要包括感知和规划两大部分。其中autoware.ai是ros1，autoware.auto是ros2版本，autoware.core是稳定版，Autoware.universe 是开发者版本。

## 1. 环境安装

### 1.1 Docker创建脚本

这里我们选择docker作为

```bash
#!/bin/bash

# Color definitions
CLEAR=' \e[0m'
GREEN=" \033[1;32m"
RED=' \033[0;31m'
YELLOW=' \033[33m'

# Default values
DOCKER_REPO="ubuntu"
VERSION="20.04"
CONTAINER_NAME="docker_autoware"
REFLECT_DIR='/home/xxx/workspace/code'
DOCKER_RUN="docker run"

# Function to display usage
function show_usage() {
    cat <<EOF
Usage: $0 [options] ...
OPTIONS:
    -h, --help             Display this help and exit.
    -n, --name <NAME>      Specify container name (default: ${CONTAINER_NAME}).
    -v, --volume <DIR>     Specify reflect volumes (default: ${REFLECT_DIR}).
    stop                   Stop the running container.

Example: $0 -n my_container -v /mnt/
EOF
}

# Function to build volume arguments
function local_volumes() {
    echo "-v $(pwd):/bst -v /media:/media -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /etc/localtime:/etc/localtime:ro -v ${REFLECT_DIR}:/code"
}

# Function to parse arguments
function parse_arguments() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -n|--name)
                CONTAINER_NAME="$2"
                shift 2
                ;;
            -v|--volume)
                REFLECT_DIR="$2"
                shift 2
                ;;
            -h|--help)
                show_usage
                exit 0
                ;;
            stop)
                docker stop ${CONTAINER_NAME} 1>/dev/null && echo -e "${RED}Stopped container ${CONTAINER_NAME}${CLEAR}"
                exit 0
                ;;
            *)
                echo "Unknown option: $1"
                exit 1
                ;;
        esac
    done
}

# Main function
function main() {
    parse_arguments "$@"

    echo -e "${GREEN}Starting docker container ${CONTAINER_NAME} with image: ${YELLOW}${DOCKER_REPO}:${VERSION} ...${CLEAR}"

    # Check if the container already exists
    if docker ps -a --format "{{.Names}}" | grep -q ${CONTAINER_NAME}; then
        echo -e "${RED}Warning: Removing old container: ${CONTAINER_NAME}${CLEAR}"
        docker stop ${CONTAINER_NAME} 1>/dev/null
        docker rm -v -f ${CONTAINER_NAME} 1>/dev/null
    fi

    # Set DISPLAY variable for X11 forwarding
    export DISPLAY=${DISPLAY:-:0}

    # Run the Docker container
    ${DOCKER_RUN} -it -d \
        --privileged \
        --net=host \
        --name ${CONTAINER_NAME} \
        -e DISPLAY=${DISPLAY} \
        $(local_volumes) \
        ${DOCKER_REPO}:${VERSION}

    if [[ $? -ne 0 ]]; then
        echo -e "${RED}Failed to start docker container ${CONTAINER_NAME}${CLEAR}"
        exit 1
    fi

    echo -e "${GREEN}Container ${CONTAINER_NAME} started successfully!${CLEAR}"
    echo "To log into the container, run: docker exec -it ${CONTAINER_NAME} bash"
}

main "$@"
```

运行脚本时可以选择容器名称和目录映射，使用示例：
```bash
./script.sh -n my_container -v /path/to/local/dir
```

可通过 `stop` 参数停止正在运行的容器：
```bash
./script.sh stop
```

### 1.2 Docker进入脚本

```bash
#!/bin/bash

# Color definitions
NORMAL=" \033[0m"
GREEN=" \033[1;32m"
RED=' \033[0;31m'
CLEAR=' \e[0m'

CONTAINER_NAME="docker_autoware"

function main() {
    [[ -n "$1" ]] && CONTAINER_NAME=$1

    echo "Container: ${CONTAINER_NAME}"
    echo -e "${GREEN}Hello, Enjoy bst_qnx!${CLEAR}"

    # Execute shell in the Docker container
    if ! docker exec -it "${CONTAINER_NAME}" /bin/bash; then
        echo -e "${RED}Failed to exec into container ${CONTAINER_NAME}!${CLEAR}"
        exit 1
    fi
}

main "$@"
```

到这一步我们的docker环境就安装好了，并支持可视化显示，我们用clock来测试：

```bash
sudo apt-get install xarclock
xarclock
```

### 1.3 Autoware universe环境安装

这里我们选择鱼香ros一键安装，并安装依赖的内容。这里我们都是用`sudo`，在docker中实际上是不需要的。

```bash
sudo apt-get update
sudo apt --fix-broken install 
sudo apt install fonts-wqy-microhei wget git locate vim
wget http://fishros.com/install -O fishros && . fishros
```

然后再将rosdep也使用这个脚本安装一下，到这一步环境就装完了ros环境，下面就是来装Autoware universe这个项目了首先要对pacmod进行安装：

```bash
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env

# Taken from https://github.com/astuff/pacmod3#installation
sudo apt install apt-transport-https
sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'
sudo apt-get update
sudo apt install ros-${rosdistro}-pacmod3
```

然后就是下载Autoware universe并拉repo文件，并完成ros2的编译：

```bash
git clone https://github.com/autowarefoundation/autoware.git -b galactic # 这个也支持，只是标定的时候需要变一下脚本
# git https://github.com/leo-drive/autoware.tutorial_vehicle.git # 这个是官方版本
cd autoware
mkdir src
pip install vcstool
vcs import src < autoware.repos #用于从一个 .repos 文件中导入多个 Git 仓库

source /opt/ros/galactic/setup.bash
rosdepc update --include-eol-distros ##这个是Galactic版本需要的
rosdepc install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

常见的安装问题都可以在相关博客中找到解决方法。最终编译结束后如下图所示。

在编译阶段我们可以选择多种编译方式，可以实现单包编译：

```bash
#由于系统比较庞大，需多次执行，直到重复出现一个错误，再去排查
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
# 只编译指定包
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select 包名
# 忽略指定包
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-ignore 包名
# 遇到编译错误继续编译其他模块
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --continue-on-error
#单线程编译
MAKEFLAGS="-j1" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
```

### 1.4 地图下载以及demo运行

#### 1.4.1 下载地图

可以选择手动下载地图或使用命令行下载（推荐）。

手动下载：
- 下载地址：[地图下载链接](https://drive.google.com/file/d/1499_nsbUbIeturZaDj7jhUownh5fvXHd/view?usp=sharing)

命令行下载（推荐）：

```bash
gdown -O ~/autoware_map/ 'https://docs.google.com/uc?export=download&id=1499_nsbUbIeturZaDj7jhUownh5fvXHd'
unzip -d ~/autoware_map  ~/autoware_map/sample-map-planning.zip
```

#### 1.4.2 检查文件夹

检查是否存在 `~/autoware_data` 文件夹及其内容：

```bash
mv ~/autoware_data ~/autoware/map_data/
cd  ~/autoware/map_data/
ls -C -w 30
```

应显示以下文件：

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

如果没有 `~/autoware_data` 文件夹，请使用以下步骤手动下载：

#### 1.4.3 安装 Ansible 收集程序

```bash
cd ~/autoware # 进入克隆的仓库根目录
ansible-galaxy collection install -f -r "ansible-galaxy-requirements.yaml"
```

#### 1.4.4 运行 Ansible Playbook

运行以下命令以下载并解压工件到指定目录：

```bash
ansible-playbook autoware.dev_env.download_artifacts -e "data_dir=$HOME/autoware_data" --ask-become-pass
```

#### 1.4.5 运行程序

准备工作完成后，您可以运行以下命令启动 Autoware 的规划模拟器：

```bash
source install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware/map_data/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

这样，就可以开始使用 Autoware 进行自动驾驶仿真了。

## 2. Autoware 标定文件包

### 2.1 标定文件包安装

这个标定文件其实主要的就是依赖CalibrationTools库来完成训练学习的。我们后面主要的就是看这个部分：

```bash
cd autoware
wget https://raw.githubusercontent.com/tier4/CalibrationTools/tier4/universe/calibration_tools.repos # 这个里面bug有点多，可以直接替换下面的calibration_tools.repos文件
vcs import src < calibration_tools.repos
rosdepc install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
 
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

如果repo拉不下来可以将下面的内容创建为一个calibration_tools.repos文件：

```yaml
repositories:
  autoware/calibration_tools:
    type: git
    url: https://github.com/tier4/CalibrationTools.git
    version: galactic
  vendor/lidartag:
    type: git
    url: https://github.com/tier4/LiDARTag.git
    version: galactic
  vendor/lidartag_msgs:
    type: git
    url: https://github.com/tier4/LiDARTag_msgs.git
    version: tier4/universe
  vendor/apriltag_msgs:
    type: git
    url: https://github.com/christianrauch/apriltag_msgs.git
    version: 2.0.0
  vendor/apriltag_ros:
    type: git
    url: https://github.com/christianrauch/apriltag_ros.git
    version: dashing
  vendor/ros2_numpy:
    type: git
    url: https://github.com/Box-Robotics/ros2_numpy.git
    version: rolling
  vendor/image_pipeline:
    type: git
    url: https://github.com/tier4/image_pipeline.git
    version: 47964112293eb19f9f57254b2e6b68706954cc63
```

### 2.2 数据集

官方对数据集阐述在这里：[https://autowarefoundation.github.io/autoware-documentation/main/datasets/#leo-drive-isuzu-sensor-data](https://autowarefoundation.github.io/autoware-documentation/main/datasets/#leo-drive-isuzu-sensor-data)。包含1个Camera，2个Lidar，1个GNSS/INS。autoware有tutorial_vehicle模型的官方分支，但是适配最新的ubuntu22.04+ros2 humble。

如果你使用的ros2版本正好是humble，那么可以完全跟着官方教程走，但若不是humble，比如我使用的是ubuntu 20.04+ ros2 galactic，就算安装依赖时没问题，但在编译calibrtion_tools时会出现模型xxx未定义的问题，这是由于tutorial_vehicle模型数据对应的存储库使用的方法全部默认适配最新的humble版本，对应的一些模型autoware-galactic版本没有。

首先我们会先安装applanix来解析原始消息包：

```bash
# Create a workspace and clone the repository
mkdir -p ~/applanix_ws/src && cd "$_"
git clone https://github.com/autowarefoundation/applanix.git
cd ..
 
# Build the workspace
colcon build --symlink-install --packages-select applanix_msgs
 
# Source the workspace
source ~/applanix_ws/install/setup.bash
 
# Now you can play back the messages
```

数据下载：

```bash
# Install awscli
sudo apt update && sudo apt install awscli -y
 
# This will download the entire dataset to the current directory.
# (About 10.9GB of data)
aws s3 sync s3://autoware-files/collected_data/2022-08-22_leo_drive_isuzu_bags/ ./2022-08-22_leo_drive_isuzu_bags  --no-sign-request
 
# Optionally,
# If you instead want to download a single bag file, you can get a list of the available files with following:
aws s3 ls s3://autoware-files/collected_data/2022-08-22_leo_drive_isuzu_bags/ --no-sign-request
#    PRE all-sensors-bag1_compressed/
#    PRE all-sensors-bag2_compressed/
#    PRE all-sensors-bag3_compressed/
#    PRE all-sensors-bag4_compressed/
#    PRE all-sensors-bag5_compressed/
#    PRE driving_20_kmh_2022_06_10-16_01_55_compressed/
#    PRE driving_30_kmh_2022_06_10-15_47_42_compressed/
 
# Then you can download a single bag file with the following:
aws s3 sync s3://autoware-files/collected_data/2022-08-22_leo_drive_isuzu_bags/all-sensors-bag1_compressed/ ./all-sensors-bag1_compressed  --no-sign-request
```

下载后的数据格式为.deb3.zstd格式，解压后再进行数据查看：

```bash
zstd -d xxx.db3.zstd -o xxx.db3
ros2 bag info xxx.db3
```

## 3. 校准逻辑

详情请参照[校准逻辑](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-vehicle-and-sensor-model/calibrating-sensors/extrinsic-manual-calibration/)

对于 `tutorial_vehicle`数据集，有四个传感器（两个激光雷达，一个相机，一个 GNSS/INS），因此应为：

```xml
<param name="child_frames" value="[velodyne_top_base_link, livox_front_left_base_link, ...]"/>
```

### calibrator.launch.xml

最后，我们将为每个传感器的手动校准器启动一个实例，在 `calibrator.launch.xml` 启动文件参数中更新命名空间（ns）和子帧参数：

```xml
<!-- extrinsic_manual_calibrator -->
<include file="$(find-pkg-share extrinsic_manual_calibrator)/launch/calibrator.launch.xml">
  <arg name="ns" value="$(var parent_frame)/rs_helios_top_base_link"/>
  <arg name="parent_frame" value="$(var parent_frame)"/>
  <arg name="child_frame" value="rs_helios_top_base_link"/>
</include>

<include file="$(find-pkg-share extrinsic_manual_calibrator)/launch/calibrator.launch.xml">
  <arg name="ns" value="$(var parent_frame)/rs_bpearl_front_base_link"/>
  <arg name="parent_frame" value="$(var parent_frame)"/>
  <arg name="child_frame" value="rs_bpearl_front_base_link"/>
</include>

<include file="$(find-pkg-share extrinsic_manual_calibrator)/launch/calibrator.launch.xml">
  <arg name="ns" value="$(var parent_frame)/camera0/camera_link"/>
  <arg name="parent_frame" value="$(var parent_frame)"/>
  <arg name="child_frame" value="camera0/camera_link"/>
</include>

<include file="$(find-pkg-share extrinsic_manual_calibrator)/launch/calibrator.launch.xml">
  <arg name="ns" value="$(var parent_frame)/gnss_link"/>
  <arg name="parent_frame" value="$(var parent_frame)"/>
  <arg name="child_frame" value="gnss_link"/>
</include>
```

### manual_sensor_kit.launch.xml与calibrator.launch.xml结合

`tutorial_vehicle` 的 `manual_sensor_kit.launch.xml` 最终版本应该是这样的：

```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
    <arg name="vehicle_id" default="tutorial_vehicle"/> <!-- You can update with your own vehicle_id -->
    <let name="sensor_model" value="tutorial_vehicle_sensor_kit"/> <!-- You can update with your own sensor model -->
    <let name="parent_frame" value="sensor_kit_base_link"/>

    <!-- extrinsic_calibration_client -->
    <arg name="src_yaml" default="$(find-pkg-share individual_params)/config/$(var vehicle_id)/$(var sensor_model)/sensor_kit_calibration.yaml"/>
    <arg name="dst_yaml" default="$(env HOME)/sensor_kit_calibration.yaml"/>

    <node pkg="extrinsic_calibration_client" exec="extrinsic_calibration_client" name="extrinsic_calibration_client" output="screen">
        <param name="src_path" value="$(var src_yaml)"/>
        <param name="dst_path" value="$(var dst_yaml)"/>
    </node>

    <!-- extrinsic_calibration_manager -->
    <node pkg="extrinsic_calibration_manager" exec="extrinsic_calibration_manager" name="extrinsic_calibration_manager" output="screen">
        <param name="parent_frame" value="$(var parent_frame)"/>
        <!-- Please Update with your own sensor frames -->
        <param name="child_frames" value="
    [rs_helios_top_base_link,
    rs_bpearl_front_base_link,
    camera0/camera_link,
    gnss_link]"/>
    </node>

    <!-- extrinsic_manual_calibrator -->
    <!-- Please create a launch for all sensors that you used. -->
    <include file="$(find-pkg-share extrinsic_manual_calibrator)/launch/calibrator.launch.xml">
        <arg name="ns" value="$(var parent_frame)/rs_helios_top_base_link"/>
        <arg name="parent_frame" value="$(var parent_frame)"/>
        <arg name="child_frame" value="rs_helios_top_base_link"/>
    </include>

    <include file="$(find-pkg-share extrinsic_manual_calibrator)/launch/calibrator.launch.xml">
        <arg name="ns" value="$(var parent_frame)/rs_bpearl_front_base_link"/>
        <arg name="parent_frame" value="$(var parent_frame)"/>
        <arg name="child_frame" value="rs_bpearl_front_base_link"/>
    </include>

    <include file="$(find-pkg-share extrinsic_manual_calibrator)/launch/calibrator.launch.xml">
        <arg name="ns" value="$(var parent_frame)/camera0/camera_link"/>
        <arg name="parent_frame" value="$(var parent_frame)"/>
        <arg name="child_frame" value="camera0/camera_link"/>
    </include>

    <include file="$(find-pkg-share extrinsic_manual_calibrator)/launch/calibrator.launch.xml">
        <arg name="ns" value="$(var parent_frame)/gnss_link"/>
        <arg name="parent_frame" value="$(var parent_frame)"/>
        <arg name="child_frame" value="gnss_link"/>
    </include>
</launch>
```

### 校准传感器

完成 `manual.launch.xml` 和 `manual_sensor_kit.launch.xml` 文件后，我们需要构建包：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select extrinsic_calibration_manager
```

现在我们准备启动并使用手动校准器：

```bash
ros2 launch extrinsic_calibration_manager calibration.launch.xml mode:=manual sensor_model:=<这个就是我们刚刚在launch中创建的名字----YOUR-OWN-SENSOR-KIT-NAME> vehicle_model:=<OWN-VEHICLE-MODEL> vehicle_id:=<VEHICLE-ID>
```

例如，对于 `tutorial_vehicle`：

```bash
ros2 launch extrinsic_calibration_manager calibration.launch.xml mode:=manual sensor_model:=tutorial_vehicle_sensor_kit vehicle_model:=tutorial_vehicle vehicle_id:=tutorial_vehicle 	
ros2 launch extrinsic_calibration_manager calibration.launch.xml mode:=manual sensor_model:=sample_sensor_kit vehicle_model:=sample_vehicle vehicle_id:=sample_vehicle
```

然后播放 ROS 2 包文件：

```bash
ros2 bag play <rosbag_path> --clock -l -r 0.2 --remap /tf:=/null/tf /tf_static:=/null/tf_static # 如果已记录 tf
```

您将看到一个手动的 `rqt_reconfigure` 窗口，我们将根据 `rviz2` 结果手动更新校准。

按下刷新按钮，然后按下展开所有按钮。`tutorial_vehicle` 的帧应如下所示，其他几章类似参考官网的配置即可。

## 4. 参考链接

- [ros2不同版本的差异 foxy galactic humble rolling](https://blog.csdn.net/zyh821351004/article/details/130180172)
- [Autoware不同版本的学习及标定工具的使用](https://blog.csdn.net/qq_56719965/article/details/138266422)
- [wsl2+Ubuntu22(20)安装Autowae.universe（全自动，使用官方工具一键配置环境！）](https://blog.csdn.net/weixin_44663617/article/details/135184938)
- [【基于Ubuntu20.04的Autoware.universe安装过程】方案一：虚拟机 | 详细记录 | Vmware | 全过程图文](https://blog.csdn.net/Akaxi1/article/details/136281377)
- [Autoware.universe配置](https://blog.csdn.net/m0_61291722/article/details/128075862)
- [Autoware安装教程](https://blog.csdn.net/qq_46101866/article/details/140473291)

> 本文档来源：[Autoware universe标定流程学习（一）- CSDN博客](https://blog.csdn.net/lovely_yoshino/article/details/143141911)

# Autoware版本兼容性指南

## Autoware三个主要版本的区别与维护状态

### 版本演进历史

Autoware项目经历了几个重要的发展阶段，目前主要有三个版本分支：

| 版本 | 发布时间 | ROS版本 | 维护状态 | 推荐程度 |
|------|----------|---------|----------|----------|
| **Autoware.AI** | 2015-2020 | ROS1 | ❌ 停止维护 | ⚠️ 不推荐 |
| **Autoware.Auto** | 2019-2023 | ROS2 | ❌ 停止维护 | ⚠️ 不推荐 |
| **Autoware Universe** | 2021-至今 | ROS2 | ✅ **官方维护** | ⭐ **强烈推荐** |

### 详细版本对比

#### 1. Autoware.AI (已停止维护)
- **开发时期**: 2015年 - 2020年
- **技术栈**: ROS1 (Kinetic, Melodic, Noetic)
- **平台支持**: Ubuntu 16.04/18.04/20.04
- **架构特点**: 
  - 基于ROS1的分布式架构
  - 相对简单的模块化设计
  - 主要针对学术研究和原型开发
- **维护状态**: ❌ **已停止维护** (2020年后)
- **适用场景**: 
  - 不再推荐用于新项目
  - 仅适合学习和参考历史代码

#### 2. Autoware.Auto (已停止维护)
- **开发时期**: 2019年 - 2023年
- **技术栈**: ROS2 (Foxy, Galactic)
- **平台支持**: Ubuntu 18.04/20.04
- **架构特点**:
  - 基于ROS2的现代化架构
  - 强调软件质量和测试覆盖
  - 采用严格的代码质量标准
  - 面向商业化应用设计
- **维护状态**: ❌ **已停止维护** (2023年后)
- **停止原因**: 
  - 社区资源集中到Autoware Universe
  - 功能逐步迁移到Universe版本

#### 3. Autoware Universe (当前主力版本) ⭐
- **开发时期**: 2021年 - 至今
- **技术栈**: ROS2 (Humble为主，部分支持Galactic)
- **平台支持**: Ubuntu 22.04 (主要), Ubuntu 20.04 (部分支持)
- **架构特点**:
  - **微自治架构**: 模块化设计，便于替换和扩展
  - **双层包管理**: Core + Universe包管理体系
  - **标准化接口**: 内部组件接口和外部AD API
  - **现代化工具链**: 支持Docker、CI/CD、自动化测试
- **维护状态**: ✅ **官方积极维护**
- **核心优势**:
  - 官方长期支持和更新
  - 活跃的社区贡献
  - 完整的仿真和测试环境
  - 与最新硬件和传感器兼容
  - 支持CARLA 0.9.15官方集成

### 包管理体系对比

#### Autoware Core vs Universe
```
Autoware Universe = Autoware Core + Community Packages
```

| 分类 | 描述 | 维护方 | 质量标准 |
|------|------|--------|----------|
| **Autoware Core** | 基础核心包 | Autoware Foundation | 严格的质量标准 |
| **Universe Packages** | 社区贡献包 | 外部开发者/机构 | 各自维护标准 |

### 维护状态详情

#### 当前维护情况
- **Autoware Universe**: 
  - ✅ 官方积极维护
  - ✅ 定期发布更新
  - ✅ 活跃的社区支持
  - ✅ 完整的文档和教程

#### 历史版本状态
- **Autoware.AI**: 
  - ❌ 2020年后停止官方维护
  - ❌ 不再接收新功能
  - ❌ 仅有社区零星维护

- **Autoware.Auto**: 
  - ❌ 2023年正式停止维护
  - ❌ 功能迁移到Universe
  - ❌ 不推荐新项目使用

### 选择建议

#### 🎯 强烈推荐: Autoware Universe
- **适用于**: 所有新项目和生产环境
- **优势**: 官方支持、持续更新、功能完整
- **环境**: Ubuntu 22.04 + ROS2 Humble + CARLA 0.9.15

#### ⚠️ 不推荐: Autoware.AI / Auto
- **原因**: 已停止维护，安全性和兼容性问题
- **例外**: 仅用于学习历史代码或维护遗留系统

## 系统环境配置

### 基础平台要求
- **操作系统**: Ubuntu 22.04 LTS
- **ROS版本**: ROS2 Humble 
- **CARLA版本**: 0.9.15

### GPU和CUDA要求 (可选但推荐)

| 组件 | 版本 | 说明 |
|------|------|------|
| **CUDA** | **12.4** | 官方支持版本 |
| **cuDNN** | **8.9.7.29-1+cuda12.2** | 深度学习库 |
| **TensorRT** | **10.8.0.43-1+cuda12.8** | 推理优化库 |
| **NVIDIA Driver** | 兼容CUDA 12.4 | 通常需要535+版本 |

**重要说明**:
- GPU非必需，但**强烈推荐**用于感知模块（目标检测、交通灯识别等）
- 仅支持官方指定的CUDA版本，其他版本不保证兼容性
- 最低GPU显存推荐: 4GB

## Autoware版本推荐

基于您的环境配置（CARLA 0.9.15、ROS2 Humble、Ubuntu 22.04），推荐使用以下Autoware版本：

### 推荐版本：Autoware Universe (主线版本)

**最新稳定版本**: 
- Repository: `autowarefoundation/autoware`
- Branch: `main` (推荐) 或 `latest`
- 支持的平台: Ubuntu 22.04 + ROS2 Humble

### Autoware Universe官方支持的CARLA版本

根据autoware_carla_interface的官方文档，支持的环境配置为：

| Ubuntu | ROS2 | CARLA | 状态 |
|--------|------|-------|------|
| 22.04  | Humble | **0.9.15** | ✅ 官方支持 |

**注意**: 目前Autoware Universe的autoware_carla_interface **仅官方支持CARLA 0.9.15版本**

### 版本兼容性说明

| 组件 | 版本 | 兼容性状态 |
|------|------|-----------|
| Ubuntu | 22.04 LTS | ✅ 官方支持 |
| ROS2 | Humble | ✅ 官方支持 |
| CARLA | **0.9.15** | ✅ **官方支持** |
| CUDA | **12.4** | ✅ **官方指定版本** |
| cuDNN | **8.9.7.29** | ✅ 官方支持 |
| TensorRT | **10.8.0.43** | ✅ 官方支持 |
| Autoware | Universe (main) | ✅ 推荐使用 |

### CARLA版本特别说明

- **CARLA 0.9.15**: Autoware Universe官方唯一支持的CARLA版本
- **其他CARLA版本**: 目前不被官方支持，可能需要社区维护的桥接方案
- **Python包**: 需要安装专门的`carla-0.9.15-ubuntu-22.04` Python包用于ROS2通信

### CUDA版本特别说明

- **CUDA 12.4**: Autoware Universe官方唯一支持的CUDA版本
- **版本兼容性**: 其他CUDA版本（如11.8、12.0等）可能工作但不被官方支持
- **自动安装**: 使用`./setup-dev-env.sh`脚本会自动安装正确的CUDA版本
- **手动安装**: 如需手动安装，请严格按照官方指定的版本号
- **无GPU运行**: 可使用`./setup-dev-env.sh --no-nvidia`跳过CUDA安装（部分功能受限）

## CARLA-Autoware集成方案

### 1. autoware_carla_interface (官方推荐) ⭐
- **项目地址**: https://github.com/autowarefoundation/autoware_universe/tree/main/simulator/autoware_carla_interface
- **状态**: 集成在autoware_universe中，官方维护
- **支持版本**: 
  - Ubuntu 22.04
  - ROS2 Humble
  - **CARLA 0.9.15** (官方唯一支持版本)
- **兼容性**: 与最新Autoware更新保持同步
- **特点**: 
  - 官方支持，稳定性最高
  - 完整的传感器支持
  - 同步模式支持
  - 交通管理器集成

### 2. 社区维护方案

#### carla_autoware_bridge
- **项目地址**: https://github.com/Robotics010/carla_autoware_bridge
- **状态**: 社区维护
- **兼容性**: 支持CARLA到Autoware Universe的连接

#### zenoh_carla_bridge  
- **项目地址**: https://github.com/evshary/zenoh_carla_bridge
- **特点**: 支持多车辆控制，使用Zenoh协议
- **文档**: https://autoware-carla-launch.readthedocs.io/

### 重要提醒
由于Autoware Universe官方仅支持CARLA 0.9.15，建议：
1. **首选方案**: 使用autoware_carla_interface + CARLA 0.9.15
2. **其他CARLA版本**: 需使用社区维护的桥接方案，稳定性可能有限

## 安装步骤

### 1. 安装Autoware Universe

```bash
# 克隆仓库
git clone https://github.com/autowarefoundation/autoware.git
cd autoware

# 设置开发环境
./setup-dev-env.sh

# 创建工作空间
mkdir src
vcs import src < autoware.repos

# 安装依赖
source /opt/ros/humble/setup.bash
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# 构建
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 2. 配置CARLA集成

#### 安装CARLA 0.9.15相关组件

```bash
# 1. 下载CARLA 0.9.15
# 从 https://carla.readthedocs.io/en/latest/start_quickstart/ 下载

# 2. 安装CARLA Python包 (用于ROS2 Humble通信)
# 方法A: 使用pip安装wheel文件
pip install carla==0.9.15

# 方法B: 下载专用的Python包
# 从 https://github.com/gezp/carla_ros/releases/tag/carla-0.9.15-ubuntu-22.04 下载
# 并将egg文件添加到PYTHONPATH

# 3. 下载CARLA Lanelet2地图
# 从 https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/ 下载
```

#### 配置autoware_carla_interface

```bash
# 1. 设置地图文件夹结构
mkdir -p ~/autoware_map/Town01
# 将下载的地图文件重命名并放置：
# point_cloud/Town01.pcd -> autoware_map/Town01/pointcloud_map.pcd
# vector_maps/lanelet2/Town01.osm -> autoware_map/Town01/lanelet2_map.osm

# 2. 创建地图投影配置文件
echo "projector_type: Local" > ~/autoware_map/Town01/map_projector_info.yaml
```

## 注意事项

1. **版本匹配**: 确保所有组件版本匹配，避免兼容性问题
2. **GPU要求**: 
   - CUDA 12.4是官方唯一支持版本
   - GPU非必需但强烈推荐用于感知功能
   - 最低显存要求: 4GB
3. **依赖管理**: 使用rosdep管理ROS依赖包
4. **内存要求**: 建议16-32GB系统内存，构建时可能需要大量内存
5. **网络配置**: 参考Autoware文档配置网络设置
6. **CUDA故障排除**:
   - 如遇CUDA版本冲突，使用`./setup-dev-env.sh --no-nvidia`跳过
   - 或完全卸载现有CUDA后重新安装

## 验证安装

### 基础环境验证
```bash
# 1. 验证ROS2环境
ros2 --version

# 2. 验证CUDA安装 (可选)
nvcc --version
# 应显示: CUDA compilation tools, release 12.4

# 3. 验证NVIDIA驱动
nvidia-smi
# 应显示GPU信息和CUDA Version: 12.4

# 4. 验证cuDNN和TensorRT (可选)
python3 -c "import tensorflow as tf; print('GPU Available:', tf.config.list_physical_devices('GPU'))"
```

### CARLA和Autoware集成验证
```bash
# 1. 启动CARLA服务器 (0.9.15)
cd /path/to/CARLA_0.9.15
./CarlaUE4.sh -prefernvidia -quality-level=Low -RenderOffScreen

# 2. 验证Autoware与CARLA集成
source install/setup.bash
ros2 launch autoware_launch e2e_simulator.launch.xml \
  map_path:=$HOME/autoware_map/Town01 \
  vehicle_model:=sample_vehicle \
  sensor_model:=awsim_sensor_kit \
  simulator_type:=carla \
  carla_map:=Town01

# 3. 验证CARLA连接
# 在Autoware界面中：
# - 设置初始位置 (Init by GNSS)
# - 设置目标点
# - 等待路径规划
# - 点击Engage开始自动驾驶
```

### CUDA故障排除
```bash
# 如果遇到CUDA版本冲突问题，可以尝试以下方法：

# 方法1: 无GPU模式安装
./setup-dev-env.sh --no-nvidia

# 方法2: 清理CUDA相关包后重新安装
sudo apt purge "cuda*" "libcudnn*" "libnvinfer*" "tensorrt*" "nvidia*"
sudo apt autoremove
./setup-dev-env.sh

# 方法3: 解除CUDA包的hold状态
sudo apt-mark unhold "cuda*" "libcudnn*" "libnvinfer*" "tensorrt*" "nvidia*"
./setup-dev-env.sh
```

## 相关资源

- [Autoware官方文档](https://autowarefoundation.github.io/autoware-documentation/main/)
- [CARLA官方文档](https://carla.readthedocs.io/)
- [ROS2 Humble文档](https://docs.ros.org/en/humble/)
- [Ubuntu 22.04发布说明](https://releases.ubuntu.com/22.04/)

## 更新日期
2025年10月14日

## 版本信息摘要

### 推荐配置 (2025年10月)
- **操作系统**: Ubuntu 22.04 LTS
- **ROS版本**: ROS2 Humble
- **Autoware版本**: Universe (main分支)
- **CARLA版本**: 0.9.15 (官方唯一支持)
- **CUDA版本**: 12.4 (官方唯一支持)
- **cuDNN版本**: 8.9.7.29-1+cuda12.2
- **TensorRT版本**: 10.8.0.43-1+cuda12.8

这个配置组合经过官方测试和长期维护，是最稳定和功能最完整的选择。

# CARLA Path Distributor Example / CARLA経路配信サンプル

In case3, we give an example of path.  
ケース3では、経路の例を示します。

After decided the distance, we can calculate a global path for the vehicle.  
距離を決定した後、車両のグローバル経路を計算できます。

And the ego car will follow this path.  
そして、エゴカーはこの経路に従って走行します。

## 1. Start the CARLA server / CARLAサーバーを開始
    CarlaUE4.sh

## 2. Start the carla_ros_bridge, connect carla and ros2 system / carla_ros_bridgeを開始し、CARLAとROS2システムを接続
    ros2 launch carla_ros_bridge  carla_ros_bridge_with_example_ego_vehicle.launch.py 

## 3. Start the rviz2 to see the sensor output data / RViz2を開始してセンサー出力データを確認
    ros2 run rviz2 rviz2 -d /home/kotei/huqianqian/software/CARLA_0.9.13/carla_ros_bridge/src/ros-bridge/carla_ad_demo/config/carla_ad_demo_ros2.rviz 

## 4. Create a path, which the ego car will followed the path / エゴカーが従う経路を作成
    ros2 launch carla_waypoint_publisher carla_waypoint_publisher.launch.py 

## 5. Make the ego car into autopilot mode / エゴカーをオートパイロットモードにする
The ego car can be in autonomous driving mode, and you can see the sensor data is changed according to the car position.  
エゴカーは自動運転モードになり、車両の位置に応じてセンサーデータが変化するのを確認できます。
    p
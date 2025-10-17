# CARLA ROS Bridge Connection / CARLA ROSブリッジ接続

In case2, we can build a connect bridge between carla and ros2.  
ケース2では、CARLAとROS2の間に接続ブリッジを構築できます。

CARLA can send original sensor data to the ros2 in the format of ros2 message.  
CARLAは、ROS2メッセージ形式で元のセンサーデータをROS2に送信できます。

In ros2 rviz application, we can see the sensor data transate by the carla.  
ROS2 RVizアプリケーションでは、CARLAによって送信されたセンサーデータを確認できます。

## 1. Start the CARLA server / CARLAサーバーを開始
    CarlaUE4.sh

## 2. Start the carla_ros_bridge, connect carla and ros2 system / carla_ros_bridgeを開始し、CARLAとROS2システムを接続
    ros2 launch carla_ros_bridge  carla_ros_bridge_with_example_ego_vehicle.launch.py 

## 3. Start the rviz2 to see the sensor output data / RViz2を開始してセンサー出力データを確認
    ros2 run rviz2 rviz2 -d /home/kotei/huqianqian/software/CARLA_0.9.13/carla_ros_bridge/src/ros-bridge/carla_ad_demo/config/carla_ad_demo_ros2.rviz 

## 4. Make the ego car into autopilot mode / エゴカーをオートパイロットモードにする
The ego car can be in autonomous driving mode, and you can see the sensor data is changed according to the car position.  
エゴカーは自動運転モードになり、車両の位置に応じてセンサーデータが変化するのを確認できます。
    p


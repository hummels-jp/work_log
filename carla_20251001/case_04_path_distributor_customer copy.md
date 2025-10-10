# CARLA Path Distributor with AD Agent / CARLA自動運転エージェント付き経路配信

Case 4 is also the example of distribute a path.  
ケース4も経路配信の例です。

But in this example, we use a ad agent to guide the ego car to follow the path.  
しかし、この例では、自動運転エージェントを使用してエゴカーが経路に従うよう誘導します。

We use the simple car control algorithm named pid method.  
PID法という単純な車両制御アルゴリズムを使用します。

## 1. Start the CARLA server / CARLAサーバーを開始
    CarlaUE4.sh

## 2. Start the carla_ros_bridge, connect carla and ros2 system, without create the ego car / carla_ros_bridgeを開始し、CARLAとROS2システムを接続（エゴカー作成なし）
    ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py

## 3. Start the rviz2 to see the sensor output data / RViz2を開始してセンサー出力データを確認
    ros2 run rviz2 rviz2 -d /home/kotei/huqianqian/software/CARLA_0.9.13/carla_ros_bridge/src/ros-bridge/carla_ad_demo/config/carla_ad_demo_ros2.rviz 

## 4. Create a path, which the ego car will followed the path / エゴカーが従う経路を作成
    ros2 launch carla_waypoint_publisher carla_waypoint_publisher.launch.py 

## 5. Appoint the ad agent for the ego car / エゴカーに自動運転エージェントを配置
    ros2 launch carla_ad_agent  carla_ad_agent.launch.py 

## 6. Release a speed signal / 速度信号を送信
To make the ego car follow the path by the speed value in the speed message.  
速度メッセージの速度値によってエゴカーが経路に従うようにします。
    ros2 topic pub /carla/ego_vehicle/target_speed std_msgs/msg/Float64 "{data: 10.0}"
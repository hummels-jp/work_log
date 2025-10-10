# CARLA Scenario Test - Following Front Car / CARLAシナリオテスト - 前車追従

In case5, we do a scenario test, named followingFrontCar.  
ケース5では、前車追従（followingFrontCar）というシナリオテストを実行します。

## 1. Start the CARLA server / CARLAサーバーを開始
    CarlaUE4.sh

## 2. Start the carla_ad_demo / carla_ad_demoを開始
This is an example of following the front car.  
これは前車追従の例です。
    ros2 launch carla_ad_demo carla_ad_demo_with_scenario.launch.py
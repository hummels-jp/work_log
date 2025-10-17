# CARLA Scenario Runner Test Cases / CARLAシナリオランナーテストケース

In case6, there are a lot of test case, we can config the parameter, and test the ad algorithm weather the ad algorithm is strong enough to pass all the scenario test.  
ケース6では、多くのテストケースがあり、パラメータを設定して、自動運転アルゴリズムがすべてのシナリオテストに合格するのに十分強力かどうかをテストできます。

## 1. Start the CARLA server / CARLAサーバーを開始
    CarlaUE4.sh

## 2. Show the list of the scenario runner / シナリオランナーのリストを表示

Parameters / パラメータ:
  --list                List all supported scenarios and exit / サポートされているすべてのシナリオをリストして終了
  --output              Provide results on stdout / 結果を標準出力に提供
  --reloadWorld         Reload the CARLA world before starting a scenario (default=True) / シナリオ開始前にCARLAワールドをリロード（デフォルト=True）
  --randomize           Scenario parameters are randomized / シナリオパラメータをランダム化
  --repetitions REPETITIONS
                        Number of scenario executions / シナリオ実行回数

    ./scenario_runner.py --list

    ChangeLane_2
StationaryObjectCrossing_1
DynamicObjectCrossing_1
StationaryObjectCrossing_2
DynamicObjectCrossing_2
StationaryObjectCrossing_3
DynamicObjectCrossing_3
StationaryObjectCrossing_4
DynamicObjectCrossing_4
StationaryObjectCrossing_5
DynamicObjectCrossing_5
StationaryObjectCrossing_6
DynamicObjectCrossing_6
StationaryObjectCrossing_7
DynamicObjectCrossing_7
StationaryObjectCrossing_8
DynamicObjectCrossing_8
DynamicObjectCrossing_9
ConstructionSetupCrossing
ManeuverOppositeDirection_1
ManeuverOppositeDirection_2
ManeuverOppositeDirection_3
ManeuverOppositeDirection_4
SignalizedJunctionLeftTurn_1
SignalizedJunctionLeftTurn_2
SignalizedJunctionLeftTurn_3
SignalizedJunctionLeftTurn_4
SignalizedJunctionLeftTurn_5
SignalizedJunctionLeftTurn_6
CutInFrom_left_Lane
CutInFrom_right_Lane
SignalizedJunctionRightTurn_1
SignalizedJunctionRightTurn_2
SignalizedJunctionRightTurn_3
SignalizedJunctionRigh ros2 run rviz2 rviz2 -d /home/kotei/huqianqian/software/Carla_0.9.15/CARLA_0.9.15/wk_ros_bridge/src/carla_ad_demo/config/carla_ad_demo_ros2.rviz tTurn_4
SignalizedJunctionRightTurn_5
SignalizedJunctionRightTurn_6
SignalizedJunctionRightTurn_7
OtherLeadingVehicle_1
OtherLeadingVehicle_2
OtherLeadingVehicle_3
OtherLeadingVehicle_4
OtherLeadingVehicle_5
OtherLeadingVehicle_6
OtherLeadingVehicle_7
OtherLeadingVehicle_8
OtherLeadingVehicle_9
OtherLeadingVehicle_10
VehicleTurningRight_1

## 3. You can choose one of the scenario to test / テストするシナリオを選択

    ./scenario_runner.py --reloadWorld --scenario FollowLeadingVehicle_1 --repetitions 3 --output

    ./scenario_runner.py --reloadWorld --scenario VehicleTurningRight_1 --repetitions 3 --output

## 4. Add can ego car into the test scenario / テストシナリオにエゴカーを追加
    ./manual_control.py

## 5. Start the autopilot mode / オートパイロットモードを開始
The ego car can be autonomous driving.  
エゴカーは自動運転が可能になります。
    p


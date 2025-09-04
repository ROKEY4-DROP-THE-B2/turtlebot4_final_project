# turtlebot4_final_project
turtlebot4를 활용한 최종 프로젝트

## 노드 실행 순서
1. ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot2 map:=$HOME/rokey_ws/maps/first_map.yaml params_file:=$HOME/rokey_ws/configs/local2.yaml
2. ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot2
3. 1번 노드에서 set initial pose 메시지 출력 시 rviz에서 [2D Pose Estimate]를 클릭하여 로봇의 초기 위치, 방향(각도)를 정확히 셋팅한다
4. ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot2 params_file:=$HOME/rokey_ws/configs/nav2_net2.yaml
    1. ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot2 params_file:=$HOME/rokey_ws/configs/nav2_net3.yaml
    
    → costmap(맵에서 보라색, 분홍색, 핑크색 테두리)이 제대로 출력되는지 확인한다.
    
    ![스크린샷 2025-08-28 14-27-09.png](attachment:01dfa173-8be9-4e4d-83ac-32ef1bfb1b8c:스크린샷_2025-08-28_14-27-09.png)
    
5. ping 192.168.0.49 (robot 1은 ping 192.168.101.3)
    
    → 4번 노드 실행 동안 네트워크 상태를 확인한다.
    
    ![스크린샷 2025-08-28 14-30-55.png](attachment:fa8a2fd9-9ff3-4a62-a938-d0b346e6d95b:스크린샷_2025-08-28_14-30-55.png)
    
6. (6, 7중 선택) packbot robot2:
    1. ros2 run rokey_pjt tf --ros-args -r __ns:=/robot2 -r /tf:=/robot2/tf -r /tf_static:=/robot2/tf_static
        1. ros2 run rokey_pjt tf2 --ros-args -r __ns:=/robot2 -r /tf:=/robot2/tf -r /tf_static:=/robot2/tf_static
    2. ros2 run rokey_pjt yolo
    3. ros2 run rokey_pjt packbot --ros-args -r __ns:=/robot2
    4. ros2 run rokey_pjt packbot_cc
    5. ros2 topic pub -r 1 /robot1/keepout_costmap_filter_info nav2_msgs/msg/CostmapFilterInfo "{type: 0, filter_mask_topic: '/robot1/explosive_keepout_mask', base: 0.0, multiplier: 1.0}"
7. (6,7중 선택) supplybot robot1:
    1. ros2 run rokey_pjt supplybot --ros-args -r __ns:=/robot1
    2. ros2 run rokey_pjt supplybot_cc
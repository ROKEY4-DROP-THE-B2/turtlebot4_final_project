# turtlebot4_final_project
turtlebot4를 활용한 최종 프로젝트

## 적군 위치 마커 띄우기 실행 방법
1. ros2 launch turtlebot4_navigation localization.launch.py   namespace:=/robot2   map:=$HOME/rokey_ws/maps/first_map.yaml   params_file:=$HOME/rokey_ws/configs/local2.yaml
2. ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot2
3. ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot2
4. ros2 launch turtlebot4_navigation slam.launch.py namespace:=/robot2
5. ros2 run rokey_pjt tf --ros-args -r __ns:=/robot2 -r /tf:=/robot2/tf -r /tf_static:=/robot2/tf_static
6. ros2 run rokey_pjt yolo

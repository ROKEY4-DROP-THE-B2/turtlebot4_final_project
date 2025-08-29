# #혹시 필요하다면 터미널에서 설치
# # python3 -m pip install --user nav2-simple-commander PyYAML

# # tf_point_transform.py
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int16
# #from paho.mqtt import client as mqtt_client
# from geometry_msgs.msg import Twist
# from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
# WAYPOINT_FILE = "/home/rokey/rokey_ws/src/rokey_pjt/configs/waypoints.yaml"
# class MoveSubscriber(Node):
#     def __init__(self):
#         # super().__init__('move_subscriber')
#         # self.create_subscription(
#         #     Int16, '/robot2/packbot', self.moving, 10  
#         # )
        
#         # 좌표값 저장할 dictinary
#         self.locations = {}
#         # 셋팅 예시 INITIAL_POSE_POSITION = [0.01, 0.01]
#         #  Goal.poses=[([0.0,0.0],TurtleBot4Directions.NORTH)]
#         # .yaml 파일에서 좌표 불러오기
    
#     def moving(self, _num):
#         num = _num.data
#         self.get_logger().info(f'num={num}')
#         if num == 1:
#             # 1번 동작 수행
#             pass
#         elif num == 2:
#             # 2번 동작 수행
#             pass
#         elif num == 3:
#             # 3번 동작 수행
#             pass

# def main():
#     rclpy.init()
#     node = MoveSubscriber()

#     try:
#         while rclpy.ok():
#             rclpy.spin_once(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



#gemini

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
import yaml # YAML 파일을 다루기 위한 라이브러리
from ament_index_python.packages import get_package_share_directory # 패키지 경로를 찾기 위한 함수
import os

class MoveSubscriber(Node):
    def __init__(self):
        super().__init__('move_subscriber')

        # .yaml 파일에서 좌표를 불러와 self.locations에 저장
        self.locations = self._load_locations()
        
        if not self.locations:
            self.get_logger().error("좌표 파일을 불러오는 데 실패했습니다. 노드를 종료합니다.")
            # rclpy.shutdown() # 즉시 종료가 필요할 경우
            return

        self.create_subscription(
            Int16, '/robot1/packbot', self.moving, 10
        )
        self.get_logger().info('MoveSubscriber 노드가 준비되었습니다. 명령을 기다립니다.')

    def _load_locations(self):
        """YAML 파일에서 목표 위치들을 불러옵니다."""
        try:
            # rokey_pjt 패키지의 share 디렉토리 경로를 찾습니다.
            package_share_directory = get_package_share_directory('rokey_pjt')
            # 설정 파일의 전체 경로를 조합합니다.
            file_path = os.path.join(package_share_directory, 'config', 'waypoints.yaml')
            
            self.get_logger().info(f"좌표 파일 로딩 시도: {file_path}")
            
            with open(file_path, 'r') as file:
                config = yaml.safe_load(file)
                # 'locations' 키 아래의 딕셔너리를 반환합니다.
                return config.get('locations', {})
        except Exception as e:
            self.get_logger().error(f"좌표 파일 로딩 중 에러 발생: {e}")
            return None

    def moving(self, msg):
        """수신된 번호에 해당하는 좌표로 이동하는 동작을 수행합니다."""
        num = str(msg.data) # YAML의 키가 문자열이므로 수신 데이터도 문자열로 변환
        self.get_logger().info(f"명령 수신: '{num}'번 위치로 이동")

        # self.locations 딕셔너리에서 해당 번호의 좌표 정보 가져오기
        target_location = self.locations.get(num)

        if target_location:
            # 좌표 정보가 존재할 경우
            name = target_location.get('name', '이름 없음')
            position = target_location.get('position', {})
            orientation = target_location.get('orientation', {})
            
            self.get_logger().info(f" >> 목표 지점: {name}")
            self.get_logger().info(f" >> 좌표: Position(x={position.get('x')}, y={position.get('y')}), Orientation(w={orientation.get('w')})")

            # --- 여기에 실제 로봇을 움직이는 코드를 추가합니다 ---
            # 예: Nav2 스택에 목표 지점(GoalPose)을 발행(publish)하는 로직
            # goal_msg = PoseStamped()
            # goal_msg.header.stamp = self.get_clock().now().to_msg()
            # goal_msg.header.frame_id = 'map'
            # goal_msg.pose.position.x = position.get('x', 0.0)
            # ... (메시지 채우기)
            # self.goal_publisher.publish(goal_msg)
            # ----------------------------------------------------
            pass

        else:
            # 해당 번호의 좌표 정보가 YAML 파일에 없을 경우
            self.get_logger().warn(f"'{num}'번에 해당하는 좌표 정보가 'move_points.yaml' 파일에 정의되지 않았습니다.")

def main(args=None):
    rclpy.init(args=args)
    node = MoveSubscriber()
    # 노드 초기화 실패 시 바로 종료
    if node.locations is not None:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# tf_point_transform.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler
import time
from .mqtt_controller import MqttController

NUM_OF_WAYPOINTS = 4

def create_pose(x, y, yaw_deg, navigator: BasicNavigator) -> PoseStamped:
    """x, y, yaw(도 단위) → PoseStamped 생성"""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y

    yaw_rad = yaw_deg * 3.141592 / 180.0
    q = quaternion_from_euler(0, 0, yaw_rad)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose

class Packbot(Node):
    def __init__(self):
        super().__init__('packbot')
        self.create_subscription(
            Int16, '/robot2/packbot', self.moving, 10  
        )

        # 도킹 및 경로 이동용 Navigator
        self.dock_navigator = TurtleBot4Navigator()
        self.nav_navigator = BasicNavigator()

        # 1. 초기 위치 설정
        initial_pose = create_pose(4.767, 2.581, 0.0, self.nav_navigator)
        self.nav_navigator.setInitialPose(initial_pose)
        self.nav_navigator.get_logger().info(f'초기 위치 설정 중...')
        time.sleep(1.0) #AMCL이 초기 pose 처리 시 필요한 시간과 TF를 얻을 수 있게 됨

        self.nav_navigator.waitUntilNav2Active()

        # 2. 도킹 상태면 언도킹
        if self.dock_navigator.getDockedStatus():
            self.dock_navigator.get_logger().info('도킹 상태 → 언도킹')
            self.dock_navigator.undock()

        # 3. 개별 Pose 생성 (경유지 명시)
        self.waypoints = [
            create_pose(3.266, 2.034, 90.0, self.nav_navigator),
            create_pose(2.32, 0.39, 180.0, self.nav_navigator),
            create_pose(0.46, 0.46, -90.0, self.nav_navigator),
            create_pose(0.225, 3.04, 90.0, self.nav_navigator),
            # create_pose(3.266, 2.034, 0.0, self.nav_navigator),
        ]
        # TODO: .yaml 파일 만들어서 불러오기

        self.current_index = -1
        self.is_moving = False
        self.supplybot_current_index = -1
        
        # 좌표값 저장할 dictinary
        self.locations = {}
        # 셋팅 예시 INITIAL_POSE_POSITION = [0.01, 0.01]
        #  Goal.poses=[([0.0,0.0],TurtleBot4Directions.NORTH)]
        # .yaml 파일에서 좌표 불러오기

        def on_message(client, userdata, msg):
            topic = msg.topic
            data = msg.payload.decode()
            self.get_logger().info(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
            if topic == '/robot2/go_next_waypoint':
                self.supplybot_current_index = int(data)
        self.mqttController = MqttController(on_message)
    
    def moving(self, _num):
        num = _num.data

        if self.is_moving:
            return
        self.is_moving = True

        if num == 1:
            # 1번 동작 수행
            pass
        elif num == 2:
            # 2번 동작 수행
            pass
        elif num == 3:
            # 3번 동작 수행
            pass
        elif num == 4:
            # 장애물 없을 때 정상 주행
            # 4. Waypoint 경로 이동 시작
            self.current_index = -1
            self.supplybot_current_index = -1

            while self.current_index < NUM_OF_WAYPOINTS:
                nav_start = self.nav_navigator.get_clock().now()
                self.nav_navigator.followWaypoints(self.waypoints)

                # 5. 이동 중 피드백 확인
                while not self.nav_navigator.isTaskComplete():
                    feedback = self.nav_navigator.getFeedback()
                    if feedback:
                        elapsed = self.nav_navigator.get_clock().now() - nav_start
                        self.get_logger().info(
                            f'현재 waypoint: {feedback.current_waypoint + 1}/{NUM_OF_WAYPOINTS}, '
                            f'경과 시간: {elapsed.nanoseconds / 1e9:.1f}초'
                        )

                # 6. 도달한 waypoint 인덱스 확인
                result = self.nav_navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(f'도착 완료')
                    return
                
                self.current_index = result
                self.get_logger().info(f'Waypoint {self.current_index} 까지 도달 완료')
                self.get_logger().info(f'Waypoint supply {self.supplybot_current_index} 까지 도달 완료')

                # 보급로봇에게 도착메시지 전송 후 이전 단계의 waypoint에 도착할 떼 까지 대기
                self.mqttController.publish('/robot1/go_next_waypoint', self.current_index)
                while self.current_index != self.supplybot_current_index:
                    time.sleep(0.1)
        else:
            self.get_logger().info(f'도착 완료')
        
        if 1 <= num <= 4:
            self.is_moving = False

    
    def docking(self):
        # 7. 자동 도킹 요청
        self.get_logger().info('도킹 요청 완료')
        self.dock_navigator.dock()
        # 8. 종료 처리
        self.dock_navigator.destroy_node()
        self.nav_navigator.destroy_node()


def main():
    rclpy.init()
    node = Packbot()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.docking()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

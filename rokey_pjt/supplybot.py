import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from rclpy.executors import MultiThreadedExecutor
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler
import time, threading
from .mqtt_controller import MqttController

NUM_OF_WAYPOINTS = 5
MY_NAMESPACE = '/robot1'
OTHER_NAMESPACE = '/robot2'

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

class Supplybot(Node):
    def __init__(self):
        super().__init__('supplybot')
        # Publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, f'{MY_NAMESPACE}/cmd_vel', 10)
        # Subscribe
        self.create_subscription(
            Int16, f'{MY_NAMESPACE}/move', self.moving, 10  
        )

        # 도킹 및 경로 이동용 Navigator
        self.dock_navigator = TurtleBot4Navigator()
        self.nav_navigator = BasicNavigator()

        # 1. 초기 위치 설정
        initial_pose = create_pose(4.197, 0.8521, 0.0, self.nav_navigator)
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
            [
                create_pose(4.638, 1.465, 0.0, self.nav_navigator),
                create_pose(0.414, 3.101, 90.0, self.nav_navigator),
            ],
            [
                create_pose(4.638, 1.465, 0.0, self.nav_navigator),
                create_pose(3.266, 2.034, 90.0, self.nav_navigator),
                create_pose(2.32, 0.39, 180.0, self.nav_navigator),
                create_pose(0.46, 0.46, -90.0, self.nav_navigator),
                create_pose(0.414, 3.101, 90.0, self.nav_navigator),
            ],
        ]
        # TODO: .yaml 파일 만들어서 불러오기

        self.current_index = -1
        self.is_moving = False
        self.packbot_current_index = -1
        self.course_index = 0
        self.supplybot_current_index = lambda: self.current_index - 1
        self.is_course_changed = False
        
        def on_message(client, userdata, msg):
            data = msg.payload.decode()
            topic = msg.topic
            if topic == f'{MY_NAMESPACE}/go_next_waypoint':
                self.packbot_current_index = int(data)
                self.get_logger().info(f"변경됨 self.packbot_current_index={self.packbot_current_index}")
            elif topic == 'enemy_detected':
                self.course_index = 1
                self.is_course_changed = True
        
        topics = [
            f'{MY_NAMESPACE}/go_next_waypoint',
            'enemy_detected',
        ]
        self.mqttController = MqttController(topics, on_message)
        # MQTT 스레드 시작
        self.mqtt_thread = threading.Thread(target=self.mqttController.start_mqtt, args=(), daemon=True)
        self.mqtt_thread.start()

        self.get_logger().info(f'Initialized.')
    
    def pause(self):
        self.nav_navigator.cancelTask()
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
    
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
            self.current_index = 0
            self.packbot_current_index = 0
            self.course_index = 0
            self.is_course_changed = False

            # 정찰로봇이 2번 위치에 갈때 까지 대기
            while self.packbot_current_index == 0:
                pass

            # 좌표배열 순서대로 이동 수행 
            self.nav_navigator.followWaypoints(self.waypoints[self.course_index])

            # 5. 이동 중 피드백 확인
            while not self.nav_navigator.isTaskComplete():
                feedback = self.nav_navigator.getFeedback()
                if feedback:
                    self.get_logger().info(
                        f'현재 목적지: {self.current_index + 1}/{NUM_OF_WAYPOINTS}, '
                        f'packbot 위치: {self.packbot_current_index}/{NUM_OF_WAYPOINTS}'
                    )

                    # 적군 감지 시 코스 변경
                    if self.is_course_changed:
                        self.is_course_changed = False
                        old_packbot_current_index = self.packbot_current_index
                        self.get_logger().info('적군 감지해서 대기함')
                        self.pause()
                        while old_packbot_current_index == self.packbot_current_index or self.packbot_current_index == 0:
                            pass

                        self.get_logger().info(f'정찰로봇이 코스 변경후 {self.packbot_current_index}지점에 도착하여 다시 주행함')
                        self.current_index = self.packbot_current_index - 1
                        remaining_waypoints = self.waypoints[self.course_index][self.current_index:]
                        self.nav_navigator.followWaypoints(remaining_waypoints)
                        continue

                    # feedback.current_waypoint의 값은 현재 로직에서만 0 or 1
                    if feedback.current_waypoint == 1:
                        self.current_index += 1
                        # 정찰로봇에게 도착메시지 전송 후 이전 단계의 waypoint에 도착할 때 까지 대기
                        if self.current_index < 4:
                            self.mqttController.publish(f'{OTHER_NAMESPACE}/go_next_waypoint', self.supplybot_current_index())
                            # 대기 명령 내림
                            self.pause()
                            while self.supplybot_current_index() != self.packbot_current_index - 2 or self.is_course_changed:
                                pass
                        
                        if self.is_course_changed:
                            continue
                            
                        if self.current_index < NUM_OF_WAYPOINTS:
                            remaining_waypoints = self.waypoints[self.course_index][self.current_index:]
                            self.nav_navigator.followWaypoints(remaining_waypoints)

            # 6. 도달한 waypoint 인덱스 확인
            result = self.nav_navigator.getResult()
            self.get_logger().info(f'{result}')

        elif num == 5:
            goal_pose = create_pose(3.266, 2.034, 0.0, self.nav_navigator)
            self.nav_navigator.goToPose(goal_pose)

            # 4. 이동 중 피드백 표시
            while not self.nav_navigator.isTaskComplete():
                feedback = self.nav_navigator.getFeedback()
                if feedback:
                    remaining = feedback.distance_remaining
                    self.nav_navigator.get_logger().info(f'남은 거리: {remaining:.2f} m')

            # 5. 결과 확인
            result = self.nav_navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.nav_navigator.get_logger().info('목표 위치 도달 성공')
            elif result == TaskResult.CANCELED:
                self.nav_navigator.get_logger().warn('이동이 취소되었습니다.')
            elif result == TaskResult.FAILED:
                error_code, error_msg = self.nav_navigator.getTaskError()
                self.nav_navigator.get_logger().error(f'이동 실패: {error_code} - {error_msg}')
            else:
                self.nav_navigator.get_logger().warn('알 수 없는 결과 코드 수신')
            
            self.docking()
        else:
            self.get_logger().info(f'유효하지 않은 입력값')
        
        if 1 <= num <= 5:
            self.is_moving = False
    
    def docking(self):
        # 7. 자동 도킹 요청
        self.get_logger().info('도킹 요청 완료')
        self.dock_navigator.dock()

def main():
    rclpy.init()
    node = Supplybot()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.nav_navigator.cancelTask()
    finally:
        executor.shutdown()
        node.mqttController.stop_mqtt()
        node.dock_navigator.destroy_node()
        node.nav_navigator.destroy_node()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

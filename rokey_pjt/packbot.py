import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from rclpy.executors import MultiThreadedExecutor
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.callback_groups import ReentrantCallbackGroup
from tf_transformations import quaternion_from_euler
import time, threading, math
from .mqtt_controller import MqttController

NUM_OF_WAYPOINTS = 4
MY_NAMESPACE = '/robot2'
OTHER_NAMESPACE = '/robot1'
ENEMY_DETECTED = f'{MY_NAMESPACE}/enemy_detected'

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

def find_closest_point_index(point, array):
    min_distance = float('inf')  # 최소 거리를 무한대로 초기화
    closest_index = -1           # 가장 가까운 원소의 인덱스 초기화

    x1, y1 = point

    for index, pose in enumerate(array):
        x2, y2 = pose.pose.position.x, pose.pose.position.y

        # 유클리드 거리 계산
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

        # 현재 거리가 최소 거리보다 작으면 갱신
        if distance < min_distance:
            min_distance = distance
            closest_index = index

    return closest_index

class Packbot(Node):
    def __init__(self):
        super().__init__('packbot')
        self.reentrant_cbg = ReentrantCallbackGroup()
        # Publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, f'{MY_NAMESPACE}/cmd_vel', 10)
        # Subscribe
        self.create_subscription(
            Int16, f'{MY_NAMESPACE}/move', self.moving, 10,
            callback_group=self.reentrant_cbg
        )

        self.create_subscription(
            Vector3, ENEMY_DETECTED, self.change_waypoint, 10,
            callback_group=self.reentrant_cbg
        )

        # 도킹 및 경로 이동용 Navigator
        self.dock_navigator = TurtleBot4Navigator()
        self.nav_navigator = BasicNavigator()

        # 1. 초기 위치 설정
        initial_pose = create_pose(4.663, 2.581, 0.0, self.nav_navigator)
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
                create_pose(3.266, 2.034, 0.0, self.nav_navigator),
                create_pose(-0.0309, 3.053, 90.0, self.nav_navigator),
            ],
            [
                create_pose(3.266, 2.034, 0.0, self.nav_navigator),
                create_pose(2.32, 0.39, -90.0, self.nav_navigator),
                create_pose(0.46, 0.46, 0.0, self.nav_navigator),
                create_pose(-0.0309, 3.053, 90.0, self.nav_navigator),
            ],
        ]
        # TODO: .yaml 파일 만들어서 불러오기

        self.current_index = -1
        self.is_moving = False
        self.supplybot_current_index = -1
        self.course_index = 0
        self.is_course_changed = False

        def on_message(client, userdata, msg):
            data = msg.payload.decode()
            self.supplybot_current_index = int(data)
            self.get_logger().info(f"변경됨 self.supplybot_current_index={self.supplybot_current_index}")

        self.mqttController = MqttController(f'{MY_NAMESPACE}/go_next_waypoint', on_message)
        # MQTT 스레드 시작
        self.mqtt_thread = threading.Thread(target=self.mqttController.start_mqtt, args=(), daemon=True)
        self.mqtt_thread.start()

        self.nav_navigator.get_logger().info(f'Initialized.')
    
    def pause(self):
        self.nav_navigator.cancelTask()
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
    
    def moving(self, _num):
        num = _num.data

        if self.is_moving:
            self.dock_navigator.get_logger().info('이동중이라 명령을 무시합니다.')
            return
        self.is_moving = True
        is_error = False

        if num == 1:
            # 1번 동작 수행
            self.dock_navigator.get_logger().info('도킹 상태 → 언도킹')
            self.dock_navigator.undock()
        elif num == 2:
            # 2번 동작 수행
            pass
        elif num in (31, 32, 33, 312, 313, 323, 3123):
            # 보금품 수령
            self.nav_navigator.get_logger().info('보급품 로봇이 보급품을 수령중입니다.')
        elif num == 4:
            # 장애물 없을 때 정상 주행
            # 4. Waypoint 경로 이동 시작
            self.dock_navigator.get_logger().info('보급풉 운반을 위한 정찰 시작')
            self.current_index = 0
            self.supplybot_current_index = -1
            self.course_index = 0
            self.is_course_changed = False

            # 좌표배열 순서대로 이동 수행 
            self.nav_navigator.followWaypoints(self.waypoints[self.course_index])
            def show():
                if not self.nav_navigator.isTaskComplete():
                    self.get_logger().info(
                        f'현재 목적지: {self.current_index + 1}/{NUM_OF_WAYPOINTS}, '
                        f'supplybot 위치: {self.supplybot_current_index}/{NUM_OF_WAYPOINTS}'
                    )

            timer = self.create_timer(1.0, show)
            go_to_final = False

            # 5. 이동 중 피드백 확인
            while not self.nav_navigator.isTaskComplete():
                feedback = self.nav_navigator.getFeedback()
                if feedback:

                    # 적군 감지 시 코스 변경
                    if self.is_course_changed and not go_to_final:
                        self.nav_navigator.cancelTask()
                        go_to_final = True

                        # 최종 목적지로 바로 가도록 수정
                        remaining_waypoints = self.waypoints[self.course_index][-1:]
                        self.nav_navigator.followWaypoints(remaining_waypoints)
                        self.get_logger().info(f'최종 목적지로 경로를 변경합니다.')
                        continue

                    # feedback.current_waypoint의 값은 현재 로직에서만 0 or 1
                    if not go_to_final and feedback.current_waypoint == 1:
                        self.current_index += 1
                        # 보급로봇에게 도착메시지 전송 후 이전 단계의 waypoint에 도착할 때 까지 대기
                        self.mqttController.publish(f'{OTHER_NAMESPACE}/go_next_waypoint', self.current_index)
                        # 대기 명령 내림
                        self.pause()
                        
                        while self.current_index != self.supplybot_current_index + 1:
                            pass
                        
                        if self.current_index < NUM_OF_WAYPOINTS:
                            remaining_waypoints = self.waypoints[self.course_index][self.current_index:]
                            self.nav_navigator.followWaypoints(remaining_waypoints)

            # 6. 도달한 waypoint 인덱스 확인
            result = self.nav_navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.current_index += 1
                self.mqttController.publish(f'{OTHER_NAMESPACE}/go_next_waypoint', self.current_index)
            self.get_logger().info(f'{result}')
            timer.cancel()
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
            is_error = True
            self.get_logger().info(f'유효하지 않은 입력값')
        
        if not is_error:
            self.is_moving = False
    
    def change_waypoint(self, msg: Vector3):
        if self.is_moving and not self.is_course_changed:
            # x, y = msg.x, msg.y
            self.course_index = 1
            # self.current_index = max(0, find_closest_point_index((x, y), self.waypoints[self.course_index]) - 1)
            self.is_course_changed = True

            # self.get_logger().info(f"현재 위치 index = {self.current_index}")
            self.get_logger().info(f"적군을 감지하여 다른 경로로 탐색합니다.")
            time.sleep(2)
            self.mqttController.publish(f'enemy_detected', self.current_index)
    
    def docking(self):
        # 7. 자동 도킹 요청
        self.get_logger().info('도킹 요청 완료')
        self.dock_navigator.dock()


def main():
    rclpy.init()
    node = Packbot()
    executor = MultiThreadedExecutor()
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

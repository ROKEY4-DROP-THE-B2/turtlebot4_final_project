# dock_state_subscriber.py

import rclpy
from rclpy.node import Node
from paho.mqtt import client as mqtt_client
from geometry_msgs.msg import Twist
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

# ======================
# 초기 설정 (파일 안에서 직접 정의)
# ======================
INITIAL_POSE_POSITION = [0.01, 0.01]
INITIAL_POSE_DIRECTION = TurtleBot4Directions.NORTH

GOAL_POSES = [
    ([2.57, 0.73], TurtleBot4Directions.NORTH),
]

# --- MQTT 접속 정보 ---
BROKER = 'r1782871.ala.us-east-1.emqxsl.com'
PORT = 8883
USERNAME = 'sg'
PASSWORD = '1234'
TOPIC = "/robot2/execute"
CLIENT_ID = "robot2-action-subscriber"
# --------------------

# --- 로봇 동작 설정 ---
MOVE_SPEED = 0.2  # 로봇의 전진 속도 (m/s)
MOVE_DISTANCE = 1.0 # 목표 이동 거리 (m)
# --------------------

class DockStateSubscriber(Node):
    def __init__(self):
        super().__init__('dock_state_subscriber')
        self.mqtt_client = self.connect_mqtt()
        self.mqtt_client.loop_start()
        self.navigator = TurtleBot4Navigator()

        # if not self.navigator.getDockedStatus():
        #     self.navigator.info('Docking before initializing pose')
        #     self.navigator.dock()
        
        # initial_pose = self.navigator.getPoseStamped(INITIAL_POSE_POSITION, INITIAL_POSE_DIRECTION)
        # self.navigator.setInitialPose(initial_pose)
        # self.navigator.waitUntilNav2Active()

    def connect_mqtt(self) -> mqtt_client.Client:
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                self.get_logger().info("MQTT 브로커에 성공적으로 연결되었습니다!")
                client.subscribe(TOPIC)
                self.get_logger().info(f"'{TOPIC}' 토픽 구독을 시작합니다.")
            else:
                self.get_logger().error(f"MQTT 연결 실패, 코드: {rc}")

        def on_message(client, userdata, msg):
            received_message = msg.payload.decode()
            self.get_logger().info(f"메시지 수신 완료 <<-- [Topic: {msg.topic}] [Message: {received_message}]")
            self.get_logger().info(f'{type(received_message)}')
            self.navigator.undock()

        client = mqtt_client.Client(client_id=CLIENT_ID, protocol=mqtt_client.MQTTv311)
        client.tls_set()
        client.username_pw_set(USERNAME, PASSWORD)
        client.on_connect = on_connect
        client.on_message = on_message
        client.connect(BROKER, PORT)
        return client

    def destroy_node(self):
        self.mqtt_client.loop_stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    subscriber_node = DockStateSubscriber()
    while rclpy.ok():
        try:
            rclpy.spin(subscriber_node)
        except KeyboardInterrupt:
            pass
        finally:
            subscriber_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
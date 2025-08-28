# dock_state_subscriber.py

import rclpy
from rclpy.node import Node
from paho.mqtt import client as mqtt_client
from geometry_msgs.msg import Twist

# --- MQTT 접속 정보 ---
BROKER = 'r1782871.ala.us-east-1.emqxsl.com'
PORT = 8883
USERNAME = 'sg'
PASSWORD = '1234'
TOPIC = "turtlebot/robot1/status"
CLIENT_ID = "robot2-action-subscriber"
# --------------------

# --- 로봇 동작 설정 ---
MOVE_SPEED = 0.2  # 로봇의 전진 속도 (m/s)
MOVE_DISTANCE = 1.0 # 목표 이동 거리 (m)
# --------------------

class DockStateSubscriber(Node):
    """
    MQTT 토픽을 구독하여, 'docked' 메시지 수신 시 
    지정된 거리만큼 로봇을 전진시킨 후 정지시키는 노드.
    """
    def __init__(self):
        super().__init__('dock_state_subscriber')

        self.cmd_vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.mqtt_client = self.connect_mqtt()
        self.mqtt_client.loop_start()

        self.move_timer = None  # 타이머 객체를 저장할 변수
        self.is_moving = False  # 현재 로봇이 이동 명령을 수행 중인지 여부를 나타내는 플래그

        self.get_logger().info("상태 구독 및 동작 노드 시작.")
        self.get_logger().info(f"'{TOPIC}' 수신 시 {MOVE_DISTANCE}m 전진 후 정지합니다.")

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

            # 메시지가 'docked'이고, 현재 움직이는 중이 아닐 때만 동작 실행
            if received_message == "undocked" and not self.is_moving:
                self.start_moving_forward()

        client = mqtt_client.Client(client_id=CLIENT_ID, protocol=mqtt_client.MQTTv311)
        client.tls_set()
        client.username_pw_set(USERNAME, PASSWORD)
        client.on_connect = on_connect
        client.on_message = on_message
        client.connect(BROKER, PORT)
        return client
        
    def start_moving_forward(self):
        """로봇 전진을 시작하고, 정지를 위한 타이머를 설정합니다."""
        self.is_moving = True
        self.get_logger().info(f"{MOVE_SPEED} m/s 속도로 전진을 시작합니다.")

        # 1. 전진 명령 발행
        twist_msg = Twist()
        twist_msg.linear.x = MOVE_SPEED
        self.cmd_vel_publisher_.publish(twist_msg)

        # 2. 정지 타이머 설정
        # 이동에 필요한 시간 계산 (시간 = 거리 / 속도)
        duration = MOVE_DISTANCE / MOVE_SPEED
        self.get_logger().info(f"{duration:.1f}초 후에 정지할 예정입니다.")
        
        # 기존에 타이머가 있다면 취소 (안정성 확보)
        if self.move_timer is not None:
            self.move_timer.cancel()
            
        self.move_timer = self.create_timer(duration, self.stop_robot)

    def stop_robot(self):
        """로봇을 정지시키고 타이머를 제거합니다."""
        self.get_logger().info("목표 도달. 로봇을 정지합니다.")
        
        # 1. 정지 명령 발행
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        self.cmd_vel_publisher_.publish(twist_msg)

        # 2. 타이머 취소 및 상태 초기화
        if self.move_timer is not None:
            self.move_timer.cancel()
            self.move_timer = None
        
        self.is_moving = False

    def destroy_node(self):
        self.mqtt_client.loop_stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    subscriber_node = DockStateSubscriber()
    rclpy.spin(subscriber_node)
    subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# dock_state_subscriber.py

import rclpy
from rclpy.node import Node
from paho.mqtt import client as mqtt_client

# --- MQTT 접속 정보 (발행자와 동일해야 함) ---
BROKER = 'r1782871.ala.us-east-1.emqxsl.com'
PORT = 8883
USERNAME = 'sg'
PASSWORD = '1234'
TOPIC = "turtlebot/robot1/status"  # 구독할 토픽 (robot1의 상태 토픽)
CLIENT_ID = "robot2-status-subscriber"
# ---------------------------------------------

class DockStateSubscriber(Node):
    """
    MQTT 토픽을 구독하여 수신된 메시지를 터미널에 출력하는 노드.
    """
    def __init__(self):
        super().__init__('dock_state_subscriber')

        self.mqtt_client = self.connect_mqtt()
        self.mqtt_client.loop_start()  # MQTT 비동기 루프 시작

        self.get_logger().info("상태 구독 및 출력 노드 시작.")

    def connect_mqtt(self) -> mqtt_client.Client:
        """MQTT 브로커에 연결하고 토픽을 구독합니다."""
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                self.get_logger().info("MQTT 브로커에 성공적으로 연결되었습니다!")
                # 연결 성공 시 토픽 구독 실행
                client.subscribe(TOPIC)
                self.get_logger().info(f"'{TOPIC}' 토픽 구독을 시작합니다.")
            else:
                self.get_logger().error(f"MQTT 연결 실패, 코드: {rc}")

        def on_message(client, userdata, msg):
            """MQTT 메시지 수신 시 호출될 콜백 함수."""
            # 수신된 메시지를 decode하여 터미널에 출력
            received_message = msg.payload.decode()
            self.get_logger().info(f"메시지 수신 완료 <<-- [Topic: {msg.topic}] [Message: {received_message}]")

        client = mqtt_client.Client(client_id=CLIENT_ID, protocol=mqtt_client.MQTTv311)
        client.tls_set()
        client.username_pw_set(USERNAME, PASSWORD)
        client.on_connect = on_connect
        client.on_message = on_message  # 메시지 수신 콜백 함수 등록
        client.connect(BROKER, PORT)
        return client
        
    def destroy_node(self):
        """노드 종료 시 MQTT 루프를 정지합니다."""
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
# ros_to_mqtt_bridge.py

import rclpy
from rclpy.node import Node
from paho.mqtt import client as mqtt_client
# DockStatus 메시지 타입을 사용하기 위해 import 합니다.
from irobot_create_msgs.msg import DockStatus 

# --- MQTT 접속 정보 ---
BROKER = 'r1782871.ala.us-east-1.emqxsl.com'
PORT = 8883
USERNAME = 'sg'
PASSWORD = '1234'
TOPIC = "turtlebot/robot1/status"  # robot1의 상태를 발행할 MQTT 토픽
CLIENT_ID = "robot1-status-publisher"
# --------------------------

class RosToMqttBridge(Node):
    """
    ROS 2의 /dock_status 토픽을 구독하여, 그 상태를 MQTT로 발행하는 브릿지 노드.
    """
    def __init__(self):
        # 노드 이름을 'ros_to_mqtt_bridge'로 초기화
        super().__init__('ros_to_mqtt_bridge')
        
        # 1. MQTT 클라이언트 설정 및 연결
        self.mqtt_client = self.connect_mqtt()
        self.mqtt_client.loop_start()  # MQTT 비동기 루프 시작

        # 2. ROS 2 구독자 설정
        # 'dock_status'라는 상대 토픽 이름을 사용합니다.
        # 노드를 /robot1 네임스페이스로 실행하면, 자동으로 /robot1/dock_status를 구독하게 됩니다.
        self.subscription = self.create_subscription(
            DockStatus,
            'dock_status',
            self.dock_status_callback,
            10)
        
        self.get_logger().info("ROS to MQTT 브릿지 노드 시작.")
        self.get_logger().info(f"ROS 토픽 'dock_status' 구독 중...")
        self.get_logger().info(f"MQTT 토픽 '{TOPIC}'으로 발행 준비 완료.")
        
        # 마지막으로 보낸 MQTT 메시지를 저장하여, 상태가 변경될 때만 보내도록 함
        self.last_published_state = None

    def connect_mqtt(self) -> mqtt_client.Client:
        """MQTT 브릿지 연결 로직 (이전과 동일)"""
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                self.get_logger().info("MQTT 브로커에 성공적으로 연결되었습니다!")
            else:
                self.get_logger().error(f"MQTT 연결 실패, 코드: {rc}")

        client = mqtt_client.Client(client_id=CLIENT_ID, protocol=mqtt_client.MQTTv311)
        client.tls_set()
        client.username_pw_set(USERNAME, PASSWORD)
        client.on_connect = on_connect
        client.connect(BROKER, PORT)
        return client

    def dock_status_callback(self, msg):
        """
        /dock_status 토픽에서 메시지를 수신했을 때 호출되는 콜백 함수.
        """
        # is_docked 필드 값에 따라 현재 상태 결정
        current_state = "docked" if msg.is_docked else "undocked"
        
        # 상태가 변경되었을 때만 MQTT 메시지를 발행 (네트워크 부하 감소)
        if current_state != self.last_published_state:
            result = self.mqtt_client.publish(TOPIC, current_state)
            
            if result[0] == 0:
                self.get_logger().info(f'ROS 상태 변경 감지 -> MQTT 발행: "{current_state}"')
                self.last_published_state = current_state
            else:
                self.get_logger().warn(f'MQTT 발행 실패.')

    def destroy_node(self):
        """노드 종료 시 MQTT 루프를 정지합니다."""
        self.mqtt_client.loop_stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    bridge_node = RosToMqttBridge()
    rclpy.spin(bridge_node)
    bridge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
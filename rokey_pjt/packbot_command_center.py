# tf_point_transform.py
import rclpy, sys, threading
from rclpy.node import Node
from std_msgs.msg import Int16
from .mqtt_controller import MqttController
from rclpy.executors import MultiThreadedExecutor

MY_NAMESPACE = '/robot2'
OTHER_NAMESPACE = '/robot1'

COMMAND = {
    1: '언도킹',
    2: 'UNKNOWN',
    3: '보급품 수령',
    4: '보급품 운반',
    5: '도킹',
}

class PackbotCommandCenter(Node):
    def __init__(self):
        super().__init__('packbot_command_center')
        self._publishers = self.create_publisher(
            Int16, f'{MY_NAMESPACE}/move', 10  
        )
        
        def on_message(client, userdata, msg):
            # packbot쪽에서는 mqtt 받을 일 없음
            pass

        self.mqttController = MqttController(f'{MY_NAMESPACE}/move', on_message)
        # MQTT 스레드 시작
        self.mqtt_thread = threading.Thread(target=self.mqttController.start_mqtt, args=(), daemon=True)
        self.mqtt_thread.start()
    
    def publish(self, n):
        msg = Int16()
        msg.data = int(n)
        self._publishers.publish(msg)
    
    def publish_to_mqtt(self, msg):
        self.mqttController.publish(f'{OTHER_NAMESPACE}/move', msg)


def main(args=None):
    rclpy.init(args=args)
    node = PackbotCommandCenter()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok():
            for k, v in COMMAND.items():
                node.get_logger().info(f"{k}: {v}")
            n = int(input('명령을 입력하세요: '))
            if n == 3:
                node.get_logger().info('요청할 보급품을 입력하세요.(1: 피복, 2: 탄약, 3: 식량): ')
                node.get_logger().info('2개 이상을 요청할 경우 ,로 구분해 주세요(ex: 1,3)')
                menu = ''.join(input().split(','))
                n = int(str(n) + menu)
                node.get_logger().info('보급품을 요청하였습니다.')
            node.publish(n)
            node.publish_to_mqtt(n)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqttController.stop_mqtt()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

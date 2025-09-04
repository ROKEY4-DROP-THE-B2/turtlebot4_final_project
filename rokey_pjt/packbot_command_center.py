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
    31: '보급품 수령',
    32: '보급품 수령',
    33: '보급품 수령',
    312: '보급품 수령', 
    313: '보급품 수령',
    323: '보급품 수령',
    3123: '보급품 수령',
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
            data = msg.payload.decode()
            self.publish(data)
            self.publish_to_mqtt(data)

        self.mqttController = MqttController(f'{MY_NAMESPACE}/move', on_message)
        # MQTT 스레드 시작
        self.mqtt_thread = threading.Thread(target=self.mqttController.start_mqtt, args=(), daemon=True)
        self.mqtt_thread.start()
    
    def publish(self, n):
        msg = Int16()
        msg.data = int(n)
        self.get_logger().info(f'data={msg.data}')
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
            pass
    except KeyboardInterrupt:
        pass
    finally:
        node.mqttController.stop_mqtt()
        executor.shutdown()
        rclpy.shutdown()

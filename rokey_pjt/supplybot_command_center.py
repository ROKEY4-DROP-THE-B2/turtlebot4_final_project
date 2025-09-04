# tf_point_transform.py
import rclpy, sys, threading
from rclpy.node import Node
from std_msgs.msg import Int16
from .mqtt_controller import MqttController
from rclpy.executors import MultiThreadedExecutor

NAMESPACE = '/robot1'

class SupplybotCommandCenter(Node):
    def __init__(self):
        super().__init__('supplybot_command_center')

        self._publishers = self.create_publisher(
            Int16, f'{NAMESPACE}/move', 10  
        )

        def on_message(client, userdata, msg):
            data = msg.payload.decode()
            self.publish(data)

        self.mqttController = MqttController(f'{NAMESPACE}/move', on_message)
        # MQTT 스레드 시작
        self.mqtt_thread = threading.Thread(target=self.mqttController.start_mqtt, args=(), daemon=True)
        self.mqtt_thread.start()
    
    def publish(self, n):
        msg = Int16()
        msg.data = int(n)
        self._publishers.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SupplybotCommandCenter()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.mqttController.stop_mqtt()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

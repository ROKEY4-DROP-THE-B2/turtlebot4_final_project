# tf_point_transform.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from rclpy.executors import MultiThreadedExecutor
from mqtt_controller import MqttController
import time, threading

NUM_OF_WAYPOINTS = 4

class Supplybot(Node):
    def __init__(self):
        super().__init__('supplybot')
        self.create_subscription(
            Int16, '/robot2/packbot', self.moving, 10  
        )

        self.current_index = -1
        self.packbot_current_index = -1

        def on_message(client, userdata, msg):
            topic = msg.topic
            data = msg.payload.decode()
            print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
            if topic == '/robot1/go_next_waypoint':
                self.packbot_current_index = int(data)
        self.mqttController = MqttController('/robot1/go_next_waypoint', on_message)
        # MQTT 스레드 시작
        self.mqtt_thread = threading.Thread(target=self.mqttController.start_mqtt, args=(), daemon=True)
        self.mqtt_thread.start()

        self.get_logger().info(f'init_finished')
    
    def moving(self, _num):
        num = _num.data

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
            self.packbot_current_index = 0

            while self.current_index < NUM_OF_WAYPOINTS:
                while self.current_index + 2 != self.packbot_current_index:
                    pass
                self.get_logger().info(f'Waypoint {self.current_index} 주행시작')
                # 5초 후에 도착하는 것으로 가정
                time.sleep(5)
                self.current_index += 1
                self.get_logger().info(f'Waypoint {self.current_index} 까지 도달 완료')

                # 보급로봇에게 도착메시지 전송 후 이전 단계의 waypoint에 도착할 떼 까지 대기
                self.mqttController.publish('/robot2/go_next_waypoint', self.current_index)
        else:
            self.get_logger().info(f'도착 완료')

def main():
    rclpy.init()
    node = Supplybot()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

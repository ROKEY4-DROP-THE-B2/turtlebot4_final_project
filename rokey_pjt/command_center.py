# tf_point_transform.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

class CommandCenter(Node):
    def __init__(self):
        super().__init__('command_center')
        self._publishers = self.create_publisher(
            Int16, '/robot2/packbot', 10  
        )
    
    def publish(self, n):
        msg = Int16()
        msg.data = n
        print(n)
        self._publishers.publish(msg)

def main():
    rclpy.init()
    node = CommandCenter()

    try:
        while rclpy.ok():
            n = int(input())
            node.publish(n)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

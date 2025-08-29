# tf_point_transform.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

class MoveSubscriber(Node):
    def __init__(self):
        super().__init__('move_subscriber')
        self.create_subscription(
            Int16, '/robot2/packbot', self.moving, 10  
        )
        
        # 좌표값 저장할 dictinary
        self.locations = {}
        # .yaml 파일에서 좌표 불러오기
    
    def moving(self, _num):
        num = _num.data
        self.get_logger().info(f'num={num}')
        if num == 1:
            # 1번 동작 수행
            pass
        elif num == 2:
            # 2번 동작 수행
            pass
        elif num == 3:
            # 3번 동작 수행
            pass

def main():
    rclpy.init()
    node = MoveSubscriber()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

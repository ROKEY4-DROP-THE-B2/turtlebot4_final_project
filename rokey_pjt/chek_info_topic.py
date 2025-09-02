import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav2_msgs.msg import CostmapFilterInfo

class KeepoutInfoRelay(Node):
    def __init__(self):
        super().__init__('keepout_info_relay')

        qos_in  = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_out = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)

        # 원본 info를 여기에 받는다고 가정 (서버가 내보내는 이름을 *_in 으로)
        self.sub = self.create_subscription(CostmapFilterInfo,
                                            'keepout_costmap_filter_info_in',
                                            self.cb, qos_in)
        # TL로 다시 발행 (플러그인이 구독할 공식 이름)
        self.pub = self.create_publisher(CostmapFilterInfo,
                                         'keepout_costmap_filter_info',
                                         qos_out)
        self.last = None
        # 하트비트 재발행(구독자 늦게 붙어도 보장)
        self.timer = self.create_timer(2.0, self.heartbeat)

    def cb(self, msg):
        self.last = msg
        self.pub.publish(msg)

    def heartbeat(self):
        if self.last is not None:
            self.pub.publish(self.last)

def main():
    rclpy.init()
    rclpy.spin(KeepoutInfoRelay())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
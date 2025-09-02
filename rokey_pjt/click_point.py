import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
import math
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
class ClickedPointToKeepout(Node):
    def __init__(self):
        super().__init__('clicked_point_to_keepout')

        # /clicked_point (RViz Publish Point tool)
        self.sub = self.create_subscription(
            PointStamped, 'clicked_point', self.cb_point, 10)

        # /explosive_keepout_mask (Nav2 KeepoutFilter용)
        # self.pub = self.create_publisher(OccupancyGrid, 'explosive_keepout_mask', 10)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability  = DurabilityPolicy.TRANSIENT_LOCAL

        self.pub = self.create_publisher(
            OccupancyGrid,
            '/robot1/explosive_keepout_mask', 
            qos
        )
        # 기본 맵 메타데이터 (로컬 윈도우 크기와 맞춰줌)
        self.resolution = 0.05  # 0.05cm 현재는 임시로 글로벌로 맞춤
        self.width  = 112
        self.height = 80
        self.origin_x = -0.486
        self.origin_y = -0.458
        self.get_logger().info("Clicked point → KeepoutMask 변환 노드 시작됨. RViz에서 점을 찍어보세요.")

    def cb_point(self, msg: PointStamped):
        self.get_logger().info(f"Clicked at: ({msg.point.x:.2f}, {msg.point.y:.2f})")

        # OccupancyGrid 준비
        grid = OccupancyGrid()
        current_time = self.get_clock().now()
        grid.header.stamp = current_time.to_msg()
        grid.header.frame_id = "map"   # costmap frame과 맞춰야 함
        # grid.header.stamp = self.get_clock().now().to_msg()
        info = MapMetaData()
        info.resolution = self.resolution
        info.width = self.width
        info.height = self.height
        info.origin.position.x = self.origin_x
        info.origin.position.y = self.origin_y
        info.origin.orientation.w = 1.0 
        grid.info = info

        # 초기값 0 (모두 free)
        data = np.zeros((self.height, self.width), dtype=np.int8)

        # 클릭 좌표 → grid index 변환
        gx = int((msg.point.x - self.origin_x) / self.resolution)
        gy = int((msg.point.y - self.origin_y) / self.resolution)

        radius = int(0.5/self.resolution)  # 반경 0.5m keepout
        for y in range(max(0, gy - radius), min(self.height, gy + radius)):
            for x in range(max(0, gx - radius), min(self.width, gx + radius)):
                dx, dy = x - gx, y - gy
                if math.sqrt(dx*dx + dy*dy) <= radius:
                    data[y, x] = 100  # 100=차단

        grid.data = data.flatten().tolist()

        # 퍼블리시
        self.pub.publish(grid)
        # self.get_logger().info(self.data)
        self.get_logger().info("Keepout mask 퍼블리시 완료!")

def main(args=None):
    rclpy.init(args=args)
    node = ClickedPointToKeepout()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

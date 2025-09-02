# tf_point_transform.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import OccupancyGrid, MapMetaData
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import numpy as np

DISTANCE_INFO_TOPIC = '/robot1/distance'
MASK_TOPIC = '/robot1/explosive_keepout_mask'

class TfPointTransform(Node):
    def __init__(self):
        super().__init__('tf_point_transform')

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 파라미터
        self.declare_parameter('target_frame', 'map')
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        # 상태
        self.publish_count = 0
        self.max_publish   = 500000
        self.map_info_synced = False
        self.done_once = False
        # /map 구독 (라칭 QoS)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        # 절대 경로라면 앞에 / 필요
        self.map_sub = self.create_subscription(OccupancyGrid, '/robot1/map', self.map_cb, map_qos)

        # keepout 퍼블리셔 (라칭)
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self.mask_pub = self.create_publisher(OccupancyGrid, MASK_TOPIC, latched_qos)

        # 거리 구독(메시지 콜백은 'distance_callback'로 별도)
        

        # 초기값 (곧 /map으로 덮임)
        self.resolution = 0.05
        self.width  = 112
        self.height = 80
        self.origin_x = -0.486
        self.origin_y = -0.458

        # 주기 타이머 (인자 없는 콜백)
        self.get_logger().info("TF Tree 안정화 시작. 50초 후 변환 시작합니다.")
        self.start_timer = self.create_timer(5.0, self.start_transform)

    # ---------- 콜백들 ----------
    def map_cb(self, msg: OccupancyGrid):
        info = msg.info
        self.resolution = info.resolution
        self.width      = info.width
        self.height     = info.height
        self.origin_x   = info.origin.position.x
        self.origin_y   = info.origin.position.y
        self.map_info_synced = True
        self.get_logger().info("Keepout mask info synced to /map")
        self.get_logger().info(f"resolution: {self.resolution}")
        self.get_logger().info(f"width: {self.width}")
        self.get_logger().info(f"height: {self.height}")
        self.get_logger().info(f"origin_x: {self.origin_x}")
        self.get_logger().info(f"origin_y: {self.origin_y}")
        self.get_logger().info("Keepout mask info synced to /map")
        self.destroy_subscription(self.map_sub)

    def start_transform(self):
        self.get_logger().info("TF Tree 안정화 완료. 변환 시작합니다.")
        # 2초마다 "앞으로 1m"를 변환 테스트 (옵션)
        self.distance_subscription = self.create_subscription(
            Float32, DISTANCE_INFO_TOPIC, self.distance_callback, 10
        )
        self.start_timer.cancel()

   

    def distance_callback(self, msg: Float32):
        # 거리값(전방 x)을 base_link->target_frame으로 변환
        pt = PointStamped()
        pt.header.frame_id = 'base_link'
        pt.header.stamp = Time().to_msg()
        pt.point.x = float(msg.data)
        pt.point.y = 0.0
        pt.point.z = 0.0

        p = self._transform_point(pt, self.target_frame)
        if p is None:
            return
        self.get_logger().info(f"[{p.header.frame_id}] ({p.point.x:.2f}, {p.point.y:.2f}, {p.point.z:.2f})")
        self.publish_mask(p.point.x, p.point.y, frame_id=p.header.frame_id)

    # ---------- 유틸 ----------
    def _transform_point(self, pt: PointStamped, target_frame: str):
        try:
            if not self.tf_buffer.can_transform(target_frame, pt.header.frame_id, Time(), timeout=Duration(seconds=0.3)):
                self.get_logger().warn(f"TF not ready: {target_frame} <- {pt.header.frame_id}")
                return None
            tf = self.tf_buffer.lookup_transform(target_frame, pt.header.frame_id, Time(), timeout=Duration(seconds=0.5))
            p_out = tf2_geometry_msgs.do_transform_point(pt, tf)
            p_out.header.stamp = tf.header.stamp
            p_out.header.frame_id = target_frame
            return p_out
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
            self.get_logger().warn(f"TF lookup/connectivity error ({target_frame}): {e}")
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f"TF extrapolation error ({target_frame}): {e}")
        except Exception as e:
            self.get_logger().warn(f"Unexpected TF error ({target_frame}): {e}")
        return None

    def publish_mask(self, x_m, y_m, frame_id: str):
        # 발행 제한
        if self.done_once==False:
            
  
        # map 기준으로 그릴 때는 /map info가 준비되어 있어야 안전

            grid = OccupancyGrid()
            grid.header.stamp = self.get_clock().now().to_msg()
            grid.header.frame_id = frame_id

            info = MapMetaData()
            info.resolution = self.resolution
            info.width = self.width
            info.height = self.height
            info.origin.position.x = self.origin_x
            info.origin.position.y = self.origin_y
            info.origin.orientation.w = 1.0
            grid.info = info

            data = np.zeros((self.height, self.width), dtype=np.int8)

            cx = int((x_m - self.origin_x) / self.resolution)
            cy = int((y_m - self.origin_y) / self.resolution)
            r  = int(0.8 / self.resolution)  # 0.5 m

            self.get_logger().info(
                f"mask ROI: world=({x_m:.2f},{y_m:.2f}), grid=({cx},{cy}), size=({self.width},{self.height}), r={r}"
            )

            before = np.count_nonzero(data == 100)
            x0 = max(0, cx - r); x1 = min(self.width,  cx + r + 1)
            y0 = max(0, cy - r); y1 = min(self.height, cy + r + 1)
            for ix in range(x0, x1):
                dx2 = (ix - cx) * (ix - cx)
                for iy in range(y0, y1):
                    dy = iy - cy
                    if dx2 + dy*dy <= r*r:
                        data[iy, ix] = 100
            after = np.count_nonzero(data == 100)
            self.get_logger().info(f"blocked cells: {after - before}")
            if after- before >250:
                self.done_once=True
                self.get_logger().info("True")
            grid.data = data.flatten().tolist()
            self.mask_pub.publish(grid)
            
            self.get_logger().info(f"Keepout mask 퍼블리시 완료! ({self.publish_count}/{self.max_publish})")

def main():
    rclpy.init()
    node = TfPointTransform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

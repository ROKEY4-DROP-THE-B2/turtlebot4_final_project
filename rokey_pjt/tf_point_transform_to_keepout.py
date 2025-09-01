# tf_point_transform.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32,Header
from geometry_msgs.msg import PointStamped,Pose
import tf2_ros
import tf2_geometry_msgs  # 꼭 필요
from nav_msgs.msg import OccupancyGrid, MapMetaData
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy,HistoryPolicy
#occupy grid 추가
import math
import numpy as np
from nav_msgs.msg import OccupancyGrid


NORMALIZE_DEPTH_RANGE = 3.0
DISTANCE_INFO_TOPIC = '/robot1/distance'     # 현재 퍼블리시 방식 유지 (절대 토픽)
MASK_TOPIC = '/robot1/explosive_keepout_mask'        # 상대 토픽(네임스페이스 자동 적용)
TARGET_FRAME = 'map'                        # local_costmap.global_frame과 일치
class TfPointTransform(Node):
    def __init__(self):
        super().__init__('tf_point_transform')
        self.publish_count = 0
        self.max_publish   = 50
        # TF Buffer와 Listener 준비
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.map_info_synced = False
        # 5초 후에 변환 시작
        self.get_logger().info("TF Tree 안정화 시작. 5초 후 변환 시작합니다.")
        self.start_timer = self.create_timer(5.0, self.start_transform)

      
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)

        # --- keepout mask 퍼블리셔 QoS (라칭) ---
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self.mask_pub = self.create_publisher(OccupancyGrid, MASK_TOPIC, latched_qos)

        

    

        self.distance_subscription = self.create_subscription(
            Float32, DISTANCE_INFO_TOPIC, self.distance_subscription_callback, 10
        )

        # self.distance_subscription = self.create_subscription(
        #     Float32, DISTANCE_INFO_TOPIC, self.distance_subscription_callback, 10
        # )
        #occupy grid
        self.resolution = 0.05  # 0.05cm 현재는 임시로 글로벌로 맞춤
        self.width  = 112
        self.height = 80
        self.origin_x = -0.486  
        self.origin_y = -0.458
        
        
       
        # OccupancyGrid(keepout 마스크) 발행
    
    def map_cb(self, msg: OccupancyGrid):
        info = msg.info
        self.resolution = info.resolution
        self.width      = info.width
        self.height     = info.height
        self.origin_x   = info.origin.position.x
        self.origin_y   = info.origin.position.y
        self.map_info_synced = True #여기까지 수정 함
        self.get_logger().info("Keepout mask info synced to /map")
        self.destroy_subscription(self.map_sub) # 한 번만 받아도 되면 해제
    def publish_mask(self, x_m, y_m):
        if self.publish_count <self.max_publish:
            grid = OccupancyGrid()
            current_time = self.get_clock().now()
            grid.header.stamp = current_time.to_msg()
            grid.header.frame_id = "map"

            info = MapMetaData()
            info.resolution = self.resolution
            info.width = self.width
            info.height = self.height
            # 맵 중심을 (0,0)로 두는 간단한 로컬 마스크 (원하면 원점/크기 조정)
            info.origin.position.x = self.origin_x
            info.origin.position.y = self.origin_y
            info.origin.orientation.w = 1.0
            grid.info = info
            
            # 전부 0(자유)로 시작
            data = np.zeros((self.height, self.width), dtype=np.int8)
        #카운트

            # (x_m, y_m) → grid index
            cx = int((x_m - self.origin_x) / self.resolution)
            cy = int((y_m - self.origin_y) / self.resolution)

            r = int(1 / self.resolution)  # 반경 1m keepout
            self.get_logger().info(
                f"mask ROI: map=({x_m:.2f},{y_m:.2f}), "
                f"grid=({cx},{cy}), size=({self.width},{self.height}), r={r}"
            )



            before = np.count_nonzero(data == 100)

            # 원형으로 100(장애/금지) 채우기
            x0 = max(0, cx - r); x1 = min(self.width,  cx + r + 1)
            y0 = max(0, cy - r); y1 = min(self.height, cy + r + 1)
            for ix in range(x0, x1):
                dx = ix - cx
                dx2 = dx * dx
                for iy in range(y0, y1):
                    dy = iy - cy
                    if dx2 + dy*dy <= r*r:
                        # data[iy * self.width + ix] = 100  # keepout
                        data[iy, ix] = 100
            after = np.count_nonzero(data == 100)
            self.get_logger().info(f"blocked cells: {after - before}")
            grid.data = data.flatten().tolist()
            self.mask_pub.publish(grid)
            
            self.get_logger().info("Keepout mask 퍼블리시 완료!")
            self.publish_count += 1


    def distance_subscription_callback(self, msg: Float32):
    
        p_map = self.transform_base_to_map(msg.data)
        if p_map is None:
            return
        self.get_logger().info(f"[map] ({p_map.point.x:.2f}, {p_map.point.y:.2f}, {p_map.point.z:.2f})")
        self.publish_mask(p_map.point.x, p_map.point.y)  # <- 여기서 마스크 작성/퍼블리시

    def transform_base_to_map(self, forward_x_m: float):
        # 1) base_link 기준 포인트 (stamp는 '최신' 의미의 Time()로)
        pt = PointStamped()
        pt.header.frame_id = 'base_link'
        pt.header.stamp = Time().to_msg()   # 최신 TF 기준을 쓰게 함
        pt.point.x = float(forward_x_m)
        pt.point.y = 0.0
        pt.point.z = 0.0

        # 2) 최신 TF가 준비됐는지 확인 (짧은 타임아웃)
        if not self.tf_buffer.can_transform('map', 'base_link', Time(), timeout=Duration(seconds=0.5)):
            self.get_logger().warn("TF not ready yet (map<-base_link)")
            return None

        # 3) 최신 TF로 변환
        tf = self.tf_buffer.lookup_transform('map', 'base_link', Time(), timeout=Duration(seconds=0.5))
        p_map = tf2_geometry_msgs.do_transform_point(pt, tf)

        # 4) 결과 포인트의 stamp를 'TF의 시각'으로 맞춰서 미래 외삽 경고 방지
        p_map.header.stamp = tf.header.stamp
        p_map.header.frame_id = 'map'
        return p_map
    def start_transform(self):
        self.get_logger().info("TF Tree 안정화 완료. 변환 시작합니다.")
        
        self.start_timer.cancel()  # 한 번만 실행

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
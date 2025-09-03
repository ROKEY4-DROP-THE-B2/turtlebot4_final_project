##

# tf_point_transform.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped, Vector3
import tf2_ros
from rcl_interfaces.msg import SetParametersResult

import tf2_geometry_msgs
from nav_msgs.msg import OccupancyGrid, MapMetaData
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import numpy as np 
from nav2_msgs.msg import CostmapFilterInfo 
DISTANCE_INFO_TOPIC = '/robot2/distance'
MASK_TOPIC = '/robot2/explosive_keepout_mask'
INFO_TOPIC = '/robot2/keepout_filter_info' 
ENEMY_DETECTED = 'enemy_detected'
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
        
        # ★ 재발행 엔진 상태(A/B 공용)
        self.last_mask = None
        self._prev_subs = 0
        self._cooldown = False
        self._burst_timer = None
        self._burst_remaining = 0
        self.declare_parameter('hb_period_sec', 3.0)        # 하트비트 주기(초)
        self.declare_parameter('publish_on_change', True)   # 변경될 때만 발행
        self.declare_parameter('min_changed_cells', 10)     # 최소 변경 셀 수

        self.hb_period_sec     = float(self.get_parameter('hb_period_sec').value)
        self.publish_on_change = bool(self.get_parameter('publish_on_change').value)
        self.min_changed_cells = int(self.get_parameter('min_changed_cells').value)
        # /map 구독 (라칭 QoS)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        # 절대 경로라면 앞에 / 필요
        self.map_sub = self.create_subscription(OccupancyGrid, '/robot2/map', self.map_cb, map_qos)

        # keepout 퍼블리셔 (라칭)
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self.mask_pub = self.create_publisher(OccupancyGrid, MASK_TOPIC, latched_qos)
        self.hb_timer = self.create_timer(self.hb_period_sec, self._heartbeat)
        self.add_on_set_parameters_callback(self._on_set_params)
        self._last_np = None
        self.detect_enemy_publisher = self.create_publisher(Vector3, ENEMY_DETECTED, 10)

        # 거리 구독(메시지 콜백은 'distance_callback'로 별도)

        

        # info_qos = QoSProfile(
        #     depth=1,
        #     reliability=ReliabilityPolicy.RELIABLE,
        #     durability=DurabilityPolicy.TRANSIENT_LOCAL,     # ★ TL (래치 수신)
        #     history=HistoryPolicy.KEEP_LAST
        # )
        # # self.create_subscription(CostmapFilterInfo, INFO_TOPIC, self._on_info, info_qos)
      
        


        # 초기값 (곧 /map으로 덮임)
        self.resolution = 0.05
        self.width  = 112
        self.height = 80
        self.origin_x = -0.486
        self.origin_y = -0.458

        # 주기 타이머 (인자 없는 콜백)
        self.get_logger().info("TF Tree 안정화 시작. 5초 후 변환 시작합니다.")
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
        self._publish_empty_mask_once()
        self.get_logger().info("Keepout mask info synced to /map")
        self.get_logger().info(f"resolution: {self.resolution}")
        self.get_logger().info(f"width: {self.width}")
        self.get_logger().info(f"height: {self.height}")
        self.get_logger().info(f"origin_x: {self.origin_x}")
        self.get_logger().info(f"origin_y: {self.origin_y}")
        self.get_logger().info("Keepout mask info synced to /map")
        self.destroy_subscription(self.map_sub)
    def _publish_empty_mask_once(self):
        if self.last_mask is not None:
            return
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'  # ★ 고정 권장
        info = MapMetaData()
        info.resolution = self.resolution
        info.width      = self.width
        info.height     = self.height
        info.origin.position.x = self.origin_x
        info.origin.position.y = self.origin_y
        info.origin.orientation.w = 1.0
        grid.info = info
        zeros = np.zeros((self.height, self.width), dtype=np.int8)
        grid.data = np.zeros((self.height, self.width), dtype=np.int8).flatten().tolist()
        self.last_mask = grid
        self._last_np  = zeros.copy()
        self.mask_pub.publish(grid)
        self.get_logger().info("Published empty keepout mask (warm-up).")
    
    def start_transform(self):
        self.get_logger().info("TF Tree 안정화 완료. 변환 시작합니다.")
        # 2초마다 "앞으로 1m"를 변환 테스트 (옵션)
        self.create_subscription(
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
        self.publish_enemy_detected()

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
        # /map 메타 아직이면 안전하게 리턴
        if not self.map_info_synced:
            return

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'  # 권장: 고정

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

        w_m = getattr(self, 'rect_w_m', 0.1)   # 가로 총 길이(대칭)
        h_m = getattr(self, 'rect_h_m', 1.6)   # 세로 총 길이(비대칭으로 분할)

        # --- Y만 비대칭 ---
        # 방법 A) 비율로 분할 (기본: 위쪽 75%, 아래쪽 25%)
        up_ratio = float(getattr(self, 'rect_up_ratio', 0.75))  # 0<up_ratio<1
        up_ratio = min(0.95, max(0.05, up_ratio))
        up_m   = h_m * up_ratio
        down_m = h_m - up_m

        # --- 셀로 변환 ---
        half_w = max(1, int(round((w_m/2) / self.resolution)))  # X는 대칭
        uy = max(1, int(round( up_m   / self.resolution)))      # +Y(위쪽)
        dy = max(1, int(round( down_m / self.resolution)))      # -Y(아래쪽)

        # --- 경계 보정 후 채우기 ---
        x0 = max(0, cx - half_w); x1 = min(self.width  - 1, cx + half_w)
        y0 = max(0, cy - dy);     y1 = min(self.height - 1, cy + uy)
        if x0 <= x1 and y0 <= y1:
            data[y0:y1+1, x0:x1+1] = 100

        # 변경 감지: 이전 마스크와 다를 때만 발행
        if self.publish_on_change and self._last_np is not None:
            changed = int(np.count_nonzero(data != self._last_np))
            if changed < self.min_changed_cells:
                self.get_logger().debug(f'no publish (changed={changed} < {self.min_changed_cells})')
                return

        grid.data = data.flatten().tolist()
        
        self.last_mask = grid
        self._last_np = data.copy()     # ★ 다음 비교를 위해 보관
        self.mask_pub.publish(grid)
        self.get_logger().info('Keepout mask publish (change or first)')
    
    def _heartbeat(self):
        if self.last_mask is None:
            return
        # ★ 새 샘플로 인식되도록 stamp를 매번 갱신
        self.last_mask.header.stamp = self.get_clock().now().to_msg()
        self.mask_pub.publish(self.last_mask)

    def _on_set_params(self, params):
        for p in params:
            if p.name == 'hb_period_sec':
                try:
                    new = max(0.5, float(p.value))
                    self.hb_period_sec = new
                    self.hb_timer.cancel()
                    self.hb_timer = self.create_timer(self.hb_period_sec, self._heartbeat)
                    self.get_logger().info(f'HB period -> {self.hb_period_sec}s')
                except Exception as e:
                    return SetParametersResult(successful=False, reason=str(e))
            elif p.name == 'publish_on_change':
                self.publish_on_change = bool(p.value)
                self.get_logger().info(f'publish_on_change -> {self.publish_on_change}')
            elif p.name == 'min_changed_cells':
                self.min_changed_cells = int(p.value)
                self.get_logger().info(f'min_changed_cells -> {self.min_changed_cells}')
        return SetParametersResult(successful=True)
    
    
   


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

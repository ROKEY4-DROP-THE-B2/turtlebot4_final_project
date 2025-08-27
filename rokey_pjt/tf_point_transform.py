# tf_point_transform.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs  # 꼭 필요
from visualization_msgs.msg import Marker

NORMALIZE_DEPTH_RANGE = 3.0
DISTANCE_INFO_TOPIC = '/robot2/distance'
MARKER_TOPIC = 'detected_objects_marker'

class TfPointTransform(Node):
    def __init__(self):
        super().__init__('tf_point_transform')

        # TF Buffer와 Listener 준비
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 5초 후에 변환 시작
        self.get_logger().info("TF Tree 안정화 시작. 5초 후 변환 시작합니다.")
        self.start_timer = self.create_timer(5.0, self.start_transform)

        self.point_x = None

        self.distance_subscription = self.create_subscription(
            Float32, DISTANCE_INFO_TOPIC, self.distance_subscription_callback, 10
        )
    

        self.marker_pub = self.create_publisher(Marker, MARKER_TOPIC, 10)
        self.marker_id = 0

    def publish_marker(self, x, y, z, label):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id = 'detected_objects', self.marker_id
        self.marker_id += 1
        marker.type, marker.action = Marker.SPHERE, Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 1.0, 0.0, 1.0
        marker.lifetime.sec = 5
        self.marker_pub.publish(marker)
    
    def distance_subscription_callback(self, msg):
        self.point_x = msg.data

    def start_transform(self):
        self.get_logger().info("TF Tree 안정화 완료. 변환 시작합니다.")

        # 주기적 변환 타이머 등록
        self.transform_timer = self.create_timer(2.0, self.timer_callback)

        # 시작 타이머 중지 (한 번만 실행)
        self.start_timer.cancel()

    def timer_callback(self):
        try:
            if self.point_x is None or self.point_x < 0:
                self.get_logger().info(f"point_x is not initialized.")
                return
            # base_link 기준 포인트 생성
            point_base = PointStamped()
            point_base.header.stamp = rclpy.time.Time().to_msg()
            point_base.header.frame_id = 'base_link'
            point_base.point.x = self.point_x
            point_base.point.y = 0.0
            point_base.point.z = 0.0

            # base_link → map 변환
            try:
                point_map = self.tf_buffer.transform(
                    point_base,
                    'map',
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                self.get_logger().info(f"[Base_link] ({point_base.point.x:.2f}, {point_base.point.y:.2f}, {point_base.point.z:.2f})")
                self.get_logger().info(f"[Map]       ({point_map.point.x:.2f}, {point_map.point.y:.2f}, {point_map.point.z:.2f})")
            except Exception as e:
                self.get_logger().warn(f"TF transform to map failed: {e}")

        except Exception as e:
            self.get_logger().warn(f"Unexpected error: {e}")

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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct

class VirtualObstaclePublisher(Node):
    def __init__(self):
        super().__init__('virtual_obstacle_publisher')
        
        # PointCloud2 메시지 발행을 위한 퍼블리셔 생성
        self.publisher_ = self.create_publisher(PointCloud2, 'virtual_obstacles', 10)
        self.timer_ = self.create_timer(1.0, self.publish_obstacle_callback)
        self.get_logger().info("Virtual obstacle publisher started.")

    def publish_obstacle_callback(self):
        # 가상 장애물의 중심 좌표 (예: 맵의 5, 5 지점)
        center_x = 5.0
        center_y = 5.0
        
        # 장애물의 크기 (반지름)
        radius = 0.5
        
        # PointCloud2 메시지 생성
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'  # 비용 지도의 프레임과 일치시켜야 함
        
        # PointCloud2 필드 정의 (x, y, z)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        
        # 가상 장애물을 구성할 포인트 데이터
        num_points = 50
        points = []
        for i in range(num_points):
            angle = 2 * np.pi * i / num_points
            x = center_x + radius * np.cos(angle)
            y = center_y + radius * np.sin(angle)
            z = 0.5 # z 좌표 (장애물의 높이)
            
            # 각 포인트를 numpy 배열로 생성
            points.append([x, y, z])
        
        points_np = np.array(points, dtype=np.float32)
        
        # PointCloud2 메시지에 맞게 데이터 변환
        point_step = 12  # 3 * 4바이트 (float32)
        row_step = point_step * num_points
        
        data = points_np.tobytes()

        # 메시지 완성
        point_cloud = PointCloud2(
            header=header,
            height=1,
            width=num_points,
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=point_step,
            row_step=row_step,
            data=data
        )

        self.publisher_.publish(point_cloud)
        self.get_logger().info('Publishing a virtual obstacle PointCloud2.')

def main(args=None):
    rclpy.init(args=args)
    virtual_obstacle_publisher = VirtualObstaclePublisher()
    rclpy.spin(virtual_obstacle_publisher)
    virtual_obstacle_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import math
import os
import sys
import threading
import queue
import numpy as np
from ultralytics import YOLO
# ========================
# 상수 정의
# ========================
MODEL_PATH = '/home/rokey/rokey_ws/model/best_MOH50_polyfit_v11_ver4.pt'
RGB_IMAGE_TOPIC = '/robot1/oakd/rgb/preview/image_raw'
DEPTH_IMAGE_TOPIC = '/robot1/oakd/stereo/image_raw'
CAMERA_INFO_TOPIC = '/robot1/oakd/stereo/camera_info'
DISTANCE_INFO_TOPIC = '/robot1/distance'
TARGET_CLASS_ID = 0
NORMALIZE_DEPTH_RANGE = 3.0
# ========================
# YOLO 객체 인식 + 추적 + 깊이 노드
# ========================
class YOLOTrackerNode(Node):
    def __init__(self):
        super().__init__('yolo_tracker_node')
        if not os.path.exists(MODEL_PATH):
            self.get_logger().error(f"Model not found: {MODEL_PATH}")
            sys.exit(1)
        self.model = YOLO(MODEL_PATH)
        self.class_names = getattr(self.model, 'names', [])
        self.bridge = CvBridge()
        self.should_shutdown = False
        self.window_name = "YOLO & Depth"
        # 데이터 공유를 위한 멤버 변수 및 Lock
        self.rgb_image_queue = queue.Queue(maxsize=1)
        self.latest_depth_vis_frame = None
        self.latest_raw_depth_frame = None
        self.data_lock = threading.Lock()
        self.camera_info_K = None
        # ROS2 발행 설정
        self.publisher_ = self.create_publisher(
            Float32, DISTANCE_INFO_TOPIC, 10
        )
        # ROS 2 구독 설정
        self.rgb_subscription = self.create_subscription(
            Image, RGB_IMAGE_TOPIC, self.rgb_image_callback, 10)
        self.depth_subscription = self.create_subscription(
            Image, DEPTH_IMAGE_TOPIC, self.depth_image_callback, 10)
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 10)
        # 백그라운드 처리 스레드 시작
        self.worker_thread = threading.Thread(target=self.visualization_loop)
        self.worker_thread.daemon = True
        self.worker_thread.start()
    def rgb_image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if not self.rgb_image_queue.full():
                self.rgb_image_queue.put(frame)
            else:
                try:
                    self.rgb_image_queue.get_nowait()
                except queue.Empty:
                    pass
                self.rgb_image_queue.put(frame)
        except Exception as e:
            self.get_logger().error(f"Error in rgb_image_callback: {e}")
    def depth_image_callback(self, msg):
        try:
            if self.camera_info_K is None:
                self.get_logger().warn('Waiting for CameraInfo...')
                return
            raw_depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_vis = np.nan_to_num(raw_depth_mm, nan=0.0)
            depth_vis = np.clip(depth_vis, 0, NORMALIZE_DEPTH_RANGE * 1000)
            depth_vis = (depth_vis / (NORMALIZE_DEPTH_RANGE * 1000) * 255).astype(np.uint8)
            with self.data_lock:
                self.latest_depth_vis_frame = depth_vis
                self.latest_raw_depth_frame = raw_depth_mm
        except Exception as e:
            self.get_logger().error(f"Error in depth_image_callback: {e}")
    def camera_info_callback(self, msg):
        try:
            if self.camera_info_K is None:
                with self.data_lock:
                    self.camera_info_K = np.array(msg.k).reshape(3, 3)
                    self.get_logger().info("CameraInfo received. K matrix stored.")
        except Exception as e:
            self.get_logger().error(f"Error in camera_info_callback: {e}")
    def visualization_loop(self):
        while not self.should_shutdown:
            try:
                frame = self.rgb_image_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            results = self.model.track(source=frame, persist=True, stream=True,conf=0.6, verbose=False)
            with self.data_lock:
                depth_vis_frame = self.latest_depth_vis_frame
                raw_depth_frame = self.latest_raw_depth_frame
            if depth_vis_frame is not None and raw_depth_frame is not None:
                self.draw_yolo_results(frame, results, raw_depth_frame)
                display_depth = depth_vis_frame.copy()
                display_depth_bgr = cv2.cvtColor(display_depth, cv2.COLOR_GRAY2BGR)
                display_depth_colored = cv2.applyColorMap(depth_vis_frame, cv2.COLORMAP_JET)
                self.draw_yolo_results(display_depth_bgr, results, raw_depth_frame)

                final_image = np.hstack((frame, display_depth_colored))
                cv2.imshow(self.window_name, final_image)
            else:
                self.draw_yolo_results(frame, results, depth_frame=None)
                cv2.imshow(self.window_name, frame)
            key = cv2.waitKey(1)
            if key == ord('q'):
                self.should_shutdown = True
                #self.get_logger().info("Q pressed. Shutting down...")
                break
    
    def draw_yolo_results(self, img, results, depth_frame=None):
        object_count = 0
        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                if cls != TARGET_CLASS_ID:
                    continue
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = math.ceil(box.conf[0] * 100) / 100
                label = self.class_names[cls] if cls < len(self.class_names) else f"class_{cls}"
                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                distance_m = -1.0
                if depth_frame is not None:
                    if 0 <= center_y < depth_frame.shape[0] and 0 <= center_x < depth_frame.shape[1]:
                        distance_mm = depth_frame[center_y, center_x]
                        if distance_mm > 0:
                            distance_m = distance_mm / 10.0
                if distance_m > 0:
                    msg = Float32()
                    self.get_logger().info(f"cm:{distance_m :.2f}")
                    self.get_logger().info(f"conf:{conf:.2f}")
                    msg.data = distance_m
                    self.publisher_.publish(msg)
                    text = f"{label} {distance_m:.2f}m ({conf:.2f})"
                else:
                    text = f"{label} ({conf:.2f})"
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(img, text, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                object_count += 1
        cv2.putText(img, f"Objects: {object_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
# ========================
# 메인 함수
# ========================
def main():
    rclpy.init()
    node = YOLOTrackerNode()
    try:
        while rclpy.ok() and not node.should_shutdown:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        print("Shutdown complete.")
        sys.exit(0)
if __name__ == '__main__':
    main()
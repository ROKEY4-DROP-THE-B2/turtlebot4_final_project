import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math
import os
import sys
import threading
import queue
from ultralytics import YOLO

# ========================
# 상수 정의
# ========================
MODEL_PATH = '/home/rokey/rokey_ws/src/yolov8_ros/runs/detect/yolov8n_train_results/weights'  # YOLO 모델 경로
IMAGE_TOPIC = '/robot2/oakd/rgb/preview/image_raw'     # 이미지 토픽
TARGET_CLASS_ID = 0                                     # 인식할 클래스 ID

# ========================
# YOLO 객체 인식 노드 정의
# ========================
class YOLOViewerNode(Node):
    def __init__(self):
        super().__init__('yolo_viewer_node')

        # 모델 로딩
        if not os.path.exists(MODEL_PATH):
            self.get_logger().error(f"Model not found: {MODEL_PATH}")
            sys.exit(1)
        self.model = YOLO(MODEL_PATH)
        self.class_names = getattr(self.model, 'names', [])

        self.bridge = CvBridge()

        # 이미지 큐: 백그라운드 스레드와 공유
        self.image_queue = queue.Queue(maxsize=1)  # 최신 프레임만 유지
        self.should_shutdown = False
        self.window_name = "YOLO Detection"

        # 이미지 구독
        self.subscription = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_callback,
            10
        )

        # 백그라운드 처리 스레드 시작
        self.worker_thread = threading.Thread(target=self.visualization_loop)
        self.worker_thread.daemon = True
        self.worker_thread.start()

    def image_callback(self, msg):
        # ROS 메시지를 OpenCV 이미지로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 큐에 프레임 추가 (이전 프레임 덮어쓰기)
        if not self.image_queue.full():
            self.image_queue.put(frame)
        else:
            try:
                self.image_queue.get_nowait()  # 이전 프레임 버림
            except queue.Empty:
                pass
            self.image_queue.put(frame)

    def visualization_loop(self):
        # YOLO + 시각화 백그라운드 루프
        while not self.should_shutdown:
            try:
                frame = self.image_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            results = self.model(frame, stream=True)
            object_count = 0

            for r in results:
                for box in r.boxes:
                    cls = int(box.cls[0])
                    if cls != TARGET_CLASS_ID:
                        continue

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = math.ceil(box.conf[0] * 100) / 100
                    label = self.class_names[cls] if cls < len(self.class_names) else f"class_{cls}"

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(frame, f"{label}: {conf}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                    object_count += 1

            cv2.putText(frame, f"Objects: {object_count}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # 시각화
            display_img = cv2.resize(frame, (frame.shape[1]*2, frame.shape[0]*2))
            cv2.imshow(self.window_name, display_img)

            # 종료 감지
            key = cv2.waitKey(1)
            if key == ord('q'):
                self.should_shutdown = True
                self.get_logger().info("Q pressed. Shutting down...")
                break

# ========================
# 메인 함수
# ========================
def main():
    rclpy.init()
    node = YOLOViewerNode()

    try:
        # ROS 메시지 처리 루프 (YOLO는 스레드에서 따로 처리됨)
        while rclpy.ok() and not node.should_shutdown:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # 자원 정리 및 종료
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        print("Shutdown complete.")
        sys.exit(0)

if __name__ == '__main__':
    main()

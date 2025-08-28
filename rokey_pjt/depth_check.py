import cv2
import depthai as dai
import numpy as np

# Depth 설정
pipeline = dai.Pipeline()

monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
depth = pipeline.create(dai.node.StereoDepth)
xout = pipeline.create(dai.node.XLinkOut)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth.setLeftRightCheck(True)
depth.setExtendedDisparity(False)
depth.setSubpixel(False)

monoLeft.out.link(depth.left)
monoRight.out.link(depth.right)
depth.disparity.link(xout.input)
xout.setStreamName("disparity")

# 클릭된 좌표 저장용 변수
clicked_point = None

# 마우스 클릭 콜백 함수
def on_mouse_click(event, x, y, flags, param):
    global clicked_point
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_point = (x, y)

# 디바이스 실행
with dai.Device(pipeline) as device:
    q = device.getOutputQueue(name="disparity", maxSize=4, blocking=False)
    max_disp = depth.initialConfig.getMaxDisparity()

    cv2.namedWindow("Disparity Viewer")
    cv2.setMouseCallback("Disparity Viewer", on_mouse_click)

    while True:
        inDisparity = q.get()
        disp_frame_raw = inDisparity.getFrame()

        # 시각화용 정규화 및 colormap
        vis_frame = (disp_frame_raw * (255.0 / max_disp)).astype(np.uint8)
        vis_color = cv2.applyColorMap(vis_frame, cv2.COLORMAP_JET)

        # 클릭된 위치가 있으면 픽셀 값 출력
        if clicked_point:
            x, y = clicked_point
            if 0 <= x < vis_color.shape[1] and 0 <= y < vis_color.shape[0]:
                raw_disp_value = disp_frame_raw[y, x]
                print(f"[{x}, {y}] disparity: {raw_disp_value} → 실제 시차: {raw_disp_value / 16.0:.2f}")
                clicked_point = None  # 한 번 출력 후 초기화

        cv2.imshow("Disparity Viewer", vis_color)

        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()

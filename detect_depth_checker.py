import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import math
import os
import sys
import threading
import queue
from ultralytics import YOLO
import numpy as np
from rclpy.executors import MultiThreadedExecutor
import time
from geometry_msgs.msg import PointStamped 
from rclpy.time import Time 

# ========================
# 상수 정의
# ========================
MODEL_PATH = '/home/happy/rokey_ws/models/best_for_robot0.pt'  # YOLO 모델 경로
IMAGE_TOPIC = '/robot0/oakd/rgb/preview/image_raw'     # 이미지 토픽
TARGET_CLASS_ID = [1]                                    # 인식할 클래스 ID
######################################
DEPTH_TOPIC = '/robot0/oakd/stereo/image_raw'  # Depth 이미지 토픽
NORMALIZE_DEPTH_RANGE = 3.0  
#######################################
# ========================
# YOLO 객체 인식 노드 정의
# ========================



class DepthImage(Node):
    def __init__(self):
        super().__init__('depth_image_node')
        ###################
        self.bridge = CvBridge()
        self.image_queue_depth = queue.Queue(maxsize=1)

        #############
        self.depth_subscription = self.create_subscription(
            Image,
            DEPTH_TOPIC,
            self.depth_callback,
            10)
        

    def depth_callback(self, msg):
        # print("[DepthImage] Received depth frame")  # 확인용 로그
        depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if not self.image_queue_depth.full():
            self.image_queue_depth.put(depth_mm)
        else:
            try:
                self.image_queue_depth.get_nowait()
            except queue.Empty:
                pass
            self.image_queue_depth.put(depth_mm)
        #############
###################
class RGBImage(Node):
    def __init__(self):
        super().__init__('rgb_image_node')
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=1)  # 최신 프레임만 유지

        self.subscription = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        # ROS 메시지를 OpenCV 이미지로 변환
        # print("[RGBImage] Received RGB frame")  # 확인용 로그
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

class YOLOViewerNode(Node):
    def __init__(self,image_queue, depth_queue):
        super().__init__('yolo_viewer_node')
        # 모델 로딩
        self.image_queue = image_queue
        self.image_queue_depth = depth_queue
        self.frame_counter = 0
        self.annotation_interval = 5 ### 이 부분이 경량화에 기여함
        
        if not os.path.exists(MODEL_PATH):
            self.get_logger().error(f"Model not found: {MODEL_PATH}")
            sys.exit(1)
        self.model = YOLO(MODEL_PATH)
        self.class_names = getattr(self.model, 'names', [])
        
        # 이미지 큐: 백그라운드 스레드와 공유
        
        
        self.should_shutdown = False
        self.window_name = "YOLO Detection"
        self.object_detection_publisher = self.create_publisher(PointStamped, '/object_detection_info', 10)
        # 이미지 구독
        
        
        # 백그라운드 처리 스레드 시작
        self.worker_thread = threading.Thread(target=self.visualization_loop)
        self.worker_thread.daemon = True
        self.worker_thread.start()
    
    
    def visualization_loop(self):
        # YOLO + 시각화 백그라운드 루프
        while not self.should_shutdown:
            try:
                depth_colored = np.zeros((320, 320, 3), dtype=np.uint8)  # 초기값
                if self.frame_counter % self.annotation_interval == 0:
                    frame_depth = self.image_queue_depth.get(timeout=0.1)
                else:
                    self.frame_counter += 1
                    continue
                self.frame_counter += 1
                
                frame = self.image_queue.get(timeout=0.1)
                ##################
                # 원본 이미지 크기
                orig_h, orig_w = frame.shape[:2]

                # 1/2로 축소
                resized = cv2.resize(frame, (orig_w // 2, orig_h // 2), interpolation=cv2.INTER_AREA)

                # 흰색(255,255,255) 배경 320x320 이미지 생성
                canvas = np.ones((320, 320, 3), dtype=np.uint8) * 255

                # 축소 이미지의 크기
                resized_h, resized_w = resized.shape[:2]

                # 중앙 배치를 위한 오프셋 계산
                y_offset = (320 - resized_h) // 2
                x_offset = (320 - resized_w) // 2

                # 축소 이미지를 중앙에 배치
                canvas[y_offset:y_offset+resized_h, x_offset:x_offset+resized_w] = resized

                # 최종 YOLO 입력 이미지
                frame = canvas.copy()
                ################
            except queue.Empty:
                continue
            
            if self.frame_counter == 5:
                self.frame_counter = 0
            results = self.model(frame, stream=True)

            
            object_count = 0
#####################
            ####################
            for r in results:
                for box in r.boxes:
                    cls = int(box.cls[0])
                    if cls not in TARGET_CLASS_ID:
                        continue
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = math.ceil(box.conf[0] * 100) / 100
                    label = self.class_names[cls] if cls < len(self.class_names) else f"class_{cls}"
                    ###########################o.h.t
                    cx = int((x1+x2)/2)
                    cy = int((y1+y2)/2)
                    patch = frame_depth[max(0, cy-2):cy+3, max(0, cx-2):cx+3]
                    valid = patch[patch > 0]
                    if valid.size > 0:
                        distance_mm = np.mean(valid)
                        distance_m = distance_mm / 1000.0
                    else:
                        continue  # 거리 없으면 이 감지는 버림

                    # distance_mm = frame_depth[cy, cx]
                    # distance_m = distance_mm / 1000.0

                    ##############################
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(frame, f"{label}: {conf}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                        ##############o.h.t
                    cv2.putText(frame, f"Distance = {distance_m:.2f}m", (x1,y2+10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,  (0,0,255), 2)
                        ############
                    # PointStamped 메시지 생성 및 발행
                    point_msg = PointStamped()
                    point_msg.header.stamp = self.get_clock().now().to_msg()
                    point_msg.header.frame_id = "oakd_rgb_camera_optical_frame"  # 카메라 프레임 ID (object_xyz.py와 일치해야 함)
                    point_msg.point.x = float(cx)
                    point_msg.point.y = float(cy)
                    point_msg.point.z = float(distance_m) # 깊이 정보 (미터 단위)
                    self.object_detection_publisher.publish(point_msg)
                    self.get_logger().info(f"Published object at pixel ({cx}, {cy}) with depth {distance_m:.2f}m")

                    ###############
                    # # 시각화용 정규화 (mm → m 고려)
                    # depth_vis = np.nan_to_num(frame_depth, nan=0.0)
                    # depth_vis = np.clip(depth_vis, 0, NORMALIZE_DEPTH_RANGE * 1000)  # mm
                    # depth_vis = (depth_vis / (NORMALIZE_DEPTH_RANGE * 1000) * 255).astype(np.uint8)

                    # 컬러맵 적용
                    # depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                    # cv2.circle(depth_colored, (cx, cy), 2, (255,0,0), 1)
                    # cv2.rectangle(depth_colored, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    ################
                    object_count += 1
            cv2.putText(frame, f"Objects: {object_count}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # 시각화
            display_img = cv2.resize(frame, (frame.shape[1]*1, frame.shape[0]*1))
            
            
            cv2.imshow(self.window_name, display_img)
            # cv2.imshow("try", depth_colored)
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
    rgb_image = RGBImage()
    depth_image = DepthImage()

    
    node = YOLOViewerNode(
        image_queue = rgb_image.image_queue,
        depth_queue = depth_image.image_queue_depth

    )
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(rgb_image)
    executor.add_node(depth_image)

    # executor.spin()을 별도 스레드로 실행
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        while rclpy.ok() and not node.should_shutdown:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        for n in [rgb_image, depth_image, node]:
            if n.context.ok():
                n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()
        print("Shutdown complete.")
        sys.exit(0)
if __name__ == '__main__':
    main()

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
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Int8, Bool
import time
import random
from paho.mqtt import client as mqtt_client
# ========================
# 상수 정의
# ========================
# MODEL_PATH = '/home/rokey/rokey_ws/src/yolov8_ros/yolov8_ros/best_for_robot0.pt'  # YOLO 모델 경로
MODEL_PATH = '/home/rokey/rokey_ws/src/yolov8_ros/yolov8_ros/best_for_robot1.pt'  # YOLO 모델 경로
IMAGE_TOPIC = '/robot1/oakd/rgb/preview/image_raw'     # 이미지 토픽
# TARGET_CLASS_ID = [0,1,2,3,4]                                   # 인식할 클래스 ID
TARGET_CLASS_ID = [0,1,2,3]                                   # 인식할 클래스 ID
###################################### **************************
DEPTH_TOPIC = '/robot1/oakd/stereo/image_raw'  # Depth 이미지 토픽
NORMALIZE_DEPTH_RANGE = 3.0
#######################################
# ========================
# YOLO 객체 인식 노드 정의
# ========================
####stb####
broker = 'c41c91fd.ala.eu-central-1.emqxsl.com'
port = 8883
username = 'taek'
password = '991031'
topic = "mqtt_empty_shelf"  
client_id = f'python-mqtt-{random.randint(0, 100)}'
####stb####
class DepthImage(Node): #*****************************
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
    def __init__(self,image_queue, depth_queue): #, depth_queue (depth 사용시 추가)
        super().__init__('yolo_viewer_node')
        #######초당 프레임 보기
        self.prev_time = time.time()
        self.fps_counter = 0
        self.fps_display = 0

        #######
        # 모델 로딩
        self.destination = None
        self.command_key = 0
        self.close = ""
        self.yolo_point_pub = self.create_publisher(PointStamped, "/object_detection_info", 10)
        
        self.image_queue = image_queue
        self.image_queue_depth = depth_queue ###*************************
        self.frame_counter = 0
        self.annotation_interval = 10 ### 이 부분이 경량화에 기여함
        self.agree = False
        ####stb####
        # self.ros_pub = self.create_publisher(Int8, 'mqtt_log_int', 10)
        self.mqtt_client = self.connect_mqtt()
        self.mqtt_client.loop_start()
        self.get_logger().info("MQTT Int Publisher Node started!")
        ####stb####
        self.agree_subscription = self.create_subscription(
            Bool,
            '/agree_topic',
            self.agree_callback,
            10
        )
        self.close_subscription = self.create_subscription(
            String,
            '/close_topic',
            self.close_callback,
            10
        )
        self.destination_subscription = self.create_subscription(
            Int8,
            '/destination',
            self.destination_callback,
            10
        )
        if not os.path.exists(MODEL_PATH):
            self.get_logger().error(f"Model not found: {MODEL_PATH}")
            sys.exit(1)
        self.model = YOLO(MODEL_PATH)
        self.class_names = getattr(self.model, 'names', [])
        print(self.class_names)
        # 이미지 큐: 백그라운드 스레드와 공유
        self.should_shutdown = False
        self.window_name = "YOLO Detection"
        # 이미지 구독
        # 백그라운드 처리 스레드 시작
        self.worker_thread = threading.Thread(target=self.visualization_loop)
        self.worker_thread.daemon = True
        self.worker_thread.start()
    
    ####stb####
    def connect_mqtt(self) -> mqtt_client.Client:
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                self.get_logger().info("Connected to MQTT Broker!")
            else:
                self.get_logger().error(f"Failed to connect, return code {rc}")

        client = mqtt_client.Client(client_id=client_id,
                                    protocol=mqtt_client.MQTTv311)
        client.tls_set()                         # TLS 사용(Eclipse EMQX 클라우드용)
        client.username_pw_set(username, password)
        client.on_connect = on_connect
        client.connect(broker, port)
        return client
####stb####

    def destination_callback(self, msg: Int8):
        self.destination = msg.data
    def close_callback(self, msg:String):
        self.close = msg.data
    def agree_callback(self, msg: Bool):
        """Bool 토픽 수신 콜백"""
        self.agree = msg.data
    def visualization_loop(self):
        # YOLO + 시각화 백그라운드 루프
        while not self.should_shutdown:
            try:
                ###시각화 ***************************
                # depth_colored = np.zeros((320, 320, 3), dtype=np.uint8)  # 초기값
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
            # if (현재좌표) in label_locate.values():
            if self.agree == True:
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
                        ###########################o.h.t #*****************************
                        cx = int((x1+x2)/2)
                        cy = int((y1+y2)/2)
                        distance_mm = frame_depth[cy, cx]
                        distance_m = distance_mm / 1000.0
                        msg = PointStamped()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = "camera_rgb_optical_frame"
                        msg.point.x = float(cx)
                        msg.point.y = float(cy)
                        msg.point.z = float(distance_m)
                        self.yolo_point_pub.publish(msg)
                        ##############################
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        cv2.putText(frame, f"{label}: {conf}", (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                            ##############o.h.t #**********************************
                        cv2.putText(frame, f"Distance = {distance_m:.2f}m", (x1,y2+10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,  (0,0,255), 2)
                            ############
                        # ############### 이 부분이 시각화 부분
                        # # 시각화용 정규화 (mm → m 고려)
                        # depth_vis = np.nan_to_num(frame_depth, nan=0.0)
                        # depth_vis = np.clip(depth_vis, 0, NORMALIZE_DEPTH_RANGE * 1000)  # mm
                        # depth_vis = (depth_vis / (NORMALIZE_DEPTH_RANGE * 1000) * 255).astype(np.uint8)
                        # # 컬러맵 적용
                        # depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                        # # cv2.circle(depth_colored, (cx, cy), 2, (255,0,0), 1)
                        # cv2.rectangle(depth_colored, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        # ################
                        object_count += 1
                cv2.putText(frame, f"Objects: {object_count}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                time.sleep(0.5)
                if object_count == 0:
                    object_detection_count = Int8()
                    object_detection_count.data = object_count
                    
                    ################## 여기서 mqtt
                    ####stb####
                    self.mqtt_client.publish(topic, str(self.destination))
                    
            
            # 시각화
            display_img = cv2.resize(frame, (frame.shape[1]*1, frame.shape[0]*1))
            cv2.imshow(self.window_name, display_img)
            # cv2.imshow("try", depth_colored)
            # ##########fps 계산
            # self.fps_counter += 1
            # current_time = time.time()
            # elapsed = current_time - self.prev_time
            # if elapsed >= 1.0:  # 1초마다 FPS 업데이트
            #     self.fps_display = self.fps_counter / elapsed
            #     self.fps_counter = 0
            #     self.prev_time = current_time
            #     self.get_logger().info(f"Current FPS: {self.fps_display:.2f}")
            # #########3

            # 종료 감지
            key = cv2.waitKey(1)
            if self.close == 'e':
                self.should_shutdown = True
                self.get_logger().info("Q pressed. Shutting down...")
                break
# ========================
# 메인 함수
# ========================
def main():
    rclpy.init()
    rgb_image = RGBImage()
    depth_image = DepthImage() #*************
    node = YOLOViewerNode(
        image_queue = rgb_image.image_queue,
        depth_queue = depth_image.image_queue_depth #***************
    )
    executor = MultiThreadedExecutor(num_threads=2) #*************** depth는 2로
    executor.add_node(rgb_image)
    executor.add_node(depth_image)#********************
    # executor.spin()을 별도 스레드로 실행
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    try:
        while rclpy.ok() and not node.should_shutdown:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        ####stb####
        node.mqtt_client.loop_stop()
        ####stb####
        for n in [rgb_image,depth_image, node]: # depth_image,  (depth사용시 추가할것)
            if n.context.ok():
                n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()
        print("Shutdown complete.")
        sys.exit(0)
if __name__ == '__main__':
    main()
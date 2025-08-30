import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int8, String, Bool
from geometry_msgs.msg import PoseStamped
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from tf_transformations import quaternion_from_euler
import time
import threading
import random
from paho.mqtt import client as mqtt_client
from rclpy.qos import QoSProfile, ReliabilityPolicy

####stb####
broker = 'c41c91fd.ala.eu-central-1.emqxsl.com'
port = 8883
username = 'taek'
password = '991031'
topic = "mqtt_robot0_status"  
client_id = f'python-mqtt-{random.randint(0, 100)}'
####stb####

class ShelfRestocker(Node):
    def __init__(self):
        super().__init__('shelf_restocker')

        # 네비게이션 컨트롤러
        self.nav = TurtleBot4Navigator(namespace='/robot0')

        # 작업 큐 및 상태 변수
        self.task_queue = []
        self.is_busy = False
        ####stb####
        self.ros_pub = self.create_publisher(Bool, 'mqtt_log', 10)
        # self.msg_count = 1

        # MQTT 연결
        self.mqtt_client = self.connect_mqtt()
        self.mqtt_client.loop_start()  # MQTT 비동기 루프 시작

        self.get_logger().info("MQTT Publisher Node started!")
        # msg = f"Message number {self.msg_count}"
        # self.mqtt_client.publish(topic, msg)   

        ####stb#### 
        self.lock = threading.Lock()

        # 좌표 정보
        self.shelf_positions = [
            (-3.594, 1.954, 0.0),  # 진열장 0: 포키
            (-2.845, 1.989, 0.0), # 진열장 1: 레쓰비
            (-1.932, 2.024, 0.0), # 진열장 2: 초코송이
            (-1.033, 1.977, 0.0), # 진열장 3: 씨리얼
        ]

        self.storage_positions = [
            (-1.101, -0.191, 180.0),  # 창고 0: 포키
            (-1.863, -0.165, 180.0), # 창고 1: 레쓰비
            (-2.845, -0.206, 180.0), # 창고 2: 초코송이
            (-3.478, -0.220, 180.0), # 창고 3: 씨리얼
            (-4.164, -0.230, 90.0),   # 창고 나가는 좌표
            (-4.164, 2.000, 0.0),     # 진열장 가기 직전 좌표
            (-0.300, 1.977, 0.0),     # 마지막 포인트
        ]

        self.wait_position = (0.0, 0.0, 0.0)
        self.final_position = (-0.2, -0.01, 180.0)

        # 외부(진열장순찰로봇)에서 오는 작업 요청 구독 (빈 진열장 번호)
        custom_qos = QoSProfile(
            depth=200,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.task_subscription = self.create_subscription(
            Int8,
            '/empty_shelf',
            self.empty_shelf_callback,
            custom_qos
        )

        # 웹UI 신호 대기용 이벤트 및 토픽 구독자 추가
        self.task_event = threading.Event()
        self.signal_sub = self.create_subscription(
            Int8,
            '/restock_signal',
            self.signal_callback,
            10
        )

        initial_pose = self.create_pose(*self.wait_position)
        self.nav.setInitialPose(initial_pose)
        self.get_logger().info("초기 위치 설정 완료.")
        self.get_logger().info("ShelfRestocker 노드 시작. Nav2 활성화를 기다립니다...")
        self.nav.waitUntilNav2Active()
        self.get_logger().info("Nav2 활성화 완료. 로봇이 준비되었습니다.")
        time.sleep(1.5)

        # 사용자 입력을 받는 별도 스레드 시작 (무슨뜻)
        self.run_main_thread()

    # 출발 신호 콜백 추가

        ####stb
    def connect_mqtt(self) -> mqtt_client.Client:
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                self.get_logger().info("Connected to MQTT Broker!")
            else:
                self.get_logger().error(f"Failed to connect, return code {rc}")

        client = mqtt_client.Client(client_id=client_id, protocol=mqtt_client.MQTTv311)
        client.tls_set()
        client.username_pw_set(username, password)
        client.on_connect = on_connect
        client.connect(broker, port)
        return client
####stb

    def signal_callback(self, msg):
        if msg.data == 1:
            self.get_logger().info("작업자 신호 수신 → 다음 단계로 진행")
            self.task_event.set()

    def empty_shelf_callback(self, msg):
        self.get_logger().info(f"디버깅로그: 콜백실행됨 새 작업({msg.data}) 수신")
        with self.lock:
            if self.is_busy:
                self.get_logger().warn(f"로봇이 작업 중입니다. 새 작업({msg.data})은 무시됩니다.")
                return
            
            shelf_id = msg.data
            if shelf_id not in self.task_queue:
                self.task_queue.append(shelf_id)
                self.get_logger().info(f"새 작업 추가: 진열장 {shelf_id}")
                self.get_logger().info(f"현재 작업 큐: {self.task_queue}")

    # 오일러yaw -> 쿼터니언 
    def create_pose(self, x, y, yaw_deg):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw_deg * 3.141592 / 180.0)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose
    
    def run_main_thread(self):
        thread = threading.Thread(target=self.wait_for_input_loop, daemon=True)
        thread.start()

    def wait_for_input_loop(self):
        while rclpy.ok():
            # input("\n엔터를 눌러 작업을 시작하세요 (큐가 비어있으면 도킹합니다).\n")
            self.get_logger().info(f"버튼을 클릭하여 작업을 시작하세요. (큐가 비어있으면 도킹합니다.)")
            self.task_event.clear()
            self.task_event.wait()   
                     
            with self.lock:
                if self.is_busy:
                    self.get_logger().warn("이미 다른 작업이 진행 중입니다.")
                    continue

                if not self.task_queue:
                    if not self.nav.getDockedStatus():
                        self.get_logger().info("수행할 작업이 없습니다. 도킹을 시작합니다.")
                        self.nav.dock()
                    else:
                        self.get_logger().info("수행할 작업이 없으며, 이미 도킹된 상태입니다.")
                    continue

                # 작업 시작 준비
                self.is_busy = True
                current_queue = sorted(set(self.task_queue))
                self.task_queue.clear()

                # 실제 작업은 별도 스레드에서 실행
                task_thread = threading.Thread(
                    target=self.execute_task_sequence,
                    args=(current_queue,),
                    daemon=True
                )
                task_thread.start()

    def execute_task_sequence(self, tasks):
        # 언도킹
        with self.lock:
            is_docked = self.nav.getDockedStatus()

        if is_docked:
            self.get_logger().info("작업 시작을 위해 언도킹합니다...")
            self.nav.undock()

        # 창고 방문 및 적재
        for shelf_id in tasks:
            self.get_logger().info(f":앞쪽_화살표: 창고 {shelf_id}로 이동합니다.")
            storage_pose = self.create_pose(*self.storage_positions[shelf_id])
            self.nav.goToPose(storage_pose)
            while not self.nav.isTaskComplete():
                time.sleep(0.1)

            # 출발 신호 대기 (수정됨: input → 이벤트)
            self.get_logger().info(f"  ↳ 창고 {shelf_id} 도착. 출발 신호를 기다립니다... (/restock_signal)")
            self.task_event.clear()
            self.task_event.wait()

        # 창고 나가기 경유지
        self.nav.goToPose(self.create_pose(*self.storage_positions[4]))
        while not self.nav.isTaskComplete():
            time.sleep(0.1)

        # 진열장 진입 경유지
        self.nav.goToPose(self.create_pose(*self.storage_positions[5]))
        while not self.nav.isTaskComplete():
            time.sleep(0.1)

        # 진열장 방문 및 진열
        for shelf_id in tasks:
            self.get_logger().info(f":앞쪽_화살표: 진열장 {shelf_id}으로 이동합니다.")
            shelf_pose = self.create_pose(*self.shelf_positions[shelf_id])
            self.nav.goToPose(shelf_pose)
            while not self.nav.isTaskComplete():
                time.sleep(0.1)

            # 출발 신호 대기 (수정됨: input → 이벤트)
            self.get_logger().info(f"  ↳ 진열장 {shelf_id} 도착. 신호를 기다립니다... (/restock_signal)")
            self.task_event.clear()
            self.task_event.wait()

        # 대기위치 가기 전 경유지
        self.nav.goToPose(self.create_pose(*self.storage_positions[6]))
        while not self.nav.isTaskComplete():
            time.sleep(0.1)

        # 대기 위치로 이동
        wait_pose = self.create_pose(*self.final_position)
        self.get_logger().info("모든 작업을 완료했습니다. 대기 위치로 이동합니다.")
        self.nav.goToPose(wait_pose)

        # 작업 완료 후 상태 업데이트
        with self.lock:
            self.is_busy = False
            self.mqtt_client.publish(topic, str(self.is_busy).lower())                        
        self.get_logger().info("로봇이 다음 명령을 기다립니다.")

def main():
    rclpy.init()
    node = ShelfRestocker()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.loop_stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
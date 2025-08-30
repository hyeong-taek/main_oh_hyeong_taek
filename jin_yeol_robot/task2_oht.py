import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from tf_transformations import quaternion_from_euler
from nav2_simple_commander.robot_navigator import TaskResult
from std_msgs.msg import String, Bool, Int8
import time
import threading
class ShelfRestocker(Node):
    def __init__(self):
        super().__init__('shelf_check')
        # 네비게이션 컨트롤러
        self.nav = TurtleBot4Navigator(namespace='/robot1')
        self.detect_publisher = self.create_publisher(Bool, '/agree_topic', 10)
        self.close = self.create_publisher(String, "/close_topic", 10)
        self.destination = self.create_publisher(Int8, '/destination',10)
        self.mqtt_result = None  # 초기값
        self.close_UI = self.create_subscription(String, "/close_from_ui",self.command_on_off, 10)
        self.cmd = ''  # 초기화
        self.object_num = None
        # 작업 큐 및 상태 변수
        self.is_busy = False
        self.lock = threading.Lock()
        #############o.h.t
        self.stop_event = threading.Event()   # ← 순회 중단용 이벤트 추가
# 좌표 정보
        self.shelf_positions = [
            (-3.67296195, -0.31519493460, 90.0),    # 진열장 0: 포키
            (-2.90967226,  -0.30821943283, 91.0),    # 진열장 1: 레쓰비
            (-1.98989188, -0.1464289, 91.0),    # 진열장 2: 초코송이
            (-1.18594396, -0.2057736, 92.0),        # 진열장 3: 씨리얼
        ]
        #창고 입구좌표 -0.3, 0.3, 각도
        #창고 출구 좌표 -4.164, 0.230, 각도
        self.wait_position = (0.0, 0.0, 0.0)
        self.final_position = (-0.5, -0.2, 0.54)
        # --- ROS2 인터페이스 ---
        # 외부에서 오는 작업 요청 구독
        # --------------------
        initial_pose = self.create_pose(*self.wait_position)
        self.nav.setInitialPose(initial_pose)
        self.get_logger().info("초기 위치 설정 완료.")
        self.get_logger().info("ShelfRestocker 노드 시작. Nav2 활성화를 기다립니다...")
        self.nav.waitUntilNav2Active()
        self.get_logger().info("Nav2 활성화 완료. 로봇이 준비되었습니다.")
        time.sleep(1.5)
        # :흰색_확인_표시: 여기 추가!!!
        # 사용자 입력을 받는 별도 스레드 시작
        self.run_main_thread()



    def command_on_off(self,msg):
        self.cmd = msg.data

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
        # 데몬 스레드로 설정하여 메인 프로그램 종료 시 함께 종료되도록 함
        thread = threading.Thread(target=self.wait_for_input_loop, daemon=True)
        thread.start()
    def patrol_shelves(self):
        """0→1→2→3→2→1 순회 반복"""
        pattern = [0, 1, 2, 3, 2, 1]
        hold_time = 4.0  # 각 위치에서 유지 시간(초)
        self.get_logger().info("언도킹 시도 중...")
        # 순회 시작 전 무조건 언도킹 (이미 언도킹 상태면 TaskResult.SUCCEEDED 가 바로 나옴)
        dock_status = self.nav.getDockedStatus()  # e.g., "Docked" or "Undocked"
        # 언도킹 시도
        if dock_status == "Undocked":
            self.get_logger().info("이미 언도킹 상태입니다. 바로 순회를 시작합니다.")
        else:
            self.nav.undock()
            while not self.nav.isTaskComplete():
                time.sleep(0.1)

            print(self.nav.getDockedStatus()) ##### 확인
            if self.nav.getDockedStatus() == False:
                self.get_logger().info("언도킹 완료. 순회를 시작합니다.")
            else:
                self.get_logger().warn(f"언도킹 실패({result})! 순회 시작을 건너뜁니다.")
                with self.lock:
                    self.is_busy = False
                return
        while not self.stop_event.is_set():
            for shelf_id in pattern:
                
                if self.stop_event.is_set():
                    break
                shelf_pose = self.create_pose(*self.shelf_positions[shelf_id])
                self.nav.goToPose(shelf_pose)
                self.get_logger().info(f"진열장 {shelf_id} 이동 시작")
                while not self.nav.isTaskComplete():
                    feedback = self.nav.getFeedback()
                    if feedback:
                        self.get_logger().info(
                            f"남은 거리: {feedback.distance_remaining:.2f} m"
                        )
                    if self.stop_event.is_set():
                        self.nav.cancelTask()
                        break
                    time.sleep(0.2)
                # :흰색_확인_표시: 이동 결과 확인
                result = self.nav.getResult()
                self.get_logger().info(f"진열장 {shelf_id} 이동 결과: {result}")
                # :흰색_확인_표시: 도착 후 hold_time 대기
                self.get_logger().info(f"진열장 {shelf_id} 도착. {hold_time}초 대기...")
                for _ in range(int(hold_time * 10)):
                    a = Bool()
                    a.data = True
                    self.detect_publisher.publish(a)
                    dest = Int8()
                    dest.data = shelf_id
                    self.destination.publish(dest)

                    if self.stop_event.is_set():
                        break
                    time.sleep(0.1)
                    ###########stb
                    # if self.object_num == 0:
                    #     self.get_logger().info("객체 분석중")
                    #     while not self.stop_event.is_set(): #mqtt_통신 결과
                    #         rclpy.spin_once(self, timeout_sec=0.2)  # 콜백 처리 + 대기
                    #         if not self.mqtt_result or self.object_num != 0:
                                
                    #             time.sleep(0.1)
                    #             break
                            
                        
                a.data = False
                self.detect_publisher.publish(a)
            

        # 순회 종료 후 대기 위치 복귀
        wait_pose = self.create_pose(*self.final_position)
        self.nav.goToPose(wait_pose)
        # 대기 위치 도착까지 기다리기
        while not self.nav.isTaskComplete():
            time.sleep(0.1)
        self.get_logger().info("도킹 시도 중...")
        if self.nav.dock():
            pass
        else:
            pass
        with self.lock:
            self.is_busy = False
        self.get_logger().info("순회를 종료하고 대기 위치로 복귀했습니다.")
    def wait_for_input_loop(self):
        last_cmd = ''   
        while rclpy.ok():
            time.sleep(0.1)
            cmd = self.cmd
             # 새 명령일 때만 처리
            if cmd == last_cmd or not cmd:
                continue
            last_cmd = cmd
            cmd_msg = String()
            cmd_msg.data = cmd
            self.close.publish(cmd_msg)
            if cmd.strip().lower() == 'q':
                if not self.is_busy:
                    # 순회 전에도 q 입력 시 도킹 후 종료
                    self.get_logger().info("종료 요청 감지 → 대기 장소로 이동 후 도킹 시도")
                    # stop_event를 먼저 set해서 순회 중이라면 즉시 중단
                    self.stop_event.set()
                    final_pose = self.create_pose(*self.final_position)
                    self.nav.goToPose(final_pose)
                    while not self.nav.isTaskComplete():
                        time.sleep(0.1)
                    if self.nav.dock():
                        self.get_logger().info("도킹 완료 후 종료")
                    else:
                        self.get_logger().warn("도킹 실패 후 종료")
                    rclpy.shutdown()
                    return
                else:
                    # 순회 중이라면 stop_event만 set
                    self.get_logger().info("순회 중단 요청 감지 → stop_event set")
                    self.stop_event.set()
                continue
            elif cmd.strip().lower() == 'a':
                with self.lock:
                    if self.is_busy:
                        self.get_logger().warn("이미 순회 중입니다.")
                        continue
                    self.is_busy = True
                    self.stop_event.clear()
                    patrol_thread = threading.Thread(target=self.patrol_shelves, daemon=True)
                    patrol_thread.start()
                continue

            # 3) 나머지 입력은 무시
            else:
                continue
            
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
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import time
import threading
import rclpy
from rclpy.node import Node

# ✅ 패키지 경로 통일 (중요)
import DR_init
from DR_common2 import posx, posj
from std_msgs.msg import Float32MultiArray, String, Bool
from rclpy.executors import SingleThreadedExecutor

# ---- 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30
OFF, ON = 0, 1

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# ---- DSR API를 동적으로 import (노드 주입 후에!)
def import_dsr_modules():
    global set_tool, set_tcp, movej, movel, wait, movejx, amovej
    global task_compliance_ctrl, set_desired_force, check_force_condition
    global release_force, release_compliance_ctrl
    global get_digital_input, set_digital_output
    global get_current_posx, get_current_posj, check_position_condition
    global amove_periodic, movesx, movesj, move_periodic, drl_script_stop
    global DR_FC_MOD_REL, DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z, DR_AXIS_A, DR_AXIS_B, DR_AXIS_C
    global DR_MV_MOD_ABS, DR_TOOL, DR_QSTOP, DR_BASE
    global set_robot_mode, get_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS

    from DSR_ROBOT2 import (
        set_tool, set_tcp, movej, movel, wait, movejx, amovej,
        task_compliance_ctrl, set_desired_force, check_force_condition,
        release_force, release_compliance_ctrl,
        get_digital_input, set_digital_output,
        get_current_posx, get_current_posj, check_position_condition,
        amove_periodic, movesx, movesj, move_periodic, drl_script_stop,
        DR_FC_MOD_REL, DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z, DR_AXIS_A, DR_AXIS_B, DR_AXIS_C,
        DR_MV_MOD_ABS, DR_TOOL, DR_QSTOP, DR_BASE,
        set_robot_mode, get_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    )


# -----------------------------------------------------------------------------
# 컨트롤러: rclpy.Node를 직접 상속하지 않고, 외부에서 생성된 node를 받아 사용
# -----------------------------------------------------------------------------
class PluckController:
    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger()

        # 상태
        self.sol = 0
        self.hz = None
        self.line_state = ""
        self.tuning_start = False
        self.tuning_end = False
        self.get_hz = False
        self.get_line = False
        self.valid_lines = ("G3","D4","A4","E5")
        self.active_line = ""     # 현재 튜닝 중인 줄
        self.did_initial = False  # 현재 active_line에 대해 초기 이동/플럭을 했는지
        self.retry = False        # hz 인식 기에서 hz_err를 Pub하지 않는 경우 다시 줄 튕기기

        # 퍼플리시
        self.pub_pluck = self.node.create_publisher(Bool, 'pluck_done', 10)

        # 구독 (필요없으면 주석처리 가능)
        self.node.create_subscription(Float32MultiArray, 'hz_value', self.on_hz, 10)
        self.node.create_subscription(String, 'line_state', self.on_line_state, 10)
        self.node.create_subscription(Bool, 'tunning_start', self.cb_start, 10)
        self.node.create_subscription(Bool, 'tunning_end', self.cb_end, 10)
        self.node.create_subscription(Bool, 'retry', self.cb_retry, 10)
        

        # 속도 기본값
        self.v_joint = VELOCITY
        self.a_joint = ACC
        self.deg_hz  = 0.0
        self.v_hz = 0.0
        self.a_hz = 0.0
        self.time_hz = 0.0

        self.K_ANGLE   = [1.0,1.0, 1.0, 1.0]   # [deg/Hz]
        self.ANGLE_MIN = 5.0   # [deg]
        self.ANGLE_MAX = 30.0    # [deg]

        self.K_VEL   = 1.0      # [deg/s per Hz]
        self.VEL_MIN = 3.0      # [deg/s]
        self.VEL_MAX = 30.0     # [deg/s]

        self.K_ACC   = 1.0     # [deg/s^2 per Hz] (원하는 값으로 조정)
        self.ACC_MIN = 3.0     # [deg/s^2]
        self.ACC_MAX = 30.0    # [deg/s^2]

        self.TIME_MIN = 3.0     # [s]
        self.TIME_MAX = 5.0     # [s]
        
        # 포즈
        self.home = posj([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # 줄 위치
        self.string_pos = {
            "G3": posj([-5.94374990e+00,  5.22824173e+01,  2.12601242e+01, -1.00702964e-01, 1.06431801e+02, -9.53369446e+01]),
            "D4": posj([-3.74980259,  52.00120926,  19.62848854,  -0.26566568, 108.83249664, -94.17108154]),
            "A4": posj([-2.13562489e+00,  5.52894020e+01,  1.41101236e+01, -1.03258930e-01, 1.10654922e+02, -9.26544113e+01]),
            "E5": posj([  4.66657877,  51.01257706,  23.3678093 , -13.34218788, 106.11769104, -91.07872009]),
        }   

        # 페그 초기 위치
        self.peg_initial_pos = {
            "G3": posx([352.422, -68.903, 159.011, 89.279, 89.991, -64.992]),
            "D4": posx([312.241, -67.474, 151.978, 88.509, 89.148, -117.256]),
            "A4": posx([303.376, 56.695, 140.411, 92.360, -89.021, 96.600]),
            "E5": posx([336.318, 57.743, 153.645, 92.418, -89.331, 86.872]),
        }
        
        # 연주 위치
        self.play_bowing_pos = {
            "G3": posj([  4.66499996,  48.25497437,  18.26940155,  18.27683258,105.48226166,  17.34959412]),
            "D4": posj([11.91256618, 45.57639313, 30.44005013,  1.67233956, 97.20911407,16.70033073]),
            "A4": posj([ 26.93003273,  57.98439407,  21.27257729, -18.66949654, 102.08357239,  23.4121151 ]),
            "E5": posj([23.69565773,  51.03977203,  39.02797699, -31.71795082,98.26450348,  16.66594696]),
        }
        

        # 중간 경유위치
        self.string_left_pos = posj([-15.23, 48.10, 18.22, -0.10, 113.67, -86.53])
        self.string_right_pos = posj([13.28, 54.65, 14.08, -0.10, 109.63, -60.93])
        self.pluck_end_pos = posj([-1.21736836e+00,  2.23665543e+01,  5.43280563e+01, -6.26955703e-02, 1.03305428e+02, -9.20945740e+01])
        self.bow_pos = posj([ 3.91683540e+01,  7.21440315e+00,  1.07559883e+02, -2.39331387e-02, 6.53145828e+01, -4.98668060e+01])
        self.peg_wait_pos = posj([ 6.64144754e-01,  1.98868027e+01,  4.61496849e+01, -1.44893503e+00, 1.14353416e+02,  2.25135195e-03])
        self.d4_before=posj([ 6.68407917, 40.59456253, 37.73485565,  2.09959912, 93.68097687,11.47840595])
        # 초기 설정
        self.set_manual_mode()
        set_tool("Tool Weight")
        set_tcp("tcp01")
        self.set_auto_mode()
        self.release()
        self.move_home()

        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()
        self.logger.info("PluckController ready (run-loop worker started).")
    
    # ------------------ 전체 시퀀스 동작 run ------------------

    def run(self):
        while True:
            rclpy.spin_once(self.node)
            time.sleep(0.03)

            if not self.tuning_start:
                self.logger.info("Not Start")
                continue
            
            # (A) 라인이 들어온 후 -> 그것이 새로운 줄인 경우 did_initial = False
            if self.get_line and self.line_state in self.valid_lines:
                self.get_line = False
                if self.active_line != self.line_state:
                    self.active_line = self.line_state
                    self.did_initial = False   # ✅ 새 라인 → 초기부터 다시
                    self.logger.info(f"Active_line : {self.active_line}")

            # (B) 새로운 줄을 받은 경우 그 줄을 pluck -> did_innitail = True
            if not self.did_initial and self.active_line in self.valid_lines:
                self.logger.info("New line Plucking")
                line = self.active_line  # 스냅샷
                self.move_to_string(line)
                self.pluck_string(line)
                self.did_initial = True
                continue  # 초기 끝났으면 다음 루프에서 Hz 대기

            # (C) hz가 안들어 온 경우 retry
            if self.retry:
                self.retry = False
                self.logger.info("Retry line Plucking")
                line = self.active_line  # 스냅샷
                self.move_to_string(line)
                self.pluck_string(line)
                continue
            
            # (D) hz가 들어온 경우 peg 조이고 줄을 튕긴다. 
            if self.get_hz and self.active_line in self.valid_lines:
                self.logger.info("Tunning Current String")
                self.get_hz = False
                line = self.active_line      # 스냅샷(도중 변경 방지)
                deg  = self.deg_hz           # on_hz에서 계산됨

                if deg != 0.0:
                    self.tune_peg(line, approach=30.0, deg=deg, sol=2)
                    self.move_to_string(line)
                    self.pluck_string(line)
                continue
            
            if self.tuning_end:
                self.tuning_end = False
                self.pick_bow()
                self.play_bowing()
                self.release_bow()
                self.move_home()
                





    # ------------------ 모드 전환 ------------------
    def set_manual_mode(self) -> bool:
        J = get_current_posj()
        movej(J, vel=5, acc=5)
        set_robot_mode(ROBOT_MODE_MANUAL)
        ok = (get_robot_mode() == ROBOT_MODE_MANUAL)
        if not ok:
            self.logger.warn("Manual Mode Change Failed")
        return ok

    def set_auto_mode(self) -> bool:
        J = get_current_posj()
        movej(J, vel=5, acc=5)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        ok = (get_robot_mode() == ROBOT_MODE_AUTONOMOUS)
        if not ok:
            self.logger.warn("Auto Mode Change Failed")
        return ok

    # ------------------ Gripper I/O ------------------
    def wait_digital_input(self, sig_num: int, poll_s: float = 0.5):
        while not get_digital_input(sig_num):
            time.sleep(poll_s)

    def grip(self):
        # DO1=ON, DO2=OFF
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        self.wait_digital_input(1)

    def release(self):
        # DO2=ON, DO1=OFF
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        self.wait_digital_input(2)
    
    # ------------------ Move 래퍼 ------------------
    def move_joint(self, pose, vel=VELOCITY, acc=ACC, time=None, mod=None):
        if mod is None:
            movej(pose, vel=vel, acc=acc, time=time, mod=DR_MV_MOD_ABS)
        else:
            movej(pose, vel=vel, acc=acc, time=time, mod=mod)

    def move_linear(self, pose, vel=VELOCITY, acc=ACC, time=None, ref=None, mod=None):
        if mod is None:
            movel(pose, vel=vel, acc=acc, time=time, ref=ref, mod=DR_MV_MOD_ABS)
        else:
            movel(pose, vel=vel, acc=acc, time=time, ref=ref, mod=mod)

    # ------------------ 유틸 ------------------
    def pose_trans(self, pose, delta_pose):
        return [x + y for x, y in zip(list(pose), list(delta_pose) + [0]*(len(pose)-len(delta_pose)))]

    #------------------ 홈 ------------------
    def move_home(self):
        self.logger.info("Moving to home position...")
        self.move_joint(self.home, vel=self.v_joint, acc=self.a_joint)

    #------------------ 페그 회전 ------------------
    def rotate_peg(self, delta_deg: float, vj=None, aj=None, time = None):
        self.logger.info(f"Rotating peg by {delta_deg:.2f} deg...")
        j = get_current_posj()
        j[5] += delta_deg  # Doosan이 deg라면 그대로, rad면 math.degrees/radians 조정
        self.move_joint(posj(j), vel=vj, acc=aj, time=time)

    # --------------- 페그 이동 및 튜닝 ---------------
    def tune_peg(self, peg_name: str, approach: float = 20.0, deg: float = 0.0, sol: int = 0):
        """
        1) peg_initial_pos[peg]로 movejx
        2) TOOL Z축으로 +approach 만큼 접근 (ref=DR_TOOL)  ← 방향은 장비에 맞게 부호 조정
        3) grip -> J6 회전 -> release
        4) TOOL Z축으로 -approach 만큼 복귀
        5) 현재 posx를 읽어 peg_initial_pos[peg] 갱신
        """
        if peg_name not in self.peg_initial_pos:
            self.logger.error(f"[{peg_name}] peg_initial_pos key not found.")
            return False

        self.logger.info(f"[{peg_name}] Step1: movejx -> approach (sol={sol})")
        movejx(pos=self.peg_initial_pos[peg_name], vel=VELOCITY, acc=ACC, sol=sol)

        self.logger.info(f"[{peg_name}] Step2: movel -> TOOL Z (dz=+{approach})")
        self.move_linear([0, 0, +abs(approach), 0, 0, 0], ref=DR_TOOL)  # 필요시 부호 반대로

        self.logger.info(f"[{peg_name}] Step3: grip -> rotate({deg}) -> release")
        self.grip()

        if abs(deg) > 0.0:
            self.rotate_peg(deg, self.v_hz, self.a_hz, self.time_hz)
        self.release()

        self.logger.info(f"[{peg_name}] Step4: movel -> back (dz=-{approach})")
        self.move_linear([0, 0, -abs(approach), 0, 0, 0], ref=DR_TOOL)

        self.peg_pos, self.sol = get_current_posx() # 현재 페그 어프로치 위치 및 자세 읽기 
        self.peg_initial_pos[peg_name] = self.peg_pos # 페그 어프로치 위치 업데이트
    
    # --------------- 줄에 접근 ---------------
    def move_to_string(self, line_state):
        self.logger.info("Moving to string position...")
        if line_state in ["G3", "D4", "A4", "E5"]:
            if line_state == "G3":
                self.logger.info(f"target : {line_state}")
                self.move_joint(self.string_left_pos)
                self.move_joint(self.string_pos["G3"])

            elif line_state == "D4":
                self.logger.info(f"target : {line_state}")
                self.move_joint(self.string_left_pos)
                self.move_joint(self.string_pos["D4"])  

            elif line_state == "A4":
                self.logger.info(f"target : {line_state}")
                self.move_joint(self.string_right_pos)
                self.move_joint(self.string_pos["A4"]) 
            
            elif line_state == "E5":
                self.logger.info(f"target : {line_state}")
                self.move_joint(self.string_right_pos)
                self.move_joint(self.string_pos["E5"]) 

    # ------------------ 줄 튕기기 ------------------
    def pluck_string(self, line):
        self.logger.info("Plucking the string...")
        if line == "G3":
            self.logger.info(f"Plucking {line}")
            self.move_linear([-23.0, 0.0, -3.0, 0.0, 0.0, 0.0], ref= DR_TOOL)
            self.move_linear([ 0.0, 0.0, -10.0, 0.0, 0.0, 0.0], ref= DR_TOOL)
            self.move_joint(self.string_left_pos)
            #self.move_home()

        elif line == "D4":
            self.logger.info(f"Plucking {line}")
            self.move_linear([0.0, 0.0, 6.0, 0.0, 0.0, 0.0], ref= DR_TOOL)
            self.move_linear([-10.0, 0.0, 0.0, 0.0, 0.0, 0.0], ref= DR_TOOL)
            self.move_linear([ 0.0, 0.0, -10.0, 0.0, 0.0, 0.0], ref= DR_TOOL)
            self.move_joint(self.string_left_pos)
            #self.move_home()
        
        elif line == "A4":
            self.logger.info(f"Plucking {line}")
            self.move_linear([0.0, 0.0, 3.0, 0.0, 0.0, 0.0], ref= DR_TOOL)
            self.move_linear([10.0, 0.0, 0.0, 0.0, 0.0, 0.0], ref= DR_TOOL)
            self.move_linear([ 0.0, 0.0, -10.0, 0.0, 0.0, 0.0], ref= DR_TOOL)
            self.move_joint(self.string_right_pos)
            #self.move_home()

        elif line == "E5":
            self.logger.info(f"Plucking {line}")
            self.move_linear([0.0, 0.0, 2.3, 0.0, 0.0, 0.0], ref= DR_TOOL)
            self.move_linear([-10.0, 0.0, 0.0, 0.0, 0.0, 0.0], ref= DR_TOOL)
            self.move_linear([ 0.0, 0.0, -15.0, 0.0, 0.0, 0.0], ref= DR_TOOL)
            self.move_joint(self.string_right_pos)
            #self.move_home()

        self.pub_pluck.publish(Bool(data=True))
            

    # ------------------ 활 잡기 ------------------
    def pick_bow(self):
        self.logger.info("Picking up the bow...")
        self.move_joint(self.bow_pos)
        self.move_linear([0.0, 0.0, 30.0, 0.0, 0.0, 0.0], ref=DR_TOOL)
        self.grip()
        self.move_linear([0.0, 0.0, -200.0, 0.0, 0.0, 0.0], ref=DR_TOOL)


    def release_bow(self):
        self.logger.info("Releasing the bow...")
        self.move_joint([2.48753285e+00,  2.96678295e+01,  3.10789547e+01, -8.99472088e-02,1.19257904e+02,  6.21740520e-03])
        self.move_joint([ 3.91704292e+01,  4.63152933e+00,  1.02116608e+02, -2.41642557e-02, 7.33433762e+01, -4.98693237e+01])
        self.move_linear([0.0, 0.0, 78.0, 0.0, 0.0, 0.0], ref=DR_TOOL)
        self.release()
        self.move_linear([0.0, 0.0, -50.0, 0.0, 0.0, 0.0], ref=DR_TOOL)

    # ------------------ 바이올린 연주하기 ------------------
    def play_bowing(self):
        self.logger.info("Playing with the bow...")
        # -------------------------- A4 -------------------------------
        self.move_joint(self.play_bowing_pos["A4"])
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        time.sleep(0.5)

        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0],mod =DR_FC_MOD_REL)

        while check_force_condition(DR_AXIS_Z, max=3) != -1:
            print("Waiting for an external force greater than 5 ")
            time.sleep(0.5)
            pass
        
        release_force()
        time.sleep(0.5)
          
        release_compliance_ctrl()
        example_amp = [0.0, 30, 0.0, 0.0, 0.0, 0.0]
        period = [0.0, 0.7, 0.0, 0.0, 0.0, 0.0]
        self.move_linear([0.0, -120.0, 1.0, 0.0, 0.0, 0.0], vel=150, acc=100, ref=DR_TOOL)
        move_periodic(amp=example_amp, period=period, atime=0.2, repeat=4, ref=DR_TOOL)
        
        # -------------------------- E5 -------------------------------
        self.move_joint(self.play_bowing_pos["E5"],vel=VELOCITY, acc=ACC)
        # task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        # time.sleep(0.5)

        # set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0],mod =DR_FC_MOD_REL)

        # while check_force_condition(DR_AXIS_Z, max=3) != -1:
        #     print("Waiting for an external force greater than 3 ")
        #     time.sleep(0.5)
        #     pass
        
        # release_force()
        # time.sleep(0.5)
             
        # release_compliance_ctrl()
        example_amp = [0.0, 30, 0.0, 0.0, 0.0, 0.0]
        period = [0.0, 0.7, 0.0, 0.0, 0.0, 0.0]
        move_periodic(amp=example_amp, period=period, atime=0.2, repeat=4, ref=DR_TOOL)
        
        # -------------------------- D4  -------------------------------
        self.move_linear([0.0, 0.0, -20.0, 0.0, 0.0, 0.0], vel=150, acc=100, ref=DR_TOOL)
        self.move_joint(self.d4_before,vel=VELOCITY, acc=ACC)
        self.move_joint(self.play_bowing_pos["D4"])
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        time.sleep(0.5)

        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0],mod =DR_FC_MOD_REL)

        while check_force_condition(DR_AXIS_Z, max=3) != -1:
            print("Waiting for an external force greater than 5 ")
            time.sleep(0.5)
            pass

        release_force()
        time.sleep(0.5)
          
        release_compliance_ctrl()
        example_amp = [0.0, 30, 0.0, 0.0, 0.0, 0.0]
        period = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]
        self.move_linear([0.0, -60.0, -1.0, 0.0, 0.0, 0.0], vel=150, acc=100, ref=DR_TOOL)
        move_periodic(amp=example_amp, period=period, atime=0.2, repeat=4, ref=DR_TOOL)
        
        # -------------------------- G3 -------------------------------
        self.move_linear([0.0, 0.0, -20.0, 0.0, 0.0, 0.0], vel=150, acc=100, ref=DR_TOOL)
        self.move_joint(self.play_bowing_pos["G3"])

        self.move_linear([0.0, 0.0, 5.0, 0.0, 0.0, 0.0], vel=150, acc=100, ref=DR_TOOL)
        self.move_linear([0.0, -100.0, 7.0, 0.0, 0.0, 0.0], vel=150, acc=100, ref=DR_TOOL)


    # ------------------ calculate deg / vel / acc /time ------------------
    def cal_rotate_param(self, sig: float, hz_err: float):
        # 로그
        if sig == 1.0:
            self.logger.info(f"sig:{sig}, hz_err:{hz_err:.3f} → Hz 올리기 지시")
        
        elif sig == 2.0:
            self.logger.info(f"sig:{sig}, hz_err:{hz_err:.3f} → Hz 내리기 지시")
       
        elif sig == 3.0:
            self.logger.info(f"sig:{sig} → 튜닝 정지")
            self.deg_hz  = 0.0
            self.v_hz    = 0.0
            self.a_hz    = 0.0
            self.time_hz = 0.0
            return self.deg_hz, self.v_hz, self.a_hz, self.time_hz
        
        else:
            self.logger.warn(f"Unknown sig: {sig}")

        e = abs(float(hz_err))

        # 1) 각도 크기 (선형 + 클램프)
        if self.line_state == "G3":
            angle_mag = max(self.ANGLE_MIN, min(self.ANGLE_MAX, self.K_ANGLE[0] * e))

        elif self.line_state == "D4":
            angle_mag = max(self.ANGLE_MIN, min(self.ANGLE_MAX, self.K_ANGLE[1] * e))
        
        elif self.line_state == "A4":
            angle_mag = max(self.ANGLE_MIN, min(self.ANGLE_MAX, self.K_ANGLE[2] * e))

        elif self.line_state == "E5": 
            angle_mag = max(self.ANGLE_MIN, min(self.ANGLE_MAX, self.K_ANGLE[3] * e))

        # 2) 속도 (선형 + 클램프)
        v = max(self.VEL_MIN, min(self.VEL_MAX, self.K_VEL * e))

        # 3) 가속 (선형 + 클램프)
        a = max(self.ACC_MIN, min(self.ACC_MAX, self.K_ACC * e))

        # 4) 시간: angle / v 를 기본으로, min/max 클램프
        t = angle_mag / max(v, 1e-6)
        t = max(self.TIME_MIN, min(self.TIME_MAX, t))
        
        # 5) 부호 결정
        if self.line_state in ("G3", "D4"):
            inc_dir = -1
        elif self.line_state in ("A4", "E5"):
            inc_dir = +1
        
        if sig == 1.0:
            sig_sign = 1
        elif sig == 2.0:
            sig_sign = -1
        else:
            sig_sign = 0

        signed_angle = angle_mag * inc_dir * sig_sign 

        # 저장 & 리턴
        self.deg_hz  = signed_angle
        self.v_hz    = v
        self.a_hz    = a
        self.time_hz = t
        self.logger.info(f'deg : {self.deg_hz}\nv : {self.v_hz}n\a : {self.a_hz}n\time : {self.time_hz}')
        return self.deg_hz, self.v_hz, self.a_hz, self.time_hz

    
    # --------------- 콜백 (옵션) ---------------
    def on_hz(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) == 2:
            sig = data[0]  # 1.0 : 올려라, 2.0 : 내려라, 3.0 : 멈춰라
            hz_err = data[1]
            self.cal_rotate_param(sig, hz_err)
            self.get_hz = True
            self.logger.info(f"Get_Hz")

        else:
            self.logger.warn(f"hz_value expects 2 elems, got {len(data)}")
            self.get_hz = False

    def on_line_state(self, msg: String):
        if msg.data in ["G3", "D4", "A4", "E5"]:
            self.line_state = msg.data
            self.logger.info(f"[line] {self.line_state}")
            self.get_line = True

        else:
            self.logger.warn(f"line_state_value is not correct, got {msg.data}")
            self.get_line = False

    def cb_retry(self, msg: Bool):
        self.retry = msg.data
        self.logger.info("Retry")

    def cb_start(self, msg: Bool):
        self.tuning_start = msg.data
        self.logger.info("Tuning_start")
    
    def cb_end(self, msg: Bool):
        self.tuning_end = msg.data
        self.logger.info("Tuning_end")

        


# -----------------------------------------------------------------------------
# main: 노드 생성 → DR_init에 주입 → (구버전 대비) g_node도 같이 주입 → DSR 모듈 import → 컨트롤러 생성
# -----------------------------------------------------------------------------
def main():
    rclpy.init()

    # ✅ rclpy 노드 먼저 생성
    node = rclpy.create_node('pluck_controller', namespace=ROBOT_ID)

    # ✅ DSR에 같은 모듈 객체로 노드 주입 (중요)
    DR_init.__dsr__node = node

    # ✅ 이제야 DSR_ROBOT2 import
    import_dsr_modules()

    ctrl = PluckController(node)
    ex = SingleThreadedExecutor()  # ROS2 콜백 처리 실행자
    ex.add_node(node)
    ex.spin()

    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

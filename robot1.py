# app.py
import streamlit as st
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import subprocess

# -----------------------------
# ROS 퍼블리셔 노드 (단일 파일)
# -----------------------------
class UICommander(Node):
    def __init__(self):
        super().__init__('ui_commander')
        self.pub_close = self.create_publisher(String, '/close_from_topic', 10)

    def send_enter(self):
        msg = String(); msg.data = ""     # 엔터 == 빈 문자열
        self.pub_close.publish(msg)

    def send_e(self):
        msg = String(); msg.data = "e"
        self.pub_close.publish(msg)

    def send_q(self):
        msg = String(); msg.data = "q"
        self.pub_close.publish(msg)

# -----------------------------
# ROS 컨텍스트 1회만 초기화
# -----------------------------
def init_ros_once():
    if 'ros_inited' not in st.session_state:
        try:
            if not rclpy.ok():
                rclpy.init()
            st.session_state['ui_node'] = UICommander()
            st.session_state['ros_inited'] = True
            st.session_state['last_cmd'] = '없음'
            st.success("✅ ROS 2 연결 완료!")
        except RuntimeError as e:
            # 이미 초기화된 경우 등 예외 처리
            st.warning(f"⚠️ ROS2 초기화 경고: {e}")
            # 노드만 재생성 시도
            st.session_state['ui_node'] = UICommander()
            st.session_state['ros_inited'] = True

init_ros_once()
ui_node: UICommander = st.session_state['ui_node']

# -----------------------------
# Streamlit UI
# -----------------------------
st.title("🛒 TurtleBot 제어 (Enter / e / q)")
st.markdown("진열장 시스템용 **간단 제어판**입니다. 아래 버튼으로 `/close_topic` 에 신호를 보냅니다.")

# 버튼 3개 (Enter / e / q)
col1, col2, col3 = st.columns(3)
with col1:
    if st.button("▶ 시작 (Enter)"):
        ui_node.send_enter()
        st.success("엔터(빈 문자열) 신호 전송 완료!")
        st.session_state['last_cmd'] = "Enter(빈 문자열)"

with col2:
    if st.button("🟠 e 보내기"):
        ui_node.send_e()
        st.info("e 신호 전송 완료! (예: 뷰어 종료 트리거)")
        st.session_state['last_cmd'] = "e"

with col3:
    if st.button("🛑 종료 (q)"):
        ui_node.send_q()
        st.warning("q 신호 전송 완료! (순회 정지/복귀 등)")
        st.session_state['last_cmd'] = "q"

st.markdown("---")
st.subheader("📋 시스템 상태")
st.write("**최근 명령:**", st.session_state.get('last_cmd', '없음'))
st.write("**현재 시각:**", time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))

# (선택) ROS 노드 상태 확인
with st.expander("📡 ROS 노드 상태 확인 (옵션)"):
    if st.button("상태 점검 실행"):
        result = subprocess.getoutput(
            "for node in $(ros2 lifecycle nodes 2>/dev/null); do echo -n \"$node: \"; "
            "timeout 2s ros2 lifecycle get $node || echo \"응답 없음\"; done"
        )
        st.code(result)

st.caption("실행: `streamlit run app.py`")


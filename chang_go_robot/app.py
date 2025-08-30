# app.py
import streamlit as st
import rclpy
from ros_interface.shelf_publisher import ShelfPublisher  # 🧩 진열장 + 출발 신호 모두 퍼블리시
import time
import subprocess

# ✅ 수정됨: ROS 노드 초기화 (최초 1회만 실행)
if 'rclpy_initialized' not in st.session_state:
    rclpy.init()
    st.session_state['shelf_node'] = ShelfPublisher()
    st.session_state['rclpy_initialized'] = True
    st.success("✅ ROS 2 연결 완료!")

shelf_node = st.session_state['shelf_node']

# UI 타이틀
st.title("🛒 TurtleBot 진열장 재고 보충 시스템")
st.markdown("진열장 보충 요청 또는 다음 단계 출발 명령을 전송할 수 있습니다.")

# 진열장 선택 UI
shelf_dict = {
    0: "포키",
    1: "레쓰비",
    2: "초코송이",
    3: "씨리얼"
}

selected_shelf = st.selectbox(
    "진열장 선택",
    options=list(shelf_dict.keys()),
    format_func=lambda x: f"{x}번 - {shelf_dict[x]}"
)

# 진열장 보충 요청 버튼
if st.button("🚀 진열장 보충 요청 보내기"):
    shelf_node.publish_shelf_id(selected_shelf)
    st.success(f"{selected_shelf}번 진열장 ({shelf_dict[selected_shelf]}) 보충 요청을 전송했습니다.")
    st.session_state['last_command'] = f"{selected_shelf}번 진열장"

# ✅ 수정됨: 출발 명령 UI 추가
st.markdown("---")
st.subheader("▶ 출발 명령 전송 (로봇이 대기 중일 때만 동작)")

if st.button("📤 다음 단계로 출발"):
    shelf_node.publish_start_signal()  # ✅ 수정됨: 출발 신호 퍼블리시
    st.success("출발 신호를 전송했습니다. 로봇이 다음 단계로 이동합니다.")
    st.session_state['last_command'] = "출발 신호 전송"

# 시스템 상태 출력
st.markdown("---")
st.subheader("📋 시스템 상태")
st.write("**최근 명령:**", st.session_state.get('last_command', '없음'))
st.write("**현재 시각:**", time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))

# ROS 노드 상태 확인 버튼
if st.button("📡 ROS 노드 상태 확인"):
    result = subprocess.getoutput(
        "for node in $(ros2 lifecycle nodes); do echo -n \"$node: \"; timeout 2s ros2 lifecycle get $node || echo \"응답 없음\"; done"
    )
    st.code(result)
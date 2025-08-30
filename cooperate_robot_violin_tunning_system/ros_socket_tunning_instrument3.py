#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node

import numpy as np
import sounddevice as sd
import aubio
import math
import queue
import sys
import time
import threading
import json
import asyncio
import websockets
from collections import deque
from std_msgs.msg import Float32MultiArray, String, Bool

# 맨 위 import 근처 
#%#%
from collections import deque, Counter  # ← Counter 추가

# =========================
# 기본 설정(파라미터로 덮어쓰기 가능)
# =========================
SAMPLE_RATE   = 48000
HOP_SIZE      = 1024
BUFFER_SIZE   = 8
CONF_MIN      = 0.90
MIN_FREQ      = 40.0
A4_REF        = 440.0
SMOOTH_N      = 5
TOL_HZ        = 5.0
SAMPLES_PER_SEC = 20
N_SAMPLES_PER_STRING = 200

STR_ORDER = ["G3", "D4", "A4", "E5"]
KEY_TO_STR = {"g":"G3", "d":"D4", "a":"A4", "e":"E5"}

NOTE_NAMES = ['C','C#','D','D#','E','F','F#','G','G#','A','A#','B']


HARM_TOL = 0.12
SUB_TOL  = 0.08

# 허용 센트(절대값). k가 클수록 엄격
HARM_TOL_CENTS = {2: 20, 3: 15, 4: 12, 5: 10, 6: 8, 7: 8, 8: 7}
SUB_TOL_CENTS  = 15  # 1/2배(서브고조파) 허용치

def fold_harmonic_to_base(f_meas: float, base_hz: float):
    """측정 f를 기본음으로 접는다. 2~8배 고조파와 1/2배 서브고조파를 센트 기반으로 안정적으로 보정."""
    if f_meas <= 0 or base_hz <= 0:
        return f_meas, ''
    ratio = f_meas / base_hz

    # 기본값(접지 않음)
    best_err = abs(f_meas - base_hz)
    best_f   = f_meas
    best_tag = ''

    # k배 고조파(2..8) 접기
    for k in (2, 3, 4, 5, 6, 7, 8):
        cents = 1200.0 * math.log2(ratio / k)
        if abs(cents) <= HARM_TOL_CENTS.get(k, 8):
            cand = f_meas / k
            err  = abs(cand - base_hz)
            if err < best_err:
                best_err = err
                best_f   = cand
                best_tag = f"H{k}"

    # 1/2배(서브고조파) 접기
    cents_sub = 1200.0 * math.log2(ratio / 0.5)
    if abs(cents_sub) <= SUB_TOL_CENTS:
        cand = f_meas * 2.0
        err  = abs(cand - base_hz)
        if err < best_err:
            best_err = err
            best_f   = cand
            best_tag = "S2"

    return best_f, best_tag


def snap_octave_to_target(f_meas: float, base_hz: float, band=(0.70, 1.40)):
    """f, f/2, f*2 후보 중 타깃(base)에 합리적으로 스냅.
    - f가 base보다 많이(>=15%) 높고 f/2가 base보다 확실히(>=5%) 낮으면: 옥타브 할빙 우선.
    - 그 외에는 밴드(0.70~1.40×) 안 후보를 우선, 그다음 오차 최소로 선택.
    """
    if f_meas <= 0 or base_hz <= 0:
        return f_meas, ''
    lo = base_hz * band[0]
    hi = base_hz * band[1]

    half = f_meas * 0.5
    # --- 옥타브 할빙 우선 휴리스틱 ---
    # 예: base=196, f=240 → f/base=1.22(>1.15) 이고 half/base=0.61(<0.95) → 120Hz 선택
    if (f_meas >= base_hz * 1.15) and (half <= base_hz * 0.95):
        return half, 'HALVE'

    # --- 기본 스냅: 밴드 안 우선 + 오차 최소 ---
    best = None
    for k, tag in ((1.0, ''), (0.5, 'HALVE'), (2.0, 'DOUBLE')):
        cand = f_meas * k
        in_band = (lo <= cand <= hi)
        score = abs(cand - base_hz)
        key = (not in_band, score)
        if best is None or key < best[0]:
            best = (key, cand, tag)
    return best[1], best[2]


def violin_string_freqs(a4):
    E5 = a4 * (2 ** ( 7/12))
    D4 = a4 * (2 ** (-7/12))
    G3 = a4 * (2 ** (-14/12))
    A4 = a4
    return {"G3": G3, "D4": D4, "A4": A4, "E5": E5}

# =========================
# 웹소켓 (브로드캐스트 + 명령 수신)
# =========================
class WSRelay:
    def __init__(self, host: str = "0.0.0.0", port: int = 8765):
        self.host = host
        self.port = port
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.clients = set()
        self.out_queue = asyncio.Queue()   # 브로드캐스트
        self.cmd_queue = queue.Queue()     # 노드 스레드에서 읽는 명령
        self._server = None
        self.thread.start()

    async def _handler(self, websocket):
        self.clients.add(websocket)
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    self.cmd_queue.put_nowait(data)
                except Exception:
                    pass
        finally:
            self.clients.discard(websocket)

    async def _start_server(self):
        self._server = await websockets.serve(self._handler, self.host, self.port)
        asyncio.create_task(self._broadcast())

    async def _broadcast(self):
        while True:
            msg = await self.out_queue.get()
            if not self.clients:
                continue
            dead = []
            for ws in list(self.clients):
                try:
                    await ws.send(msg)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                self.clients.discard(ws)

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._start_server())
        self.loop.run_forever()

    def send(self, obj: dict):
        try:
            msg = json.dumps(obj, ensure_ascii=False)
        except Exception:
            return
        if self.loop.is_running():
            self.loop.call_soon_threadsafe(self.out_queue.put_nowait, msg)

    def poll_cmd(self, timeout=0.05):
        try:
            return self.cmd_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def stop(self):
        if not self.loop.is_running():
            return
        def _shutdown():
            for ws in list(self.clients):
                try:
                    asyncio.create_task(ws.close())
                except Exception:
                    pass
            if self._server:
                self._server.close()
            self.loop.stop()
        self.loop.call_soon_threadsafe(_shutdown)
        self.thread.join(timeout=1.0)

# =========================
# 메인 노드
# =========================
class ViolinTunerNode(Node):
    def __init__(self):
        super().__init__('violin_tuner_node')
        self.STREAK_N = 12
        self._streak_kind = 0
        self._streak_count = 0
        self._stop_event = threading.Event()
        self._quit_requested = threading.Event()
        self._stop_requested = threading.Event()
        self._worker = None
        self._worker_lock = threading.Lock()
        self.pluck_ok = False
        self._last_hz_pub_ts = 0.0           # 마지막 hz_value publish 시간 (monotonic)
        self._hz_recent = False              # 최근 10초 내 퍼블리시 여부
        self.hz_recent_pub = self.create_publisher(Bool, '/dsr01/hz_recent', 5)

        # 상태를 주기적으로 갱신/퍼블리시하는 타이머 (0.2초 간격)
        self.create_timer(0.2, self._tick_hz_recent)


        # 모드 상태
        self.mode = None  # "auto" | "manual"
        self._session_abort = threading.Event()

        # 웹소켓
        self.ws = WSRelay(host="0.0.0.0", port=8765)

        # ROS 퍼블리셔
        self.hz_publisher = self.create_publisher(Float32MultiArray, '/dsr01/hz_value', 10)
        self.line_publisher = self.create_publisher(String, '/dsr01/line_state', 10)
        self.robot_active_pub = self.create_publisher(Bool, '/dsr01/tunning_start', 10)  # << True/False
        self.pluck_subscriber = self.create_subscription(Bool, "/dsr01/pluck_done", self.pluck_state, 10)
        self.retry_publisher = self.create_publisher(Bool, "/dsr01/retry", 10)
        self.tunning_end_publisher = self.create_publisher(Bool, "/dsr01/tunning_end",10)

        # 파라미터
        #%#%
        self.declare_parameter('unique_window_sec', 15.0)
        self.declare_parameter('unique_bin_hz', 1.0)
        #%#%
        self.declare_parameter('sample_rate', SAMPLE_RATE)
        self.declare_parameter('hop_size', HOP_SIZE)
        self.declare_parameter('buffer_size', BUFFER_SIZE)
        self.declare_parameter('conf_min', CONF_MIN)
        self.declare_parameter('min_freq', MIN_FREQ)
        self.declare_parameter('a4_ref', A4_REF)
        self.declare_parameter('smooth_n', SMOOTH_N)
        self.declare_parameter('tol_hz', TOL_HZ)
        self.declare_parameter('samples_per_sec', SAMPLES_PER_SEC)
        self.declare_parameter('n_samples_per_string', N_SAMPLES_PER_STRING)
        self.declare_parameter('cooldown_sec', 2.0)
        self.declare_parameter('hz_recent_window_sec', 10.0)   # 최근 판정 창 (초)
        self.declare_parameter('retry_pulse_sec', 0.15)        # retry 펄스 길이 (초)

        self.HZ_RECENT_WINDOW_SEC = float(self.get_parameter('hz_recent_window_sec').value)
        self.RETRY_PULSE_SEC      = float(self.get_parameter('retry_pulse_sec').value)


        self.SAMPLE_RATE   = int(self.get_parameter('sample_rate').value)
        self.HOP_SIZE      = int(self.get_parameter('hop_size').value)
        self.BUFFER_SIZE   = int(self.get_parameter('buffer_size').value)
        self.CONF_MIN      = float(self.get_parameter('conf_min').value)
        self.MIN_FREQ      = float(self.get_parameter('min_freq').value)
        self.a4_ref        = float(self.get_parameter('a4_ref').value)
        self.SMOOTH_N      = int(self.get_parameter('smooth_n').value)
        self.TOL_HZ        = float(self.get_parameter('tol_hz').value)
        self.SAMPLES_PER_SEC = int(self.get_parameter('samples_per_sec').value)
        self.N_SAMPLES_PER_STRING = int(self.get_parameter('n_samples_per_string').value)
        self.COOLDOWN_SEC = float(self.get_parameter('cooldown_sec').value)
        self.EMIT_INTERVAL = 1.0 / self.SAMPLES_PER_SEC
        #%#%
        self.UNIQUE_WINDOW_SEC = float(self.get_parameter('unique_window_sec').value)
        self.UNIQUE_BIN_HZ     = float(self.get_parameter('unique_bin_hz').value)
        # 15초 창 내 고유 Hz 집계 구조
        self._freq_lock   = threading.Lock()
        self._freq_counts = Counter()   # {양자화된 Hz: 개수}
        self._win_started = time.monotonic()
        # 15초마다 flush 타이머
        self.create_timer(self.UNIQUE_WINDOW_SEC, self._flush_unique_window)
        #%#%
        # 오디오/피치
        self.q = queue.Queue(maxsize=self.BUFFER_SIZE)
        self.pitch_o = aubio.pitch("yin", 2048, self.HOP_SIZE, self.SAMPLE_RATE)
        self.pitch_o.set_unit("Hz")
        self.pitch_o.set_silence(-40)

        self.stream = sd.InputStream(
            channels=1,
            samplerate=self.SAMPLE_RATE,
            blocksize=self.HOP_SIZE,
            callback=lambda indata, frames, time_info, status: self._audio_callback(indata, frames, time_info, status)
        )
        self.stream.start()

        # 명령 처리 스레드
        self._cmd_thread = threading.Thread(target=self._command_loop, daemon=True)
        self._cmd_thread.start()

        # # 하트비트/모드 브로드캐스트
        # self.create_timer(3.0, lambda: self.ws.send({"type":"mode","value":self.mode}))
        # 하트비트/모드 브로드캐스트 (모드 선택된 경우에만 전송)
        def _send_mode_if_set():
            if self.mode is not None:
                self.ws.send({"type": "mode", "value": self.mode})
        self.create_timer(3.0, _send_mode_if_set)

        # 초기 로그도 모드 문구 제거
        self.get_logger().info(f"[Violin Tuner] A4={self.a4_ref:.1f} Hz | strings: G3 D4 A4 E5")
    ########################3
    #%#%
    def _tick_hz_recent(self):
        """마지막 hz_value 퍼블리시 시각을 기준으로 최근 여부를 계산, 토픽으로 내보냄."""
        now = time.monotonic()
        new_state = (now - self._last_hz_pub_ts) <= self.HZ_RECENT_WINDOW_SEC
        # 상태가 바뀌었거나 주기적으로 항상 퍼블리시하고 싶다면 아래 그대로 두세요.
        self._hz_recent = new_state
        msg = Bool(); msg.data = new_state
        self.hz_recent_pub.publish(msg)
    def _quantize_hz(self, f: float) -> float:
        """연속 Hz를 bin 크기(UNIQUE_BIN_HZ)로 양자화하여 중복 제거."""
        if not math.isfinite(f) or f <= 0:
            return 0.0
        # 예: bin=1.0이면 440.49→440.0, 440.5→441.0
        q = round(f / self.UNIQUE_BIN_HZ) * self.UNIQUE_BIN_HZ
        # float 보기 좋게 소수 3자리까지만
        return float(round(q, 3))
    def pluck_state(self, msg):
        # if msg.data:
        #     self.pluck_ok = True
        #     self.get_logger().info("pluck complete")
        if msg.data:
            self.pluck_ok = True
            self.get_logger().info("pluck complete")

            #%#%  추가: pluck 직후에도 최근 hz_value가 없으면 retry 펄스
            if not self._hz_recent:
                self.get_logger().warn("[retry] pluck 완료됐지만 최근 hz_value가 없어 retry를 보냅니다.")
                tmsg = Bool(); tmsg.data = True
                self.retry_publisher.publish(tmsg)
                # 잠깐 대기 후 False로 내려 펄스 형태로 전달 (구독자 트리거용)
                
    def _record_unique_hz(self, f: float):
        q = self._quantize_hz(f)
        if q <= 0: 
            return
        with self._freq_lock:
            self._freq_counts[q] += 1

    def _flush_unique_window(self):
        """15초 윈도우 종료 시 수집된 고유 Hz 목록/카운트 전송 후 비움."""
        with self._freq_lock:
            if not self._freq_counts:
                return
            items = sorted((float(k), int(v)) for k, v in self._freq_counts.items())
            self._freq_counts.clear()
            started = self._win_started
            self._win_started = time.monotonic()

        # 웹소켓으로도 보내서 UI에서 표/리스트로 띄울 수 있게
        self.ws.send({
            "type": "hz_window",
            "bin_hz": float(self.UNIQUE_BIN_HZ),
            "duration_sec": float(self.UNIQUE_WINDOW_SEC),
            "since": float(started),
            "items": items  # [ [hz_bin, count], ... ]
        })

        # 로그에도 간단히 남김 (상위 10개 미리보기)
        preview = items[:]
        self.get_logger().info(f"[hz_window] bins={len(items)} bin={self.UNIQUE_BIN_HZ}Hz items(sample)={preview}")

    #%#%
    def _start_worker(self, target, *args):
        with self._worker_lock:
            # 이전 세션이 돌고 있으면 중단 지시
            self._session_abort.set()
            self._stop_requested.clear()
            self._quit_requested.clear()
            # 바로 새 워커 시작
            self._session_abort.clear()
            self._worker = threading.Thread(target=target, args=args, daemon=True)
            self._worker.start()
    def _run_auto_session(self):
        try:
            self.ws.send({"type":"session","status":"start","mode":"auto"})
            self._publish_robot_active(True)
            for i, s in enumerate(STR_ORDER):
                if self._session_abort.is_set() or self._stop_event.is_set():
                    break
                ok = self.tune_one_string(s, announce_target=True)
                if self._session_abort.is_set() or self._stop_event.is_set():
                    break
                if ok and i < len(STR_ORDER)-1:
                    self.ws.send({"type":"session","status":"cooldown",
                                "seconds": float(self.COOLDOWN_SEC), "next": STR_ORDER[i+1]})
                    end = time.monotonic() + self.COOLDOWN_SEC
                    while not self._session_abort.is_set() and not self._stop_event.is_set() and time.monotonic() < end:
                        time.sleep(0.1)
                    if self._session_abort.is_set() or self._stop_event.is_set():
                        break
                    self.ws.send({"type":"session","status":"next","next":STR_ORDER[i+1]})
        finally:
            self._publish_robot_active(False)
            end_signal = Bool();end_signal.data = True
            self.tunning_end_publisher.publish(end_signal)
            if self._quit_requested.is_set():
                self.ws.send({"type":"session","status":"quit"})
                self._quit_requested.clear()
            else:
                if not self._stop_requested.is_set():
                    self.ws.send({"type":"session","status":"done","mode":"auto"})
                else:
                    self._stop_requested.clear()

    def _run_manual_session(self, s):
        try:
            self.ws.send({"type":"session","status":"start","mode":"manual","line": s})
            self._publish_robot_active(True)
            self.tune_one_string(s, announce_target=True)
        finally:
            self._publish_robot_active(False)
            if self._quit_requested.is_set():
                self.ws.send({"type":"session","status":"quit"})
                self._quit_requested.clear()
            else:
                if not self._stop_requested.is_set():
                    self.ws.send({"type":"session","status":"done","mode":"manual"})
                else:
                    self._stop_requested.clear()

    #################################
    
    # ===== 유틸 =====
    def _audio_callback(self, indata, frames, time_info, status):
        if status:
            print(status, file=sys.stderr, flush=True)
        mono = np.mean(indata, axis=1).astype(np.float32)
        try:
            self.q.put_nowait(mono)
        except queue.Full:
            pass

    def _make_smoother(self, n=None):
        if n is None:
            n = self.SMOOTH_N
        buf = deque(maxlen=n)
        def push_get(x):
            buf.append(float(x))
            arr = sorted(buf)
            m = len(arr)//2
            return arr[m] if len(arr)%2==1 else 0.5*(arr[m-1]+arr[m])
        return push_get

    def _dir_from_hz(self, delta_hz, harmonic_tag: str=""):
        if harmonic_tag:
            if harmonic_tag.startswith("H"):
                return "↓ 내려요"
            if harmonic_tag.startswith("S"):
                return "↑ 올려요"
        if delta_hz < -self.TOL_HZ: return "↑ 올려요"
        elif delta_hz > self.TOL_HZ: return "↓ 내려요"
        else: return "✓ 맞아요"

    
    # ===== 명령 처리 =====
    def _command_loop(self):
        while not self._stop_event.is_set():
            cmd = self.ws.poll_cmd(timeout=0.1)
            if not cmd: 
                continue
            try:
                c = cmd.get("cmd")
                if c == "get_mode":
                    # 미선택이면 굳이 보내지 않거나, 상태를 명시적으로 알리고 싶으면 아래처럼
                    if self.mode is None:
                        self.ws.send({"type": "mode", "value": None, "status": "unset"})
                    else:
                        self.ws.send({"type":"mode","value":self.mode})

                
                elif c == "mode":
                    m = str(cmd.get("mode","manual")).lower()
                    if m not in ("auto","manual"):
                        continue
                    self.mode = m
                    self.ws.send({"type":"mode","value":self.mode})
                    if self.mode == "auto":
                        self._start_worker(self._run_auto_session)

                elif c == "start":
                    s = cmd.get("string","")
                    if s not in ("G3","D4","A4","E5"):
                        continue
                    # 수동 세션 시작 시에만 manual 설정
                    self.mode = "manual"
                    self.ws.send({"type":"mode","value":self.mode})
                    self._start_worker(self._run_manual_session, s)

                elif c == "stop":
                    self._session_abort.set()
                    # self._publish_robot_active(False)
                    self.mode = None
                    self._stop_requested.set()
                    self.ws.send({"type":"session","status":"done","mode":self.mode})

                elif c == "quit":
                    self._session_abort.set()
                    self._publish_robot_active(False)
                    self._quit_requested.set()

                ############
                    
                    
            except Exception as e:
                self.get_logger().error(f"command error: {e}")
    ################
    def _publish_robot_active(self, val: bool):
        msg = Bool(); msg.data = bool(val)
        if val == True:
            self.robot_active_pub.publish(msg)
        self.ws.send({"type":"robot_active","value":bool(val)})  # ➕ 추가
        self.get_logger().info(f"[robot_active] {val}")

    #################
    # ===== 한 현 튜닝 =====
    def tune_one_string(self, string_name, announce_target=False):
        refs = violin_string_freqs(self.a4_ref)
        base = refs[string_name]
        smoother = self._make_smoother()
        taken = 0
        last_emit = 0.0
        self._streak_kind = 0
        self._streak_count = 0
        near_hint_sent = False  # ← 추가

        # UI에 타깃 알림 (스펙트럼 빨간 줄 위해 base 포함)
        if announce_target:
            self.ws.send({"type":"target","line": string_name, "base": float(base), "tol_hz": float(self.TOL_HZ)})
        # ROS 타깃 현 퍼블리시(기존)
        ls = String(); ls.data = string_name
        self.line_publisher.publish(ls)

        ok_window = deque(maxlen=4)

        while (not self._session_abort.is_set()) and taken < self.N_SAMPLES_PER_STRING and not self._stop_event.is_set():
            try:
                buf = self.q.get(timeout=1.0)
            except queue.Empty:
                continue

            f0 = float(self.pitch_o(buf)[0])
            conf = float(self.pitch_o.get_confidence())
            if conf < self.CONF_MIN or f0 < self.MIN_FREQ:
                continue

            now = time.monotonic()
            f0_s = smoother(f0)

            # 유효 대역
            lo = math.floor(base / 2.0) + 4
            hi = math.ceil(2.0 * base) - 4
            if not (lo <= f0_s <= hi):
                continue

            f_fold, tag = fold_harmonic_to_base(f0_s, base)

            # === PATCH: 옥타브 스냅퍼 ===
            f_fold, tag2 = snap_octave_to_target(f_fold, base, band=(0.70, 1.40))
            if tag2 and not tag:
                tag = tag2   # 스냅에서 옥타브 보정이 일어났으면 태그로 표시(안정창 리셋에 활용)
            # === /PATCH ===

            # 안정 게이트
            STABLE_N  = 5 # 13
            STABLE_HZ = 1.2 #1.0
            try:
                stab_buf
            except NameError:
                stab_buf = deque(maxlen=STABLE_N)

            if tag:
                stab_buf.clear(); continue
            else:
                stab_buf.append(f_fold)
                if len(stab_buf) < STABLE_N: continue
                dev = max(stab_buf) - min(stab_buf)
                if dev > STABLE_HZ:
                    stab_buf.popleft(); continue

            delta_hz = f_fold - base
            ok_window.append((not tag) and (abs(delta_hz) <= self.TOL_HZ))
            consecutive_ok = len(ok_window) == ok_window.maxlen and all(ok_window)

            decision = "✓ 맞아요" if consecutive_ok else self._dir_from_hz(delta_hz, harmonic_tag=tag)
            if decision.startswith("↑"): kind = 1
            elif decision.startswith("↓"): kind = 2
            elif decision.startswith("✓"): kind = 3
            else: kind = 0

            # 트리거 퍼블리시 (기존 로직 유지)
            if kind == 0:
                self._streak_kind = 0; self._streak_count = 0
            else:
                if self._streak_kind == kind: self._streak_count += 1
                else: self._streak_kind = kind; self._streak_count = 1

                if self._streak_count >= self.STREAK_N:
                    msg = Float32MultiArray(); 
                    msg.data = [float(kind), (delta_hz)]
                    self.hz_publisher.publish(msg)
                    self._last_hz_pub_ts = time.monotonic()
                    self._hz_recent = True
                    time.sleep(3)
                    self._streak_count = 0
                    if kind == 3:
                        # OK → 세션 성공 종료
                        # 최종 상태 프레임을 UI에 한번 더
                        # === 현 OK 이벤트 전송 ===
                        self.ws.send({"type":"session","status":"ok","line":string_name,"hz":float(f_fold),"base":float(base)})
                        # 최종 상태 프레임도 한번 더
                        self.ws.send({
                            "type": "state",
                            # "mode": self.mode,  # 그대로 보내면 None -> null이 가므로, 원하면 생략/보호
                            "mode": (self.mode or ""),   # UI가 빈 문자열이면 미표시하게 할 수도 있음
                            "line": string_name,
                            "hz": float(f_fold),
                            "base": float(base),
                            "tol_hz": float(self.TOL_HZ)
                        })
                        return True

            # 전송 레이트 제한
            if (now - last_emit) < (1.0 / self.SAMPLES_PER_SEC):
                continue
            last_emit = now
            taken += 1

            self._record_unique_hz(f_fold)

            # UI 상태 프레임 (UI가 스펙트럼과 바늘 갱신)
            self.ws.send({
                "type":"state",
                "mode": self.mode,
                "line": string_name,
                "hz": float(f_fold),
                "base": float(base),
                "tol_hz": float(self.TOL_HZ)  # ← 추가
            })

        # 비정상/중단
        return False

    # ===== 정리 =====
    def destroy_node(self):
        self._stop_event.set()
        self._session_abort.set()
        try:
            if self.stream:
                self.stream.stop()
                self.stream.close()
        except Exception:
            pass
        try:
            self.ws.stop()
        except Exception:
            pass
        return super().destroy_node()

def main():
    rclpy.init()
    node = ViolinTunerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n중단되었습니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

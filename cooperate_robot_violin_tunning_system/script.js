// === script.js (WS 수신 + 스펙트럼 + 모드/현 제어 + 세션 UX) ===
const tunerScreen = document.getElementById('tunerScreen');
const noteNameEl = document.getElementById('noteName');
const freqEl = document.getElementById('frequency');
const centsEl = document.getElementById('centsText');
const needleEl = document.getElementById('needle');
const stringTargetEl = document.getElementById('stringTarget');
const statusEl = document.getElementById('status');
const modeLabelEl = document.getElementById('modeLabel');
const stringEls = [...document.querySelectorAll('.string')];
const toastEl = document.getElementById('toast');
const modalEl = document.getElementById('modal');
const modalTextEl = document.getElementById('modalText');
let tolHz = 5.0; // 노드에서 오면 갱신됨
const NOTE_NAMES = ['C','C#','D','D#','E','F','F#','G','G#','A','A#','B'];
//const WS_URL = "ws://localhost:8765";
// 접속한 호스트 기준으로 자동 설정
const WS_PORT = 8765;
const WS_SCHEME = (location.protocol === 'https:' ? 'wss://' : 'ws://');
const WS_URL = WS_SCHEME + (location.hostname || 'localhost') + ':' + WS_PORT;
// AUTO 버튼 레퍼런스 (id 없이 선택)
const btnAutoCtrl = document.querySelector('.controls .mode button:not(.ghost)'); // 하단 컨트롤의 AUTO
const btnManualCtrl = document.querySelector('.controls .mode button.ghost');

const queryAutoGate = () => document.querySelector('#modeGate .gate-buttons button:not(.ghost)'); // 게이트의 AUTO (게이트는 동적 생성/삭제라 함수로)
const queryManualGate = () => document.querySelector('#modeGate .gate-buttons button.ghost');




// ===== 스펙트럼 =====
const F_MIN = 100;
const F_MAX = 1000;
let currentFreq = 0;   // 흰 줄
let targetBase = null; // 빨간 줄

const specCanvas = document.getElementById('spectrum');
const specCtx = specCanvas.getContext('2d');
const dpr = window.devicePixelRatio || 1;

let selectedMode = null; // 'auto' | 'manual' | null
let ws;
let rafIdSpectrum;

// ------- 유틸: 토스트/모달/게이트 -------
let toastTimer;

// 모드에 맞춰 버튼들을 비/활성화
function updateModeButtonsLock(selectedMode) {
  const isAuto   = (selectedMode === 'auto');
  const isManual = (selectedMode === 'manual');

  if (btnAutoCtrl)   btnAutoCtrl.disabled   = isManual; // manual이면 AUTO 비활성
  if (btnManualCtrl) btnManualCtrl.disabled = isAuto;   // auto이면 MANUAL 비활성

  const gA = queryAutoGate();   if (gA) gA.disabled = isManual;
  const gM = queryManualGate(); if (gM) gM.disabled = isAuto;
}

function showToast(text, ms=1500){
  if (!toastEl) return;
  toastEl.textContent = text;
  toastEl.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(()=> toastEl.classList.remove('show'), ms);
}
function showModal(text){
  if (!modalEl) return;
  modalTextEl.textContent = text || '끝났습니다';
  modalEl.classList.remove('hidden');
}
function hideModal(){
  if (!modalEl) return;
  modalEl.classList.add('hidden');
}
function rebuildModeGate(){
  // 이미 있으면 패스
  if (document.getElementById('modeGate')) return;
  const gate = document.createElement('section');
  gate.id = 'modeGate';
  gate.className = 'mode-gate';
  gate.innerHTML = `
    <h2>모드를 선택하세요</h2>
    <div class="gate-buttons">
      <button onclick="app.chooseMode('auto')">AUTO</button>
      <button class="ghost" onclick="app.chooseMode('manual')">MANUAL</button>
    </div>
    <p class="hint">Auto: 4현 자동 튜닝 시작 / Manual: 현을 직접 선택</p>
  `;
  // header 다음에 끼워넣기
  const header = tunerScreen.querySelector('header');
  header.after(gate);
  const g = queryAutoGate();
  if (g) g.disabled = (selectedMode === 'manual');

  updateModeButtonsLock(selectedMode);


}
function setControlsForMode() {
  updateModeButtonsLock(selectedMode);
  const controls = document.querySelector('.controls');
  const manualStrip = document.querySelector('.manual-strings');
  if (!controls || !manualStrip) return;

  if (!selectedMode) {
    controls.classList.add('disabled');
    manualStrip.classList.add('disabled');
    if (modeLabelEl) modeLabelEl.textContent = '현재 모드: -';
    return;
  }
  controls.classList.remove('disabled');
  if (selectedMode === 'auto') {
    manualStrip.classList.add('disabled');
    if (modeLabelEl) modeLabelEl.textContent = '현재 모드: AUTO';
  } else {
    manualStrip.classList.remove('disabled');
    if (modeLabelEl) modeLabelEl.textContent = '현재 모드: MANUAL';
  }
  // --- AUTO 버튼 비/활성화 ---
  if (btnAutoCtrl) btnAutoCtrl.disabled = (selectedMode === 'manual');
  const g = queryAutoGate();
  if (g) g.disabled = (selectedMode === 'manual');

}

// ------- 스펙트럼 그리기 -------
function setupCanvas() {
  specCanvas.width  = Math.floor(specCanvas.clientWidth  * dpr);
  specCanvas.height = Math.floor(specCanvas.clientHeight * dpr);
  specCtx.setTransform(1,0,0,1,0,0);
  specCtx.scale(dpr, dpr);
}
function freqToX(freq){
  const pad = 8;
  const cw = specCanvas.clientWidth;
  const w = cw - pad*2;
  const clamped = Math.max(F_MIN, Math.min(F_MAX, freq));
  const t = (clamped - F_MIN) / (F_MAX - F_MIN);
  return pad + t * w;
}
function drawSpectrum(){
  const ctx = specCtx;
  const cw = specCanvas.clientWidth;
  const ch = specCanvas.clientHeight;

  ctx.clearRect(0,0,cw,ch);
  ctx.fillStyle = '#0f1521';
  ctx.fillRect(0,0,cw,ch);

  ctx.strokeStyle = '#1c2435';
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let f = Math.ceil(F_MIN/100)*100; f <= F_MAX; f += 100){
    const x = freqToX(f);
    ctx.moveTo(x, 8);
    ctx.lineTo(x, ch-8);
  }
  ctx.stroke();

  ctx.fillStyle = '#aab7cf';
  ctx.font = '12px ui-sans-serif';
  for (let f = Math.ceil(F_MIN/200)*200; f <= F_MAX; f += 200){
    const x = freqToX(f);
    ctx.fillText(`${f} Hz`, x-18, ch-12);
  }
  // 목표 허용오차 하이라이트 (형광 노란색)
if (targetBase && tolHz > 0) {
  const leftF  = Math.max(F_MIN, targetBase - tolHz);
  const rightF = Math.min(F_MAX, targetBase + tolHz);
  if (rightF > leftF) {
    const x1 = freqToX(leftF);
    const x2 = freqToX(rightF);
    const y  = 8;
    const h  = ch - 16;

    // 중앙이 진하고 가장자리가 옅은 형광펜 느낌
    const grad = ctx.createLinearGradient(x1, 0, x2, 0);
    grad.addColorStop(0.00, 'rgba(253, 224, 71, 0)');   // 형광 노랑
    grad.addColorStop(0.15, 'rgba(253, 224, 71, 0.28)');
    grad.addColorStop(0.85, 'rgba(253, 224, 71, 0.28)');
    grad.addColorStop(1.00, 'rgba(253, 224, 71, 0)');

    ctx.save();
    ctx.fillStyle = grad;
    ctx.fillRect(x1, y, (x2 - x1), h);

    // 약간의 글로우(선택)
    ctx.shadowColor = 'rgba(253, 224, 71, 0.45)';
    ctx.shadowBlur = 10;
    ctx.strokeStyle = 'rgba(253, 224, 71, 0.35)';
    ctx.lineWidth = 2;
    ctx.strokeRect(x1, y+2, (x2 - x1), h-4);
    ctx.restore();
  }
}

  if (targetBase){
    const x = freqToX(targetBase);
    ctx.strokeStyle = '#ef4444';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x, 8);
    ctx.lineTo(x, ch-8);
    ctx.stroke();
    ctx.fillStyle = '#ef4444';
    ctx.fillText(`target ${targetBase.toFixed(2)} Hz`, Math.min(x+6, cw-110), 16);
  }
  if (currentFreq > 0){
    const x = freqToX(currentFreq);
    ctx.strokeStyle = '#e8eef8';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x, 8);
    ctx.lineTo(x, ch-8);
    ctx.stroke();
    ctx.fillStyle = '#e8eef8';
    ctx.fillText(`${currentFreq.toFixed(2)} Hz`, Math.max(8, x-40), 30);
  }
  rafIdSpectrum = requestAnimationFrame(drawSpectrum);
}

// ------- WS & UI -------
function connectWebSocket() {
  statusEl.textContent = "ROS 서버 연결 중…";
  ws = new WebSocket(WS_URL);

  ws.onopen = () => {
    statusEl.textContent = "ROS 서버 연결됨";
    tunerScreen.classList.add('active');
    setupCanvas(); drawSpectrum();
    setControlsForMode();
    sendCmd({cmd:"get_mode"});
  };

  ws.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);

      // 상태 프레임
      if (msg.type === "state") {
        const { line, hz, base, mode, tol_hz  } = msg;
        if (typeof tol_hz === "number") tolHz = Math.max(0, tol_hz);
        if (typeof hz === "number") {
          updateUIWithFrequency(hz);
          currentFreq = hz || 0;
        }
        if (line) setTarget(line, base);
        if (typeof base === "number") targetBase = base;
        if (mode) modeLabelEl.textContent = `현재 모드: ${mode.toUpperCase()}`;
        return;
      }

      // 타깃 알림
      if (msg.type === "target") {
        const { line, base, tol_hz } = msg;
        if (typeof tol_hz === "number") tolHz = Math.max(0, tol_hz);
        setTarget(line, base);
        if (typeof base === "number") targetBase = base;
        return;
      }

      // 모드 브로드캐스트
      if (msg.type === "mode") {
        if (!selectedMode && (msg.value === 'auto' || msg.value === 'manual')) {
          selectedMode = msg.value;
          setControlsForMode();
          updateModeButtonsLock(selectedMode);
        }
        modeLabelEl.textContent = `현재 모드: ${String(msg.value).toUpperCase()}`;
        return;
        
      }


      // 세션 이벤트
      if (msg.type === "session") {
        const { status } = msg;
        if (status === "start") {
          if (msg.mode === "auto") statusEl.textContent = "AUTO: 자동 튜닝 시작";
          else statusEl.textContent = "MANUAL: 현을 선택해 진행 중…";
        }
        if (status === "ok") {
          // 현 OK 순간
          if (selectedMode === 'auto') {
            showToast("맞아요! 다음 현으로 갑니다…(5초 대기)", 1300);
          } else {
            showToast("맞아요!", 1200);
          }
        }
        if (status === "next") {
          // 다음 현으로 실제 넘어감
          const n = msg.next || "";
          showToast(`다음 현(${n})입니다.`, 1400);
        }
        if (status === "done") {
          // 세션 종료 → 모달 + 게이트 복구
          showModal("끝났습니다");
          selectedMode = null;
          setControlsForMode();
          rebuildModeGate();
          statusEl.textContent = "세션 종료";
        }
        return;
      }

      // 구 포맷 호환
      if (msg.hz !== undefined) {
        updateUIWithFrequency(msg.hz);
        currentFreq = msg.hz || 0;
      }
      if (msg.line !== undefined) {
        setTarget(msg.line);
      }

    } catch (e) {
      console.error("Invalid WS message:", e);
    }
    // if (msg.type === "retry") {
    //   const text = msg.message || "소리가 인식되지 않습니다. 다시 움직이는 신호를 보냅니다";
    //   showToast(text, 1800);      // 하단 토스트
    //   statusEl.textContent = text; // 상태 라벨에도 반영
    //   return;
    // }
  };

  ws.onclose = () => {
    statusEl.textContent = "서버 연결 끊김 - 2초 후 재시도";
    setTimeout(connectWebSocket, 2000);
  };
  ws.onerror = () => { statusEl.textContent = "서버 오류"; };
}

function sendCmd(obj) {
  if (!ws || ws.readyState !== WebSocket.OPEN) {
    statusEl.textContent = "WS가 아직 연결되지 않았습니다.";
    return;
  }
  ws.send(JSON.stringify(obj));
}

function frequencyToNote(freq, A4 = 440) {
  const noteNumber = 12 * (Math.log(freq / A4) / Math.log(2)) + 69;
  const rounded = Math.round(noteNumber);
  const noteIndex = (rounded % 12 + 12) % 12;
  const octave = Math.floor(rounded / 12) - 1;
  const name = NOTE_NAMES[noteIndex] + octave;
  const refFreq = A4 * Math.pow(2, (rounded - 69) / 12);
  const cents = Math.round(1200 * Math.log(freq / refFreq) / Math.log(2));
  return { name, cents };
}

function updateUIWithFrequency(freq) {
  if (!freq || freq <= 0) {
    statusEl.textContent = "신호 대기 중…";
    return;
  }
  statusEl.textContent = "신호 감지됨";
  freqEl.textContent = `${freq.toFixed(2)} Hz`;

  const note = frequencyToNote(freq);
  noteNameEl.textContent = `${note.name}`;
  centsEl.textContent = `${note.cents > 0 ? '+' : ''}${note.cents} cent`;

  const clamped = Math.max(-50, Math.min(50, note.cents));
  const percent = (clamped + 50) / 100;
  needleEl.style.left = `${percent * 100}%`;
}

function setTarget(line, base) {
  if (!line) return;
  stringTargetEl.textContent = base
    ? `목표 현: ${line} (${Number(base).toFixed(2)} Hz)`
    : `목표 현: ${line}`;
  stringEls.forEach(el => {
    el.classList.toggle('active', el.dataset.note === line);
  });
  if (typeof base === "number") targetBase = base;
}

// 공개 API (버튼 핸들러)
window.app = {
  chooseMode: (mode) => {
    if (!['auto','manual'].includes(mode)) return;
    hideModal(); // 혹시 열려있다면 닫기
    selectedMode = mode;
    setControlsForMode();
    document.getElementById('modeGate')?.remove();
    sendCmd({cmd:"mode", mode});
    if (mode === 'auto') statusEl.textContent = "AUTO 시작…";
    else statusEl.textContent = "MANUAL 모드: 현을 선택하세요.";
  },
  setAuto:   () => { selectedMode='auto'; setControlsForMode(); sendCmd({cmd:"mode", mode:"auto"}); },
  setManual: () => { selectedMode='manual'; setControlsForMode(); sendCmd({cmd:"mode", mode:"manual"}); },
  startString: (name) => {
    if (selectedMode !== 'manual') { alert('먼저 MANUAL 모드를 선택하세요.'); return; }
    sendCmd({cmd:"start", string:name});
  },
  stop: () => {
    sendCmd({cmd:"stop"});
    // 낙관적 UI: 서버 응답을 기다리지 않고 즉시 게이트 복구
    showModal("중지했습니다");
    selectedMode = null;
    setControlsForMode();
    rebuildModeGate();
    statusEl.textContent = "세션 중지";
  },

  restart: () => { hideModal(); selectedMode=null; setControlsForMode(); rebuildModeGate(); },
};

// 시작
connectWebSocket();

// 스펙트럼 루프 시작 + 리사이즈 대응
setupCanvas(); drawSpectrum();
window.addEventListener('resize', () => {
  cancelAnimationFrame(rafIdSpectrum);
  setupCanvas();
  drawSpectrum();
});

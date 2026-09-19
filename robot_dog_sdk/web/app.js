/* 机器狗遥控前端
 *
 * 核心思路：所有输入（按钮/摇杆/键盘）都折算成同一个"意图" {x, y, yaw}，
 * 以 10Hz 持续发给服务端。服务端有 400ms 看门狗 —— 只要意图停发，
 * 它就强制归零。所以"松手事件丢了""标签页被关了""笔记本休眠了"
 * 这三种情况都由同一道闸兜住，前端不需要为它们各写一套逻辑。
 */

'use strict';

const INTENT_HZ = 10;
const clamp = v => Math.max(-1, Math.min(1, v));

let ws = null;
let myId = null;
let state = {};
let retryTimer = null;

// ---------- 意图 ----------
const inputs = new Map();          // id -> {x, y, yaw}
let intent = { x: 0, y: 0, yaw: 0 };
let dirty = false;

function recompute() {
  let x = 0, y = 0, yaw = 0;
  for (const v of inputs.values()) { x += v.x; y += v.y; yaw += v.yaw; }
  intent = { x: clamp(x), y: clamp(y), yaw: clamp(yaw) };
  dirty = true;
  sendIntent();                     // 变化立刻生效，不等下一个周期
}

function setInput(id, vec) {
  if (vec) inputs.set(id, vec); else inputs.delete(id);
  recompute();
}

function releaseAll() {
  inputs.clear();
  intent = { x: 0, y: 0, yaw: 0 };
  dirty = true;
  sendIntent();
}

function sendIntent() {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  if (!state.can_control) return;
  send({ t: 'intent', x: intent.x, y: intent.y, yaw: intent.yaw });
  dirty = false;
}

function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}

// 10Hz 保活：只要还有非零意图就持续重发，喂看门狗
setInterval(() => {
  if (dirty || intent.x || intent.y || intent.yaw) sendIntent();
}, 1000 / INTENT_HZ);

// ---------- WebSocket ----------
function connect() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(`${proto}://${location.host}/ws`);

  ws.onopen = () => {
    setConn(true);
    log('info', '已连接到服务端');
  };

  ws.onmessage = ev => {
    let m;
    try { m = JSON.parse(ev.data); } catch { return; }
    if (m.t === 'hello') {
      myId = m.you;
      // 速度滑条上限由服务端决定（--max-scale），不要写死在 HTML 里
      if (m.max_scale) {
        const sc = document.getElementById('scale');
        sc.max = Math.round(m.max_scale * 100);
      }
      return;
    }
    if (m.t === 'event') { log(m.level, m.text); return; }
    if (m.t === 'state') { render(m); return; }
  };

  ws.onclose = () => {
    setConn(false);
    // 断线了也必须停 —— 不能假设服务端一定收到了我们的停止意图
    inputs.clear(); intent = { x: 0, y: 0, yaw: 0 };
    log('warn', '与服务端断开，正在重连…');
    clearTimeout(retryTimer);
    retryTimer = setTimeout(connect, 1000);
  };

  ws.onerror = () => { /* onclose 会处理 */ };
}

function setConn(on) {
  document.getElementById('connDot').className = 'dot ' + (on ? 'on' : 'off');
  document.getElementById('connText').textContent = on ? '已连接' : '未连接';
}

// ---------- 渲染 ----------
const fmt = v => (v === null || v === undefined) ? '—' : v;
const num = (v, n = 2) => (typeof v === 'number') ? v.toFixed(n) : '—';

function render(s) {
  state = s;

  document.getElementById('sMotion').textContent = `${fmt(s.motion_state)} ${s.motion_state_name || ''}`;
  document.getElementById('sGait').textContent = s.gait != null ? `0x${s.gait.toString(16).padStart(4, '0')} ${s.gait_name || ''}` : '—';
  document.getElementById('sMode').textContent = `${fmt(s.usage_mode)} ${s.usage_mode_name || ''}`;
  document.getElementById('sVx').textContent = num(s.linear_x);
  document.getElementById('sWz').textContent = num(s.angular_z);
  document.getElementById('sAxis').textContent = s.cmd_label || '—';
  const hes = document.getElementById('sHes');
  hes.textContent = s.hes === 1 ? '已触发' : '正常';
  hes.style.color = s.hes === 1 ? 'var(--err)' : '';
  document.getElementById('sModel').textContent = fmt(s.model);
  document.getElementById('sHz').textContent = s.control_hz ? s.control_hz.toFixed(1) : '—';

  // 你是哪个客户端 / 谁拿着控制权。两个 ID 不一样 = 你的操控会被忽略。
  document.getElementById('sYou').textContent = s.you || '—';
  const ctrlEl = document.getElementById('sCtrl');
  ctrlEl.textContent = s.controller || '（无）';
  ctrlEl.style.color = (s.controller && s.controller === s.you) ? 'var(--ok)' : 'var(--dim)';

  // 控制权
  const can = !!s.can_control;
  document.body.classList.toggle('nocontrol', !can);
  document.getElementById('btnClaim').disabled = can;
  document.getElementById('ctrlHint').textContent = can
    ? '你持有控制权。关闭页面或断开连接会立即释放并归零。'
    : `控制权在 ${s.controller || '（无）'} 手上。点"接管控制"可转移过来。`;

  // 急停
  document.getElementById('estop').classList.toggle('hidden', !!s.estop);
  document.getElementById('estopReset').classList.toggle('hidden', !s.estop);

  // 横幅
  const banner = document.getElementById('banner');
  let cls = 'hidden', text = '';
  if (s.observe) { cls = 'banner nowarn'; text = '👁 只读观察模式 —— 界面照常显示，但不会向机器狗发送任何轴指令。拖动摇杆只影响本地显示。'; }
  else if (s.estop) { cls = 'banner estop'; text = '⛔ 急停中 —— 已锁存零速，点"解除急停"恢复。注意这是软件零速，不是硬件急停。'; }
  else if (s.hes === 1) { cls = 'banner estop'; text = '⛔ 机器人硬急停(HES)已触发 —— 请用本体上的物理急停按钮复位'; }
  else if (!s.connected) { cls = 'banner nowarn'; text = '⚠ 未收到机器人状态上报。检查 robotserve 是否已关闭 30004 端口加密（文档 §1.1.2）'; }
  else if (s.gate) { cls = 'banner nowarn'; text = '⚠ 无法运动：' + s.gate; }
  else if (!can) { cls = 'banner nowarn'; text = 'ℹ 你还没有控制权 —— 点右上角「接管控制」后才能操控。在此之前可以先用「起立/趴下」等机身动作。'; }
  else if (s.must_release) { cls = 'banner nowarn'; text = 'ℹ 请先松开所有操作，再重新按住才能继续运动'; }
  banner.className = cls;
  banner.textContent = text;

  // 故障
  const f = document.getElementById('faults');
  if (s.faults && s.faults.length) {
    f.className = 'faults';
    f.innerHTML = s.faults.map(x =>
      `<div><b>${sevName(x.severity)}</b> 0x${x.code.toString(16).padStart(4, '0')} ` +
      `${x.name || ''} ${x.resources && x.resources.length ? '部件=' + x.resources.join(',') : ''}</div>`
    ).join('');
  } else {
    f.className = 'faults hidden';
  }

  document.getElementById('axisTag').textContent = s.axis_kind || '';

  // 步态按钮（只建一次）
  const gb = document.getElementById('gaits');
  if (gb.children.length === 0 && s.gaits) {
    s.gaits.forEach(g => {
      const b = document.createElement('button');
      b.className = 'act';
      b.dataset.value = g.value;
      b.textContent = g.label;
      b.dataset.needsControl = '1';
      b.onclick = () => send({ t: 'action', name: 'gait', value: g.value });
      gb.appendChild(b);
    });
  }
  [...gb.children].forEach(b => {
    b.classList.toggle('sel', s.gait === Number(b.dataset.value));
    b.disabled = !!s.action_running;
  });

  // 使用模式按钮（只建一次）
  const mb = document.getElementById('modes');
  if (mb.children.length === 0 && s.modes) {
    s.modes.forEach(m => {
      const b = document.createElement('button');
      b.className = 'act';
      b.dataset.value = m.value;
      b.textContent = m.label;
      b.dataset.needsControl = '1';
      b.onclick = () => send({ t: 'action', name: 'mode', value: m.value });
      mb.appendChild(b);
    });
  }
  [...mb.children].forEach(b => {
    b.classList.toggle('sel', s.usage_mode === Number(b.dataset.value));
    b.disabled = !!s.action_running;
  });

  // 动作按钮
  document.getElementById('btnStand').disabled = !!s.action_running;
  document.getElementById('btnCrouch').disabled = !!s.action_running;

  renderTemps(s);

  // 速度滑条（服务端拥有最终话语权，回读以免本地与服务端不一致）
  const sc = document.getElementById('scale');
  if (document.activeElement !== sc) {
    sc.value = Math.round((s.scale || 0) * 100);
    paintScale(s.scale || 0);
  }
}

function sevName(v) { return { 3: 'WARN', 4: 'ERROR', 5: 'FATAL' }[v] || v; }

// 速度值着色：越高越危险。1.0 约等于 1.67 m/s（6 km/h），室内很容易失控。
function paintScale(v) {
  const el = document.getElementById('scaleVal');
  el.textContent = v.toFixed(2);
  el.style.color = v >= 0.8 ? 'var(--err)' : (v >= 0.5 ? 'var(--warn)' : '');
}

// 关节温度：4 条腿 × 4 个关节，按机器狗的实际布局排（前排/后排、左/右）。
// 关节编号顺序来自文档 §1.3.1.2 对 MotorStatus.Joint 的说明。
const LEGS = [
  { key: 'lf', label: '左前腿', cls: 'front', base: 0 },
  { key: 'rf', label: '右前腿', cls: 'front', base: 4 },
  { key: 'lb', label: '左后腿', cls: 'back',  base: 8 },
  { key: 'rb', label: '右后腿', cls: 'back',  base: 12 },
];
const LEG_ROW = ['髋X', '髋Y', '膝', '轮'];

function tempClass(t) {
  if (t == null) return '';
  if (t >= 75) return 'hot';
  if (t >= 60) return 'warn';
  return '';
}

function renderTemps(s) {
  const grid = document.getElementById('tempGrid');
  const temps = s.temps;
  const hint = document.getElementById('tempHint');

  if (!temps || !temps.length) {
    hint.textContent = '（未收到设备状态上报）';
    if (!grid.children.length) grid.innerHTML = '<div class="hint">等待设备状态上报…</div>';
    return;
  }
  hint.textContent = '';

  // 只在结构变化时重建，避免每 100ms 重建 DOM 造成闪烁
  if (grid.children.length !== LEGS.length) {
    grid.innerHTML = LEGS.map(leg => `
      <div class="tleg ${leg.cls}">
        <h4>${leg.label}</h4>
        ${LEG_ROW.map((_, r) => `
          <div class="trow" data-i="${leg.base + r}">
            <span class="tname">${LEG_ROW[r]}</span>
            <span class="tval"></span>
          </div>`).join('')}
      </div>`).join('');
  }

  let hottest = null;
  temps.forEach(t => {
    const row = grid.querySelector(`.trow[data-i="${t.i}"]`);
    if (!row) return;
    const val = row.querySelector('.tval');
    val.textContent = (t.motor == null ? '--' : t.motor.toFixed(0)) + ' / ' +
                      (t.driver == null ? '--' : t.driver.toFixed(0));
    val.className = 'tval ' + tempClass(t.driver);
    row.classList.toggle('flagged', !!t.flagged);
    let flag = row.querySelector('.tflag');
    if (t.flagged && !flag) {
      flag = document.createElement('span');
      flag.className = 'tflag';
      flag.textContent = '⚠';
      val.appendChild(flag);
    } else if (!t.flagged && flag) {
      flag.remove();
    }
    if (t.driver != null && (hottest === null || t.driver > hottest)) hottest = t.driver;
  });

  // 标题里带上最高温和电池，一眼能看到
  const b = s.battery_info || {};
  const bits = [];
  if (hottest != null) bits.push(`最高 ${hottest.toFixed(0)}°C`);
  if (b.BatteryLevel != null) bits.push(`电量 ${b.BatteryLevel}%`);
  if (b.battery_temperature != null) bits.push(`电池 ${b.battery_temperature}°C`);
  if (s.temps_age != null && s.temps_age > 3) bits.push(`数据 ${s.temps_age.toFixed(0)}s 前`);
  hint.textContent = bits.length ? '· ' + bits.join(' · ') : '';
}

function log(level, text) {
  const box = document.getElementById('log');
  const d = document.createElement('div');
  const t = new Date();
  const hh = String(t.getHours()).padStart(2, '0');
  const mm = String(t.getMinutes()).padStart(2, '0');
  const ss = String(t.getSeconds()).padStart(2, '0');
  d.innerHTML = `<time>${hh}:${mm}:${ss}</time><span class="${level}">${escapeHtml(text)}</span>`;
  box.appendChild(d);
  while (box.children.length > 300) box.removeChild(box.firstChild);
  box.scrollTop = box.scrollHeight;
}

function escapeHtml(s) {
  return String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}

// ---------- 方向按钮：按住持续动 ----------
function bindHold(btn) {
  const axis = btn.dataset.axis;
  const dir = Number(btn.dataset.dir || 0);
  if (axis === 'stop') {
    const on = e => { e.preventDefault(); releaseAll(); flash(btn); };
    btn.addEventListener('pointerdown', on);
    return;
  }
  const id = `btn-${axis}-${dir}`;
  const start = e => {
    e.preventDefault();
    if (!state.can_control) return;
    btn.setPointerCapture?.(e.pointerId);
    btn.classList.add('held');
    setInput(id, { x: axis === 'x' ? dir : 0, y: axis === 'y' ? dir : 0, yaw: axis === 'yaw' ? dir : 0 });
  };
  const end = e => {
    btn.classList.remove('held');
    setInput(id, null);
  };
  btn.addEventListener('pointerdown', start);
  btn.addEventListener('pointerup', end);
  btn.addEventListener('pointercancel', end);
  btn.addEventListener('pointerleave', end);
  btn.addEventListener('contextmenu', e => e.preventDefault());
}

function flash(btn) {
  btn.classList.add('held');
  setTimeout(() => btn.classList.remove('held'), 120);
}

document.querySelectorAll('.mv').forEach(bindHold);

// ---------- 摇杆 ----------
(function initJoystick() {
  const joy = document.getElementById('joy');
  const knob = document.getElementById('knob');
  const R = 51;                     // 摇杆可移动半径(px)
  let pid = null;

  const center = () => {
    knob.style.transform = 'translate(0,0)';
    knob.classList.remove('active');
    setInput('joy', null);
  };

  const move = e => {
    const r = joy.getBoundingClientRect();
    let dx = e.clientX - (r.left + r.width / 2);
    let dy = e.clientY - (r.top + r.height / 2);
    const d = Math.hypot(dx, dy);
    if (d > R) { dx = dx / d * R; dy = dy / d * R; }
    knob.style.transform = `translate(${dx}px,${dy}px)`;
    knob.classList.add('active');
    // 上 = 前进(x+)；右 = 右转(yaw+)
    setInput('joy', { x: clamp(-dy / R), y: 0, yaw: clamp(dx / R) });
  };

  joy.addEventListener('pointerdown', e => {
    if (!state.can_control) return;
    e.preventDefault();
    pid = e.pointerId;
    joy.setPointerCapture(pid);
    move(e);
  });
  joy.addEventListener('pointermove', e => { if (e.pointerId === pid) move(e); });
  const up = e => { if (pid !== null && e.pointerId === pid) { pid = null; center(); } };
  joy.addEventListener('pointerup', up);
  joy.addEventListener('pointercancel', up);
  joy.addEventListener('contextmenu', e => e.preventDefault());
})();

// ---------- 键盘 ----------
// 平移符号按文档 §1.2.5：Y=+1 左移、Y=-1 右移。
// 所以 Q（左移）= +1，E（右移）= -1，别按直觉写反。
const KEYMAP = {
  KeyW: { x: 1 }, KeyS: { x: -1 },
  KeyA: { yaw: -1 }, KeyD: { yaw: 1 },
  KeyQ: { y: 1 }, KeyE: { y: -1 },
  ArrowUp: { x: 1 }, ArrowDown: { x: -1 },
  ArrowLeft: { yaw: -1 }, ArrowRight: { yaw: 1 },
};

addEventListener('keydown', e => {
  if (e.code === 'Space') { e.preventDefault(); doEstop(); return; }
  const k = KEYMAP[e.code];
  if (!k || e.repeat) return;
  e.preventDefault();
  if (!state.can_control) return;
  setInput('key-' + e.code, k);
});

addEventListener('keyup', e => {
  const k = KEYMAP[e.code];
  if (!k) return;
  setInput('key-' + e.code, null);
});

// ---------- 失焦/离开：一律停 ----------
addEventListener('blur', releaseAll);
document.addEventListener('visibilitychange', () => { if (document.hidden) releaseAll(); });
addEventListener('beforeunload', () => { releaseAll(); send({ t: 'release' }); });

// ---------- 按钮绑定 ----------
function doEstop() {
  releaseAll();
  send({ t: 'estop' });
}

document.getElementById('estop').onclick = doEstop;
document.getElementById('estopReset').onclick = () => send({ t: 'estop_reset' });
document.getElementById('btnClaim').onclick = () => send({ t: 'claim' });
document.getElementById('btnStand').onclick = () => send({ t: 'action', name: 'stand' });
document.getElementById('btnCrouch').onclick = () => send({ t: 'action', name: 'crouch' });

document.getElementById('scale').oninput = e => {
  const v = Number(e.target.value) / 100;
  const el = document.getElementById('scaleVal');
  el.textContent = v.toFixed(2);
  // 高速度给个视觉警示：1.0 约等于 1.67 m/s（6 km/h），室内很容易失控
  el.style.color = v >= 0.8 ? 'var(--err)' : (v >= 0.5 ? 'var(--warn)' : '');
  send({ t: 'scale', value: v });
};

// ---------- 启动 ----------
connect();

/* follow 观测界面 —— 只读。5Hz 轮询 /api/state，直写 DOM。
   学 robot_dog_sdk/web/app.js 的做法：不做虚拟 DOM、hot path 上不重建 innerHTML，
   只对元素逐个赋值 —— 否则 5Hz 刷起来数字会闪。 */

'use strict';

const POLL_MS = 200;          // 5Hz。嫌慢就调小，代价是 HTTP 请求变多
const EV_MAX = 300;

const LEGS = [
  { name: '左前', cls: 'front', i: [0, 1, 2, 3] },
  { name: '右前', cls: 'front', i: [4, 5, 6, 7] },
  { name: '左后', cls: 'back',  i: [8, 9, 10, 11] },
  { name: '右后', cls: 'back',  i: [12, 13, 14, 15] },
];
const LEG_ROW = ['髋X', '髋Y', '膝', '轮'];

let lastEventId = 0;
let kvKeys = {};              // 每个 .kv 容器上次的键序列 —— 只在结构变化时重建

const $ = id => document.getElementById(id);
const esc = s => String(s).replace(/[&<>"']/g,
  c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));

function num(x, d = 2) {
  return (x === null || x === undefined || Number.isNaN(x)) ? '—' : Number(x).toFixed(d);
}
function ago(s) {
  if (s === null || s === undefined) return '—';
  if (s < 1) return `${Math.round(s * 1000)}ms`;
  if (s < 60) return `${s.toFixed(1)}s`;
  return `${Math.floor(s / 60)}m${Math.round(s % 60)}s`;
}
function setText(id, txt, cls) {
  const el = $(id);
  if (!el) return;
  el.textContent = txt;
  if (cls !== undefined) el.className = cls;
}

/* 键值行：结构变化才重建 DOM，否则只改文本（防 5Hz 闪烁） */
function renderKv(boxId, rows) {
  const box = $(boxId);
  if (!box) return;
  const keys = rows.map(r => r.k).join('|');
  if (kvKeys[boxId] !== keys) {
    box.innerHTML = rows.map(r =>
      `<div class="kv"><span class="k">${esc(r.k)}</span>` +
      `<span class="v" id="${boxId}_${esc(r.k)}"></span></div>`).join('');
    kvKeys[boxId] = keys;
  }
  for (const r of rows) {
    const el = $(`${boxId}_${r.k}`);
    if (el) { el.textContent = r.v; el.className = 'v ' + (r.lvl || ''); }
  }
}

function tempClass(t) {
  if (t === null || t === undefined) return '';
  return t >= 75 ? 'hot' : (t >= 60 ? 'warn' : '');
}

function renderTemps(temps) {
  const grid = $('tempGrid');
  if (!grid) return;
  if (grid.children.length !== LEGS.length) {          // 只在结构变化时重建
    grid.innerHTML = LEGS.map(L =>
      `<div class="tleg ${L.cls}"><h4>${L.name}</h4>` +
      L.i.map(j =>
        `<div class="trow"><span class="tname">${LEG_ROW[L.i.indexOf(j)]}</span>` +
        `<span><span class="tval" id="tm${j}"></span>` +
        `<span class="tdrv" id="td${j}"></span></span></div>`).join('') +
      `</div>`).join('');
  }
  const motor = temps.motor || [], driver = temps.driver || [];
  for (let j = 0; j < 16; j++) {
    const m = motor[j], d = driver[j];
    setText(`tm${j}`, m === undefined ? '—' : `${num(m, 0)}°`, tempClass(m));
    setText(`td${j}`, d === undefined ? '' : ` /${num(d, 0)}°`, 'tdrv');
  }
  const mm = temps.motor_max, dm = temps.driver_max;
  $('tempHint').textContent = mm === null || mm === undefined
    ? `（设备状态 ${ago(temps.age_s)} 前）`
    : `电机 max ${num(mm, 1)}°C · 驱动器 max ${num(dm, 1)}°C · ${ago(temps.age_s)} 前`;
}

function renderFaults(s) {
  const box = $('faults');
  const f = s.faults || [];
  if (!f.length) { box.className = 'faults hidden'; return; }
  const SEV = { 3: 'WARN', 4: 'ERROR', 5: 'FATAL' };
  box.className = 'faults';
  box.innerHTML = `<b>活跃故障 ${f.length} 条</b>` + f.map(x =>
    `<div>　[${esc(SEV[x.severity] || x.severity)}] 0x${(x.code || 0).toString(16).padStart(4, '0')} ` +
    `${esc(x.name || '?')} ${esc((x.resources || []).join(','))}</div>`).join('');
}

function log(events) {
  if (!events || !events.length) return;
  const box = $('log');
  let added = false;
  for (const e of events) {
    if (e.id <= lastEventId) continue;
    lastEventId = Math.max(lastEventId, e.id);
    const d = document.createElement('div');
    const t = new Date(e.ts * 1000);
    d.innerHTML = `<time>${t.toTimeString().slice(0, 8)}</time>` +
      `<span class="${esc(e.level || 'info')}">${esc(e.text)}</span>`;
    box.appendChild(d);
    added = true;
  }
  if (!added) return;
  while (box.children.length > EV_MAX) box.removeChild(box.firstChild);
  box.scrollTop = box.scrollHeight;
}

/* ---------------------------------------------------------------- 主渲染 */
function render(s) {
  const app = s.app || {}, robot = s.robot || {}, cam = s.camera || {};
  const vel = s.vel || {}, bat = s.battery || {}, det = s.det || {};

  $('appTag').textContent = app.name || '—';
  $('modeTag').textContent = app.go ? '真实控制' : '干跑';

  // 顶部条
  setText('sCtlHz', num(app.control_hz, 1));
  setText('sDetHz', num(app.det_hz, 1));
  setText('sDetMs', num(app.det_ms, 0));
  setText('sMotion', robot.motion_state === null || robot.motion_state === undefined
    ? '—' : `${robot.motion_state} ${robot.motion_name || ''}`);
  setText('sGait', robot.gait === null || robot.gait === undefined
    ? '—' : `0x${Number(robot.gait).toString(16)} ${robot.gait_name || ''}`);
  setText('sMode', robot.mode === null || robot.mode === undefined
    ? '—' : `${robot.mode} ${robot.mode_name || ''}`);
  setText('sCamFps', num(cam.fps, 1));
  setText('sHit', cam.hit_rate === null || cam.hit_rate === undefined
    ? '—' : `${(cam.hit_rate * 100).toFixed(0)}%`);
  setText('sTempMax', num((s.temps || {}).motor_max, 0),
    tempClass((s.temps || {}).motor_max));
  setText('sBat', bat.BatteryLevel === undefined ? '—' : String(bat.BatteryLevel));

  // 控制程序（后端决定显示什么，前端只管画 —— 两个主程序共用这套界面）
  $('appHint').textContent = app.uptime_s === undefined ? '' : `已运行 ${ago(app.uptime_s)}`;
  renderKv('detRows', det.rows || []);

  // 相机
  $('camHint').textContent = cam.ok ? '' : '⚠ 不正常';
  renderKv('camRows', [
    { k: '状态', v: cam.ok ? '通畅' : '中断', lvl: cam.ok ? 'ok' : 'err' },
    { k: '帧率', v: `${num(cam.fps, 1)} Hz` },
    { k: '最新一帧', v: cam.last_frame_age_s === undefined ? '—' : `${ago(cam.last_frame_age_s)} 前`,
      lvl: cam.last_frame_age_s > 0.5 ? 'warn' : '' },
    { k: '分辨率', v: cam.width ? `${cam.width}×${cam.height}` : '—' },
    { k: '编码', v: cam.encoding || '—' },
    { k: '累计帧数', v: cam.frames === undefined ? '—' : String(cam.frames) },
    { k: '命中 / 命中率', v: cam.frames === undefined ? '—'
        : `${cam.hits} / ${(100 * (cam.hit_rate || 0)).toFixed(0)}%` },
  ]);

  // 机器人
  renderKv('robotRows', [
    { k: '状态上报', v: robot.status_age_s === undefined ? '—' : `${ago(robot.status_age_s)} 前`,
      lvl: (robot.status_age_s || 0) > 1.2 ? 'err' : 'ok' },
    { k: 'MotionState', v: robot.motion_state === null || robot.motion_state === undefined
        ? '—' : `${robot.motion_state} ${robot.motion_name || ''}` },
    { k: 'Gait', v: robot.gait === null || robot.gait === undefined
        ? '—' : `0x${Number(robot.gait).toString(16)} ${robot.gait_name || ''}` },
    { k: 'ControlUsageMode', v: robot.mode === null || robot.mode === undefined
        ? '—' : `${robot.mode} ${robot.mode_name || ''}` },
    { k: 'HES 硬急停', v: robot.hes === 1 ? '已触发' : (robot.hes === 0 ? '正常' : '—'),
      lvl: robot.hes === 1 ? 'err' : 'ok' },
    { k: 'Charge', v: String(robot.charge) },
    { k: '型号 / 版本', v: `${robot.model || '—'} / ${robot.version || '—'}` },
    { k: '温度上报', v: (s.temps || {}).age_s === undefined ? '—' : `${ago((s.temps || {}).age_s)} 前` },
  ]);

  // 速度
  renderKv('velRows', [
    { k: 'LinearX 前进', v: `${num(vel.LinearX, 3)} m/s` },
    { k: 'LinearY 侧移', v: `${num(vel.LinearY, 3)} m/s` },
    { k: 'LinearZ 升降', v: `${num(vel.LinearZ, 3)} m/s` },
    { k: 'AngularZ 转向', v: `${num(vel.AngularZ, 3)} rad/s` },
    { k: 'Roll / Pitch', v: `${num(vel.Roll, 2)} / ${num(vel.Pitch, 2)}` },
    { k: 'Yaw', v: num(vel.Yaw, 2) },
  ]);

  // 电池
  renderKv('batRows', [
    { k: '电量', v: bat.BatteryLevel === undefined ? '—' : `${bat.BatteryLevel} %`,
      lvl: bat.BatteryLevel !== undefined && bat.BatteryLevel < 30 ? 'warn' : '' },
    { k: '电压', v: bat.Voltage === undefined ? '—' : `${num(bat.Voltage, 2)} V` },
    { k: '电池温度', v: bat.battery_temperature === undefined
        ? '—' : `${num(bat.battery_temperature, 1)} °C` },
    { k: '充电中', v: bat.charge ? '是' : '否' },
  ]);

  renderTemps(s.temps || {});
  renderFaults(s);
  log(app.events);
}

/* ---------------------------------------------------------------- 轮询 */
let failures = 0;
async function tick() {
  try {
    const r = await fetch('/api/state', { cache: 'no-store' });
    if (!r.ok) throw new Error(`HTTP ${r.status}`);
    render(await r.json());
    failures = 0;
    $('connDot').className = 'dot on';
    $('connText').textContent = '已连接';
  } catch (e) {
    failures++;
    $('connDot').className = 'dot off';
    $('connText').textContent = `连接断开（${failures}）`;
  }
  setTimeout(tick, POLL_MS);
}
tick();

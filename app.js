import {
  runExperiment, calibrate, replay, summarize, conformalBound, protectiveDistance, POLICIES, BINS,
  SSM, DT, SENSOR, mulberry32, gauss, binOf,
} from './core/ssm.mjs';

// ------------------------------------------------------------ experiment ---
const calSeeds = Array.from({ length: 30 }, (_, i) => 1000 + i);
const testSeeds = Array.from({ length: 30 }, (_, i) => 5000 + i);
const ALPHAS = [0.0005, 0.001, 0.005, 0.01];
const t0 = performance.now();
const E = runExperiment({ calSeeds, testSeeds, alpha: 0.001 });
const buildMs = performance.now() - t0;
const calRecs = { position: E.cal.flatMap((s) => s.recs.position.flat()), doppler: E.cal.flatMap((s) => s.recs.doppler.flat()) };
const state = { alpha: 0.001, frame: 0, playing: false };
let M; // current metrics

function evaluate(alpha) {
  const calib = { position: calibrate(calRecs.position, alpha), doppler: calibrate(calRecs.doppler, alpha) };
  const run = (mode, fn) => E.test.map((s) => replay(s.session, s.recs[mode], fn));
  const runs = {
    static: run('doppler', POLICIES.static.bound),
    naive: run('doppler', POLICIES.naive.bound),
    confPos: run('position', (r) => conformalBound(calib.position, r.vhat)),
    confDop: run('doppler', (r) => conformalBound(calib.doppler, r.vhat)),
  };
  const res = Object.fromEntries(Object.entries(runs).map(([k, v]) => [k, summarize(v)]));
  return { calib, runs, res };
}

// ------------------------------------------------------------ formatting ---
const $ = (s) => document.querySelector(s);
const pct = (v, d = 1) => `${(100 * v).toFixed(d)}%`;
const pctSmall = (v) => (v < 0.01 ? `${(100 * v).toFixed(2)}%` : `${(100 * v).toFixed(1)}%`);
const fmtA = (a) => `${+(100 * a).toFixed(2)}%`;
const clock = (k) => { const s = k * DT; return `${String(Math.floor(s / 60)).padStart(2, '0')}:${(s % 60).toFixed(1).padStart(4, '0')}`; };

function renderAlpha() {
  $('#alpha').innerHTML = ALPHAS.map((a) => `<button data-a="${a}" class="${a === state.alpha ? 'on' : ''}">≤ ${fmtA(a)}</button>`).join('');
  document.querySelectorAll('#alpha button').forEach((b) => b.addEventListener('click', () => { state.alpha = +b.dataset.a; refresh(); }));
}

function renderPanels() {
  const { static: st, naive, confPos, confDop } = M.res;
  const shiftHours = (st.downtime - confDop.downtime) * 8;
  $('#hours').textContent = `${testSeeds.length} recordings · ${st.hours.toFixed(1)} h · ${st.frames.toLocaleString('en-US')} person-frames`;
  $('#kpis').innerHTML = `
    <div class="kpi hl"><div class="l">Robot downtime</div><div class="v"><span class="from">${pct(st.downtime)}</span>${pct(confDop.downtime)}</div><div class="s">vs. the 1.6 m/s default, same recordings</div></div>
    <div class="kpi"><div class="l">Under-protected frames</div><div class="v ${confDop.underRate <= state.alpha * 1.25 ? '' : 'warn'}">${pctSmall(confDop.underRate)}</div><div class="s">target ≤ ${fmtA(state.alpha)} · 1.6 m/s default: ${pctSmall(st.underRate)}</div></div>
    <div class="kpi"><div class="l">Robot time recovered</div><div class="v">${shiftHours.toFixed(1)} h</div><div class="s">per 8 h shift at this dock</div></div>
    <div class="kpi"><div class="l">Doppler vs. position tracking</div><div class="v">${pct(confPos.downtime)}<span class="s"> → </span>${pct(confDop.downtime)}</div><div class="s">downtime at the same target rate</div></div>`;
  const rows = [
    ['static', '#fbbf24', 'ISO 13855 default, 1.6 m/s', st],
    ['naive', '#f43f5e', 'Measured speed, no margin', naive],
    ['confPos', '#a855f7', 'Conformal · position-only tracking', confPos],
    ['confDop', '#46ffc7', 'Conformal · LyteVision Doppler', confDop],
  ];
  $('#policies').innerHTML = rows.map(([k, c, name, r]) => {
    const cls = r.underRate > Math.max(0.01, state.alpha * 5) ? 'bad' : r.underRate > state.alpha * 1.25 ? 'warn' : 'ok';
    return `<tr class="${k === 'confDop' ? 'hl' : ''}"><td><span class="sw" style="background:${c}"></span>${name}</td><td class="n">${pct(r.downtime)}</td><td class="n">${r.stopsPerHour.toFixed(0)}</td><td class="n ${cls}">${pctSmall(r.underRate)}</td></tr>`;
  }).join('');
  const qmax = Math.max(...M.calib.position.q, ...M.calib.doppler.q, 0.1);
  $('#calN').innerHTML = `<i class="key pos"></i>position-only <i class="key dop"></i>LyteVision Doppler`;
  $('#calFrames').textContent = M.calib.doppler.n.reduce((a, b) => a + b, 0).toLocaleString('en-US');
  $('#margins').innerHTML = BINS.map((b, i) => `
    <span class="nm">${b.name}</span>
    <span class="bars">
      <span class="b pos"><span style="width:${(100 * M.calib.position.q[i] / qmax).toFixed(1)}%"></span><em>+${M.calib.position.q[i].toFixed(2)} m/s</em></span>
      <span class="b dop"><span style="width:${(100 * M.calib.doppler.q[i] / qmax).toFixed(1)}%"></span><em>+${M.calib.doppler.q[i].toFixed(2)} m/s</em></span>
    </span>`).join('');
}

// ------------------------------------------------------------- timeline ---
const view = E.test[0];
const N = view.session.frames;
function renderLanes() {
  const lanes = [['static', 'static', '1.6 m/s default'], ['pos', 'confPos', 'Conformal · position'], ['dop', 'confDop', 'Conformal · Doppler']];
  $('#lanes').innerHTML = lanes.map(([cls, key, name]) => `<div class="lane ${cls}"><span>${name}</span><div class="bar">${M.runs[key][0].stops.map(([a, b]) => `<span class="seg2" style="left:${(100 * a / N).toFixed(3)}%;width:${Math.max(0.15, 100 * (b - a) / N).toFixed(3)}%"></span>`).join('')}</div></div>`).join('') + '<div class="playhead" id="ph"></div>';
  const lanesEl = $('#lanes');
  lanesEl.onclick = (ev) => {
    const bar = lanesEl.querySelector('.bar').getBoundingClientRect();
    state.frame = Math.max(0, Math.min(N - 1, Math.round(((ev.clientX - bar.left) / bar.width) * N))); draw();
  };
}
const stoppedAt = (key, k) => M.runs[key][0].stops.some(([a, b]) => k >= a && k < b);

// --------------------------------------------------------------- scene ---
const canvas = $('#scene');
const ctx = canvas.getContext('2d');
let W = 0, H = 0, DPR = 1;
const eye = [-7.4, 11.6, 6.4], target = [0.2, 3.3, 0.3];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (a) => { const l = Math.hypot(...a); return a.map((v) => v / l); };
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const fwd = norm(sub(target, eye)), rgt = norm(cross(fwd, [0, 0, 1])), up = cross(rgt, fwd);
let F = 1;
function P(x, y, z) {
  const d = [x - eye[0], y - eye[1], z - eye[2]];
  const zc = dot(d, fwd);
  return [W / 2 + (F * dot(d, rgt)) / zc, H * 0.47 - (F * dot(d, up)) / zc, zc];
}
function resize() {
  DPR = Math.min(2, window.devicePixelRatio || 1);
  const r = canvas.getBoundingClientRect(); W = r.width; H = r.height;
  canvas.width = Math.round(W * DPR); canvas.height = Math.round(H * DPR);
  ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
  F = W * 0.95;
}
function line(a, b, style, w = 1, dash = null) {
  const p = P(...a), q = P(...b);
  ctx.strokeStyle = style; ctx.lineWidth = w; ctx.setLineDash(dash || []);
  ctx.beginPath(); ctx.moveTo(p[0], p[1]); ctx.lineTo(q[0], q[1]); ctx.stroke(); ctx.setLineDash([]);
}
function poly(pts, stroke, fill, w = 1, dash = null) {
  ctx.beginPath(); pts.forEach((v, i) => { const p = P(...v); i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]); }); ctx.closePath();
  if (fill) { ctx.fillStyle = fill; ctx.fill(); }
  if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = w; ctx.setLineDash(dash || []); ctx.stroke(); ctx.setLineDash([]); }
}
// turbo-ish ramp by range, like LyteVision range renders
function turbo(t) {
  t = Math.max(0, Math.min(1, t));
  const r = Math.round(255 * Math.max(0, Math.min(1, 1.6 * t * t * (3 - 2 * t) + (t > 0.75 ? 0.4 : 0))));
  const g = Math.round(255 * Math.max(0, Math.min(1, Math.sin(Math.PI * t) * 1.05)));
  const b = Math.round(255 * Math.max(0, Math.min(1, 1 - 1.5 * t)));
  return `rgb(${r},${g},${b})`;
}
// static scene points: pallet stacks and racking, colored by range from the sensor
const rngScene = mulberry32(77);
const PALLETS = [[-5, 8.5], [-2.5, 9.6], [2.5, 9.6], [5, 8.5], [-5.6, 4.2], [5.6, 4.2], [-3.4, 1.4], [3.6, 1.6]];
const scenePts = [];
for (const [cx, cy] of PALLETS) {
  for (let i = 0; i < 520; i++) {
    const face = rngScene();
    let x, y, z;
    if (face < 0.34) { x = cx - 0.6 + 1.2 * rngScene(); y = cy - 0.5 + rngScene(); z = 1.25; }
    else if (face < 0.67) { x = cx - 0.6 + 1.2 * rngScene(); y = cy + 0.5; z = 1.25 * rngScene(); }
    else { x = cx - 0.6; y = cy - 0.5 + rngScene(); z = 1.25 * rngScene(); }
    scenePts.push([x, y, z, Math.hypot(x - SENSOR.x, y - SENSOR.y, z - SENSOR.z)]);
  }
}
// body point template (local frame: x lateral, y forward, z up)
const body = [];
const rb = mulberry32(5);
for (let i = 0; i < 260; i++) {
  const u = rb(); let x, y, z, part;
  if (u < 0.12) { const th = rb() * 2 * Math.PI, ph = Math.acos(2 * rb() - 1); x = 0.1 * Math.sin(ph) * Math.cos(th); y = 0.1 * Math.sin(ph) * Math.sin(th); z = 1.62 + 0.11 * Math.cos(ph); part = 'head'; }
  else if (u < 0.52) { const th = rb() * 2 * Math.PI; x = 0.19 * Math.cos(th); y = 0.12 * Math.sin(th); z = 0.95 + 0.55 * rb(); part = 'torso'; }
  else if (u < 0.82) { const side = rb() < 0.5 ? -1 : 1, th = rb() * 2 * Math.PI; x = side * 0.1 + 0.06 * Math.cos(th); y = 0.06 * Math.sin(th); z = 0.95 * rb(); part = side < 0 ? 'legL' : 'legR'; }
  else { const side = rb() < 0.5 ? -1 : 1, th = rb() * 2 * Math.PI; x = side * 0.26 + 0.04 * Math.cos(th); y = 0.04 * Math.sin(th); z = 0.85 + 0.55 * rb(); part = side < 0 ? 'armL' : 'armR'; }
  body.push({ x, y, z, part });
}
function personPoints(s, k, rng) {
  const sp = Math.hypot(s.vx, s.vy), hx = sp > 0.05 ? s.vx / sp : 0, hy = sp > 0.05 ? s.vy / sp : 1;
  const ph = k * 0.9 * Math.min(1.6, sp + 0.2);
  return body.map((b) => {
    let swing = 0;
    if (b.part === 'legL' || b.part === 'armR') swing = Math.sin(ph) * sp * 0.35 * (b.part.startsWith('leg') ? (0.95 - b.z) : (1.4 - b.z));
    if (b.part === 'legR' || b.part === 'armL') swing = -Math.sin(ph) * sp * 0.35 * (b.part.startsWith('leg') ? (0.95 - b.z) : (1.4 - b.z));
    const ly = b.y + swing;
    const wx = s.x + b.x * hy + ly * hx, wy = s.y - b.x * hx + ly * hy, wz = b.z;
    const dx = wx - SENSOR.x, dy = wy - SENSOR.y, dz = wz - SENSOR.z, r = Math.hypot(dx, dy, dz);
    const limb = (b.part === 'torso' || b.part === 'head') ? 0 : Math.cos(ph) * sp * 0.6 * (b.part.startsWith('leg') ? 1 - b.z : 0.6);
    const vx = s.vx + limb * hx, vy = s.vy + limb * hy;
    const radial = (dx * vx + dy * vy) / r + 0.03 * gauss(rng);
    return [wx, wy, wz, radial];
  });
}
function dopplerColor(radial) { // approaching (negative radial) = red, receding = blue
  const a = Math.min(1, Math.abs(radial) / 1.6);
  if (radial < -0.08) return `rgba(${244},${Math.round(90 - 40 * a)},${Math.round(110 - 60 * a)},${0.55 + 0.45 * a})`;
  if (radial > 0.08) return `rgba(${Math.round(80 - 30 * a)},${Math.round(150 + 20 * a)},255,${0.55 + 0.45 * a})`;
  return 'rgba(186,230,253,.75)';
}
function ring(r, style, w, dash) {
  const pts = Array.from({ length: 73 }, (_, i) => { const t = (i / 72) * Math.PI * 2; return [SENSOR.x + r * Math.cos(t), SENSOR.y + Math.max(-0.2, r * Math.sin(t)), 0.01]; });
  ctx.strokeStyle = style; ctx.lineWidth = w; ctx.setLineDash(dash || []);
  ctx.beginPath(); pts.forEach((v, i) => { const p = P(...v); i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]); }); ctx.stroke(); ctx.setLineDash([]);
}
function box(cx, cy, w, d, h, style, wdt = 1) {
  const x0 = cx - w / 2, x1 = cx + w / 2, y0 = cy - d / 2, y1 = cy + d / 2;
  const c = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
  for (let i = 0; i < 4; i++) {
    const [a, b] = [c[i], c[(i + 1) % 4]];
    line([a[0], a[1], 0], [b[0], b[1], 0], style, wdt); line([a[0], a[1], h], [b[0], b[1], h], style, wdt); line([a[0], a[1], 0], [a[0], a[1], h], style, wdt);
  }
}
let placed = [];
function label(x, y, z, lines, color, warn, dx = 0) {
  const p = P(x, y, z);
  ctx.font = '600 10.5px Inter, sans-serif';
  const wmax = Math.max(...lines.map((l, i) => { ctx.font = i ? '500 10.5px "JetBrains Mono", monospace' : '600 10.5px Inter, sans-serif'; return ctx.measureText(l).width; })) + 14;
  const hgt = 8 + lines.length * 14;
  const base = p[1] - hgt - 6 - (dx ? 18 : 0);
  const cands = [[p[0] - wmax / 2 + dx, base], [p[0] - wmax - 14, base + 10], [p[0] + 14, base + 10], [p[0] - wmax / 2 + dx, base + hgt + 20]];
  const free = (x, y) => y > 40 && x > 4 && x + wmax < W - 4 && !placed.some((r) => x < r[0] + r[2] + 4 && x + wmax + 4 > r[0] && y < r[1] + r[3] + 4 && y + hgt + 4 > r[1]);
  let [bx, by] = cands.find(([x, y]) => free(x, y)) || cands[0];
  placed.push([bx, by, wmax, hgt]);
  ctx.fillStyle = 'rgba(2,5,22,.82)'; ctx.strokeStyle = warn ? 'rgba(251,191,36,.9)' : 'rgba(94,140,135,.85)'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.roundRect(bx, by, wmax, hgt, 4); ctx.fill(); ctx.stroke();
  const ax = Math.max(bx + 8, Math.min(bx + wmax - 8, p[0])), ay = by + hgt < p[1] ? by + hgt : by;
  ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(p[0], p[1]); ctx.stroke();
  lines.forEach((l, i) => { ctx.font = i ? '500 10.5px "JetBrains Mono", monospace' : '600 10px Inter, sans-serif'; ctx.fillStyle = i ? '#e2e8f0' : (warn ? '#fbbf24' : '#9fd8cd'); ctx.fillText(i ? l : l.split('').join(String.fromCharCode(8202)), bx + 7, by + 15 + i * 14); });
}

function draw() {
  const k = state.frame;
  ctx.clearRect(0, 0, W, H);
  placed = [[0, -40, W, 78]]; // keep tags clear of the HUD row
  // floor grid
  for (let x = -7; x <= 7; x++) line([x, 0, 0], [x, 11.5, 0], 'rgba(34,211,238,.13)');
  for (let y = 0; y <= 11.5; y += 1) line([-7, y, 0], [7, y, 0], 'rgba(34,211,238,.13)');
  // painted walkway lines and the loader's travel lane, as in Lyte's dock renders
  line([-2.4, 2.6, 0.005], [-2.4, 11.5, 0.005], 'rgba(250,204,21,.55)', 2); line([2.4, 2.6, 0.005], [2.4, 11.5, 0.005], 'rgba(250,204,21,.55)', 2);
  line([0, 1.6, 0.005], [0, 11.5, 0.005], 'rgba(34,211,238,.55)', 2, [10, 8]);
  // dock wall + door + trailer
  poly([[-7, 0, 0], [-1.45, 0, 0], [-1.45, 0, 3.1], [1.45, 0, 3.1], [1.45, 0, 0], [7, 0, 0], [7, 0, 4.2], [-7, 0, 4.2]], 'rgba(148,163,184,.25)', 'rgba(15,23,42,.55)');
  poly([[-1.3, -0.1, 0], [1.3, -0.1, 0], [1.3, -9, 0], [-1.3, -9, 0]], null, 'rgba(30,41,59,.5)');
  line([-1.3, -0.1, 2.8], [-1.3, -9, 2.8], 'rgba(148,163,184,.35)'); line([1.3, -0.1, 2.8], [1.3, -9, 2.8], 'rgba(148,163,184,.35)');
  ctx.fillStyle = 'rgba(148,163,184,.55)'; ctx.font = '600 20px Inter, sans-serif'; const dl = P(0, 0, 3.55); ctx.fillText('23', dl[0] - 11, dl[1]);
  // static points
  for (const [x, y, z, r] of scenePts) { const p = P(x, y, z); ctx.fillStyle = turbo(1 - (r - 1) / 11); ctx.fillRect(p[0], p[1], 1.6, 1.6); }
  // protective zones: 1.6 m/s default (constant) and conformal (driven by the closest person)
  const recsK = view.recs.doppler.map((recs) => recs.find((r) => r.k === k)).filter(Boolean);
  const sStatic = protectiveDistance(SSM.vStatic);
  let crit = null;
  for (const r of recsK) { const sp = protectiveDistance(conformalBound(M.calib.doppler, r.vhat)); if (!crit || r.d - sp < crit.d - crit.sp) crit = { ...r, sp }; }
  ring(sStatic, 'rgba(251,191,36,.85)', 1.5, [6, 5]);
  const sConf = crit ? crit.sp : protectiveDistance(0);
  const g = ctx.createRadialGradient(...P(0, 0, 0).slice(0, 2), 0, ...P(0, 0, 0).slice(0, 2), 200);
  g.addColorStop(0, 'rgba(70,255,199,.10)'); g.addColorStop(1, 'rgba(70,255,199,0)');
  ring(sConf, 'rgba(70,255,199,.95)', 2);
  // robot (autonomous trailer loader) at the hazard point
  box(0, 0.75, 1.1, 1.3, 1.05, 'rgba(70,255,199,.8)', 1.2);
  line([0, 0.75, 1.05], [0, 0.75, 1.9], 'rgba(70,255,199,.8)', 1.2); line([0, 0.75, 1.9], [0, 1.6, 1.6], 'rgba(70,255,199,.8)', 1.2);
  const sensorP = P(SENSOR.x, 0.2, SENSOR.z); ctx.fillStyle = '#22d3ee'; ctx.beginPath(); ctx.arc(sensorP[0], sensorP[1], 3, 0, 7); ctx.fill();
  const robotStopped = stoppedAt('confDop', k);
  label(0.55, 0.2, 1.05, ['ROBOT', robotStopped ? 'protective stop' : `running · v_r ${SSM.vr.toFixed(1)} m/s`], '#46ffc7', robotStopped, 70);
  // people: Doppler-colored point clouds, boxes and tags
  const rng = mulberry32(1000 + k);
  const persons = view.session.persons.map((p, i) => ({ i, s: p[k], est: view.recs.doppler[i].find((r) => r.k === k) }));
  persons.sort((a, b) => Math.hypot(b.s.x - eye[0], b.s.y - eye[1]) - Math.hypot(a.s.x - eye[0], a.s.y - eye[1]));
  for (const { i, s, est } of persons) {
    const pts = personPoints(s, k, rng);
    for (const [x, y, z, rad] of pts) { const p = P(x, y, z); ctx.fillStyle = dopplerColor(rad); ctx.fillRect(p[0] - 1, p[1] - 1, 2.2, 2.2); }
    box(s.x, s.y, 0.62, 0.62, 1.85, 'rgba(34,211,238,.55)', 1);
    if (est) {
      const b = conformalBound(M.calib.doppler, est.vhat), sp = protectiveDistance(b);
      const inside = est.d < sp;
      label(s.x, s.y, 1.95, [`PERSON ${i + 1}`, `${est.d.toFixed(2)} m · ${Math.abs(est.vhat) < 0.05 ? 'still' : est.vhat > 0 ? '→ ' + est.vhat.toFixed(2) + ' m/s' : '← ' + (-est.vhat).toFixed(2) + ' m/s'}`, `bound ${b.toFixed(2)} · S_p ${sp.toFixed(2)} m`], '#22d3ee', inside);
    } else {
      label(s.x, s.y, 1.95, [`PERSON ${i + 1}`, 'out of range'], '#64748b', false);
    }
  }
  // HUD
  const st = stoppedAt('static', k), cd = stoppedAt('confDop', k);
  $('#hud').innerHTML = `<span class="chip">t <b>${clock(k)}</b></span><span class="chip ${st ? 'stop' : ''}">1.6 m/s policy <b>${st ? 'STOPPED' : 'RUNNING'}</b></span><span class="chip ${cd ? 'stop' : ''}">Conformal · Doppler <b>${cd ? 'STOPPED' : 'RUNNING'}</b></span><span class="chip">S<sub>p</sub> <b>${sConf.toFixed(2)} m</b> vs ${sStatic.toFixed(2)} m</span>`;
  $('#clock').textContent = clock(k);
  const bar = document.querySelector('#lanes .bar');
  if (bar) { const lanesR = $('#lanes').getBoundingClientRect(), br = bar.getBoundingClientRect(); $('#ph').style.left = `${br.left - lanesR.left + (k / N) * br.width}px`; }
}

// Opening frame: the first moment the 1.6 m/s policy has stopped the robot while
// the conformal policy keeps it running with a person close by.
function interestingFrame() {
  // the 1.6 m/s policy has stopped the robot, the conformal one keeps it running,
  // and a person is close: pick the frame where the two zones differ the most.
  let best = 0, gap = -1;
  for (let k = 150; k < N; k++) {
    if (!stoppedAt('static', k) || stoppedAt('confDop', k)) continue;
    for (const recs of view.recs.doppler) {
      const r = recs.find((x) => x.k === k);
      if (!r || r.d > 1.9) continue;
      const g = protectiveDistance(SSM.vStatic) - protectiveDistance(conformalBound(M.calib.doppler, r.vhat));
      if (g > gap + 0.05) { gap = g; best = k; }
    }
  }
  return best;
}

function refresh() {
  M = evaluate(state.alpha);
  renderAlpha(); renderPanels(); renderLanes(); draw();
  if (window.lucide) window.lucide.createIcons();
}
let last = 0;
function loop(ts) {
  if (state.playing && ts - last >= DT * 1000) { last = ts; state.frame = (state.frame + 1) % N; draw(); }
  requestAnimationFrame(loop);
}
$('#play').addEventListener('click', () => {
  state.playing = !state.playing;
  $('#play').innerHTML = `<i data-lucide="${state.playing ? 'pause' : 'play'}"></i>`; window.lucide && window.lucide.createIcons();
});
window.addEventListener('resize', () => { resize(); draw(); });
resize();
M = evaluate(state.alpha);
state.frame = interestingFrame();
refresh();
$('#play').innerHTML = '<i data-lucide="play"></i>'; window.lucide && window.lucide.createIcons();
requestAnimationFrame(loop);
window.__lyte = { E, get M() { return M; }, buildMs, state };
void binOf; void ALPHAS;

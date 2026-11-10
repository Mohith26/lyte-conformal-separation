// Conformal speed & separation monitoring (SSM) from 4D coherent (Doppler) perception.
//
// ISO/TS 15066 lets the human term of the protective separation distance use a
// measured "directed speed" instead of the ISO 13855 default of 1.6 m/s. A
// measured speed is only useful for safety if it bounds how fast the person can
// close distance during the robot's reaction + stopping time. This module:
//   1. simulates people moving in a trailer-loading dock (seeded, synthetic),
//   2. measures them like a coherent 4D sensor would (centroid + radial velocity),
//   3. tracks them with a position-only Kalman filter and a Doppler-aided one,
//   4. calibrates a distribution-free (split conformal, Mondrian by speed state)
//      upper bound on closing speed over the stopping horizon,
//   5. replays held-out sessions and counts protective stops and under-protected
//      frames for each policy.
// Pure ES module, no dependencies; runs in the browser and under `node --test`.

// ---------------------------------------------------------------- params ---
export const SSM = {
  Tr: 0.15, // s, system reaction time (sense + decide)
  Ts: 0.45, // s, robot stopping time
  vr: 1.0, // m/s, robot directed speed toward the person
  Ss: 0.25, // m, robot stopping distance
  C: 0.20, // m, intrusion distance for whole-body detection
  Zd: 0.05, // m, position uncertainty of the sensing system
  Zr: 0.02, // m, robot position uncertainty
  vStatic: 1.6, // m/s, ISO 13855 default human approach speed
};
export const HORIZON = SSM.Tr + SSM.Ts; // s
export const DT = 0.1; // s, 10 Hz safety loop
export const RANGE = 8; // m, people farther than this are not scored
export const SENSOR = { x: 0, y: 0, z: 1.2 }; // mounted on the loader, at the hazard point

// Protective separation distance S_p = S_h + S_r + S_s + C + Z_d + Z_r (ISO/TS 15066 5.5.4).
export function protectiveDistance(vh, p = SSM) {
  return Math.max(0, vh) * (p.Tr + p.Ts) + p.vr * p.Tr + p.Ss + p.C + p.Zd + p.Zr;
}

// ------------------------------------------------------------------ rng ----
export function mulberry32(seed) {
  let t = seed >>> 0;
  return function () {
    t += 0x6d2b79f5;
    let r = Math.imul(t ^ (t >>> 15), 1 | t);
    r ^= r + Math.imul(r ^ (r >>> 7), 61 | r);
    return ((r ^ (r >>> 14)) >>> 0) / 4294967296;
  };
}
export function gauss(rng) { let u = 0; while (u === 0) u = rng(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * rng()); }
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));

// ------------------------------------------------------- dock simulation ---
// Dock floor: x in [-7, 7] m, y in [0.6, 11] m; the loader sits at the trailer door (0, 0).
export const DOCK = { xmin: -7, xmax: 7, ymin: 0.6, ymax: 11 };
const STAGING = [[-5, 8.5], [-2.5, 9.5], [2.5, 9.5], [5, 8.5], [-5.5, 4], [5.5, 4], [-3, 6], [3, 6], [0, 10.2]];

// One worker's trajectory: waypoints with walk/slow/hurried legs, pauses, and
// visits close to the loader (staging a pallet, checking the trailer).
export function simulatePerson(rng, frames) {
  const traj = new Array(frames);
  let x = DOCK.xmin + rng() * (DOCK.xmax - DOCK.xmin), y = 3 + rng() * 7;
  let vx = 0, vy = 0, pause = rng() * 30, tx = x, ty = y, speed = 0, tau = 0.45;
  for (let f = 0; f < frames; f++) {
    if (pause > 0) {
      pause -= 1;
      if (pause <= 0) { // choose the next leg
        const u = rng();
        if (u < 0.3) { // walk up to the loader (inspect / stage), stop 0.9-1.8 m away
          const ang = (-0.5 + rng()) * 1.6, r = 0.9 + rng() * 0.9;
          tx = r * Math.sin(ang); ty = Math.max(0.7, r * Math.cos(ang));
        } else { const s = STAGING[Math.floor(rng() * STAGING.length)]; tx = s[0] + gauss(rng) * 0.4; ty = s[1] + gauss(rng) * 0.4; }
        const m = rng();
        speed = m < 0.72 ? clamp(1.25 + 0.2 * gauss(rng), 0.8, 1.65) : m < 0.94 ? clamp(0.7 + 0.15 * gauss(rng), 0.35, 1.0) : clamp(1.95 + 0.25 * gauss(rng), 1.6, 2.6);
        tau = speed > 1.6 ? 0.3 : 0.45; // hurried people accelerate harder
      }
    } else {
      const dx = tx - x, dy = ty - y, d = Math.hypot(dx, dy);
      if (d < 0.15 && Math.hypot(vx, vy) < 0.2) { pause = 10 + Math.floor(rng() * 80); }
      const want = d < 0.15 ? 0 : Math.min(speed, 1.3 * Math.sqrt(Math.max(0, d - 0.1)));
      const cx = d > 1e-6 ? (dx / d) * want : 0, cy = d > 1e-6 ? (dy / d) * want : 0;
      let ax = (cx - vx) / tau, ay = (cy - vy) / tau;
      const am = Math.hypot(ax, ay), amax = speed > 1.6 ? 3.0 : 2.0;
      if (am > amax) { ax *= amax / am; ay *= amax / am; }
      vx += ax * DT; vy += ay * DT;
    }
    if (pause > 0) { vx *= 0.5; vy *= 0.5; } // settle while standing
    // gait sway: small lateral oscillation and speed ripple
    const sway = 0.04 * Math.sin(f * 0.9 + x);
    x = clamp(x + vx * DT, DOCK.xmin, DOCK.xmax); y = clamp(y + vy * DT, DOCK.ymin, DOCK.ymax);
    traj[f] = { x: x + sway * (vy / (Math.hypot(vx, vy) + 1e-9)), y: y - sway * (vx / (Math.hypot(vx, vy) + 1e-9)), vx, vy };
  }
  return traj;
}

export function simulateSession(seed, { minutes = 5, people = 3 } = {}) {
  const rng = mulberry32(seed);
  const frames = Math.round((minutes * 60) / DT);
  const persons = Array.from({ length: people }, () => simulatePerson(rng, frames));
  return { seed, frames, persons };
}

// ------------------------------------------------------- sensing model ---
// What a coherent 4D sensor gives per person per frame after clustering:
// a centroid (limb motion and occlusion make it jitter) and the mean radial
// (Doppler) velocity of the cluster's points relative to the sensor.
export const NOISE = { pos: 0.06, doppler: 0.04 };

export function measure(person, rng, noise = NOISE) {
  return person.map((s) => {
    const zx = s.x + noise.pos * gauss(rng), zy = s.y + noise.pos * gauss(rng);
    const rx = zx - SENSOR.x, ry = zy - SENSOR.y, r = Math.hypot(rx, ry) || 1e-9;
    const radial = (rx / r) * s.vx + (ry / r) * s.vy + noise.doppler * gauss(rng);
    return { zx, zy, radial, ux: rx / r, uy: ry / r };
  });
}

// ------------------------------------------------------------- trackers ---
// Constant-velocity Kalman filter, state [x, y, vx, vy]. With useDoppler, each
// frame also fuses the scalar radial velocity measurement u . v (linear in the
// state given the measured line-of-sight unit vector u).
export function track(meas, { useDoppler, q = 2.0, noise = NOISE } = {}) {
  const out = new Array(meas.length);
  const x = new Float64Array([meas[0].zx, meas[0].zy, 0, 0]);
  const P = new Float64Array(16); // row-major 4x4
  P[0] = P[5] = noise.pos ** 2; P[10] = P[15] = 1;
  const R = noise.pos ** 2, Rd = noise.doppler ** 2, dt = DT;
  const q11 = (dt ** 4 / 4) * q, q13 = (dt ** 3 / 2) * q, q33 = dt * dt * q;
  const h = new Float64Array(4), Ph = new Float64Array(4);
  const update = (z, r) => {
    for (let i = 0; i < 4; i++) Ph[i] = P[i * 4] * h[0] + P[i * 4 + 1] * h[1] + P[i * 4 + 2] * h[2] + P[i * 4 + 3] * h[3];
    const S = h[0] * Ph[0] + h[1] * Ph[1] + h[2] * Ph[2] + h[3] * Ph[3] + r;
    const innov = z - (h[0] * x[0] + h[1] * x[1] + h[2] * x[2] + h[3] * x[3]);
    for (let i = 0; i < 4; i++) x[i] += (Ph[i] / S) * innov;
    for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) P[i * 4 + j] -= (Ph[i] * Ph[j]) / S;
  };
  for (let k = 0; k < meas.length; k++) {
    if (k > 0) { // predict: x = F x, P = F P F' + Q with F = [[I, dt I], [0, I]]
      x[0] += dt * x[2]; x[1] += dt * x[3];
      // P <- F P
      for (let j = 0; j < 4; j++) { P[j] += dt * P[8 + j]; P[4 + j] += dt * P[12 + j]; }
      // P <- P F'
      for (let i = 0; i < 4; i++) { P[i * 4] += dt * P[i * 4 + 2]; P[i * 4 + 1] += dt * P[i * 4 + 3]; }
      P[0] += q11; P[5] += q11; P[10] += q33; P[15] += q33;
      P[2] += q13; P[8] += q13; P[7] += q13; P[13] += q13;
    }
    const m = meas[k];
    h[0] = 1; h[1] = 0; h[2] = 0; h[3] = 0; update(m.zx, R);
    h[0] = 0; h[1] = 1; update(m.zy, R);
    if (useDoppler) { h[1] = 0; h[2] = m.ux; h[3] = m.uy; update(m.radial, Rd); }
    out[k] = { x: x[0], y: x[1], vx: x[2], vy: x[3] };
  }
  return out;
}

// --------------------------------------------------------- frame scoring ---
// For each frame: distance to the hazard point, the estimated directed speed
// toward it (positive = approaching), and the truth the bound must cover: the
// largest distance the person actually closes within the next HORIZON seconds,
// expressed as a speed.
export function frameRecords(person, est) {
  const n = person.length, steps = Math.round(HORIZON / DT);
  const dist = person.map((s) => Math.hypot(s.x - SENSOR.x, s.y - SENSOR.y));
  const recs = [];
  for (let k = 0; k + steps < n; k++) {
    if (dist[k] > RANGE) continue;
    let dmin = dist[k];
    for (let j = 1; j <= steps; j++) dmin = Math.min(dmin, dist[k + j]);
    const e = est[k], ex = e.x - SENSOR.x, ey = e.y - SENSOR.y, ed = Math.hypot(ex, ey) || 1e-9;
    const vhat = -(ex * e.vx + ey * e.vy) / ed; // closing speed estimate
    recs.push({ k, d: dist[k], dEst: ed, vhat, closing: (dist[k] - dmin) / HORIZON });
  }
  return recs;
}

// ------------------------------------------------------------ conformal ---
export const BINS = [
  { name: 'Standing / receding', lo: -Infinity, hi: 0.3 },
  { name: 'Walking toward', lo: 0.3, hi: 1.0 },
  { name: 'Fast toward', lo: 1.0, hi: Infinity },
];
export const binOf = (vhat) => BINS.findIndex((b) => vhat >= b.lo && vhat < b.hi);

// Finite-sample split-conformal quantile: the ceil((n+1)(1-alpha))-th smallest score.
export function conformalQuantile(scores, alpha) {
  const n = scores.length;
  const k = Math.ceil((n + 1) * (1 - alpha));
  if (k > n) return Infinity;
  const s = Float64Array.from(scores).sort();
  return s[k - 1];
}

// Mondrian calibration: one quantile per speed state, score = closing - max(0, vhat)
// (a receding person who keeps receding needs no margin; one who turns around does).
export function calibrate(records, alpha) {
  const per = BINS.map(() => []);
  for (const r of records) per[binOf(r.vhat)].push(r.closing - Math.max(0, r.vhat));
  return { alpha, q: per.map((s) => conformalQuantile(s, alpha)), n: per.map((s) => s.length) };
}
export const conformalBound = (cal, vhat) => Math.max(0, vhat) + cal.q[binOf(vhat)];

// -------------------------------------------------------------- policies ---
export const POLICIES = {
  static: { label: 'ISO 13855 default (1.6 m/s)', bound: () => SSM.vStatic },
  naive: { label: 'Measured speed, no margin', bound: (r) => Math.max(0, r.vhat) },
  conformal: { label: 'Conformal bound', bound: (r, cal) => conformalBound(cal, r.vhat) },
};

// Replay one session for one policy. A protective stop starts when any person
// is inside their protective distance; the robot restarts after the condition
// has been clear for `restart` seconds. Returns stop intervals and counters.
export function replay(session, recsByPerson, boundFn, { restart = 1.0 } = {}) {
  const n = session.frames;
  const need = new Uint8Array(n);
  let frames = 0, under = 0;
  recsByPerson.forEach((recs) => {
    for (const r of recs) {
      const b = boundFn(r);
      frames++;
      if (r.closing > b + 1e-12) under++;
      if (r.d < protectiveDistance(b)) need[r.k] = 1;
    }
  });
  const stops = [];
  let stopped = false, clear = 0, start = 0, down = 0;
  const hold = Math.round(restart / DT);
  for (let k = 0; k < n; k++) {
    if (need[k]) {
      if (!stopped) { stopped = true; start = k; }
      clear = 0;
    } else if (stopped && ++clear >= hold) { stopped = false; stops.push([start, k]); }
    if (stopped) down++;
  }
  if (stopped) stops.push([start, n]);
  return { stops, frames, under, downFrames: down, hours: (n * DT) / 3600 };
}

// ----------------------------------------------------------- experiment ---
// Simulate, measure and track a set of sessions. Seeds are deterministic.
export const TRACKER_Q = { position: 1.0, doppler: 2.0 }; // tuned on calibration seeds only

export function buildSessions(seeds, opts = {}, qs = TRACKER_Q) {
  return seeds.map((seed) => {
    const session = simulateSession(seed, opts);
    const rng = mulberry32(seed ^ 0x9e3779b9);
    const meas = session.persons.map((p) => measure(p, rng));
    const recs = {};
    for (const mode of ['position', 'doppler']) {
      recs[mode] = session.persons.map((p, i) => frameRecords(p, track(meas[i], { useDoppler: mode === 'doppler', q: qs[mode] })));
    }
    return { session, meas, recs };
  });
}

export function summarize(results) {
  const t = results.reduce((a, r) => ({ stops: a.stops + r.stops.length, frames: a.frames + r.frames, under: a.under + r.under, down: a.down + r.downFrames, hours: a.hours + r.hours, total: a.total + r.hours * 3600 / DT }), { stops: 0, frames: 0, under: 0, down: 0, hours: 0, total: 0 });
  return { stopsPerHour: t.stops / t.hours, underRate: t.under / Math.max(1, t.frames), downtime: t.down / t.total, stops: t.stops, frames: t.frames, under: t.under, hours: t.hours };
}

export function runExperiment({ calSeeds, testSeeds, alpha = 0.001, opts = {} }) {
  const cal = buildSessions(calSeeds, opts), test = buildSessions(testSeeds, opts);
  const calib = {};
  for (const mode of ['position', 'doppler']) calib[mode] = calibrate(cal.flatMap((s) => s.recs[mode].flat()), alpha);
  const evalPolicy = (mode, fn) => summarize(test.map((s) => replay(s.session, s.recs[mode], fn)));
  const res = {
    static: evalPolicy('doppler', POLICIES.static.bound),
    naive: evalPolicy('doppler', POLICIES.naive.bound),
    confPos: evalPolicy('position', (r) => POLICIES.conformal.bound(r, calib.position)),
    confDop: evalPolicy('doppler', (r) => POLICIES.conformal.bound(r, calib.doppler)),
  };
  return { cal, test, calib, res, alpha };
}

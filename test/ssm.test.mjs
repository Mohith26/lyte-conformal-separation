import test from 'node:test';
import assert from 'node:assert/strict';
import {
  SSM, HORIZON, DT, protectiveDistance, mulberry32, gauss, conformalQuantile, calibrate, conformalBound,
  binOf, track, measure, simulateSession, frameRecords, replay, runExperiment, NOISE, SENSOR,
} from '../core/ssm.mjs';

test('protective separation distance follows ISO/TS 15066 S_p = S_h + S_r + S_s + C + Z_d + Z_r', () => {
  const sp = protectiveDistance(1.6);
  const expect = 1.6 * (SSM.Tr + SSM.Ts) + SSM.vr * SSM.Tr + SSM.Ss + SSM.C + SSM.Zd + SSM.Zr;
  assert.ok(Math.abs(sp - expect) < 1e-12);
  assert.ok(Math.abs(protectiveDistance(-3) - protectiveDistance(0)) < 1e-12, 'receding speed never shrinks S_p below the constant terms');
});

test('conformal quantile uses the finite-sample ceil((n+1)(1-alpha)) rank', () => {
  const s = Array.from({ length: 99 }, (_, i) => i + 1); // 1..99
  assert.equal(conformalQuantile(s, 0.1), 90); // ceil(100*0.9) = 90th smallest
  assert.equal(conformalQuantile(s, 0.001), Infinity); // too few samples for alpha
});

test('split conformal covers exchangeable data at >= 1 - alpha (binomial check)', () => {
  const rng = mulberry32(7);
  const alpha = 0.05; let miss = 0, total = 0;
  for (let trial = 0; trial < 40; trial++) {
    const draw = () => Math.abs(gauss(rng)) * (1 + rng()); // heavy-ish tail
    const cal = Array.from({ length: 400 }, draw);
    const q = conformalQuantile(cal, alpha);
    for (let i = 0; i < 200; i++) { total++; if (draw() > q) miss++; }
  }
  const rate = miss / total, sd = Math.sqrt(alpha * (1 - alpha) / total);
  assert.ok(rate <= alpha + 3 * sd, `miss rate ${rate}`);
});

test('Doppler-aided tracker estimates approach speed better than position-only, especially when people start moving', () => {
  const sess = simulateSession(42, { minutes: 3, people: 3 });
  const rng = mulberry32(43);
  let ep = 0, ed = 0, n = 0, epA = 0, edA = 0, nA = 0;
  sess.persons.forEach((p) => {
    const m = measure(p, rng);
    const tp = track(m, { useDoppler: false, q: 1 }), td = track(m, { useDoppler: true, q: 2 });
    for (let k = 20; k < p.length; k++) {
      const s = p[k], r = Math.hypot(s.x - SENSOR.x, s.y - SENSOR.y);
      const truth = -((s.x - SENSOR.x) * s.vx + (s.y - SENSOR.y) * s.vy) / r;
      const est = (e) => -((e.x - SENSOR.x) * e.vx + (e.y - SENSOR.y) * e.vy) / Math.hypot(e.x - SENSOR.x, e.y - SENSOR.y);
      const a = Math.abs(est(tp[k]) - truth), b = Math.abs(est(td[k]) - truth);
      ep += a; ed += b; n++;
      const acc = Math.hypot(s.vx - p[k - 1].vx, s.vy - p[k - 1].vy) / DT;
      if (acc > 1.0) { epA += a; edA += b; nA++; }
    }
  });
  assert.ok(ed / n < 0.6 * (ep / n), `doppler ${ed / n} vs position ${ep / n}`);
  assert.ok(nA > 50 && edA / nA < ep / n, 'still better during accelerations');
});

test('frame records: closing speed is the largest approach within the horizon', () => {
  const steps = Math.round(HORIZON / DT);
  // person walks straight at the sensor at 1 m/s from 5 m, then stops
  const person = Array.from({ length: 60 }, (_, k) => { const y = Math.max(1, 5 - k * DT); return { x: 0, y, vx: 0, vy: y > 1 ? -1 : 0 }; });
  const est = person.map((s) => ({ ...s }));
  const recs = frameRecords(person, est);
  assert.ok(Math.abs(recs[0].closing - 1) < 1e-9, `closing ${recs[0].closing}`);
  assert.ok(Math.abs(recs[0].vhat - 1) < 1e-9);
  assert.equal(recs.length, 60 - steps);
  assert.ok(recs[recs.length - 1].closing === 0);
});

test('Mondrian calibration: bound covers held-out frames per speed state at about 1 - alpha', () => {
  const E = runExperiment({ calSeeds: [1, 2, 3, 4, 5, 6, 7, 8], testSeeds: [101, 102, 103, 104, 105, 106, 107, 108], alpha: 0.01 });
  const recs = E.test.flatMap((s) => s.recs.doppler.flat());
  const miss = [0, 0, 0], tot = [0, 0, 0];
  for (const r of recs) { const b = binOf(r.vhat); tot[b]++; if (r.closing > conformalBound(E.calib.doppler, r.vhat)) miss[b]++; }
  for (let b = 0; b < 3; b++) {
    const rate = miss[b] / tot[b], sd = Math.sqrt(0.01 * 0.99 / tot[b]);
    // frames are autocorrelated, so allow a wider band than iid
    assert.ok(rate <= 0.01 + 8 * sd, `bin ${b}: ${rate} on ${tot[b]}`);
  }
});

test('replay: stop/restart hysteresis and counters are consistent', () => {
  const session = { frames: 100 };
  const recs = [[...Array.from({ length: 10 }, (_, i) => ({ k: 10 + i, d: 0.5, vhat: 0, closing: 0 })), ...Array.from({ length: 5 }, (_, i) => ({ k: 50 + i, d: 0.5, vhat: 0, closing: 2 }))]];
  const r = replay(session, recs, () => 0, { restart: 1.0 });
  assert.equal(r.stops.length, 2);
  assert.deepEqual(r.stops[0], [10, 29]); // clear from frame 20, restarts after 10 clear frames
  assert.equal(r.frames, 15); assert.equal(r.under, 5);
  assert.deepEqual(r.stops[1], [50, 64]);
  assert.equal(r.downFrames, (29 - 10) + (64 - 50), "downtime equals the summed stop intervals");
});

test('experiment: conformal Doppler policy keeps the target rate and stops the robot less than the 1.6 m/s default', () => {
  const seeds = (b) => Array.from({ length: 12 }, (_, i) => b + i);
  const E = runExperiment({ calSeeds: seeds(1000), testSeeds: seeds(5000), alpha: 0.002 });
  const { static: st, naive, confPos, confDop } = E.res;
  assert.ok(naive.underRate > 0.1, 'measured speed with no margin is unsafe');
  assert.ok(confDop.underRate < 0.006, `conformal under-protection ${confDop.underRate}`);
  assert.ok(confDop.downtime < st.downtime, 'less downtime than the static default');
  assert.ok(confDop.downtime < confPos.downtime, 'Doppler tightens the bound vs position-only tracking');
  assert.ok(E.calib.doppler.q.every((q, b) => q <= E.calib.position.q[b] + 1e-9), 'Doppler margins are no wider in any speed state');
});

test('simulation and experiment are deterministic', () => {
  const a = simulateSession(9, { minutes: 1 }), b = simulateSession(9, { minutes: 1 });
  assert.equal(JSON.stringify(a), JSON.stringify(b));
  const n1 = measure(a.persons[0], mulberry32(1)), n2 = measure(b.persons[0], mulberry32(1));
  assert.equal(JSON.stringify(n1), JSON.stringify(n2));
  assert.ok(NOISE.pos > 0 && NOISE.doppler > 0);
});

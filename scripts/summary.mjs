// Reproduces every headline number on the page from the seeded recordings.
// Usage: node scripts/summary.mjs [alpha]
import { runExperiment, BINS } from '../core/ssm.mjs';

const alpha = Number(process.argv[2] || 0.001);
const calSeeds = Array.from({ length: 30 }, (_, i) => 1000 + i);
const testSeeds = Array.from({ length: 30 }, (_, i) => 5000 + i);
const E = runExperiment({ calSeeds, testSeeds, alpha });
const p = (v, d = 1) => `${(100 * v).toFixed(d)}%`;
console.log(`alpha ${alpha}: calibration ${calSeeds.length} recordings, test ${testSeeds.length} recordings (${E.res.static.hours.toFixed(1)} h, ${E.res.static.frames} person-frames)`);
for (const [k, name] of [['static', '1.6 m/s default'], ['naive', 'measured, no margin'], ['confPos', 'conformal, position-only'], ['confDop', 'conformal, Doppler']]) {
  const r = E.res[k];
  console.log(`  ${name.padEnd(26)} downtime ${p(r.downtime).padStart(6)}  stops/h ${r.stopsPerHour.toFixed(0).padStart(4)}  under-protected ${p(r.underRate, 3)}`);
}
BINS.forEach((b, i) => console.log(`  margin ${b.name.padEnd(20)} position +${E.calib.position.q[i].toFixed(2)} m/s  doppler +${E.calib.doppler.q[i].toFixed(2)} m/s  (n=${E.calib.doppler.n[i]})`));
console.log(`  robot time recovered per 8 h shift: ${((E.res.static.downtime - E.res.confDop.downtime) * 8).toFixed(1)} h`);

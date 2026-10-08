// Offline algorithm checks: no browser, robot or network access.
const assert = require('node:assert/strict');
const { SimulationPlanner: P } = require('./js/45_simulation.js');
const p = { v: 0.25, w: 60, settle: 0.05, stop: 0.20 };
const leg = P.leg(350, 1, p);
assert.equal(leg.kind, 'detour');
assert.ok(Math.abs(leg.a - 1.034) < 0.001);
assert.ok(Math.abs(leg.beta - 254.2) < 0.1);
assert.ok(Math.abs(leg.b - 0.180) < 0.001);
assert.ok(Math.abs(leg.time - 9.34) < 0.01);
const actual = { ...p, v: 0.03 };
assert.equal(P.leg(354, 0.3, actual).kind, 'detour');
assert.equal(P.leg(354, 0.4, actual).kind, 'direct');
assert.equal(P.leg(359, 0.3, actual).beta, 0);
let seed = 7;
const random = () => { seed = (1664525 * seed + 1013904223) >>> 0; return seed / 4294967296; };
for (let i = 0; i < 1000; ++i) {
  const phi = 4 + 352 * random(), d = 0.03 + 2 * random();
  const a = P.leg(phi, d, actual), b = P.leg(phi, d, actual, false);
  assert.ok(Number.isFinite(a.time) && a.time <= b.time + 1e-9);
  const bearing = phi * Math.PI / 180, goal = { x: d * Math.cos(bearing), y: d * Math.sin(bearing) };
  const run = P.route([goal], { x: 0, y: 0, h: 0 }, actual, true);
  assert.ok(Math.hypot(run.final.x - goal.x, run.final.y - goal.y) < 1e-8);
  assert.ok(Math.abs(run.time - run.phases.reduce((t, phase) => t + phase.duration, 0)) < 1e-8);
  for (const phase of run.phases) if (phase.mode !== 'drive') {
    assert.equal(phase.start.x, phase.end.x); assert.equal(phase.start.y, phase.end.y);
  }
}
console.log('PASS: homework example, measured-speed detour/direct cases, tolerance, and 1000 route geometries');

// Fixed-step, seeded model checks. No browser, robot or network access.
const assert = require('node:assert/strict');
const { SimulationPlanner: P, SimulationNoise: N } = require('./js/45_simulation.js');
const p = { v: 0.03, w: 60, settle: 0.05, stop: 0.20 };
const initial = { x: 0, y: 0, h: 25, bodyH: 12 };
const bearing = 31 * Math.PI / 180;
const points = [{ x: 0.3 * Math.cos(bearing), y: 0.3 * Math.sin(bearing) }, { x: 0.5, y: 0.3 }];
const ideal = P.route(points, initial, p, true);
const options = { seed: 42, level: 1 };
const a = N.realize(ideal, options), b = N.realize(ideal, options);
assert.deepEqual(a, b, 'same inputs reproduce every integration sample');
assert.notDeepEqual(a.final, N.realize(ideal, { ...options, seed: 43 }).final, 'another seed changes the model');
assert.equal(a.dt, 0.02);
assert.ok(Math.hypot(a.final.x - ideal.final.x, a.final.y - ideal.final.y) > 0.001);

// Animation queries are interpolation only; frame cadence cannot alter results.
const before = JSON.stringify(a.samples);
for (let t = 0; t < a.time; t += 1 / 17) N.poseAt(a, t);
for (let t = 0; t < a.time; t += 1 / 144) N.poseAt(a, t);
assert.equal(JSON.stringify(a.samples), before);
assert.deepEqual(N.poseAt(a, a.time + 1), a.final);

// At zero intensity geometry, timing, length and both sensors match ideal.
const zero = N.realize(ideal, { ...options, level: 0 });
assert.equal(zero.time, ideal.time);
assert.equal(zero.length, ideal.length);
for (let t = 0; t <= ideal.time; t += 0.027) {
  const expected = P.poseAt(ideal, t), actual = N.poseAt(zero, t);
  for (const key of ['x', 'y', 'h']) assert.ok(Math.abs(expected[key] - actual[key]) < 1e-9, key);
  assert.equal(actual.bodyError, 0); assert.equal(actual.wheelError, 0);
}

// Both error sources occur together, independently, and heading is their
// physical combination rather than an overloaded single yaw state.
assert.notEqual(a.samples[0].bodyError, 0); assert.notEqual(a.samples[0].wheelError, 0);
assert.notEqual(a.samples[0].bodyError, a.samples[0].wheelError);
for (let i = 0; i < a.samples.length; ++i) {
  const s = a.samples[i];
  assert.ok(Math.abs(s.h - s.bodyH - s.wheelAngle) < 1e-10);
  assert.ok(Math.abs(s.imuEstimate - s.bodyH - s.bodyError) < 1e-10);
  assert.ok(Math.abs(s.wheelEstimate - s.wheelAngle - s.wheelError) < 1e-10);
  if (i) assert.ok(s.t - a.samples[i - 1].t <= N.STEP + 1e-10);
}
// Different strategies see the same sensor biases at the same start time.
const direct = N.realize(P.route(points, initial, p, false), options);
for (const key of ['bodyError', 'wheelError']) assert.equal(direct.samples[0][key], a.samples[0][key]);

// Vibration is display-only and vanishes when stopped or disabled.
const finalBeforeShake = JSON.stringify(a.final);
assert.notDeepEqual(N.shake(a, 0.3, 'drive', 3), { x: 0, y: 0, angle: 0 });
assert.deepEqual(N.shake(a, 0.3, 'done', 3), { x: 0, y: 0, angle: 0 });
assert.deepEqual(N.shake(zero, 0.3, 'drive', 3), { x: 0, y: 0, angle: 0 });
assert.equal(JSON.stringify(a.final), finalBeforeShake);

// A range of seeds and maximum intensity remains finite and modest on this
// half-metre route, without changing the immutable ideal source plan.
const original = JSON.stringify(ideal);
for (let seed = 0; seed < 40; ++seed) {
  const run = N.realize(ideal, { seed, level: 2 });
  assert.ok(run.time > ideal.time * 0.7 && run.time < ideal.time * 1.3);
  assert.ok(Math.hypot(run.final.x - ideal.final.x, run.final.y - ideal.final.y) < 0.15);
  for (const s of run.samples) for (const key of ['x', 'y', 'h', 'bodyH', 'wheelAngle', 'imuEstimate', 'wheelEstimate']) assert.ok(Number.isFinite(s[key]));
}
assert.equal(JSON.stringify(ideal), original);
assert.throws(() => N.realize({ ...ideal, time: 1201 }, options), /20/);
console.log('PASS: seeded errors, fixed 20 ms steps, frame-independent replay, zero intensity, separate simultaneous body/wheel errors, shared environment, bounded output');

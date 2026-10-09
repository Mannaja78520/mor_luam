// Offline UI guards and firmware-result handling. No HTTP or motor commands.
const assert = require('node:assert/strict');
const { RouteTestHistory, RouteTestPanel } = require('./js/46_route_test.js');
const { RouteModel } = require('./js/30_route.js');
const done = (planner = 'direct', extra = {}) => ({ id: 1, active: false, phase: 'done', planner,
  startHeadingDeg: 0, actualStartHeadingDeg: 359, startX: 0, startY: 0, startThetaDeg: 0,
  speedMps: 0.03, tolM: 0.05, steerDps: 60, elapsedMs: 12345, valid: true, pidRevision: 1, ...extra });

const history = new RouteTestHistory();
history.context('route-a');
assert.equal(history.record(done('direct', { phase: 'stopped', valid: false }), 'route-a'), false);
assert.equal(history.record(done('direct', { active: true }), 'route-a'), false);
assert.equal(history.record(done('direct', { startX: null }), 'route-a'), false);
assert.equal(history.record(done(), 'route-b'), false);
assert.equal(history.record(done(), 'route-a'), true);
assert.equal(history.record(done('detour', { actualStartHeadingDeg: 1 }), 'route-a'), true);
assert.deepEqual(Object.keys(history.results).sort(), ['detour', 'direct']);
history.record(done('detour', { startX: 0.03 }), 'route-a');
assert.deepEqual(Object.keys(history.results), ['detour'], 'changed real starting pose drops old comparison');
history.context('route-b');
assert.deepEqual(history.results, {});
history.record(done('direct', { pidRevision: 3 }), 'route-b');
history.record(done('detour', { pidRevision: 4 }), 'route-b');
assert.deepEqual(Object.keys(history.results), ['detour'], 'advanced PID revision changes drop previous result');
history.record(done('direct', { pidRevision: 4 }), 'route-b');
assert.deepEqual(Object.keys(history.results).sort(), ['detour', 'direct'], 'same PID revision remains comparable');
assert.equal(history.record(done('direct', { pidRevision: undefined }), 'route-b'), false, 'unknown PID revision cannot be compared');

const nodes = new Map();
global.$ = selector => {
  if (!nodes.has(selector)) nodes.set(selector, { value: '', checked: false, disabled: false, textContent: '' });
  return nodes.get(selector);
};
$('#testHeading').value = '0';
const route = { loaded: true, points: [{ x: 0.3, y: 0.03 }], dirty: true, listeners: [],
  onChange(fn) { this.listeners.push(fn); },
  async save() { calls.push({ path: '/api/waypoints' }); this.dirty = false; this.listeners.forEach(fn => fn()); return { ok: true }; },
};
const calls = [];
const settings = { navSpeedMps: 0.03, navTolM: 0.05, steerDps: 60, otaPass: 'private-sentinel', apPass: 'private-sentinel' };
const pid = { spin: [40, 30, 0, 85, 0.3], steer: [27.8, 0.26, 7.4, 0, 3.5] };
let beforeGet = null;
const api = {
  async get(path) { if (beforeGet) beforeGet(); return { ok: true, data: path === '/api/settings' ? settings : pid }; },
  async post(path, body) { calls.push({ path, body }); return { ok: true, data: { nav: { test: { id: 2 } } } }; },
};
const toast = { result(r) { return r.ok; } };
const panel = new RouteTestPanel(api, route, toast, () => {}, () => settings);
const status = (test = done('direct', { id: 1, phase: 'idle', valid: false })) => ({
  robot: { mode: 'halt', coasting: false, wheelHeadingDeg: 0 },
  nav: { status: 'idle', test }, sys: { uptimeS: 10 },
});

(async () => {
  // Real RouteModel deferred HTTP regression: a later edit must not be marked
  // saved when the server only received the earlier submitted points.
  let resolveSave, submitted;
  const deferredApi = { post(path, body) { submitted = body.points; return new Promise(resolve => { resolveSave = resolve; }); } };
  const realRoute = new RouteModel(deferredApi);
  realRoute.loaded = true; realRoute.points = [{ x: 0.3, y: 0.03 }];
  const saving = realRoute.save();
  realRoute.move(0, 0.7, 0.04);
  assert.deepEqual(submitted, [{ x: 0.3, y: 0.03 }], 'pending request owns an independent snapshot');
  resolveSave({ ok: true });
  await saving;
  assert.deepEqual(realRoute.saved, [{ x: 0.3, y: 0.03 }]);
  assert.deepEqual(realRoute.points, [{ x: 0.7, y: 0.04 }]);
  assert.equal(realRoute.dirty, true, 'edit during pending save remains unsaved');

  // Button routes (slots 1/2): own load/save URL, edits kept per slot.
  const slotCalls = [];
  const slotApi = {
    async get(path) { slotCalls.push(['get', path]); return { ok: true, data: { points: path.endsWith('slot=1') ? [{ x: 1, y: 0 }] : [{ x: 0.3, y: 0.03 }] } }; },
    async post(path, body) { slotCalls.push(['post', path, body]); return { ok: true }; },
  };
  const slotRoute = new RouteModel(slotApi);
  await slotRoute.load();
  slotRoute.add(0.5, 0.5);
  await slotRoute.select(1);
  assert.deepEqual(slotCalls[1], ['get', '/api/waypoints?slot=1'], 'slot loads with ?slot=');
  assert.deepEqual(slotRoute.points, [{ x: 1, y: 0 }]);
  assert.equal(slotRoute.isDirty(0), true, 'web route edit kept while a button route is shown');
  slotRoute.add(1, 1);
  await slotRoute.save();
  assert.deepEqual(slotCalls[2], ['post', '/api/waypoints', { slot: 1, points: [{ x: 1, y: 0 }, { x: 1, y: 1 }] }]);
  assert.equal(slotRoute.dirty, false);
  await slotRoute.select(0);
  assert.equal(slotCalls.length, 3, 'a loaded slot is not fetched again');
  assert.deepEqual(slotRoute.points, [{ x: 0.3, y: 0.03 }, { x: 0.5, y: 0.5 }]);
  await slotRoute.save();
  assert.deepEqual(slotCalls[3], ['post', '/api/waypoints', { points: [{ x: 0.3, y: 0.03 }, { x: 0.5, y: 0.5 }] }], 'web route body unchanged');
  panel.render(status(), 'live');
  assert.equal(panel.canStart(), false, 'readiness required');
  panel.ready.checked = true;
  panel.render(status(), 'stale');
  assert.equal(panel.canStart(), false, 'stale status cannot start');
  const moving = status(); moving.robot.mode = 'drive';
  panel.render(moving, 'live');
  assert.equal(panel.canStart(), false, 'manual motion cannot start');
  panel.render(status(), 'live');
  panel.ready.checked = true;
  await panel.start('direct');
  assert.deepEqual(calls.map(x => x.path), ['/api/waypoints', '/api/nav/test']);
  assert.deepEqual(calls[1].body, { planner: 'direct', startHeadingDeg: 0, ready: true });
  assert.equal(panel.ready.checked, false, 'each trial requires a new placement check');
  assert.equal(panel.paramsKey.includes('private-sentinel'), false, 'passwords never enter comparison context');
  panel.render(status(done('direct', { id: 1 })), 'live');
  assert.deepEqual(panel.history.results, {}, 'prior completed result is ignored');
  panel.render(status(done('direct', { id: 2, phase: 'aligning', active: true, elapsedMs: 0 })), 'live');
  assert.deepEqual(panel.history.results, {}, 'alignment is not timed as a completed run');
  panel.render(status(done('direct', { id: 2 })), 'live');
  assert.equal(panel.history.results.direct.elapsedMs, 12345);
  assert.equal($('#testDirectTime').textContent, '12.35 s', 'time comes from firmware elapsedMs');
  panel.heading.value = '90'; panel.heading.oninput();
  assert.deepEqual(panel.history.results, {}, 'heading edit clears comparison');
  panel.ready.checked = true;
  calls.length = 0;
  let edited = false;
  beforeGet = () => {
    if (edited) return;
    edited = true; route.points = [{ x: 0.5, y: 0 }]; route.listeners.forEach(fn => fn());
  };
  await panel.start('detour');
  assert.deepEqual(calls, [], 'route edited during preflight cannot start under old confirmation');
  calls.length = 0; panel.ready.checked = true;
  let cancelled = false;
  beforeGet = () => {
    if (cancelled) return;
    cancelled = true; panel.stop.onclick();
  };
  await panel.start('direct');
  assert.deepEqual(calls.map(x => x.path), ['/api/nav/stop'], 'Stop during preflight cancels start');
  beforeGet = null; calls.length = 0; panel.ready.checked = true; route.dirty = false;
  const normalPost = api.post;
  api.post = async (path, body) => {
    if (path === '/api/nav/test') await panel.stop.onclick();
    return normalPost(path, body);
  };
  await panel.start('direct');
  assert.deepEqual(calls.map(x => x.path), ['/api/waypoints', '/api/nav/stop', '/api/nav/test', '/api/nav/stop'],
    'Stop racing a late accepted start is repeated after the start reply');
  assert.equal(calls[0].path, '/api/waypoints', 'even a locally clean route is saved again before test');
  console.log('Route test UI: offline guards, preparation, results and comparison checks PASS');
})().catch(e => { console.error(e); process.exitCode = 1; });

// Malformed HTTP bodies must never be presented to views as fresh telemetry.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const Api = vm.runInNewContext(fs.readFileSync(require.resolve('./js/10_api.js'), 'utf8') + ';Api',
  { AbortController, setTimeout, clearTimeout, fetch: (...args) => global.fetch(...args) });
(async () => {
  const original = global.fetch;
  try {
    const api = new Api();
    global.fetch = async () => ({ ok: true, json: async () => { throw new SyntaxError('empty response'); } });
    assert.equal((await api.get('/api/status')).ok, false);
    global.fetch = async () => ({ ok: true, json: async () => ({}) });
    assert.equal((await api.get('/api/status')).ok, false);
    global.fetch = async () => ({ ok: true, json: async () => ({ robot: { mode: 'halt' }, nav: {}, net: {}, ros: {}, sys: {} }) });
    assert.equal((await api.get('/api/status')).data.robot.mode, 'halt');
    global.fetch = async () => ({ ok: false, status: 503, json: async () => ({ error: 'unavailable' }) });
    assert.equal((await api.get('/api/status')).error, 'unavailable');
    const scheduled = [];
    const Poller = vm.runInNewContext(fs.readFileSync(require.resolve('./js/10_api.js'), 'utf8') + ';Poller',
      { Date, document: { hidden: false }, setTimeout: callback => { scheduled.push(callback); return scheduled.length; } });
    let frames = 0;
    const poller = new Poller({ get: async () => ({ ok: true, data: {} }) }, '/api/status', 250,
      { onData: () => { if (++frames === 1) throw new Error('bad frame'); }, onConn: () => {} });
    poller.startMs = Date.now();
    await poller.tick();
    assert.equal(poller.busy, false);
    assert.equal(poller.lastOkMs, 0, 'failed processing must not mark data fresh');
    assert.equal(scheduled.length, 1, 'a failed frame must schedule another poll');
    await scheduled.shift()();
    assert.equal(frames, 2);
    assert.ok(poller.lastOkMs > 0, 'the next good frame recovers freshness');
    console.log('API/Poller: invalid status refused, HTTP errors preserved, polling recovers after a bad frame PASS');
  } finally { global.fetch = original; }
})().catch(error => { console.error(error); process.exitCode = 1; });

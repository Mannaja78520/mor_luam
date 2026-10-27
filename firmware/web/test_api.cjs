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
    global.fetch = async () => ({ ok: true, json: async () => ({ robot: { mode: 'halt' } }) });
    assert.equal((await api.get('/api/status')).data.robot.mode, 'halt');
    global.fetch = async () => ({ ok: false, status: 503, json: async () => ({ error: 'unavailable' }) });
    assert.equal((await api.get('/api/status')).error, 'unavailable');
    console.log('API: empty JSON refused, valid status and HTTP errors preserved PASS');
  } finally { global.fetch = original; }
})().catch(error => { console.error(error); process.exitCode = 1; });

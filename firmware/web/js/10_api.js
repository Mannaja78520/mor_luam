// Talking to the robot. Every call resolves to {ok, data, error} - it never
// throws - so a view only has to handle two cases. The endpoint list is at
// the top of src/web/WebApp.h.

class Api {
  constructor(base = '') { this.base = base; }

  async request(method, path, body = null, timeoutMs = 4000) {
    const ctl = new AbortController();
    const timer = setTimeout(() => ctl.abort(), timeoutMs);
    try {
      const res = await fetch(this.base + path, {
        method,
        cache: 'no-store',
        headers: body ? { 'Content-Type': 'application/json' } : {},
        body: body ? JSON.stringify(body) : undefined,
        signal: ctl.signal,
      });
      let data = {};
      try { data = await res.json(); } catch { /* empty body */ }
      if (!res.ok || data.ok === false) return { ok: false, error: data.error || `หุ่นตอบ HTTP ${res.status}`, data };
      return { ok: true, data };
    } catch (e) {
      const error = e.name === 'AbortError' ? 'หุ่นไม่ตอบ (เกิน ' + timeoutMs / 1000 + ' วินาที)' : 'ติดต่อหุ่นไม่ได้';
      return { ok: false, error, offline: true };
    } finally {
      clearTimeout(timer);
    }
  }

  get(path, timeoutMs) { return this.request('GET', path, null, timeoutMs); }
  post(path, body = null, timeoutMs) { return this.request('POST', path, body, timeoutMs); }

  // Firmware upload with progress (fetch cannot report upload progress).
  upload(path, file, headers, onProgress) {
    return new Promise((resolve) => {
      const xhr = new XMLHttpRequest();
      xhr.open('POST', this.base + path);
      for (const [k, v] of Object.entries(headers)) xhr.setRequestHeader(k, v);
      xhr.upload.onprogress = (e) => e.lengthComputable && onProgress(e.loaded / e.total);
      xhr.onload = () => {
        let data = {};
        try { data = JSON.parse(xhr.responseText); } catch { /* not JSON */ }
        resolve(xhr.status === 200 && data.ok ? { ok: true, data } : { ok: false, error: data.error || `HTTP ${xhr.status}` });
      };
      xhr.onerror = () => resolve({ ok: false, error: 'การเชื่อมต่อหลุดระหว่างอัปโหลด' });
      const form = new FormData();
      form.append('firmware', file, file.name);
      xhr.send(form);
    });
  }
}

// Polls one endpoint and says how fresh the data is:
//   loading -> live -> stale (no answer for staleMs) -> offline (offlineMs)
// The next request starts only after the last one finished, so a slow robot
// is never buried under queued requests.
class Poller {
  constructor(api, path, periodMs, { onData, onConn, staleMs = 1500, offlineMs = 5000 }) {
    Object.assign(this, { api, path, periodMs, onData, onConn, staleMs, offlineMs });
    this.lastOkMs = 0;
    this.conn = 'loading';
    this.timer = 0;
  }

  start() { this.startMs = Date.now(); this.tick(); setInterval(() => this.updateConn(), 500); }

  // ask again right away (after a command, so the page reacts at once)
  now() { clearTimeout(this.timer); this.tick(); }

  async tick() {
    if (this.busy) return;
    this.busy = true;
    const r = await this.api.get(this.path, 3000);
    this.busy = false;
    if (r.ok) { this.lastOkMs = Date.now(); this.onData(r.data); }
    this.updateConn();
    const slow = document.hidden ? 1000 : this.periodMs;   // background tab: go easy on the robot (still under staleMs)
    this.timer = setTimeout(() => this.tick(), slow);
  }

  updateConn() {
    const age = Date.now() - (this.lastOkMs || this.startMs);
    const c = age > this.offlineMs ? 'offline' : !this.lastOkMs ? 'loading' : age > this.staleMs ? 'stale' : 'live';
    if (c !== this.conn) { this.conn = c; this.onConn(c); }
  }
}

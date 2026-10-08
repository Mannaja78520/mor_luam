// Real trials are separate from RouteSimulation. Only firmware elapsedMs is
// a measurement; preparation and browser/network delays never enter the result.
class RouteTestHistory {
  constructor() { this.key = ''; this.results = {}; }
  reset(key = '') { this.key = key; this.results = {}; }
  context(key) { if (key !== this.key) this.reset(key); }
  static angleDiff(a, b) { return Math.abs(((a - b + 540) % 360) - 180); }
  record(test, key) {
    if (key !== this.key || test.phase !== 'done' || !test.valid || test.active ||
      !['direct', 'detour'].includes(test.planner) || !Number.isFinite(test.elapsedMs) || test.elapsedMs < 0 ||
      !Number.isSafeInteger(test.pidRevision) || test.pidRevision < 0 ||
      !['actualStartHeadingDeg', 'startX', 'startY', 'startThetaDeg'].every(k => Number.isFinite(test[k]))) return false;
    const other = this.results[test.planner === 'direct' ? 'detour' : 'direct'];
    // Both the manual placement check and the odometry must agree. Odometry
    // alone cannot prove that somebody put the real robot back on the floor.
    if (other && (test.pidRevision !== other.pidRevision ||
      Math.hypot(test.startX - other.startX, test.startY - other.startY) > 0.02 ||
      RouteTestHistory.angleDiff(test.startThetaDeg, other.startThetaDeg) > 3.5 ||
      RouteTestHistory.angleDiff(test.actualStartHeadingDeg, other.actualStartHeadingDeg) > 3.5)) this.results = {};
    this.results[test.planner] = { ...test };
    return true;
  }
}

class RouteTestPanel {
  constructor(api, route, toast, onCommand, savedSettings) {
    Object.assign(this, { api, route, toast, onCommand, savedSettings });
    this.heading = $('#testHeading'); this.ready = $('#routeTestReady');
    this.direct = $('#testDirect'); this.detour = $('#testDetour'); this.stop = $('#testStop');
    this.message = $('#routeTestMsg'); this.history = new RouteTestHistory();
    this.conn = 'loading'; this.busy = false; this.pending = null; this.notice = '';
    this.cancelGeneration = 0;
    this.pointsKey = JSON.stringify(route.points); this.headingKey = this.heading.value;
    this.paramsKey = ''; this.lastSavedParams = '';
    this.ready.onchange = () => this.draw();
    this.heading.oninput = () => {
      if (this.heading.value !== this.headingKey) { this.headingKey = this.heading.value; this.invalidate('มุมเริ่มเปลี่ยนแล้ว ผลเดิมถูกล้าง'); }
      this.draw();
    };
    $('#testUseHeading').onclick = () => {
      if (!this.last || this.conn !== 'live') return;
      this.heading.value = (((this.last.robot.wheelHeadingDeg % 360) + 360) % 360).toFixed(1);
      this.heading.oninput();
    };
    route.onChange(() => {
      const key = JSON.stringify(route.points);
      if (key !== this.pointsKey) { this.pointsKey = key; this.invalidate('จุดเปลี่ยนแล้ว ผลเดิมถูกล้าง'); }
      this.draw();
    });
    this.direct.onclick = () => this.start('direct');
    this.detour.onclick = () => this.start('detour');
    this.stop.onclick = async () => {
      ++this.cancelGeneration;
      this.ready.checked = false;
      if (this.pending) this.pending.invalid = true;
      this.notice = 'กำลังหยุดทดสอบ…'; this.draw();
      this.toast.result(await api.post('/api/nav/stop'), 'หยุดทดสอบแล้ว'); onCommand();
    };
    this.draw();
  }
  static params(settings, pid) {
    return { speedMps: settings.navSpeedMps, tolM: settings.navTolM, steerDps: settings.steerDps, pid };
  }
  key() { return JSON.stringify([this.route.points, Number(this.heading.value), this.paramsKey]); }
  invalidate(notice) {
    this.history.reset(); this.ready.checked = false; this.notice = notice;
    // An edited trial is no longer comparable even if it later finishes.
    if (this.pending) this.pending.invalid = true;
  }
  canStart() {
    const rb = this.last && this.last.robot, nav = this.last && this.last.nav;
    const h = Number(this.heading.value);
    return this.conn === 'live' && !this.busy && rb && rb.mode === 'halt' && !rb.coasting &&
      nav.status !== 'running' && this.route.loaded && this.route.points.length > 0 &&
      this.ready.checked && this.heading.value.trim() !== '' && Number.isFinite(h) && h >= 0 && h < 360;
  }
  async start(planner) {
    if (!this.canStart()) return;
    const pointsKey = this.pointsKey, headingKey = this.headingKey;
    const generation = this.cancelGeneration;
    this.busy = true; this.ready.checked = false; this.notice = 'กำลังตรวจจุดและค่าทดสอบ…'; this.draw();
    try {
      // Settings contain passwords: select only public motion parameters and
      // never log, persist or display the complete response.
      const [settings, pid] = await Promise.all([this.api.get('/api/settings'), this.api.get('/api/pid')]);
      if (!settings.ok || !pid.ok) {
        this.toast.result(!settings.ok ? settings : pid); this.notice = 'อ่านค่าทดสอบไม่ได้ กรุณายืนยันความพร้อมใหม่'; return;
      }
      const params = RouteTestPanel.params(settings.data, pid.data);
      const paramsKey = JSON.stringify(params);
      if (this.paramsKey && paramsKey !== this.paramsKey) this.invalidate('ค่าทดสอบเปลี่ยนแล้ว ผลเดิมถูกล้าง');
      this.paramsKey = paramsKey; this.history.context(this.key());
      // Check again after the requests; status or edited points may have changed.
      if (generation !== this.cancelGeneration || pointsKey !== this.pointsKey || headingKey !== this.headingKey || !this.route.points.length ||
        this.conn !== 'live' || !this.last || this.last.robot.mode !== 'halt' || this.last.robot.coasting || this.last.nav.status === 'running') {
        this.notice = 'หุ่นยังไม่พร้อม กรุณาหยุดและยืนยันใหม่'; return;
      }
      // Another page may have changed the robot's saved route. Always submit
      // this page's selected points, even when its local copy looks clean.
      const saved = await this.route.save();
      if (!this.toast.result(saved, 'บันทึกจุดแล้ว')) { this.notice = 'บันทึกจุดไม่ได้ ยังไม่เริ่มทดสอบ'; return; }
      if (generation !== this.cancelGeneration || pointsKey !== this.pointsKey || headingKey !== this.headingKey) { this.notice = 'ยกเลิกหรือจุดและมุมเปลี่ยน กรุณายืนยันใหม่'; return; }
      const old = this.last.nav.test;
      this.pending = { planner, key: this.key(), previousId: old && old.id, params, invalid: false, accepted: false };
      const r = await this.api.post('/api/nav/test', { planner, startHeadingDeg: Number(this.heading.value), ready: true });
      if (generation !== this.cancelGeneration) {
        // If Stop raced an in-flight start request, stop again after its reply
        // so a late accepted start cannot resume the test.
        if (this.pending) this.pending.invalid = true;
        await this.api.post('/api/nav/stop');
        this.notice = 'ยกเลิกการทดสอบแล้ว'; this.onCommand(); return;
      }
      if (!r.ok && !r.offline) this.pending = null;
      else if (r.ok && this.pending) {
        this.pending.accepted = true;
        if (r.data && r.data.nav && r.data.nav.test) this.pending.id = r.data.nav.test.id;
      }
      this.toast.result(r, `เริ่มทดสอบ ${planner === 'direct' ? 'แบบที่ 1' : 'แบบที่ 2'}: เตรียมมุมล้อก่อนจับเวลา`);
      this.notice = r.ok ? '' : 'ยังยืนยันการเริ่มไม่ได้ ดูสถานะหุ่น หรือกดหยุดทดสอบ';
      if (this.last) this.render(this.last, this.conn);
      this.onCommand();
    } finally { this.busy = false; this.draw(); }
  }
  render(s, conn) {
    if (this.last && s.sys.uptimeS < this.last.sys.uptimeS) this.invalidate('หุ่นเริ่มใหม่แล้ว ผลเดิมถูกล้าง');
    this.last = s; this.conn = conn;
    const saved = this.savedSettings && this.savedSettings();
    if (saved) {
      const key = JSON.stringify([saved.navSpeedMps, saved.navTolM, saved.steerDps]);
      if (this.lastSavedParams && key !== this.lastSavedParams) this.invalidate('ค่าทดสอบเปลี่ยนแล้ว ผลเดิมถูกล้าง');
      this.lastSavedParams = key;
    }
    const t = s.nav.test, p = this.pending;
    if (conn === 'live' && p && t && t.id !== p.previousId && t.planner === p.planner &&
      (p.id === undefined || t.id === p.id)) {
      if (!t.active && ['done', 'stopped', 'failed'].includes(t.phase)) {
        const sameParams = ['speedMps', 'tolM', 'steerDps'].every(k => Math.abs(t[k] - p.params[k]) < 0.00001);
        // A lost POST reply may belong to an unconfirmed request. Its live
        // motion is still visible/stoppable, but never credited as our trial.
        if (!p.accepted && this.busy) return this.draw();
        const recorded = p.accepted && !p.invalid && sameParams && this.history.record(t, p.key);
        this.notice = recorded ? 'ถึงครบทุกจุดแล้ว เก็บเวลาจากหุ่นแล้ว' : 'ทดสอบไม่ครบหรือค่าทดสอบเปลี่ยน จึงไม่เก็บเป็นผลเปรียบเทียบ';
        this.pending = null;
      }
    }
    this.draw();
  }
  draw() {
    const can = this.canStart(), running = !!(this.last && this.last.nav.status === 'running');
    this.direct.disabled = this.detour.disabled = !can;
    this.stop.disabled = this.conn !== 'live' || !(running || this.pending || this.busy);
    this.heading.disabled = $('#testUseHeading').disabled = running || this.busy;
    this.ready.disabled = running || this.busy;
    const test = this.last && this.last.nav.test;
    let message = this.notice;
    if (this.conn !== 'live') message = 'รอข้อมูลสดจากหุ่นก่อนทดสอบ';
    else if (test && test.active) message = test.phase === 'aligning'
      ? 'กำลังหันล้อไปมุมเริ่มร่วมกัน ยังไม่จับเวลา'
      : `กำลังทดสอบ ${test.planner === 'direct' ? 'แบบที่ 1' : 'แบบที่ 2'} · ${(test.elapsedMs / 1000).toFixed(2)} s จากหุ่น`;
    else if (running) message = 'เส้นทางอื่นกำลังทำงาน หยุดก่อนทดสอบ';
    else if (!message) message = this.ready.checked ? 'พร้อมทดสอบ ตรวจว่าคืนจุดเริ่มเดิมก่อนเลือกแบบ' : 'วางจุดและยืนยันว่าพร้อมก่อนทดสอบแต่ละครั้ง';
    this.message.textContent = message;
    for (const [planner, prefix] of [['direct', 'testDirect'], ['detour', 'testDetour']]) {
      const r = this.history.results[planner];
      $('#' + prefix + 'Time').textContent = r ? `${(r.elapsedMs / 1000).toFixed(2)} s` : '–';
      $('#' + prefix + 'Detail').textContent = r
        ? `มุมล้อจริง ${r.actualStartHeadingDeg.toFixed(1)}° · เริ่ม (${r.startX.toFixed(2)}, ${r.startY.toFixed(2)}) m · ช่องว่างตรวจสถานะสูงสุด ${r.maxUpdateGapMs ?? r.observationMaxGapMs ?? '–'} ms`
        : 'ยังไม่มีผล';
    }
    const a = this.history.results.direct, b = this.history.results.detour;
    $('#routeTestCompare').textContent = a && b
      ? `เวลาต่างกัน ${((a.elapsedMs - b.elapsedMs) / 1000).toFixed(2)} s (Direct − Detour) · ใช้จุด มุมเริ่ม และค่าทดสอบชุดเดียวกัน`
      : 'เก็บผลเฉพาะวิ่งถึงครบทุกจุด หากจุด มุม ค่า หรือท่าเริ่มเปลี่ยน จะล้างผลเดิมก่อนเทียบ';
  }
}

if (typeof module !== 'undefined') module.exports = { RouteTestHistory, RouteTestPanel };

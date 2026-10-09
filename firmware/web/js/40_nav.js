// Route tab, right side: the waypoint list and the start/stop panel.

const NAV_STATUS = {
  idle: ['พร้อม', ''],
  running: ['กำลังวิ่ง', 'run'],
  done: ['ถึงครบแล้ว', 'done'],
  stopped: ['หยุดแล้ว', 'stop'],
  failed: ['ไปไม่ถึง', 'fail'],
};
const PLANNER_NAMES = { detour: 'Detour Steer', direct: 'เลี้ยวตรงไปหาจุด' };

// Which route the plane and the list edit: the web route, or the demo button's
// route 1 / 2 (saved on the robot). " •" marks a slot with unsaved edits.
const SLOT_NAMES = { 0: 'หน้าเว็บ', 1: 'ปุ่ม 1 ครั้ง', 2: 'ปุ่ม 2 ครั้ง' };
class RouteSlotBar {
  constructor(route, toast, onSwitch) {
    Object.assign(this, { route, toast });
    this.buttons = [...document.querySelectorAll('[data-slot]')];
    this.hint = $('#slotHint');
    this.buttons.forEach((b) => {
      b.onclick = async () => {
        const r = await route.select(Number(b.dataset.slot));
        if (r.ok) onSwitch(); else this.toast.result(r);
      };
    });
    route.onChange(() => this.render());
    this.render();
  }

  render() {
    const r = this.route;
    for (const b of this.buttons) {
      const slot = Number(b.dataset.slot), on = slot === r.slot;
      b.classList.toggle('active', on);
      b.setAttribute('aria-pressed', String(on));
      b.textContent = SLOT_NAMES[slot] + (r.isDirty(slot) ? ' •' : '');
    }
    this.hint.classList.toggle('hidden', !r.slot);
    this.hint.textContent = r.slot
      ? `เส้นทางของปุ่มบนหุ่น กด ${r.slot} ครั้ง: (0,0) = ที่หุ่นยืนตอนกดปุ่ม · +x = ทิศหน้าหุ่น · จบแล้วหุ่นกลับจุดเริ่มเอง · กด "บันทึกลงหุ่น" หุ่นจึงใช้จุดชุดใหม่`
      : '';
  }
}

// The list of points as a table, editable by keyboard as well as on the plane.
class WaypointTable {
  constructor(route, toast) {
    Object.assign(this, { route, toast });
    this.body = $('#wpBody');
    this.count = $('#wpCount');
    this.dirtyLine = $('#wpDirty');
    this.saveBtn = $('#wpSave');
    this.revertBtn = $('#wpRevert');
    this.clearBtn = $('#wpClear');
    this.activeIdx = -1;
    this.locked = '';
    this.table = null;
    route.onChange(() => this.render());
    this.saveBtn.onclick = async () => this.toast.result(await route.save(), 'บันทึกจุดลงหุ่นแล้ว');
    this.revertBtn.onclick = () => route.revert();
    this.clearBtn.onclick = () => {
      if (route.points.length && confirm(`ลบทั้ง ${route.points.length} จุดในหน้านี้? (จุดในหุ่นยังอยู่จนกว่าจะกดบันทึก)`)) route.clear();
    };
    this.render();
  }

  setState(activeIdx, locked) {
    if (activeIdx === this.activeIdx && locked === this.locked) return;
    this.activeIdx = activeIdx;
    this.locked = locked;
    this.render();
  }

  render() {
    const r = this.route, n = r.points.length;
    this.count.textContent = `${n} / ${r.max}`;
    const dirty = r.loaded && r.dirty;
    this.dirtyLine.textContent = dirty ? 'ยังไม่ได้บันทึก: หุ่นยังใช้จุดชุดเดิม' : ' ';
    this.saveBtn.classList.toggle('attention', dirty);
    this.saveBtn.disabled = !dirty || !!this.locked;
    this.revertBtn.disabled = !dirty;
    this.clearBtn.disabled = !n || !!this.locked;

    if (!r.loaded) {
      this.table = null;
      this.body.replaceChildren(r.loadError
        ? el('div', { class: 'errbox' }, `โหลดจุดจากหุ่นไม่ได้: ${r.loadError}`,
          el('button', { class: 'btn', type: 'button', onclick: () => r.load() }, 'ลองใหม่'))
        : el('p', { class: 'loading' }, 'กำลังโหลดจุดจากหุ่น…'));
      return;
    }
    if (!n) {
      this.table = null;
      this.body.replaceChildren(el('div', { class: 'empty' },
        el('b', { text: 'ยังไม่มีจุด' }), 'แตะบนระนาบด้านซ้าย (หรือด้านบนในมือถือ) เพื่อวางจุดแรก'));
      return;
    }
    if (!this.table || this.table.tBodies[0].rows.length !== n) this.build(n);
    const rows = this.table.tBodies[0].rows;
    r.points.forEach((p, i) => {
      const row = rows[i];
      row.classList.toggle('cur', i === this.activeIdx);
      const [ix, iy, iw] = row.querySelectorAll('input');
      for (const [inp, v] of [[ix, p.x.toFixed(2)], [iy, p.y.toFixed(2)], [iw, String(p.waitS || 0)]]) {
        if (document.activeElement !== inp) inp.value = v;   // never overwrite while typing
        inp.disabled = !!this.locked;
      }
      row.querySelectorAll('button').forEach((b) => { b.disabled = !!this.locked; });
      row.querySelector('[data-up]').disabled = !!this.locked || i === 0;
    });
  }

  build(n) {
    const tb = el('tbody');
    for (let i = 0; i < n; ++i) {
      const num = (axis) => el('input', {
        type: 'number', step: '0.1', inputmode: 'decimal', 'aria-label': `จุด ${i + 1} ${axis} (m)`,
        onchange: (e) => this.edit(i, axis, e.target),
      });
      const wait = el('input', {
        type: 'number', step: '0.5', min: '0', max: String(ROUTE_MAX_WAIT_S), inputmode: 'decimal',
        'aria-label': `จุด ${i + 1} หยุดรอ (วินาที)`, onchange: (e) => this.editWait(i, e.target),
      });
      tb.append(el('tr', {},
        el('td', { text: String(i + 1) }),
        el('td', {}, num('x')),
        el('td', {}, num('y')),
        el('td', { class: 'wait' }, wait),
        el('td', { class: 'act' },
          el('button', { class: 'iconbtn', type: 'button', 'data-up': '1', title: 'เลื่อนขึ้น', 'aria-label': `เลื่อนจุด ${i + 1} ขึ้น`, onclick: () => this.route.moveUp(i) }, '↑'),
          el('button', { class: 'iconbtn danger', type: 'button', title: 'ลบ', 'aria-label': `ลบจุด ${i + 1}`, onclick: () => this.route.remove(i) }, '✕'))));
    }
    this.table = el('table', { class: 'wptable' },
      el('thead', {}, el('tr', {}, el('th', { text: '#' }), el('th', { text: 'x (m)' }), el('th', { text: 'y (m)' }),
        el('th', { text: 'รอ (s)', title: 'ถึงจุดแล้วหยุดรอกี่วินาที (0 = ไปต่อเลย)' }), el('th'))), tb);
    this.body.replaceChildren(el('div', { class: 'wpscroll' }, this.table));
  }

  edit(i, axis, inp) {
    const v = parseFloat(inp.value);
    const okv = Number.isFinite(v) && Math.abs(v) <= ROUTE_LIMIT_M;
    inp.classList.toggle('bad', !okv);
    if (!okv) { this.toast.show(`ค่าต้องเป็นตัวเลขระหว่าง -${ROUTE_LIMIT_M} ถึง ${ROUTE_LIMIT_M} m`, true); return; }
    const p = this.route.points[i];
    this.route.move(i, axis === 'x' ? v : p.x, axis === 'y' ? v : p.y);
  }

  editWait(i, inp) {
    const v = inp.value.trim() === '' ? 0 : parseFloat(inp.value);
    const okv = Number.isFinite(v) && v >= 0 && v <= ROUTE_MAX_WAIT_S;
    inp.classList.toggle('bad', !okv);
    if (!okv) { this.toast.show(`เวลารอต้องอยู่ระหว่าง 0 ถึง ${ROUTE_MAX_WAIT_S} วินาที`, true); return; }
    this.route.setWait(i, v);
  }
}

// Start / stop, what the robot is doing on the route, and the plan it chose.
class NavPanel {
  constructor(api, route, toast, { onPoseReset, onCommand }) {
    Object.assign(this, { api, route, toast, onCommand });
    this.badge = $('#navBadge');
    this.msg = $('#navMsg');
    this.prog = $('#navProg');
    this.plan = $('#navPlan');
    this.algo = $('#navAlgo');
    this.hint = $('#navHint');
    this.startBtn = $('#navStart');
    this.stopBtn = $('#navStop');
    this.resetBtn = $('#poseReset');
    this.running = false;
    this.heartbeat = 0;
    this.ready = $('#realRunReady');
    this.ready.onchange = () => { if (this.lastState) this.render(...this.lastState); };
    route.onChange(() => { if (this.lastState) this.render(...this.lastState); });   // slot switch

    this.startBtn.onclick = () => this.start();
    this.stopBtn.onclick = async () => { this.toast.result(await api.post('/api/nav/stop'), 'หยุดเส้นทางแล้ว'); onCommand(); };
    this.resetBtn.onclick = async () => {
      if (!confirm('ให้ตำแหน่งตอนนี้เป็น (0,0) และทิศที่หันอยู่เป็น +x?\nจุดที่วางไว้จะไม่ขยับตาม')) return;
      if (this.toast.result(await api.post('/api/pose/reset'), 'ตั้งจุดเริ่มต้นใหม่แล้ว')) onPoseReset();
      onCommand();
    };
  }

  async start() {
    if (this.route.slot) return;                // the start button drives the web route only
    if (!this.ready.checked) { this.toast.show('ตรวจพื้นและยืนยันว่าอยู่ข้างหุ่นก่อนวิ่งจริง', true); return; }
    this.ready.checked = false;
    if (this.route.dirty) {
      const s = await this.route.save();
      if (!this.toast.result(s, 'บันทึกจุดแล้ว')) return;
    }
    this.toast.result(await this.api.post('/api/nav/start'), 'เริ่มวิ่งตามจุด');
    this.onCommand();
  }

  // While a route runs, every open page tells the robot "someone is watching".
  // No word for 3 s (page closed, phone asleep, Wi-Fi lost) and the robot stops.
  keepAlive(running) {
    if (running === this.running) return;
    this.running = running;
    clearInterval(this.heartbeat);
    if (running) this.heartbeat = setInterval(() => this.api.post('/api/nav/heartbeat', null, 900), 1000);
  }

  render(s, conn) {
    this.lastState = [s, conn];
    const nav = s.nav, robot = s.robot;
    const running = nav.status === 'running';
    this.keepAlive(running && conn !== 'offline');
    const [word, cls] = NAV_STATUS[nav.status] || [nav.status, ''];
    this.badge.textContent = word;
    this.badge.className = 'badge ' + cls;
    const m = nav.message && nav.message !== word ? nav.message : '';   // the badge already says it
    this.msg.textContent = running && nav.waitLeftMs > 0
      ? `${m || 'หยุดรอ'} · อีก ${(nav.waitLeftMs / 1000).toFixed(1)} วินาที`   // the robot's message says why
      : m || (running ? `ไปจุดที่ ${nav.index + 1} จาก ${nav.count}` : ' ');
    this.msg.title = this.msg.textContent;
    const done = nav.status === 'done' ? nav.count : running ? nav.index : 0;
    this.prog.style.width = nav.count ? `${(100 * done) / nav.count}%` : '0';
    this.algo.textContent = PLANNER_NAMES[nav.planner] || nav.planner || '–';

    const p = nav.plan || {};
    if (running && p.distM > 0) {
      const head = `${p.kind === 'detour' ? 'อ้อม' : 'ตรง'} · φ ${fmt.deg(p.phiDeg)} · d ${fmt.num(p.distM)} m`;
      const leg = p.kind === 'detour' ? ` · ขับ a ${fmt.num(p.a)} m → เลี้ยว β* ${fmt.deg(p.betaDeg)} → ขับ b ${fmt.num(p.b)} m` : '';
      const extra = nav.overshoots ? ` · เลี้ยวเกิน ${nav.overshoots} ครั้ง` : '';
      this.plan.textContent = `${head}${leg} · ~${fmt.num(p.timeS, 1)} s${extra}`;
    } else {
      this.plan.textContent = ' ';
    }

    const offline = conn === 'offline';
    const n = this.route.points.length, slot = this.route.slot;
    this.startBtn.disabled = offline || running || !n || !this.route.loaded || !this.ready.checked || !!slot;
    this.stopBtn.disabled = offline || !running;
    this.resetBtn.disabled = offline || running || robot.mode !== 'halt';
    let hint = '', warn = false;
    if (offline) { hint = 'ติดต่อหุ่นไม่ได้: ปุ่มจะใช้ได้เมื่อต่อกลับ'; warn = true; }
    else if (running && nav.byButton) hint = 'เริ่มจากปุ่มบนหุ่น: กดปุ่มอีกครั้ง หรือ E-STOP เพื่อหยุด';
    else if (running) hint = 'ถ้าปิดหน้านี้หรือเน็ตหลุดเกิน 3 วินาที หุ่นจะหยุดเอง';
    else if (robot.source === 'ros' && robot.mode !== 'halt') { hint = 'หุ่นกำลังทำตามคำสั่งจาก ROS อยู่'; warn = true; }
    else if (slot) hint = `กำลังแก้เส้นทางปุ่ม ${slot} ครั้ง: ปุ่มเริ่มวิ่งใช้เส้นทางหน้าเว็บ เลือก "หน้าเว็บ" ก่อน`;
    else if (!this.route.loaded) hint = 'รอโหลดจุดจากหุ่น';
    else if (!n) hint = 'วางจุดบนระนาบก่อน แล้วจึงเริ่มวิ่ง';
    else if (!robot.imuOk || !robot.steerOk) { hint = 'เซนเซอร์ยังไม่พร้อม (ดูช่องด้านบน): หุ่นอาจวิ่งผิดทิศ'; warn = true; }
    else if (this.route.dirty) hint = 'จุดยังไม่ได้บันทึก: กดเริ่มวิ่งแล้วจะบันทึกให้ก่อน';
    else hint = `พร้อมวิ่ง ${n} จุด`;
    this.hint.textContent = hint;
    this.hint.classList.toggle('warn-text', warn);
    // the demo button on the robot (GPIO19): live state + last event
    const b = nav.button || {};
    const last = b.text && b.ageMs < 120000 ? ` · ล่าสุด: ${b.text}` : '';
    const btn = document.getElementById('btnInfo');
    if (btn) btn.textContent = `ปุ่มบนหุ่น: ${b.pressed ? 'กำลังกดอยู่' : 'ปล่อยอยู่'}${last}`;
  }
}

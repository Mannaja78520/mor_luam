// Settings tab: robot settings (a form built from SETTINGS_SCHEMA) and PIDF.

// One line per setting. To add one: add it here AND in src/app/Settings.cpp
// (struct, update() with its check, toJson()). Types: text number check select password.
const SETTINGS_SCHEMA = [
  { group: 'หุ่น' },
  { key: 'robotName', label: 'ชื่อหุ่น', type: 'text', max: 31, help: 'แสดงบนหน้านี้และตอนค้นหาหุ่นในวง' },
  { key: 'hostname', label: 'ชื่อในวง (mDNS)', type: 'text', max: 31, unit: '.local', help: 'a-z 0-9 และ - เท่านั้น · เปิดหน้านี้ด้วย http://ชื่อ.local' },
  { group: 'วิ่งตามจุด' },
  { key: 'planner', label: 'Algorithm', type: 'select', from: 'planners', names: PLANNER_NAMES,
    help: 'Detour Steer: ถ้าจุดอยู่ด้านที่ล้อต้องหมุนไกล จะขับอ้อมแทน (HW04) · แก้ algorithm ได้ใน src/algorithm/' },
  { key: 'navSpeedMps', label: 'ความเร็ววิ่ง', type: 'number', min: 0.005, max: 0.035, step: 0.001, unit: 'm/s',
    help: 'ล้อหมุนเต็มกำลังได้ ~0.039 m/s (9.7 rpm) ค่าแนะนำ 0.03 · ใช้คำนวณ Detour Steer ด้วย' },
  { key: 'navTolM', label: 'ถือว่าถึงจุดเมื่อห่างไม่เกิน', type: 'number', min: 0.02, max: 0.5, step: 0.01, unit: 'm' },
  { key: 'steerDps', label: 'ความเร็วเลี้ยวล้อ (ω)', type: 'number', min: 5, max: 720, step: 1, unit: '°/s', help: 'ใช้คำนวณว่าอ้อมหรือเลี้ยวตรงเร็วกว่า: ตั้งให้ใกล้ความเร็วเลี้ยวจริง' },
  { key: 'navLoop', label: 'วนเส้นทางซ้ำ (ถึงจุดสุดท้ายแล้วกลับไปจุดแรก)', type: 'check' },
  { group: 'ROS 2' },
  { key: 'agentHost', label: 'เครื่องที่รัน micro-ROS agent', type: 'text', max: 63, help: 'เว้นว่าง = หาเอง (เครื่องที่ปล่อย WiFi) · ใส่ IP หรือชื่อ .local ได้' },
  { key: 'agentPort', label: 'port ของ agent', type: 'number', min: 1, max: 65535, step: 1, unit: 'UDP' },
  { group: 'รหัสผ่าน' },
  { key: 'otaPass', label: 'รหัส OTA', type: 'password', min: 4, max: 63, help: 'ใช้ตอนอัปโหลดเฟิร์มแวร์ผ่าน WiFi' },
  { key: 'apPass', label: 'รหัส hotspot ของหุ่น', type: 'password', min: 8, max: 63, help: 'hotspot mor-luam-XXXX เปิดเมื่อหุ่นต่อ WiFi ไม่ได้' },
];

// Builds labelled inputs from a schema, fills them, reads them back, checks them.
class SchemaForm {
  constructor(form, schema, onInput) {
    this.schema = schema.filter((f) => f.key);
    this.inputs = {};
    this.base = {};
    let box = form;
    for (const f of schema) {
      if (f.group) { box = el('fieldset', {}, el('legend', { text: f.group })); form.append(box); continue; }
      box.append(this.field(f));
    }
    form.addEventListener('input', onInput);
    form.addEventListener('change', onInput);
  }

  field(f) {
    const id = 'set-' + f.key;
    let inp;
    if (f.type === 'select') inp = el('select', { id });
    else if (f.type === 'check') inp = el('input', { id, type: 'checkbox' });
    else inp = el('input', {
      id, type: f.type === 'password' ? 'password' : f.type === 'number' ? 'number' : 'text',
      min: f.min, max: f.type === 'number' ? f.max : undefined, step: f.step, maxlength: f.type !== 'number' ? f.max : undefined,
      inputmode: f.type === 'number' ? 'decimal' : undefined, autocapitalize: 'off', spellcheck: 'false',
    });
    this.inputs[f.key] = inp;
    if (f.type === 'check') return el('label', { class: 'field check', for: id }, inp, el('span', { text: f.label }));
    let control = inp;
    if (f.type === 'password') control = el('span', { class: 'pass' }, inp, el('button', { class: 'iconbtn', type: 'button', 'data-reveal': id }, 'แสดง'));
    else if (f.unit) control = el('span', { class: 'unit' }, inp, el('em', { text: f.unit }));
    return el('label', { class: 'field', for: id }, el('span', { text: f.label }), control, f.help ? el('small', { text: f.help }) : null);
  }

  fill(obj) {
    for (const f of this.schema) {
      const inp = this.inputs[f.key];
      if (f.type === 'select') {
        const names = obj[f.from] || [obj[f.key]];
        inp.replaceChildren(...names.map((n) => el('option', { value: n }, (f.names && f.names[n]) || n)));
      }
      if (f.type === 'check') inp.checked = !!obj[f.key];
      else inp.value = obj[f.key] ?? '';
    }
    this.base = this.read();
  }

  read() {
    const o = {};
    for (const f of this.schema) {
      const inp = this.inputs[f.key];
      o[f.key] = f.type === 'check' ? inp.checked : f.type === 'number' ? parseFloat(inp.value) : inp.value.trim();
    }
    return o;
  }

  get dirty() { return JSON.stringify(this.read()) !== JSON.stringify(this.base); }

  // first problem as a sentence, or ''
  check() {
    const o = this.read();
    for (const f of this.schema) {
      const v = o[f.key], inp = this.inputs[f.key];
      let bad = '';
      if (f.type === 'number' && !(Number.isFinite(v) && v >= f.min && v <= f.max)) bad = `${f.label}: ต้องอยู่ระหว่าง ${f.min}-${f.max}`;
      if ((f.type === 'text' || f.type === 'password') && f.min && v.length < f.min) bad = `${f.label}: ต้องยาวอย่างน้อย ${f.min} ตัว`;
      inp.classList.toggle('bad', !!bad);
      if (bad) return bad;
    }
    return '';
  }
}

class SettingsView {
  constructor(api, toast, onSaved) {
    Object.assign(this, { api, toast, onSaved });
    this.loaded = false;
    this.form = new SchemaForm($('#settingsForm'), SETTINGS_SCHEMA, () => this.renderDirty());
    wireRevealButtons($('#settingsForm'));
    this.saveBtn = $('#settingsSave');
    this.saveBtn.onclick = () => this.save();
    $('#settingsRevert').onclick = () => this.load();
    this.renderDirty();
  }

  async load() {
    const r = await this.api.get('/api/settings');
    const line = $('#settingsDirty');
    if (r.ok) { this.form.fill(r.data); this.loaded = true; this.renderDirty(); }
    else { line.textContent = `โหลดการตั้งค่าไม่ได้: ${r.error}`; line.className = 'hint reserve err-text'; }
    return r;
  }

  renderDirty() {
    const dirty = this.loaded && this.form.dirty;
    const line = $('#settingsDirty');
    line.className = 'hint reserve warn-text';
    line.textContent = !this.loaded ? 'กำลังโหลด…' : dirty ? 'มีค่าที่ยังไม่ได้บันทึก' : ' ';
    this.saveBtn.disabled = !dirty;
    this.saveBtn.classList.toggle('attention', dirty);
  }

  async save() {
    const bad = this.form.check();
    if (bad) { this.toast.show(bad, true); return; }
    const o = this.form.read();
    const hostChanged = o.hostname !== this.form.base.hostname;
    const r = await this.api.post('/api/settings', o);
    if (!this.toast.result(r, hostChanged ? `บันทึกแล้ว: ต่อไปเปิดหน้านี้ด้วย http://${o.hostname}.local` : 'บันทึกการตั้งค่าแล้ว')) return;
    this.form.base = o;
    this.renderDirty();
    this.onSaved(o);
  }
}

// PIDF gains of the two loops. Applied at once in RAM; a reboot goes back to
// config/PIDF_config.h (copy good values there to keep them).
class PidView {
  constructor(api, toast) {
    Object.assign(this, { api, toast });
    this.box = $('#pidBody');
    this.loops = [
      { key: 'spin', title: 'ล้อขับ (spin) · หน่วย rpm' },
      { key: 'steer', title: 'ล้อเลี้ยว (steer) · หน่วยองศา' },
    ];
    this.box.replaceChildren(el('p', { class: 'loading' }, 'กำลังโหลด…'));
  }

  async load() {
    const r = await this.api.get('/api/pid');
    if (!r.ok) {
      this.box.replaceChildren(el('div', { class: 'errbox' }, `โหลด PID ไม่ได้: ${r.error}`,
        el('button', { class: 'btn', type: 'button', onclick: () => this.load() }, 'ลองใหม่')));
      return r;
    }
    this.box.replaceChildren(...this.loops.map((l) => this.loopForm(l, r.data[l.key] || [])));
    return r;
  }

  loopForm(loop, values) {
    const names = ['Kp', 'Ki', 'Kd', 'Kf', 'tol'];
    const inputs = names.map((n, i) => el('input', {
      type: 'number', step: 'any', inputmode: 'decimal', value: String(+(+values[i]).toFixed(4)),
      'aria-label': `${loop.key} ${n}`,
    }));
    const save = async () => {
      const v = inputs.map((x) => parseFloat(x.value));
      if (!v.every(Number.isFinite) || v.some((x) => x < 0)) { this.toast.show('ค่า PID ต้องเป็นตัวเลขไม่ติดลบ', true); return; }
      this.toast.result(await this.api.post('/api/pid', { loop: loop.key, values: v }), `ใช้ค่า PID ${loop.key} แล้ว (ถึงรีบูต)`);
    };
    return el('fieldset', { class: 'form' }, el('legend', { text: loop.title }),
      el('div', { class: 'pidgrid' }, ...names.map((n, i) => el('label', { class: 'field' }, el('span', { text: n }), inputs[i]))),
      el('div', { class: 'row' }, el('button', { class: 'btn save', type: 'button', onclick: save }, `ใช้ค่า ${loop.key}`)));
  }

  // Learned steering coast and drive power (from /api/status).
  renderCoast(robot) {
    fillKv($('#coastTable'), [
      ['สวิตช์ฉุกเฉินจริง', 'ไม่มีสายสัญญาณเข้า ESP32 · ตรวจสถานะไม่ได้'],
      ['หุ่นสัมผัสพื้น', 'ไม่ทราบ · ไม่มีเซนเซอร์ตรวจพื้น'],
      ['IMU ขณะนี้', robot.imuMotionFresh ? `${fmt.num(robot.imuGyroDps, 1)} °/s · ${fmt.num(robot.imuAccelMps2, 2)} m/s² · ดูการสั่นในไฟล์ trace` : 'ข้อมูลการหมุน/ความเร่งยังไม่พร้อม'],
      ['การตอบสนองมอเตอร์', robot.motionFault ? 'ไม่มีสัญญาณหมุน · หยุดแล้ว · ตรวจไฟ/มอเตอร์/เซนเซอร์' : 'ยังไม่พบข้อผิดพลาด · ไม่ใช่หลักฐานว่าสวิตช์หรือพื้นพร้อม'],
      ['ตัวคูณกำลังล้อขับ (เรียนรู้เอง)', `${fmt.num(robot.driveGain, 3)} เท่า · จากการวิ่งคงที่ ${fmt.num(robot.driveLearnedS, 1)} s`],
      ['เวลาไถลของล้อเลี้ยว (เรียนรู้เอง)', `${fmt.num(robot.coastS, 3)} s · จาก ${robot.coastSamples} ครั้ง`],
      ['ความเร็วเลี้ยวตอนนี้', `${fmt.num(robot.steerRateDps, 0)} °/s${robot.coasting ? ' · กำลังไถล' : ''}`],
      ['กำลังเลี้ยวสูงสุด', `${robot.steerPowerLimit ?? '–'} PWM · จำกัดเพื่อลดการสั่น`],
      ['เลี้ยวเกินเป้าล่าสุด', robot.overshot ? `${fmt.num(robot.overshootDeg, 1)}°` : 'ไม่มี'],
    ]);
  }
}

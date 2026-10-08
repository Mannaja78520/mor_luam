// Wi-Fi tab: the link now, the saved networks (passwords visible on request),
// add / edit / delete / reorder, and a scan of what the robot can hear.
// Firmware side: net/WifiStore (the list) and net/NetworkManager (the link).

class WifiView {
  constructor(api, toast) {
    Object.assign(this, { api, toast });
    this.saved = null;               // null = not loaded yet
    this.max = 6;
    this.loadError = '';
    this.net = null;
    this.showAll = false;
    this.shown = new Set();          // SSIDs whose password is visible
    this.editing = '';               // SSID being edited ('' = adding)
    this.scan = null;

    this.form = $('#wifiForm');
    this.ssid = $('#wifiSsid');
    this.pass = $('#wifiPass');
    this.form.onsubmit = (e) => { e.preventDefault(); this.save(); };
    $('#wifiCancel').onclick = () => this.edit('');
    $('#wifiShowAll').onclick = (e) => {
      this.showAll = !this.showAll;
      e.target.classList.toggle('active', this.showAll);
      e.target.setAttribute('aria-pressed', String(this.showAll));
      e.target.textContent = this.showAll ? 'ซ่อนรหัสทั้งหมด' : 'แสดงรหัสทั้งหมด';
      this.renderSaved();
    };
    $('#wifiReconnect').onclick = async () => this.toast.result(
      await api.post('/api/wifi/reconnect'), 'หุ่นกำลังเชื่อมต่อใหม่: หน้านี้อาจหลุดสักครู่');
    $('#scanBtn').onclick = () => this.startScan();
    this.renderSaved();
    this.renderScan();
  }

  async load() {
    const r = await this.api.get('/api/wifi');
    if (r.ok) {
      this.saved = r.data.saved || [];
      this.max = r.data.max || this.max;
      this.loadError = '';
      this.renderLink(r.data.net);
    } else {
      this.loadError = r.error;
    }
    this.renderSaved();
    return r;
  }

  // ---- the link now (also refreshed from every /api/status) ---------------
  renderLink(net) {
    if (!net) return;
    this.net = net;
    const ap = net.ap || {};
    fillKv($('#linkTable'), [
      ['สถานะ', net.connected ? 'ต่อ WiFi แล้ว' : net.connecting ? `กำลังต่อ ${net.connecting}…` : 'ยังไม่ได้ต่อ WiFi'],
      ['WiFi', net.connected ? `${net.ssid}${net.priority ? ` (ลำดับ ${net.priority})` : ' (ไม่อยู่ในรายการ)'}` : '–'],
      ['สัญญาณ', net.connected ? fmt.rssi(net.rssi) : '–'],
      ['IP', net.connected ? net.ip : '–'],
      ['ชื่อในวง', net.host],
      ['Gateway (agent)', net.connected ? net.gateway : '–'],
      ['Hotspot ของหุ่น', ap.active ? `${ap.ssid} · ${ap.ip} · ${ap.clients} เครื่อง` : 'ปิด'],
    ]);
    // the "connected" tag on the saved list follows the link
    if (this.saved && this.lastSsid !== (net.connected ? net.ssid : '')) {
      this.lastSsid = net.connected ? net.ssid : '';
      this.renderSaved();
    }
  }

  // ---- saved networks -----------------------------------------------------
  renderSaved() {
    const box = $('#wifiSaved');
    $('#wifiCount').textContent = this.saved ? `${this.saved.length} / ${this.max}` : '–';
    if (!this.saved) {
      box.replaceChildren(this.loadError
        ? el('div', { class: 'errbox' }, `โหลดรายการ WiFi ไม่ได้: ${this.loadError}`,
          el('button', { class: 'btn', type: 'button', onclick: () => this.load() }, 'ลองใหม่'))
        : el('p', { class: 'loading' }, 'กำลังโหลด…'));
      return;
    }
    if (!this.saved.length) {
      box.replaceChildren(el('div', { class: 'empty' }, el('b', { text: 'ยังไม่มี WiFi ที่บันทึก' }),
        'เพิ่มด้วยฟอร์ม "เพิ่ม WiFi" หรือแตะชื่อในรายการสแกน'));
      return;
    }
    const cur = this.net && this.net.connected ? this.net.ssid : '';
    box.replaceChildren(el('div', { class: 'list' }, ...this.saved.map((w, i) => {
      const visible = this.showAll || this.shown.has(w.ssid);
      const passText = !w.pass ? 'ไม่มีรหัส' : visible ? w.pass : '•'.repeat(Math.min(w.pass.length, 12));
      return el('div', { class: 'item' + (w.ssid === cur ? ' cur' : '') },
        el('div', {},
          el('div', { class: 'name' }, `${i + 1}. ${w.ssid}`, w.ssid === cur ? el('span', { class: 'tag ok', text: 'ต่ออยู่' }) : null),
          el('div', { class: 'sub' }, 'รหัส: ', el('span', { class: 'mono', text: passText }))),
        el('div', { class: 'acts' },
          w.pass ? el('button', { class: 'iconbtn', type: 'button', disabled: this.showAll, onclick: () => this.toggle(w.ssid) }, visible ? 'ซ่อน' : 'แสดง') : null,
          el('button', { class: 'iconbtn', type: 'button', onclick: () => this.edit(w.ssid) }, 'แก้'),
          el('button', { class: 'iconbtn', type: 'button', disabled: i === 0, title: 'เลื่อนขึ้น', 'aria-label': `เลื่อน ${w.ssid} ขึ้น`, onclick: () => this.move(w.ssid, i - 1) }, '↑'),
          el('button', { class: 'iconbtn danger', type: 'button', onclick: () => this.remove(w.ssid) }, 'ลบ')));
    })));
  }

  toggle(ssid) { this.shown.has(ssid) ? this.shown.delete(ssid) : this.shown.add(ssid); this.renderSaved(); }

  edit(ssid) {
    this.editing = ssid;
    const w = (this.saved || []).find((x) => x.ssid === ssid);
    this.ssid.value = w ? w.ssid : '';
    this.pass.value = w ? w.pass : '';
    $('#wifiFormTitle').textContent = w ? `แก้ WiFi: ${w.ssid}` : 'เพิ่ม WiFi';
    $('#wifiCancel').classList.toggle('hidden', !w);
    if (w) { $('#wifiFormCard').scrollIntoView({ behavior: 'smooth', block: 'start' }); this.pass.focus(); }
  }

  async save() {
    const ssid = this.ssid.value.trim(), pass = this.pass.value;
    const bytes = new TextEncoder().encode(ssid).length;
    if (!bytes || bytes > 32) { this.toast.show('ชื่อ WiFi ต้องยาว 1-32 ตัว', true); return; }
    if (pass && (pass.length < 8 || pass.length > 63)) { this.toast.show('รหัส WiFi ต้องยาว 8-63 ตัว หรือเว้นว่าง', true); return; }
    const r = await this.api.post('/api/wifi/save', { ssid, pass, original: this.editing });
    if (!this.toast.result(r, `บันทึก ${ssid} แล้ว`)) return;
    this.edit('');
    this.load();
  }

  async remove(ssid) {
    const last = this.saved.length === 1;
    const msg = `ลบ WiFi "${ssid}"?` + (last ? '\nนี่คือวงสุดท้าย: ถ้าหุ่นหลุดจากวงนี้ จะเปิด hotspot ของตัวเองให้ตั้งค่าใหม่' : '');
    if (!confirm(msg)) return;
    if (this.toast.result(await this.api.post('/api/wifi/delete', { ssid }), `ลบ ${ssid} แล้ว`)) this.load();
  }

  async move(ssid, to) {
    if (this.toast.result(await this.api.post('/api/wifi/move', { ssid, to }), 'ย้ายลำดับแล้ว')) this.load();
  }

  // ---- scan ---------------------------------------------------------------
  async startScan() {
    const btn = $('#scanBtn');
    btn.disabled = true;
    const r = await this.api.post('/api/wifi/scan');
    if (!r.ok) { this.toast.show(r.error, true); btn.disabled = false; return; }
    this.scan = { scanning: true, nets: this.scan ? this.scan.nets : [] };
    this.renderScan();
    for (let i = 0; i < 25; ++i) {                     // a scan takes ~2-5 s
      await new Promise((ok) => setTimeout(ok, 800));
      const s = await this.api.get('/api/wifi/scan');
      if (s.ok && !s.data.scanning) { this.scan = s.data; break; }
    }
    if (this.scan.scanning) { this.scan.scanning = false; this.scan.error = 'สแกนไม่เสร็จ: ลองกดอีกครั้ง'; }
    btn.disabled = false;
    this.renderScan();
  }

  renderScan() {
    const box = $('#scanBody'), s = this.scan;
    $('#scanAge').textContent = s && s.ageMs ? fmt.age(s.ageMs) : '';
    if (!s) {
      box.replaceChildren(el('div', { class: 'empty' }, el('b', { text: 'ยังไม่ได้สแกน' }), 'กด "สแกน" เพื่อดู WiFi ที่หุ่นได้ยิน'));
      return;
    }
    if (s.scanning) { box.replaceChildren(el('p', { class: 'loading', 'aria-live': 'polite' }, 'กำลังสแกน… (2-5 วินาที)')); return; }
    if (s.error) { box.replaceChildren(el('div', { class: 'errbox' }, s.error)); return; }
    if (!s.nets.length) { box.replaceChildren(el('div', { class: 'empty' }, el('b', { text: 'ไม่พบ WiFi' }), 'ลองย้ายหุ่นเข้าใกล้ router แล้วสแกนอีกครั้ง')); return; }
    const known = new Set((this.saved || []).map((w) => w.ssid));
    $('#scanList').replaceChildren(...s.nets.map((n) => el('option', { value: n.ssid })));
    box.replaceChildren(el('div', { class: 'list' }, ...s.nets.map((n) => el('div', { class: 'item' },
      el('button', { class: 'pick', type: 'button', onclick: () => this.pick(n.ssid) },
        el('div', { class: 'name' }, n.ssid || '(ไม่มีชื่อ)', known.has(n.ssid) ? el('span', { class: 'tag', text: 'บันทึกแล้ว' }) : null),
        el('div', { class: 'sub' }, `${fmt.rssi(n.rssi)} · ${n.secure ? 'มีรหัส' : 'ไม่มีรหัส'}`)),
      el('button', { class: 'iconbtn', type: 'button', onclick: () => this.pick(n.ssid) }, 'ใช้วงนี้')))));
  }

  pick(ssid) {
    const w = (this.saved || []).find((x) => x.ssid === ssid);
    if (w) { this.edit(ssid); return; }
    this.edit('');
    this.ssid.value = ssid;
    $('#wifiFormCard').scrollIntoView({ behavior: 'smooth', block: 'start' });
    this.pass.focus();
  }
}

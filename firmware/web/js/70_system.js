// System tab: firmware upload (OTA), other robots/modules on the network,
// ROS link, system info, reboot, theme.

const APP_SLOT_BYTES = 1966080;       // min_spiffs.csv app partition

class OtaView {
  constructor(api, toast) {
    Object.assign(this, { api, toast });
    this.busyReason = '';
    this.uploading = false;
    this.btn = $('#otaBtn');
    this.msg = $('#otaMsg');
    this.prog = $('#otaProg');
    $('#otaForm').onsubmit = (e) => { e.preventDefault(); this.upload(); };
    wireRevealButtons($('#otaForm'));
  }

  // the robot refuses an update while it moves; say so before the upload
  setBusy(reason) {
    const changed = reason !== this.busyReason;
    this.busyReason = reason;
    this.btn.disabled = !!reason || this.uploading;
    // Status polling must not erase the upload's success/error message.
    if (!this.uploading && (changed || !this.msg.textContent.trim())) this.say(reason || ' ', !!reason);
  }

  say(text, warn = false, bad = false) {
    this.msg.textContent = text;
    this.msg.className = 'hint reserve' + (bad ? ' err-text' : warn ? ' warn-text' : '');
  }

  async upload() {
    const file = $('#otaFile').files[0], pass = $('#otaPass').value;
    if (!file) { this.say('เลือกไฟล์ firmware.bin ก่อน', false, true); return; }
    if (!file.name.endsWith('.bin') || file.size > APP_SLOT_BYTES) { this.say('ต้องเป็นไฟล์ .bin ขนาดไม่เกิน 1.9 MB', false, true); return; }
    if (!pass) { this.say('ใส่รหัส OTA ก่อน (ดูในแท็บตั้งค่า)', false, true); return; }
    this.uploading = true;
    this.btn.disabled = true;
    this.say('กำลังอัปโหลด… อย่าปิดหน้านี้หรือปิดหุ่น');
    const r = await this.api.upload('/api/ota', file, { 'X-OTA-Pass': pass }, (f) => {
      this.prog.style.width = `${Math.round(f * 100)}%`;
      this.say(`กำลังอัปโหลด ${Math.round(f * 100)}%… อย่าปิดหน้านี้หรือปิดหุ่น`);
    });
    this.uploading = false;
    if (r.ok) {
      this.prog.style.width = '100%';
      this.say('อัปโหลดเสร็จ หุ่นกำลังรีบูต (~10 วินาที) หน้านี้จะต่อกลับเอง');
      this.toast.show('อัปเดตเฟิร์มแวร์สำเร็จ');
    } else {
      this.prog.style.width = '0';
      this.say(`อัปโหลดไม่สำเร็จ: ${r.error}`, false, true);
      this.btn.disabled = !!this.busyReason;
    }
  }
}

// Other robots and modules found by mDNS (_module._tcp, like the mice project).
class FleetView {
  constructor(api) {
    this.api = api;
    this.data = null;
    this.error = '';
    $('#peersBtn').onclick = () => this.search();
    this.render();
  }

  async load() {
    const r = await this.api.get('/api/peers');
    if (r.ok) { this.data = r.data; this.error = ''; } else this.error = r.error;
    this.render();
    return r;
  }

  async search() {
    const btn = $('#peersBtn');
    btn.disabled = true;
    const r = await this.api.post('/api/peers');
    if (r.ok) {
      this.data = { ...(this.data || { peers: [] }), searching: true };
      this.render();
      for (let i = 0; i < 15; ++i) {
        await new Promise((ok) => setTimeout(ok, 700));
        const g = await this.api.get('/api/peers');
        if (g.ok && !g.data.searching) { this.data = g.data; break; }
      }
      if (this.data.searching) { this.data.searching = false; this.error = 'ค้นหาไม่เสร็จ: ลองกดอีกครั้ง'; }
    } else {
      this.error = r.error;
    }
    btn.disabled = false;
    this.render();
  }

  render() {
    const box = $('#peersBody'), d = this.data;
    if (this.error) { box.replaceChildren(el('div', { class: 'errbox' }, this.error)); this.error = ''; return; }
    if (!d) { box.replaceChildren(el('p', { class: 'loading' }, 'กำลังโหลด…')); return; }
    if (d.searching) { box.replaceChildren(el('p', { class: 'loading', 'aria-live': 'polite' }, 'กำลังค้นหาในวง… (ไม่เกิน 10 วินาที)')); return; }
    if (!d.peers.length) {
      box.replaceChildren(el('div', { class: 'empty' }, el('b', { text: d.ageMs ? 'ไม่พบหุ่นหรือโมดูลอื่นในวง' : 'ยังไม่ได้ค้นหา' }),
        'กด "ค้นหา" · จากคอม ใช้ docker\\mor_luam.bat find'));
      return;
    }
    box.replaceChildren(
      el('p', { class: 'hint' }, `ค้นล่าสุด ${fmt.age(d.ageMs) || 'เมื่อกี้'}`),
      el('div', { class: 'list' }, ...d.peers.map((p) => el('div', { class: 'item' },
        el('div', {},
          el('div', { class: 'name' }, p.name || p.host, el('span', { class: 'tag', text: p.type || 'module' })),
          el('div', { class: 'sub' }, el('span', { class: 'mono', text: `${p.host} · ${p.ip}` }))),
        el('a', { class: 'btn', href: `http://${p.ip}/`, target: '_blank', rel: 'noopener' }, 'เปิด')))));
  }
}

class SystemView {
  render(s, conn) {
    const sys = s.sys, ros = s.ros, net = s.net;
    $('#robotName').textContent = sys.name || 'mor_luam';
    document.title = `${sys.name || 'mor_luam'} · mor_luam`;
    fillKv($('#sysTable'), [
      ['เฟิร์มแวร์', sys.fw],
      ['build', sys.build],
      ['เปิดมาแล้ว', fmt.uptime(sys.uptimeS)],
      ['RAM ว่าง', `${Math.round(sys.heap / 1024)} KB`],
      ['รอบควบคุม 100 Hz ใช้', `${sys.tickUs} µs จาก 10000`],
      ['ชื่อในวง / IP', `${net.host} · ${net.connected ? net.ip : (net.ap && net.ap.ip) || '–'}`],
      ['ข้อมูล', conn === 'live' ? 'สด' : conn === 'stale' ? 'ช้า' : 'หลุด'],
    ]);
    const state = { connected: 'ต่อแล้ว', waiting: 'รอ agent (ยังไม่เจอ)', 'no-wifi': 'ไม่มี WiFi' }[ros.state] || ros.state;
    fillKv($('#rosTable'), [
      ['สถานะ', state],
      ['agent', ros.agent],
      ['หา agent จาก', ros.from || '–'],
      ['ROS_DOMAIN_ID', ros.domain],
      ['ต่อสำเร็จ', `${ros.connects} ครั้ง`],
      ['ต่อมาแล้ว', ros.upMs ? fmt.uptime(Math.round(ros.upMs / 1000)) : '–'],
    ]);
  }
}

// auto (follow the device) -> light -> dark; remembered per browser
class ThemeSwitch {
  constructor(btn, onChange) {
    this.btn = btn;
    this.onChange = onChange;
    this.apply(Prefs.get('theme', 'auto'));
    btn.onclick = () => this.apply({ auto: 'light', light: 'dark', dark: 'auto' }[this.mode]);
  }
  apply(mode) {
    this.mode = mode;
    if (mode === 'auto') document.documentElement.removeAttribute('data-theme');
    else document.documentElement.setAttribute('data-theme', mode);
    this.btn.textContent = `ธีม: ${{ auto: 'ตามเครื่อง', light: 'สว่าง', dark: 'มืด' }[mode]}`;
    Prefs.set('theme', mode);
    this.onChange();
  }
}

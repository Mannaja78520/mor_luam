// Puts the page together. One object per part of the page:
//
//   Api, Poller          10_api.js       talk to the robot, poll /api/status 4x a second
//   RouteModel           30_route.js     the points being edited + the robot's copy (web route, button routes 1/2)
//   RouteSlotBar         40_nav.js       which of those routes the plane edits
//   DemoCompareView      47_demo_compare.js  picture of button demos 3/4 (Direct vs Detour)
//   Plane2D              35_plane2d.js   the canvas: draw / edit points, robot, trail
//   WaypointTable,
//   NavPanel             40_nav.js       point list, start / stop, plan, heartbeat
//   WifiView             50_wifi.js      Wi-Fi tab
//   SettingsView,
//   PidView              60_settings.js  settings tab (form built from SETTINGS_SCHEMA)
//   OtaView, FleetView,
//   SystemView           70_system.js    system tab
//   TopBar, Tabs, App    this file
//
// Data flow: Poller -> App.onStatus(s) -> each view's render(s). Views send
// commands with Api and call App.refresh() so the page reacts at once.

class Tabs {
  constructor(nav, onShow) {
    this.nav = nav;
    this.onShow = onShow;
    nav.addEventListener('click', (e) => {
      const b = e.target.closest('[data-tab]');
      if (b) this.show(b.dataset.tab);
    });
    this.show(Prefs.get('tab', 'drive'));
  }
  show(name) {
    if (!document.getElementById('tab-' + name)) name = 'drive';
    this.nav.querySelectorAll('[data-tab]').forEach((b) => {
      const on = b.dataset.tab === name;
      b.classList.toggle('active', on);
      b.setAttribute('aria-selected', String(on));
    });
    document.querySelectorAll('.tabpage').forEach((p) => p.classList.toggle('active', p.id === 'tab-' + name));
    Prefs.set('tab', name);
    this.onShow(name);
  }
}

// name, health pills, E-STOP; plus the banner under it
class TopBar {
  constructor(api, toast, onCommand) {
    this.banner = $('#banner');
    $('#estop').onclick = async () => {
      const r = await api.post('/api/estop', null, 2000);
      toast.result(r, 'หยุดฉุกเฉินแล้ว: ล้อหยุด เส้นทางหยุด');
      onCommand();
    };
    // the banner sits under the toolbar, whatever height the toolbar wraps to
    const bar = $('#topbar');
    new ResizeObserver(() => document.documentElement.style.setProperty('--toolbar-h', `${bar.offsetHeight}px`)).observe(bar);
  }

  pill(id, text, cls) {
    const p = document.getElementById(id);
    p.className = 'pill ' + cls;
    p.lastElementChild.textContent = text;
  }

  renderConn(conn) {
    const [t, c] = { live: ['สด', 'ok'], stale: ['ช้า', 'warn'], offline: ['หลุด', 'bad'], loading: ['…', ''] }[conn];
    this.pill('pillLink', t, c);
    this.offline = conn === 'offline';
    if (this.offline) $('#subtitle').textContent = 'ติดต่อหุ่นไม่ได้';
    this.renderBanner();
  }

  renderBanner() {
    let text = '', cls = '';
    if (this.offline) text = 'ติดต่อหุ่นไม่ได้: กำลังลองใหม่ ค่าที่เห็นเป็นค่าเก่า';
    else if (this.otaPct >= 0) { text = `กำลังอัปเดตเฟิร์มแวร์ ${this.otaPct}%: อย่าปิดหุ่น`; cls = 'warn'; }
    this.banner.textContent = text;
    this.banner.className = `banner ${cls}${text ? '' : ' hidden'}`;
  }

  render(s, conn) {
    const net = s.net, ros = s.ros, robot = s.robot, nav = s.nav;
    if (conn === 'offline') {                 // old values must not look live
      this.pill('pillWifi', '?', '');
      this.pill('pillRos', '?', '');
      this.otaPct = -1;
      this.renderBanner();
      return;
    }
    if (net.connected) this.pill('pillWifi', `${net.rssi} dBm`, net.rssi > -75 ? 'ok' : 'warn');
    else this.pill('pillWifi', net.ap && net.ap.active ? 'hotspot' : 'ไม่ต่อ', net.ap && net.ap.active ? 'warn' : 'bad');
    document.getElementById('pillWifi').title = net.connected ? net.ssid : 'ยังไม่ได้ต่อ WiFi';
    const rs = { connected: ['ต่อแล้ว', 'ok'], waiting: ['รอ agent', 'warn'], 'no-wifi': ['ไม่มีเน็ต', 'bad'] }[ros.state] || [ros.state, ''];
    this.pill('pillRos', rs[0], rs[1]);

    let sub = 'พร้อม';
    if (nav.status === 'running') sub = `วิ่งตามจุด ${nav.index + 1}/${nav.count}`;
    else if (robot.mode !== 'halt') sub = robot.source === 'ros' ? 'ทำตามคำสั่ง ROS' : 'กำลังเคลื่อนที่';
    else if (robot.haltWhy) sub = `หยุด: ${robot.haltWhy}`;
    if (conn !== 'offline') $('#subtitle').textContent = sub;

    this.otaPct = s.sys.otaPct;
    this.renderBanner();
  }
}

class Tiles {
  constructor() { this.first = true; }
  set(id, text, warn = false) {
    const n = document.getElementById(id);
    if (n.textContent !== text) n.textContent = text;
    n.classList.toggle('warn-text', warn);
  }
  render(r) {
    if (this.first) { document.querySelectorAll('#tiles .skeleton').forEach((n) => n.classList.remove('skeleton')); this.first = false; }
    this.set('tPos', `${fmt.num(r.x)}, ${fmt.num(r.y)}`);
    this.set('tTheta', fmt.deg(r.thetaDeg));
    this.set('tImu', r.imuOk ? 'IMU พร้อม' : 'IMU ไม่พร้อม', !r.imuOk);
    this.set('tWheel', fmt.deg(r.wheelHeadingDeg));
    this.set('tSteer', r.steerOk ? `ล้อ ${fmt.deg(r.steerDeg)} · ห่างเป้า ${fmt.deg(r.steerErrDeg)}` : 'เซนเซอร์ล้อไม่พร้อม', !r.steerOk);
    this.set('tRpm', `${Math.round(r.rpm)} rpm`);
    const mode = { halt: 'หยุดนิ่ง', steer: 'กำลังเลี้ยวล้อ', drive: 'กำลังขับ' }[r.mode] || r.mode;
    this.set('tMode', `${mode}${r.coasting ? ' · ไถล' : ''}${r.mode !== 'halt' && r.source !== 'none' ? ` · จาก ${r.source}` : ''}`);
  }
}

class App {
  constructor() {
    this.api = new Api();
    this.toast = new Toast($('#toast'));
    this.conn = 'loading';
    this.last = null;

    this.route = new RouteModel(this.api);
    this.plane = new Plane2D($('#plane'), this.route, {
      onCursor: (p) => { $('#cursorXY').textContent = p ? `x ${p[0].toFixed(2)}, y ${p[1].toFixed(2)} m` : 'x –, y –'; },
      onRefuse: (why) => this.toast.show(why, true),
      onFollowOff: () => this.setChip($('#followBtn'), false),
    });
    this.wpTable = new WaypointTable(this.route, this.toast);
    this.slots = new RouteSlotBar(this.route, this.toast, () => this.plane.fit());
    const refresh = () => this.poller.now();
    this.nav = new NavPanel(this.api, this.route, this.toast, { onPoseReset: () => this.plane.clearTrail(), onCommand: refresh });
    this.top = new TopBar(this.api, this.toast, refresh);
    this.tiles = new Tiles();
    this.wifi = new WifiView(this.api, this.toast);
    this.settings = new SettingsView(this.api, this.toast, refresh);
    this.pid = new PidView(this.api, this.toast);
    this.simulation = new RouteSimulation(this.route, () => this.last && this.last.robot);
    this.demo = new DemoCompareView(this.api);
    this.routeTest = new RouteTestPanel(this.api, this.route, this.toast, refresh, () => this.settings.loaded ? this.settings.form.base : null);
    this.ota = new OtaView(this.api, this.toast);
    this.fleet = new FleetView(this.api);
    this.system = new SystemView();
    new ThemeSwitch($('#themeBtn'), () => { this.plane.readColors(); this.plane.redraw(); this.demo.draw(); });
    this.wirePlaneTools();
    wireRevealButtons($('#wifiForm'));
    $('#rebootBtn').onclick = async () => {
      if (confirm('รีบูตหุ่นตอนนี้? หน้านี้จะหลุดประมาณ 10 วินาที')) this.toast.result(await this.api.post('/api/reboot'), 'กำลังรีบูต…');
    };
    new Tabs($('#tabs'), (name) => { if (name === 'drive') requestAnimationFrame(() => this.plane.resize()); });

    this.poller = new Poller(this.api, '/api/status', 250, {
      onData: (s) => this.onStatus(s),
      onConn: (c) => this.onConn(c),
    });
    this.poller.start();
    this.loads = [
      () => this.route.load().then((r) => { if (r.ok) this.plane.fit(); return r; }),
      () => this.demo.load(), () => this.wifi.load(), () => this.settings.load(), () => this.pid.load(), () => this.fleet.load(),
    ];
    this.loadAll();
  }

  // one-time loads; anything that failed is tried again when the link comes back
  async loadAll() {
    const failed = [];
    for (const f of this.loads) { const r = await f(); if (!r.ok) failed.push(f); }
    this.loads = failed;
  }

  onConn(c) {
    const wasOffline = this.conn === 'offline';
    this.conn = c;
    document.body.dataset.conn = c;
    this.top.renderConn(c);
    if (this.last) this.render(this.last);
    if (wasOffline && c === 'live' && this.loads.length) this.loadAll();
  }

  onStatus(s) {
    this.last = s;
    this.render(s);
    if (!this.fittedRobot && this.route.loaded) { this.fittedRobot = true; this.plane.fit(); }   // once: points + robot in view
  }

  render(s) {
    const r = s.robot, nav = s.nav, running = nav.status === 'running';
    this.top.render(s, this.conn);
    this.tiles.render(r);
    const locked = running ? 'หุ่นกำลังวิ่งตามเส้นทาง: หยุดก่อนแล้วค่อยแก้จุด' : '';
    this.plane.locked = locked;
    const lock = $('#planeLock');
    lock.textContent = locked;
    lock.classList.toggle('hidden', !locked);
    this.plane.setRobot(r, nav, this.conn !== 'live');
    this.wpTable.setState(running ? nav.index : -1, locked);
    this.nav.render(s, this.conn);
    this.routeTest.render(s, this.conn);
    this.demo.onStatus(s, this.conn);
    this.wifi.renderLink(s.net);
    this.pid.renderCoast(r);
    this.system.render(s, this.conn);
    this.ota.setBusy(this.conn === 'offline' ? 'ติดต่อหุ่นไม่ได้'
      : running || r.mode !== 'halt' ? 'หุ่นกำลังเคลื่อนที่: กดหยุดก่อนแล้วค่อยอัปเดต' : '');
  }

  setChip(b, on) { b.classList.toggle('active', on); b.setAttribute('aria-pressed', String(on)); }

  wirePlaneTools() {
    document.querySelectorAll('[data-mode]').forEach((b) => {
      b.onclick = () => {
        this.plane.mode = b.dataset.mode;
        document.querySelectorAll('[data-mode]').forEach((x) => this.setChip(x, x === b));
      };
    });
    $('#snapBtn').onclick = (e) => { this.plane.snap = !this.plane.snap; this.setChip(e.target, this.plane.snap); };
    $('#followBtn').onclick = (e) => {
      this.plane.follow = !this.plane.follow;
      this.setChip(e.target, this.plane.follow);
      if (this.plane.follow && this.last) this.plane.setRobot(this.last.robot, this.last.nav, this.conn !== 'live');
    };
    $('#fitBtn').onclick = () => this.plane.fit();
    $('#trailBtn').onclick = () => this.plane.clearTrail();
  }
}

window.app = new App();

// Route tab: a picture of the button demos 3 (Direct) and 4 (Detour), from the
// robot's own record (GET /api/demo/compare). Each run is drawn in its own start
// frame - start at the origin, the wheel's start heading to the right - so both
// runs share one start and one goal. Dashed = the plan, solid = where the robot
// went (odometry). Turning in place leaves no line, so each turn is marked and
// labelled. "Test 3 rounds" (POST /api/demo/series/start) makes the robot run
// Direct and Detour 3 times each; then the card shows every run (GET /api/demo/series).
// The page only starts/stops it; the robot runs the series itself.

const DEMO_RUNS = [
  { key: 'direct', name: 'Direct', clicks: 3 },
  { key: 'detour', name: 'Detour', clicks: 4 },
];

class DemoCompareView {
  constructor(api, toast, onCommand = () => {}) {
    Object.assign(this, { api, toast, onCommand });
    this.series = null;
    this.seriesKey = '';
    this.seriesPaths = new Map();              // run id -> flat xy (each run fetched once)
    this.seriesLoading = false;
    this.seriesReady = $('#seriesReady');
    this.seriesStart = $('#seriesStart');
    this.seriesStop = $('#seriesStop');
    this.seriesProg = $('#seriesProg');
    this.seriesMsg = $('#seriesMsg');
    this.lastStatus = null;
    this.seriesReady.onchange = () => this.renderSeriesControls();
    this.seriesStart.onclick = () => this.startSeries();
    this.seriesStop.onclick = async () => {
      this.toast.result(await this.api.post('/api/nav/stop'), 'หยุดชุดทดสอบแล้ว');
      this.onCommand();
    };
    this.plot = $('#demoPlot');
    this.timeline = $('#demoTimeline');
    this.canvas = $('#demoCanvas');
    this.result = $('#demoResult');
    this.hint = $('#demoHint');
    this.data = null;
    this.error = '';
    this.loading = false;
    this.conn = 'loading';
    this.key = '';
    this.lastLiveMs = 0;
    this.uptime = 0;
    new ResizeObserver(() => this.draw()).observe(this.plot);
    window.addEventListener('resize', () => this.draw());   // some views resize without telling the observer
    matchMedia('(prefers-color-scheme: light)').addEventListener('change', () => this.draw());
    this.render();
  }

  async load() {
    if (this.loading) return { ok: true };
    this.loading = true;
    const r = await this.api.get('/api/demo/compare');
    this.loading = false;
    if (r.ok) { this.data = r.data; this.error = ''; } else this.error = r.error || 'หุ่นไม่ตอบ';
    this.render();
    return r;
  }

  // the summary first, then each run's path on its own: the robot answers only
  // small replies reliably (one 6-run reply never arrived, 2026-10-10)
  async loadSeries() {
    if (this.seriesLoading) return { ok: true };
    this.seriesLoading = true;
    try {
      const r = await this.api.get('/api/demo/series', 6000);
      if (!r.ok) return r;
      const se = r.data;
      for (let i = 0; i < (se.runs || []).length; i++) {
        const run = se.runs[i];
        if (!this.seriesPaths.has(run.id)) {
          const one = await this.api.get(`/api/demo/series?run=${i}`, 6000);
          if (one.ok && one.data.id === run.id) this.seriesPaths.set(run.id, one.data.xy || []);
        }
        run.xy = this.seriesPaths.get(run.id) || [];
      }
      this.series = se;
      this.render();
      return r;
    } finally {
      this.seriesLoading = false;
    }
  }

  async startSeries() {
    if (!this.seriesReady.checked) { this.toast.show('ยืนยันว่าอยู่ข้างหุ่นและข้างหน้าว่างก่อน', true); return; }
    this.seriesReady.checked = false;
    const r = await this.api.post('/api/demo/series/start', { rounds: 3, ready: true });
    this.toast.result(r, 'เริ่มทดสอบ 3 รอบแล้ว: อย่าปิดหน้านี้');
    this.onCommand();
    this.renderSeriesControls();
  }

  // From the status poll: load again when a demo 3/4 run starts or ends, after a
  // reboot, and once a second while a run is being timed (the live path).
  onStatus(s, conn) {
    const was = this.conn;
    this.conn = conn;
    this.lastStatus = s;
    const se = s.nav.series || {};
    const seKey = `${se.id}:${se.count}:${se.active}`;
    if (conn === 'live' && se.id && seKey !== this.seriesKey) { this.seriesKey = seKey; this.loadSeries(); }
    this.renderSeriesControls();
    const t = s.nav.test || {};
    const key = `${t.id}:${t.phase}`;
    const rebooted = s.sys.uptimeS < this.uptime;
    this.uptime = s.sys.uptimeS;
    const live = t.demo && t.phase === 'running';
    if (conn === 'live' && ((t.demo && key !== this.key) || rebooted || (live && Date.now() - this.lastLiveMs > 1000))) {
      this.lastLiveMs = Date.now();
      this.load();
    }
    this.key = key;
    if (was !== conn) this.renderHint();
  }

  // ---- geometry ------------------------------------------------------------

  // a run in its own start frame: start at (0,0), the wheel's start heading = +x
  static local(run) {
    const h = (run.headingDeg * Math.PI) / 180, c = Math.cos(h), s = Math.sin(h);
    const f = (x, y) => { const dx = x - run.startX, dy = y - run.startY; return { x: dx * c + dy * s, y: -dx * s + dy * c }; };
    return { goal: f(run.goalX, run.goalY), path: (run.path || []).map((p) => f(p.x, p.y)) };
  }

  // the plan from the start, with the robot's maths (SimulationPlanner.leg = DetourSteer)
  static plan(goal, run, detour) {
    const d = Math.hypot(goal.x, goal.y);
    const phi = SimulationPlanner.wrap((Math.atan2(goal.y, goal.x) * 180) / Math.PI);
    const leg = SimulationPlanner.leg(phi, d, { v: run.speedMps, w: run.steerDps, settle: 0.05, stop: 0.2 }, detour);
    if (leg.kind === 'detour') {
      return { kind: 'detour', a: leg.a, turnAt: { x: leg.a, y: 0 }, turnDeg: leg.beta, pts: [{ x: 0, y: 0 }, { x: leg.a, y: 0 }, goal] };
    }
    return { kind: 'direct', a: 0, turnAt: { x: 0, y: 0 }, turnDeg: leg.beta, pts: [{ x: 0, y: 0 }, goal] };
  }

  static prepared(meta, run) {
    const loc = DemoCompareView.local(run);
    const plan = DemoCompareView.plan(loc.goal, run, meta.key === 'detour');
    const end = loc.path[loc.path.length - 1];
    const missMm = end ? 1000 * Math.hypot(end.x - loc.goal.x, end.y - loc.goal.y) : NaN;
    return { ...meta, run, ...loc, plan, missMm };
  }

  // a series run keeps its path as flat [x0, y0, x1, y1, ...]
  static unflat(run) {
    if (run.path || !run.xy) return run;
    const path = [];
    for (let i = 0; i + 1 < run.xy.length; i += 2) path.push({ x: run.xy[i], y: run.xy[i + 1] });
    return { ...run, path };
  }

  // the series is shown while it runs, and after it while its last run is the newest
  showSeries() {
    const se = this.series;
    if (!se || !se.runs || !se.runs.length) return false;
    if (se.active) return true;
    const d = this.data || {};
    const newest = Math.max(...DEMO_RUNS.map((r) => (d[r.key] ? d[r.key].id : 0)));
    return se.runs.some((r) => r.id === newest);
  }

  runs() {
    if (this.showSeries()) {
      const out = this.series.runs.map((run, i) => {
        const meta = DEMO_RUNS.find((m) => m.key === run.planner) || DEMO_RUNS[0];
        return DemoCompareView.prepared({ ...meta, label: `${Math.floor(i / 2) + 1}·${meta.name}` }, DemoCompareView.unflat(run));
      });
      const live = DEMO_RUNS.map((m) => (this.data || {})[m.key] && this.data[m.key].open ? m : null).find(Boolean);
      if (this.series.active && live) {        // the run being timed now, from the live record
        out.push(DemoCompareView.prepared({ ...live, label: `${Math.floor(out.length / 2) + 1}·${live.name}` }, this.data[live.key]));
      }
      return out;
    }
    const d = this.data;
    if (!d) return [];
    return DEMO_RUNS.filter((r) => d[r.key]).map((r) => DemoCompareView.prepared({ ...r, label: r.name }, d[r.key]));
  }

  // ---- "test 3 rounds" controls ------------------------------------------------

  renderSeriesControls() {
    const s = this.lastStatus;
    const se = (s && s.nav.series) || {};
    const offline = this.conn !== 'live';
    const running = s && s.nav.status === 'running';
    const busy = !!se.active || running;
    this.seriesStart.disabled = offline || busy || !this.seriesReady.checked;
    this.seriesStop.disabled = offline || !busy;
    this.seriesReady.disabled = busy;
    const total = se.total || 6;
    this.seriesProg.style.width = se.active || se.count ? `${(100 * (se.count || 0)) / total}%` : '0';
    let msg, warn = false;
    if (offline) { msg = 'ติดต่อหุ่นไม่ได้: ปุ่มจะใช้ได้เมื่อต่อกลับ'; warn = true; }
    else if (se.active) {
      const i = se.count || 0, t = s.nav.test || {};
      const what = running ? `${t.planner === 'detour' ? 'Detour' : 'Direct'} ${t.phase === 'aligning' ? 'กำลังตั้งล้อ' : 'กำลังวิ่ง'}` : 'พักก่อนครั้งต่อไป';
      msg = `รอบ ${Math.floor(i / 2) + 1} จาก ${total / 2} · ครั้งที่ ${Math.min(i + 1, total)}/${total}: ${what} · อย่าปิดหน้านี้`;
    } else if (se.why) { msg = `ชุดทดสอบหยุดก่อนครบ: ${se.why}`; warn = true; }
    else if (se.total && se.count === se.total) msg = 'ทดสอบครบแล้ว: ผลอยู่ด้านล่าง';
    else if (running) msg = 'หุ่นกำลังทำอย่างอื่นอยู่: หยุดก่อน';
    else if (!this.seriesReady.checked) msg = 'ติ๊กยืนยันก่อน แล้วกด "ทดสอบ 3 รอบ"';
    else msg = 'พร้อม: กด "ทดสอบ 3 รอบ"';
    this.seriesMsg.textContent = msg;
    this.seriesMsg.classList.toggle('warn-text', warn);
  }

  // ---- text: the results (also the accessible view of the picture) ----------

  render() { this.renderTimeline(); this.renderText(); this.renderHint(); this.draw(); }

  // segments [{a, t0, t1}] in seconds from the run's record (acts are change points)
  static segments(run) {
    const end = run.elapsedMs / 1000, acts = run.acts || [];
    return acts.map((x, i) => ({ a: x.a, t0: x.t / 1000, t1: i + 1 < acts.length ? acts[i + 1].t / 1000 : end }))
      .filter((s) => s.t1 > s.t0);
  }
  static total(segs, a) { return segs.filter((s) => s.a === a).reduce((sum, s) => sum + s.t1 - s.t0, 0); }

  // ---- the time line: the clearest way to see the difference ------------------

  renderTimeline() {
    const runs = this.runs().filter((r) => (r.run.acts || []).length);
    this.timeline.classList.toggle('hidden', !runs.length);
    if (!runs.length) return;
    const maxS = Math.max(...runs.map((r) => r.run.elapsedMs / 1000), 1);
    const pct = (t) => `${(100 * t) / maxS}%`;
    const word = { turn: 'หมุนล้อ', drive: 'วิ่ง', still: 'นิ่ง' };
    const series = this.showSeries();
    const done = runs.filter((r) => r.run.valid && !r.run.open);
    const fast = !series && done.length === 2 ? done.reduce((a, b) => (a.run.elapsedMs <= b.run.elapsedMs ? a : b)) : null;
    const slowS = fast ? Math.max(...done.map((r) => r.run.elapsedMs / 1000)) : 0;
    const rows = runs.map((r) => {
      const segs = DemoCompareView.segments(r.run);
      const labels = segs.filter((s) => s.a !== 'still' && (s.t1 - s.t0) / maxS >= 0.14)
        .map((s) => el('span', { class: 'tl-lab', style: `left:${pct((s.t0 + s.t1) / 2)}`,
          text: `${word[s.a]} ${(s.t1 - s.t0).toFixed(1)}s` }));
      const track = el('div', { class: 'tl-track' },
        ...segs.map((s) => el('div', { class: `tl-seg ${s.a}`, style: `left:${pct(s.t0)};width:${pct(s.t1 - s.t0)}`,
          title: `${word[s.a]} ${s.t0.toFixed(1)}-${s.t1.toFixed(1)} s` })));
      if (fast === r) {                                // the time it saved, on its own lane
        const endS = r.run.elapsedMs / 1000;
        track.append(el('div', { class: 'tl-gain', style: `left:${pct(endS)};width:${pct(slowS - endS)}`,
          title: `ถึงก่อน ${(slowS - endS).toFixed(2)} s` }));
        if ((slowS - endS) / maxS >= 0.1) labels.push(el('span', { class: 'tl-lab', style: `left:${pct((endS + slowS) / 2)}`, text: 'ถึงก่อน' }));
      }
      const time = r.run.open ? 'กำลังวิ่ง' : r.run.valid ? `${(r.run.elapsedMs / 1000).toFixed(1)} s` : 'ไม่ครบ';
      return el('div', { class: 'tl-row' },
        el('span', { class: 'tl-name' }, el('i', { class: `sw ${r.key}`, 'aria-hidden': 'true' }), r.label),
        el('div', { class: 'tl-lane' }, el('div', { class: 'tl-labels' }, ...labels), track),
        el('span', { class: 'tl-end', text: time }));
    });
    const step = maxS > 30 ? 10 : maxS > 12 ? 5 : 2;
    const ticks = [];
    for (let t = 0; t <= maxS + 1e-9; t += step) ticks.push(el('span', { style: `left:${pct(t)}`, text: `${t}` }));
    const stats = this.seriesStats();
    const head = series
      ? (stats.pairs ? stats.head : 'ชุดทดสอบ: แต่ละครั้งทำอะไรตอนไหน')
      : fast
        ? `${fast.name} ถึงเป้าก่อน ${(slowS - fast.run.elapsedMs / 1000).toFixed(1)} วินาที`
        : 'แต่ละรอบทำอะไรตอนไหน';
    // per planner (a series: the mean of its finished runs)
    const noteList = DEMO_RUNS.map((m) => {
      const mine = runs.filter((r) => r.key === m.key && !r.run.open);
      if (!mine.length) return '';
      const avg = (a) => mine.reduce((sum, r) => sum + DemoCompareView.total(DemoCompareView.segments(r.run), a), 0) / mine.length;
      return `${m.name}${mine.length > 1 ? ` (เฉลี่ย ${mine.length} ครั้ง)` : ''}: หมุนล้อ ${avg('turn').toFixed(1)} s · วิ่ง ${avg('drive').toFixed(1)} s`;
    }).filter(Boolean);
    const notes = noteList.join(' · ');
    this.timeline.replaceChildren(
      el('p', { class: 'tl-head', text: head }),
      el('div', { class: 'tl-legend', 'aria-hidden': 'true' },
        el('span', {}, el('i', { class: 'tl-key turn' }), 'หมุนล้ออยู่กับที่'),
        el('span', {}, el('i', { class: 'tl-key drive' }), 'วิ่ง'),
        el('span', {}, el('i', { class: 'tl-key still' }), 'นิ่ง (ตั้งล้อ/หยุดเปลี่ยนท่า)')),
      ...rows,
      el('div', { class: 'tl-axis', 'aria-hidden': 'true' }, ...ticks),
      el('p', { class: 'tl-note' }, ...noteList.map((n) => el('span', { class: 'tl-line', text: n })),
        el('span', { class: 'tl-line hint', text: 'แกนล่าง = วินาทีนับจากเริ่มจับเวลา' })));
    this.timeline.setAttribute('aria-label', `${head}. ${notes}`);
  }

  // Direct vs Detour per round of the series (runs 2r and 2r+1), only finished valid runs
  seriesStats() {
    const se = this.showSeries() ? this.series : null;
    if (!se) return { pairs: 0, rounds: [] };
    const rounds = [];
    for (let i = 0; i + 1 < se.runs.length; i += 2) {
      const pair = {};
      for (const run of [se.runs[i], se.runs[i + 1]]) pair[run.planner] = run;
      rounds.push(pair);
    }
    const ok = rounds.filter((p) => p.direct && p.detour && p.direct.valid && p.detour.valid);
    const mean = (k) => ok.reduce((s, p) => s + p[k].elapsedMs / 1000, 0) / ok.length;
    if (!ok.length) return { pairs: 0, rounds };
    const d = mean('direct'), t = mean('detour'), diff = d - t;
    const wins = ok.filter((p) => p.detour.elapsedMs < p.direct.elapsedMs).length;
    const head = diff > 0
      ? `Detour เร็วกว่าเฉลี่ย ${diff.toFixed(2)} s (${((100 * diff) / d).toFixed(1)}%) · ชนะ ${wins} จาก ${ok.length} รอบ`
      : `เฉลี่ยแล้ว Direct เร็วกว่า ${(-diff).toFixed(2)} s · Detour ชนะ ${wins} จาก ${ok.length} รอบ`;
    return { pairs: ok.length, rounds, d, t, diff, wins, head };
  }

  renderSeriesTable() {
    const st = this.seriesStats();
    const cell = (run) => (!run ? '–' : run.valid ? `${(run.elapsedMs / 1000).toFixed(2)} s` : 'ไม่ครบ');
    const signed = (v) => `${v > 0 ? '+' : ''}${v.toFixed(2)} s`;   // Detour − Direct: minus = Detour faster
    const rows = st.rounds.map((p, i) => {
      const both = p.direct && p.detour && p.direct.valid && p.detour.valid;
      const diff = both ? (p.direct.elapsedMs - p.detour.elapsedMs) / 1000 : NaN;
      return el('tr', {},
        el('td', { text: `รอบ ${i + 1}` }),
        el('td', { text: cell(p.direct) }),
        el('td', { text: cell(p.detour) }),
        el('td', { text: Number.isFinite(diff) ? signed(-diff) : '–' }));
    });
    if (st.pairs) {
      rows.push(el('tr', { class: 'mean' },
        el('td', { text: `เฉลี่ย ${st.pairs} รอบ` }),
        el('td', { text: `${st.d.toFixed(2)} s` }),
        el('td', { text: `${st.t.toFixed(2)} s` }),
        el('td', { text: signed(-st.diff) })));
    }
    const head = el('tr', {},
      el('th', { text: '' }),
      el('th', {}, el('i', { class: 'sw direct', 'aria-hidden': 'true' }), 'Direct'),
      el('th', {}, el('i', { class: 'sw detour', 'aria-hidden': 'true' }), 'Detour'),
      el('th', { text: 'Detour − Direct' }));
    const se = this.series;
    const left = se.active ? `ยังวิ่งอยู่ ${se.count}/${se.total} ครั้ง` : se.why ? `หยุดก่อนครบ (${se.count}/${se.total} ครั้ง)` : '';
    this.result.replaceChildren(...[
      el('table', { class: 'series-table' }, el('thead', {}, head), el('tbody', {}, ...rows)),
      el('p', { class: 'demo-sum', text: st.pairs ? st.head : 'รอให้ครบอย่างน้อย 1 รอบ (Direct + Detour)' }),
      left && el('p', { class: 'hint', text: left }),
      el('p', { class: 'hint', text: 'ค่าลบ = Detour ใช้เวลาน้อยกว่า · เวลาจับบนหุ่น ตั้งแต่ล้อหันเสร็จจนถึงเป้า' }),
    ].filter(Boolean));
  }

  renderText() {
    if (this.showSeries()) { this.renderSeriesTable(); return; }
    if (!this.data) {
      this.result.replaceChildren(this.error
        ? el('div', { class: 'errbox' }, `โหลดผลเดโมไม่ได้: ${this.error}`,
          el('button', { class: 'btn', type: 'button', onclick: () => this.load() }, 'ลองใหม่'))
        : el('p', { class: 'loading' }, 'กำลังโหลดผลเดโมจากหุ่น…'));
      return;
    }
    const runs = this.runs();
    if (!runs.length) {
      this.result.replaceChildren(el('div', { class: 'empty' }, el('b', { text: 'ยังไม่มีผล' }),
        'กดปุ่มบนหุ่น 3 ครั้ง (Direct) แล้ว 4 ครั้ง (Detour) ภาพสองรอบจะขึ้นที่นี่'));
      return;
    }
    const rows = runs.map((r) => {
      const s = (r.run.elapsedMs / 1000).toFixed(2);
      const time = r.run.open ? `กำลังวิ่ง ${s} s` : r.run.valid ? `${s} s` : 'หยุดก่อนถึงเป้า (ไม่นับเวลา)';
      const how = r.plan.kind === 'detour'
        ? `วิ่งตรงไป ${(r.plan.a * 100).toFixed(0)} ซม. ก่อน แล้วหมุน ${fmt.deg(r.plan.turnDeg)}`
        : `หมุนล้อ ${fmt.deg(r.plan.turnDeg)} ก่อนออกวิ่ง`;
      const miss = !r.run.open && Number.isFinite(r.missMm) ? ` · จบห่างเป้า ${r.missMm.toFixed(1)} mm` : '';
      return el('div', { class: 'demo-row' },
        el('i', { class: `sw ${r.key}`, 'aria-hidden': 'true' }),
        el('span', {}, el('b', { text: `${r.name} ` }), `(กด ${r.clicks} ครั้ง)`),
        el('b', { class: 'demo-time mono', text: time }),
        el('span', { class: 'demo-how', text: how + miss }));
    });
    const [a, b] = DEMO_RUNS.map((x) => this.data[x.key]);
    let sum = '';
    if (a && b && a.valid && b.valid) {
      const diff = (a.elapsedMs - b.elapsedMs) / 1000, pct = (100 * Math.abs(diff)) / (a.elapsedMs / 1000);
      sum = diff > 0 ? `Detour เร็วกว่า ${diff.toFixed(2)} s (${pct.toFixed(1)}%)`
        : diff < 0 ? `รอบนี้ Direct เร็วกว่า ${(-diff).toFixed(2)} s` : 'เวลาเท่ากัน';
    } else if (!a || !b) {
      sum = `กดปุ่ม ${a ? 4 : 3} ครั้ง เพื่อวิ่งอีกแบบมาเทียบ`;
    } else {
      sum = 'มีรอบที่ไม่ครบ จึงยังเทียบเวลาไม่ได้';
    }
    this.result.replaceChildren(...rows, el('p', { class: 'demo-sum', text: sum }));
  }

  renderHint() {
    const stale = this.data && this.conn !== 'live' && this.conn !== 'loading';
    this.hint.textContent = stale
      ? 'ไม่ได้ต่อกับหุ่นตอนนี้: แสดงผลล่าสุดที่โหลดไว้'
      : 'เส้นประ = ทางตามแผน · เส้นทึบ = ทางที่หุ่นวัดได้เอง (odometry) · วาดจากจุดเริ่มของแต่ละรอบ ให้ซ้อนกันได้';
    this.hint.classList.toggle('warn-text', !!stale);
  }

  // ---- the picture -----------------------------------------------------------

  draw() {
    const runs = this.runs();
    this.plot.classList.toggle('hidden', !runs.length);
    if (!runs.length) return;
    const w = this.plot.clientWidth;
    if (!w) return;                                // tab hidden: drawn when shown
    // fit every point with room for the labels; one scale for x and y (no distortion)
    const pts = runs.flatMap((r) => [...r.path, ...r.plan.pts, r.goal]).concat([{ x: 0, y: 0 }]);
    let [x0, x1, y0, y1] = [Infinity, -Infinity, Infinity, -Infinity];
    for (const p of pts) { x0 = Math.min(x0, p.x); x1 = Math.max(x1, p.x); y0 = Math.min(y0, p.y); y1 = Math.max(y1, p.y); }
    const padX = 48, padY = 44, spanX = Math.max(0.1, x1 - x0), spanY = Math.max(0.1, y1 - y0);
    const h = Math.round(Math.min(360, Math.max(170, (spanY * (w - 2 * padX)) / spanX + 2 * padY)));   // no empty band
    const dpr = window.devicePixelRatio || 1;
    const cv = this.canvas, ctx = cv.getContext('2d');
    cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); cv.style.height = `${h}px`;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const css = getComputedStyle(document.documentElement);
    const C = Object.fromEntries(['--s-direct', '--s-detour', '--txt', '--mut', '--line', '--card', '--f-ui']
      .map((n) => [n.slice(2), css.getPropertyValue(n).trim()]));

    const scale = Math.min((w - 2 * padX) / spanX, (h - 2 * padY) / spanY);
    const mx = (x0 + x1) / 2, my = (y0 + y1) / 2;
    const S = (p) => [w / 2 + (p.x - mx) * scale, h / 2 - (p.y - my) * scale];

    // 10 cm grid, recessive
    ctx.strokeStyle = C.line; ctx.lineWidth = 1;
    const g = 0.1;
    for (let gx = Math.ceil((mx - w / 2 / scale) / g) * g; gx <= mx + w / 2 / scale; gx += g) {
      const [sx] = S({ x: gx, y: 0 }); ctx.beginPath(); ctx.moveTo(sx, 0); ctx.lineTo(sx, h); ctx.stroke();
    }
    for (let gy = Math.ceil((my - h / 2 / scale) / g) * g; gy <= my + h / 2 / scale; gy += g) {
      const [, sy] = S({ x: 0, y: gy }); ctx.beginPath(); ctx.moveTo(0, sy); ctx.lineTo(w, sy); ctx.stroke();
    }
    ctx.font = `12px ${C['f-ui']}`;
    ctx.fillStyle = C.mut; ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
    ctx.fillText('ตาราง 10 ซม.', 8, h - 6);

    const line = (list, color, dash, width) => {
      ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash); ctx.lineJoin = 'round'; ctx.lineCap = 'round';
      ctx.beginPath();
      list.forEach((p, i) => { const [x, y] = S(p); if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
      ctx.stroke(); ctx.setLineDash([]);
    };
    const firstOf = DEMO_RUNS.map((m) => runs.find((r) => r.key === m.key)).filter(Boolean);
    for (const r of firstOf) line(r.plan.pts, C[`s-${r.key}`], [6, 5], 2);   // plans under the real paths
    for (const r of runs) line(r.path, C[`s-${r.key}`], [], 2.5);

    // the turn in place: an arc arrow at the spot, in the run's colour
    const turn = (p, color) => {
      const [x, y] = S(p), rad = 13;
      ctx.strokeStyle = color; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(x, y, rad, 0.35 * Math.PI, 2.15 * Math.PI, false); ctx.stroke();
      const ang = 2.15 * Math.PI, ex = x + rad * Math.cos(ang), ey = y + rad * Math.sin(ang);
      ctx.fillStyle = color; ctx.beginPath();
      ctx.moveTo(ex + 5, ey - 1); ctx.lineTo(ex - 3, ey - 5); ctx.lineTo(ex - 2, ey + 5); ctx.closePath(); ctx.fill();
    };
    const label = (text, x, y, align, base) => {
      ctx.textAlign = align; ctx.textBaseline = base; ctx.font = `600 12px ${C['f-ui']}`;
      ctx.lineWidth = 4; ctx.strokeStyle = C.card; ctx.strokeText(text, x, y);   // halo over the grid
      ctx.fillStyle = C.txt; ctx.fillText(text, x, y);
    };
    for (const r of firstOf) turn(r.plan.turnAt, C[`s-${r.key}`]);

    // start: arrow along the start heading (+x)
    const [sx, sy] = S({ x: 0, y: 0 });
    ctx.fillStyle = C.txt;
    ctx.beginPath(); ctx.moveTo(sx + 10, sy); ctx.lineTo(sx - 6, sy - 7); ctx.lineTo(sx - 6, sy + 7); ctx.closePath(); ctx.fill();
    label('เริ่ม', sx - 10, sy - 12, 'right', 'bottom');

    // goal: ring
    const goal = runs[0].goal, [gx, gy] = S(goal);
    ctx.strokeStyle = C.txt; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(gx, gy, 7, 0, 2 * Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.arc(gx, gy, 2, 0, 2 * Math.PI); ctx.fillStyle = C.txt; ctx.fill();
    label('เป้า', gx, gy + 12, 'center', 'top');

    // a run being timed now: where the robot is
    for (const r of runs.filter((x) => x.run.open && x.path.length)) {
      const [rx, ry] = S(r.path[r.path.length - 1]);
      ctx.fillStyle = C[`s-${r.key}`]; ctx.beginPath(); ctx.arc(rx, ry, 6, 0, 2 * Math.PI); ctx.fill();
      ctx.strokeStyle = C.card; ctx.lineWidth = 2; ctx.stroke();
    }

    // turn labels: Direct under the start, Detour above its turn point
    for (const r of firstOf) {
      const [tx, ty] = S(r.plan.turnAt);
      const text = `${r.name}: หมุน ${fmt.deg(r.plan.turnDeg)}${r.plan.kind === 'detour' ? ' ตรงนี้' : ' ก่อนออก'}`;
      if (r.key === 'direct') label(text, Math.max(8, tx - 16), ty + 20, 'left', 'top');
      else label(text, Math.min(w - 8, tx + 16), ty - 20, 'right', 'bottom');
    }
    this.canvas.setAttribute('aria-label', this.result.textContent || 'ภาพเดโม Direct เทียบ Detour');
  }
}

if (typeof module !== 'undefined') module.exports = { DemoCompareView, DEMO_RUNS };

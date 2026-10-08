// The 2D plane: the route is drawn and edited here, in metres.
//
// World frame = the robot's odometry: (0,0) where the pose was last reset,
// +x the way the robot faced then, +y to its left, angles counter-clockwise
// (same as atan2 in nav/WaypointRunner.cpp). The screen's y is flipped.
//
//   tap empty      add a point (mode "add")        drag a point   move it
//   tap a point    delete it (mode "delete")       drag empty     pan
//   wheel / pinch  zoom
//
// It edits a RouteModel and draws the robot it is given; it never talks to
// the robot itself.

class Plane2D {
  constructor(canvas, route, { onCursor, onRefuse, onFollowOff }) {
    Object.assign(this, { c: canvas, route, onCursor, onRefuse, onFollowOff });
    this.ctx = canvas.getContext('2d');
    this.mode = 'add';
    this.snap = true;                // snap new points to a 0.1 m grid
    this.follow = false;             // keep the robot centred
    this.locked = '';                // why editing is off now ('' = editable)
    this.view = { x: 0, y: 0, ppm: 80 };   // world point at the centre, pixels per metre
    this.robot = null;
    this.trail = [];
    this.activeIdx = -1;
    this.loop = false;
    this.stale = false;
    this.pointers = new Map();
    this.gesture = null;
    this.w = 0;
    this.h = 0;
    this.readColors();
    route.onChange(() => this.redraw());
    new ResizeObserver(() => this.resize()).observe(canvas);
    this.bindInput();
    matchMedia('(prefers-color-scheme: light)').addEventListener('change', () => { this.readColors(); this.redraw(); });
  }

  // ---- public ------------------------------------------------------------

  setRobot(robot, nav, stale) {
    this.robot = robot;
    this.stale = stale;
    this.activeIdx = nav.status === 'running' ? nav.index : -1;
    this.loop = !!nav.loop;
    const last = this.trail[this.trail.length - 1];
    if (!last || Math.hypot(robot.x - last[0], robot.y - last[1]) > 0.02) {
      this.trail.push([robot.x, robot.y]);
      if (this.trail.length > 800) this.trail.shift();
    }
    if (this.follow) { this.view.x = robot.x; this.view.y = robot.y; }
    this.redraw();
  }

  clearTrail() { this.trail = []; this.redraw(); }

  fit() {
    const xs = [0], ys = [0];
    for (const p of this.route.points) { xs.push(p.x); ys.push(p.y); }
    if (this.robot) { xs.push(this.robot.x); ys.push(this.robot.y); }
    const minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys);
    this.view.x = (minX + maxX) / 2;
    this.view.y = (minY + maxY) / 2;
    if (this.w) {
      const sx = Math.max(maxX - minX, 2) * 1.3, sy = Math.max(maxY - minY, 2) * 1.3;
      this.view.ppm = Plane2D.clampPpm(Math.min(this.w / sx, this.h / sy));
    }
    this.redraw();
  }

  readColors() {
    const cs = getComputedStyle(document.documentElement);
    const v = (n) => cs.getPropertyValue(n).trim();
    this.C = {
      sunk: v('--sunk'), line: v('--line'), mut: v('--mut'), txt: v('--txt'), bg: v('--bg'),
      acc: v('--acc'), onAcc: v('--on-acc'), ok: v('--ok'), warn: v('--warn'), font: v('--f-ui'),
    };
  }

  // ---- geometry ----------------------------------------------------------

  static clampPpm(p) { return Math.min(1000, Math.max(3, p)); }
  toScreen(x, y) { return [this.w / 2 + (x - this.view.x) * this.view.ppm, this.h / 2 - (y - this.view.y) * this.view.ppm]; }
  toWorld(px, py) { return [this.view.x + (px - this.w / 2) / this.view.ppm, this.view.y - (py - this.h / 2) / this.view.ppm]; }

  snapXY(x, y) {
    const q = this.snap ? 10 : 100;                       // 0.1 m or 1 cm
    const lim = (v) => Math.max(-ROUTE_LIMIT_M, Math.min(ROUTE_LIMIT_M, Math.round(v * q) / q));
    return [lim(x), lim(y)];
  }

  hit(px, py, pointerType) {
    const r = pointerType === 'touch' ? 22 : 12;
    const pts = this.route.points;
    for (let i = pts.length - 1; i >= 0; --i) {
      const [sx, sy] = this.toScreen(pts[i].x, pts[i].y);
      if (Math.hypot(sx - px, sy - py) <= r) return i;
    }
    return -1;
  }

  zoomAt(px, py, factor) {
    const [wx, wy] = this.toWorld(px, py);
    this.view.ppm = Plane2D.clampPpm(this.view.ppm * factor);
    this.view.x = wx - (px - this.w / 2) / this.view.ppm;          // keep that point under the finger
    this.view.y = wy + (py - this.h / 2) / this.view.ppm;
    this.redraw();
  }

  resize() {
    const r = this.c.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
    if (!r.width) return;                                           // hidden tab
    this.w = r.width;
    this.h = r.height;
    this.c.width = Math.round(r.width * dpr);
    this.c.height = Math.round(r.height * dpr);
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    if (!this.fitted) { this.fitted = true; this.fit(); }
    this.redraw();
  }

  // ---- input -------------------------------------------------------------

  bindInput() {
    const c = this.c;
    c.addEventListener('pointerdown', (e) => this.down(e));
    c.addEventListener('pointermove', (e) => this.moveEv(e));
    c.addEventListener('pointerup', (e) => this.up(e));
    c.addEventListener('pointercancel', (e) => { this.pointers.delete(e.pointerId); this.gesture = null; });
    c.addEventListener('pointerleave', (e) => { if (e.pointerType === 'mouse') this.onCursor(null); });
    c.addEventListener('wheel', (e) => {
      e.preventDefault();
      const [px, py] = this.pos(e);
      this.zoomAt(px, py, Math.exp(-e.deltaY * 0.0015));
    }, { passive: false });
  }

  pos(e) { const r = this.c.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; }

  down(e) {
    this.c.setPointerCapture(e.pointerId);
    const p = this.pos(e);
    this.pointers.set(e.pointerId, p);
    if (this.pointers.size === 2) {
      const [a, b] = [...this.pointers.values()];
      this.gesture = { kind: 'pinch', dist: Math.hypot(a[0] - b[0], a[1] - b[1]) || 1, ppm: this.view.ppm };
      return;
    }
    const idx = this.hit(p[0], p[1], e.pointerType);
    const dragPoint = idx >= 0 && this.mode === 'add' && !this.locked;
    this.gesture = { kind: dragPoint ? 'point' : 'pan', idx, start: p, view: { ...this.view }, moved: false };
  }

  moveEv(e) {
    const [px, py] = this.pos(e);
    this.onCursor(this.snapXY(...this.toWorld(px, py)));
    if (!this.pointers.has(e.pointerId)) return;
    this.pointers.set(e.pointerId, [px, py]);
    const g = this.gesture;
    if (!g) return;
    if (g.kind === 'pinch') {
      if (this.pointers.size < 2) return;
      const [a, b] = [...this.pointers.values()];
      const target = Plane2D.clampPpm(g.ppm * Math.hypot(a[0] - b[0], a[1] - b[1]) / g.dist);
      this.zoomAt((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, target / this.view.ppm);
      return;
    }
    if (!g.moved && Math.hypot(px - g.start[0], py - g.start[1]) < 6) return;   // still a tap
    g.moved = true;
    if (g.kind === 'point') {
      this.route.move(g.idx, ...this.snapXY(...this.toWorld(px, py)));
    } else {
      if (this.follow) { this.follow = false; this.onFollowOff(); }
      this.view.x = g.view.x - (px - g.start[0]) / this.view.ppm;
      this.view.y = g.view.y + (py - g.start[1]) / this.view.ppm;
      this.redraw();
    }
  }

  up(e) {
    this.pointers.delete(e.pointerId);
    const g = this.gesture;
    if (g && g.kind === 'pinch') { if (!this.pointers.size) this.gesture = null; return; }
    this.gesture = null;
    if (!g || g.moved) return;
    // a tap
    if (this.mode === 'delete') {
      if (g.idx < 0) return;
      if (this.locked) { this.onRefuse(this.locked); return; }
      this.route.remove(g.idx);
      return;
    }
    if (g.idx >= 0) return;                         // tapped a point: nothing to add
    if (this.locked) { this.onRefuse(this.locked); return; }
    const err = this.route.add(...this.snapXY(...this.toWorld(...g.start)));
    if (err) this.onRefuse(err);
  }

  // ---- drawing -----------------------------------------------------------

  redraw() {
    if (this.raf) return;
    this.raf = requestAnimationFrame(() => { this.raf = 0; this.draw(); });
  }

  draw() {
    if (!this.w) return;
    const { ctx, C } = this;
    ctx.fillStyle = C.sunk;
    ctx.fillRect(0, 0, this.w, this.h);
    ctx.font = `12px ${C.font}`;
    this.drawGrid();
    this.drawTrail();
    this.drawRoute();
    this.drawLeg();
    this.drawRobot();
  }

  gridStep() {
    for (const s of [0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20]) if (s * this.view.ppm >= 32) return s;
    return 50;
  }

  drawGrid() {
    const { ctx, C, w, h } = this;
    const step = this.gridStep();
    const major = step < 1 ? 1 : step * 5;
    const isMajor = (v) => Math.abs(v / major - Math.round(v / major)) < 1e-6;
    const label = (v) => (step < 1 && major === 1 ? v.toFixed(0) : String(+v.toFixed(2)));
    const [x0, y1] = this.toWorld(0, 0), [x1, y0] = this.toWorld(w, h);
    ctx.lineWidth = 1;
    const line = (ax, ay, bx, by, strong) => {
      ctx.strokeStyle = strong ? C.mut : C.line;
      ctx.globalAlpha = strong ? 0.35 : 1;
      ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
      ctx.globalAlpha = 1;
    };
    for (let i = Math.ceil(x0 / step); i * step <= x1; ++i) {
      const x = i * step, sx = Math.round(this.toScreen(x, 0)[0]) + 0.5;
      line(sx, 0, sx, h, isMajor(x));
    }
    for (let i = Math.ceil(y0 / step); i * step <= y1; ++i) {
      const y = i * step, sy = Math.round(this.toScreen(0, y)[1]) + 0.5;
      line(0, sy, w, sy, isMajor(y));
    }
    // axes through (0,0)
    const [ox, oy] = this.toScreen(0, 0);
    ctx.strokeStyle = C.mut;
    ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(0, oy); ctx.lineTo(w, oy); ctx.moveTo(ox, 0); ctx.lineTo(ox, h); ctx.stroke();
    // metre labels on the major lines, along the bottom and left edges
    ctx.fillStyle = C.mut;
    ctx.textBaseline = 'bottom';
    ctx.textAlign = 'center';
    for (let i = Math.ceil(x0 / major); i * major <= x1; ++i) {
      const sx = this.toScreen(i * major, 0)[0];
      if (sx > 24 && sx < w - 24) ctx.fillText(label(i * major), sx, h - 4);
    }
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';
    for (let i = Math.ceil(y0 / major); i * major <= y1; ++i) {
      const sy = this.toScreen(0, i * major)[1];
      if (sy > 12 && sy < h - 24) ctx.fillText(label(i * major), 4, sy);
    }
    ctx.textAlign = 'right';
    ctx.textBaseline = 'top';
    ctx.fillText('x (m) →', w - 6, Math.min(Math.max(oy + 4, 4), h - 36));
    ctx.textAlign = 'left';
    ctx.fillText('↑ y (m)', Math.min(Math.max(ox + 6, 30), w - 60), 4);
  }

  drawTrail() {
    if (this.trail.length < 2) return;
    const { ctx, C } = this;
    ctx.strokeStyle = C.txt;
    ctx.globalAlpha = 0.3;
    ctx.lineWidth = 2;
    ctx.beginPath();
    this.trail.forEach(([x, y], i) => { const [sx, sy] = this.toScreen(x, y); i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy); });
    ctx.stroke();
    ctx.globalAlpha = 1;
  }

  drawRoute() {
    const pts = this.route.points;
    if (!pts.length) return;
    const { ctx, C } = this;
    const sp = pts.map((p) => this.toScreen(p.x, p.y));
    ctx.strokeStyle = C.acc;
    ctx.lineWidth = 2;
    ctx.beginPath();
    sp.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
    ctx.stroke();
    if (this.loop && sp.length > 2) {             // back to the first point
      ctx.setLineDash([6, 6]);
      ctx.beginPath(); ctx.moveTo(...sp[sp.length - 1]); ctx.lineTo(...sp[0]); ctx.stroke();
      ctx.setLineDash([]);
    }
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.font = `600 12px ${C.font}`;
    sp.forEach(([x, y], i) => {
      ctx.globalAlpha = this.activeIdx > i ? 0.45 : 1;     // already reached
      if (i === this.activeIdx) {
        ctx.strokeStyle = C.ok; ctx.lineWidth = 3;
        ctx.beginPath(); ctx.arc(x, y, 15, 0, 2 * Math.PI); ctx.stroke();
      }
      ctx.fillStyle = C.acc;
      ctx.beginPath(); ctx.arc(x, y, 10, 0, 2 * Math.PI); ctx.fill();
      ctx.fillStyle = C.onAcc;
      ctx.fillText(String(i + 1), x, y + 0.5);
    });
    ctx.globalAlpha = 1;
    ctx.font = `12px ${C.font}`;
  }

  // the leg being driven now: from the robot to the end of its current move
  drawLeg() {
    const r = this.robot;
    if (!r || !r.goalActive) return;
    const { ctx, C } = this;
    const [ax, ay] = this.toScreen(r.x, r.y), [bx, by] = this.toScreen(r.goalX, r.goalY);
    ctx.strokeStyle = C.warn;
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 4]);
    ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
    ctx.setLineDash([]);
    ctx.beginPath(); ctx.moveTo(bx - 5, by - 5); ctx.lineTo(bx + 5, by + 5); ctx.moveTo(bx + 5, by - 5); ctx.lineTo(bx - 5, by + 5); ctx.stroke();
  }

  drawRobot() {
    const r = this.robot;
    if (!r) return;
    const { ctx, C } = this;
    const [sx, sy] = this.toScreen(r.x, r.y);
    // wheel direction (world frame), the thing the planner steers
    const wa = (-r.wheelHeadingDeg * Math.PI) / 180;
    ctx.strokeStyle = C.ok;
    ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(sx + 26 * Math.cos(wa), sy + 26 * Math.sin(wa)); ctx.stroke();
    // body: a triangle pointing along theta
    ctx.save();
    ctx.translate(sx, sy);
    ctx.rotate((-r.thetaDeg * Math.PI) / 180);
    ctx.beginPath(); ctx.moveTo(15, 0); ctx.lineTo(-10, 9); ctx.lineTo(-10, -9); ctx.closePath();
    ctx.lineWidth = 2;
    ctx.strokeStyle = this.stale ? C.warn : C.bg;
    ctx.fillStyle = C.txt;
    if (!this.stale) ctx.fill();
    ctx.stroke();
    ctx.restore();
    if (this.stale) {
      ctx.fillStyle = C.warn;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      const text = 'ตำแหน่งล่าสุด (ไม่สด)', half = ctx.measureText(text).width / 2 + 4;
      ctx.fillText(text, Math.min(Math.max(sx, half), this.w - half), Math.min(sy + 16, this.h - 20));
    }
  }
}

// The route being edited on this page, and the copy saved in the robot.
// Plane2D (the canvas) and WaypointTable (the list) both edit THIS object and
// redraw when it changes; only save() sends it to the robot.
//
// The robot keeps several routes (slots): 0 = the web route (start button,
// route test), 1 and 2 = what the demo button drives on 1 / 2 clicks. select()
// switches which one is edited; each slot keeps its own unsaved edits.

const ROUTE_LIMIT_M = 50;            // nav/WaypointRunner.cpp refuses points further out
const ROUTE_MAX_WAIT_S = 60;         // firmware NAV_MAX_WAIT_S: longest stop at one point
const ROUTE_SLOTS = [0, 1, 2];       // firmware DEMO_ROUTES = 2

class RouteModel {
  constructor(api, max = 32) {
    Object.assign(this, { api, max });
    this.slot = 0;
    this.slots = new Map();          // slot -> {points, saved, loaded, loadError}
    this.listeners = [];
  }

  st(slot = this.slot) {
    if (!this.slots.has(slot)) this.slots.set(slot, { points: [], saved: [], loaded: false, loadError: '' });
    return this.slots.get(slot);
  }
  get points() { return this.st().points; }          // [{x, y}] metres, being edited
  set points(v) { this.st().points = v; }
  get saved() { return this.st().saved; }            // what the robot has
  set saved(v) { this.st().saved = v; }
  get loaded() { return this.st().loaded; }
  set loaded(v) { this.st().loaded = v; }
  get loadError() { return this.st().loadError; }
  set loadError(v) { this.st().loadError = v; }

  // edit another slot; loads it from the robot the first time
  async select(slot) {
    if (slot === this.slot) return { ok: true };
    this.slot = slot;
    this.emit();
    return this.loaded ? { ok: true } : this.load();
  }
  isDirty(slot) {
    const st = this.slots.get(slot);
    return !!st && st.loaded && JSON.stringify(st.points) !== JSON.stringify(st.saved);
  }

  onChange(fn) { this.listeners.push(fn); }
  emit() { this.listeners.forEach((fn) => fn(this)); }

  // {x, y} metres to 1 mm; waitS (stop there, seconds, 0.1 s steps) only when > 0
  static norm(p) {
    const out = { x: Math.round(p.x * 1000) / 1000, y: Math.round(p.y * 1000) / 1000 };
    const w = Math.round((p.waitS || 0) * 10) / 10;
    if (w > 0) out.waitS = w;
    return out;
  }
  get dirty() { return JSON.stringify(this.points) !== JSON.stringify(this.saved); }

  add(x, y) {
    if (this.points.length >= this.max) return `วางได้ไม่เกิน ${this.max} จุด`;
    this.points.push(RouteModel.norm({ x, y }));
    this.emit();
    return '';
  }
  move(i, x, y) { this.points[i] = RouteModel.norm({ ...this.points[i], x, y }); this.emit(); }
  setWait(i, s) { this.points[i] = RouteModel.norm({ ...this.points[i], waitS: s }); this.emit(); }
  remove(i) { this.points.splice(i, 1); this.emit(); }
  moveUp(i) {
    if (i <= 0) return;
    [this.points[i - 1], this.points[i]] = [this.points[i], this.points[i - 1]];
    this.emit();
  }
  clear() { this.points = []; this.emit(); }
  revert() { this.points = this.saved.map((p) => ({ ...p })); this.emit(); }

  async load() {
    const slot = this.slot, st = this.st(slot);     // the reply belongs to this slot even if the user switches
    const r = await this.api.get(slot ? `/api/waypoints?slot=${slot}` : '/api/waypoints');
    if (r.ok) {
      st.saved = (r.data.points || []).map(RouteModel.norm);
      st.points = st.saved.map((p) => ({ ...p }));
      st.loaded = true;
      st.loadError = '';
    } else {
      st.loadError = r.error;
    }
    this.emit();
    return r;
  }

  async save() {
    // An edit may happen while HTTP is pending. Only the exact submitted
    // snapshot becomes saved; newer edits must remain visibly dirty.
    const slot = this.slot, st = this.st(slot);
    const submitted = st.points.map((p) => ({ ...p }));
    const r = await this.api.post('/api/waypoints', slot ? { slot, points: submitted } : { points: submitted });
    if (r.ok) { st.saved = submitted; this.emit(); }
    return r;
  }
}

if (typeof module !== 'undefined') module.exports = { RouteModel, ROUTE_SLOTS, ROUTE_MAX_WAIT_S };

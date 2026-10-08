// The route being edited on this page, and the copy saved in the robot.
// Plane2D (the canvas) and WaypointTable (the list) both edit THIS object and
// redraw when it changes; only save() sends it to the robot.

const ROUTE_LIMIT_M = 50;            // nav/WaypointRunner.cpp refuses points further out

class RouteModel {
  constructor(api, max = 32) {
    Object.assign(this, { api, max });
    this.points = [];                // [{x, y}] metres, being edited
    this.saved = [];                 // what the robot has
    this.loaded = false;
    this.loadError = '';
    this.listeners = [];
  }

  onChange(fn) { this.listeners.push(fn); }
  emit() { this.listeners.forEach((fn) => fn(this)); }

  static norm(p) { return { x: Math.round(p.x * 1000) / 1000, y: Math.round(p.y * 1000) / 1000 }; }
  get dirty() { return JSON.stringify(this.points) !== JSON.stringify(this.saved); }

  add(x, y) {
    if (this.points.length >= this.max) return `วางได้ไม่เกิน ${this.max} จุด`;
    this.points.push(RouteModel.norm({ x, y }));
    this.emit();
    return '';
  }
  move(i, x, y) { this.points[i] = RouteModel.norm({ x, y }); this.emit(); }
  remove(i) { this.points.splice(i, 1); this.emit(); }
  moveUp(i) {
    if (i <= 0) return;
    [this.points[i - 1], this.points[i]] = [this.points[i], this.points[i - 1]];
    this.emit();
  }
  clear() { this.points = []; this.emit(); }
  revert() { this.points = this.saved.map((p) => ({ ...p })); this.emit(); }

  async load() {
    const r = await this.api.get('/api/waypoints');
    if (r.ok) {
      this.saved = (r.data.points || []).map(RouteModel.norm);
      this.points = this.saved.map((p) => ({ ...p }));
      this.loaded = true;
      this.loadError = '';
    } else {
      this.loadError = r.error;
    }
    this.emit();
    return r;
  }

  async save() {
    // An edit may happen while HTTP is pending. Only the exact submitted
    // snapshot becomes saved; newer edits must remain visibly dirty.
    const submitted = this.points.map((p) => ({ ...p }));
    const r = await this.api.post('/api/waypoints', { points: submitted });
    if (r.ok) { this.saved = submitted; this.emit(); }
    return r;
  }
}

if (typeof module !== 'undefined') module.exports = { RouteModel };

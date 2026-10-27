// Browser-only waypoints. There is deliberately no API/client reference here:
// editing or playing the simulator can never save or start a robot route.
class SimulationWaypoints {
  constructor(points = []) { this.points = []; this.listeners = []; this.max = 32; this.replace(points); }
  onChange(fn) { this.listeners.push(fn); }
  emit() { this.listeners.forEach(fn => fn(this)); }
  static norm(p) {
    return { x: Math.round(p.x * 1000) / 1000, y: Math.round(p.y * 1000) / 1000,
      waitS: Math.round((p.waitS || 0) * 10) / 10 };
  }
  replace(points) {
    if (!Array.isArray(points) || points.length > this.max || points.some(p =>
      !Number.isFinite(p.x) || !Number.isFinite(p.y) || Math.abs(p.x) > 50 || Math.abs(p.y) > 50)) return 'จุดจำลองต้องอยู่ในระยะ ±50 m ไม่เกิน 32 จุด';
    this.points = points.map(SimulationWaypoints.norm); this.emit(); return '';
  }
  add(x, y) { return this.replace([...this.points, { x, y }]); }
  move(i, x, y) { return this.replace(this.points.map((p, j) => i === j ? { ...p, x, y } : p)); }
  remove(i) { return this.replace(this.points.filter((_, j) => j !== i)); }
  clear() { return this.replace([]); }
  static comparison(distance, segments, rightDeg = 6) {
    let x = 0, y = 0;
    return Array.from({ length: segments }, (_, i) => {
      const a = -(i + 1) * rightDeg * Math.PI / 180;
      x += distance / segments * Math.cos(a); y += distance / segments * Math.sin(a);
      return { x, y };
    });
  }
}
if (typeof module !== 'undefined') module.exports = { SimulationWaypoints };

// HW04 Detour Steer geometry. World headings are CCW; physical steering is
// counter-clockwise too (adds angles, measured 2026-10-08). NO HTTP or motor calls.
class SimulationPlanner {
  static wrap(deg) { return ((deg % 360) + 360) % 360; }
  static leg(phi, d, p, detour = true) {
    phi = this.wrap(phi);
    if (Math.min(phi, 360 - phi) <= 3.5) phi = 0;
    const direct = { a: 0, beta: phi, b: d, kind: 'direct',
      time: d / p.v + (phi ? phi / p.w + p.settle : 0) };
    if (!detour || phi <= 180) return direct;
    const r = Math.PI / 180, k = d * p.w * r / p.v * Math.abs(Math.sin(phi * r));
    if (k >= 2) return direct;
    const beta = 360 - Math.acos(k - 1) / r;
    if (beta >= phi) return direct;
    const a = d * Math.sin((beta - phi) * r) / Math.sin(beta * r);
    const b = d * Math.sin(phi * r) / Math.sin(beta * r);
    const time = (a + b) / p.v + p.stop + beta / p.w + p.settle;
    return a >= 0 && b >= 0 && time < direct.time ? { a, beta, b, time, kind: 'detour' } : direct;
  }
  static route(points, initial, p, detour) {
    let pose = { ...initial }, time = 0, length = 0, detours = 0;
    const phases = [], path = [{ ...pose }];
    const phase = (duration, end, mode) => {
      if (duration <= 0) return;
      phases.push({ start: { ...pose }, end: { ...end }, t0: time, duration, mode });
      time += duration; pose = { ...end }; path.push({ ...pose });
    };
    const drive = d => {
      const r = pose.h * Math.PI / 180;
      phase(d / p.v, { x: pose.x + d * Math.cos(r), y: pose.y + d * Math.sin(r), h: pose.h }, 'drive');
      length += d;
    };
    for (const goal of points) {
      const d = Math.hypot(goal.x - pose.x, goal.y - pose.y);
      const wait = () => { if (goal.waitS > 0) phase(goal.waitS, pose, 'wait'); };   // stop at the point
      if (d <= 0.02) { wait(); continue; }
      const bearing = Math.atan2(goal.y - pose.y, goal.x - pose.x) * 180 / Math.PI;
      const plan = this.leg(bearing - pose.h, d, p, detour);
      if (plan.kind === 'detour') { drive(plan.a); phase(p.stop, pose, 'stop'); ++detours; }
      if (plan.beta) { phase(plan.beta / p.w, { ...pose, h: pose.h + plan.beta }, 'steer'); phase(p.settle, pose, 'settle'); }
      drive(plan.b);
      // Same tolerance as firmware: an already-aimed wheel need not point
      // exactly at the goal. Replan the remaining error rather than teleport.
      if (Math.hypot(goal.x - pose.x, goal.y - pose.y) > 0.02) {
        const remaining = Math.hypot(goal.x - pose.x, goal.y - pose.y);
        const aim = Math.atan2(goal.y - pose.y, goal.x - pose.x) * 180 / Math.PI;
        const turn = this.wrap(aim - pose.h);
        phase(turn / p.w, { ...pose, h: pose.h + turn }, 'steer'); phase(p.settle, pose, 'settle'); drive(remaining);
      }
      wait();
    }
    return { phases, path, time, length, detours, final: pose };
  }
  static poseAt(run, t) {
    const f = run.phases.find(x => t < x.t0 + x.duration);
    if (!f) return { ...run.final, bodyH: run.phases[0]?.start.bodyH || 0, mode: 'done' };
    const a = Math.max(0, Math.min(1, (t - f.t0) / f.duration));
    return { x: f.start.x + (f.end.x - f.start.x) * a, y: f.start.y + (f.end.y - f.start.y) * a,
      h: f.start.h + (f.end.h - f.start.h) * a, bodyH: run.phases[0]?.start.bodyH || 0, mode: f.mode };
  }
}

// Illustrative open-loop error model, not a calibrated dynamics model. Each run
// uses the same seeded environment fields and fixed 20 ms integration steps.
// Body yaw, relative wheel angle and their two sensor errors are independent.
class SimulationNoise {
  static STEP = 0.02;
  static MAX_TIME = 1200;
  static clamp(x, a, b) { return Math.max(a, Math.min(b, x)); }
  static signed(deg) { return SimulationPlanner.wrap(deg + 180) - 180; }
  static field(seed, channel, time) {
    const hash = n => {
      let x = (seed ^ Math.imul(channel + 1, 0x9e3779b9) ^ Math.imul(n, 0x85ebca6b)) >>> 0;
      x = Math.imul(x ^ (x >>> 16), 0x7feb352d);
      x = Math.imul(x ^ (x >>> 15), 0x846ca68b);
      return ((x ^ (x >>> 16)) >>> 0) / 2147483648 - 1;
    };
    const n = Math.floor(time), f = time - n, smooth = f * f * (3 - 2 * f);
    return hash(n) * (1 - smooth) + hash(n + 1) * smooth;
  }
  static sensors(seed, level, t, bodyH, wheelAngle) {
    if (!level) return { bodyError: 0, wheelError: 0, imuEstimate: bodyH, wheelEstimate: wheelAngle };
    const f = (channel, time = 0) => this.field(seed, channel, time);
    const bodyError = level * (0.8 * f(1) + 0.008 * f(2) * t + 0.08 * f(3, t * 5));
    const wheelError = level * (0.6 * f(4) + 0.006 * f(5) * t + 0.06 * f(6, t * 7));
    return { bodyError, wheelError, imuEstimate: bodyH + bodyError, wheelEstimate: wheelAngle + wheelError };
  }
  static realize(ideal, options = {}) {
    const seed = Number(options.seed ?? 42) >>> 0;
    const level = this.clamp(Number(options.level ?? 1) || 0, 0, 2);
    if (!Number.isFinite(ideal.time) || ideal.time > this.MAX_TIME) throw new Error('จำลองแบบสุ่มได้ไม่เกิน 20 นาทีต่อรอบ');
    const initial = ideal.phases[0]?.start || ideal.final;
    let x = initial.x, y = initial.y, bodyH = initial.bodyH || 0;
    let wheelAngle = initial.h - bodyH, time = 0, length = 0, lastDriveSpeed = 0;
    const samples = [], phases = [];
    const snapshot = mode => ({ x, y, h: bodyH + wheelAngle, bodyH, wheelAngle, t: time, mode,
      ...this.sensors(seed, level, time, bodyH, wheelAngle) });
    samples.push(snapshot(ideal.phases[0]?.mode || 'done'));
    const f = (channel, t = time) => this.field(seed, channel, t);
    for (const phase of ideal.phases) {
      const start = snapshot(phase.mode), plannedTurn = phase.end.h - phase.start.h;
      // A steering command uses BOTH estimated angles. Bias in either sensor
      // changes the command even when the other sensor is correct.
      const estimatedHeading = start.imuEstimate + start.wheelEstimate;
      const commandTurn = plannedTurn + (phase.mode === 'steer' ? this.signed(phase.start.h - estimatedHeading) : 0);
      const gain = phase.mode === 'steer' ? 1 + level * (0.04 * f(10, 0) + 0.025 * f(11, time / 8))
        : 1 + level * (0.035 * f(12, 0) + 0.025 * f(13, time / 8));
      const duration = phase.mode === 'drive' ? phase.duration / gain
        : phase.mode === 'steer' ? phase.duration * Math.abs(commandTurn / plannedTurn) / gain : phase.duration;
      if (time + duration > this.MAX_TIME) throw new Error('จำลองแบบสุ่มได้ไม่เกิน 20 นาทีต่อรอบ');
      const distance = Math.hypot(phase.end.x - phase.start.x, phase.end.y - phase.start.y);
      const nominalV = distance / phase.duration;
      const steps = Math.ceil(duration / this.STEP);
      for (let i = 0; i < steps; ++i) {
        const elapsed = i * this.STEP, dt = Math.min(this.STEP, duration - elapsed), mid = time + dt / 2;
        const moving = phase.mode === 'drive' || phase.mode === 'steer';
        const bodyRate = moving ? level * (0.035 * f(20, mid / 3) + 0.08 * f(21, mid * 2)) : 0;
        let wheelRate = 0, speed = 0;
        if (phase.mode === 'steer') {
          wheelRate = commandTurn / duration * (1 + level * 0.008 * f(22, mid * 3));
          lastDriveSpeed = 0;
        } else if (phase.mode === 'drive') {
          wheelRate = level * 0.02 * f(23, mid * 2);
          const ramp = 1 - level * 0.08 * (1 - Math.min(1, (elapsed + dt / 2) / 0.12, (duration - elapsed - dt / 2) / 0.12));
          const slip = level * (0.012 + 0.008 * (f(24, mid / 4) + 1));
          speed = nominalV * gain * (1 - slip) * (1 + level * 0.018 * f(25, mid * 2)) * ramp;
          lastDriveSpeed = speed;
        } else if (phase.mode === 'stop') {
          speed = lastDriveSpeed * level * 0.25 * Math.exp(-(elapsed + dt / 2) / 0.06);
        } else lastDriveSpeed = 0;
        const heading = (bodyH + wheelAngle + (bodyRate + wheelRate) * dt / 2) * Math.PI / 180;
        x += speed * dt * Math.cos(heading); y += speed * dt * Math.sin(heading);
        bodyH += bodyRate * dt; wheelAngle += wheelRate * dt;
        length += Math.abs(speed * dt); time += dt;
        // Zero level reproduces the ideal geometry exactly, including corners.
        if (!level) {
          const a = (elapsed + dt) / duration;
          x = phase.start.x + (phase.end.x - phase.start.x) * a;
          y = phase.start.y + (phase.end.y - phase.start.y) * a;
          wheelAngle = phase.start.h + plannedTurn * a - bodyH;
        }
        samples.push(snapshot(phase.mode));
      }
      phases.push({ start, end: snapshot(phase.mode), t0: start.t, duration, mode: phase.mode });
    }
    const final = snapshot('done');
    samples[samples.length - 1] = final;
    return { phases, samples, path: samples, time: level ? time : ideal.time, length: level ? length : ideal.length,
      detours: ideal.detours, final, seed, level, dt: this.STEP, ideal };
  }
  static poseAt(run, t) {
    if (t >= run.time) return { ...run.final };
    let lo = 0, hi = run.samples.length - 1;
    while (hi - lo > 1) { const mid = (lo + hi) >>> 1; if (run.samples[mid].t <= t) lo = mid; else hi = mid; }
    const a = run.samples[lo], b = run.samples[hi], k = this.clamp((t - a.t) / (b.t - a.t || 1), 0, 1);
    const result = { ...a };
    for (const key of ['x', 'y', 'h', 'bodyH', 'wheelAngle', 'bodyError', 'wheelError', 'imuEstimate', 'wheelEstimate']) result[key] = a[key] + (b[key] - a[key]) * k;
    return result;
  }
  // Display-only vibration in pixels/degrees. It never changes position,
  // timing, sensor estimates or the stored trajectory error.
  static shake(run, t, mode, amplification) {
    if (!run.level || !['drive', 'steer'].includes(mode)) return { x: 0, y: 0, angle: 0 };
    const phase = this.field(run.seed, 30, 0) * Math.PI;
    const amount = run.level * amplification * (mode === 'drive' ? 0.65 : 0.4);
    return { x: amount * Math.sin(t * 2 * Math.PI * 8 + phase),
      y: amount * Math.sin(t * 2 * Math.PI * 11 + phase / 2),
      angle: amount * 0.25 * Math.sin(t * 2 * Math.PI * 9 + phase) };
  }
}

class RouteSimulation {
  constructor(route, robot) {
    Object.assign(this, { route, robot });
    this.canvas = $('#simCanvas'); this.ctx = this.canvas.getContext('2d');
    this.t = 0; this.playing = false; this.runs = null;
    $('#simPlay').onclick = () => this.start();
    $('#simPause').onclick = () => { this.playing = !this.playing && !!this.runs; this.lastFrame = performance.now(); this.animate(); };
    $('#simReset').onclick = () => { this.playing = false; this.t = 0; this.draw(); };
    $('#simExample').onclick = () => this.start(true);
    for (const id of ['simNoiseEnabled', 'simNoiseLevel', 'simNoiseSeed', 'simShakeScale']) {
      const input = $('#' + id);
      if (input) input.onchange = () => { this.playing = false; this.runs = null; this.draw(); $('#simSummary').textContent = 'ค่าจำลองเปลี่ยนแล้ว · กดจำลองอีกครั้ง'; };
    }
    route.onChange(() => { this.playing = false; this.runs = null; this.draw(); $('#simSummary').textContent = 'จุดเปลี่ยนแล้ว · กดจำลองอีกครั้ง'; });
    this.draw();
  }
  start(example = false) {
    const v = Number($('#simSpeed').value), w = Number($('#simSteer').value);
    if (!(v >= 0.005 && v <= 0.035 && w >= 5 && w <= 720)) {
      $('#simSummary').textContent = 'ความเร็วขับ 0.005–0.035 m/s · เลี้ยว 5–720 °/s'; return;
    }
    const rb = this.robot() || {}, initial = { x: rb.x || 0, y: rb.y || 0, h: rb.wheelHeadingDeg || 0, bodyH: rb.thetaDeg || 0 };
    const r = (initial.h + 6) * Math.PI / 180;
    const points = example ? [{ x: initial.x + 0.3 * Math.cos(r), y: initial.y + 0.3 * Math.sin(r) }] : this.route.points.map(p => ({ ...p }));
    if (!points.length) { $('#simSummary').textContent = 'วางจุดก่อน หรือใช้ตัวอย่างจำลอง'; return; }
    const p = { v, w, settle: 0.05, stop: 0.20 };
    this.ideals = [SimulationPlanner.route(points, initial, p, true), SimulationPlanner.route(points, initial, p, false)];
    this.noiseOn = !!$('#simNoiseEnabled')?.checked;
    this.noise = { level: Number($('#simNoiseLevel')?.value ?? 100) / 100, seed: Number($('#simNoiseSeed')?.value ?? 42) };
    this.shakeScale = SimulationNoise.clamp(Number($('#simShakeScale')?.value ?? 3) || 0, 0, 10);
    this.playing = false; this.runs = null;
    try { this.runs = this.noiseOn ? this.ideals.map(run => SimulationNoise.realize(run, this.noise)) : this.ideals; }
    catch (error) { this.draw(); $('#simSummary').textContent = error.message; return; }
    this.points = points; this.t = 0; this.playing = true; this.lastFrame = performance.now();
    this.animate();
  }
  animate() {
    if (!this.playing || this.framePending) return;
    this.framePending = true;
    requestAnimationFrame(now => {
      this.framePending = false;
      if (!this.playing) return;
      this.t += Math.min(0.1, (now - this.lastFrame) / 1000) * Number($('#simRate').value);
      this.lastFrame = now;
      if (this.runs && this.t >= Math.max(...this.runs.map(r => r.time))) this.playing = false;
      this.draw(); this.animate();
    });
  }
  draw() {
    const c = this.ctx, width = this.canvas.width, height = this.canvas.height;
    const sensorStatus = $('#simNoiseStatus');
    if (sensorStatus) sensorStatus.hidden = !this.runs || !this.noiseOn;
    c.clearRect(0, 0, width, height);
    if (!this.runs) { c.fillStyle = '#d4e6f5'; c.font = '18px sans-serif'; c.fillText('จำลองการเคลื่อนที่ · แบบที่ 1 / แบบที่ 2', 24, 40); return; }
    const all = this.runs.flatMap(r => r.path).concat(this.ideals.flatMap(r => r.path), this.points);
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (const p of all) { minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x); minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y); }
    const scale = Math.min((width - 120) / Math.max(0.2, maxX - minX), (height - 120) / Math.max(0.2, maxY - minY));
    const xy = p => [width / 2 + (p.x - (minX + maxX) / 2) * scale, height / 2 - (p.y - (minY + maxY) / 2) * scale];
    c.strokeStyle = '#3a5064'; c.lineWidth = 1;
    c.beginPath(); c.moveTo(40, height - 30); c.lineTo(100, height - 30); c.moveTo(40, height - 30); c.lineTo(40, height - 90); c.stroke();
    c.fillStyle = '#d4e6f5'; c.font = '15px sans-serif'; c.fillText('+x', 104, height - 25); c.fillText('+y', 30, height - 98);
    const colors = ['#66edbc', '#ffb566'];
    const strokePath = path => { c.beginPath(); const stride = Math.max(1, Math.floor(path.length / 2000));
      for (let j = 0; j < path.length; j += stride) { const [x, y] = xy(path[j]); if (j) c.lineTo(x, y); else c.moveTo(x, y); }
      const [x, y] = xy(path[path.length - 1]); c.lineTo(x, y); c.stroke(); };
    this.runs.forEach((run, i) => {
      c.strokeStyle = colors[i]; c.lineWidth = 2;
      if (this.noiseOn) { c.globalAlpha = 0.3; c.setLineDash([5, 5]); strokePath(this.ideals[i].path); c.globalAlpha = 1; }
      c.setLineDash(i ? [7, 5] : []); strokePath(run.path); c.setLineDash([]);
      const pose = this.noiseOn ? SimulationNoise.poseAt(run, this.t) : SimulationPlanner.poseAt(run, this.t);
      const [x, y] = xy(pose), shake = this.noiseOn ? SimulationNoise.shake(run, this.t, pose.mode, this.shakeScale) : { x: 0, y: 0, angle: 0 };
      c.save(); c.translate(x + shake.x, y + shake.y); c.rotate(-((pose.bodyH || 0) + shake.angle) * Math.PI / 180);
      c.fillStyle = colors[i]; c.globalAlpha = 0.45; c.fillRect(-11, -8, 22, 16); c.globalAlpha = 1; c.restore();
      c.save(); c.translate(x + shake.x, y + shake.y); c.rotate(-(pose.h + shake.angle) * Math.PI / 180);
      c.fillStyle = colors[i]; c.beginPath(); c.moveTo(17, 0); c.lineTo(-8, -5); c.lineTo(-8, 5); c.closePath(); c.fill(); c.restore();
      const goal = this.points[this.points.length - 1], residual = Math.hypot(run.final.x - goal.x, run.final.y - goal.y);
      c.fillStyle = colors[i]; c.fillText(`แบบ ${i ? 1 : 2} ${i ? 'Direct' : 'Detour'}: ${pose.mode} · ${run.time.toFixed(2)} s · ${run.length.toFixed(3)} m${this.noiseOn ? ` · คลาด ${(residual * 100).toFixed(1)} cm` : ''}`, 24, 28 + i * 24);
    });
    this.points.forEach((p, i) => { const [x, y] = xy(p); c.strokeStyle = '#e5f0fa'; c.beginPath(); c.arc(x, y, 5, 0, Math.PI * 2); c.stroke(); c.fillStyle = '#e5f0fa'; c.fillText(String(i + 1), x + 9, y - 8); });
    const [det, direct] = this.runs;
    const errors = this.noiseOn ? this.runs.map((run, i) => { const pose = SimulationNoise.poseAt(run, this.t);
      return `แบบ ${i ? 1 : 2}: IMU คลาด ${pose.bodyError.toFixed(2)}° / เซนเซอร์ล้อคลาด ${pose.wheelError.toFixed(2)}°`; }).join(' · ') : '';
    $('#simSummary').textContent = `จำลอง ${this.t.toFixed(1)} s · แบบ 1 Direct ${direct.time.toFixed(2)} s / แบบ 2 Detour ${det.time.toFixed(2)} s · ต่างกัน ${(direct.time - det.time).toFixed(2)} s · อ้อม ${det.detours} ครั้ง` +
      (this.noiseOn ? ` · seed ${det.seed} · เส้นจาง = อุดมคติ · ภาพสั่นขยาย ×${this.shakeScale} · ${errors} (ค่าจำลอง)` : ' · อุดมคติ');
    if (sensorStatus && this.noiseOn) sensorStatus.textContent = [1, 0].map(i => {
      const pose = SimulationNoise.poseAt(this.runs[i], this.t), angle = n => SimulationPlanner.wrap(n).toFixed(2);
      return `แบบ ${i ? 1 : 2} (ค่าจำลอง): มุมตัวหุ่น ${angle(pose.bodyH)}° → IMU อ่าน ${angle(pose.imuEstimate)}°; มุมล้อเทียบตัวหุ่น ${angle(pose.wheelAngle)}° → เซนเซอร์ล้ออ่าน ${angle(pose.wheelEstimate)}°`;
    }).join('\n');
  }
}

if (typeof module !== 'undefined') module.exports = { SimulationPlanner, SimulationNoise };

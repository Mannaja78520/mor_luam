// Small helpers shared by every view. Plain DOM, no framework, no CDN:
// the robot's hotspot has no internet.
'use strict';

const $ = (sel, root = document) => root.querySelector(sel);

// el('td', {class: 'x', onclick: fn}, 'text', childNode)
function el(tag, attrs = {}, ...kids) {
  const n = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (v === undefined || v === null || v === false) continue;
    if (k.startsWith('on')) n.addEventListener(k.slice(2), v);
    else if (k === 'class') n.className = v;
    else if (k === 'text') n.textContent = v;
    else n.setAttribute(k, v === true ? '' : v);
  }
  for (const k of kids) if (k !== null && k !== undefined) n.append(k);
  return n;
}

const fmt = {
  num(v, d = 2) { return Number.isFinite(v) ? v.toFixed(d) : '–'; },
  deg(v) { return Number.isFinite(v) ? `${Math.round(v)}°` : '–'; },
  age(ms) {
    if (!ms) return '';
    const s = Math.round(ms / 1000);
    if (s < 2) return 'เมื่อกี้';
    return s < 60 ? `${s} วินาทีก่อน` : `${Math.round(s / 60)} นาทีก่อน`;
  },
  uptime(s) {
    const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60);
    return h ? `${h} ชม. ${m} นาที` : `${m} นาที ${s % 60} วินาที`;
  },
  rssi(r) {
    if (!r) return '–';
    const word = r > -60 ? 'ดีมาก' : r > -70 ? 'ดี' : r > -80 ? 'พอใช้' : 'อ่อน';
    return `${r} dBm (${word})`;
  },
};

// Fill a <table class="kv"> from [[label, value], ...] without rebuilding rows
// that did not change (keeps text selection and avoids flicker at 4 Hz).
function fillKv(table, rows) {
  while (table.rows.length > rows.length) table.deleteRow(-1);
  rows.forEach(([k, v], i) => {
    const tr = table.rows[i] || table.insertRow();
    if (tr.cells.length === 0) { tr.append(el('th'), el('td')); }
    if (tr.cells[0].textContent !== k) tr.cells[0].textContent = k;
    const val = String(v ?? '–');
    if (tr.cells[1].textContent !== val) tr.cells[1].textContent = val;
  });
}

// One message at a time at the bottom of the screen.
class Toast {
  constructor(node) { this.node = node; this.timer = 0; }
  show(msg, bad = false, ms = 3500) {
    this.node.textContent = msg;
    this.node.classList.toggle('bad', bad);
    this.node.classList.remove('hidden');
    clearTimeout(this.timer);
    this.timer = setTimeout(() => this.node.classList.add('hidden'), ms);
  }
  result(r, okMsg) { r.ok ? this.show(okMsg) : this.show(r.error, true, 6000); return r.ok; }
}

// localStorage for per-viewer conveniences only; private windows can throw.
const Prefs = {
  get(k, d) { try { const v = localStorage.getItem('morluam.' + k); return v === null ? d : JSON.parse(v); } catch { return d; } },
  set(k, v) { try { localStorage.setItem('morluam.' + k, JSON.stringify(v)); } catch { /* not kept */ } },
};

// "แสดง / ซ่อน" next to every password field (data-reveal="inputId").
function wireRevealButtons(root = document) {
  root.querySelectorAll('[data-reveal]').forEach((b) => {
    b.addEventListener('click', () => {
      const inp = document.getElementById(b.dataset.reveal);
      const show = inp.type === 'password';
      inp.type = show ? 'text' : 'password';
      b.textContent = show ? 'ซ่อน' : 'แสดง';
    });
  });
}

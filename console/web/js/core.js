/* Core of the SONAIR Lab page: the agent connection, the shared state, small helpers, charts and dialogs.
   Plain ES modules, no build step, no network access beyond the console agent. */

export const $ = (s, el = document) => el.querySelector(s);
export const $$ = (s, el = document) => Array.from(el.querySelectorAll(s));
export const esc = (s) => String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
export const fx = (v, d = 3) => (v === null || v === undefined || Number.isNaN(Number(v)) || v === "" ? "-" : Number(v).toFixed(d));
export const DEG = 180 / Math.PI;
export const JOINTS = ["Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"];
export const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
export const SERIES = () => [1, 2, 3, 4, 5, 6].map((i) => css(`--s${i}`));
export const icon = (id, cls = "i") => `<svg class="${cls}"><use href="#i-${id}"/></svg>`;
export const store = {
  get(k, d) { try { const v = localStorage.getItem("sonair." + k); return v === null ? d : JSON.parse(v); } catch { return d; } },
  set(k, v) { try { localStorage.setItem("sonair." + k, JSON.stringify(v)); } catch { /* private mode */ } },
};

/* ------------------------------------------------------------------ the shared state */
export const S = { ws: null, rid: 0, pending: new Map(), hello: null, st: null, page: "overview", listeners: new Set(), helloListeners: new Set(),
  logListeners: new Set(), token: new URLSearchParams(location.search).get("token") };
export const onState = (fn) => S.listeners.add(fn);
export const onHello = (fn) => S.helloListeners.add(fn);
export const onLog = (fn) => S.logListeners.add(fn);
export const withToken = (url) => (S.token ? url + (url.includes("?") ? "&" : "?") + "token=" + encodeURIComponent(S.token) : url);

function wsUrl() {
  return `${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws` + (S.token ? `?token=${encodeURIComponent(S.token)}` : "");
}
export function connect(onOpen, onClose) {
  const ws = new WebSocket(wsUrl());
  S.ws = ws;
  ws.onopen = () => { $("#offline").classList.remove("on"); onOpen && onOpen(); };
  ws.onclose = () => { $("#offline").classList.add("on"); onClose && onClose(); setTimeout(() => connect(onOpen, onClose), 1500); };
  ws.onmessage = (ev) => {
    let m; try { m = JSON.parse(ev.data); } catch { return; }
    if (m.rid !== undefined && S.pending.has(m.rid)) { S.pending.get(m.rid)(m); S.pending.delete(m.rid); return; }
    if (m.type === "state") { S.st = m; S.listeners.forEach((f) => { try { f(m); } catch (e) { console.error(e); } }); }
    else if (m.type === "hello") { S.hello = m; S.helloListeners.forEach((f) => { try { f(m); } catch (e) { console.error(e); } }); }
    else if (m.type === "log") S.logListeners.forEach((f) => f(m));
  };
}
export function send(msg) { if (S.ws && S.ws.readyState === 1) S.ws.send(JSON.stringify(msg)); }
export function req(msg, timeout = 60000) {
  return new Promise((res) => {
    if (!S.ws || S.ws.readyState !== 1) return res({ ok: false, error: "not connected to the console agent" });
    const rid = ++S.rid; S.pending.set(rid, res);
    S.ws.send(JSON.stringify({ ...msg, rid }));
    setTimeout(() => { if (S.pending.has(rid)) { S.pending.delete(rid); res({ ok: false, error: "no answer from the agent" }); } }, timeout);
  });
}

/* ------------------------------------------------------------------ feedback */
export function toast(msg, kind = "") {
  const d = document.createElement("div"); d.className = kind; d.textContent = msg;
  $("#toast").appendChild(d); setTimeout(() => d.remove(), kind === "bad" ? 9000 : 4500);
}
export function result(r, okMsg) {
  const ok = r && r.ok !== false;
  toast(ok ? (okMsg || r.msg || "done") : (r.error || r.msg || "failed"), ok ? "good" : "bad");
  return ok;
}
export function confirmBox(title, html, yes = "Confirm") {
  return new Promise((res) => {
    $("#mTitle").textContent = title; $("#mBody").innerHTML = html; $("#mYes").textContent = yes;
    $("#modal").classList.add("on"); $("#mNo").focus();
    const done = (v) => { $("#modal").classList.remove("on"); $("#mYes").onclick = $("#mNo").onclick = null; res(v); };
    $("#mYes").onclick = () => done(true); $("#mNo").onclick = () => done(false);
  });
}
export const MOVE_CHECKS = `<ul><li>Cell clear, hand on the emergency stop</li><li>Pendant in Remote Control</li><li>Payload on the pendant matches what is on the flange</li></ul>`;

/* ------------------------------------------------------------------ a small line chart (canvas, hover read-out) */
export class Chart {
  constructor(host, opts = {}) {
    host.classList.add("chart"); if (opts.cls) host.classList.add(opts.cls);
    host.innerHTML = `<canvas></canvas><div class="tip"></div>`;
    this.c = $("canvas", host); this.tip = $(".tip", host); this.o = opts; this.legend = opts.legend || null;
    this.t = []; this.y = []; this.series = []; this.hover = null; this.dirty = true; this.marks = [];
    this.setSeries(opts.series || []);
    this.c.addEventListener("mousemove", (e) => { const r = this.c.getBoundingClientRect(); this.hover = e.clientX - r.left; this.dirty = true; });
    this.c.addEventListener("mouseleave", () => { this.hover = null; this.tip.style.display = "none"; this.dirty = true; });
    if (opts.onClick) this.c.addEventListener("click", (e) => { const t = this.tAt(e); if (t !== null) opts.onClick(t); });
    Chart.all.add(this);
  }
  tAt(e) { const r = this.c.getBoundingClientRect(), L = 52, R = 10, x = e.clientX - r.left; if (!this.t.length) return null;
    const t0 = this.o.window ? this.t[this.t.length - 1] - this.o.window : this.t[0], t1 = this.t[this.t.length - 1];
    return t0 + ((x - L) / (r.width - L - R)) * (t1 - t0); }
  setSeries(series) {
    this.series = series; this.y = series.map(() => []); this.t = [];
    if (this.legend) this.legend.innerHTML = series.map((s) => `<span><i style="background:${s.color}"></i>${esc(s.name)}</span>`).join("");
    this.dirty = true;
  }
  push(t, vals) {
    if (this.t.length && t <= this.t[this.t.length - 1]) return;
    this.t.push(t); vals.forEach((v, i) => this.y[i] && this.y[i].push(v));
    const w = this.o.window;
    while (w && this.t.length > 2 && this.t[0] < t - w) { this.t.shift(); this.y.forEach((a) => a.shift()); }
    this.dirty = true;
  }
  setData(t, ys) { this.t = t || []; this.y = ys || []; this.dirty = true; }
  setMarks(m) { this.marks = m || []; this.dirty = true; }
  draw() {
    if (!this.dirty || !this.c.offsetParent) return;
    this.dirty = false;
    const dpr = window.devicePixelRatio || 1, W = this.c.clientWidth, H = this.c.clientHeight;
    if (!W || !H) return;
    if (this.c.width !== Math.round(W * dpr) || this.c.height !== Math.round(H * dpr)) { this.c.width = Math.round(W * dpr); this.c.height = Math.round(H * dpr); }
    const g = this.c.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, H);
    const L = 52, R = 10, T = 8, B = 20, pw = W - L - R, ph = H - T - B, n = this.t.length;
    g.font = "11px " + css("--mono"); g.fillStyle = css("--text-3");
    if (n < 2 || !this.y.length) { g.fillText(this.o.empty || "no data yet", L + 8, T + 18); return; }
    let t0 = this.t[0], t1 = this.t[n - 1];
    if (this.o.window) t0 = t1 - this.o.window;
    let lo = Infinity, hi = -Infinity;
    if (this.o.yRange) [lo, hi] = this.o.yRange;
    else this.y.forEach((a) => a.forEach((v) => { if (v !== null && Number.isFinite(v)) { if (v < lo) lo = v; if (v > hi) hi = v; } }));
    if (!Number.isFinite(lo)) { g.fillText("no finite values", L + 8, T + 18); return; }
    const ms = this.o.minSpan || 1e-6;
    if (hi - lo < ms) { const m = (hi + lo) / 2; lo = m - ms / 2; hi = m + ms / 2; }
    if (!this.o.yRange) { const pad = (hi - lo) * 0.08; lo -= pad; hi += pad; }
    const X = (t) => L + ((t - t0) / (t1 - t0 || 1)) * pw, Y = (v) => T + (1 - (v - lo) / (hi - lo)) * ph;
    g.strokeStyle = css("--line"); g.lineWidth = 1;
    for (let k = 0; k <= 4; k++) { const v = lo + ((hi - lo) * k) / 4, y = Math.round(Y(v)) + 0.5; g.beginPath(); g.moveTo(L, y); g.lineTo(W - R, y); g.stroke(); g.fillText(fmtTick(v, hi - lo), 4, y + 4); }
    if (lo < 0 && hi > 0) { g.strokeStyle = css("--line-3"); const y = Math.round(Y(0)) + 0.5; g.beginPath(); g.moveTo(L, y); g.lineTo(W - R, y); g.stroke(); }
    const tl = this.o.window ? `-${this.o.window} s` : `${fx(t0, 1)} s`, tr = this.o.window ? "now" : `${fx(t1, 1)} s`;
    g.fillText(tl, L, H - 5); g.fillText(tr, W - R - g.measureText(tr).width, H - 5);
    if (this.o.yLabel) g.fillText(this.o.yLabel, L + pw / 2 - g.measureText(this.o.yLabel).width / 2, H - 5);
    this.marks.forEach((m) => { if (m.t < t0 || m.t > t1) return; const x = Math.round(X(m.t)) + 0.5; g.strokeStyle = m.color || css("--warn"); g.setLineDash([3, 3]); g.beginPath(); g.moveTo(x, T); g.lineTo(x, T + ph); g.stroke(); g.setLineDash([]); });
    g.lineWidth = 1.6; g.lineJoin = "round";
    this.y.forEach((a, i) => {
      if (!this.series[i]) return;
      g.strokeStyle = this.series[i].color; g.setLineDash(this.series[i].dash || []); g.beginPath(); let pen = false;
      const step = Math.max(1, Math.floor(n / (pw * 2)));
      for (let k = 0; k < n; k += step) { const v = a[k]; if (v === null || v === undefined || !Number.isFinite(v) || this.t[k] < t0) { pen = false; continue; }
        const x = X(this.t[k]), y = Y(v); if (pen) g.lineTo(x, y); else { g.moveTo(x, y); pen = true; } }
      g.stroke(); g.setLineDash([]);
    });
    if (this.hover !== null && this.hover > L && this.hover < W - R) {
      const tt = t0 + ((this.hover - L) / pw) * (t1 - t0);
      let lo2 = 0, hi2 = n - 1; while (hi2 - lo2 > 1) { const mid = (lo2 + hi2) >> 1; if (this.t[mid] < tt) lo2 = mid; else hi2 = mid; }
      const k = Math.abs(this.t[lo2] - tt) < Math.abs(this.t[hi2] - tt) ? lo2 : hi2;
      const x = X(this.t[k]); g.strokeStyle = css("--text-3"); g.lineWidth = 1; g.beginPath(); g.moveTo(x + 0.5, T); g.lineTo(x + 0.5, T + ph); g.stroke();
      const rows = this.y.map((a, i) => { const v = a[k]; if (!this.series[i] || v === null || v === undefined || !Number.isFinite(v)) return "";
        g.fillStyle = this.series[i].color; g.beginPath(); g.arc(x, Y(v), 3.5, 0, 7); g.fill();
        return `<div><span style="color:${this.series[i].color}">&#9632;</span> ${esc(this.series[i].name)} ${fmtTick(v, hi - lo, 3)}</div>`; }).join("");
      this.tip.innerHTML = `<div class="muted">${this.o.window ? fx(this.t[k] - t1, 2) + " s" : "t " + fx(this.t[k], 3) + " s"}</div>${rows}`;
      this.tip.style.display = "block"; const tw = this.tip.offsetWidth; this.tip.style.left = Math.min(W - tw - 4, x + 12) + "px"; this.tip.style.top = T + 4 + "px";
    }
  }
}
Chart.all = new Set();
export function fmtTick(v, span, extra = 0) {
  const d = span >= 100 ? 0 : span >= 10 ? 1 : span >= 1 ? 2 : span >= 0.1 ? 3 : 4;
  const s = Number(v).toFixed(Math.min(6, d + extra));
  return /^-0(\.0*)?$/.test(s) ? s.slice(1) : s;
}
export function chartLoop() { Chart.all.forEach((c) => c.draw()); requestAnimationFrame(chartLoop); }

/* ------------------------------------------------------------------ small builders */
export const panel = (title, ic, body, extra = "", cls = "") =>
  `<div class="panel ${cls}"><header><h2>${ic ? icon(ic) : ""}${title}</h2>${extra}</header><div class="pb">${body}</div></div>`;
export const pageHead = (crumb, title, lede, acts = "") =>
  `<div class="ph"><div><div class="crumb">${crumb}</div><h1>${title}</h1>${lede ? `<p>${lede}</p>` : ""}</div><div class="acts">${acts}</div></div>`;
export function setLamp(id, cls, text, title) {
  const el = $(id); if (!el) return;
  el.className = "lamp " + (cls || ""); $(".lv", el).textContent = text; if (title !== undefined) el.title = title;
}
export const rotvecToMat = (rv) => {
  const th = Math.hypot(rv[0], rv[1], rv[2]); if (th < 1e-12) return [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  const k = rv.map((v) => v / th), c = Math.cos(th), s = Math.sin(th), C = 1 - c;
  return [[c + k[0] * k[0] * C, k[0] * k[1] * C - k[2] * s, k[0] * k[2] * C + k[1] * s],
    [k[1] * k[0] * C + k[2] * s, c + k[1] * k[1] * C, k[1] * k[2] * C - k[0] * s],
    [k[2] * k[0] * C - k[1] * s, k[2] * k[1] * C + k[0] * s, c + k[2] * k[2] * C]];
};
export const matVec = (M, v) => [0, 1, 2].map((i) => M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2]);
export function busy() { return !!(S.st && S.st.busy); }

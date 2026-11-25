/* SONAIR console UI. Plain JavaScript, no build step, no network access beyond the console agent. */
"use strict";

const $ = (s, el = document) => el.querySelector(s);
const $$ = (s, el = document) => Array.from(el.querySelectorAll(s));
const esc = (s) => String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
const fx = (v, d = 3) => (v === null || v === undefined || Number.isNaN(v) ? "-" : Number(v).toFixed(d));
const DEG = 180 / Math.PI;
const JOINTS = ["Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"];
const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
const SERIES = () => [1, 2, 3, 4, 5, 6].map((i) => css(`--s${i}`));

const S = { ws: null, rid: 0, pending: new Map(), hello: null, st: null, page: "overview", cmds: [], payloads: {},
  selected: new Set(), benchRuns: [], sessions: [], dataTab: "sessions", lastSessionId: null, units: [{ name: "tool", bus: 1, addr: "0x6b" }] };

/* ------------------------------------------------------------------ connection */
function wsUrl() {
  const tok = new URLSearchParams(location.search).get("token");
  return `${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws` + (tok ? `?token=${encodeURIComponent(tok)}` : "");
}
function connect() {
  const ws = new WebSocket(wsUrl());
  S.ws = ws;
  ws.onopen = () => { $("#offline").classList.remove("on"); };
  ws.onclose = () => { $("#offline").classList.add("on"); releaseJog(); setTimeout(connect, 1500); };
  ws.onmessage = (ev) => {
    let m; try { m = JSON.parse(ev.data); } catch { return; }
    if (m.rid !== undefined && S.pending.has(m.rid)) { S.pending.get(m.rid)(m); S.pending.delete(m.rid); return; }
    if (m.type === "state") onState(m);
    else if (m.type === "hello") onHello(m);
    else if (m.type === "log") addLog(m);
  };
}
function send(msg) { if (S.ws && S.ws.readyState === 1) S.ws.send(JSON.stringify(msg)); }
function req(msg, timeout = 60000) {
  return new Promise((res) => {
    if (!S.ws || S.ws.readyState !== 1) return res({ ok: false, error: "not connected" });
    const rid = ++S.rid; S.pending.set(rid, res);
    S.ws.send(JSON.stringify({ ...msg, rid }));
    setTimeout(() => { if (S.pending.has(rid)) { S.pending.delete(rid); res({ ok: false, error: "no answer from the agent" }); } }, timeout);
  });
}
function toast(msg, kind = "") {
  const d = document.createElement("div"); d.className = kind; d.textContent = msg;
  $("#toast").appendChild(d); setTimeout(() => d.remove(), kind === "bad" ? 9000 : 5000);
}
function result(r, okMsg) {
  const ok = r && r.ok !== false;
  const msg = ok ? (okMsg || r.msg || "done") : (r.error || r.msg || "failed");
  toast(msg, ok ? "good" : "bad"); return ok;
}
function confirmBox(title, html, yes = "Confirm") {
  return new Promise((res) => {
    $("#m-title").textContent = title; $("#m-body").innerHTML = html; $("#m-yes").textContent = yes;
    $("#modal").classList.add("on");
    const done = (v) => { $("#modal").classList.remove("on"); $("#m-yes").onclick = $("#m-no").onclick = null; res(v); };
    $("#m-yes").onclick = () => done(true); $("#m-no").onclick = () => done(false);
  });
}

/* ------------------------------------------------------------------ charts */
class Chart {
  constructor(canvas, legend, opts) {
    this.c = canvas; this.tip = canvas.parentElement.querySelector(".tip"); this.leg = legend; this.o = opts;
    this.t = []; this.y = []; this.hover = null; this.dirty = true;
    this.setSeries(opts.series || []);
    canvas.addEventListener("mousemove", (e) => { const r = canvas.getBoundingClientRect(); this.hover = e.clientX - r.left; this.dirty = true; });
    canvas.addEventListener("mouseleave", () => { this.hover = null; this.tip.style.display = "none"; this.dirty = true; });
  }
  setSeries(series) {
    this.series = series; this.y = series.map(() => []); this.t = [];
    if (this.leg) this.leg.innerHTML = series.length > 1 ? series.map((s) => `<span><i style="background:${s.color}"></i>${esc(s.name)}</span>`).join("") : "";
    this.dirty = true;
  }
  push(t, vals) {
    if (this.t.length && t <= this.t[this.t.length - 1]) return;
    this.t.push(t); vals.forEach((v, i) => this.y[i] && this.y[i].push(v));
    const w = this.o.window;
    while (w && this.t.length && this.t[0] < t - w) { this.t.shift(); this.y.forEach((a) => a.shift()); }
    this.dirty = true;
  }
  setData(t, ys) { this.t = t; this.y = ys; this.dirty = true; }
  draw() {
    if (!this.dirty || !this.c.offsetParent) return;
    this.dirty = false;
    const dpr = window.devicePixelRatio || 1, W = this.c.clientWidth, H = this.c.clientHeight;
    if (this.c.width !== W * dpr || this.c.height !== H * dpr) { this.c.width = W * dpr; this.c.height = H * dpr; }
    const g = this.c.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, H);
    const L = 52, R = 10, T = 8, B = 22, pw = W - L - R, ph = H - T - B;
    const n = this.t.length;
    g.font = "11px " + css("--mono"); g.fillStyle = css("--muted");
    if (n < 2) { g.fillText(this.o.empty || "no data yet", L + 8, T + 18); return; }
    let t0 = this.t[0], t1 = this.t[n - 1];
    if (this.o.window) t0 = t1 - this.o.window;
    let lo = Infinity, hi = -Infinity;
    this.y.forEach((a) => a.forEach((v) => { if (v !== null && Number.isFinite(v)) { if (v < lo) lo = v; if (v > hi) hi = v; } }));
    if (!Number.isFinite(lo)) { g.fillText("no finite values", L + 8, T + 18); return; }
    if (hi - lo < (this.o.minSpan || 1e-6)) { const m = (hi + lo) / 2, h = (this.o.minSpan || 1e-6) / 2; lo = m - h; hi = m + h; }
    const pad = (hi - lo) * 0.08; lo -= pad; hi += pad;
    const X = (t) => L + ((t - t0) / (t1 - t0 || 1)) * pw, Y = (v) => T + (1 - (v - lo) / (hi - lo)) * ph;
    g.strokeStyle = css("--line"); g.lineWidth = 1;
    for (let k = 0; k <= 4; k++) {
      const v = lo + ((hi - lo) * k) / 4, y = Math.round(Y(v)) + 0.5;
      g.beginPath(); g.moveTo(L, y); g.lineTo(W - R, y); g.stroke();
      g.fillText(fmtTick(v, hi - lo), 4, y + 4);
    }
    const tl = this.o.window ? `-${this.o.window} s` : `${fx(t0, 1)} s`;
    g.fillText(tl, L, H - 6); const tr = this.o.window ? "now" : `${fx(t1, 1)} s`;
    g.fillText(tr, W - R - g.measureText(tr).width, H - 6);
    if (this.o.yLabel) g.fillText(this.o.yLabel, L + pw / 2 - g.measureText(this.o.yLabel).width / 2, H - 6);
    g.lineWidth = 2; g.lineJoin = "round";
    this.y.forEach((a, i) => {
      g.strokeStyle = this.series[i].color; g.beginPath(); let pen = false;
      for (let k = 0; k < n; k++) {
        const v = a[k]; if (v === null || !Number.isFinite(v) || this.t[k] < t0) { pen = false; continue; }
        const x = X(this.t[k]), y = Y(v); if (pen) g.lineTo(x, y); else { g.moveTo(x, y); pen = true; }
      }
      g.stroke();
    });
    if (this.hover !== null && this.hover > L && this.hover < W - R) {
      const tt = t0 + ((this.hover - L) / pw) * (t1 - t0);
      let k = 0, best = Infinity; for (let j = 0; j < n; j++) { const d = Math.abs(this.t[j] - tt); if (d < best) { best = d; k = j; } }
      const x = X(this.t[k]); g.strokeStyle = css("--text-2"); g.lineWidth = 1; g.beginPath(); g.moveTo(x + 0.5, T); g.lineTo(x + 0.5, T + ph); g.stroke();
      const rows = this.y.map((a, i) => {
        const v = a[k]; if (v === null || !Number.isFinite(v)) return "";
        g.fillStyle = this.series[i].color; g.beginPath(); g.arc(x, Y(v), 4, 0, 7); g.fill();
        g.strokeStyle = css("--panel"); g.lineWidth = 2; g.stroke();
        return `<div><span style="color:${this.series[i].color}">&#9632;</span> ${esc(this.series[i].name)} ${fmtTick(v, hi - lo, 4)}</div>`;
      }).join("");
      const tlab = this.o.window ? `${fx(this.t[k] - t1, 2)} s` : `t ${fx(this.t[k], 3)} s`;
      this.tip.innerHTML = `<div class="muted">${tlab}</div>${rows}`; this.tip.style.display = "block";
      const tw = this.tip.offsetWidth; this.tip.style.left = Math.min(W - tw - 4, x + 12) + "px"; this.tip.style.top = T + 4 + "px";
    }
  }
}
function fmtTick(v, span, extra = 0) {
  const d = span >= 100 ? 0 : span >= 10 ? 1 : span >= 1 ? 2 : span >= 0.1 ? 3 : 4;
  const s = Number(v).toFixed(Math.min(6, d + extra));
  return /^-0(\.0*)?$/.test(s) ? s.slice(1) : s;
}
const charts = {};
function frame() { Object.values(charts).forEach((c) => c.draw()); requestAnimationFrame(frame); }

/* ------------------------------------------------------------------ navigation */
function show(p) {
  S.page = p; $$(".page").forEach((el) => el.classList.toggle("on", el.id === "pg-" + p));
  $$("#nav a").forEach((a) => a.classList.toggle("on", a.dataset.p === p));
  Object.values(charts).forEach((c) => (c.dirty = true));
  if (p === "data") refreshData();
  if (p === "benchmark") loadBench();
}
window.addEventListener("hashchange", () => show(location.hash.slice(1) || "overview"));
document.addEventListener("keydown", (e) => {
  if (e.key === "Escape") { e.preventDefault(); stopAll(); return; }
  if (/INPUT|SELECT|TEXTAREA/.test(document.activeElement.tagName)) return;
  const pages = ["overview", "robot", "sensors", "sessions", "campaign", "data", "benchmark"];
  const k = parseInt(e.key, 10); if (k >= 1 && k <= 7) location.hash = pages[k - 1];
});
async function stopAll() {
  releaseJog();
  const r = await req({ type: "stop_all", reason: "operator STOP" }, 15000);
  toast(r.ok ? "STOP sent: executor, campaign, jog and robot stopped" : "STOP: " + (r.error || "check the robot"), r.ok ? "good" : "bad");
}

/* ------------------------------------------------------------------ hello and state */
function onHello(m) {
  S.hello = m; S.cmds = m.commands || []; S.payloads = m.payloads || {};
  $("#p-mock").hidden = !m.mock;
  $("#foot").innerHTML = `${esc(m.version)}<br>${m.mock ? "MOCK: simulated controller and IMU" : "robot " + esc(m.robot_host || "not set")}<br>sessions in ${esc(m.runs_dir)}`;
  if (!$("#r-host").value) $("#r-host").value = m.robot_host || "";
  $("#log").innerHTML = ""; (m.log || []).forEach(addLog);
  const popts = Object.entries(S.payloads).map(([k, v]) => `<option value="${esc(k)}">${esc(k)}: ${esc(v.label || "")}</option>`).join("");
  ["#r-payload", "#s-payload", "#c-payload"].forEach((s) => ($(s).innerHTML = (s === "#r-payload" ? "" : '<option value="">not set</option>') + popts));
  $("#s-cmd").innerHTML = S.cmds.map((c) => `<option value="${esc(c.id)}">${esc(c.id)}</option>`).join("");
  $("#c-preset").innerHTML = (m.presets || []).map((p) => `<option>${esc(p)}</option>`).join("");
  renderCmdTable(); cmdInfo(); payloadInfo(); renderUnits();
}
function addLog(e) {
  const el = $("#log"); const d = document.createElement("div"); d.className = e.level || "";
  d.innerHTML = `<span class="ts">${esc(e.t)}</span>${esc(e.msg)}`; el.appendChild(d);
  while (el.children.length > 400) el.firstChild.remove();
  el.scrollTop = el.scrollHeight;
}
function pill(id, cls, text) { const p = $(id); p.className = "pill " + cls; p.querySelector("span").textContent = text; }

function onState(m) {
  S.st = m;
  const L = m.link || {}, R = m.robot || {};
  const linkOk = L.enabled && L.connected && L.source === "rtde";
  pill("#p-link", !L.enabled ? "" : linkOk ? (L.requested_hz >= 499 ? "good" : "warn") : "bad",
    !L.enabled ? "Robot link off" : `${L.source || "no data"} ${L.rate_hz ? Math.round(L.rate_hz) + " Hz" : ""}`);
  const mode = R.robot_mode_text, saf = R.safety_mode_text;
  pill("#p-mode", !mode ? "" : mode === "RUNNING" && saf === "NORMAL" ? "good" : saf && saf !== "NORMAL" ? "bad" : "warn", mode ? `${mode} / ${saf || "?"}` : "Mode");
  const imus = Object.values(m.imu || {});
  pill("#p-imu", !imus.length ? "" : imus.every((u) => u.alive && !u.error) ? "good" : "bad", imus.length ? `IMU ${imus.map((u) => u.name + " " + (u.rate_hz ? Math.round(u.rate_hz) : "-")).join(", ")}` : "IMU off");
  const se = m.session;
  pill("#p-rec", se && se.state === "recording" ? "bad" : se && se.qc ? (se.qc === "PASS" ? "good" : "warn") : "",
    se && se.state === "recording" ? `REC ${fx(se.elapsed_s, 0)} s` : se ? `last: ${se.qc || se.state}` : "No session");
  $("#busy").classList.toggle("on", !!m.busy);
  $("#busy-txt").textContent = m.exec ? `${m.exec.cmd} at x${m.exec.speed}, ${fx(m.exec.elapsed_s, 0)} s` : "";
  if (R.actual_q && R.timestamp) charts.joints.push(R.timestamp, R.actual_q.map((v) => v * DEG));
  imus.forEach((u, i) => { if (u.latest && u.latest.gyro) S.imuT = u.latest.t_host; });
  if (imus.length && S.imuT) {
    if (charts.imu.series.length !== imus.length) charts.imu.setSeries(imus.map((u, i) => ({ name: u.name + " |gyro|", color: SERIES()[i] })));
    charts.imu.push(S.imuT, imus.map((u) => (u.latest && u.latest.gyro ? Math.hypot(...u.latest.gyro) : null)));
  }
  renderOverview(m); renderRobot(m); renderSensors(m); renderSession(m); renderCampaign(m);
}

/* ------------------------------------------------------------------ overview */
function renderOverview(m) {
  if (S.page !== "overview") return;
  const L = m.link || {}, R = m.robot || {};
  $("#o-link").textContent = L.enabled ? (L.source || "-").toUpperCase() : "OFF";
  $("#o-link-sub").textContent = L.enabled ? `${L.host || ""}  ${L.rate_hz ? fx(L.rate_hz, 0) + " Hz" : ""}  ${L.controller_version ? "controller " + L.controller_version : ""}` : "not started";
  $("#o-mode").textContent = R.robot_mode_text || "-";
  $("#o-mode-sub").textContent = R.safety_mode_text ? `safety ${R.safety_mode_text}, program ${R.runtime_state_text || "-"}, slider ${fx((R.speed_scaling ?? 0) * 100, 0)} %` : "";
  const imus = Object.values(m.imu || {});
  $("#o-imu").textContent = imus.length ? String(imus.length) : "0";
  $("#o-imu-sub").textContent = imus.map((u) => `${u.name} (${u.kind}) ${u.rate_hz ? fx(u.rate_hz, 0) + " Hz" : "-"}`).join(", ") || "none streaming";
  const se = m.session;
  $("#o-rec").textContent = se ? (se.state === "recording" ? "REC" : se.qc || se.state) : "-";
  $("#o-rec-sub").textContent = se ? se.run_id : "no session yet";
  const checks = [
    ["Robot link started", L.enabled], ["Robot data over RTDE (no 30003 fallback)", L.enabled && L.source === "rtde"],
    ["Link at 500 Hz", (L.requested_hz || 0) >= 499], ["Robot RUNNING", R.robot_mode_text === "RUNNING"],
    ["Safety NORMAL", R.safety_mode_text === "NORMAL"], ["Speed slider 100 %", (R.speed_scaling ?? 0) > 0.999, true],
    ["Inertial units streaming", imus.length > 0 && imus.every((u) => u.alive && !u.error), true],
    ["Nothing executing", !m.busy], ["Platform robot link unchanged (hashes)", m.locked_ok],
  ];
  $("#o-checks").innerHTML = checks.map(([t, ok, soft]) => `<li class="${ok ? "ok" : soft ? "wn" : "no"}"><span class="mk">${ok ? "&#10003;" : soft ? "!" : "&#10005;"}</span><span>${esc(t)}</span></li>`).join("")
    + (m.mock ? `<li class="wn"><span class="mk">!</span><span>Mock mode: sessions are labelled mock and never enter a dataset</span></li>` : "");
  const c = m.campaign;
  $("#o-camp").innerHTML = c ? `<b>${esc(c.name)}</b>: ${c.steps.filter((s) => s.state === "done").length} of ${c.steps.length} steps done, state <b>${esc(c.state)}</b>` : "No plan loaded.";
  $("#o-lock").innerHTML = m.locked_ok ? `<span class="status good">unchanged</span> ur_control, ur_telemetry, ur_bridge_ext, ur_jog, ur_kin, scan3d match LOCKED.sha256`
    : `<span class="status bad">changed</span> a locked robot-link file differs from LOCKED.sha256: see the agent log`;
  if (!S.dataCounted) { S.dataCounted = true; countData(); }
}
async function countData() {
  const [a, b] = await Promise.all([req({ type: "data_sessions" }), req({ type: "data_bench" })]);
  const runs = b.runs || [], ses = a.sessions || [];
  const tr = runs.filter((r) => r.split === "train").length, te = runs.length - tr;
  $("#o-data").innerHTML = `<dt>Sessions here</dt><dd>${ses.length} (${ses.filter((s) => s.verdict === "PASS").length} PASS)</dd>
    <dt>Benchmark v1</dt><dd>${runs.length} runs</dd><dt>published</dt><dd>${tr} with targets</dd><dt>held-out</dt><dd>${te} inputs only</dd>`;
}
$("#log-clear").onclick = () => ($("#log").innerHTML = "");

/* ------------------------------------------------------------------ robot */
function renderRobot(m) {
  if (S.page !== "robot") return;
  const R = m.robot || {}, L = m.link || {};
  $("#r-link-msg").textContent = L.enabled ? `${L.connected ? "connected" : "connecting"} to ${L.host}${L.last_error ? ": " + L.last_error : ""}` : "not connected";
  $("#r-connect").textContent = L.enabled ? "Reconnect" : "Connect";
  const q = R.actual_q || [], tq = R.target_q || [], qd = R.actual_qd || [], cur = R.actual_current || [], tmp = R.joint_temperatures || [];
  $("#r-joints").innerHTML = JOINTS.map((j, i) => `<tr><td>${j}</td><td class="num">${fx(q[i] * DEG, 2)}</td><td class="num">${fx(tq[i] * DEG, 2)}</td>
    <td class="num">${fx(qd[i] * DEG, 2)}</td><td class="num">${fx(cur[i], 3)}</td><td class="num">${fx(tmp[i], 1)}</td></tr>`).join("");
  const p = R.actual_TCP_pose || [], f = R.actual_TCP_force || [];
  $("#r-kv").innerHTML = `<dt>tool0 x y z (mm)</dt><dd>${fx(p[0] * 1e3, 1)}  ${fx(p[1] * 1e3, 1)}  ${fx(p[2] * 1e3, 1)}</dd>
    <dt>rotation vector (rad)</dt><dd>${fx(p[3], 4)}  ${fx(p[4], 4)}  ${fx(p[5], 4)}</dd>
    <dt>TCP force (N)</dt><dd>${fx(Math.hypot(f[0] || 0, f[1] || 0, f[2] || 0), 1)}</dd>
    <dt>Robot mode</dt><dd>${esc(R.robot_mode_text || "-")}</dd><dt>Safety</dt><dd>${esc(R.safety_mode_text || "-")}</dd>
    <dt>Program</dt><dd>${esc(R.runtime_state_text || "-")}</dd><dt>Speed scaling</dt><dd>${fx((R.speed_scaling ?? NaN) * 100, 1)} %</dd>
    <dt>Data source</dt><dd>${esc(R._source || "-")}</dd><dt>Link rate</dt><dd>${L.rate_hz ? fx(L.rate_hz, 1) + " Hz" : "-"} (requested ${L.requested_hz || "-"})</dd>
    <dt>Jog</dt><dd>${m.jog ? (m.jog.active ? "moving" : "idle") : "-"}</dd>`;
  $("#r-ctrlt").textContent = R.timestamp ? `controller t ${fx(R.timestamp, 3)} s` : "";
}
$("#r-connect").onclick = async () => {
  const host = $("#r-host").value.trim(); if (!host) return toast("enter the controller address", "bad");
  result(await req({ type: "ur_service_start", host, frequency: parseFloat($("#r-hz").value) }, 30000), "robot link started");
};
$$("[data-ur]").forEach((b) => (b.onclick = async () => {
  if (b.dataset.confirm && !(await confirmBox(b.textContent, esc(b.dataset.confirm)))) return;
  result(await req({ type: b.dataset.ur }, 15000));
}));
function payloadInfo() {
  const p = S.payloads[$("#r-payload").value];
  $("#r-payload-info").textContent = p ? `${p.mass_kg} kg, centre of gravity ${p.cog_m.map((v) => fx(v * 1e3, 1)).join(", ")} mm in tool0. CAD masses until weighed.` : "";
}
$("#r-payload").onchange = payloadInfo;
$("#r-payload-set").onclick = async () => {
  const k = $("#r-payload").value, p = S.payloads[k]; if (!p) return;
  if (!(await confirmBox("Set the payload on the controller", `<p>${esc(k)}: ${esc(p.label)}</p><ul><li>mass ${p.mass_kg} kg</li><li>centre of gravity ${p.cog_m.map((v) => fx(v, 4)).join(", ")} m (tool0)</li></ul><p>Check it matches what is on the flange. A wrong payload changes the joint currents the benchmark scores.</p>`))) return;
  result(await req({ type: "ur_set_payload", mass: p.mass_kg, cog: p.cog_m }));
};

// hold-to-jog: jog_vel every 100 ms with a 400 ms time to live; release, blur or a dropped socket stops it
let jogTimer = null, jogBtn = null;
function jogVec(ax, s) {
  const v = new Array(6).fill(0);
  v[ax] = ax < 3 ? s * Math.min(100, Math.max(1, parseFloat($("#jog-v").value) || 20)) / 1000 : s * Math.min(20, Math.max(1, parseFloat($("#jog-w").value) || 5)) / DEG;
  return v;
}
function releaseJog() {
  if (jogTimer) { clearInterval(jogTimer); jogTimer = null; send({ type: "jog_stop" }); }
  if (jogBtn) { jogBtn.classList.remove("held"); jogBtn = null; }
}
$$(".jog button").forEach((b) => {
  b.addEventListener("pointerdown", (e) => {
    e.preventDefault(); releaseJog(); b.setPointerCapture(e.pointerId); jogBtn = b; b.classList.add("held");
    const xd = jogVec(+b.dataset.ax, +b.dataset.s);
    const tick = async () => {
      const r = await req({ type: "jog_vel", xd, ttl_ms: 400 }, 2000);
      if (r && r.ok === false && r.msg) { toast(r.msg, "bad"); releaseJog(); }
    };
    tick(); jogTimer = setInterval(tick, 100);
  });
  ["pointerup", "pointercancel", "lostpointercapture"].forEach((ev) => b.addEventListener(ev, releaseJog));
});
window.addEventListener("blur", releaseJog);
document.addEventListener("visibilitychange", () => document.hidden && releaseJog());
const step = async (sgn) => result(await req({ type: "jog_step", axis: $("#st-ax").value, distance_mm: sgn * parseFloat($("#st-d").value || "0"), frame: $("#st-fr").value, speed: 0.05 }));
$("#st-pos").onclick = () => step(1); $("#st-neg").onclick = () => step(-1);
$("#r-slider").oninput = () => ($("#r-slider-v").textContent = $("#r-slider").value + " %");
$("#r-slider").onchange = async () => result(await req({ type: "ur_speed_slider", fraction: $("#r-slider").value / 100 }));
$("#fd-on").onclick = async () => result(await req({ type: "ur_freedrive", enable: true }));
$("#fd-off").onclick = async () => result(await req({ type: "ur_freedrive", enable: false }));
$("#u-set").onclick = async () => { const r = await req({ type: "unity_set", target: $("#u-target").value.trim() }); result(r, "forwarding to Unity"); };
$("#u-off").onclick = async () => result(await req({ type: "unity_set", target: "" }), "Unity forwarding off");

/* ------------------------------------------------------------------ sensors */
function renderUnits() {
  $("#i-units").innerHTML = S.units.map((u, i) => `<tr><td><input type="text" data-i="${i}" data-k="name" value="${esc(u.name)}" style="width:90px"></td>
    <td><input type="number" data-i="${i}" data-k="bus" value="${u.bus}" style="width:60px"></td>
    <td><input type="text" data-i="${i}" data-k="addr" value="${esc(u.addr)}" style="width:70px"> <button class="btn sm" data-del="${i}">remove</button></td></tr>`).join("");
  $$("#i-units input").forEach((el) => (el.onchange = () => (S.units[+el.dataset.i][el.dataset.k] = el.dataset.k === "bus" ? parseInt(el.value, 10) : el.value)));
  $$("#i-units [data-del]").forEach((b) => (b.onclick = () => { S.units.splice(+b.dataset.del, 1); renderUnits(); }));
}
$("#i-add").onclick = () => { S.units.push({ name: `unit${S.units.length + 1}`, bus: 1, addr: "0x6a" }); renderUnits(); };
$("#i-start").onclick = async () => {
  const r = await req({ type: "imu_start", units: S.units, odr: parseInt($("#i-odr").value, 10) }, 20000);
  $("#i-msg").textContent = r.ok ? `streaming: ${r.units.join(", ")}` : (r.errors || [r.error]).join("; "); result(r, "IMU streaming");
};
$("#i-stop").onclick = async () => result(await req({ type: "imu_stop" }), "IMU stopped");
function renderSensors(m) {
  if (S.page !== "sensors") return;
  const imus = Object.values(m.imu || {});
  $("#i-live").innerHTML = imus.map((u) => { const l = u.latest || {}, a = l.accel || [], g = l.gyro || [];
    return `<tr><td>${esc(u.name)}</td><td>${esc(u.kind)}</td><td class="num">${fx(u.rate_hz, 1)}</td><td class="num">${a.length ? fx(Math.hypot(...a), 3) : "-"}</td>
    <td class="num">${fx(g[0], 4)}</td><td class="num">${fx(g[1], 4)}</td><td class="num">${fx(g[2], 4)}</td><td class="small">${esc(u.error || "")}</td></tr>`; }).join("")
    || `<tr><td colspan="8" class="muted">No unit streaming. On the Jetson the units are on I2C; in mock mode a simulated unit follows the robot.</td></tr>`;
}
$("#b-run").onclick = async () => {
  const secs = parseFloat($("#b-secs").value) || 10;
  $("#b-out").textContent = `Recording ${secs} s with the robot still...`;
  const r = await req({ type: "bench_check", seconds: secs }, (secs + 30) * 1000);
  if (r.ok === false) { $("#b-out").textContent = r.error; return; }
  const rc = (r.robot || {}).clock || {};
  let h = `<table><thead><tr><th>Stream</th><th class="num">Samples</th><th class="num">Rate (Hz)</th><th class="num">|g| (m/s2)</th><th class="num">Gyro bias (deg/s)</th><th class="num">Gyro noise (deg/s)</th><th class="num">Clock drift (ppm)</th><th class="num">Delay p95 (ms)</th><th>Verdict</th></tr></thead><tbody>`;
  h += `<tr><td>robot (RTDE)</td><td class="num">${(r.robot || {}).packets ?? "-"}</td><td class="num">-</td><td class="num">-</td><td class="num">-</td><td class="num">-</td><td class="num">${fx(rc.drift_ppm, 1)}</td><td class="num">${fx(rc.delay_above_envelope_p95_ms, 2)}</td><td>${r.robot && r.robot.still ? "still" : '<span class="badge FAIL">MOVED</span>'}</td></tr>`;
  Object.entries(r.imu || {}).forEach(([n, u]) => {
    if (u.error) { h += `<tr><td>${esc(n)}</td><td colspan="8">${esc(u.error)}</td></tr>`; return; }
    const c = u.clock || {};
    h += `<tr><td>${esc(n)}</td><td class="num">${u.samples}</td><td class="num">${fx(u.rate_hz, 2)}</td><td class="num">${fx(u.gravity_norm, 4)}</td><td class="num">${u.gyro_bias_dps.map((v) => fx(v, 3)).join(" ")}</td>
      <td class="num">${u.gyro_noise_dps.map((v) => fx(v, 3)).join(" ")}</td><td class="num">${fx(c.drift_ppm, 1)}</td><td class="num">${fx(c.delay_above_envelope_p95_ms, 2)}</td><td><span class="badge ${u.verdict === "PASS" ? "PASS" : "WARN"}">${u.verdict}</span></td></tr>`;
  });
  $("#b-out").innerHTML = h + "</tbody></table>" + (r.note ? `<p class="small" style="color:var(--warning)">${esc(r.note)}</p>` : "");
};

/* ------------------------------------------------------------------ sessions */
function cmdInfo() {
  const c = S.cmds.find((x) => x.id === $("#s-cmd").value), sp = parseFloat($("#s-speed").value) || 0;
  if (!c) { $("#s-cmd-info").textContent = ""; return; }
  const peak = (c.peak_qd ?? NaN) * sp, ok = peak <= 0.6 + 0.005;
  $("#s-cmd-info").innerHTML = `${esc(c.kind || "")}, payload <b>${esc(c.payload || c.payload_schedule || "-")}</b>, ${fx((c.duration_s || 0) / (sp || 1), 1)} s at x${sp}, peak <b style="color:${ok ? "inherit" : "var(--critical)"}">${fx(peak, 2)} rad/s</b>${ok ? "" : " (the executor will refuse: limit 0.6)"}, collision check ${esc(c.collision || c.collision_check || "-")}`;
  if (c.payload && S.payloads[c.payload] && !$("#s-payload").dataset.touched) $("#s-payload").value = c.payload;
}
$("#s-cmd").onchange = cmdInfo; $("#s-speed").oninput = cmdInfo;
$("#s-payload").onchange = () => ($("#s-payload").dataset.touched = "1");
$("#s-motion").onchange = () => ($("#s-cmd-row").style.display = $("#s-motion").value === "executor" ? "" : "none");
$("#s-start").onclick = async () => {
  const motion = $("#s-motion").value, v = $("#s-v").value.trim(), arm = $("#s-arm").value, traj = $("#s-traj").value;
  const msg = { type: "session_start", motion, cmd: motion === "executor" ? $("#s-cmd").value : null, speed: parseFloat($("#s-speed").value),
    cell: v && arm && traj ? `${v}/${arm}/${traj}` : "", rep: parseInt($("#s-rep").value, 10) || 0, payload: $("#s-payload").value || null,
    operator: $("#s-op").value.trim(), experiment: $("#s-exp").value, notes: $("#s-notes").value.trim(), payload_on_pendant: $("#s-payload").value || "" };
  if (motion === "executor") {
    const c = S.cmds.find((x) => x.id === msg.cmd) || {};
    const ok = await confirmBox("Move the robot", `<p><b>${esc(msg.cmd)}</b> at x${msg.speed} (${fx((c.duration_s || 0) / msg.speed, 1)} s, peak ${fx((c.peak_qd || 0) * msg.speed, 2)} rad/s).</p>
      <ul><li>Payload on the pendant: <b>${esc(msg.payload || "not set")}</b></li><li>Speed slider at 100 %</li><li>Cell clear, hand on the emergency stop</li></ul>
      <p>The executor first moves slowly to the command's start pose.</p>`, "Run");
    if (!ok) return;
  }
  const r = await req(msg, 30000); $("#s-msg").textContent = r.ok ? (r.run_id || `started ${r.cmd}`) : r.error; result(r, "session started");
};
$("#s-stop").onclick = async () => { const r = await req({ type: "session_stop" }, 60000); showLast(r); result(r, "session stopped"); };
function renderSession(m) {
  const se = m.session;
  if (se && se.state === "done" && se.run_id !== S.lastSessionId) { S.lastSessionId = se.run_id; S.dataCounted = false; showLast({ run_id: se.run_id, qc: { verdict: se.qc, reasons: se.reasons } }); }
  if (S.page !== "sessions") return;
  $("#s-now").innerHTML = se ? `<dt>Session</dt><dd>${esc(se.run_id)}</dd><dt>State</dt><dd>${esc(se.state)}</dd><dt>Elapsed</dt><dd>${fx(se.elapsed_s, 1)} s</dd>
    <dt>Robot rows</dt><dd>${se.rows ?? "-"}</dd><dt>Packet sources</dt><dd>${esc(JSON.stringify(se.sources || {}))}</dd>
    <dt>Command</dt><dd>${esc(se.meta.cmd || "passive")} ${se.meta.speed ? "x" + se.meta.speed : ""}</dd><dt>Cell</dt><dd>${esc(se.meta.cell || "-")}</dd>
    <dt>Payload</dt><dd>${esc(se.meta.payload || "-")}</dd>${se.mock ? "<dt>Mock</dt><dd>yes</dd>" : ""}` : `<dt>Session</dt><dd>none</dd>`;
  $("#s-exec").innerHTML = m.exec ? m.exec.tail.map((l) => `<div>${esc(l)}</div>`).join("") + (m.exec.running ? "" : `<div class="muted">exit code ${m.exec.rc}</div>`) : `<div class="muted">not started</div>`;
}
function showLast(r) {
  if (!r || !r.run_id) return;
  const q = r.qc || {};
  $("#s-last").innerHTML = `<div class="row"><span class="badge ${esc(q.verdict || "na")}">${esc(q.verdict || "-")}</span><span class="mono">${esc(r.run_id)}</span><span class="sp"></span>
    <button class="btn sm" id="s-exp-btn">Export</button><button class="btn sm" id="s-prev-btn">Preview</button></div>
    <ul class="small">${(q.reasons || []).map((x) => `<li>${esc(x)}</li>`).join("") || "<li>no reasons: every gate passed</li>"}</ul>`;
  $("#s-exp-btn").onclick = () => exportSession(r.run_id);
  $("#s-prev-btn").onclick = () => { location.hash = "data"; preview("session", r.run_id); };
}
async function exportSession(id) {
  const r = await req({ type: "session_export", run_id: id }, 120000);
  result(r, r.ok ? `exported: ${r.platform_run.split("/").slice(-3).join("/")} (IMU lag ${r.lag_ms} ms applied)` : "");
}

/* ------------------------------------------------------------------ campaign */
function renderCmdTable() {
  const f = ($("#c-filter").value || "").toLowerCase(), sp = parseFloat($("#c-speed").value) || 1;
  $("#c-cmds").innerHTML = S.cmds.filter((c) => !f || c.id.toLowerCase().includes(f)).map((c) => {
    const pk = (c.peak_qd ?? NaN) * sp;
    return `<tr><td><input type="checkbox" data-id="${esc(c.id)}" ${S.selected.has(c.id) ? "checked" : ""}></td><td class="mono small">${esc(c.id)}</td><td>${esc(c.kind || "")}</td>
      <td>${esc(c.payload || (c.payload_schedule ? "schedule" : "-"))}</td><td class="num">${fx(c.duration_s, 1)}</td><td class="num">${fx(c.peak_qd, 2)}</td>
      <td class="num" style="color:${pk > 0.605 ? "var(--critical)" : "inherit"}">${fx(pk, 2)}</td><td>${c.split ? `<span class="badge ${c.split === "test" ? "test" : "train"}">${esc(c.split)}</span>` : ""}</td></tr>`;
  }).join("");
  $$("#c-cmds input").forEach((el) => (el.onchange = () => (el.checked ? S.selected.add(el.dataset.id) : S.selected.delete(el.dataset.id))));
}
$("#c-filter").oninput = renderCmdTable; $("#c-speed").oninput = renderCmdTable;
$("#c-build").onclick = async () => {
  if (!S.selected.size) return toast("tick at least one command", "bad");
  const r = await req({ type: "campaign_build", commands: [...S.selected], speed: parseFloat($("#c-speed").value), repeats: parseInt($("#c-rep").value, 10),
    settle_s: parseFloat($("#c-settle").value), experiment: $("#c-exp").value, payload: $("#c-payload").value, payload_on_pendant: $("#c-payload").value,
    name: $("#c-name").value.trim() || "custom" });
  $("#c-msg").textContent = r.ok ? `${r.steps} steps` : r.error; result(r, `plan built: ${r.steps} steps`);
};
$("#c-build-preset").onclick = async () => { const r = await req({ type: "campaign_build", preset: $("#c-preset").value }); $("#c-msg").textContent = r.ok ? `${r.steps} steps` : r.error; result(r, `plan built: ${r.steps} steps`); };
$("#c-start").onclick = async () => {
  const c = S.st && S.st.campaign; if (!c) return toast("build a plan first", "bad");
  const fast = c.steps.filter((s) => { const k = S.cmds.find((x) => x.id === s.cmd); return k && k.peak_qd * s.speed > 0.605; });
  const ok = await confirmBox("Start the campaign", `<p><b>${esc(c.name)}</b>: ${c.steps.length} steps, ${$("#c-auto").checked ? "auto-advance after " + c.settle_s + " s" : "the console waits for you before each step"}.</p>
    <ul><li>Payload on the pendant matches the plan</li><li>Speed slider at 100 %</li><li>Cell clear, hand on the emergency stop</li></ul>${fast.length ? `<p style="color:var(--critical)">${fast.length} steps exceed 0.6 rad/s and will be refused by the executor.</p>` : ""}`, "Start");
  if (ok) result(await req({ type: "campaign_start", auto: $("#c-auto").checked }), "campaign started");
};
$("#c-next").onclick = async () => result(await req({ type: "campaign_next" }), "next step started");
$("#c-resume").onclick = async () => result(await req({ type: "campaign_resume" }), "campaign resumed");
$("#c-abort").onclick = async () => { if (await confirmBox("Abort the campaign", "Stops the executor and the robot, and ends the current session.", "Abort")) result(await req({ type: "campaign_abort" }, 15000), "campaign aborted"); };
$("#c-auto").onchange = () => send({ type: "campaign_auto", auto: $("#c-auto").checked });
function renderCampaign(m) {
  if (S.page !== "campaign") return;
  const c = m.campaign;
  $("#c-plan-name").textContent = c ? `${c.name}: ${c.state}, step ${Math.min(c.index + 1, c.steps.length)} of ${c.steps.length}` : "";
  const key = c ? JSON.stringify([c.state, c.index, c.steps.map((s) => [s.state, s.qc])]) : "";
  if (key === S.campKey) return; S.campKey = key;
  $("#c-steps").innerHTML = c ? c.steps.map((s, i) => `<tr class="${i === c.index && c.state === "running" ? "sel" : ""}"><td class="num">${i + 1}</td><td class="mono small">${esc(s.cmd)}</td>
    <td class="num">${s.speed}</td><td class="num">${s.rep}</td><td>${esc(s.state)}</td><td>${s.qc ? `<span class="badge ${esc(s.qc)}">${esc(s.qc)}</span>` : ""}</td>
    <td class="mono small">${esc(s.run_id || "")}</td><td class="small">${esc(s.note || "")}</td></tr>`).join("") : `<tr><td colspan="8" class="muted">No plan. Tick commands above and build one, or build a preset.</td></tr>`;
}

/* ------------------------------------------------------------------ data */
$$("#d-tabs button").forEach((b) => (b.onclick = () => {
  S.dataTab = b.dataset.t; $$("#d-tabs button").forEach((x) => x.classList.toggle("on", x === b));
  $("#d-sess-wrap").hidden = S.dataTab !== "sessions"; $("#d-bench-wrap").hidden = S.dataTab !== "bench";
}));
$("#d-refresh").onclick = () => refreshData();
async function refreshData() {
  const [a, b] = await Promise.all([req({ type: "data_sessions" }), S.benchRuns.length ? Promise.resolve(null) : req({ type: "data_bench" })]);
  S.sessions = a.sessions || [];
  if (b) {
    S.benchRuns = b.runs || [];
    const ss = [...new Set(S.benchRuns.map((r) => r.session))].sort();
    $("#d-f-sess").innerHTML = `<option value="">all sessions</option>` + ss.map((s) => `<option>${esc(s)}</option>`).join("");
  }
  $("#d-sess").innerHTML = S.sessions.map((s) => `<tr><td class="mono small">${esc(s.run_id)}${s.mock ? ' <span class="badge na">mock</span>' : ""}</td>
    <td><span class="badge ${esc(s.verdict || "na")}" title="${esc((s.reasons || []).join("\n"))}">${esc(s.verdict || "-")}</span></td><td class="num">${fx(s.duration_s, 1)}</td>
    <td class="small">${esc((s.motion || {}).command_id || (s.motion || {}).source || "-")}</td>
    <td><button class="btn sm" data-prev="${esc(s.run_id)}">Preview</button> <button class="btn sm" data-exp="${esc(s.run_id)}">${s.exported ? "Re-export" : "Export"}</button></td></tr>`).join("")
    || `<tr><td colspan="5" class="muted">No sessions recorded yet.</td></tr>`;
  $$("#d-sess [data-prev]").forEach((b) => (b.onclick = () => preview("session", b.dataset.prev)));
  $$("#d-sess [data-exp]").forEach((b) => (b.onclick = () => exportSession(b.dataset.exp)));
  renderBench();
}
function renderBench() {
  const sp = $("#d-f-split").value, se = $("#d-f-sess").value, q = $("#d-f-q").value.toLowerCase();
  const rows = S.benchRuns.filter((r) => (!sp || r.split === sp) && (!se || r.session === se) && (!q || r.run_id.includes(q)));
  $("#d-bench").innerHTML = rows.slice(0, 400).map((r) => `<tr data-id="${esc(r.run_id)}" style="cursor:pointer"><td class="mono small">${esc(r.run_id)}</td>
    <td><span class="badge ${r.split === "test" ? "test" : "train"}">${esc(r.split)}</span></td><td class="small">${esc(r.session)}</td><td class="num">${esc(r.joint_vel)}</td>
    <td class="small">${esc(r.arm_config)}</td><td class="small">${esc(r.traj_type)}</td><td class="num">${esc(r.rows)}</td></tr>`).join("");
  $("#d-bench-n").textContent = `${rows.length} of ${S.benchRuns.length} runs`;
  $$("#d-bench tr").forEach((tr) => (tr.onclick = () => preview("bench", tr.dataset.id)));
}
["#d-f-split", "#d-f-sess"].forEach((s) => ($(s).onchange = renderBench)); $("#d-f-q").oninput = renderBench;
async function preview(kind, id) {
  $("#d-prev-id").textContent = id; $("#d-prev-info").textContent = "loading...";
  const r = await req({ type: "data_preview", kind, run_id: id }, 60000);
  if (r.error) { $("#d-prev-info").textContent = r.error; return; }
  const c = SERIES();
  const ys = [r.cmd_elbow.map((v) => (v === null ? null : v * DEG))], ser = [{ name: "elbow commanded (deg)", color: c[0] }];
  if (r.q_elbow) { ys.push(r.q_elbow.map((v) => (v === null ? null : v * DEG))); ser.push({ name: "elbow measured (deg)", color: c[1] }); }
  charts.d1.setSeries(ser); charts.d1.setData(r.t, ys);
  if (r.current_elbow) { charts.d2.setSeries([{ name: "elbow current (A)", color: c[2] }]); charts.d2.setData(r.t, [r.current_elbow]); $("#d-leg2").innerHTML = `<span><i style="background:${c[2]}"></i>elbow current (A)</span>`; }
  else { charts.d2.setSeries([]); charts.d2.setData([], []); charts.d2.o.empty = "held-out run: measured channels are withheld"; $("#d-leg2").innerHTML = ""; }
  const man = r.manifest || {}, q = r.qc;
  $("#d-prev-info").innerHTML = (r.split === "session" ? `QC <span class="badge ${esc((q || {}).verdict || "na")}">${esc((q || {}).verdict || "-")}</span> ` : `<span class="badge ${r.split === "train" ? "train" : "test"}">${r.split === "train" ? "published" : "held-out inputs"}</span> `)
    + esc(Object.entries(man).filter(([k]) => ["joint_vel", "arm_config", "traj_type", "repeat_idx", "carrier_id", "started_utc", "session"].includes(k)).map(([k, v]) => `${k} ${v}`).join(", "));
}

/* ------------------------------------------------------------------ benchmark */
async function loadBench() {
  if (S.benchLoaded) return; S.benchLoaded = true;
  const r = await req({ type: "bench_info" }, 30000);
  const a = r.audit || {}, d = a.data || {}, fl = a.floors || {}, cu = a.current || {}, gy = a.gyro || {};
  const tile = (l, v, s) => `<div class="card tile"><div class="lbl">${l}</div><div class="val">${v}</div><div class="sub">${s}</div></div>`;
  $("#bm-tiles").innerHTML = tile("Benchmark-v1 real runs", d.runs ?? "-", `${d.cells ?? "-"} cells, ${d.held_out_runs ?? "-"} held-out runs, ${fx(d.hours, 2)} h`)
    + tile("Real-to-real floor, current p95", fx(fl.current_p95_A, 3) + " A", `S0 as shipped: ${fx(cu.S0_p95_A, 2)} A`)
    + tile("Gyro GCR p95, K0 (task/2 rows)", fx(((gy.rows || {}).K0_gcr_p95), 3), `task/3 own timestamps minus lag: ${fx(((gy.rows_task3 || {}).K0_gcr_p95), 3)}`)
    + tile("Tool position floor p95", fx(fl.tcp_p95_mm, 3) + " mm", `max per cell ${fx(fl.tcp_p95_mm_max, 3)} mm`);
  const maps = cu.maps || {};
  $("#bm-rules").innerHTML = `<p>Package <b class="mono">${esc(r.task_version || "?")}</b> as defined by the platform, with the <b>sonair-task/3</b> changes applied as options (patches/sonair_benchmark_task3.patch):</p>
    <ul><li><b>Torque to current map fitted per simulator entry</b> on its own runs of the published cells. The single map fitted on S0 has the wrong physical sign (slope ${maps.S0 ? fx(maps.S0.a, 4) : "-"} A per N m; per-entry maps: B2 ${maps.B2 ? fx(maps.B2.a, 4) : "-"}, controller model ${maps.CM ? fx(maps.CM.a, 4) : "-"}).</li>
    <li><b>Gyro target from the IMU's own timestamps</b>, shifted by the measured lag and interpolated onto the robot rows, instead of the latest-sample hold rows that turned half of the gyro gap into a timing artefact.</li>
    <li>One reference simulator (MuJoCo, S0). Isaac Sim, Unity PhysX and any other engine enter as scored Track A entries.</li></ul>
    ${r.task_error ? `<p class="small" style="color:var(--warning)">task module: ${esc(r.task_error)}</p>` : ""}`;
  const lb = r.leaderboard;
  if (lb && lb.entries) {
    const ds = lb.dataset || {};
    $("#bm-lb").innerHTML = `<p class="small">${esc(lb.headline_metric_name || "")} on the tool position, held-out cells. Reference ${esc(lb.reference_simulator || "")}.</p>
      <div class="scroll"><table><thead><tr><th>Entry</th><th>Track</th><th class="num">GCR p95</th><th class="num">GCR median</th><th class="num">Worst cell</th><th class="num">Runs</th></tr></thead><tbody>`
      + lb.entries.map((e) => `<tr><td>${esc(e.name)}<div class="muted small">${esc(e.kind || "")}</div></td><td>${esc(e.track)}</td><td class="num">${fx(e.gcr_p95, 3)}</td><td class="num">${fx(e.gcr_median, 3)}</td><td class="num">${fx(e.worst_cell_gcr, 3)}</td><td class="num">${e.n_runs ?? "-"}</td></tr>`).join("")
      + `</tbody></table></div><p class="small muted">${esc(lb.position_role || "")}. Dataset: ${ds.runs ? Object.entries(ds.runs).map(([k, v]) => k + " " + v).join(", ") : "-"} runs, ${ds.cells ?? "-"} cells, ${ds.held_out_cells ?? "-"} held out.</p>`;
  } else $("#bm-lb").textContent = "No leaderboard file in data/benchmark_v1/site/.";
  $("#bm-cli").textContent = [
    "# M1 report: the MuJoCo twin against the versioned commands",
    "python -m sonair report-m1",
    "# real-data audit (floors, current maps, gyro rule): it scores the held-out cells, so it needs the private full",
    "# run set (bench_runs on the lab drive); the public data/benchmark_v1 has targets for the published cells only",
    "python -m sonair audit-v1 --runs <full bench_runs folder> --cal data/benchmark_v1/calib/imu_cal.json",
    "# a Jetson session into the platform run format",
    "python -m sonair session-export runs_jetson/<run_id>",
    "# the platform's benchmark tools (task package, evaluator, cards)",
    "cd cell_platform/benchmark && python -m sonair_benchmark --help",
  ].join("\n");
}

/* ------------------------------------------------------------------ start */
function init() {
  const c = SERIES();
  charts.joints = new Chart($("#r-chart"), $("#r-leg"), { window: 30, series: JOINTS.map((j, i) => ({ name: j + " (deg)", color: c[i] })), minSpan: 1 });
  charts.imu = new Chart($("#i-chart"), $("#i-leg"), { window: 20, series: [], minSpan: 0.01, empty: "no unit streaming" });
  charts.d1 = new Chart($("#d-chart"), $("#d-leg"), { series: [], empty: "pick a session or a run" });
  charts.d2 = new Chart($("#d-chart2"), $("#d-leg2"), { series: [], empty: "" });
  $("#stop").onclick = stopAll;
  show(location.hash.slice(1) || "overview");
  connect(); requestAnimationFrame(frame);
}
init();

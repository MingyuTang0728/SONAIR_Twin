/* SONAIR Lab: the page shell. Navigation, the header lamps, the run bar, the live arm column, STOP. */
import { $, $$, S, connect, req, send, toast, fx, DEG, JOINTS, onState, onHello, store, setLamp, chartLoop, esc } from "./core.js";
import { twin } from "./twin.js";
import { initJog, release } from "./jog.js";
import overview from "./pages/overview.js";
import robot from "./pages/robot.js";
import rtde from "./pages/rtde.js";
import cameras from "./pages/cameras.js";
import sensors from "./pages/sensors.js";
import grasp from "./pages/grasp.js";
import calibrate from "./pages/calibrate.js";
import record from "./pages/record.js";
import campaign from "./pages/campaign.js";
import data from "./pages/data.js";
import bench from "./pages/bench.js";
import system from "./pages/system.js";

const PAGES = { overview, robot, rtde, cameras, sensors, grasp, calibrate, record, campaign, data, bench, system };
const KEYS = { 1: "overview", 2: "robot", 3: "rtde", 4: "cameras", 5: "sensors", 6: "grasp", 7: "calibrate", 8: "record", 9: "campaign", 0: "data", b: "bench", l: "system" };

/* ------------------------------------------------------------------ theme and columns */
function theme(t) { document.documentElement.dataset.theme = t; store.set("theme", t); twin.setBackground(); }
theme(store.get("theme", "dark"));
$("#btnTheme").onclick = () => theme(document.documentElement.dataset.theme === "light" ? "dark" : "light");
function monitor(on) { $("#body").classList.toggle("nomon", !on); $("#btnMon").setAttribute("aria-pressed", String(on)); store.set("monitor", on); twin.resize(); }
$("#btnMon").onclick = () => monitor($("#body").classList.contains("nomon"));
$("#btnMonHide").onclick = () => monitor(false);
monitor(store.get("monitor", true));
setInterval(() => { $("#clock").textContent = new Date().toLocaleTimeString("en-GB"); }, 1000);

/* ------------------------------------------------------------------ navigation */
function show(p) {
  if (!PAGES[p]) p = "overview";
  S.page = p;
  $$(".page").forEach((el) => el.classList.toggle("on", el.id === "pg-" + p));
  $$("#rail a").forEach((a) => a.classList.toggle("on", a.dataset.p === p));
  // the one 3D view sits on the page when the page has a place for it (Overview, Gripper and sequences, Data),
  // in the live arm column otherwise
  const big = $(`#pg-${p} [data-twin]:not([hidden])`);
  twin.mount(big || $("#monSlot"));
  if (big) $("#monSlot").innerHTML = `<div class="viewport" style="height:70px"><div class="vempty">The twin view is on this page.</div></div>`;
  const pg = PAGES[p];
  if (!S.hello) return;                                    // the page's requests wait for the agent (see onHello below)
  if (pg.onShow) pg.onShow();
  if (S.st) pg.onState && pg.onState(S.st);
}
let shown = false;
onHello(() => { if (!shown) { shown = true; show(location.hash.slice(1) || "overview"); } });
window.addEventListener("hashchange", () => show(location.hash.slice(1)));
document.addEventListener("keydown", (e) => {
  if (e.key === "Escape") { e.preventDefault(); stopAll(); return; }
  if (/INPUT|SELECT|TEXTAREA/.test(document.activeElement.tagName) || e.ctrlKey || e.metaKey || e.altKey) return;
  if (document.querySelector("#jogKeys:checked")) return;          // keyboard jog owns the keys
  const p = KEYS[e.key.toLowerCase()]; if (p) location.hash = p;
});

/* ------------------------------------------------------------------ STOP */
async function stopAll() {
  release();
  const r = await req({ type: "stop_all", reason: "operator STOP" }, 15000);
  toast(r.ok ? "STOP sent: campaign, executor, calibration moves, jog, gripper and robot stopped" : "STOP: " + (r.error || "check the robot"), r.ok ? "good" : "bad");
}
$("#btnStop").onclick = stopAll;

/* ------------------------------------------------------------------ freedrive: held, never latched */
let fdHeld = false;
const fdOn = async (e) => {
  e.preventDefault(); if (fdHeld) return;
  if (S.st && S.st.busy) { toast("refused: a checked command is executing", "bad"); return; }
  fdHeld = true; $("#btnFreedrive").classList.add("held"); $("#fdTag").textContent = "loose"; $("#fdTag").className = "tag warn";
  const r = await req({ type: "ur_freedrive", enable: true }, 5000);
  if (r.ok === false) { toast(r.msg || r.error || "freedrive refused", "bad"); fdOff(); }
};
const fdOff = () => {
  if (!fdHeld) return; fdHeld = false; $("#btnFreedrive").classList.remove("held"); $("#fdTag").textContent = "locked"; $("#fdTag").className = "tag";
  send({ type: "ur_freedrive", enable: false });
};
$("#btnFreedrive").addEventListener("pointerdown", (e) => { $("#btnFreedrive").setPointerCapture(e.pointerId); fdOn(e); });
["pointerup", "pointercancel", "lostpointercapture"].forEach((ev) => $("#btnFreedrive").addEventListener(ev, fdOff));
window.addEventListener("blur", fdOff);

/* ------------------------------------------------------------------ hello, state */
onHello((m) => {
  const b = $("#modeBadge");
  b.textContent = m.mock ? "MOCK CELL" : m.robot_host ? "CELL" : "NO ROBOT";
  b.className = "modebadge " + (m.mock ? "mock" : m.robot_host ? "cell" : "");
  b.title = m.mock ? "Stand-in controller, simulated IMU, rendered cameras: sessions are labelled mock and never enter a dataset" : "";
  $("#rbHost").textContent = (m.robot_host || "-") + (m.mock ? " (mock)" : "");
  $("#railFoot").innerHTML = `${esc(m.version)}<br>sessions in ${esc(m.runs_dir)}<br>STOP is a software stop. The physical emergency stop is the safety device.`;
});

const trBuf = [];
function monitorRender(m) {
  const R = m.robot || {}, L = m.link || {};
  const q = R.actual_q, tq = R.target_q, p = R.actual_TCP_pose;
  $("#monAge").textContent = L.enabled ? (L.connected ? `${L.source || "?"} ${L.rate_hz ? Math.round(L.rate_hz) + " Hz" : ""}` : "connecting") : "no data";
  $("#monAge").className = "tag " + (L.connected && L.source === "rtde" ? "ok" : L.enabled ? "warn" : "");
  if (p) { $("#monX").textContent = fx(p[0] * 1e3, 1); $("#monY").textContent = fx(p[1] * 1e3, 1); $("#monZ").textContent = fx(p[2] * 1e3, 1); }
  $("#monJ").innerHTML = JOINTS.map((j, i) => `<div><b>${j}</b><span>${q ? fx(q[i] * DEG, 2) : "-"}</span></div>`).join("");
  if (q && tq) { trBuf.push([m.t, q.map((v, i) => tq[i] - v)]); while (trBuf.length && trBuf[0][0] < m.t - 5) trBuf.shift(); }
  const rms = [0, 1, 2, 3, 4, 5].map((i) => trBuf.length ? Math.sqrt(trBuf.reduce((a, r) => a + r[1][i] * r[1][i], 0) / trBuf.length) * DEG : null);
  $("#monTr").innerHTML = JOINTS.map((j, i) => `<div class="${rms[i] > 0.5 ? "badv" : rms[i] > 0.05 ? "warnv" : ""}"><b>${j}</b><span>${rms[i] === null ? "-" : fx(rms[i], 3)}</span></div>`).join("");
  $("#monMode").textContent = R.robot_mode_text || "-"; $("#monSafety").textContent = R.safety_mode_text || "-";
  $("#monMode").parentElement.className = "tx " + (R.robot_mode_text && R.robot_mode_text !== "RUNNING" ? "warnv" : "");
  $("#monSafety").parentElement.className = "tx " + (R.safety_mode_text && R.safety_mode_text !== "NORMAL" ? "badv" : "");
  $("#monSpeed").textContent = R.speed_scaling !== undefined ? fx(R.speed_scaling * 100, 0) + " %" : "-";
  const g = m.gripper || {};
  $("#monGrip").textContent = g.connected ? (g.width_mm !== null && g.width_mm !== undefined ? fx(g.width_mm, 1) + " mm" : "-") : "off";
  const f = R.actual_TCP_force; $("#monForce").textContent = f ? fx(Math.hypot(f[0], f[1], f[2]), 1) + " N" : "-";
  $("#monCur").textContent = R.actual_current ? fx(R.actual_current[2], 2) + " A" : "-";
  twin.live(q, tq, g.connected ? g.closure : null);
}

function header(m) {
  const L = m.link || {}, R = m.robot || {};
  const linkOk = L.enabled && L.connected && L.source === "rtde";
  setLamp("#lpLink", !L.enabled ? "" : linkOk ? ((L.requested_hz || 0) >= 499 ? "ok" : "warn") : "bad",
    !L.enabled ? "off" : `${(L.source || "no data").toUpperCase()} ${L.rate_hz ? Math.round(L.rate_hz) + " Hz" : ""}`, L.last_error || "");
  const mode = R.robot_mode_text, saf = R.safety_mode_text;
  setLamp("#lpArm", !mode ? "" : mode === "RUNNING" && saf === "NORMAL" ? "ok" : saf && saf !== "NORMAL" ? "bad" : "warn", mode ? `${mode} / ${saf || "?"}` : "-");
  const g = m.gripper || {};
  setLamp("#lpGrip", !g.connected ? "" : g.fault ? "bad" : g.activated ? "ok" : "warn", !g.connected ? "off" : g.activated ? `${fx(g.width_mm, 0)} mm${g.object ? " holding" : ""}` : "not active");
  const cams = Object.entries(m.cams || {});
  setLamp("#lpCam", !cams.length ? "" : cams.some(([, c]) => c.error) ? "warn" : "ok", cams.length ? cams.map(([n, c]) => `${n}${c.streaming ? " " + Math.round(c.fps) : ""}`).join(" ") : "off");
  const imus = Object.values(m.imu || {});
  setLamp("#lpImu", !imus.length ? "" : imus.every((u) => u.alive && !u.error) ? "ok" : "bad", imus.length ? imus.map((u) => `${u.name} ${u.rate_hz ? Math.round(u.rate_hz) : "-"}`).join(", ") : "off");
  const se = m.session;
  setLamp("#lpRec", se && se.state === "recording" ? "rec" : se && se.qc ? (se.qc === "PASS" ? "ok" : "warn") : "",
    se && se.state === "recording" ? `REC ${fx(se.elapsed_s, 0)} s` : se ? `last ${se.qc || se.state}` : "idle");
  // run bar
  const c = m.campaign, ex = m.exec;
  $("#runBar").classList.toggle("busy", !!m.busy);
  $("#rbState").textContent = m.busy ? (m.cal_busy ? "calibration moves" : "executing") : se && se.state === "recording" ? "recording" : "idle";
  $("#rbJob").textContent = c && c.state !== "ready" ? `${c.name} (${c.state})` : ex ? `${ex.cmd} x${ex.speed}` : m.cal_busy ? "hand-eye poses" : "-";
  const done = c ? c.steps.filter((s) => s.state === "done" || s.state === "refused").length : 0;
  $("#rbStep").textContent = c ? `${Math.min(c.index + 1, c.steps.length)} of ${c.steps.length}` : m.cal_msg || "-";
  $("#rbFill").style.width = c && c.steps.length ? (100 * done / c.steps.length).toFixed(0) + "%" : "0";
  $("#rbTime").textContent = ex && ex.running ? fx(ex.elapsed_s, 0) + " s" : se && se.state === "recording" ? fx(se.elapsed_s, 0) + " s" : "0 s";
  $("#rbRec").textContent = se && se.state === "recording" ? se.run_id : "no";
  $("#busyBanner").classList.toggle("on", !!m.busy);
  $("#busyTxt").textContent = ex && ex.running ? `${ex.cmd} at x${ex.speed}, ${fx(ex.elapsed_s, 0)} s` : m.cal_msg || "";
}

onState((m) => {
  header(m); monitorRender(m);
  const pg = PAGES[S.page]; if (pg && pg.onState) pg.onState(m);
  Object.entries(PAGES).forEach(([k, p]) => { if (k !== S.page && p.always) p.always(m); });
});

/* ------------------------------------------------------------------ start */
Object.entries(PAGES).forEach(([k, p]) => p.init($("#pg-" + k)));
initJog();
twin.load();
show(location.hash.slice(1) || "overview");
connect();
requestAnimationFrame(chartLoop);

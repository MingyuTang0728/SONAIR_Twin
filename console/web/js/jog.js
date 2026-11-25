/* Moving the robot by hand: the two pads (on the Robot page and in the dock), the keyboard and a gamepad.
   Every source writes into ONE velocity, and one timer sends it: jog_vel every 100 ms with a 400 ms time to live,
   so the platform's own watchdog stops the arm if this page stalls, loses focus or loses the socket. Releasing
   everything sends jog_stop. The pads and keys are the ones of the cell console the operators already use. */
import { $, $$, S, send, req, toast, rotvecToMat, matVec, store, fx } from "./core.js";

const J = {
  frame: store.get("jog.frame", "base"), speed: store.get("jog.speed", 40), keys: false,
  src: { pad_xy: [0, 0], pad_zr: [0, 0], key: [0, 0, 0, 0], gp: [0, 0, 0, 0] },
  timer: null, moving: false, refusedAt: 0, gamepad: null, held: new Set(), slow: false,
};
export const jogState = J;

function shape(v) { return Math.sign(v) * Math.pow(Math.abs(v), 1.6); }

function vector() {
  // x, y, z (unit -1..1) and rz from every source, the strongest of each wins
  const pick = (...vals) => vals.reduce((a, b) => (Math.abs(b) > Math.abs(a) ? b : a), 0);
  const x = pick(J.src.pad_xy[0], J.src.key[0], J.src.gp[0]);
  const y = pick(J.src.pad_xy[1], J.src.key[1], J.src.gp[1]);
  const z = pick(J.src.pad_zr[1], J.src.key[2], J.src.gp[2]);
  const r = pick(J.src.pad_zr[0], J.src.key[3], J.src.gp[3]);
  return [x, y, z, r];
}

function xdFromUnit(u) {
  const lin = (J.speed * (J.slow ? 0.25 : 1)) / 1000;                  // m/s
  const ang = Math.min(30, J.speed * 0.25) * (J.slow ? 0.25 : 1) * Math.PI / 180;   // rad/s
  let v = [shape(u[0]) * lin, shape(u[1]) * lin, shape(u[2]) * lin], w = [0, 0, shape(u[3]) * ang];
  if (J.frame === "tool") {
    const p = (S.st && S.st.robot && S.st.robot.actual_TCP_pose) || null;
    if (!p) return null;
    const R = rotvecToMat(p.slice(3, 6)); v = matVec(R, v); w = matVec(R, w);
  }
  return [...v, ...w];
}

async function tick() {
  const u = vector();
  const active = u.some((x) => Math.abs(x) > 1e-3);
  if (!active) { stopTimer(); return; }
  const xd = xdFromUnit(u);
  if (!xd) { toast("tool axes need the robot's tool pose: connect the robot", "bad"); release(); return; }
  const r = await req({ type: "jog_vel", xd, ttl_ms: 400 }, 2000);
  if (r && r.ok === false) {
    if (Date.now() - J.refusedAt > 3000) toast(r.msg || r.error || "jog refused", "bad");
    J.refusedAt = Date.now(); release();
  }
}
function ensureTimer() {
  if (J.timer) return;
  J.moving = true; tick(); J.timer = setInterval(tick, 100); paintDot();
}
function stopTimer() {
  if (J.timer) { clearInterval(J.timer); J.timer = null; send({ type: "jog_stop" }); }
  J.moving = false; paintDot();
}
export function release() {
  J.src.pad_xy = [0, 0]; J.src.pad_zr = [0, 0]; J.src.key = [0, 0, 0, 0]; J.src.gp = [0, 0, 0, 0]; J.held.clear();
  $$(".disc").forEach((d) => { d.classList.remove("on"); const k = $(".knob", d); if (k) k.style.transform = ""; });
  stopTimer();
}
function paintDot() { const d = $("#dockDot"); if (d) d.classList.toggle("on", J.moving); }

/* ------------------------------------------------------------------ pads */
export function bindPad(disc) {
  const which = disc.dataset.pad, knob = $(".knob", disc);
  let id = null;
  const move = (e) => {
    const r = disc.getBoundingClientRect(), R = r.width / 2 - 14;
    let dx = e.clientX - (r.left + r.width / 2), dy = e.clientY - (r.top + r.height / 2);
    const d = Math.hypot(dx, dy); if (d > R) { dx *= R / d; dy *= R / d; }
    knob.style.transform = `translate(${dx}px, ${dy}px)`;
    const ux = dx / R, uy = -dy / R;
    J.src[which === "xy" ? "pad_xy" : "pad_zr"] = [Math.abs(ux) < 0.08 ? 0 : ux, Math.abs(uy) < 0.08 ? 0 : uy];
    ensureTimer();
  };
  disc.addEventListener("pointerdown", (e) => {
    if (S.st && S.st.busy) { toast("refused: a checked command is executing (STOP to take over)", "bad"); return; }
    e.preventDefault(); id = e.pointerId; disc.setPointerCapture(id); disc.classList.add("on"); move(e);
  });
  disc.addEventListener("pointermove", (e) => { if (e.pointerId === id) move(e); });
  const up = () => { id = null; disc.classList.remove("on"); knob.style.transform = ""; J.src[which === "xy" ? "pad_xy" : "pad_zr"] = [0, 0]; if (!vector().some((x) => x)) stopTimer(); };
  ["pointerup", "pointercancel", "lostpointercapture"].forEach((ev) => disc.addEventListener(ev, up));
}

/* ------------------------------------------------------------------ keyboard */
const KEYMAP = { ArrowLeft: [0, -1], ArrowRight: [0, 1], ArrowUp: [1, 1], ArrowDown: [1, -1], q: [2, -1], e: [2, 1], a: [3, 1], d: [3, -1] };
function keyVec() {
  const k = [0, 0, 0, 0];
  J.held.forEach((key) => { const m = KEYMAP[key]; if (m) k[m[0]] = m[1]; });
  J.src.key = k;
}
document.addEventListener("keydown", (e) => {
  if (!J.keys || /INPUT|SELECT|TEXTAREA/.test(document.activeElement.tagName)) return;
  const key = e.key.length === 1 ? e.key.toLowerCase() : e.key;
  if (key === " ") { e.preventDefault(); release(); send({ type: "jog_halt" }); return; }
  if (e.key === "Shift") { J.slow = true; return; }
  if (!KEYMAP[key]) return;
  e.preventDefault();
  if (S.st && S.st.busy) { if (Date.now() - J.refusedAt > 3000) toast("refused: a checked command is executing", "bad"); J.refusedAt = Date.now(); return; }
  J.held.add(key); keyVec(); ensureTimer();
});
document.addEventListener("keyup", (e) => {
  const key = e.key.length === 1 ? e.key.toLowerCase() : e.key;
  if (e.key === "Shift") { J.slow = false; return; }
  if (J.held.delete(key)) { keyVec(); if (!vector().some((x) => x)) stopTimer(); }
});
window.addEventListener("blur", release);
document.addEventListener("visibilitychange", () => document.hidden && release());

/* ------------------------------------------------------------------ gamepad */
window.addEventListener("gamepadconnected", (e) => { J.gamepad = e.gamepad.index; paintGp(); });
window.addEventListener("gamepaddisconnected", () => { J.gamepad = null; J.src.gp = [0, 0, 0, 0]; paintGp(); });
function paintGp() {
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  const gp = J.gamepad !== null ? pads[J.gamepad] : null;
  $$(".gp").forEach((el) => { el.classList.toggle("on", !!gp); el.textContent = gp ? `Gamepad: ${gp.id.slice(0, 48)}. Left stick: across the table, right stick: up and rotate.` : "No gamepad. Plug one in and press a button."; });
}
function pollGamepad() {
  if (J.gamepad !== null && navigator.getGamepads) {
    const gp = navigator.getGamepads()[J.gamepad];
    if (gp) {
      const dz = (v) => (Math.abs(v) < 0.15 ? 0 : (v - Math.sign(v) * 0.15) / 0.85);
      const v = [dz(gp.axes[0] || 0), -dz(gp.axes[1] || 0), -dz(gp.axes[3] || 0), -dz(gp.axes[2] || 0)];
      const was = J.src.gp.some((x) => x);
      J.src.gp = v;
      if (v.some((x) => x)) { if (!(S.st && S.st.busy)) ensureTimer(); }
      else if (was && !vector().some((x) => x)) stopTimer();
    }
  }
  requestAnimationFrame(pollGamepad);
}

/* ------------------------------------------------------------------ shared controls (frame and speed, in both places) */
export function setFrame(f) {
  J.frame = f; store.set("jog.frame", f);
  $$("[data-frame]").forEach((b) => b.classList.toggle("on", b.dataset.frame === f));
  $$(".disc .lab").forEach((l) => { const t = f === "tool" ? l.dataset.tool : l.dataset.base; if (t) l.textContent = t; });
}
export function setSpeed(v) {
  J.speed = Math.max(5, Math.min(150, +v || 40)); store.set("jog.speed", J.speed);
  $$("#jogSpeed, #dockSpeed").forEach((el) => { if (+el.value !== J.speed) el.value = J.speed; });
  $$("#jogSpeedV, #dockSpeedV").forEach((el) => (el.textContent = J.speed + " mm/s"));
}
export function setKeys(on) { J.keys = on; $$("#jogKeys").forEach((c) => (c.checked = on)); }

export function step(axis, amount) {
  return req({ type: "jog_step", axis, distance_mm: amount, frame: J.frame, speed: 0.05 }, 20000);
}

export function initJog() {
  $$(".disc").forEach(bindPad);
  document.addEventListener("click", (e) => { const b = e.target.closest("[data-frame]"); if (b) setFrame(b.dataset.frame); });
  document.addEventListener("input", (e) => { if (e.target.id === "jogSpeed" || e.target.id === "dockSpeed") setSpeed(e.target.value); });
  $("#dockStop").onclick = () => { release(); send({ type: "jog_halt" }); };
  setFrame(J.frame); setSpeed(J.speed); paintGp(); requestAnimationFrame(pollGamepad);
  // the dock shows where the tool is
  setInterval(() => { const p = S.st && S.st.robot && S.st.robot.actual_TCP_pose; if (p) { $("#dockX").textContent = "X " + fx(p[0] * 1e3, 1); $("#dockY").textContent = "Y " + fx(p[1] * 1e3, 1); $("#dockZ").textContent = "Z " + fx(p[2] * 1e3, 1); } }, 250);
}

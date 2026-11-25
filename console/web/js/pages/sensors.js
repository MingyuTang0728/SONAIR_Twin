/* Motion sensors: the LSM6DSV16X units, read through their FIFO with their own timestamps; live readings, every
   axis, and the still bench check (gravity, gyro bias and noise, both clock maps). */
import { $, $$, esc, fx, req, result, panel, pageHead, icon, Chart, SERIES, S } from "../core.js";

let el, units = [{ name: "tool", bus: 1, addr: "0x6b" }], gyro, acc, norm, lastT = 0;

export default {
  init(root) {
    el = root;
    el.innerHTML = pageHead("Cell", "Motion sensors", "LSM6DSV16X inertial units on the Jetson's I2C bus, read through their FIFO with the sensor's own timestamps. Each stream keeps its own clock; a session maps it to the host clock afterwards.", `<button class="btn sm" data-dock="move">Move</button><button class="btn sm" data-dock="rec">Session</button>`) + `
      <div class="grid" style="grid-template-columns:minmax(300px,1fr) minmax(0,2fr);gap:12px;margin-bottom:12px">
        ${panel("Units", "imu", `<div class="tbox" style="max-height:none"><table class="t"><thead><tr><th>Name</th><th>I2C bus</th><th>Address</th><th></th></tr></thead><tbody id="iUnits"></tbody></table></div>
          <div class="row" style="margin-top:8px"><button class="btn sm" id="iAdd">Add unit</button><span class="sp"></span>
            <label class="field"><span>Output rate</span><select id="iOdr"><option value="480">480 Hz</option><option value="960">960 Hz</option><option value="240">240 Hz</option></select></label></div>
          <div class="row" style="margin-top:10px"><button class="btn pri" id="iStart">Start</button><button class="btn" id="iStop">Stop</button></div>
          <div class="note" id="iMsg" style="margin-top:10px">Two addresses per board (0x6A, 0x6B); a 400 kHz bus carries two or three units at 480 Hz. Along the arm the bus is differential (QwiicBus).</div>`)}
        ${panel("Live", "rtde", `<div class="tbox" style="max-height:none"><table class="t"><thead><tr><th>Unit</th><th>Kind</th><th class="num">Rate (Hz)</th><th class="num">|accel| (m/s2)</th>
          <th class="num">gyro x</th><th class="num">gyro y</th><th class="num">gyro z (rad/s)</th><th class="num">Samples</th><th>Error</th></tr></thead><tbody id="iLive"></tbody></table></div>
          <div class="grid g3" style="margin-top:12px">
            <div><div class="lbl-s">Gyroscope (rad/s)</div><div class="legend" id="lgG"></div><div id="chG"></div></div>
            <div><div class="lbl-s">Accelerometer (m/s2)</div><div class="legend" id="lgA"></div><div id="chA"></div></div>
            <div><div class="lbl-s">|gyro| (rad/s)</div><div class="legend" id="lgN"></div><div id="chN"></div></div>
          </div><div class="muted small" style="margin-top:6px">First unit, on its own clock, last 20 s (the display thins the stream; sessions keep every sample).</div>`)}
      </div>
      ${panel("Bench check (robot still)", "check", `<div class="row"><label class="field"><span>Seconds</span><input type="number" id="bSecs" class="w80" value="10" min="3" max="60"></label>
          <button class="btn pri" id="bRun" style="align-self:flex-end">Run check</button><span class="muted small" style="align-self:flex-end">Gravity norm, gyro bias and noise per unit, and both clock maps (controller and each IMU to the host). Nothing moves. Run it before each session day.</span></div>
        <div id="bOut" class="t2" style="margin-top:10px">Not run yet.</div>`)}`;
    const c = SERIES();
    gyro = new Chart($("#chG", el), { cls: "short", window: 20, legend: $("#lgG", el), minSpan: 0.02, series: ["x", "y", "z"].map((a, i) => ({ name: a, color: c[i] })) });
    acc = new Chart($("#chA", el), { cls: "short", window: 20, legend: $("#lgA", el), minSpan: 0.2, series: ["x", "y", "z"].map((a, i) => ({ name: a, color: c[i] })) });
    norm = new Chart($("#chN", el), { cls: "short", window: 20, legend: $("#lgN", el), minSpan: 0.02, series: [{ name: "|gyro|", color: c[5] }] });
    renderUnits();
    $("#iAdd", el).onclick = () => { units.push({ name: `unit${units.length + 1}`, bus: 1, addr: "0x6a" }); renderUnits(); };
    $("#iStart", el).onclick = async () => { const r = await req({ type: "imu_start", units, odr: +$("#iOdr", el).value }, 20000); $("#iMsg", el).textContent = r.ok ? `streaming: ${r.units.join(", ")}` : (r.errors || [r.error]).join("; "); result(r, "IMU streaming"); };
    $("#iStop", el).onclick = async () => result(await req({ type: "imu_stop" }), "IMU stopped");
    $("#bRun", el).onclick = bench;
  },
  always(m) { push(m); },
  onState(m) {
    push(m);
    const imus = Object.values(m.imu || {});
    $("#iLive", el).innerHTML = imus.map((u) => { const l = u.latest || {}, a = l.accel || [], g = l.gyro || [];
      return `<tr><td>${esc(u.name)}</td><td>${esc(u.kind)}</td><td class="num">${fx(u.rate_hz, 1)}</td><td class="num">${a.length ? fx(Math.hypot(...a), 3) : "-"}</td>
        <td class="num">${fx(g[0], 4)}</td><td class="num">${fx(g[1], 4)}</td><td class="num">${fx(g[2], 4)}</td><td class="num">${u.samples ?? "-"}</td><td class="small">${esc(u.error || "")}</td></tr>`; }).join("")
      || `<tr><td colspan="9" class="muted">No unit streaming. On the Jetson the units are on I2C; in mock mode a simulated unit follows the robot.</td></tr>`;
  },
};

function push(m) {
  const u = Object.values(m.imu || {})[0]; if (!u || !u.latest || !u.latest.gyro) return;
  const t = u.latest.t_dev; if (t === undefined || t <= lastT) return; lastT = t;
  const g = u.latest.gyro, a = u.latest.accel;
  gyro.push(t, g); acc.push(t, a); norm.push(t, [Math.hypot(...g)]);
}

function renderUnits() {
  $("#iUnits", el).innerHTML = units.map((u, i) => `<tr><td><input type="text" data-i="${i}" data-k="name" value="${esc(u.name)}" class="w100"></td>
    <td><input type="number" data-i="${i}" data-k="bus" value="${u.bus}" class="w60"></td><td><input type="text" data-i="${i}" data-k="addr" value="${esc(u.addr)}" class="w80"></td>
    <td><button class="btn sm ghost" data-del="${i}">remove</button></td></tr>`).join("");
  $$("#iUnits input", el).forEach((x) => (x.onchange = () => (units[+x.dataset.i][x.dataset.k] = x.dataset.k === "bus" ? parseInt(x.value, 10) : x.value)));
  $$("#iUnits [data-del]", el).forEach((b) => (b.onclick = () => { units.splice(+b.dataset.del, 1); renderUnits(); }));
}

async function bench() {
  const secs = +$("#bSecs", el).value || 10;
  $("#bOut", el).textContent = `Recording ${secs} s with the robot still...`;
  const r = await req({ type: "bench_check", seconds: secs }, (secs + 30) * 1000);
  if (r.ok === false) { $("#bOut", el).textContent = r.error; return; }
  const rc = (r.robot || {}).clock || {};
  let h = `<div class="tbox" style="max-height:none"><table class="t"><thead><tr><th>Stream</th><th class="num">Samples</th><th class="num">Rate (Hz)</th><th class="num">|g| (m/s2)</th><th class="num">Gyro bias (deg/s)</th><th class="num">Gyro noise (deg/s)</th><th class="num">Clock drift (ppm)</th><th class="num">Delay p95 (ms)</th><th>Verdict</th></tr></thead><tbody>`;
  h += `<tr><td>robot (RTDE)</td><td class="num">${(r.robot || {}).packets ?? "-"}</td><td class="num">-</td><td class="num">-</td><td class="num">-</td><td class="num">-</td><td class="num">${fx(rc.drift_ppm, 1)}</td><td class="num">${fx(rc.delay_above_envelope_p95_ms, 2)}</td><td>${r.robot && r.robot.still ? '<span class="badge ok">still</span>' : '<span class="badge FAIL">moved</span>'}</td></tr>`;
  Object.entries(r.imu || {}).forEach(([n, u]) => {
    if (u.error) { h += `<tr><td>${esc(n)}</td><td colspan="8">${esc(u.error)}</td></tr>`; return; }
    const c = u.clock || {};
    h += `<tr><td>${esc(n)}</td><td class="num">${u.samples}</td><td class="num">${fx(u.rate_hz, 2)}</td><td class="num">${fx(u.gravity_norm, 4)}</td><td class="num">${u.gyro_bias_dps.map((v) => fx(v, 3)).join(" ")}</td>
      <td class="num">${u.gyro_noise_dps.map((v) => fx(v, 3)).join(" ")}</td><td class="num">${fx(c.drift_ppm, 1)}</td><td class="num">${fx(c.delay_above_envelope_p95_ms, 2)}</td><td><span class="badge ${u.verdict}">${u.verdict}</span></td></tr>`;
  });
  $("#bOut", el).innerHTML = h + "</tbody></table></div>" + (r.note ? `<div class="note warn" style="margin-top:8px">${esc(r.note)}</div>` : "");
}

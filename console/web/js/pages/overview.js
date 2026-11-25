/* Overview: is the cell ready to record, and what every device is doing. */
import { $, esc, fx, req, panel, pageHead, icon, S, onLog } from "../core.js";

let el, dataCounted = false;

function devTile(id, ic, name) {
  return `<div class="tile" id="${id}"><div class="row" style="gap:8px">${icon(ic)}<span class="k">${name}</span><span class="sp"></span><span class="badge" data-b>-</span></div>
    <div class="v" style="font-size:16px" data-v>-</div><div class="d" data-d></div></div>`;
}
function setDev(id, cls, badge, v, d) {
  const t = $("#" + id, el); if (!t) return;
  t.className = "tile " + (cls || ""); const b = $("[data-b]", t); b.className = "badge " + (cls || ""); b.textContent = badge;
  $("[data-v]", t).textContent = v; $("[data-d]", t).textContent = d || "";
}

export default {
  init(root) {
    el = root;
    el.innerHTML = pageHead("Cell", "Overview", "Whether the cell is fit to record: every device, the checks a recording needs, the robot link, the current plan and what has been recorded.",
      `<a class="btn" href="#robot">${icon("arm")}Robot</a><a class="btn" href="#calibrate">${icon("target")}Calibrate</a><a class="btn pri" href="#record">${icon("rec")}Record a session</a>`) + `
      <div class="grid g6" style="margin-bottom:12px">
        ${devTile("dvLink", "rtde", "Robot link")}${devTile("dvArm", "arm", "Arm")}${devTile("dvGrip", "grip", "Gripper 2F-85")}
        ${devTile("dvCam", "camera", "Cameras")}${devTile("dvImu", "imu", "IMUs")}${devTile("dvRec", "rec", "Session")}
      </div>
      <div class="grid g2" style="align-items:start">
        <div class="stack">
          ${panel("Robot link", "link", `<div class="row"><label class="field grow"><span>Controller address</span><input type="text" id="ovHost" placeholder="192.168.0.20"></label>
            <label class="field"><span>Rate</span><select id="ovHz"><option value="500">500 Hz (record)</option><option value="125">125 Hz (view only)</option></select></label>
            <button class="btn pri" id="ovConnect" style="align-self:flex-end">Connect</button></div>
            <div class="note" style="margin-top:10px">One address for every channel to the robot: RTDE state, URScript motion, the dashboard and the gripper socket. Simulator first, then the robot. The pendant must be in <b>Remote Control</b>.</div>`)}
          ${panel("Ready to record", "check", `<ul class="checks" id="ovChecks"></ul>`, `<span class="tag" id="ovReady">-</span>`)}
        </div>
        <div class="stack">
          ${panel("Campaign", "campaign", `<div id="ovCamp" class="t2">No plan loaded.</div>`, `<a class="btn sm ghost" href="#campaign">Open</a>`)}
          ${panel("Data", "data", `<dl class="kv" id="ovData"><dt>loading</dt><dd>-</dd></dl>`, `<a class="btn sm ghost" href="#data">Open</a>`)}
          ${panel("Recent events", "log", `<div class="log" id="ovLog" style="max-height:260px"></div>`, `<a class="btn sm ghost" href="#system">Full log</a>`)}
        </div>
      </div>`;
    $("#ovConnect", el).onclick = async () => {
      const host = $("#ovHost", el).value.trim(); if (!host) return;
      const { result } = await import("../core.js");
      result(await req({ type: "ur_service_start", host, frequency: +$("#ovHz", el).value }, 30000), "robot link started");
    };
    onLog((e) => {
      const L = $("#ovLog", el); const d = document.createElement("div"); d.className = e.level || "";
      d.innerHTML = `<span class="ts">${esc(e.t)}</span>${esc(e.msg)}`; L.appendChild(d);
      while (L.children.length > 60) L.firstChild.remove(); L.scrollTop = L.scrollHeight;
    });
  },
  onShow() { if (S.hello && !$("#ovHost", el).value) $("#ovHost", el).value = S.hello.robot_host || ""; if (S.hello) countData(); },
  always(m) { if (!dataCounted && S.hello) countData(); },
  // (counted once the socket is up: a request sent before that answers nothing)
  onState(m) {
    if (!dataCounted) countData();
    if (S.hello && !$("#ovLog", el).children.length) (S.hello.log || []).slice(-40).forEach((e) => { const d = document.createElement("div"); d.className = e.level || ""; d.innerHTML = `<span class="ts">${esc(e.t)}</span>${esc(e.msg)}`; $("#ovLog", el).appendChild(d); });
    const L = m.link || {}, R = m.robot || {}, g = m.gripper || {}, se = m.session;
    setDev("dvLink", !L.enabled ? "" : L.connected && L.source === "rtde" ? ((L.requested_hz || 0) >= 499 ? "ok" : "warn") : "bad", !L.enabled ? "off" : L.connected ? (L.source || "?").toUpperCase() : "connecting",
      L.rate_hz ? `${fx(L.rate_hz, 0)} Hz` : "-", L.enabled ? `${L.host || ""} ${L.controller_version ? "controller " + L.controller_version : ""}` : "not started");
    setDev("dvArm", !R.robot_mode_text ? "" : R.robot_mode_text === "RUNNING" && R.safety_mode_text === "NORMAL" ? "ok" : R.safety_mode_text !== "NORMAL" ? "bad" : "warn",
      R.robot_mode_text || "-", R.safety_mode_text || "-", R.runtime_state_text ? `program ${R.runtime_state_text}, slider ${fx((R.speed_scaling ?? 0) * 100, 0)} %` : "");
    setDev("dvGrip", !g.connected ? "" : g.activated ? "ok" : "warn", !g.connected ? "off" : g.activated ? "active" : "reset", g.connected ? `${fx(g.width_mm, 1)} mm` : "-",
      g.connected ? `position ${g.pos} of 255${g.object ? ", holding an object" : ""}` : "Robotiq URCap socket, port 63352");
    const cams = Object.entries(m.cams || {});
    setDev("dvCam", !cams.length ? "" : "ok", cams.length ? (cams[0][1].kind === "mock" ? "mock" : "on") : "off", cams.length ? cams.map(([n, c]) => n).join(" + ") : "-",
      cams.length ? cams.map(([n, c]) => `${n} ${c.streaming ? fx(c.fps, 0) + " fps" : "idle"}`).join(", ") : "start them on the Cameras page");
    const imus = Object.values(m.imu || {});
    setDev("dvImu", !imus.length ? "" : imus.every((u) => u.alive && !u.error) ? "ok" : "bad", imus.length ? imus[0].kind : "off", imus.length ? imus.map((u) => u.name).join(", ") : "-",
      imus.map((u) => `${u.name} ${u.rate_hz ? fx(u.rate_hz, 0) + " Hz" : "-"}`).join(", ") || "LSM6DSV16X over I2C");
    setDev("dvRec", se && se.state === "recording" ? "bad" : se && se.qc === "PASS" ? "ok" : se && se.qc ? "warn" : "", se ? (se.state === "recording" ? "REC" : se.qc || se.state) : "idle",
      se ? (se.state === "recording" ? fx(se.elapsed_s, 0) + " s" : se.qc || "-") : "-", se ? se.run_id : "no session yet");
    const imuOk = imus.length > 0 && imus.every((u) => u.alive && !u.error);
    const checks = [
      ["Robot link started", L.enabled], ["Robot data over RTDE (no 30003 fallback)", L.enabled && L.source === "rtde"], ["Link at 500 Hz", (L.requested_hz || 0) >= 499],
      ["Robot RUNNING", R.robot_mode_text === "RUNNING"], ["Safety NORMAL", R.safety_mode_text === "NORMAL"], ["Speed slider at 100 %", (R.speed_scaling ?? 0) > 0.999, true],
      ["Inertial units streaming", imuOk, true], ["Gripper active", !!(g.connected && g.activated), true], ["Nothing executing", !m.busy], ["Platform robot link unchanged (hashes)", m.locked_ok],
    ];
    $("#ovChecks", el).innerHTML = checks.map(([t, ok, soft]) => `<li class="${ok ? "ok" : soft ? "wn" : "no"}"><span class="mk">${ok ? "&#10003;" : soft ? "!" : "&#10005;"}</span><span>${esc(t)}</span></li>`).join("")
      + (m.mock ? `<li class="wn"><span class="mk">!</span><span>Mock mode: sessions are labelled mock and never enter a dataset</span></li>` : "");
    const hard = checks.filter((c) => !c[2] && !c[1]).length;
    $("#ovReady", el).className = "tag " + (hard ? "bad" : "ok"); $("#ovReady", el).textContent = hard ? `${hard} blocking` : "ready";
    const c = m.campaign;
    $("#ovCamp", el).innerHTML = c ? `<b>${esc(c.name)}</b>: ${c.steps.filter((s) => s.state === "done").length} of ${c.steps.length} steps done, <span class="badge">${esc(c.state)}</span>
      <div class="prog" style="margin-top:8px"><i style="width:${(100 * c.steps.filter((s) => s.state === "done").length / Math.max(1, c.steps.length)).toFixed(0)}%"></i></div>` : "No plan loaded.";
  },
};

async function countData() {
  dataCounted = true;
  const [a, b] = await Promise.all([req({ type: "data_sessions" }), req({ type: "data_bench" })]);
  const runs = b.runs || [], ses = a.sessions || [];
  const tr = runs.filter((r) => r.split === "train").length;
  const h = runs.reduce((x, r) => x + (+r.duration_s || 0), 0) / 3600;
  $("#ovData", el).innerHTML = `<dt>Sessions recorded here</dt><dd>${ses.length} (${ses.filter((s) => s.verdict === "PASS" && !s.mock).length} PASS, ${ses.filter((s) => s.mock).length} mock)</dd>
    <dt>benchmark-v1 runs</dt><dd>${runs.length} (${tr} published, ${runs.length - tr} held-out inputs)</dd><dt>benchmark-v1 motion</dt><dd>${fx(h, 2)} h</dd>`;
}

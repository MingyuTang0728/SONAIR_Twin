/* Record: one session. The robot at every controller cycle and every IMU on its own clock, a checked command through
   the executor or a passive recording, then QC and export to the platform's run format. */
import { $, esc, fx, req, result, confirmBox, panel, pageHead, icon, S, MOVE_CHECKS } from "../core.js";

let el, lastId = null;

export default {
  init(root) {
    el = root;
    el.innerHTML = pageHead("Experiment", "Record a session", "A session records the robot at every controller cycle (500 Hz RTDE) and every inertial unit on its own clock, then runs QC. A command moves the robot only through the executor: half speed by default, refused above 0.6 rad/s or if it failed the collision check.") + `
      <div class="grid" style="grid-template-columns:minmax(360px,1fr) minmax(0,1.3fr);gap:12px;align-items:start">
        ${panel("What this run is", "rec", `
          <div class="row"><label class="field grow"><span>Motion</span><select id="sMotion"><option value="executor">Checked command (executor, servoJ)</option><option value="passive">Passive (pendant program or jog)</option></select></label></div>
          <div class="row" id="sCmdRow"><label class="field grow"><span>Command</span><select id="sCmd"></select></label><label class="field"><span>Speed</span><input type="number" id="sSpeed" class="w80" value="0.5" step="0.05" min="0.05" max="1"></label></div>
          <div class="small t2" id="sCmdInfo" style="margin-top:6px"></div>
          <h3 class="sub" style="margin-top:14px">Condition cell (E2 grid)</h3>
          <div class="row"><label class="field"><span>Elbow speed (rad/s)</span><select id="sV"><option value="">none</option>${["0.2", "0.4", "0.5", "0.6", "0.7", "0.9"].map((v) => `<option>${v}</option>`).join("")}</select></label>
            <label class="field grow"><span>Arm configuration</span><select id="sArm"><option value="">none</option><option>near_singular</option><option>mid_workspace</option><option>extended</option></select></label>
            <label class="field grow"><span>Trajectory</span><select id="sTraj"><option value="">none</option><option>contour</option><option>point_to_point</option><option>stop_start</option></select></label></div>
          <div class="row"><label class="field"><span>Repeat</span><input type="number" id="sRep" class="w60" value="0" min="0"></label>
            <label class="field grow"><span>Payload on the pendant</span><select id="sPayload"></select></label>
            <label class="field"><span>Experiment</span><select id="sExp"><option>E2</option><option>E1</option><option>E3</option><option>CAL</option><option>PILOT</option></select></label></div>
          <div class="row"><label class="field"><span>Operator</span><input type="text" id="sOp" class="w100" placeholder="initials"></label><label class="field grow"><span>Notes</span><input type="text" id="sNotes"></label></div>
          <div class="row" style="margin-top:12px"><button class="btn pri lg" id="sStart">${icon("rec", "i sm")}Start recording</button><button class="btn danger lg" id="sStop">Stop</button></div>
          <div class="note" id="sMsg" style="margin-top:10px">The executor first moves slowly to the command's start pose. Calibration files in use are recorded in the manifest.</div>`)}
        <div class="stack">
          ${panel("Now", "gauge", `<div class="grid g4" id="sTiles"></div><dl class="kv" id="sNow" style="margin-top:12px"></dl>`, `<span class="tag" id="sState">idle</span>`)}
          ${panel("Executor", "log", `<div class="log" id="sExec" style="max-height:160px"></div>`)}
          ${panel("Last result", "check", `<div id="sLast" class="t2">Nothing recorded in this console run yet.</div>`)}
        </div>
      </div>`;
    $("#sCmd", el).onchange = cmdInfo; $("#sSpeed", el).oninput = cmdInfo;
    $("#sPayload", el).onchange = () => ($("#sPayload", el).dataset.touched = "1");
    $("#sMotion", el).onchange = () => ($("#sCmdRow", el).style.display = $("#sMotion", el).value === "executor" ? "" : "none");
    $("#sStart", el).onclick = start;
    $("#sStop", el).onclick = async () => { const r = await req({ type: "session_stop" }, 60000); showLast(r); result(r, "session stopped"); };
  },
  onShow() {
    const h = S.hello || {};
    if (!$("#sCmd", el).options.length) {
      $("#sCmd", el).innerHTML = (h.commands || []).map((c) => `<option value="${esc(c.id)}">${esc(c.id)}</option>`).join("");
      $("#sPayload", el).innerHTML = `<option value="">not set</option>` + Object.entries(h.payloads || {}).map(([k, v]) => `<option value="${esc(k)}">${esc(k)}: ${esc(v.label || "")}</option>`).join("");
      cmdInfo();
    }
  },
  always(m) { const se = m.session; if (se && se.state === "done" && se.run_id !== lastId) { lastId = se.run_id; showLast({ run_id: se.run_id, qc: { verdict: se.qc, reasons: se.reasons } }); } },
  onState(m) {
    this.always(m);
    const se = m.session;
    $("#sState", el).textContent = se ? se.state : "idle"; $("#sState", el).className = "tag " + (se && se.state === "recording" ? "bad" : "");
    const t = (k, v, d, cls = "") => `<div class="tile sm ${cls}"><div class="k">${k}</div><div class="v">${v}</div><div class="d">${d}</div></div>`;
    const src = se ? Object.entries(se.sources || {}) : [];
    $("#sTiles", el).innerHTML = t("Elapsed", se ? fx(se.elapsed_s, 1) + "<small>s</small>" : "-", "") + t("Robot rows", se ? se.rows ?? "-" : "-", "one per controller cycle")
      + t("Packet source", src.length ? src.map(([k]) => k).join(" + ") : "-", src.map(([k, v]) => `${k} ${v}`).join(", "), src.length === 1 && src[0][0] === "rtde" ? "ok" : src.length ? "bad" : "")
      + t("IMU units", Object.keys(m.imu || {}).length, Object.keys(m.imu || {}).join(", "));
    $("#sNow", el).innerHTML = se ? `<dt>Session</dt><dd>${esc(se.run_id)}</dd><dt>Command</dt><dd>${esc(se.meta.cmd || "passive")} ${se.meta.speed ? "x" + se.meta.speed : ""}</dd>
      <dt>Cell</dt><dd>${esc(se.meta.cell || "-")}</dd><dt>Payload</dt><dd>${esc(se.meta.payload || "-")}</dd>${se.mock ? "<dt>Mock</dt><dd>yes: never enters a dataset</dd>" : ""}` : `<dt>Session</dt><dd>none</dd>`;
    $("#sExec", el).innerHTML = m.exec ? m.exec.tail.map((l) => `<div>${esc(l)}</div>`).join("") + (m.exec.running ? "" : `<div class="ts">exit code ${m.exec.rc}</div>`) : `<div class="ts">not started</div>`;
  },
};

function cmdInfo() {
  const c = ((S.hello && S.hello.commands) || []).find((x) => x.id === $("#sCmd", el).value), sp = parseFloat($("#sSpeed", el).value) || 0;
  if (!c) { $("#sCmdInfo", el).textContent = ""; return; }
  const peak = (c.peak_qd ?? NaN) * sp, ok = peak <= 0.605;
  $("#sCmdInfo", el).innerHTML = `${esc(c.kind || "")}, payload <b>${esc(c.payload || (c.payload_schedule ? c.payload_schedule.map((x) => x[1]).join(" > ") : "-"))}</b>, ${fx((c.duration_s || 0) / (sp || 1), 1)} s at x${sp},
    peak <b style="color:${ok ? "inherit" : "var(--bad-text)"}">${fx(peak, 2)} rad/s</b>${ok ? "" : " (the executor will refuse: limit 0.6)"}, collision check ${esc(c.collision_check || "-")}`;
  if (c.payload && S.hello.payloads[c.payload] && !$("#sPayload", el).dataset.touched) $("#sPayload", el).value = c.payload;
}

async function start() {
  const motion = $("#sMotion", el).value, v = $("#sV", el).value, arm = $("#sArm", el).value, traj = $("#sTraj", el).value;
  const msg = { type: "session_start", motion, cmd: motion === "executor" ? $("#sCmd", el).value : null, speed: parseFloat($("#sSpeed", el).value),
    cell: v && arm && traj ? `${v}/${arm}/${traj}` : "", rep: parseInt($("#sRep", el).value, 10) || 0, payload: $("#sPayload", el).value || null,
    operator: $("#sOp", el).value.trim(), experiment: $("#sExp", el).value, notes: $("#sNotes", el).value.trim(), payload_on_pendant: $("#sPayload", el).value || "" };
  if (motion === "executor") {
    const c = (S.hello.commands || []).find((x) => x.id === msg.cmd) || {};
    if (!(await confirmBox("Move the robot", `<p><b>${esc(msg.cmd)}</b> at x${msg.speed} (${fx((c.duration_s || 0) / msg.speed, 1)} s, peak ${fx((c.peak_qd || 0) * msg.speed, 2)} rad/s).</p>
      <ul><li>Payload on the pendant: <b>${esc(msg.payload || "not set")}</b></li><li>Speed slider at 100 %</li></ul>${MOVE_CHECKS}`, "Run"))) return;
  }
  const r = await req(msg, 30000); $("#sMsg", el).textContent = r.ok ? (r.run_id || `started ${r.cmd}`) : r.error; result(r, "session started");
}

function showLast(r) {
  if (!r || !r.run_id) return;
  const q = r.qc || {};
  $("#sLast", el).innerHTML = `<div class="row"><span class="badge ${esc(q.verdict || "")}">${esc(q.verdict || "-")}</span><span class="mono small">${esc(r.run_id)}</span><span class="sp"></span>
    <button class="btn sm" id="sExpBtn">Export</button><a class="btn sm" href="#data" id="sPrevBtn">Open in Data</a></div>
    <ul class="small" style="margin:8px 0 0;padding-left:18px">${(q.reasons || []).map((x) => `<li>${esc(x)}</li>`).join("") || "<li>every gate passed</li>"}</ul>`;
  $("#sExpBtn", el).onclick = async () => { const e = await req({ type: "session_export", run_id: r.run_id }, 120000); result(e, e.ok ? `exported: ${e.platform_run.split("/").slice(-3).join("/")} (IMU lag ${e.lag_ms} ms applied)` : ""); };
  $("#sPrevBtn", el).onclick = () => { sessionStorage.setItem("sonair.open", JSON.stringify({ kind: "session", id: r.run_id })); };
}

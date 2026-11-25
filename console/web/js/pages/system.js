/* Log and diagnostics: everything the agent said, the robot link's health, the lock on the platform files, files. */
import { $, esc, fx, req, panel, pageHead, icon, S, onLog, onHello, withToken } from "../core.js";

let el, lines = [];

export default {
  init(root) {
    el = root;
    el.innerHTML = pageHead("System", "Log &amp; diagnostics", "If something stops working, look here first: every action that failed and what the agent said about it.",
      `<button class="btn" id="sysCopy">Copy for a bug report</button><button class="btn" id="sysClear">Clear view</button>`) + `
      <div class="grid" style="grid-template-columns:minmax(0,1.6fr) minmax(300px,1fr);gap:12px;align-items:start">
        ${panel("Agent log", "log", `<div class="row" style="margin-bottom:8px"><input type="search" id="sysF" placeholder="filter" style="flex:1"><label class="chk small"><input type="checkbox" id="sysW"> warnings and errors only</label></div><div class="log" id="sysLog" style="max-height:620px"></div>`)}
        <div class="stack">
          ${panel("Agent", "link", `<dl class="kv" id="sysAgent"></dl>`)}
          ${panel("Platform robot link", "check", `<div id="sysLock"></div>`)}
          ${panel("Safety rules in this console", "power", `<ul class="small t2" style="margin:0;padding-left:18px;line-height:1.65">
            <li>A versioned command moves the robot only through sonair.real.executor: x0.5 by default, refused above 0.6 rad/s, refused if it failed the collision check, safety mode checked every 100 ms.</li>
            <li>While a command or calibration moves run, jog, move, script, freedrive, program play and gripper commands are refused. Stops never are.</li>
            <li>STOP (Esc): campaign, executor, calibration moves, jog, the gripper's motion, then stopj and the dashboard stop; the session is closed with the reason.</li>
            <li>Jog: a velocity every 100 ms with a 400 ms time to live; the platform's watchdog stops the arm if the page goes quiet.</li>
            <li>Nothing in Unity or in the twin view can move the robot.</li></ul>`)}
          ${panel("Files written (not committed)", "file", `<dl class="kv small"><dt>runs_jetson/</dt><dd>sessions</dd><dt>runs/</dt><dd>executor runs</dd><dt>exports/</dt><dd>platform and twin run files</dd>
            <dt>campaigns/</dt><dd>plans with their step results</dd><dt>calibration/</dt><dd>hand-eye, six-face, IMU mount; archive/</dd><dt>captures/</dt><dd>RTDE CSV, camera views</dd></dl>`)}
        </div>
      </div>`;
    onHello((m) => { lines = (m.log || []).slice(); paint(); });
    onLog((e) => { lines.push(e); if (lines.length > 600) lines.shift(); if (el.classList.contains("on")) paint(); });
    $("#sysF", el).oninput = paint; $("#sysW", el).onchange = paint;
    $("#sysClear", el).onclick = () => { lines = []; paint(); };
    $("#sysCopy", el).onclick = async () => {
      const st = S.st || {}, txt = `SONAIR console ${S.hello && S.hello.version}\nlink ${JSON.stringify(st.link)}\nrobot mode ${(st.robot || {}).robot_mode_text} safety ${(st.robot || {}).safety_mode_text}\n\n` + lines.slice(-120).map((l) => `${l.t} ${l.level} ${l.msg}`).join("\n");
      try { await navigator.clipboard.writeText(txt); } catch { /* not allowed on http */ }
      const w = window.open("", "_blank"); if (w) { w.document.write(`<pre>${esc(txt)}</pre>`); }
    };
  },
  onShow() { paint(); },
  onState(m) {
    const L = m.link || {}, h = S.hello || {};
    $("#sysAgent", el).innerHTML = `<dt>Version</dt><dd>${esc(h.version)}</dd><dt>Mode</dt><dd>${h.mock ? "mock: stand-in controller, simulated IMU, rendered cameras" : "cell"}</dd>
      <dt>Robot</dt><dd>${esc(h.robot_host || "-")}</dd><dt>Link</dt><dd>${L.enabled ? `${esc(L.source || "?")} ${fx(L.rate_hz, 1)} Hz, requested ${L.requested_hz}` : "off"}</dd>
      <dt>Network access</dt><dd>${h.token ? "token required (lab network)" : "this computer only"}</dd><dt>Sessions</dt><dd>${esc(h.runs_dir)}</dd>
      ${L.last_error ? `<dt>Last link error</dt><dd>${esc(L.last_error)}</dd>` : ""}${L.degraded ? `<dt>Degraded</dt><dd>${esc(L.degraded)}</dd>` : ""}`;
    $("#sysLock", el).innerHTML = m.locked_ok ? `<div class="note ok"><b>Unchanged.</b> ur_control, ur_telemetry, ur_bridge_ext, ur_jog, ur_kin and scan3d match LOCKED.sha256: the link is the one debugged on the cell.</div>`
      : `<div class="note bad"><b>Changed.</b> A locked robot-link file differs from LOCKED.sha256. Do not record: see the agent log.</div>`;
  },
};

function paint() {
  const f = ($("#sysF", el).value || "").toLowerCase(), w = $("#sysW", el).checked;
  const L = $("#sysLog", el);
  L.innerHTML = lines.filter((l) => (!w || l.level === "warn" || l.level === "error") && (!f || l.msg.toLowerCase().includes(f)))
    .map((l) => `<div class="${l.level || ""}"><span class="ts">${esc(l.t)}</span>${esc(l.msg)}</div>`).join("");
  L.scrollTop = L.scrollHeight;
}

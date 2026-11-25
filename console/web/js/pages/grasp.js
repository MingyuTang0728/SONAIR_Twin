/* Gripper and sequences: the Robotiq 2F-85 by hand, and the six kit sequences (Mode A and B on S2 and S4, the two
   hand-eye calibrations) previewed in the twin from the Unity data, then run on the robot as a session. */
import { $, $$, esc, fx, req, result, confirmBox, panel, pageHead, icon, S, MOVE_CHECKS } from "../core.js";
import { twin } from "../twin.js";

let el, seqs = [], cur = null, speedRate = 1;
const CLOSED = 227, STROKE = 85;
const posFor = (mm) => Math.round(CLOSED * (1 - Math.max(0, Math.min(STROKE, mm)) / STROKE));

export default {
  init(root) {
    el = root;
    el.innerHTML = pageHead("Cell", "Gripper &amp; sequences", "The Robotiq 2F-85 through the URCap socket on the controller, and the kit sequences the Unity package authors: preview each in the twin with its phases, gripper events and payload changes, then run it on the robot as a recorded session.") + `
      <div class="grid" style="grid-template-columns:minmax(320px,1fr) minmax(0,1.7fr);gap:12px;align-items:start">
        <div class="stack">
          ${panel("Gripper 2F-85", "grip", `
            <div class="row"><button class="btn" id="gConnect">${icon("link", "i sm")}Connect</button><button class="btn" id="gAct">Activate</button><span class="sp"></span><span class="tag" id="gState">off</span></div>
            <div style="margin:14px 0 6px" class="row"><span class="lbl-s">Opening</span><span class="sp"></span><span class="mono" id="gWidth">- mm</span></div>
            <div style="position:relative;height:34px;border:1px solid var(--line-2);border-radius:3px;background:var(--sunken)">
              <div id="gL" style="position:absolute;top:4px;bottom:4px;left:4px;width:14px;background:var(--text-3);border-radius:2px;transition:left .15s"></div>
              <div id="gR" style="position:absolute;top:4px;bottom:4px;right:4px;width:14px;background:var(--text-3);border-radius:2px;transition:right .15s"></div>
              <div id="gReq" style="position:absolute;top:0;bottom:0;width:2px;background:var(--accent);left:50%"></div></div>
            <div class="row" style="margin-top:12px"><label class="field grow"><span>Position (0 open, 255 closed)</span><input type="range" id="gPos" min="0" max="255" value="0"></label><span class="mono" id="gPosV" style="min-width:36px">0</span></div>
            <div class="row"><label class="field grow"><span>Speed</span><input type="range" id="gSpe" min="0" max="255" value="128"></label><label class="field grow"><span>Force</span><input type="range" id="gFor" min="0" max="255" value="64"></label></div>
            <div class="btngrid c4" style="margin-top:10px"><button class="btn" data-w="85">Open</button><button class="btn" data-w="27">27 mm</button><button class="btn" data-w="0">Close</button><button class="btn pri" id="gGo">Go</button></div>
            <div class="row" style="margin-top:6px"><button class="btn danger sm" id="gStop">Stop the fingers</button><span class="muted small" id="gInfo"></span></div>
            <div class="note" style="margin-top:10px">27 mm closes on the camera tool and specimen handles (the kit sequences). Gripper commands are refused while a command executes; STOP also stops the fingers where they are.</div>`)}
          ${panel("Run a sequence on the robot", "play", `<div id="runBox" class="t2">Choose a sequence.</div>`)}
        </div>
        <div class="stack">
          <div data-twin class="big" style="height:440px;min-height:0"></div>
          <div class="panel"><header><h2>${icon("seq")}Kit sequences</h2><span class="tag" id="sqTag">from the Unity package</span></header><div class="pb">
            <div class="grid g3" id="sqList"></div>
            <div id="sqPlayer" hidden style="margin-top:12px">
              <div class="row"><button class="btn sm" id="sqPlay">Play</button><button class="btn sm" id="sqPause">Pause</button>
                <div class="seg" id="sqRate"><button data-r="0.5">0.5x</button><button data-r="1" class="on">1x</button><button data-r="2">2x</button><button data-r="4">4x</button></div>
                <span class="sp"></span><span class="mono" id="sqTime">0.0 s</span><button class="btn sm" id="sqLive">Back to live</button></div>
              <input type="range" id="sqScrub" min="0" max="1000" value="0" style="width:100%;margin-top:6px">
              <div class="row small" style="margin-top:4px"><span class="tag acc" id="sqPhase">-</span><span class="sp"></span><span class="muted" id="sqGrip"></span></div>
              <div class="tbox" style="margin-top:10px;max-height:260px"><table class="t"><thead><tr><th class="num">t (s)</th><th>Phase</th><th>Gripper</th></tr></thead><tbody id="sqPhases"></tbody></table></div>
            </div>
          </div></div>
        </div>
      </div>`;
    $("#gConnect", el).onclick = async () => result(await req({ type: "grip_connect" }, 10000), "gripper connected");
    $("#gAct", el).onclick = async () => result(await req({ type: "grip_activate" }, 10000));
    $("#gPos", el).oninput = () => ($("#gPosV", el).textContent = $("#gPos", el).value);
    const move = async (pos) => result(await req({ type: "grip_move", pos, speed: +$("#gSpe", el).value, force: +$("#gFor", el).value }));
    $("#gGo", el).onclick = () => move(+$("#gPos", el).value);
    $$("[data-w]", el).forEach((b) => (b.onclick = () => { const p = posFor(+b.dataset.w); $("#gPos", el).value = p; $("#gPosV", el).textContent = p; move(p); }));
    $("#gStop", el).onclick = async () => result(await req({ type: "grip_stop" }));
    $("#sqPlay", el).onclick = () => twin.play(true);
    $("#sqPause", el).onclick = () => twin.play(false);
    $("#sqLive", el).onclick = () => { twin.setMode("live"); };
    $("#sqRate", el).onclick = (e) => { const b = e.target.closest("button"); if (!b) return; speedRate = +b.dataset.r; if (twin.player) twin.player.rate = speedRate; $$("#sqRate button", el).forEach((x) => x.classList.toggle("on", x === b)); };
    $("#sqScrub", el).oninput = () => { if (twin.player) { twin.play(false); twin.seek((+$("#sqScrub", el).value / 1000) * twin.player.dur); } };
    twin.on("seek", (p) => {
      if (!cur || p.kind !== "seq") return;
      $("#sqTime", el).textContent = `${fx(p.t, 1)} / ${fx(p.dur, 1)} s`;
      if (document.activeElement !== $("#sqScrub", el)) $("#sqScrub", el).value = Math.round((p.t / p.dur) * 1000);
      const s = p.s; let k = 0; s.phaseT.forEach((t, i) => { if (t <= p.t) k = i; });
      $("#sqPhase", el).textContent = s.phaseLabel[k];
      const gi = Math.min(s.grip.length - 1, Math.floor(p.t * s.fps));
      $("#sqGrip", el).textContent = `gripper closure ${fx(s.grip[gi], 2)}`;
      $$("#sqPhases tr", el).forEach((tr, i) => tr.classList.toggle("sel", i === k));
    });
    twin.on("mode", (m) => { if (m !== "preview") { $("#sqPlayer", el).hidden = true; cur = null; $$("#sqList .tile", el).forEach((t) => t.classList.remove("acc")); paintRun(); } });
  },
  async onShow() {
    if (!seqs.length) {
      const r = await req({ type: "seq_list" });
      seqs = r.sequences || [];
      $("#sqList", el).innerHTML = seqs.map((s) => `<div class="tile" data-k="${esc(s.key)}" style="cursor:pointer"><div class="row"><span class="k">${esc(s.key)}</span><span class="sp"></span><span class="badge">Mode ${esc(s.mode)}</span></div>
        <div style="font-weight:600;margin-top:6px;font-size:13px">${esc(s.title)}</div><div class="d">${fx(s.duration, 1)} s, ${s.phaseT.length} phases, ${(s.scans || []).length} scan window${(s.scans || []).length === 1 ? "" : "s"}</div></div>`).join("")
        || `<div class="muted">No sequences: the Unity package is not in this checkout.</div>`;
      $$("#sqList .tile", el).forEach((t) => (t.onclick = () => select(t.dataset.k)));
    }
  },
  onState(m) {
    const g = m.gripper || {};
    $("#gState", el).textContent = !g.connected ? (g.error ? "error" : "off") : g.fault ? `fault ${g.fault}` : g.activated ? (g.moving ? "moving" : g.object ? "holding" : "active") : "not activated";
    $("#gState", el).className = "tag " + (!g.connected ? "" : g.fault ? "bad" : g.activated ? "ok" : "warn");
    $("#gWidth", el).textContent = g.connected ? `${fx(g.width_mm, 1)} mm (pos ${g.pos})` : "- mm";
    const f = g.connected ? Math.max(0, Math.min(1, (g.pos || 0) / CLOSED)) : 0;
    const box = $("#gL", el).parentElement.clientWidth, span = (box - 36) / 2;
    $("#gL", el).style.left = 4 + f * span + "px"; $("#gR", el).style.right = 4 + f * span + "px";
    if (g.requested !== undefined && g.requested !== null) $("#gReq", el).style.left = `calc(${(4 + (Math.min(1, g.requested / CLOSED)) * span + 14) / box * 100}% )`;
    $("#gInfo", el).textContent = g.connected ? `host ${g.host}${g.object ? ", object detected" : ""}` : (g.error || "");
  },
};

async function select(key) {
  cur = seqs.find((s) => s.key === key); if (!cur) return;
  $$("#sqList .tile", el).forEach((t) => t.classList.toggle("acc", t.dataset.k === key));
  const s = await twin.preview(key); if (!s) return;
  twin.player.rate = speedRate;
  $("#sqPlayer", el).hidden = false;
  $("#sqPhases", el).innerHTML = s.phaseT.map((t, i) => {
    const gi = Math.min(s.grip.length - 1, Math.floor(t * s.fps)), gn = Math.min(s.grip.length - 1, Math.floor(((s.phaseT[i + 1] ?? s.duration) - 0.01) * s.fps));
    const ev = Math.abs(s.grip[gn] - s.grip[gi]) > 0.05 ? (s.grip[gn] > s.grip[gi] ? "close" : "open") : "";
    return `<tr class="click" data-t="${t}"><td class="num">${fx(t, 2)}</td><td>${esc(s.phaseLabel[i])}</td><td>${ev ? `<span class="badge ${ev === "close" ? "warn" : ""}">${ev}</span>` : ""}</td></tr>`;
  }).join("");
  $$("#sqPhases tr", el).forEach((tr) => (tr.onclick = () => { twin.play(false); twin.seek(+tr.dataset.t); }));
  twin.seek(0);
  paintRun();
}

function paintRun() {
  const box = $("#runBox", el);
  if (!cur) { box.innerHTML = "Choose a sequence: it previews in the twin above."; return; }
  const cmds = (S.hello && S.hello.commands) || [];
  const half = cmds.find((c) => c.id === cur.command_half), full = cmds.find((c) => c.id === cur.command);
  const opt = (c) => c ? `<option value="${esc(c.id)}">${esc(c.id)}: ${fx(c.duration_s, 1)} s, peak ${fx(c.peak_qd, 2)} rad/s</option>` : "";
  const sched = (half || full || {}).payload_schedule;
  box.innerHTML = `<div class="row"><label class="field grow"><span>Command (versioned, collision checked)</span><select id="rnCmd">${opt(half)}${opt(full)}</select></label>
      <label class="field"><span>Speed</span><input type="number" id="rnSpeed" class="w80" value="1" min="0.05" max="1" step="0.05"></label></div>
    <div class="small t2" id="rnInfo" style="margin-top:6px"></div>
    ${sched ? `<div class="small" style="margin-top:6px">Payload schedule: ${sched.map(([t, p]) => `<span class="tag out">${fx(t, 1)} s ${esc(p)}</span>`).join(" ")}</div>` : ""}
    <div class="note warn" style="margin-top:10px">The executor streams the joints only. Gripper events are <b>not</b> synchronised with the trajectory yet (milestone M2, URSim first): run it with nothing to grasp and the gripper open, or on URSim.</div>
    <div class="row" style="margin-top:10px"><button class="btn pri" id="rnGo">${icon("rec", "i sm")}Record it as a session</button></div>`;
  const info = () => { const c = cmds.find((x) => x.id === $("#rnCmd", el).value) || {}, sp = +$("#rnSpeed", el).value || 1, pk = (c.peak_qd || 0) * sp;
    $("#rnInfo", el).innerHTML = `${fx((c.duration_s || 0) / sp, 1)} s at x${sp}, peak <b style="color:${pk > 0.605 ? "var(--bad-text)" : "inherit"}">${fx(pk, 2)} rad/s</b>${pk > 0.605 ? " (the executor will refuse: limit 0.6)" : ""}`; };
  $("#rnCmd", el).onchange = info; $("#rnSpeed", el).oninput = info; info();
  $("#rnGo", el).onclick = async () => {
    const cmd = $("#rnCmd", el).value, speed = +$("#rnSpeed", el).value;
    if (!(await confirmBox("Run a kit sequence", `<p><b>${esc(cmd)}</b> at x${speed} through the executor, recorded as a session.</p>${MOVE_CHECKS}<p>The gripper is not driven by the sequence.</p>`, "Run"))) return;
    twin.setMode("live");
    const r = await req({ type: "session_start", motion: "executor", cmd, speed, experiment: "E3", notes: `kit sequence ${cur.key}` }, 30000);
    result(r, `session started: ${cmd}`);
  };
}

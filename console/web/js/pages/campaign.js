/* Campaign: plans from the versioned commands, one session per step, the console waits for the operator before every
   step unless auto-advance is on. */
import { $, $$, esc, fx, req, send, result, confirmBox, panel, pageHead, icon, S, MOVE_CHECKS } from "../core.js";

let el, selected = new Set(), campKey = "";

export default {
  init(root) {
    el = root;
    el.innerHTML = pageHead("Experiment", "Campaign", "A plan is a list of versioned commands with speed and repeat; each step is one recorded session through the executor. By default the console waits for you before every step; auto-advance waits only the settle time. A step whose pre-flight fails pauses the plan.") + `
      <div class="grid" style="grid-template-columns:minmax(0,1.6fr) minmax(300px,1fr);gap:12px;margin-bottom:12px">
        ${panel("Commands", "file", `<div class="row" style="margin-bottom:8px"><input type="search" id="cFilter" placeholder="filter, e.g. exc_ or task_A" style="flex:1"><span class="muted small" id="cCount"></span></div>
          <div class="tbox" style="max-height:390px"><table class="t"><thead><tr><th></th><th>Command</th><th>Kind</th><th>Payload</th><th class="num">Duration (s)</th><th class="num">Peak (rad/s)</th><th class="num">At speed</th></tr></thead><tbody id="cCmds"></tbody></table></div>`)}
        ${panel("Build a plan", "campaign", `<div class="row"><label class="field"><span>Speed</span><input type="number" id="cSpeed" class="w80" value="0.5" step="0.05" min="0.05" max="1"></label>
            <label class="field"><span>Repeats</span><input type="number" id="cRep" class="w60" value="1" min="1" max="10"></label><label class="field"><span>Settle (s)</span><input type="number" id="cSettle" class="w60" value="5" min="0"></label></div>
          <div class="row"><label class="field"><span>Experiment</span><select id="cExp"><option>E2</option><option>E1</option><option>E3</option><option>CAL</option><option>PILOT</option></select></label>
            <label class="field grow"><span>Payload on the pendant</span><select id="cPayload"></select></label></div>
          <div class="row"><label class="field grow"><span>Name</span><input type="text" id="cName" placeholder="e.g. E1 day 1"></label></div>
          <div class="row" style="margin-top:10px"><button class="btn pri" id="cBuild">Build from the ticked commands</button></div>
          <h3 class="sub" style="margin-top:16px">Presets</h3>
          <div class="row"><select id="cPreset" style="flex:1"></select><button class="btn" id="cBuildPreset">Build preset</button></div>
          <div class="note" style="margin-top:10px">Commands named <span class="mono">_x0.5</span> are already half speed: speed 0.5 on top is a quarter. The at-speed column shows the scaled peak.</div>
          <div class="muted small" id="cMsg" style="margin-top:6px"></div>`)}
      </div>
      <div class="panel"><header><h2>${icon("campaign")}Plan</h2><span class="t2 small" id="cPlanName"></span><span class="sp"></span>
          <label class="chk small"><input type="checkbox" id="cAuto"> auto-advance</label>
          <button class="btn pri sm" id="cStart">Start</button><button class="btn sm" id="cNext">Run next step</button><button class="btn sm" id="cResume">Resume</button><button class="btn danger sm" id="cAbort">Abort</button></header>
        <div class="pb"><div class="prog" style="margin-bottom:10px"><i id="cProg" style="width:0"></i></div>
        <div class="tbox" style="max-height:420px"><table class="t"><thead><tr><th class="num">#</th><th>Command</th><th class="num">Speed</th><th class="num">Rep</th><th>State</th><th>QC</th><th>Session</th><th>Note</th></tr></thead><tbody id="cSteps"></tbody></table></div></div></div>`;
    $("#cFilter", el).oninput = renderCmds; $("#cSpeed", el).oninput = renderCmds;
    $("#cBuild", el).onclick = async () => {
      if (!selected.size) return result({ ok: false, error: "tick at least one command" });
      const r = await req({ type: "campaign_build", commands: [...selected], speed: +$("#cSpeed", el).value, repeats: +$("#cRep", el).value, settle_s: +$("#cSettle", el).value,
        experiment: $("#cExp", el).value, payload: $("#cPayload", el).value, payload_on_pendant: $("#cPayload", el).value, name: $("#cName", el).value.trim() || "custom" });
      $("#cMsg", el).textContent = r.ok ? `${r.steps} steps` : r.error; result(r, `plan built: ${r.steps} steps`);
    };
    $("#cBuildPreset", el).onclick = async () => { const r = await req({ type: "campaign_build", preset: $("#cPreset", el).value }); $("#cMsg", el).textContent = r.ok ? `${r.steps} steps` : r.error; result(r, `plan built: ${r.steps} steps`); };
    $("#cStart", el).onclick = async () => {
      const c = S.st && S.st.campaign; if (!c) return result({ ok: false, error: "build a plan first" });
      const fast = c.steps.filter((s) => { const k = (S.hello.commands || []).find((x) => x.id === s.cmd); return k && k.peak_qd * s.speed > 0.605; });
      if (!(await confirmBox("Start the campaign", `<p><b>${esc(c.name)}</b>: ${c.steps.length} steps, ${$("#cAuto", el).checked ? "auto-advance after " + c.settle_s + " s" : "the console waits for you before each step"}.</p>
        <ul><li>Payload on the pendant matches the plan</li><li>Speed slider at 100 %</li></ul>${MOVE_CHECKS}${fast.length ? `<p style="color:var(--bad-text)">${fast.length} steps exceed 0.6 rad/s and will be refused by the executor.</p>` : ""}`, "Start"))) return;
      result(await req({ type: "campaign_start", auto: $("#cAuto", el).checked }), "campaign started");
    };
    $("#cNext", el).onclick = async () => result(await req({ type: "campaign_next" }), "next step started");
    $("#cResume", el).onclick = async () => result(await req({ type: "campaign_resume" }), "campaign resumed");
    $("#cAbort", el).onclick = async () => { if (await confirmBox("Abort the campaign", "Stops the executor and the robot, and ends the current session.", "Abort")) result(await req({ type: "campaign_abort" }, 15000), "campaign aborted"); };
    $("#cAuto", el).onchange = () => send({ type: "campaign_auto", auto: $("#cAuto", el).checked });
  },
  onShow() {
    const h = S.hello || {};
    if (!$("#cPreset", el).options.length) {
      $("#cPreset", el).innerHTML = (h.presets || []).map((p) => `<option>${esc(p)}</option>`).join("");
      $("#cPayload", el).innerHTML = `<option value="">not set</option>` + Object.entries(h.payloads || {}).map(([k, v]) => `<option value="${esc(k)}">${esc(k)}: ${esc(v.label || "")}</option>`).join("");
    }
    renderCmds(); campKey = "";
  },
  onState(m) {
    const c = m.campaign;
    $("#cPlanName", el).textContent = c ? `${c.name}: ${c.state}, step ${Math.min(c.index + 1, c.steps.length)} of ${c.steps.length}` : "";
    const done = c ? c.steps.filter((s) => s.state === "done" || s.state === "refused").length : 0;
    $("#cProg", el).style.width = c && c.steps.length ? (100 * done / c.steps.length) + "%" : "0";
    const key = c ? JSON.stringify([c.state, c.index, c.steps.map((s) => [s.state, s.qc])]) : "none";
    if (key === campKey) return; campKey = key;
    $("#cSteps", el).innerHTML = c ? c.steps.map((s, i) => `<tr class="${i === c.index && c.state === "running" ? "sel" : ""}"><td class="num">${i + 1}</td><td class="mono small">${esc(s.cmd)}</td>
      <td class="num">${s.speed}</td><td class="num">${s.rep}</td><td>${esc(s.state)}</td><td>${s.qc ? `<span class="badge ${esc(s.qc)}">${esc(s.qc)}</span>` : ""}</td>
      <td class="mono small">${esc(s.run_id || "")}</td><td class="small">${esc(s.note || "")}</td></tr>`).join("") : `<tr><td colspan="8" class="muted">No plan. Tick commands above and build one, or build a preset.</td></tr>`;
  },
};

function renderCmds() {
  const f = ($("#cFilter", el).value || "").toLowerCase(), sp = parseFloat($("#cSpeed", el).value) || 1, cmds = (S.hello && S.hello.commands) || [];
  const rows = cmds.filter((c) => !f || c.id.toLowerCase().includes(f));
  $("#cCount", el).textContent = `${rows.length} of ${cmds.length}`;
  $("#cCmds", el).innerHTML = rows.map((c) => { const pk = (c.peak_qd ?? NaN) * sp;
    return `<tr><td><input type="checkbox" data-id="${esc(c.id)}" ${selected.has(c.id) ? "checked" : ""}></td><td class="mono small">${esc(c.id)}</td><td>${esc(c.kind || "")}</td>
      <td class="small">${esc(c.payload || (c.payload_schedule ? "schedule" : "-"))}</td><td class="num">${fx(c.duration_s, 1)}</td><td class="num">${fx(c.peak_qd, 2)}</td>
      <td class="num" style="color:${pk > 0.605 ? "var(--bad-text)" : "inherit"}">${fx(pk, 2)}</td></tr>`; }).join("");
  $$("#cCmds input", el).forEach((x) => (x.onchange = () => (x.checked ? selected.add(x.dataset.id) : selected.delete(x.dataset.id))));
}

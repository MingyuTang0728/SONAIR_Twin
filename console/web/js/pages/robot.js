/* Robot: the controls of the cell console the operators know (pads, steps, keyboard, gamepad, power, programs,
   tool, speed, joint data, I/O), on the platform's unchanged robot link. */
import { $, $$, esc, fx, req, send, result, confirmBox, panel, pageHead, icon, S, DEG, JOINTS, Chart, SERIES, store, MOVE_CHECKS } from "../core.js";
import { bindPad, setKeys, step, jogState } from "../jog.js";

let el, chart, mode = "cont", stepSize = 1;
const disc = (pad, n, s, e, w) => `<div class="disc" data-pad="${pad}"><span class="lab n" data-base="${n}" data-tool="${n.replace(/([XYZ])/, "T$1")}">${n}</span><span class="lab s" data-base="${s}" data-tool="${s.replace(/([XYZ])/, "T$1")}">${s}</span>
  <span class="lab e" data-base="${e}" data-tool="${e.replace(/([XYZ])/, "T$1")}">${e}</span><span class="lab w" data-base="${w}" data-tool="${w.replace(/([XYZ])/, "T$1")}">${w}</span><div class="knob"></div></div>`;

export default {
  init(root) {
    el = root;
    el.innerHTML = pageHead("Cell", "Robot", "Live state, motion, power and tool set-up through the platform's debugged robot link (RTDE for state, URScript for motion, the dashboard for power and programs). Jog with the pads: motion stops the moment you let go.",
      `<span class="tag" id="rbLink">link off</span>`) + `
      <div class="kstrip" style="margin-bottom:12px">
        <div><b>X</b><span id="tX">-</span><i>mm</i></div><div><b>Y</b><span id="tY">-</span><i>mm</i></div><div><b>Z</b><span id="tZ">-</span><i>mm</i></div>
        <div><b>Force</b><span id="tF">-</span><i>N</i></div><div><b>Torque</b><span id="tT">-</span><i>N m</i></div><div><b>Hottest joint</b><span id="tTemp">-</span></div>
        <div><b>Tool speed</b><span id="tV">-</span><i>mm/s</i></div><div class="txt"><b>State</b><span id="tMode">-</span></div><div class="txt"><b>Safety</b><span id="tSafe">-</span></div>
      </div>

      <div class="panel" style="margin-bottom:12px"><header><h2>${icon("arm")}Move the robot</h2><span class="tag" id="jogHealth">idle</span></header><div class="pb">
        <div class="row" style="margin-bottom:14px">
          <div class="seg" id="jogMode"><button data-m="cont" class="on">Smooth</button><button data-m="step">Precise steps</button></div>
          <span class="sp"></span>
          <div class="seg"><button data-frame="base" class="on">Table axes</button><button data-frame="tool">Tool axes</button></div>
        </div>
        <div id="jogCont"><div class="jogwrap">
          <div class="pads">
            <div class="pad"><div class="pname">Across the table</div>${disc("xy", "+Y", "-Y", "+X", "-X")}<div class="ax">left / right &middot; forward / back</div></div>
            <div class="pad"><div class="pname">Up &amp; rotate</div>${disc("zr", "+Z", "-Z", "+Rz", "-Rz")}<div class="ax">up / down &middot; turn</div></div>
          </div>
          <div>
            <div class="speedrow"><span class="lbl-s">Speed</span><input type="range" id="jogSpeed" min="5" max="150" value="40"><span class="sv" id="jogSpeedV">40 mm/s</span></div>
            <div class="kbd-help">
              <div><kbd>&larr;</kbd><kbd>&rarr;</kbd> left / right (X)</div><div><kbd>&uarr;</kbd><kbd>&darr;</kbd> forward / back (Y)</div>
              <div><kbd>Q</kbd><kbd>E</kbd> down / up (Z)</div><div><kbd>A</kbd><kbd>D</kbd> rotate the tool</div>
              <div><kbd>Shift</kbd> hold for slow</div><div><kbd>Space</kbd> stop</div>
            </div>
            <label class="chk"><input type="checkbox" id="jogKeys"> Keyboard control on (page keys are off while it is)</label>
            <div class="gp">No gamepad. Plug one in and press a button.</div>
          </div>
        </div></div>
        <div id="jogStep" hidden>
          <div class="row" style="margin-bottom:10px"><span class="lbl-s">Step size</span>
            <div class="seg" id="stepSeg">${[0.1, 1, 5, 10, 50].map((v) => `<button data-s="${v}" class="${v === 1 ? "on" : ""}">${v} mm</button>`).join("")}</div>
            <span class="muted small">rotations: the same number in degrees, at most 15</span></div>
          <div class="stepgrid">${["x", "y", "z", "rx", "ry", "rz"].map((a) => `<div class="ax">${a.toUpperCase()}</div>`).join("")}
            ${["x", "y", "z", "rx", "ry", "rz"].map((a) => `<button class="btn" data-step="${a}" data-sg="1">+ ${a.toUpperCase()}</button>`).join("")}
            ${["x", "y", "z", "rx", "ry", "rz"].map((a) => `<button class="btn" data-step="${a}" data-sg="-1">- ${a.toUpperCase()}</button>`).join("")}</div>
          <div class="note" style="margin-top:10px">Each press moves exactly that far and stops. Use it for a position you need to repeat; the smooth pads are for getting roughly there.</div>
        </div>
        <div class="note warn" style="margin-top:12px">Put the teach pendant in <b>Remote Control</b> before moving the robot from here. In Local mode the robot ignores the command and the console can only report a timeout.</div>
      </div></div>

      <div class="grid g2" style="margin-bottom:12px">
        ${panel("Power &amp; recovery", "power", `<div class="note" style="margin-bottom:10px">The pendant's initialisation buttons. Use them when the arm is powered down or after a protective stop.</div>
          <div class="btngrid c3"><button class="btn" data-ur="ur_power_on">Power on</button><button class="btn go" data-ur="ur_brake_release">Release brakes</button><button class="btn danger" data-ur="ur_power_off" data-confirm="Power the arm off?">Power off</button></div>
          <div class="btngrid c2" style="margin-top:6px"><button class="btn" data-ur="ur_unlock_protective_stop">Clear protective stop</button><button class="btn" data-ur="ur_remote_control">Is the pendant in Remote?</button>
            <button class="btn" data-ur="ur_close_popup">Dismiss pendant message</button><button class="btn" data-ur="ur_close_safety_popup">Dismiss safety message</button></div>
          <div class="note" id="powerMsg" style="margin-top:10px">-</div>`)}
        ${panel("Programs on the pendant", "file", `<div class="row"><label class="field grow"><span>Program (file name on the controller)</span><input type="text" id="urpName" list="urpList" placeholder="e.g. sonair_e2_session.urp"><datalist id="urpList"></datalist></label>
            <button class="btn" id="urpLoaded" style="align-self:flex-end">Loaded?</button></div>
          <div class="btngrid c4" style="margin-top:8px"><button class="btn" id="urpLoad">Load</button><button class="btn go" id="urpPlay">Play</button><button class="btn" data-ur="ur_pause">Pause</button><button class="btn danger" data-ur="ur_stop_program">Stop</button></div>
          <div class="note" id="urpMsg" style="margin-top:10px">The dashboard server cannot list programs: type the name. Names you load are remembered on this computer.</div>`)}
      </div>

      <div class="grid g2" style="margin-bottom:12px">
        ${panel("Tool settings", "tool", `<div class="row"><label class="field grow"><span>Payload on the flange (kit states, config/payloads.json)</span><select id="plPreset"></select></label></div>
          <div class="row"><label class="field"><span>Mass (kg)</span><input type="number" id="plMass" class="w100" step="0.01" value="1.10"></label>
            <label class="field grow"><span>Centre of mass X, Y, Z (mm, tool0)</span><input type="text" id="plCog" value="0, 0, 40"></label>
            <button class="btn" id="plSet" style="align-self:flex-end">Apply payload</button></div>
          <div class="row"><label class="field grow"><span>TCP X, Y, Z (mm), RX, RY, RZ (deg, rotation vector)</span><input type="text" id="tcpVals" value="0, 0, 0, 0, 0, 0"></label>
            <button class="btn" id="tcpSet" style="align-self:flex-end">Apply TCP</button></div>
          <div class="row"><button class="btn" data-ur="ur_zero_ft">Zero the force sensor</button><span class="sp"></span>
            <div class="seg" id="toolV"><button data-v="0" class="on">Tool power off</button><button data-v="12">12 V</button><button data-v="24">24 V</button></div></div>
          <div class="note warn" style="margin-top:10px">The benchmark convention is TCP = tool0 (offset 0). A wrong payload changes the joint currents the benchmark scores: it must match the pendant and what is on the flange.</div>`)}
        ${panel("Speed limit", "gauge", `<div class="speedrow"><input type="range" id="urSpeed" min="5" max="100" step="5" value="100"><span class="sv" id="urSpeedV">100 %</span></div>
          <div class="note" style="margin-top:10px">The pendant's speed slider. Recorded commands need <b>100 %</b>: the executor does the speed scaling itself, and a session with speed scaling below 1 while moving fails QC.</div>
          <h3 class="sub" style="margin-top:16px">Unity live mirror (display only)</h3>
          <div class="row"><label class="field grow"><span>Unity host:port</span><input type="text" id="uTarget" placeholder="127.0.0.1:5005"></label>
            <button class="btn" id="uSet" style="align-self:flex-end">Forward</button><button class="btn" id="uOff" style="align-self:flex-end">Off</button></div>
          <div class="muted small" id="uMsg" style="margin-top:6px">Measured and commanded joints at 50 Hz over UDP to the SONAIR Unity package (Live mode). Nothing from Unity reaches the robot.</div>`)}
      </div>

      <details class="panel" style="margin-bottom:12px" open><summary>Joint data <span class="tag">angle &middot; target &middot; speed &middot; current &middot; torque &middot; temperature</span></summary><div class="pb">
        <div class="tbox" style="max-height:none"><table class="t"><thead><tr><th>Joint</th><th class="num">Angle (deg)</th><th class="num">Target (deg)</th><th class="num">Speed (deg/s)</th><th class="num">Current (A)</th>
          <th class="num">Torque model (N m)</th><th class="num">Temperature (C)</th><th class="num">Voltage (V)</th></tr></thead><tbody id="jTable"></tbody></table></div>
        <div class="legend" id="jLeg" style="margin-top:12px"></div><div id="jChart"></div>
      </div></details>

      <details class="panel"><summary>Electrical &amp; input/output <span class="tag">supply &middot; digital &middot; analogue &middot; tool connector</span></summary><div class="pb">
        <div class="kstrip" id="elec" style="margin-bottom:12px"></div>
        <div class="grid g3">
          <div><div class="lbl-s" style="margin-bottom:6px">Digital inputs</div><div class="io" id="ioIn"></div></div>
          <div><div class="lbl-s" style="margin-bottom:6px">Digital outputs: click to switch</div><div class="io" id="ioOut"></div></div>
          <div><div class="lbl-s" style="margin-bottom:6px">Tool outputs</div><div class="io" id="ioTool"></div></div>
        </div>
        <div class="row" style="margin-top:12px"><label class="field"><span>Analogue out</span><select id="aoPin"><option value="0">Output 0</option><option value="1">Output 1</option></select></label>
          <label class="field"><span>Value 0 to 1</span><input type="number" id="aoVal" class="w80" value="0" step="0.05" min="0" max="1"></label><button class="btn" id="aoSet" style="align-self:flex-end">Set</button></div>
        <div class="note" style="margin-top:10px">Outputs switch real hardware. The tool connector is where a probe or a light would be wired.</div>
      </div></details>`;

    $("#jogMode", el).onclick = (e) => { const b = e.target.closest("button"); if (!b) return; mode = b.dataset.m; $$("#jogMode button", el).forEach((x) => x.classList.toggle("on", x === b)); $("#jogCont", el).hidden = mode !== "cont"; $("#jogStep", el).hidden = mode !== "step"; };
    $("#stepSeg", el).onclick = (e) => { const b = e.target.closest("button"); if (!b) return; stepSize = +b.dataset.s; $$("#stepSeg button", el).forEach((x) => x.classList.toggle("on", x === b)); };
    $$("[data-step]", el).forEach((b) => (b.onclick = async () => { const a = b.dataset.step, rot = a.startsWith("r"); result(await step(a, +b.dataset.sg * (rot ? Math.min(15, stepSize) : stepSize)), `step ${b.textContent.trim()}`); }));
    $("#jogKeys", el).onchange = (e) => setKeys(e.target.checked);
    $$("[data-ur]", el).forEach((b) => (b.onclick = async () => {
      if (b.dataset.confirm && !(await confirmBox(b.textContent, esc(b.dataset.confirm)))) return;
      const r = await req({ type: b.dataset.ur }, 15000); result(r);
      const m = b.closest(".panel").querySelector("#powerMsg, #urpMsg"); if (m) { m.className = "note " + (r.ok === false ? "bad" : ""); m.textContent = `${b.textContent}: ${r.msg || r.error || (r.ok === false ? "failed" : "done")}`; }
    }));
    // programs
    const remembered = () => store.get("programs", []);
    const paintPrograms = () => ($("#urpList", el).innerHTML = remembered().map((p) => `<option value="${esc(p)}">`).join(""));
    paintPrograms();
    $("#urpLoaded", el).onclick = async () => { const r = await req({ type: "ur_loaded_program" }); $("#urpMsg", el).textContent = r.msg || r.error || "-"; };
    $("#urpLoad", el).onclick = async () => {
      const n = $("#urpName", el).value.trim(); if (!n) return;
      const r = await req({ type: "ur_load_program", name: n }, 15000); result(r); $("#urpMsg", el).textContent = r.msg || r.error || "-";
      if (r.ok !== false) { store.set("programs", [n, ...remembered().filter((x) => x !== n)].slice(0, 12)); paintPrograms(); }
    };
    $("#urpPlay", el).onclick = async () => {
      if (!(await confirmBox("Play the loaded program", `The pendant program moves the arm on its own.${MOVE_CHECKS}`, "Play"))) return;
      const r = await req({ type: "ur_play" }, 15000); result(r); $("#urpMsg", el).textContent = r.msg || r.error || "-";
    };
    // tool
    $("#plPreset", el).onchange = () => {
      const p = (S.hello.payloads || {})[$("#plPreset", el).value]; if (!p) return;
      $("#plMass", el).value = p.mass_kg; $("#plCog", el).value = p.cog_m.map((v) => fx(v * 1e3, 1)).join(", ");
    };
    $("#plSet", el).onclick = async () => {
      const mass = +$("#plMass", el).value, cog = $("#plCog", el).value.split(",").map((v) => +v / 1000);
      if (cog.length !== 3 || cog.some((v) => !Number.isFinite(v))) return result({ ok: false, error: "centre of mass needs three numbers" });
      if (!(await confirmBox("Set the payload on the controller", `<ul><li>mass ${mass} kg</li><li>centre of gravity ${cog.map((v) => fx(v * 1e3, 1)).join(", ")} mm in tool0</li></ul><p>Check it matches what is on the flange and what the session will be labelled with.</p>`))) return;
      result(await req({ type: "ur_set_payload", mass, cog }));
    };
    $("#tcpSet", el).onclick = async () => {
      const v = $("#tcpVals", el).value.split(",").map(Number); if (v.length !== 6 || v.some((x) => !Number.isFinite(x))) return result({ ok: false, error: "the TCP needs six numbers" });
      if (!(await confirmBox("Set the TCP", `<p>${v.join(", ")} (mm, deg)</p><p>The benchmark records tool0 with TCP offset 0. Set a TCP only for work outside the benchmark.</p>`))) return;
      result(await req({ type: "ur_set_tcp", pose: [v[0] / 1e3, v[1] / 1e3, v[2] / 1e3, v[3] / DEG, v[4] / DEG, v[5] / DEG] }));
    };
    $("#toolV", el).onclick = async (e) => { const b = e.target.closest("button"); if (!b) return; const r = await req({ type: "ur_set_tool_voltage", volts: +b.dataset.v }); if (result(r)) $$("#toolV button", el).forEach((x) => x.classList.toggle("on", x === b)); };
    $("#urSpeed", el).oninput = () => ($("#urSpeedV", el).textContent = $("#urSpeed", el).value + " %");
    $("#urSpeed", el).onchange = async () => result(await req({ type: "ur_speed_slider", fraction: $("#urSpeed", el).value / 100 }));
    $("#uSet", el).onclick = async () => result(await req({ type: "unity_set", target: $("#uTarget", el).value.trim() }), "forwarding to Unity");
    $("#uOff", el).onclick = async () => result(await req({ type: "unity_set", target: "" }), "Unity forwarding off");
    $("#aoSet", el).onclick = async () => result(await req({ type: "ur_set_aout", pin: +$("#aoPin", el).value, value: Math.max(0, Math.min(1, +$("#aoVal", el).value)) }));
    el.addEventListener("click", async (e) => {
      const b = e.target.closest("[data-dout]"); if (!b) return;
      const tool = b.dataset.tool === "1", pin = +b.dataset.dout, val = !b.classList.contains("on");
      if (!(await confirmBox(`Switch ${tool ? "tool " : ""}output ${pin} ${val ? "on" : "off"}`, "Outputs switch real hardware on the cell."))) return;
      result(await req({ type: tool ? "ur_set_tool_dout" : "ur_set_dout", pin, value: val }));
    });
    chart = new Chart($("#jChart", el), { window: 30, legend: $("#jLeg", el), minSpan: 1, series: JOINTS.map((j, i) => ({ name: j + " (deg)", color: SERIES()[i] })) });
  },
  onShow() {
    const pl = (S.hello && S.hello.payloads) || {};
    if (!$("#plPreset", el).options.length) $("#plPreset", el).innerHTML = `<option value="">choose a kit state</option>` + Object.entries(pl).map(([k, v]) => `<option value="${esc(k)}">${esc(k)}: ${esc(v.label || "")} (${v.mass_kg} kg)</option>`).join("");
    $("#jogKeys", el).checked = jogState.keys;
  },
  always(m) { const R = m.robot || {}; if (R.actual_q && R.timestamp) chart.push(R.timestamp, R.actual_q.map((v) => v * DEG)); },
  onState(m) {
    this.always(m);
    const R = m.robot || {}, L = m.link || {};
    $("#rbLink", el).textContent = L.enabled ? `${L.host} ${L.connected ? (L.source || "") + " " + fx(L.rate_hz, 0) + " Hz" : "connecting"}` : "link off";
    $("#rbLink", el).className = "tag " + (L.connected && L.source === "rtde" ? "ok" : L.enabled ? "warn" : "");
    const p = R.actual_TCP_pose || [], f = R.actual_TCP_force || [], sp = R.actual_TCP_speed || [], tmp = R.joint_temperatures || [];
    $("#tX", el).textContent = fx(p[0] * 1e3, 1); $("#tY", el).textContent = fx(p[1] * 1e3, 1); $("#tZ", el).textContent = fx(p[2] * 1e3, 1);
    $("#tF", el).textContent = f.length ? fx(Math.hypot(f[0], f[1], f[2]), 1) : "-"; $("#tT", el).textContent = f.length ? fx(Math.hypot(f[3], f[4], f[5]), 2) : "-";
    if (tmp.length) { const k = tmp.indexOf(Math.max(...tmp)); $("#tTemp", el).innerHTML = `${fx(tmp[k], 1)}<i>C, ${JOINTS[k].toLowerCase()}</i>`; }
    $("#tV", el).textContent = sp.length ? fx(Math.hypot(sp[0], sp[1], sp[2]) * 1e3, 1) : "-";
    $("#tMode", el).textContent = R.robot_mode_text || "-"; $("#tSafe", el).textContent = R.safety_mode_text || "-";
    const j = m.jog || {};
    $("#jogHealth", el).textContent = m.busy ? "refused: command executing" : j.moving ? "moving" : jogState.moving ? "sending" : "idle";
    $("#jogHealth", el).className = "tag " + (m.busy ? "bad" : j.moving ? "acc" : "");
    const q = R.actual_q || [], tq = R.target_q || [], qd = R.actual_qd || [], cur = R.actual_current || [], mom = R.target_moment || [], jv = R.actual_joint_voltage || [];
    $("#jTable", el).innerHTML = JOINTS.map((n, i) => `<tr><td>${n}</td><td class="num">${fx(q[i] * DEG, 3)}</td><td class="num">${fx(tq[i] * DEG, 3)}</td><td class="num">${fx(qd[i] * DEG, 2)}</td>
      <td class="num">${fx(cur[i], 3)}</td><td class="num">${fx(mom[i], 2)}</td><td class="num">${fx(tmp[i], 1)}</td><td class="num">${fx(jv[i], 1)}</td></tr>`).join("");
    $("#elec", el).innerHTML = [["Robot", R.actual_robot_voltage, "V"], ["Robot", R.actual_robot_current, "A"], ["Main", R.actual_main_voltage, "V"], ["Tool out", R.tool_output_voltage, "V"],
      ["Tool out", R.tool_output_current, "A"], ["Tool", R.tool_temperature, "C"]].map(([k, v, u]) => `<div><b>${k}</b><span>${fx(v, 2)}</span><i>${u}</i></div>`).join("");
    const di = R.actual_digital_input_bits, dout = R.actual_digital_output_bits;
    const bits = (v, n, off = 0) => Array.from({ length: n }, (_, k) => (v !== undefined && v !== null ? Math.floor(v / 2 ** (k + off)) % 2 === 1 : false));
    $("#ioIn", el).innerHTML = bits(di, 8).map((on, k) => `<button disabled class="${on ? "on" : ""}">${k}</button>`).join("");
    $("#ioOut", el).innerHTML = bits(dout, 8).map((on, k) => `<button data-dout="${k}" class="${on ? "on" : ""}">${k}</button>`).join("");
    $("#ioTool", el).innerHTML = bits(dout, 2, 16).map((on, k) => `<button data-dout="${k}" data-tool="1" class="${on ? "on" : ""}">T${k}</button>`).join("");
  },
};

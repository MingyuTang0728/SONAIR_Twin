/* Data: every run in one place (sessions recorded here, the released benchmark-v1 runs), any channel of any joint,
   two runs overlaid, QC, and the run replayed in the twin with the chart cursor following it. */
import { $, $$, esc, fx, req, result, panel, pageHead, icon, Chart, SERIES, DEG, JOINTS, S } from "../core.js";
import { twin } from "../twin.js";

let el, tab = "bench", sessions = [], bench = [], cur = null, cmp = null, chart, ch = "q", jsel = new Set([2]), loaded = false;
const CH = { q: ["Joints", "deg"], current: ["Currents", "A"], qd: ["Velocities", "deg/s"], target_moment: ["Torque model", "N m"], tcp: ["Tool position", "mm"], gyro: ["IMU gyro", "rad/s"], accel: ["IMU accel", "m/s2"] };

export default {
  init(root) {
    el = root;
    el.innerHTML = pageHead("Analysis", "Data", "Sessions recorded on this console and the released benchmark-v1 real runs (published cells with every channel; held-out cells as command inputs only). Pick a run, choose a channel, overlay a second run, replay it in the twin.",
      `<button class="btn" id="dRefresh">Refresh</button>`) + `
      <div class="grid" style="grid-template-columns:minmax(340px,0.9fr) minmax(0,1.6fr);gap:12px;align-items:start">
        <div class="panel"><header><div class="seg" id="dTabs"><button data-t="bench" class="on">benchmark-v1</button><button data-t="sessions">Sessions here</button></div><span class="sp"></span><span class="tag" id="dN">-</span></header><div class="pb">
          <div id="dBenchF" class="row" style="margin-bottom:8px"><select id="fSplit"><option value="">all splits</option><option value="train">published</option><option value="test">held-out inputs</option></select>
            <select id="fSess"><option value="">all sessions</option></select><select id="fV"><option value="">any speed</option></select>
            <select id="fArm"><option value="">any arm</option><option>near_singular</option><option>mid_workspace</option><option>extended</option></select>
            <select id="fTraj"><option value="">any trajectory</option><option>contour</option><option>point_to_point</option><option>stop_start</option></select></div>
          <input type="search" id="fQ" placeholder="filter by run id" style="width:100%;margin-bottom:8px">
          <div class="tbox" style="max-height:600px"><table class="t"><thead id="dHead"></thead><tbody id="dRows"></tbody></table></div>
          <div class="muted small" style="margin-top:6px">Click a run to open it; Shift-click a second run to overlay it.</div>
        </div></div>
        <div class="stack">
          <div class="panel"><header><h2>${icon("data")}<span id="dTitle">No run open</span></h2><span class="sp"></span><span id="dBadges"></span></header><div class="pb">
            <div class="row" style="margin-bottom:8px"><div class="seg" id="dCh">${Object.entries(CH).map(([k, v]) => `<button data-c="${k}" class="${k === ch ? "on" : ""}">${v[0]}</button>`).join("")}</div></div>
            <div class="row" id="dJ" style="margin-bottom:8px"></div>
            <div class="legend" id="dLeg"></div><div id="dChart"></div>
            <div class="row" style="margin-top:8px"><button class="btn" id="dReplay">${icon("cube", "i sm")}Replay in the live arm view</button><button class="btn" id="dCmpOff" hidden>Remove overlay</button>
              <span class="sp"></span><button class="btn" id="dExport" hidden>Export to the platform format</button></div>
          </div></div>
          <div class="grid g2"><div class="panel"><header><h2>${icon("file")}Manifest</h2></header><div class="pb"><dl class="kv small" id="dMan"><dt>-</dt><dd>-</dd></dl></div></div>
            <div class="panel"><header><h2>${icon("check")}Quality</h2></header><div class="pb" id="dQc"><span class="muted">Sessions carry their QC; benchmark-v1 runs are covered by the audit (Benchmark page).</span></div></div></div>
        </div>
      </div>`;
    chart = new Chart($("#dChart", el), { cls: "tall", legend: $("#dLeg", el), empty: "open a run", onClick: (t) => { if (twin.mode === "replay") { twin.play(false); twin.seek(t); } } });
    $("#dTabs", el).onclick = (e) => { const b = e.target.closest("button"); if (!b) return; tab = b.dataset.t; $$("#dTabs button", el).forEach((x) => x.classList.toggle("on", x === b)); $("#dBenchF", el).hidden = tab !== "bench"; list(); };
    ["#fSplit", "#fSess", "#fV", "#fArm", "#fTraj"].forEach((s) => ($(s, el).onchange = list)); $("#fQ", el).oninput = list;
    $("#dRefresh", el).onclick = () => load(true);
    $("#dCh", el).onclick = (e) => { const b = e.target.closest("button"); if (!b) return; ch = b.dataset.c; $$("#dCh button", el).forEach((x) => x.classList.toggle("on", x === b)); paintJ(); plot(); };
    $("#dCmpOff", el).onclick = () => { cmp = null; $("#dCmpOff", el).hidden = true; plot(); };
    $("#dReplay", el).onclick = replay;
    $("#dExport", el).onclick = async () => { if (!cur) return; const r = await req({ type: "session_export", run_id: cur.run_id }, 120000); result(r, r.ok ? `exported: ${r.platform_run.split("/").slice(-3).join("/")}` : ""); };
    twin.on("seek", (p) => { if (p.kind === "run" && cur) chart.setMarks([{ t: p.t, color: "var(--s5)" }]); });
    twin.on("mode", (m) => { if (m !== "replay") chart.setMarks([]); });
    paintJ();
  },
  async onShow() {
    await load(!loaded);
    const o = sessionStorage.getItem("sonair.open"); if (o) { sessionStorage.removeItem("sonair.open"); const x = JSON.parse(o); tab = x.kind === "session" ? "sessions" : "bench"; $$("#dTabs button", el).forEach((b) => b.classList.toggle("on", b.dataset.t === tab)); list(); open(x.kind, x.id); }
  },
};

async function load(force) {
  if (!force && loaded) return list();
  const [a, b] = await Promise.all([req({ type: "data_sessions" }), req({ type: "data_bench" })]);
  sessions = a.sessions || []; bench = b.runs || []; loaded = true;
  $("#fSess", el).innerHTML = `<option value="">all sessions</option>` + [...new Set(bench.map((r) => r.session))].sort().map((s) => `<option>${esc(s)}</option>`).join("");
  $("#fV", el).innerHTML = `<option value="">any speed</option>` + [...new Set(bench.map((r) => r.joint_vel))].sort((x, y) => x - y).map((s) => `<option>${esc(s)}</option>`).join("");
  list();
}

function list() {
  if (tab === "bench") {
    const sp = $("#fSplit", el).value, se = $("#fSess", el).value, v = $("#fV", el).value, arm = $("#fArm", el).value, tr = $("#fTraj", el).value, q = $("#fQ", el).value.toLowerCase();
    const rows = bench.filter((r) => (!sp || r.split === sp) && (!se || r.session === se) && (!v || r.joint_vel === v) && (!arm || r.arm_config === arm) && (!tr || r.traj_type === tr) && (!q || r.run_id.includes(q)));
    $("#dN", el).textContent = `${rows.length} of ${bench.length}`;
    $("#dHead", el).innerHTML = `<tr><th>Run</th><th>Split</th><th class="num">s</th></tr>`;
    $("#dRows", el).innerHTML = rows.slice(0, 400).map((r) => `<tr class="click ${cur && cur.run_id === r.run_id ? "sel" : ""}" data-id="${esc(r.run_id)}" data-k="bench" title="${esc(r.session)}, ${esc(r.started_utc)}"><td class="mono small" style="white-space:nowrap">${esc(r.run_id)}</td>
      <td><span class="badge ${r.split === "test" ? "test" : "train"}">${r.split === "test" ? "held-out" : "published"}</span></td><td class="num">${fx(r.duration_s, 1)}</td></tr>`).join("");
  } else {
    const q = $("#fQ", el).value.toLowerCase(), rows = sessions.filter((s) => !q || s.run_id.toLowerCase().includes(q));
    $("#dN", el).textContent = `${rows.length}`;
    $("#dHead", el).innerHTML = `<tr><th>Session</th><th>QC</th><th class="num">s</th><th>Motion</th></tr>`;
    $("#dRows", el).innerHTML = rows.map((s) => `<tr class="click ${cur && cur.run_id === s.run_id ? "sel" : ""}" data-id="${esc(s.run_id)}" data-k="session"><td class="mono small">${esc(s.run_id)}${s.mock ? ' <span class="badge mock">mock</span>' : ""}</td>
      <td><span class="badge ${esc(s.verdict || "")}" title="${esc((s.reasons || []).join("\n"))}">${esc(s.verdict || "-")}</span></td><td class="num">${fx(s.duration_s, 1)}</td>
      <td class="small">${esc((s.motion || {}).command_id || (s.motion || {}).source || "-")}</td></tr>`).join("") || `<tr><td colspan="4" class="muted">No sessions recorded yet (Record page).</td></tr>`;
  }
  $$("#dRows tr[data-id]", el).forEach((tr) => (tr.onclick = (e) => open(tr.dataset.k, tr.dataset.id, e.shiftKey)));
}

async function open(kind, id, asCompare) {
  $("#dTitle", el).textContent = "loading " + id + "...";
  const r = await req({ type: "data_series", kind, run_id: id }, 90000);
  if (r.error) { $("#dTitle", el).textContent = r.error; return; }
  if (asCompare && cur) { cmp = r; $("#dCmpOff", el).hidden = false; }
  else { cur = r; cmp = null; $("#dCmpOff", el).hidden = true; }
  const c = cur;
  $("#dTitle", el).textContent = c.run_id + (cmp ? `  vs  ${cmp.run_id}` : "");
  $("#dBadges", el).innerHTML = c.kind === "session" ? `<span class="badge ${esc((c.qc || {}).verdict || "")}">${esc((c.qc || {}).verdict || "no QC")}</span>`
    : `<span class="badge ${c.split === "train" ? "train" : "test"}">${c.split === "train" ? "published" : "held-out: inputs only"}</span>`;
  $("#dExport", el).hidden = c.kind !== "session";
  const man = c.manifest || {};
  const keys = ["run_id", "joint_vel", "arm_config", "traj_type", "repeat_idx", "experiment", "carrier_id", "carrier_mass_kg", "started_utc", "created_utc", "operator", "notes", "payload", "mock", "console", "imu_odr"];
  $("#dMan", el).innerHTML = keys.filter((k) => man[k] !== undefined && man[k] !== null && man[k] !== "").map((k) => `<dt>${k}</dt><dd>${esc(typeof man[k] === "object" ? JSON.stringify(man[k]) : man[k])}</dd>`).join("")
    + (man.motion ? `<dt>motion</dt><dd>${esc(JSON.stringify(man.motion))}</dd>` : "") || `<dt>-</dt><dd>no manifest</dd>`;
  const q = c.qc;
  $("#dQc", el).innerHTML = q ? `<div class="row"><span class="badge ${esc(q.verdict)}">${esc(q.verdict)}</span><span class="small t2">RTDE ${q.rtde ? `${q.rtde.n} rows, ${q.rtde.missing_samples} missing, max gap ${q.rtde.max_gap_ms} ms` : ""}</span></div>
    <ul class="small" style="margin:8px 0 0;padding-left:18px">${(q.reasons || []).map((x) => `<li>${esc(x)}</li>`).join("") || "<li>every gate passed</li>"}</ul>
    ${Object.entries(q.imu || {}).map(([n, u]) => `<div class="small mono" style="margin-top:6px">${esc(n)}: ${u.rate_hz} Hz, coverage ${u.coverage}, corr ${u.robot_corr}, lag ${u.lag_ms} ms</div>`).join("")}`
    : c.kind === "session" ? `<span class="muted">No qc.json.</span>` : `<span class="muted">benchmark-v1 runs: see the audit on the Benchmark page (0 controller period anomalies, 0 stale rows over 108 runs).</span>`;
  list(); paintJ(); plot();
}

function paintJ() {
  const n = ch === "tcp" || ch === "gyro" || ch === "accel" ? 3 : 6, names = n === 3 ? ["x", "y", "z"] : JOINTS;
  $("#dJ", el).innerHTML = `<span class="lbl-s">${n === 3 ? "Axes" : "Joints"}</span>` + names.map((j, i) => `<label class="chk"><input type="checkbox" data-j="${i}" ${jsel.has(i) ? "checked" : ""}>${j}</label>`).join("");
  $$("#dJ input", el).forEach((x) => (x.onchange = () => { x.checked ? jsel.add(+x.dataset.j) : jsel.delete(+x.dataset.j); plot(); }));
}

function scale(k, v) { if (v === null || v === undefined) return null; return k === "q" || k === "target_q" || k === "qd" ? v * DEG : k === "tcp" ? v * 1000 : v; }

function plot() {
  if (!cur) return;
  const col = SERIES(), ser = [], ys = [];
  let t = cur.t;
  const add = (run, key, i, name, color, dash) => {
    const arr = run.channels[key]; if (!arr || !arr[i]) return false;
    const y = arr[i].map((v) => scale(key, v));
    if (run !== cur) { const tt = run.t; ys.push(t.map((x) => { let lo = 0, hi = tt.length - 1; if (x > tt[hi]) return null; while (hi - lo > 1) { const m = (lo + hi) >> 1; if (tt[m] < x) lo = m; else hi = m; } return y[lo]; })); }
    else ys.push(y);
    ser.push({ name, color, dash }); return true;
  };
  const names = ch === "tcp" || ch === "gyro" || ch === "accel" ? ["x", "y", "z"] : JOINTS;
  [...jsel].sort().forEach((i) => {
    if (i >= names.length) return;
    const c = col[i % 6];
    const had = add(cur, ch, i, `${names[i]}${ch === "q" ? " measured" : ""}`, c, []);
    if (ch === "q") add(cur, "target_q", i, `${names[i]} commanded`, c, had ? [2, 3] : []);
    if (cmp) add(cmp, ch === "q" && !cmp.channels.q ? "target_q" : ch, i, `${names[i]} (overlay)`, c, [7, 4]);
  });
  chart.o.empty = ys.length ? "" : cur.split === "test" ? "held-out run: measured channels are withheld (inputs only)" : "no such channel in this run";
  chart.o.yLabel = CH[ch][1];
  chart.setSeries(ser); chart.setData(t, ys);
}

function replay() {
  if (!cur) return;
  const Q = cur.channels.q || cur.channels.target_q; if (!Q) return result({ ok: false, error: "no joint data" });
  const rows = cur.t.map((_, k) => Q.map((c) => c[k]));
  twin.replay({ t: cur.t, q: rows, name: cur.run_id }); twin.play(true);
}

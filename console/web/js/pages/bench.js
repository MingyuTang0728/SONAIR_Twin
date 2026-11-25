/* Benchmark: the scoreboard at a glance. Tracks and channels, every entry's score with its interval and the ceiling
   set by the real-real floor, scores by speed, the gyro rule change, the condition grid with what has been recorded,
   how an entry gets scored, collection hours and release readiness. Read from the repository: nothing is recomputed. */
import { $, $$, esc, fx, req, panel, pageHead, icon, css } from "../core.js";

let el, ov = null, info = null, gridMetric = "runs";

const NAMES = { S0: "S0 · MuJoCo Menagerie, as shipped", CM: "CM · controller's own torque model", F1: "F1 · S0 + Coulomb and viscous friction",
  B2_s0map: "B2 · twin through the published S0 map", B2: "B2 · twin, its own map (task/3)", B2F: "B2F · B2 + Coulomb and viscous friction", B2FR: "B2FR · B2F + angle-locked ripple" };

function bar(v, lo, hi, opt = {}) {
  const X = (x) => ((x - lo) / (hi - lo)) * 100;
  const z = X(0), a = X(Math.min(0, v)), b = X(Math.max(0, v));
  let h = `<div class="track"><div class="zero" style="left:${z}%"></div>`;
  if (opt.ceil !== undefined) h += `<div class="ceil" style="left:${X(opt.ceil)}%" title="ceiling ${fx(opt.ceil, 3)}"></div>`;
  h += `<div class="fill ${opt.tail ? "tail" : opt.both ? "allp" : ""} ${v < 0 ? "neg" : ""}" style="left:${a}%;width:${b - a}%"></div>`;
  if (opt.tv !== undefined) { const ta = X(Math.min(0, opt.tv)), tb = X(Math.max(0, opt.tv)); h += `<div class="fill tail ${opt.tv < 0 ? "neg" : ""}" style="left:${ta}%;width:${tb - ta}%"></div>`; }
  if (opt.ci) h += `<div class="ci" style="left:${X(opt.ci[0])}%;width:${X(opt.ci[1]) - X(opt.ci[0])}%;${opt.tv !== undefined ? "bottom:9px" : ""}"></div>`;
  if (opt.cit) h += `<div class="ci" style="left:${X(opt.cit[0])}%;width:${X(opt.cit[1]) - X(opt.cit[0])}%;top:9px"></div>`;
  return h + "</div>";
}

export default {
  init(root) {
    el = root;
    el.innerHTML = pageHead("Analysis", "Benchmark", "One reference simulator (MuJoCo, S0 = the Menagerie UR5e as shipped); every other simulator is a scored entry. GCR = 1 - err(entry) / err(S0) per run at the 95th percentile, averaged over the held-out runs. Numbers come from the audit and the released data in the repository.",
      `<span class="tag" id="bTask">-</span>`) + `<div id="bBody" class="muted">Loading the scoreboard...</div>`;
  },
  async onShow() {
    if (ov) return;
    [ov, info] = await Promise.all([req({ type: "bench_overview" }, 60000), req({ type: "bench_info" }, 60000)]);
    render();
  },
};

function render() {
  const a = info.audit || {}, d = a.data || {}, fl = a.floors || {}, cu = a.current || {}, gy = a.gyro || {}, q = ov.quality || {};
  $("#bTask", el).textContent = `${info.task_version || "?"} + task/3 options`;
  const board = cu.board || [];
  const best = board.filter((b) => b.entry !== "S0").reduce((x, b) => (b.gcr_p95_tail > (x ? x.gcr_p95_tail : -9) ? b : x), null);
  const ceil = cu.ceiling || {};
  const tile = (k, v, s, cls = "") => `<div class="tile ${cls}"><div class="k">${k}</div><div class="v">${v}</div><div class="d">${s}</div></div>`;
  const hours = ov.hours || {}, h1 = Object.values(hours.benchmark_v1 || {}).reduce((x, y) => x + y, 0), hc = (hours.console || {}).pass_h || 0;
  const head = `<div class="grid g5" style="margin-bottom:12px">
    ${tile("Headline: effort tail GCR p95", fx(best ? best.gcr_p95_tail : null, 3), `best physics baseline (${esc(best ? best.entry : "-")}), ceiling ${fx(ceil.gcr_p95_tail, 3)}`, "acc")}
    ${tile("Reference error, S0", fx(cu.S0_p95_A, 2) + "<small>A p95</small>", `tail ${fx(cu.S0_tail_p95_A, 2)} A, elbow current`)}
    ${tile("Real-real floor", fx(fl.current_p95_A, 3) + "<small>A</small>", `gyro ${fx(fl.gyro_p95_rad_s, 4)} rad/s, tool ${fx(fl.tcp_p95_mm, 3)} mm`)}
    ${tile("Gyro K0 GCR, task/2 > task/3", `${fx((gy.rows || {}).K0_gcr_p95, 2)}<small>&gt;</small>${fx((gy.rows_task3 || {}).K0_gcr_p95, 3)}`, "rows held per sample, then own timestamps minus the lag", "warn")}
    ${tile("Data", `${d.runs ?? "-"}<small>runs audited</small>`, `${d.cells} cells, ${d.held_out_runs} held-out runs, ${fx(h1, 2)} h released`)}</div>`;

  // tracks x channels
  const ch = [
    ["effort", "Joint effort", "elbow motor current, A", "primary, ranked (headline)", "S0", `${fx(fl.current_p95_A, 3)} A`, `${fx(cu.S0_p95_A, 2)} A`, best ? `${esc(best.entry)} ${fx(best.gcr_p95, 3)} / tail ${fx(best.gcr_p95_tail, 3)}` : "-", `${fx(ceil.gcr_p95, 3)} / ${fx(ceil.gcr_p95_tail, 3)}`, "ok"],
    ["gyro", "Tool angular rate", "IMU gyroscope, own axes, rad/s", "secondary, ranked", "S0", `${fx(fl.gyro_p95_rad_s, 4)} rad/s`, `${fx((gy.rows_task3 || {}).S0_p95_rad_s, 3)} rad/s`, `K0 ${fx((gy.rows_task3 || {}).K0_gcr_p95, 3)} (task/3)`, "-", "ok"],
    ["vis", "Tool position, seen", "tool camera on a ChArUco board, m", "reported, ranked once the vision gate passes", "K0", `${fx(fl.tcp_p95_mm, 3)} mm (encoders)`, "-", "needs the camera tool in the campaign", "-", "warn"],
    ["accel", "Tool acceleration", "IMU accelerometer, m/s2", "provisional, not ranked", "S0", "-", "-", "until the six-face residual < 0.5 %", "-", "warn"],
    ["pos", "Tool position (encoders)", "controller FK, mm", "kinematics check, not the headline", "S0", `${fx(fl.tcp_p95_mm, 3)} mm`, "-", `K0 ${fx(((info.leaderboard || {}).entries || [])[0] ? info.leaderboard.entries[0].gcr_p95 : null, 3)}`, "-", ""],
  ];
  const matrix = panel("Tracks and channels", "flow", `<div class="grid g2" style="margin-bottom:12px">
      <div class="note acc"><b>Track A.</b> ${esc((ov.tracks || {}).A || "")}. Submit a simulator (a MuJoCo model, Isaac Sim, Unity PhysX...): it replays the controller's own target_q of every test run. Its torque goes through <b>its own</b> two-parameter current map fitted on the published cells (task/3).</div>
      <div class="note acc"><b>Track B.</b> ${esc((ov.tracks || {}).B || "")}. Submit predictions: per test input (clean and three stressed copies) the real current and gyro on the input's timestamps, optionally with sigma.</div></div>
    <div class="tbox" style="max-height:none"><table class="matrix"><thead><tr><th>Channel</th><th>Role</th><th>Reference</th><th>Real-real floor (p95)</th><th>S0 error (p95)</th><th>Best so far</th><th>Ceiling (all / tail)</th></tr></thead><tbody>
    ${ch.map((r) => `<tr><td class="ch">${r[1]}<small>${r[2]}</small></td><td><span class="badge ${r[9]}">${r[3]}</span></td><td class="mono">${r[4]}</td><td class="mono">${r[5]}</td><td class="mono">${r[6]}</td><td class="mono small">${r[7]}</td><td class="mono">${r[8]}</td></tr>`).join("")}
    </tbody></table></div>`);

  // effort board with intervals
  const rows = board.map((b) => ({ ...b, name: NAMES[b.entry] || b.name || b.entry }));
  const lo = Math.min(-0.2, ...rows.map((r) => Math.min(r.gcr_p95, r.gcr_p95_tail, (r.ci95 || [0])[0]))), hi = 1;
  const effort = panel("Effort track: every entry on the held-out runs", "bench", `<div class="legend" style="margin-bottom:10px"><span><i style="background:var(--accent)"></i>all samples, GCR p95</span><span><i style="background:var(--s2)"></i>tail samples (headline)</span><span><i style="border-left:1px solid var(--text-2);border-right:1px solid var(--text-2);height:9px;width:10px"></i>bootstrap 95 % interval</span><span><i style="border-left:2px dashed var(--ok);height:12px;width:2px"></i>ceiling from the real-real floor</span></div>
    <div class="bars">${rows.map((r) => `<div class="barrow"><div class="nm">${esc(r.entry)}<small>${esc(r.name.split("·")[1] || r.name)}</small></div>
      ${bar(r.gcr_p95, lo, hi, { tv: r.gcr_p95_tail, ci: r.ci95, cit: r.ci95_tail, ceil: ceil.gcr_p95_tail })}<div class="val">${fx(r.gcr_p95, 3)} / <b>${fx(r.gcr_p95_tail, 3)}</b></div></div>`).join("")}
      <div class="barrow"><div class="nm">ceiling<small>1 - floor / S0 error</small></div>${bar(ceil.gcr_p95, lo, hi, { tv: ceil.gcr_p95_tail })}<div class="val">${fx(ceil.gcr_p95, 3)} / ${fx(ceil.gcr_p95_tail, 3)}</div></div></div>
    <div class="note" style="margin-top:12px">Paired, run by run: B2F beats F1 by ${fx((cu.paired || {}).B2F_minus_F1, 3)} (95 % interval ${((cu.paired || {}).ci95 || []).map((v) => fx(v, 3)).join(" to ")}), better on ${(cu.paired || {}).runs_B2F_better} of ${(cu.paired || {}).runs} runs. Entries whose intervals overlap are tied, not ranked.</div>`,
    `<span class="tag">36 held-out runs, elbow current</span>`);

  // by speed
  const bs = ov.by_speed || [];
  const speed = panel("By elbow speed", "gauge", `<div class="tbox" style="max-height:none"><table class="t"><thead><tr><th class="num">v (rad/s)</th><th class="num">S0 p95 (A)</th><th class="num">floor (A)</th><th style="width:45%">GCR: B2F (bar), F1 (tick), ceiling (dash)</th><th class="num">runs</th></tr></thead><tbody>
    ${bs.map((r) => `<tr><td class="num">${r.v}</td><td class="num">${fx(r.S0_p95, 3)}</td><td class="num">${fx(r.floor_p95, 3)}</td>
      <td>${bar(+r.B2F_gcr, 0, 1, { ceil: +r.ceiling, ci: [+r.F1_gcr, +r.F1_gcr + 0.004] })}<div class="small muted" style="margin-top:2px">B2F ${fx(r.B2F_gcr, 3)}, F1 ${fx(r.F1_gcr, 3)}, ceiling ${fx(r.ceiling, 3)}</div></td><td class="num">${r.runs}</td></tr>`).join("")}</tbody></table></div>`);

  // gyro rule
  const g2 = gy.rows || {}, g3 = gy.rows_task3 || {}, go = gy.own_stamps || {};
  const gyro = panel("Gyro track: what task/3 changes", "imu", `<div class="bars">
      <div class="barrow"><div class="nm">task/2 rows<small>latest IMU sample held per row</small></div>${bar(g2.K0_gcr_p95, 0, 1)}<div class="val">K0 ${fx(g2.K0_gcr_p95, 3)}</div></div>
      <div class="barrow"><div class="nm">own timestamps<small>no lag removed</small></div>${bar(go.K0_gcr_p95, 0, 1)}<div class="val">K0 ${fx(go.K0_gcr_p95, 3)}</div></div>
      <div class="barrow"><div class="nm">task/3<small>own timestamps minus the 12 ms lag</small></div>${bar(g3.K0_gcr_p95, 0, 1)}<div class="val">K0 ${fx(g3.K0_gcr_p95, 3)}</div></div></div>
    <div class="note" style="margin-top:12px">K0's error p95 falls from ${fx(g2.K0_p95_rad_s, 4)} to ${fx(g3.K0_p95_rad_s, 4)} rad/s: about six tenths of the "structural vibration" K0 could not explain was timing. What remains is still above the real-real floor (${fx(gy.real_real_p95_rad_s, 4)} rad/s), so the track keeps its signal.</div>`);

  // condition grid
  const grid = panel("Condition grid (E2): 6 speeds x 3 arm configurations x 3 trajectories", "overview", `<div class="row" style="margin-bottom:10px"><div class="seg" id="gMetric"><button data-m="runs" class="${gridMetric === "runs" ? "on" : ""}">Runs recorded</button><button data-m="cur" class="${gridMetric === "cur" ? "on" : ""}">Current floor (A)</button><button data-m="tcp" class="${gridMetric === "tcp" ? "on" : ""}">Tool floor (mm)</button><button data-m="gyro" class="${gridMetric === "gyro" ? "on" : ""}">Gyro floor</button></div>
      <span class="sp"></span><span class="small muted">dashed outline = held-out cell (inputs only in the public release)</span></div><div id="gHeat"></div>`);

  // scoring paths
  const flowA = ["Command", "the controller's target_q of every test run (input files)"], flow = (steps) => `<div class="flowchart">${steps.map(([b, t]) => `<div class="st"><b>${b}</b>${t}</div>`).join("")}</div>`;
  const paths = panel("How an entry is scored", "flow", `<h3 class="sub">Track A: a simulator</h3>${flow([flowA, ["Replay", "the entry's simulator, same target_q, same 125 Hz rows (sonair-run-1)"], ["Map", "torque to current, fitted on the published cells, per entry (task/3)"], ["Error", "per sample against the real run; p95 and median per run; the tail = |cmd qdd| >= half its peak"], ["GCR", "1 - err / err(S0) per run, mean over the held-out runs, bootstrap interval"]])}
      <h3 class="sub" style="margin-top:14px">Track B: predictions</h3>${flow([["Inputs", "test/<run>.input.jsonl: cmd + S0 sim channels, clean and three stressed copies"], ["Predict", "current, gyro (and optionally vis, sigma) on the input timestamps"], ["Validate", "python -m sonair_benchmark validate (locally, no key)"], ["Submit", "hidden evaluator: team key, 5 a day; public board on Codabench"], ["Report", "GCR all and tail, by speed, cross-site, robustness on stress, calibration of sigma"]])}
      <pre class="code" style="margin-top:12px">${esc(["# the real-data audit (needs the private full run set: held-out targets are not public)", "python -m sonair audit-v1 --runs <full bench_runs> --cal data/benchmark_v1/calib/imu_cal.json",
        "# a Jetson or console session into the platform run format", "python -m sonair session-export runs_jetson/<run_id>", "# the platform's benchmark package (task, evaluator, cards)", "cd cell_platform/benchmark && python -m sonair_benchmark --help"].join("\n"))}</pre>`);

  // collection and release
  const budget = (hours.budget || []);
  const coll = panel("Collection towards 300 h at UoN", "rec", `<div class="row" style="margin-bottom:6px"><span class="lbl-s">QC-PASS motion so far</span><span class="sp"></span><span class="mono">${fx(h1 + hc, 2)} h of ${hours.target_uon_h} h</span></div>
      <div class="prog" style="height:10px"><i style="width:${Math.min(100, ((h1 + hc) / (hours.target_uon_h || 300)) * 100).toFixed(2)}%;min-width:2px"></i></div>
      <div class="small t2" style="margin-top:6px">benchmark-v1: ${Object.entries(hours.benchmark_v1 || {}).map(([k, v]) => `${esc(k)} ${fx(v, 3)} h`).join(", ")}. This console: ${fx(hc, 3)} h PASS, ${fx((hours.console || {}).warn_h, 3)} h WARN, ${fx((hours.console || {}).mock_h, 3)} h mock (never counted).</div>
      <div class="tbox" style="max-height:none;margin-top:10px"><table class="t"><thead><tr><th>Category (landing plan 6.3)</th><th class="num">Budget (h)</th><th style="width:40%">Share of 300 h</th></tr></thead><tbody>
      ${budget.map(([k, v]) => `<tr><td>${esc(k)}</td><td class="num">${v}</td><td><div class="prog"><i style="width:${(v / 300 * 100).toFixed(1)}%;background:var(--line-3)"></i></div></td></tr>`).join("")}</tbody></table></div>`);
  const rel = panel("Release readiness (proposal requirements)", "check", `<ul class="checks">${(ov.release || []).map(([t, s]) => `<li class="${s === "met" ? "ok" : s === "partial" ? "wn" : "no"}"><span class="mk">${s === "met" ? "&#10003;" : s === "partial" ? "!" : "&#10005;"}</span><span>${esc(t)}</span><span class="badge ${s}">${s}</span></li>`).join("")}</ul>
    <div class="small muted" style="margin-top:8px">From the landing plan (6.1); the page lists it, it does not measure it.</div>`);
  const qual = panel("Data quality, benchmark-v1 robot side", "check", `<dl class="kv"><dt>Controller period anomalies</dt><dd>${q.ctrl_period_anomalies} in ${q.runs} runs</dd><dt>Stale rows</dt><dd>${q.stale_rows}</dd>
      <dt>Elbow tracking RMS (median)</dt><dd>${fx(q.elbow_track_rms_deg_median, 4)} deg</dd><dt>IMU samples that reached the run file</dt><dd>${fx(q.imu_unique_frac_median, 3)} median, ${fx(q.imu_unique_frac_min, 3)} worst: the reason the new recorder keeps every stream in its own file</dd></dl>`);

  // leaderboard (position, from the site)
  const lb = info.leaderboard;
  const lbp = lb && lb.entries ? panel("Benchmark site board (tool position, task/2)", "bench", `<div class="tbox"><table class="t"><thead><tr><th>Entry</th><th>Track</th><th class="num">GCR p95</th><th class="num">GCR median</th><th class="num">Worst cell</th><th class="num">Runs</th></tr></thead><tbody>
      ${lb.entries.map((e) => `<tr><td>${esc(e.name)}<div class="muted small">${esc(e.kind || "")}</div></td><td>${esc(e.track)}</td><td class="num">${fx(e.gcr_p95, 3)}</td><td class="num">${fx(e.gcr_median, 3)}</td><td class="num">${fx(e.worst_cell_gcr, 3)}</td><td class="num">${e.n_runs ?? "-"}</td></tr>`).join("")}</tbody></table></div>
      <div class="small muted" style="margin-top:6px">Position is a kinematics check: copying the command (K0) closes about 98 % of it, which is why effort and gyro carry the score.</div>`) : "";

  $("#bBody", el).className = "";
  $("#bBody", el).innerHTML = head + `<div class="stack">${matrix}
    <div class="grid" style="grid-template-columns:minmax(0,1.35fr) minmax(0,1fr);gap:12px">${effort}<div class="stack">${gyro}${qual}</div></div>
    <div class="grid g2">${speed}${grid}</div>${paths}<div class="grid g2">${coll}${rel}</div>${lbp}</div>`;
  $("#gMetric", el).onclick = (e) => { const b = e.target.closest("button"); if (!b) return; gridMetric = b.dataset.m; $$("#gMetric button", el).forEach((x) => x.classList.toggle("on", x === b)); heat(); };
  heat();
}

function heat() {
  const g = ov.grid || {}, sp = ov.speeds, arms = ov.arms, trajs = ov.trajs;
  const val = (c) => !c ? null : gridMetric === "runs" ? c.runs : c.floor ? (gridMetric === "cur" ? c.floor.cur_p95 : gridMetric === "tcp" ? c.floor.tcp_p95_mm : c.floor.gyro_p95) : null;
  const vals = Object.values(g).map(val).filter((v) => v !== null);
  const mx = Math.max(...vals, 1e-9), mn = Math.min(...vals, 0);
  const color = (v) => { if (v === null) return "var(--sunken)"; const t = (v - mn) / (mx - mn || 1); return `color-mix(in srgb, var(--accent) ${Math.round(12 + t * 70)}%, var(--surface-2))`; };
  const cols = arms.length * trajs.length;
  let h = `<div class="heat" style="grid-template-columns:52px repeat(${cols}, 1fr)"><div></div>`;
  arms.forEach((a) => (h += `<div class="hh" style="grid-column:span 3;border-bottom:1px solid var(--line)">${a.replace("_", " ")}</div>`));
  h += `<div></div>` + arms.map(() => trajs.map((t) => `<div class="hh">${t.replace(/_/g, " ").replace("point to point", "p2p")}</div>`).join("")).join("");
  sp.forEach((v) => {
    h += `<div class="hh">${v}</div>`;
    arms.forEach((a) => trajs.forEach((t) => {
      const c = g[`${v}|${a}|${t}`], x = val(c);
      h += `<div class="hc ${c && c.held_out ? "ho" : ""}" style="background:${color(x)}" title="${v} rad/s, ${a}, ${t}: ${c ? `${c.runs} runs (${Object.entries(c.by_session).map(([k, n]) => k + " " + n).join(", ")})${c.floor ? `, floor ${c.floor.cur_p95} A, ${c.floor.tcp_p95_mm} mm` : ""}${c.held_out ? ", held out" : ""}` : "not recorded"}">${x === null ? "" : gridMetric === "runs" ? x : fx(x, gridMetric === "gyro" ? 4 : 3)}</div>`;
    }));
  });
  $("#gHeat", el).innerHTML = h + `</div><div class="small muted" style="margin-top:6px">Rows: commanded elbow speed (rad/s). benchmark-v1 has 5 repeats per cell over three sessions; campaign v2 adds five payloads, frozen poses, sessions on different days.</div>`;
}

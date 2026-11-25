/* Acquisition: the benchmark's data collection, live. While the real robot runs, this page says whether the data is
   arriving as it should: robot packets against the controller's own clock, IMU samples against their output rate,
   camera frames, and the session's completeness; below, everything recorded so far (QC verdicts, motion hours,
   condition cells covered, the plan's progress). */
import { $, esc, fx, req, panel, pageHead, icon, Chart, SERIES } from "../core.js";

let el, chRate, chImu, poll = null;
const pct = (a, b) => (b ? Math.min(100, (100 * a) / b) : 0);

function big(id, k, d = "") {
  return `<div class="tile acq" id="${id}"><div class="k">${k}</div><div class="v" data-v>-</div><div class="d" data-d>${d}</div><div class="prog" data-p hidden><i></i></div></div>`;
}
function setBig(id, v, d, cls, p) {
  const t = $("#" + id, el); if (!t) return;
  t.className = "tile acq " + (cls || ""); $("[data-v]", t).innerHTML = v; $("[data-d]", t).innerHTML = d || "";
  const pr = $("[data-p]", t); pr.hidden = p === undefined || p === null; if (!pr.hidden) $("i", pr).style.width = p.toFixed(1) + "%";
}

export default {
  init(root) {
    el = root;
    el.innerHTML = pageHead("Benchmark", "Data acquisition", "What is arriving from the robot and the sensors right now, how complete the current session is, and everything recorded so far. Updated every second.",
      `<a class="btn" href="#record">${icon("rec", "i sm")}Record</a><a class="btn" href="#campaign">${icon("campaign", "i sm")}Campaign</a><a class="btn" href="#bench">${icon("bench", "i sm")}Scoreboard</a>`) + `
      <div class="acqbanner" id="aqBanner"><span class="dot"></span><b id="aqState">IDLE</b><span class="mono" id="aqRun">no session recording</span><span class="sp"></span><span class="mono" id="aqEl"></span></div>
      <div class="grid g4" style="margin-bottom:12px">
        ${big("aqRows", "Robot rows in this session", "one per controller cycle")}
        ${big("aqRtde", "RTDE stream, last 5 s", "")}
        ${big("aqImu", "IMU samples in this session", "")}
        ${big("aqCam", "Camera frames", "")}
      </div>
      <div class="grid g2" style="margin-bottom:12px">
        ${panel("Robot packets per second", "rtde", `<div class="legend" id="lgR"></div><div id="chR"></div>`, `<span class="tag" id="aqSrc">-</span>`)}
        ${panel("IMU samples per second", "imu", `<div class="legend" id="lgI"></div><div id="chI"></div>`)}
      </div>
      <div class="grid g3" style="margin-bottom:12px">
        ${big("aqTot", "Sessions recorded here", "")}
        ${big("aqHours", "QC-PASS motion", "mock sessions never counted")}
        ${big("aqToday", "Today", "")}
      </div>
      <div class="grid g2" style="align-items:start">
        <div class="stack">
          ${panel("Plan in progress", "campaign", `<div id="aqCamp" class="t2">No plan loaded.</div>`)}
          ${panel("Condition cells covered", "overview", `<div id="aqCells" class="row" style="gap:6px"></div><div class="muted small" style="margin-top:8px">From the cell label of each session (speed / arm configuration / trajectory). Green: at least one PASS.</div>`)}
        </div>
        ${panel("Latest sessions", "data", `<div class="tbox" style="max-height:420px"><table class="t"><thead><tr><th>Session</th><th>QC</th><th class="num">s</th><th>Cell</th></tr></thead><tbody id="aqRecent"></tbody></table></div>`, `<a class="btn sm ghost" href="#data">Open in Data</a>`)}
      </div>`;
    chRate = new Chart($("#chR", el), { window: 120, legend: $("#lgR", el), minSpan: 20, series: [{ name: "controller clock", color: SERIES()[0] }, { name: "host clock", color: SERIES()[1], dash: [4, 3] }] });
    chImu = new Chart($("#chI", el), { window: 120, legend: $("#lgI", el), minSpan: 20, series: [] });
  },
  onShow() {
    clearInterval(poll);
    const tick = async () => { if (!el.classList.contains("on")) { clearInterval(poll); poll = null; return; } paint(await req({ type: "acq_status" }, 8000)); };
    tick(); poll = setInterval(tick, 1000);
  },
};

function paint(r) {
  if (!r || !r.ok) return;
  const se = r.session, rec = se && se.state === "recording", s = r.rtde || {}, t = r.totals || {};
  $("#aqBanner", el).className = "acqbanner " + (rec ? "rec" : se && se.qc ? (se.qc === "PASS" ? "ok" : "warn") : "");
  $("#aqState", el).textContent = rec ? "RECORDING" : se ? `LAST SESSION ${se.qc || se.state}`.toUpperCase() : "IDLE";
  $("#aqRun", el).textContent = se ? se.run_id + (se.mock ? "  (mock)" : "") : "no session recorded yet";
  $("#aqEl", el).textContent = se ? `${fx(se.elapsed_s, 1)} s` : "";
  if (rec) {
    const p = pct(se.rows || 0, se.expected_rows || 0);
    setBig("aqRows", `${(se.rows || 0).toLocaleString("en-GB")}<small>of ${(se.expected_rows || 0).toLocaleString("en-GB")}</small>`, `${fx(p, 2)} % of the controller cycles so far`, p > 99.5 ? "ok" : p > 95 ? "warn" : "bad", p);
  } else setBig("aqRows", "-", "starts with a session", "");
  const srcs = Object.keys(s.sources || {});
  const rtdeOk = srcs.length === 1 && srcs[0] === "rtde";
  setBig("aqRtde", s.rate_ctrl_hz ? `${fx(s.rate_ctrl_hz, 1)}<small>Hz</small>` : "-",
    s.packets_window ? `missing cycles ${s.missing_cycles}, max gap ${fx(s.max_gap_ms, 1)} ms, jitter p99 ${fx(s.host_jitter_p99_ms, 1)} ms` : "no packets: is the link up?",
    !s.packets_window ? "bad" : !rtdeOk || s.missing_cycles ? "bad" : s.rate_ctrl_hz < 499 ? "warn" : "ok");
  $("#aqSrc", el).textContent = srcs.length ? srcs.join(" + ").toUpperCase() : "-"; $("#aqSrc", el).className = "tag " + (rtdeOk ? "ok" : srcs.length ? "bad" : "");
  const imus = Object.entries(r.imu || {});
  if (imus.length) {
    const inS = imus.map(([, u]) => u.in_session).filter((x) => x !== null && x !== undefined);
    const lowRate = imus.some(([, u]) => u.rate_hz && u.odr && u.rate_hz < 0.95 * u.odr) || imus.some(([, u]) => u.error || !u.alive);
    setBig("aqImu", rec && inS.length ? inS.reduce((a, b) => a + b, 0).toLocaleString("en-GB") : imus.map(([, u]) => fx(u.rate_hz, 0)).join(" / ") + "<small>Hz</small>",
      imus.map(([n, u]) => `${esc(n)}: ${fx(u.rate_hz, 0)} of ${u.odr} Hz${u.error ? ", " + esc(u.error) : ""}`).join("; "), lowRate ? "warn" : "ok");
  } else setBig("aqImu", "-", "no IMU streaming", "bad");
  const cams = Object.entries(r.cams || {});
  setBig("aqCam", cams.length ? cams.map(([, c]) => fx(c.fps, 0)).join(" / ") + "<small>fps</small>" : "-",
    cams.length ? cams.map(([n, c]) => `${n} ${c.kind}${c.streaming ? "" : " idle"}`).join(", ") + ". Not written into sessions yet." : "no camera running", cams.length ? "" : "");
  if (s.rate_ctrl_hz) chRate.push(r.t, [s.rate_ctrl_hz, s.rate_host_hz]);
  if (imus.length) {
    if (chImu.series.length !== imus.length) chImu.setSeries(imus.map(([n], i) => ({ name: n, color: SERIES()[(i + 2) % 6] })));
    chImu.push(r.t, imus.map(([, u]) => u.rate_hz));
  }
  setBig("aqTot", `${t.n ?? 0}`, `PASS ${t.pass} &middot; WARN ${t.warn} &middot; FAIL ${t.fail} &middot; mock ${t.mock}`, "", t.n ? pct(t.pass, t.n) : null);
  setBig("aqHours", `${fx(t.pass_h, 3)}<small>h</small>`, `of ${fx(t.all_h, 3)} h recorded in all`, "", t.all_h ? pct(t.pass_h, t.all_h) : null);
  setBig("aqToday", `${t.today_n}<small>sessions</small>`, `${fx(t.today_h, 3)} h of motion today (UTC)`, "");
  const c = r.campaign;
  if (c) {
    const done = c.steps.filter((x) => x.state === "done" || x.state === "refused").length, st = c.steps[c.index];
    $("#aqCamp", el).innerHTML = `<div class="row"><b>${esc(c.name)}</b><span class="sp"></span><span class="badge">${esc(c.state)}</span></div>
      <div class="prog" style="height:12px;margin:10px 0"><i style="width:${pct(done, c.steps.length).toFixed(1)}%"></i></div>
      <div class="small t2">${done} of ${c.steps.length} steps done${st ? `; next: <span class="mono">${esc(st.cmd)}</span> x${st.speed}, repeat ${st.rep}` : ""}. QC: ${c.steps.filter((x) => x.qc === "PASS").length} PASS, ${c.steps.filter((x) => x.qc && x.qc !== "PASS").length} other.</div>`;
  } else $("#aqCamp", el).textContent = "No plan loaded.";
  const cells = Object.entries(r.cells || {});
  $("#aqCells", el).innerHTML = cells.length ? cells.sort().map(([k, v]) => `<span class="tag ${v.pass ? "ok" : "warn"}" title="${v.n} sessions, ${v.pass} PASS">${esc(k)} &times;${v.n}</span>`).join("")
    : `<span class="muted">No session carries a cell label yet (Record page: condition cell).</span>`;
  $("#aqRecent", el).innerHTML = (r.recent || []).map((x) => `<tr><td class="mono small" style="white-space:nowrap">${esc(x.run_id)}</td><td><span class="badge ${esc(x.verdict || "")}">${esc(x.verdict || "-")}</span>${x.mock ? ' <span class="badge mock">mock</span>' : ""}</td>
    <td class="num">${fx(x.duration_s, 1)}</td><td class="small">${esc(x.cell_label || "-")}</td></tr>`).join("") || `<tr><td colspan="4" class="muted">Nothing recorded yet.</td></tr>`;
}

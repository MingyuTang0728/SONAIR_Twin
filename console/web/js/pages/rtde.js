/* RTDE: the robot data interface. What the controller granted, how well the stream arrives, any field live,
   raw captures of every field. Read only: it listens to the console's own robot link, never a second connection. */
import { $, $$, esc, fx, req, result, panel, pageHead, icon, Chart, SERIES, withToken, store } from "../core.js";

let el, chart, info = null, sel = new Set(store.get("rtde.watch", ["actual_q:2", "target_q:2", "actual_current:2"])), poll = null, fieldsPainted = "";

function hist(h) {
  if (!h) return "";
  const mx = Math.max(1, ...h.counts);
  return `<div style="display:flex;align-items:flex-end;gap:1px;height:70px">${h.counts.map((c, i) => `<div title="${fx(h.edges_ms[i], 2)} to ${fx(h.edges_ms[i + 1], 2)} ms: ${c}" style="flex:1;background:${h.edges_ms[i] >= 1.75 && h.edges_ms[i] < 2.25 ? "var(--accent)" : "var(--line-3)"};height:${Math.max(c ? 2 : 0, 70 * Math.sqrt(c / mx))}px"></div>`).join("")}</div>
    <div class="row small muted" style="justify-content:space-between"><span>0 ms</span><span>host receive period (sqrt scale)</span><span>10 ms</span></div>`;
}

export default {
  init(root) {
    el = root;
    el.innerHTML = pageHead("Cell", "RTDE interface", "The Real-Time Data Exchange stream as the console receives it: the recipe the controller granted, packet timing on both clocks, any field live at the full rate, and raw captures. Read only, from the platform link's own packets.",
      `<span class="tag" id="rtSrc">-</span><button class="btn" id="rtRefresh">Refresh</button>`) + `
      <div class="grid g6" id="rtTiles" style="margin-bottom:12px"></div>
      <div class="grid" style="grid-template-columns:minmax(0,1.2fr) minmax(0,1fr);gap:12px;margin-bottom:12px">
        ${panel("Live fields", "rtde", `<div class="legend" id="rtLeg"></div><div id="rtChart"></div><div class="muted small" style="margin-top:6px">Every packet of the last 20 s (controller clock), up to 8 fields. Tick fields in the table to plot them.</div>`,
          `<span class="tag" id="rtN">-</span>`)}
        ${panel("Packet timing", "gauge", `<div id="rtHist"></div><dl class="kv" id="rtKv" style="margin-top:12px"></dl>`, `<select id="rtWin" class="w100" style="margin-left:auto"><option value="5">5 s</option><option value="10" selected>10 s</option><option value="30">30 s</option><option value="60">60 s</option></select>`)}
      </div>
      <div class="grid" style="grid-template-columns:minmax(0,1.6fr) minmax(0,1fr);gap:12px">
        ${panel("Granted recipe", "file", `<div class="row" style="margin-bottom:8px"><input type="search" id="rtFilter" placeholder="filter fields" style="flex:1"><span class="muted small" id="rtCount"></span></div>
          <div class="tbox" style="max-height:520px"><table class="t"><thead><tr><th></th><th>Field</th><th>Type</th><th>Unit</th><th>Latest value</th><th>Meaning</th></tr></thead><tbody id="rtFields"></tbody></table></div>`)}
        <div class="stack">
          ${panel("Raw capture", "rec", `<div class="row"><label class="field"><span>Seconds</span><input type="number" id="capS" class="w80" value="10" min="0.5" max="600" step="0.5"></label>
              <label class="field grow"><span>Name</span><input type="text" id="capName" placeholder="e.g. urs_dry_run"></label>
              <button class="btn pri" id="capGo" style="align-self:flex-end">${icon("rec", "i sm")}Capture</button><button class="btn" id="capStop" style="align-self:flex-end">Stop</button></div>
            <div class="note" id="capMsg" style="margin-top:10px">Every field of every packet, one CSV row per controller cycle with the host receive time. For debugging the link; sessions (Record page) are the dataset.</div>
            <div class="tbox" style="margin-top:10px;max-height:220px"><table class="t"><thead><tr><th>File</th><th class="num">Size</th><th></th></tr></thead><tbody id="capList"></tbody></table></div>`)}
          ${panel("Link", "link", `<dl class="kv" id="rtLink"></dl>`)}
          ${panel("Rules", "check", `<ul class="small t2" style="margin:0;padding-left:18px;line-height:1.6">
            <li>Recording needs RTDE at 500 Hz. The platform link falls back to port 30003 when RTDE will not start; a session with any 30003 packet fails QC.</li>
            <li>Missing cycles are gaps in the controller's own timestamp (2 ms period); host jitter is the receive period minus the controller period.</li>
            <li>The delay is the transport time above its lower envelope: what the clock map (sonair.jetson.clock) removes.</li>
            <li>The speed slider opens a second RTDE connection (inputs) only when it is first used.</li></ul>`)}
        </div>
      </div>`;
    chart = new Chart($("#rtChart", el), { cls: "tall", legend: $("#rtLeg", el), empty: "tick a field in the recipe table" });
    $("#rtRefresh", el).onclick = () => refresh(true);
    $("#rtWin", el).onchange = () => refresh();
    $("#rtFilter", el).oninput = () => paintFields(true);
    $("#capGo", el).onclick = async () => { const r = await req({ type: "rtde_capture", seconds: +$("#capS", el).value, name: $("#capName", el).value.trim() }); if (result(r, `capturing ${r.seconds} s to ${r.file}`)) setTimeout(refresh, (r.seconds + 1.5) * 1000); };
    $("#capStop", el).onclick = async () => { result(await req({ type: "rtde_capture_stop" })); setTimeout(refresh, 800); };
    el.addEventListener("change", async (e) => {
      const c = e.target.closest("[data-w]"); if (!c) return;
      if (c.checked) { if (sel.size >= 8) { c.checked = false; return result({ ok: false, error: "at most 8 fields" }); } sel.add(c.dataset.w); } else sel.delete(c.dataset.w);
      store.set("rtde.watch", [...sel]); await req({ type: "rtde_watch", fields: [...sel] }); series();
    });
  },
  async onShow() {
    await req({ type: "rtde_watch", fields: [...sel] });
    refresh(true);
    clearInterval(poll); poll = setInterval(() => { if (!el.classList.contains("on")) { clearInterval(poll); poll = null; return; } series(); }, 300);
    this.tick = setInterval(() => { if (!el.classList.contains("on")) { clearInterval(this.tick); return; } refresh(); }, 2000);
  },
};

async function series() {
  const r = await req({ type: "rtde_series", max_points: 1600 }, 5000);
  if (!r.ok) return;
  const names = r.names.join("|");
  if (chart._names !== names) { chart.setSeries(r.names.map((n, i) => ({ name: n, color: SERIES()[i % 6], dash: i >= 6 ? [5, 3] : [] }))); chart._names = names; }
  chart.setData(r.t, r.y);
  $("#rtN", el).textContent = `${r.t.length} pts`;
}

async function refresh(full) {
  const r = await req({ type: "rtde_info", window_s: +$("#rtWin", el).value }, 10000);
  if (!r.ok) return;
  info = r;
  const s = r.stats || {}, h = r.health || {}, L = r.link || {};
  const tile = (k, v, d, cls = "") => `<div class="tile sm ${cls}"><div class="k">${k}</div><div class="v">${v}</div><div class="d">${d}</div></div>`;
  const src = Object.keys(s.sources || {});
  $("#rtSrc", el).textContent = src.length ? src.join(" + ").toUpperCase() : (h.source || "none").toUpperCase();
  $("#rtSrc", el).className = "tag " + (src.length === 1 && src[0] === "rtde" ? "ok" : src.length ? "bad" : "");
  $("#rtTiles", el).innerHTML = tile("Rate, controller", s.rate_ctrl_hz ? fx(s.rate_ctrl_hz, 1) + "<small>Hz</small>" : "-", `requested ${L.requested_hz || "-"} Hz`, s.rate_ctrl_hz >= 499 ? "ok" : s.rate_ctrl_hz ? "warn" : "")
    + tile("Period", s.period_ms ? fx(s.period_ms, 3) + "<small>ms</small>" : "-", "controller timestamp")
    + tile("Missing cycles", s.missing_cycles ?? "-", `max gap ${fx(s.max_gap_ms, 1)} ms`, s.missing_cycles ? "bad" : s.packets_window ? "ok" : "")
    + tile("Host jitter p99", s.host_jitter_p99_ms !== undefined ? fx(s.host_jitter_p99_ms, 2) + "<small>ms</small>" : "-", `max ${fx(s.host_jitter_max_ms, 1)} ms`)
    + tile("Delay p95", s.delay_p95_ms !== undefined ? fx(s.delay_p95_ms, 2) + "<small>ms</small>" : "-", `above the envelope, max ${fx(s.delay_max_ms, 1)} ms`)
    + tile("Packets", s.packets_window ?? "-", `in ${s.window_s} s, ${Object.entries(s.sources || {}).map(([k, v]) => k + " " + v).join(", ") || "-"}`);
  $("#rtHist", el).innerHTML = hist(s.host_period_hist);
  $("#rtKv", el).innerHTML = `<dt>Rate on the host clock</dt><dd>${fx(s.rate_host_hz, 2)} Hz</dd><dt>Host jitter p50</dt><dd>${fx(s.host_jitter_p50_ms, 3)} ms</dd>
    <dt>Delay p50</dt><dd>${fx(s.delay_p50_ms, 3)} ms</dd><dt>Repeated timestamps</dt><dd>${s.repeats ?? "-"}</dd>`;
  $("#rtLink", el).innerHTML = `<dt>Host</dt><dd>${esc(L.host || "-")}</dd><dt>Connected</dt><dd>${L.connected ? "yes" : "no"} (${esc(h.mode || "-")} reader)</dd>
    <dt>Controller</dt><dd>${esc(h.controller_version || "-")}</dd><dt>Message</dt><dd>${esc(h.controller_message || "-")}</dd><dt>Packets total</dt><dd>${h.packets ?? "-"}</dd>
    <dt>Errors / reconnects</dt><dd>${h.errors ?? "-"} / ${h.reconnects ?? "-"}</dd><dt>RTDE drops</dt><dd>${h.rtde_drops ?? 0}</dd>
    <dt>Agent stalls</dt><dd>${h.host_stalls ?? 0} (worst ${fx(h.host_stall_max_s, 2)} s)</dd><dt>Dropped fields</dt><dd>${esc((h.dropped_fields || []).join(", ") || "none")}</dd>
    ${h.rtde_unavailable_reason ? `<dt>RTDE refused</dt><dd>${esc(h.rtde_unavailable_reason)}</dd>` : ""}${L.last_error ? `<dt>Last error</dt><dd>${esc(L.last_error)}</dd>` : ""}`;
  const cap = r.capture || {};
  if (cap.running) $("#capMsg", el).innerHTML = `Capturing <b>${esc(cap.file)}</b>: ${cap.rows} rows, ${fx(cap.remaining_s, 1)} s left.`;
  $("#capList", el).innerHTML = (r.captures || []).map((c) => `<tr><td class="mono small">${esc(c.file)}</td><td class="num">${fx(c.bytes / 1e6, 2)} MB</td><td><a class="btn sm" href="${withToken("files/" + encodeURIComponent(c.file))}" download>Download</a></td></tr>`).join("")
    || `<tr><td colspan="3" class="muted">No capture yet.</td></tr>`;
  paintFields(full);
}

function paintFields(force) {
  if (!info) return;
  const f = ($("#rtFilter", el).value || "").toLowerCase();
  const rows = info.fields.filter((x) => !f || x.name.toLowerCase().includes(f) || (x.what || "").toLowerCase().includes(f));
  $("#rtCount", el).textContent = `${rows.length} of ${info.fields.length} fields`;
  const html = rows.map((x) => {
    const boxes = x.n > 1 ? Array.from({ length: x.n }, (_, i) => `<label class="chk" title="${esc(x.name)}[${i}]"><input type="checkbox" data-w="${esc(x.name)}:${i}" ${sel.has(x.name + ":" + i) ? "checked" : ""}>${i}</label>`).join(" ")
      : x.n === 1 ? `<input type="checkbox" data-w="${esc(x.name)}" ${sel.has(x.name) ? "checked" : ""}>` : "";
    const val = Array.isArray(x.value) ? x.value.map((v) => fx(v, Math.abs(v) >= 100 ? 2 : 4)).join("  ") : esc(x.value ?? "-");
    return `<tr><td style="white-space:nowrap">${boxes}</td><td class="mono small" style="white-space:nowrap">${esc(x.name)}</td><td class="small muted">${esc(x.type)}</td><td class="small" style="white-space:nowrap">${esc(x.unit)}</td><td class="mono small" style="white-space:nowrap">${val}</td><td class="small t2" style="min-width:180px">${esc(x.what)}</td></tr>`;
  }).join("") || `<tr><td colspan="6" class="muted">No fields: connect the robot link first.</td></tr>`;
  if (force || !$("#rtFields", el).contains(document.activeElement)) $("#rtFields", el).innerHTML = html;
}

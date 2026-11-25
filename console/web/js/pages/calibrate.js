/* Calibrate: hand-eye for the collar camera, the accelerometer six faces, the IMU mounting and lag from a session,
   and the archive of every calibration run on this cell. The platform's own calib modules do the work. */
import { $, $$, esc, fx, req, result, confirmBox, panel, pageHead, icon, S, withToken, MOVE_CHECKS } from "../core.js";

let el, st = null, cam = "D435i", unit = "tool", viewLoop = false, det = null;
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const tile = (id, k, d = "") => `<div class="tile sm"><div class="k">${k}</div><div class="v" id="${id}">-</div><div class="d">${d}</div></div>`;

export default {
  init(root) {
    el = root;
    el.innerHTML = pageHead("Experiment", "Calibrate", "Every calibration a benchmark session depends on, run here and archived with its inputs and result, passed or failed. Hand-eye uses the platform's handeye module (three solvers compared, target spread checked); the IMU uses imu_align and accel_cal.",
      `<button class="btn sm" data-dock="move">Move</button><button class="btn sm" data-dock="grip">Gripper</button><button class="btn sm" data-dock="cam">Camera</button><span class="tag" id="cvTag">OpenCV -</span>`) + `
      <div class="flow" id="calFlow"><div data-f="he"><b>1</b>Camera on the tool (hand-eye)</div><div data-f="acc"><b>2</b>Accelerometer, six faces</div><div data-f="imu"><b>3</b>IMU mounting and lag</div><div data-f="use"><b>4</b>In use for the next session</div></div>

      <div class="grid" style="grid-template-columns:minmax(0,1.05fr) minmax(0,1fr);gap:12px;align-items:start;margin-bottom:12px">
        <div class="panel"><header><h2>${icon("camera")}Live view</h2><select id="hcCam" class="w140" style="margin-left:auto"><option value="D435i">D435i, collar (eye in hand)</option></select><span class="tag" id="hcSeen">looking</span></header><div class="pb">
          <div class="cam" style="aspect-ratio:auto;min-height:260px"><canvas id="hcCv"></canvas><div class="ce" id="hcEmpty">Start the cameras (Cameras page).</div></div>
          <div class="note" id="hcLive" style="margin-top:10px">Hold the board so the whole of it is in view.</div>
          <div class="note" style="margin-top:8px">The D405 is calibrated the same way when the gripper carries the camera tool (Mode B, sequence CAL_B). Docked it is a fixed camera: that is the eye-to-hand form (sequence CAL_A), which this page does not solve yet.</div>
        </div></div>
        <div class="stack">
          ${panel("1 &middot; The board", "target", `<div class="grid g3"><label class="field"><span>Pattern</span><select id="bKind"><option value="charuco">ChArUco</option><option value="chessboard">Chessboard</option></select></label>
              <label class="field"><span>Squares across</span><input type="number" id="bCols" value="9"></label><label class="field"><span>Squares down</span><input type="number" id="bRows" value="6"></label>
              <label class="field"><span>Square (mm)</span><input type="number" id="bSq" value="15" step="0.5"></label><label class="field"><span>Marker (mm)</span><input type="number" id="bMk" value="11" step="0.5"></label>
              <label class="field"><span>Dictionary</span><input type="text" id="bDict" value="DICT_5X5_100"></label></div>
            <div class="row" style="margin-top:10px"><button class="btn" id="bApply">Use this board (clears the poses)</button><span class="muted small">Default: the ChArUco board of the scene (9 x 6 squares of 15 mm). For a chessboard give <b>inner corners</b>, not squares.</span></div>`)}
          ${panel("2 &middot; Poses", "arm", `<div class="row"><button class="btn" id="hLook" title="Mock cell only: put the collar camera over the paddle">Point at the board (mock)</button><button class="btn" id="hPlan">Plan poses from here</button><span class="sp"></span><span class="tag" id="hPlanTag">no plan</span></div>
            <div class="row" style="margin-top:8px"><button class="btn" id="hNext">Next pose</button><button class="btn pri" id="hAuto">Do it automatically</button><button class="btn go" id="hCap">Capture this pose</button>
              <button class="btn" id="hUndo">Undo</button><button class="btn danger" id="hClear">Clear</button></div>
            <div class="grid g4" style="margin-top:10px">${tile("hN", "Poses", "5 minimum, 12 comfortable")}${tile("hRot", "Largest rotation", "30 deg or more")}${tile("hAx", "Axis spread", "0.15 or more")}${tile("hRep", "Corner accuracy", "px, mean")}</div>
            <div class="note" id="hAdvice" style="margin-top:10px">Move the arm so the board is seen from a different angle each time: tilt 30 to 60 degrees about all three axes and vary the distance.</div>
            <div class="note warn" style="margin-top:8px">Planned poses are single slow linear moves (50 mm/s) through the platform link, inside the cell envelope; the arm moves on its own in automatic mode. STOP ends it.</div>`)}
          ${panel("3 &middot; Work it out", "check", `<button class="btn pri wide lg" id="hSolve">Work out where the camera is</button>
            <div class="grid g3" style="margin-top:10px">${tile("hT", "Offset", "mm from the flange")}${tile("hR", "Turned by", "deg, x y z")}${tile("hErr", "Board reconstructed to", "mm, target spread")}</div>
            <div class="note" id="hVerdict" style="margin-top:10px">Not solved yet.</div>
            <div class="row" style="margin-top:10px"><button class="btn go" id="hSave" disabled>Save and use it</button><span class="muted small" id="hSaveMsg"></span></div>`)}
        </div>
      </div>

      <div class="grid g2" style="margin-bottom:12px">
        ${panel("Accelerometer, six faces", "imu", `<div class="row" style="margin-bottom:10px"><label class="field"><span>Unit</span><select id="aUnit"></select></label><span class="muted small" style="align-self:flex-end">Rest the carrier on each face, hands off, and capture each. Bias and scale per axis (accel_cal).</span></div>
            <div class="faces" id="aFaces">${["x+", "x-", "y+", "y-", "z+", "z-"].map((f) => `<div class="face" data-face="${f}"><b>${f.toUpperCase().replace("-", "−")}</b><span>-</span></div>`).join("")}</div>
            <div class="kstrip" style="margin-top:10px"><div class="txt"><b>Facing up</b><span id="aLive">-</span></div><div><b>|a|</b><span id="aNorm">-</span><i>m/s2</i></div><div class="txt"><b>Still</b><span id="aStill">-</span></div></div>
            <div class="btngrid c3" style="margin-top:10px"><button class="btn pri" id="aCap">Capture face</button><button class="btn" id="aReset">Start again</button><button class="btn go" id="aFit">Fit &amp; save</button></div>
            <div class="note" id="aMsg" style="margin-top:10px">The accelerometer track stays reported-not-ranked until the six-face residual is under 0.5 % (benchmark-v1: 2.5 to 3.1 %).</div>`)}
        ${panel("IMU mounting and lag", "rtde", `<div class="row"><label class="field grow"><span>Recorded session (best: an excitation command)</span><select id="mSess"></select></label>
              <label class="field"><span>Unit</span><input type="text" id="mUnit" class="w80" value="tool"></label><button class="btn pri" id="mRun" style="align-self:flex-end">Work it out</button></div>
            <div class="grid g3" style="margin-top:10px">${tile("mLag", "Residual lag", "ms, after the clock maps")}${tile("mCorr", "Correlation", "|gyro| vs the flange")}${tile("mWin", "Windows", "used, spread ms")}</div>
            <div id="mOut" class="small t2" style="margin-top:10px">The gyro is matched to the flange rate from the measured joints (imu_align): first the time offset, then the rotation from the flange to the IMU's axes.</div>`)}
      </div>

      <div class="grid" style="grid-template-columns:minmax(0,1fr) minmax(0,1.6fr);gap:12px">
        ${panel("In use for the next session", "check", `<ul class="checks" id="useList"></ul>`)}
        ${panel("Archive", "file", `<div class="tbox" style="max-height:300px"><table class="t"><thead><tr><th>When (UTC)</th><th>What</th><th>Result</th><th>Detail</th></tr></thead><tbody id="arch"></tbody></table></div>`, `<button class="btn sm ghost" id="archRefresh">Refresh</button>`)}
      </div>`;

    $("#hcCam", el).onchange = (e) => { cam = e.target.value; };
    $("#bApply", el).onclick = async () => {
      const spec = { kind: $("#bKind", el).value, cols: +$("#bCols", el).value, rows: +$("#bRows", el).value, square_mm: +$("#bSq", el).value, marker_mm: +$("#bMk", el).value, dictionary: $("#bDict", el).value.trim() };
      result(await req({ type: "he_clear", cam, spec }), "board set, poses cleared"); refresh();
    };
    $("#hLook", el).onclick = async () => {
      if (!(await confirmBox("Point the camera at the board", `The arm moves (joint move) so the collar camera looks down at the paddle from about 320 mm. Mock cell only.${MOVE_CHECKS}`, "Move"))) return;
      result(await req({ type: "he_look" }), "moving over the board");
    };
    $("#hPlan", el).onclick = async () => { const r = await req({ type: "he_plan", cam }, 20000); result(r, `${r.n} poses planned (${r.stage})`); refresh(); };
    $("#hNext", el).onclick = async () => { if (!(await confirmBox("Move to the next pose", `One slow linear move to the next planned pose, then a capture.${MOVE_CHECKS}`, "Move"))) return; result(await req({ type: "he_next", cam }), "moving"); };
    $("#hAuto", el).onclick = async () => {
      if (!(await confirmBox("Visit every planned pose", `<p><b>The arm moves on its own</b> through every remaining planned pose, slowly, capturing at each. STOP (Esc) ends it.</p>${MOVE_CHECKS}`, "Start"))) return;
      result(await req({ type: "he_auto", cam }), "visiting the planned poses");
    };
    $("#hCap", el).onclick = async () => { const r = await req({ type: "he_capture", cam }, 30000); result(r, `pose ${r.n} captured: ${(r.detection || {}).n_corners} corners at ${fx((r.detection || {}).distance_mm, 0)} mm`); refresh(); };
    $("#hUndo", el).onclick = async () => { result(await req({ type: "he_undo", cam })); refresh(); };
    $("#hClear", el).onclick = async () => { if (await confirmBox("Clear every pose", "The captured poses of this calibration are discarded.")) { result(await req({ type: "he_clear", cam })); refresh(); } };
    $("#hSolve", el).onclick = async () => { $("#hVerdict", el).textContent = "Solving..."; const r = await req({ type: "he_solve", cam }, 120000); paintSolve(r); refresh(); };
    $("#hSave", el).onclick = async () => { const r = await req({ type: "he_save", cam }); result(r, `saved ${r.path}`); $("#hSaveMsg", el).textContent = r.ok ? r.path : r.error; refresh(); };
    $("#aUnit", el).onchange = (e) => { unit = e.target.value; refresh(); };
    $("#aCap", el).onclick = async () => { const r = await req({ type: "acc_capture", unit }); result(r, `face ${r.face}: |a| ${fx(r.norm, 3)} m/s2`); $("#aMsg", el).textContent = r.ok ? `Captured ${r.face}. Faces so far: ${r.faces.join(", ")}.` : r.error; refresh(); };
    $("#aReset", el).onclick = async () => { await req({ type: "acc_reset", unit }); refresh(); };
    $("#aFit", el).onclick = async () => {
      const r = await req({ type: "acc_fit", unit }); result(r, "six-face calibration saved");
      $("#aMsg", el).className = "note " + (r.ok ? (r.worst_after_err_pct < 0.5 ? "ok" : "warn") : "bad");
      $("#aMsg", el).innerHTML = r.ok ? `Bias ${r.bias.map((v) => fx(v, 4)).join(", ")} m/s2, scale ${r.scale.map((v) => fx(v, 4)).join(", ")}. Spread of |g| over the faces: ${r.before_spread_pct} % before, ${r.after_spread_pct} % after; worst ${r.worst_after_err_pct} %.` : esc(r.error);
      refresh();
    };
    $("#mRun", el).onclick = async () => {
      const id = $("#mSess", el).value; if (!id) return;
      $("#mOut", el).textContent = "Working it out...";
      const r = await req({ type: "imu_mount", run_id: id, unit: $("#mUnit", el).value.trim() || "tool" }, 120000);
      const lag = r.lag || {};
      $("#mLag", el).textContent = lag.ok ? fx(lag.lag_s * 1e3, 2) : "-"; $("#mCorr", el).textContent = lag.ok ? fx(lag.whole_record_corr, 4) : "-";
      $("#mWin", el).textContent = lag.ok ? `${lag.windows_used} / ${fx(lag.spread_s * 1e3, 2)}` : "-";
      $("#mOut", el).innerHTML = r.ok ? `<div class="note ok">Saved calibration/imu_${esc(r.unit)}.json${r.mock ? " (mock)" : ""}.</div><div class="mono small" style="margin-top:8px">R_flange_imu =<br>${r.R_flange_imu.map((row) => row.map((v) => fx(v, 4).padStart(8)).join(" ")).join("<br>")}</div>`
        : `<div class="note bad">${esc(r.error || "failed")}</div>`;
      refresh();
    };
    $("#archRefresh", el).onclick = refresh;
  },
  async onShow() {
    const imus = Object.keys((S.st && S.st.imu) || {});
    $("#aUnit", el).innerHTML = (imus.length ? imus : ["tool"]).map((u) => `<option ${u === unit ? "selected" : ""}>${esc(u)}</option>`).join("");
    const s = await req({ type: "data_sessions" });
    $("#mSess", el).innerHTML = (s.sessions || []).map((x) => `<option value="${esc(x.run_id)}">${esc(x.run_id)} (${esc(x.verdict || "-")})</option>`).join("") || `<option value="">no session recorded yet</option>`;
    refresh(); liveView();
    clearInterval(this.tick); this.tick = setInterval(() => { if (!el.classList.contains("on")) { clearInterval(this.tick); return; } refresh(); }, 1200);
  },
};

async function liveView() {
  if (viewLoop) return; viewLoop = true;
  const cv = $("#hcCv", el), g = cv.getContext("2d");
  let k = 0;
  while (el.classList.contains("on")) {
    try {
      const r = await fetch(withToken(`cam/${cam}/color.jpg`), { cache: "no-store" });
      if (r.status === 200) {
        const bmp = await createImageBitmap(await r.blob());
        if (cv.width !== bmp.width) { cv.width = bmp.width; cv.height = bmp.height; }
        g.drawImage(bmp, 0, 0); $("#hcEmpty", el).hidden = true;
        if (k++ % 4 === 0) req({ type: "he_detect", cam }, 6000).then((d) => (det = d));
        if (det && det.ok && det.corners) { g.fillStyle = "#3ddc84"; det.corners.forEach(([x, y]) => { g.beginPath(); g.arc(x, y, 3, 0, 7); g.fill(); }); }
        $("#hcSeen", el).textContent = det ? (det.ok ? `${det.n_corners} corners` : "board not found") : "looking";
        $("#hcSeen", el).className = "tag " + (det && det.ok ? "ok" : "warn");
        $("#hcLive", el).textContent = det ? (det.ok ? `Board found: ${det.n_corners} corners at ${fx(det.distance_mm, 0)} mm, ${fx(det.reprojection_px, 2)} px reprojection.` : det.error || "Board not found.") : "Hold the board so the whole of it is in view.";
      } else { $("#hcEmpty", el).hidden = false; }
    } catch { $("#hcEmpty", el).hidden = false; }
    await sleep(160);
  }
  viewLoop = false;
}

async function refresh() {
  const r = await req({ type: "cal_status", cam, unit }, 8000);
  if (!r.ok) return;
  st = r;
  $("#cvTag", el).textContent = r.cv.ok ? "OpenCV ready" : "OpenCV missing"; $("#cvTag", el).className = "tag " + (r.cv.ok ? "ok" : "bad"); $("#cvTag", el).title = r.cv.error || "";
  const h = r.handeye || {}, rd = h.readiness || {};
  $("#hN", el).textContent = h.n ?? 0; $("#hRot", el).textContent = rd.max_rotation_deg !== undefined ? fx(rd.max_rotation_deg, 0) + "°" : "-";
  $("#hAx", el).textContent = fx(rd.axis_spread, 2); $("#hRep", el).textContent = fx(rd.mean_reprojection_px, 2);
  $("#hAdvice", el).className = "note " + (rd.ready ? "ok" : "");
  if (r.msg && r.busy) $("#hAdvice", el).textContent = r.msg; else if (rd.advice && rd.advice.length) $("#hAdvice", el).textContent = rd.advice.join(" ");
  const pl = h.plan; $("#hPlanTag", el).textContent = pl ? `${pl.stage}: ${pl.next} of ${pl.poses.length} visited` : "no plan";
  $("#hPlanTag", el).className = "tag " + (pl ? "acc" : "");
  ["#hLook"].forEach((id) => ($(id, el).hidden = !r.mock_truth));
  if (h.result) paintSolve({ ok: true, ...h.result }, true);
  // flow
  const use = r.in_use || {};
  const done = { he: !!(use["handeye_D435i.json"] && !use["handeye_D435i.json"].missing), acc: !!(use["accel_tool.json"] && !use["accel_tool.json"].missing), imu: !!(use["imu_tool.json"] && !use["imu_tool.json"].missing) };
  $$("#calFlow > div", el).forEach((d) => { const f = d.dataset.f; d.className = done[f] ? "done" : ""; });
  $("#useList", el).innerHTML = Object.entries(use).map(([k, v]) => `<li class="${v.missing ? "wn" : v.ok === false ? "no" : "ok"}"><span class="mk">${v.missing ? "!" : "&#10003;"}</span>
    <span>${esc(v.label)}<div class="why">${v.missing ? "not calibrated on this cell yet" : `${esc(v.utc || "")}${v.verdict ? " &middot; " + esc(String(v.verdict).split(".")[0]) : ""}${v.lag_ms !== undefined ? " &middot; lag " + v.lag_ms + " ms" : ""}`}</div></span><span class="mono small muted">${esc(k)}</span></li>`).join("");
  $("#arch", el).innerHTML = (r.archive || []).map((a) => `<tr><td class="mono small">${esc(a.utc)}</td><td>${esc(a.kind)}</td><td><span class="badge ${a.ok ? "ok" : "bad"}">${a.ok ? "passed" : "failed"}</span></td>
    <td class="small t2">${esc(Object.entries(a.summary || {}).filter(([, v]) => v !== null && v !== undefined).map(([k, v]) => `${k} ${typeof v === "object" ? JSON.stringify(v) : v}`).join(", ").slice(0, 160))}</td></tr>`).join("")
    || `<tr><td colspan="4" class="muted">Nothing archived yet.</td></tr>`;
  // six faces
  const sf = r.sixface || {}, faces = sf.faces || {}, live = sf.live || {};
  $$("#aFaces .face", el).forEach((f) => { const k = f.dataset.face, v = faces[k]; f.classList.toggle("got", !!v); f.classList.toggle("live", live.face === k);
    $("span", f).textContent = v ? fx(Math.hypot(...v), 3) : live.face === k ? "facing up" : "-"; });
  $("#aLive", el).textContent = live.face ? live.face.toUpperCase() : "not square"; $("#aNorm", el).textContent = fx(live.norm, 3); $("#aStill", el).textContent = live.still === undefined ? "-" : live.still ? "yes" : "moving";
}

function paintSolve(r, quiet) {
  if (!r.ok) { if (!quiet) { $("#hVerdict", el).className = "note bad"; $("#hVerdict", el).textContent = r.error || "failed"; } return; }
  $("#hT", el).textContent = (r.translation_mm || []).map((v) => fx(v, 1)).join(", ");
  $("#hR", el).textContent = (r.rotation_deg || []).map((v) => fx(v, 1)).join(", ");
  $("#hErr", el).textContent = r.target_rms_mm !== undefined ? fx(r.target_rms_mm, 3) : fx(r.target_spread_mm, 2);
  const good = /^Good/.test(r.verdict || "");
  $("#hVerdict", el).className = "note " + (good ? "ok" : "warn");
  $("#hVerdict", el).innerHTML = esc(r.verdict || "") + (r.mock_truth ? `<br><b>Mock check:</b> ${fx(r.mock_truth.translation_error_mm, 3)} mm and ${fx(r.mock_truth.rotation_error_deg, 4)} deg from the scene's true mount.` : "");
  $("#hSave", el).disabled = false;
}

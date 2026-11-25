/* Cameras: the D405 (camera tool) and the D435i (gripper collar). Colour and depth, frame timing, intrinsics,
   distance at a click, the calibration board found live, and a saved view with its pose. */
import { $, $$, esc, fx, req, result, panel, pageHead, icon, S, withToken } from "../core.js";

let el, loops = {}, status = {}, detect = { D405: false, D435i: false }, corners = {};
const NAMES = ["D435i", "D405"];

function card(n) {
  return `<div class="panel" id="cam-${n}"><header><h2>${icon("camera")}<span data-label>${n}</span></h2><span class="tag" data-st>off</span></header><div class="pb">
    <div class="camgrid">
      <div class="cam"><canvas data-s="color"></canvas><span class="cl">Colour</span><span class="cr" data-fps></span><div class="ce" data-e>Not streaming.</div></div>
      <div class="cam"><canvas data-s="depth"></canvas><span class="cl">Depth</span><span class="readout" data-ro hidden></span><div class="ce" data-e>Not streaming.</div></div>
    </div>
    <div class="row" style="margin-top:10px"><label class="chk"><input type="checkbox" data-det> Find the calibration board</label><span class="sp"></span>
      <button class="btn sm" data-snap>Save view with pose</button></div>
    <dl class="kv small" data-kv style="margin-top:10px"></dl></div></div>`;
}

async function frameLoop(n, stream) {
  const key = n + stream;
  if (loops[key]) return;
  loops[key] = true;
  const cv = $(`#cam-${n} canvas[data-s=${stream}]`, el), g = cv.getContext("2d"), empty = cv.parentElement.querySelector("[data-e]");
  while (loops[key] && el.classList.contains("on")) {
    const t0 = performance.now();
    try {
      const r = await fetch(withToken(`cam/${n}/${stream}.jpg`), { cache: "no-store" });
      if (r.status === 200) {
        const bmp = await createImageBitmap(await r.blob());
        if (cv.width !== bmp.width || cv.height !== bmp.height) { cv.width = bmp.width; cv.height = bmp.height; }
        g.drawImage(bmp, 0, 0); empty.hidden = true;
        if (stream === "color" && detect[n] && corners[n]) drawCorners(g, corners[n]);
      } else if (r.status === 404) { empty.hidden = false; empty.textContent = "No such camera on this agent."; await sleep(1500); }
      else { empty.hidden = false; empty.textContent = "Waiting for the first frame..."; }
    } catch { empty.hidden = false; empty.textContent = "No frame."; await sleep(1000); }
    await sleep(Math.max(30, 140 - (performance.now() - t0)));
  }
  loops[key] = false;
}
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
function drawCorners(g, c) {
  g.save();
  if (c.ok && c.corners) {
    g.fillStyle = "#3ddc84"; c.corners.forEach(([x, y]) => { g.beginPath(); g.arc(x, y, 3, 0, 7); g.fill(); });
    g.fillStyle = "rgba(0,0,0,.6)"; g.fillRect(6, g.canvas.height - 26, 300, 20); g.fillStyle = "#fff"; g.font = "12px monospace";
    g.fillText(`${c.n_corners} corners, ${fx(c.distance_mm, 0)} mm, ${fx(c.reprojection_px, 2)} px`, 12, g.canvas.height - 12);
  } else if (c.error) {
    g.fillStyle = "rgba(0,0,0,.6)"; g.fillRect(6, g.canvas.height - 26, g.canvas.width - 12, 20); g.fillStyle = "#f0c76b"; g.font = "12px monospace";
    g.fillText(String(c.error).slice(0, 80), 12, g.canvas.height - 12);
  }
  g.restore();
}

export default {
  init(root) {
    el = root;
    el.innerHTML = pageHead("Cell", "Cameras", "The two RealSense cameras of the kit: the D405 in the camera tool (scan camera, docked in Mode A, carried in Mode B) and the D435i on the gripper collar (localisation camera, always on tool0). Click the depth image to read the distance there.",
      `<button class="btn pri" id="camStart">${icon("camera", "i sm")}Start cameras</button><button class="btn" id="camStop">Stop</button>`) + `
      <div class="note" id="camNote" style="margin-bottom:12px">-</div>
      <div class="grid g2" style="margin-bottom:12px">${NAMES.map(card).join("")}</div>
      <div class="grid g2">
        ${panel("Timing and the session format", "rtde", `<ul class="small t2" style="margin:0;padding-left:18px;line-height:1.65">
          <li>Each frame keeps the camera's own timestamp and frame number (global time on), and the host receive time: the same rule as RTDE and the IMUs, mapped to the host clock afterwards.</li>
          <li>Camera frames are not written into a session yet: that comes with the camera tool in campaign v2 (milestone M4). Saved views (PNG + JSON with intrinsics and the optical frame in B) go to captures/camera/.</li>
          <li>Colour is used for the board (sub-pixel corners); depth is for part geometry (Task 2), not for the tool pose.</li></ul>`)}
        ${panel("Where they are", "cube", `<dl class="kv" id="camWhere"></dl><div class="note" style="margin-top:10px">The twin draws each camera's field of view (teal: D435i, orange: D405). Use <b>D435i eye</b> or <b>D405 eye</b> on the twin to look through it.</div>`)}
      </div>`;
    $("#camStart", el).onclick = async () => { const r = await req({ type: "cam_start" }, 30000); result(r, `cameras: ${(r.cameras || []).join(", ")}`); NAMES.forEach((n) => { frameLoop(n, "color"); frameLoop(n, "depth"); }); };
    $("#camStop", el).onclick = async () => result(await req({ type: "cam_stop" }), "cameras stopped");
    NAMES.forEach((n) => {
      const c = $(`#cam-${n}`, el);
      $("[data-det]", c).onchange = (e) => { detect[n] = e.target.checked; if (detect[n]) detLoop(n); else corners[n] = null; };
      $("[data-snap]", c).onclick = async () => { const r = await req({ type: "cam_snapshot", name: n }); result(r, r.ok ? `saved ${r.file}` : ""); };
      $("canvas[data-s=depth]", c).onclick = async (e) => {
        const cv = e.target, rc = cv.getBoundingClientRect(), u = ((e.clientX - rc.left) / rc.width) * cv.width, v = ((e.clientY - rc.top) / rc.height) * cv.height;
        const r = await req({ type: "cam_depth_at", name: n, u, v }); const ro = $("[data-ro]", c); ro.hidden = false;
        ro.textContent = r.ok ? (r.depth_m ? `${fx(r.depth_m * 1000, 1)} mm at (${u.toFixed(0)}, ${v.toFixed(0)})` : "no depth there") : r.error;
      };
    });
  },
  async onShow() {
    const r = await req({ type: "cam_status" });
    status = r.cams || {};
    $("#camNote", el).className = "note " + (Object.keys(status).length ? "" : "warn");
    $("#camNote", el).innerHTML = Object.keys(status).length ? (Object.values(status)[0].kind === "mock"
      ? "<b>Mock cameras.</b> Rendered from the twin's scene (pose from the rig and the measured joints, field of view from the scene, the ChArUco paddle where the scene puts it). Good for exercising detection and hand-eye with a known answer; not a photograph."
      : "RealSense cameras through pyrealsense2.") : esc(r.note || "No camera running. Start them; on the cell this needs pyrealsense2 and the cameras on USB 3.");
    NAMES.forEach((n) => { if (status[n]) { frameLoop(n, "color"); frameLoop(n, "depth"); } });
    this.tick = setInterval(async () => { if (!el.classList.contains("on")) { clearInterval(this.tick); return; } const s = await req({ type: "cam_status" }); status = s.cams || {}; paint(); }, 1500);
    paint();
  },
};

async function detLoop(n) {
  while (detect[n] && el.classList.contains("on")) {
    corners[n] = await req({ type: "he_detect", cam: n }, 8000);
    await sleep(600);
  }
}

function paint() {
  NAMES.forEach((n) => {
    const c = $(`#cam-${n}`, el), s = status[n];
    $("[data-label]", c).textContent = s ? s.label : n;
    $("[data-st]", c).textContent = !s ? "off" : s.streaming ? `${s.kind} ${fx(s.fps, 1)} fps` : `${s.kind} idle`;
    $("[data-st]", c).className = "tag " + (!s ? "" : s.error ? "bad" : s.streaming ? "ok" : "");
    $("[data-fps]", c).textContent = s && s.size ? `${s.size[0]} x ${s.size[1]}` : "";
    const i = (s && s.intrinsics) || {}, m = (s && s.meta) || {};
    $("[data-kv]", c).innerHTML = s ? `<dt>Intrinsics</dt><dd>fx ${fx(i.fx, 1)} fy ${fx(i.fy, 1)} cx ${fx(i.cx, 1)} cy ${fx(i.cy, 1)} (${esc(i.model || "")})</dd>
      <dt>Frame</dt><dd>#${m.frame_number ?? "-"} ${esc(m.timestamp_domain || "")}${m.exposure_us ? ", exposure " + m.exposure_us + " us" : ""}</dd>
      ${s.error ? `<dt>Error</dt><dd>${esc(s.error)}</dd>` : ""}` : `<dt>State</dt><dd>not running</dd>`;
  });
  const w = (n) => { const s = status[n]; return s && s.pose_B ? s.pose_B.slice(0, 3).map((r) => fx(r[3] * 1000, 0)).join(", ") + " mm" : "-"; };
  $("#camWhere", el).innerHTML = `<dt>D435i optical centre</dt><dd>${w("D435i")} (moves with tool0)</dd><dt>D405 optical centre</dt><dd>${w("D405")} (docked camera tool)</dd>`;
}

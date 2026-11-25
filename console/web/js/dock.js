/* The tool drawer under the live arm, on every page: move the robot, the gripper, a camera, the session.
   An experiment rarely needs one device: calibration needs the camera and the jog, a kit sequence the gripper and the
   session. The drawer keeps those within reach wherever the operator is. */
import { $, $$, S, req, result, fx, store, withToken, confirmBox } from "./core.js";

const CLOSED = 227, STROKE = 85;
let cam = store.get("dock.cam", "D435i"), stream = "color", camLoop = false;
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

export function openDock(tab) {
  const body = $("#dockBody");
  if (tab) {
    $$("#dockTabs [data-t]").forEach((b) => b.classList.toggle("on", b.dataset.t === tab));
    $$("#dockBody [data-dt]").forEach((d) => (d.hidden = d.dataset.dt !== tab));
    store.set("dock.tab", tab);
  }
  body.hidden = false; $("#dockCaret").innerHTML = "&#9660;"; store.set("dock.open", true);
  if ((tab || store.get("dock.tab", "move")) === "cam") camRun();
}
function closeDock() { $("#dockBody").hidden = true; $("#dockCaret").innerHTML = "&#9650;"; store.set("dock.open", false); }

async function camRun() {
  if (camLoop) return; camLoop = true;
  const cv = $("#dcCv"), g = cv.getContext("2d");
  while (!$("#dockBody").hidden && !$('#dockBody [data-dt="cam"]').hidden) {
    try {
      const r = await fetch(withToken(`cam/${cam}/${stream}.jpg`), { cache: "no-store" });
      if (r.status === 200) {
        const bmp = await createImageBitmap(await r.blob());
        if (cv.width !== bmp.width) { cv.width = bmp.width; cv.height = bmp.height; }
        g.drawImage(bmp, 0, 0); $("#dcEmpty").hidden = true; $("#dcInfo").textContent = `${cam} ${stream}`;
      } else { $("#dcEmpty").hidden = false; $("#dcEmpty").textContent = r.status === 404 ? "Camera not running: Start." : "Waiting for a frame..."; await sleep(400); }
    } catch { $("#dcEmpty").hidden = false; await sleep(800); }
    await sleep(160);
  }
  camLoop = false;
}

export function initDock() {
  $("#dockTabs").onclick = (e) => {
    const b = e.target.closest("[data-t]");
    if (b) { openDock(b.dataset.t); return; }
    if (e.target.closest("#dockCaret")) { $("#dockBody").hidden ? openDock() : closeDock(); }
  };
  // any element with data-dock="<tab>" anywhere on a page opens that tool
  document.addEventListener("click", (e) => { const a = e.target.closest("[data-dock]"); if (a) openDock(a.dataset.dock); });
  // gripper
  const move = async (mm) => result(await req({ type: "grip_move", pos: Math.round(CLOSED * (1 - mm / STROKE)), speed: 150, force: 64 }));
  $$("[data-dw]").forEach((b) => (b.onclick = () => move(+b.dataset.dw)));
  $("#dgConnect").onclick = async () => result(await req({ type: "grip_connect" }, 10000), "gripper connected");
  $("#dgAct").onclick = async () => result(await req({ type: "grip_activate" }, 10000));
  $("#dgStop").onclick = async () => result(await req({ type: "grip_stop" }));
  // camera
  $("#dcSel").onclick = (e) => { const b = e.target.closest("button"); if (!b) return; cam = b.dataset.c; store.set("dock.cam", cam); $$("#dcSel button").forEach((x) => x.classList.toggle("on", x === b)); };
  $("#dcStream").onclick = (e) => { const b = e.target.closest("button"); if (!b) return; stream = b.dataset.s; $$("#dcStream button").forEach((x) => x.classList.toggle("on", x === b)); };
  $$("#dcSel button").forEach((x) => x.classList.toggle("on", x.dataset.c === cam));
  $("#dcStart").onclick = async () => { result(await req({ type: "cam_start" }, 30000), "cameras started"); camRun(); };
  // session
  $("#drStart").onclick = async () => {
    if (!(await confirmBox("Start a passive session", "Records the robot and every IMU while you jog or a pendant program runs. Nothing moves by itself.", "Record"))) return;
    result(await req({ type: "session_start", motion: "passive", experiment: "PILOT", notes: "started from the tool drawer" }, 30000), "recording");
  };
  $("#drStop").onclick = async () => result(await req({ type: "session_stop" }, 60000), "session stopped");
  if (store.get("dock.open", false)) openDock(store.get("dock.tab", "move"));
  else openDock(store.get("dock.tab", "move")), closeDock();
}

export function dockState(m) {
  const g = m.gripper || {};
  $("#dgState").textContent = !g.connected ? "off" : g.fault ? `fault ${g.fault}` : g.activated ? (g.moving ? "moving" : "active") : "not activated";
  $("#dgState").className = "tag " + (!g.connected ? "" : g.fault ? "bad" : g.activated ? "ok" : "warn");
  $("#dgWidth").textContent = g.connected ? `${fx(g.width_mm, 1)} mm` : "- mm";
  $("#dgObj").textContent = g.connected ? `pos ${g.pos}${g.object ? ", holding" : ""}` : "";
  $("#dgBar").style.width = g.connected ? `${Math.min(100, (g.width_mm / STROKE) * 100).toFixed(0)}%` : "0";
  const se = m.session;
  $("#drState").textContent = se ? (se.state === "recording" ? `REC ${fx(se.elapsed_s, 0)} s` : se.qc || se.state) : "idle";
  $("#drState").className = "tag " + (se && se.state === "recording" ? "bad" : se && se.qc === "PASS" ? "ok" : se && se.qc ? "warn" : "");
  $("#drId").textContent = se ? se.run_id : "no session yet";
  $("#drQc").textContent = se && se.state !== "recording" && se.reasons ? se.reasons.slice(0, 3).join("; ") : "";
}

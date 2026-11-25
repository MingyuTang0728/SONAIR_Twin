"""
vision_track.py -- where the tool really is, measured by its own camera.

WHY THE CAMERA IS A BENCHMARK SENSOR AND NOT ONLY A RECORDER. Every position
the cell recorded so far comes from the robot itself: `tcp_pos` is the
controller's forward kinematics of its own joint encoders. That number cannot
disagree with the joints, so it cannot see anything the joints do not -- link
and gearbox compliance under load, the carrier's mounting, the deflection of
the arm in a hard stop. It also tracks the command so closely that copying the
command (K0) closes almost all of the "position gap", which is why position
was demoted to a kinematics check.

The tool camera is the one sensor in the cell that measures the tool's pose
from OUTSIDE the robot's own model. With a ChArUco board fixed in the cell and
the hand-eye calibration that already exists:

    T_base_tcp(seen) = T_base_board @ inv(T_cam_board(t)) @ inv(T_tcp_cam)

Every quantity on the right is measured: T_base_board and T_tcp_cam by the
hand-eye calibration, T_cam_board by this frame. The result is a position
target the simulator and the encoders can both be wrong about -- the
low-frequency, drift-free complement of the IMU, which is high-frequency and
drifts.

WHY COLOUR AND NOT DEPTH, for this. A board's pose from ~100 corners is good
to a fraction of a millimetre at half a metre; the D435i's stereo depth at the
same range is good to roughly half a millimetre to a millimetre per pixel. Depth
adds nothing here and is kept for what only it can do: the geometry of a part
(Task 2, docs).

WHAT IT MUST NOT DO. Like the video recorder, it never turns on the camera's
full-rate processing and never blocks the camera loop: frames are offered,
copied only when kept, and dropped -- and counted -- when the detector falls
behind. OpenCV releases the interpreter lock while it detects.

Files, one per run, beside the video:

    vision_logs/<run_id>.vtp.jsonl
        line 1  {"_vtp": {intrinsics, board, T_tcp_cam, T_base_board, calib}}
        then    {"t", "n", "rep_px", "sharp", "rvec", "tvec", "exp_us"}  per frame
                {"t", "n": 0, "why"}                                      when not seen
        last    {"_summary": {...}}

The RAW board pose is stored, not the derived tool pose, so a later
recalibration can be applied to old runs offline (sonair_benchmark/vision.py).
"""
from __future__ import annotations

import json
import logging
import math
import queue
import threading
import time
from pathlib import Path

log = logging.getLogger("vision_track")

try:
    import numpy as np
    import cv2
    _HAS_CV = hasattr(cv2, "aruco")
    _CV_ERR = "" if _HAS_CV else "this OpenCV has no aruco module (pip install opencv-contrib-python)"
except Exception as _e:                              # noqa: BLE001
    np = cv2 = None
    _HAS_CV = False
    _CV_ERR = f"OpenCV not importable ({_e})"

HERE = Path(__file__).resolve().parent
HANDEYE = HERE / "calibration" / "handeye.json"
OUT_DIR = HERE / "vision_logs"
MIN_CORNERS = 8

# What a pre-session check calls good. The board is reconstructed from the
# hand-eye calibration and compared with where the controller says the tool is;
# a camera that was knocked, or a board that was moved, shows up here first.
CHECK_OK = (2.0, 0.5)          # mm, deg
CHECK_MARGINAL = (5.0, 1.5)


# ---------------------------------------------------------------------------
# geometry
# ---------------------------------------------------------------------------

def rotvec_to_matrix(rv):
    rv = np.asarray(rv, dtype=float).reshape(3)
    th = float(np.linalg.norm(rv))
    if th < 1e-12:
        return np.eye(3)
    k = rv / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def matrix_to_rotvec(R):
    R = np.asarray(R, dtype=float)
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    if math.pi - th < 1e-6:
        # 180 deg: axis from the symmetric part
        w, v = np.linalg.eigh((R + R.T) / 2)
        return v[:, int(np.argmax(w))] * th
    return th / (2 * math.sin(th)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0],
                                                R[1, 0] - R[0, 1]])


def pose_to_matrix(pose):
    T = np.eye(4)
    T[:3, :3] = rotvec_to_matrix(pose[3:6])
    T[:3, 3] = np.asarray(pose[:3], dtype=float)
    return T


def invert(T):
    T = np.asarray(T, dtype=float)
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return out


def board_pose(rvec, tvec):
    T = np.eye(4)
    T[:3, :3] = rotvec_to_matrix(rvec)
    T[:3, 3] = np.asarray(tvec, dtype=float).reshape(3)
    return T


def tool_in_base(T_cam_board, T_base_board, T_tcp_cam):
    """The tool pose the camera saw: T_base_board @ inv(T_cam_board) @ inv(T_tcp_cam)."""
    return np.asarray(T_base_board) @ invert(T_cam_board) @ invert(T_tcp_cam)


def pose_difference(Ta, Tb) -> tuple[float, float]:
    """(position mm, rotation deg) between two 4x4 poses."""
    d = float(np.linalg.norm(np.asarray(Ta)[:3, 3] - np.asarray(Tb)[:3, 3])) * 1000.0
    R = np.asarray(Ta)[:3, :3].T @ np.asarray(Tb)[:3, :3]
    return d, math.degrees(float(np.linalg.norm(matrix_to_rotvec(R))))


# ---------------------------------------------------------------------------
# the board
# ---------------------------------------------------------------------------

_BOARDS: dict = {}


def _board(spec):
    """(board, dictionary, detector-or-None), cached per board spec."""
    key = (spec.cols, spec.rows, spec.square_mm, spec.marker_mm, spec.dictionary)
    if key not in _BOARDS:
        adict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec.dictionary))
        board = cv2.aruco.CharucoBoard((int(spec.cols), int(spec.rows)),
                                       spec.square_mm / 1000.0, spec.marker_mm / 1000.0,
                                       adict)
        det = cv2.aruco.CharucoDetector(board) if hasattr(cv2.aruco, "CharucoDetector") else None
        _BOARDS[key] = (board, adict, det)
    return _BOARDS[key]


def _corner_list(c):
    return [[round(float(x), 1), round(float(y), 1)] for x, y in np.asarray(c).reshape(-1, 2)]


def charuco_detect(gray, spec, K=None, dist=None) -> dict:
    """
    ChArUco corners in one image and, given intrinsics, the board's pose.

    Works on the OpenCV 4.7+ API (CharucoDetector, matchImagePoints, solvePnP)
    and on the legacy one. A ChArUco board is used because each corner carries
    its own id: a board only partly in view still gives a correct pose, which a
    plain chessboard -- all of it or nothing -- cannot during a moving run.
    """
    if not _HAS_CV:
        return {"ok": False, "error": _CV_ERR}
    if gray is None:
        return {"ok": False, "error": "no camera image"}
    g = np.asarray(gray)
    if g.ndim == 3:
        g = cv2.cvtColor(g, cv2.COLOR_BGR2GRAY)
    try:
        board, adict, det = _board(spec)
    except Exception as e:                          # noqa: BLE001
        return {"ok": False, "error": f"ChArUco board not available in this OpenCV ({e})"}
    try:
        if det is not None:
            ch_c, ch_ids, _mc, m_ids = det.detectBoard(g)
        else:                                       # legacy OpenCV
            m_c, m_ids, _ = cv2.aruco.detectMarkers(g, adict)
            if m_ids is None or len(m_ids) < 2:
                return {"ok": False, "error": "fewer than 2 ArUco markers visible", "n_corners": 0}
            _n, ch_c, ch_ids = cv2.aruco.interpolateCornersCharuco(m_c, m_ids, g, board)
    except Exception as e:                          # noqa: BLE001
        return {"ok": False, "error": f"detection failed: {e}", "n_corners": 0}
    n = 0 if ch_ids is None else int(len(ch_ids))
    if n < 6:
        nm = 0 if m_ids is None else len(m_ids)
        return {"ok": False, "n_corners": n,
                "error": (f"only {n} ChArUco corners ({nm} markers) -- the board is "
                          "out of view, blurred or too small in the image")}
    out = {"ok": True, "n_corners": n, "corners": _corner_list(ch_c)}
    if K is None:
        return out
    K = np.asarray(K, dtype=float)
    dist = np.zeros(5) if dist is None else np.asarray(dist, dtype=float)
    try:
        if hasattr(board, "checkCharucoCornersCollinear") and \
                board.checkCharucoCornersCollinear(ch_ids):
            out.update(ok=False, error="the visible corners lie on one line: no pose")
            return out
        if hasattr(board, "matchImagePoints"):
            obj, img = board.matchImagePoints(ch_c, ch_ids)
        else:                                       # legacy OpenCV
            obj = np.asarray(board.chessboardCorners, dtype=np.float32)[np.asarray(ch_ids).reshape(-1)]
            img = np.asarray(ch_c, dtype=np.float32).reshape(-1, 1, 2)
        ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            out.update(ok=False, error="no pose from the corners")
            return out
        rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, dist, rvec, tvec)
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        rep = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img.reshape(-1, 2)) ** 2, axis=1))))
    except Exception as e:                          # noqa: BLE001
        out.update(ok=False, error=f"pose failed: {e}")
        return out
    T = board_pose(rvec.reshape(3), tvec.reshape(3))
    out.update(T_cam_target=T.tolist(), rvec=[float(v) for v in rvec.reshape(3)],
               tvec=[float(v) for v in tvec.reshape(3)], reprojection_px=round(rep, 3),
               distance_mm=round(float(np.linalg.norm(tvec)) * 1000.0, 1))
    return out


def sharpness(gray, corners=None) -> float:
    """Variance of the Laplacian over the board's bounding box: motion blur shows here."""
    g = np.asarray(gray)
    if g.ndim == 3:
        g = cv2.cvtColor(g, cv2.COLOR_BGR2GRAY)
    if corners:
        c = np.asarray(corners)
        x0, y0 = np.maximum(c.min(axis=0).astype(int) - 10, 0)
        x1, y1 = c.max(axis=0).astype(int) + 10
        g = g[y0:y1, x0:x1] if (y1 > y0 + 8 and x1 > x0 + 8) else g
    return float(cv2.Laplacian(g, cv2.CV_64F).var())


def intrinsics_dict(intr) -> dict | None:
    """A RealSense intrinsics object, a dict or None -> a plain dict."""
    if intr is None:
        return None
    if isinstance(intr, dict):
        d = dict(intr)
    else:
        d = {k: getattr(intr, k) for k in ("width", "height", "fx", "fy", "ppx", "ppy")
             if hasattr(intr, k)}
        if hasattr(intr, "cx"):
            d.setdefault("ppx", intr.cx)
            d.setdefault("ppy", intr.cy)
        d["coeffs"] = list(getattr(intr, "coeffs", [0, 0, 0, 0, 0]))
        model = getattr(intr, "model", None)
        if model is not None:
            d["model"] = str(model)
    d.setdefault("ppx", d.get("cx"))
    d.setdefault("ppy", d.get("cy"))
    return {k: (float(v) if isinstance(v, (int, float)) else v) for k, v in d.items()}


def K_dist(intr: dict):
    K = np.array([[intr["fx"], 0, intr["ppx"]], [0, intr["fy"], intr["ppy"]], [0, 0, 1]], float)
    c = list(intr.get("coeffs") or [])
    # RealSense colour is Brown-Conrady (OpenCV's model) or "inverse" for
    # depth; colour streams have zero coefficients on a D435i anyway.
    dist = np.asarray((c + [0.0] * 5)[:5], dtype=float)
    return K, dist


# ---------------------------------------------------------------------------
# the calibration this rests on
# ---------------------------------------------------------------------------

def load_calibration(path=HANDEYE) -> dict:
    """
    The hand-eye calibration, judged for tool tracking: it needs T_tcp_cam,
    the board's place in the base frame, and a ChArUco board.
    """
    p = Path(path)
    if not p.exists():
        return {"ok": False, "error": "no hand-eye calibration yet -- do it on the "
                                      "Calibrate page with a ChArUco board"}
    try:
        d = json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError) as e:
        return {"ok": False, "error": f"{p.name} is not readable: {e}"}
    tgt = d.get("target") or {}
    if tgt.get("kind") != "charuco":
        return {"ok": False, "error": "the hand-eye calibration used a "
                f"{tgt.get('kind', 'unknown')} board; tool tracking needs a ChArUco "
                "board (a moving camera sees only part of the board, and only ChArUco "
                "corners say which part)", "calib": d}
    if not d.get("T_base_target"):
        return {"ok": False, "error": "the hand-eye calibration predates board mapping "
                                      "(no T_base_target): solve it again", "calib": d}
    return {"ok": True, "T_tcp_cam": d["T_tcp_cam"], "T_base_target": d["T_base_target"],
            "target": tgt, "calib_version": d.get("calib_version", ""),
            "target_spread_mm": d.get("target_spread_mm"),
            "solved_utc": d.get("solved_utc", "")}


def _spec(target: dict):
    import handeye
    return handeye.TargetSpec(**{k: v for k, v in target.items()
                                 if k in handeye.TargetSpec.__dataclass_fields__})


# ---------------------------------------------------------------------------
# a single look: the pre-session check
# ---------------------------------------------------------------------------

def check(frame, intr, tcp_pose, cal: dict | None = None) -> dict:
    """
    Where the camera says the tool is, against where the controller says it
    is, with the arm STILL. Large disagreement means the camera was knocked,
    the board moved, or the hand-eye calibration is stale -- and every tool-pose
    measurement of the session would inherit it.
    """
    cal = cal or load_calibration()
    if not cal.get("ok"):
        return {"ok": False, "error": cal.get("error")}
    if intr is None or frame is None:
        return {"ok": False, "error": "no colour frame -- open the Camera page or the live view"}
    if not tcp_pose or len(tcp_pose) < 6:
        return {"ok": False, "error": "no tool pose from the robot"}
    intr = intrinsics_dict(intr)
    K, dist = K_dist(intr)
    det = charuco_detect(frame, _spec(cal["target"]), K, dist)
    if not det.get("ok") or "T_cam_target" not in det:
        return {"ok": False, "error": det.get("error") or "board not found",
                "n_corners": det.get("n_corners", 0)}
    T_seen = tool_in_base(det["T_cam_target"], cal["T_base_target"], cal["T_tcp_cam"])
    T_ctrl = pose_to_matrix(tcp_pose)
    dmm, ddeg = pose_difference(T_seen, T_ctrl)
    if dmm <= CHECK_OK[0] and ddeg <= CHECK_OK[1]:
        verdict, msg = "pass", (f"The camera and the controller agree on the tool to "
                                f"{dmm:.1f} mm / {ddeg:.2f} deg. Tool tracking is good to use.")
    elif dmm <= CHECK_MARGINAL[0] and ddeg <= CHECK_MARGINAL[1]:
        verdict, msg = "marginal", (f"{dmm:.1f} mm / {ddeg:.2f} deg between camera and "
                                    "controller. Usable, but check the board is rigid and "
                                    "the camera mount is tight; recalibrate if it grows.")
    else:
        verdict, msg = "fail", (f"{dmm:.1f} mm / {ddeg:.2f} deg between camera and "
                                "controller: the camera was moved, the board was moved, or "
                                "the TCP on the pendant changed. Redo the hand-eye "
                                "calibration before recording.")
    g = np.asarray(frame)
    return {"ok": True, "verdict": verdict, "message": msg,
            "diff_mm": round(dmm, 2), "diff_deg": round(ddeg, 3),
            "n_corners": det["n_corners"], "reprojection_px": det["reprojection_px"],
            "distance_mm": det["distance_mm"],
            "sharpness": round(sharpness(g, det.get("corners")), 1),
            "seen_pose_mm": [round(float(v) * 1000.0, 2) for v in T_seen[:3, 3]],
            "controller_pose_mm": [round(float(v) * 1000.0, 2) for v in T_ctrl[:3, 3]],
            "corners": det.get("corners", [])[::max(1, det["n_corners"] // 60)],
            "calib_version": cal.get("calib_version", "")}


def floor_from(poses: list) -> dict:
    """
    The measurement floor of tool tracking: the jitter of the seen tool pose
    with the arm still. A gap smaller than this is not a gap the camera can see.
    """
    if len(poses) < 10:
        return {"ok": False, "error": f"only {len(poses)} frames with the board seen"}
    P = np.asarray([np.asarray(T)[:3, 3] for T in poses]) * 1000.0
    c = P.mean(axis=0)
    d = np.linalg.norm(P - c, axis=1)
    R0 = np.asarray(poses[0])[:3, :3]
    ang = [math.degrees(float(np.linalg.norm(matrix_to_rotvec(R0.T @ np.asarray(T)[:3, :3]))))
           for T in poses]
    a = np.asarray(ang) - np.mean(ang)
    return {"ok": True, "frames": len(poses),
            "position_rms_mm": round(float(np.sqrt(np.mean(d ** 2))), 3),
            "position_p95_mm": round(float(np.percentile(d, 95)), 3),
            "per_axis_sd_mm": [round(float(v), 3) for v in P.std(axis=0)],
            "rotation_rms_deg": round(float(np.sqrt(np.mean(a ** 2))), 4)}


# ---------------------------------------------------------------------------
# every run: the tracker hook
# ---------------------------------------------------------------------------

class VisionTracker:
    """
    A run hook (start/stop) plus a frame tap (feed), like the video recorder.
    Active only when the hand-eye calibration supports tracking; otherwise
    start() says why and the run is recorded without it.
    """

    def __init__(self, out_dir=OUT_DIR, max_fps: float = 30.0, clock=None):
        self.out_dir = Path(out_dir)
        self.max_fps = float(max_fps)
        self.clock = clock or time.perf_counter
        self.enabled = True
        self.current: dict | None = None
        self.last: dict | None = None
        self.latest: dict | None = None      # the newest tool pose seen, for the live twin
        self._q: queue.Queue | None = None
        self._thread = None
        self._lock = threading.Lock()
        self._next_t = 0.0
        self.calibration_path = HANDEYE

    def configure(self, enabled=None) -> dict:
        if enabled is not None:
            self.enabled = bool(enabled)
        return self.status()

    def ready(self) -> dict:
        if not _HAS_CV:
            return {"ok": False, "error": _CV_ERR}
        return load_calibration(self.calibration_path)

    def active(self) -> bool:
        return self.current is not None

    def start(self, run_id: str, *, simulated: bool = False) -> dict:
        if not self.enabled:
            return {"ok": True, "skipped": "tool tracking is switched off"}
        if simulated:
            return {"ok": True, "skipped": "a simulated cell has no real camera"}
        cal = self.ready()
        if not cal.get("ok"):
            return {"ok": True, "skipped": cal.get("error")}
        with self._lock:
            if self.current is not None:
                self._finish_locked()
            self.out_dir.mkdir(parents=True, exist_ok=True)
            path = self.out_dir / f"{run_id}.vtp.jsonl"
            if path.exists():
                path.rename(path.with_name(f"{path.name}.{time.strftime('%Y%m%dT%H%M%S')}.superseded"))
            self._q = queue.Queue(maxsize=6)
            self.current = {"run_id": run_id, "path": str(path), "cal": cal,
                            "offered": 0, "frames": 0, "seen": 0, "dropped": 0,
                            "started_t": self.clock(), "error": "", "header": False}
            self._next_t = 0.0
            self._thread = threading.Thread(target=self._worker, args=(self._q, self.current),
                                            daemon=True, name=f"vtp-{run_id}")
            self._thread.start()
        return {"ok": True, "run_id": run_id, "path": str(path)}

    def stop(self) -> dict:
        with self._lock:
            if self.current is None:
                return {"ok": True, "skipped": "tool tracking was not running"}
            return self._finish_locked()

    def _finish_locked(self) -> dict:
        cur, q, th = self.current, self._q, self._thread
        self.current, self._q, self._thread = None, None, None
        if q is not None:
            try:
                q.put(None, timeout=1.0)
            except queue.Full:
                pass
        if th is not None:
            th.join(timeout=10.0)
        span = max(1e-6, self.clock() - cur["started_t"])
        summ = {"run_id": cur["run_id"], "path": cur["path"], "frames": cur["frames"],
                "seen": cur["seen"], "dropped": cur["dropped"],
                "seen_fraction": round(cur["seen"] / cur["frames"], 3) if cur["frames"] else 0.0,
                "fps": round(cur["frames"] / span, 1), "error": cur["error"]}
        try:
            with open(cur["path"], "a", encoding="utf-8") as fh:
                fh.write(json.dumps({"_summary": summ}) + "\n")
        except OSError:
            pass
        self.last = summ
        log.info("vision: %s -- %d frames, board seen in %d, %d dropped",
                 cur["run_id"], cur["frames"], cur["seen"], cur["dropped"])
        return {"ok": not cur["error"], **summ}

    def feed(self, frame_bgr, t=None, intr=None, exposure_us=None) -> None:
        """Offer one colour frame (raw, unaligned). Never blocks the camera loop."""
        cur, q = self.current, self._q
        if cur is None or q is None or frame_bgr is None:
            return
        t = self.clock() if t is None else float(t)
        period = 1.0 / self.max_fps
        if self._next_t and t < self._next_t - 0.25 * period:
            return
        self._next_t = self._next_t + period if (self._next_t and t - self._next_t < period) \
            else t + period
        cur["offered"] += 1
        if q.full():
            cur["dropped"] += 1
            return
        try:
            q.put_nowait((t, frame_bgr.copy(), intrinsics_dict(intr) if not cur["header"] else None,
                          exposure_us))
        except queue.Full:
            cur["dropped"] += 1

    def _worker(self, q, cur):
        cal = cur["cal"]
        spec = _spec(cal["target"])
        K = dist = None
        T_bb, T_tc = np.asarray(cal["T_base_target"]), np.asarray(cal["T_tcp_cam"])
        try:
            fh = open(cur["path"], "w", encoding="utf-8")
        except OSError as e:
            cur["error"] = f"cannot write {cur['path']}: {e}"
            return
        with fh:
            while True:
                item = q.get()
                if item is None:
                    break
                t, frame, intr, exp = item
                if not cur["header"]:
                    if intr is None:
                        continue                    # no intrinsics yet: wait for a frame with them
                    K, dist = K_dist(intr)
                    fh.write(json.dumps({"_vtp": {
                        "schema": "sonair-vtp/1", "run_id": cur["run_id"],
                        "clock": "t is the run file's own clock at frame arrival on the host",
                        "intrinsics": intr, "board": cal["target"],
                        "T_tcp_cam": cal["T_tcp_cam"], "T_base_target": cal["T_base_target"],
                        "handeye_calib": cal.get("calib_version", ""),
                        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}}) + "\n")
                    cur["header"] = True
                cur["frames"] += 1
                det = charuco_detect(frame, spec, K, dist)
                row = {"t": round(float(t), 6)}
                if exp is not None:
                    row["exp_us"] = float(exp)
                if det.get("ok") and det.get("rvec") is not None and det["n_corners"] >= MIN_CORNERS:
                    cur["seen"] += 1
                    row.update(n=det["n_corners"], rep_px=det["reprojection_px"],
                               sharp=round(sharpness(frame, det.get("corners")), 1),
                               rvec=[round(v, 7) for v in det["rvec"]],
                               tvec=[round(v, 7) for v in det["tvec"]])
                    try:
                        T = tool_in_base(det["T_cam_target"], T_bb, T_tc)
                        self.latest = {"t": float(t), "mono": time.monotonic(),
                                       "pos": [float(v) for v in T[:3, 3]], "T": T.tolist()}
                    except Exception:               # noqa: BLE001
                        pass
                else:
                    row.update(n=int(det.get("n_corners", 0) or 0),
                               why=str(det.get("error", "not seen"))[:80])
                fh.write(json.dumps(row, separators=(",", ":")) + "\n")

    def fresh(self, max_age_s: float = 0.15) -> dict | None:
        """The newest tool pose seen, if recent enough to compare with the robot now."""
        L = self.latest
        if L and time.monotonic() - L["mono"] <= max_age_s:
            return L
        return None

    def status(self) -> dict:
        r = self.ready()
        cur = self.current
        return {"enabled": self.enabled, "ready": bool(r.get("ok")),
                "why": "" if r.get("ok") else r.get("error", ""),
                "calib_version": r.get("calib_version", ""),
                "recording": None if not cur else {k: cur[k] for k in
                                                   ("run_id", "frames", "seen", "dropped")},
                "last": self.last}


TRACKER = VisionTracker()


# ---------------------------------------------------------------------------
# the console's messages
# ---------------------------------------------------------------------------

def measure_floor(grab, seconds: float = 4.0, cal: dict | None = None) -> dict:
    """
    `grab()` -> (frame, intrinsics). Frames for `seconds` with the arm still;
    the jitter of the seen tool pose is the floor of tool tracking.
    """
    cal = cal or load_calibration()
    if not cal.get("ok"):
        return {"ok": False, "error": cal.get("error")}
    spec = _spec(cal["target"])
    poses, last_id, t_end = [], None, time.monotonic() + float(seconds)
    while time.monotonic() < t_end:
        frame, intr = grab()
        if frame is None or intr is None or id(frame) == last_id:
            time.sleep(0.02)
            continue
        last_id = id(frame)
        K, dist = K_dist(intrinsics_dict(intr))
        det = charuco_detect(frame, spec, K, dist)
        if det.get("ok") and det.get("T_cam_target"):
            poses.append(tool_in_base(det["T_cam_target"], cal["T_base_target"], cal["T_tcp_cam"]))
        time.sleep(0.02)
    out = floor_from(poses)
    out["seconds"] = seconds
    out["calib_version"] = cal.get("calib_version", "")
    return out

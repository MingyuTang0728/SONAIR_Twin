"""The Calibrate page: every calibration a benchmark session depends on, run from the console, archived with its result.

    hand-eye        where a camera sits on the tool (or, for a fixed camera, where it sits in the cell). The platform's
                    own handeye.HandEyeSession does the work: board detection (chessboard or ChArUco), the readiness
                    check on the pose set, three AX = XB solvers compared, the spread of the reconstructed target.
                    Needs OpenCV (cv2 with aruco for ChArUco). Pose planning: handeye.plan_bootstrap_poses, then
                    handeye.plan_auto_poses once a rough answer exists. Moves are single slow movel through the
                    platform link, each confirmed in the page and refused by the console interlock while a command runs.
    accelerometer   six faces: accel_cal.fit from the still mean on each face (bias and scale per axis)
    IMU mount, lag  imu_align.estimate_lag and estimate_mount on a recorded session: the robot rows and the IMU samples
                    are put on the host clock with the session's own clock maps first (sonair.jetson.clock.ClockMap)

Results go to calibration/ (not committed: they belong to the cell, and each session's manifest records which file
it was recorded with). Every run, passed or failed, is archived in calibration/archive/.
"""
from __future__ import annotations

import json
import os
import time

import numpy as np

from cell_platform import paths
from sonair import cell

ROOT = cell.ROOT
CAL_DIR = os.path.join(ROOT, "calibration")
ARCHIVE = os.path.join(CAL_DIR, "archive")
PADDLE = dict(kind="charuco", cols=9, rows=6, square_mm=15.0, marker_mm=11.0, dictionary="DICT_5X5_100")


def _utc():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def archive(kind, ok, summary, detail=None):
    os.makedirs(ARCHIVE, exist_ok=True)
    ent = {"utc": _utc(), "kind": kind, "ok": bool(ok), "summary": summary, "detail": detail}
    fn = f"{time.strftime('%Y%m%d_%H%M%S')}_{kind}.json"
    json.dump(ent, open(os.path.join(ARCHIVE, fn), "w"), indent=1, default=_jd)
    return fn


def list_archive(n=60):
    if not os.path.isdir(ARCHIVE):
        return []
    out = []
    for f in sorted(os.listdir(ARCHIVE), reverse=True)[:n]:
        try:
            d = json.load(open(os.path.join(ARCHIVE, f)))
            out.append({"file": f, "utc": d.get("utc"), "kind": d.get("kind"), "ok": d.get("ok"), "summary": d.get("summary")})
        except Exception:
            continue
    return out


def in_use():
    """The calibration files the next session is recorded with."""
    out = {}
    for name, label in (("handeye_D435i.json", "Hand-eye, D435i on the collar"), ("handeye_D405.json", "Hand-eye, D405 camera tool"),
                        ("accel_tool.json", "Accelerometer six-face (tool IMU)"), ("imu_tool.json", "IMU mounting and lag (tool IMU)")):
        p = os.path.join(CAL_DIR, name)
        if os.path.exists(p):
            try:
                d = json.load(open(p))
                out[name] = {"label": label, "utc": d.get("saved_utc") or d.get("solved_utc") or d.get("made_at"),
                             "verdict": d.get("verdict"), "ok": d.get("ok", True)}
            except Exception:
                out[name] = {"label": label, "error": "unreadable"}
        else:
            out[name] = {"label": label, "missing": True}
    pub = os.path.join(ROOT, "data", "benchmark_v1", "calib", "imu_cal.json")
    if os.path.exists(pub):
        d = json.load(open(pub))
        out["benchmark_v1/imu_cal.json"] = {"label": "benchmark-v1 IMU lag (published, LPMS kit)", "utc": d.get("made_at"),
                                            "lag_ms": round(1e3 * ((d.get("lag") or {}).get("lag_s") or 0), 2)}
    return out


def _jd(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, np.generic):
        return o.item()
    return str(o)


# ---------------------------------------------------------------------------------------------- hand-eye
class HandEye:
    def __init__(self):
        paths.add()
        import handeye
        self.he = handeye
        self.sessions = {}
        self.plans = {}
        self.last_detect = {}

    def available(self):
        return self.he.available()

    def session(self, cam, spec=None):
        if cam not in self.sessions or spec:
            self.sessions[cam] = self.he.HandEyeSession(self.he.TargetSpec(**(spec or PADDLE)))
            self.plans.pop(cam, None)
        return self.sessions[cam]

    def detect(self, cam_name, camera, quick=True):
        ok, why = self.available()
        if not ok:
            return {"ok": False, "error": why}
        img = camera.latest_bgr() if camera else None
        if img is None:
            return {"ok": False, "error": "no camera image: open the camera stream first"}
        s = self.session(cam_name)
        r = self.he.detect_target(img, s.spec, camera.intr, effort="quick" if quick else "full")
        out = {k: r.get(k) for k in ("ok", "error", "n_corners", "distance_mm", "reprojection_px", "pitch_px", "frame_px")}
        if r.get("ok") and r.get("corners"):
            c = r["corners"]
            step = max(1, len(c) // 120)
            out["corners"] = [[round(p[0], 1), round(p[1], 1)] for p in c[::step]]
        if r.get("T_cam_target"):
            self.last_detect[cam_name] = r["T_cam_target"]
        return out

    def capture(self, cam_name, camera, tcp_pose):
        if camera is None:
            return {"ok": False, "error": f"camera {cam_name} not running"}
        img, intr = camera.still_frame()
        r = self.session(cam_name).add(img, tcp_pose, intr)
        if r.get("ok") and self.session(cam_name).samples:
            self.last_detect[cam_name] = self.session(cam_name).samples[-1].T_cam_target
        return r

    def undo(self, cam_name):
        return self.session(cam_name).remove_last()

    def clear(self, cam_name, spec=None):
        self.session(cam_name, spec)
        return {"ok": True}

    def status(self, cam_name):
        s = self.session(cam_name).status()
        s["plan"] = self.plans.get(cam_name)
        return s

    def solve(self, cam_name, truth=None):
        s = self.session(cam_name)
        r = s.solve()
        out = {k: v for k, v in r.items() if k not in ("per_sample_mm",)}
        if r.get("ok") and truth is not None:
            X = np.asarray(r["T_tcp_cam"], float)
            dt = float(np.linalg.norm(X[:3, 3] - truth[:3, 3])) * 1e3
            dR = X[:3, :3].T @ truth[:3, :3]
            ang = float(np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))))
            out["mock_truth"] = {"translation_error_mm": round(dt, 3), "rotation_error_deg": round(ang, 4),
                                 "note": "mock mode: the answer is known (the scene's camera mount), so the error is exact"}
        archive(f"handeye_{cam_name}", r.get("ok"), {k: out.get(k) for k in ("verdict", "translation_mm", "rotation_deg",
                                                                                "target_rms_mm", "n_samples", "mock_truth", "error")}, out)
        return out

    def save(self, cam_name, mock=False):
        s = self.session(cam_name)
        if not s.result or not s.result.get("ok"):
            return {"ok": False, "error": "solve first"}
        res = dict(s.result)
        res["camera"] = cam_name
        res["mock"] = mock
        res["saved_utc"] = _utc()
        r = self.he.save(res, os.path.join(CAL_DIR, f"handeye_{cam_name}.json"))
        if r.get("ok"):
            r["path"] = os.path.relpath(r["path"], ROOT)
        return r

    def plan(self, cam_name, tcp_pose, envelope=None):
        s = self.session(cam_name)
        if len(s.samples) >= 4:
            try:
                rough = s.solve()
            except Exception:
                rough = {}
            if rough.get("ok"):
                last = s.samples[-1]
                p = self.he.plan_auto_poses(tcp_pose, last.T_cam_target, T_tcp_cam_guess=rough["T_tcp_cam"],
                                            stage="fine", envelope=envelope, board_size_m=s.spec.size_m())
                if p.get("ok"):
                    self.plans[cam_name] = {"stage": "fine", "poses": p["poses"], "next": 0}
                    return {"ok": True, "stage": "fine", "n": p["n"]}
        p = self.he.plan_bootstrap_poses(tcp_pose, envelope=envelope)
        if not p.get("ok"):
            return {"ok": False, "error": p.get("error") or f"no reachable pose ({p.get('reason') or p.get('rejected')})"}
        self.plans[cam_name] = {"stage": "bootstrap", "poses": p["poses"], "next": 0}
        return {"ok": True, "stage": "bootstrap", "n": p["n"]}

    def next_pose(self, cam_name):
        pl = self.plans.get(cam_name)
        if not pl or pl["next"] >= len(pl["poses"]):
            return None
        pose = pl["poses"][pl["next"]]
        pl["next"] += 1
        return {"tcp_pose": pose.get("tcp_pose"), "index": pl["next"], "of": len(pl["poses"]), "stage": pl["stage"],
                "about": pose.get("about_deg") or [pose.get("tilt_deg"), pose.get("roll_deg"), pose.get("distance_mm")]}


# ---------------------------------------------------------------------------------------------- accelerometer six-face
class SixFace:
    def __init__(self):
        paths.add()
        import accel_cal
        self.ac = accel_cal
        self.faces = {}

    def capture(self, unit_name, unit):
        rec = list(unit.recent)[-120:] if unit else []
        if len(rec) < 40:
            return {"ok": False, "error": "not enough IMU samples: is the unit streaming?"}
        g = np.array([r[1] for r in rec]); a = np.array([r[2] for r in rec])
        if np.abs(g).max() > 0.05:
            return {"ok": False, "error": f"the carrier is moving (gyro up to {np.abs(g).max():.3f} rad/s): hold it still on the face"}
        m = a.mean(0)
        face = self.ac.identify_face(m)
        if not face:
            return {"ok": False, "error": "not square to a face (more than 25 deg off): rest it flat"}
        self.faces.setdefault(unit_name, {})[face] = m.tolist()
        return {"ok": True, "face": face, "mean": m.round(4).tolist(), "norm": round(float(np.linalg.norm(m)), 4),
                "faces": sorted(self.faces[unit_name])}

    def reset(self, unit_name):
        self.faces[unit_name] = {}
        return {"ok": True}

    def fit_save(self, unit_name, mock=False):
        r = self.ac.fit(self.faces.get(unit_name, {}))
        if r.get("ok"):
            r["unit"] = unit_name; r["mock"] = mock
            os.makedirs(CAL_DIR, exist_ok=True)
            json.dump(r, open(os.path.join(CAL_DIR, f"accel_{unit_name}.json"), "w"), indent=1)
        archive(f"accel_{unit_name}", r.get("ok"), {k: r.get(k) for k in ("before_spread_pct", "after_spread_pct", "worst_after_err_pct", "error")}, r)
        return r

    def status(self, unit_name, unit=None):
        live = None
        if unit and unit.latest.get("accel"):
            a = unit.latest["accel"]
            live = {"face": self.ac.identify_face(a), "norm": round(float(np.linalg.norm(a)), 4)}
            rec = list(unit.recent)[-60:]
            if rec:
                live["still"] = bool(np.abs(np.array([r[1] for r in rec])).max() < 0.05)
        return {"faces": self.faces.get(unit_name, {}), "live": live}


# ---------------------------------------------------------------------------------------------- IMU mount and lag
def imu_mount_from_session(run_dir, unit="tool", mock=False):
    """imu_align on a recorded session: the robot's flange rate from the measured joints against the gyro, both on the
    host clock through the session's own clock maps. Best on an excitation command (every wrist axis moves)."""
    import pandas as pd
    from sonair.jetson.clock import ClockMap
    paths.add()
    import imu_align
    df = pd.read_csv(os.path.join(run_dir, "rtde.csv"))
    ck = np.load(os.path.join(run_dir, "rtde_clock.npz"))
    cm = ClockMap().fit(ck["ctrl"], ck["host"])
    t_host = cm.to_host(df["timestamp"].to_numpy())
    q = df[[f"actual_q_{i}" for i in range(6)]].to_numpy()
    qd = df[[f"actual_qd_{i}" for i in range(6)]].to_numpy()
    step = max(1, len(q) // 6000)
    ur = {"t": t_host[::step], "q": q[::step], "qd": qd[::step]}
    zp = os.path.join(run_dir, f"imu_{unit}.npz")
    if not os.path.exists(zp):
        return {"ok": False, "error": f"no imu_{unit}.npz in this session"}
    z = np.load(zp)
    ok = np.isfinite(z["burst_dev"])
    icm = ClockMap().fit(z["burst_dev"][ok], z["burst_host"][ok])
    imu = {"unit": unit, "t": icm.to_host(z["t_dev"]), "gyro": z["gyro"], "accel": z["accel"]}
    w = imu_align.flange_omega(ur["q"], ur["qd"])
    lag = imu_align.estimate_lag(ur, imu, w)
    out = {"ok": False, "unit": unit, "session": os.path.basename(run_dir), "lag": lag, "made_at": _utc(), "mock": mock,
           "note": "residual lag after the clock maps; the clock maps themselves are in the session's qc.json"}
    if lag.get("ok"):
        mount = imu_align.estimate_mount(ur, imu, lag["lag_s"], w)
        out["mount"] = mount
        if mount.get("ok"):
            out.update(ok=True, lag_s=lag["lag_s"], R_flange_imu=mount["R_flange_imu"], accel_sign=mount.get("accel_sign", 1.0))
        else:
            out["error"] = "mounting: " + str(mount.get("error"))
    else:
        out["error"] = "lag: " + str(lag.get("error"))
    if out["ok"]:
        os.makedirs(CAL_DIR, exist_ok=True)
        json.dump(out, open(os.path.join(CAL_DIR, f"imu_{unit}.json"), "w"), indent=1, default=_jd)
    archive(f"imu_{unit}", out["ok"], {"session": out["session"], "lag_ms": round(1e3 * out.get("lag_s", 0), 3) if out["ok"] else None,
                                       "R_flange_imu": out.get("R_flange_imu"), "error": out.get("error")}, out)
    return json.loads(json.dumps(out, default=_jd))

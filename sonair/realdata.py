"""Reader for the run files recorded by the existing SONAIR cell platform (schema "sonair-run-1").

One run file = JSON Lines: line 0 is {"_manifest": {...}}, every other line is one sample written by the
cell PC at the RTDE rate (125 Hz in benchmark-v1). This module turns a run into flat numpy arrays and
caches them as .npz, so every analysis reads the same numbers.

Time bases kept apart on purpose:
  t_host   host receive time (s), the clock the IMU and camera are stamped in
  t_ctrl   the controller's own RTDE timestamp (s), the clock target_q, q and current were sampled in
Rows whose t_ctrl did not advance carry a robot reading that was already written (a stale row); they are
flagged, not silently used.
"""
import glob, json, os
import numpy as np

ELBOW = 2


def _vec(x, n):
    return [np.nan] * n if x is None else [float(v) for v in x]


def load(path):
    """Returns (manifest dict, dict of arrays)."""
    with open(path, encoding="utf-8") as f:
        man = json.loads(f.readline())["_manifest"]
        rows = [json.loads(line) for line in f if line.strip()]
    n = len(rows)
    A = {k: np.full((n, 6), np.nan) for k in ("q", "qd", "tq", "tqd", "tm", "cur")}
    A.update(t_host=np.full(n, np.nan), t_ctrl=np.full(n, np.nan), age=np.full(n, np.nan),
             speed_scaling=np.full(n, np.nan), tcp=np.full((n, 3), np.nan),
             gyro=np.full((n, 3), np.nan), accel=np.full((n, 3), np.nan), t_imu=np.full(n, np.nan),
             imu_rate=np.full(n, np.nan))
    for i, s in enumerate(rows):
        A["t_host"][i] = s.get("t", np.nan)
        A["t_ctrl"][i] = (s.get("aux") or {}).get("controller_t", np.nan)
        A["age"][i] = s.get("robot_age_s", np.nan) if s.get("robot_age_s") is not None else np.nan
        A["speed_scaling"][i] = s.get("speed_scaling") if s.get("speed_scaling") is not None else np.nan
        A["q"][i] = _vec(s.get("q"), 6); A["qd"][i] = _vec(s.get("qd"), 6)
        A["tq"][i] = _vec(s.get("target_q"), 6); A["tqd"][i] = _vec(s.get("target_qd"), 6)
        A["tm"][i] = _vec(s.get("target_moment"), 6); A["tcp"][i] = _vec(s.get("tcp_pos"), 3)
        ur = (s.get("sensors") or {}).get("ur_tcp") or {}
        A["cur"][i] = _vec(ur.get("actual_current"), 6)
        imu = (s.get("sensors") or {}).get("imu_ind0") or (s.get("imu") or {}).get("ind0") or {}
        A["gyro"][i] = _vec(imu.get("gyro"), 3); A["accel"][i] = _vec(imu.get("accel"), 3)
        A["t_imu"][i] = imu.get("t", np.nan) if imu.get("t") is not None else np.nan
        A["imu_rate"][i] = imu.get("rate_hz", np.nan) if imu.get("rate_hz") is not None else np.nan
    A["stale"] = np.r_[False, np.diff(A["t_ctrl"]) <= 0]
    return man, A


def load_dir(root, pattern="*.jsonl", cache=None):
    """All run files under root (superseded copies ignored). Cached as npz in `cache` if given."""
    out = {}
    for p in sorted(glob.glob(os.path.join(root, pattern))):
        rid = os.path.basename(p)[:-6]
        c = os.path.join(cache, rid + ".npz") if cache else None
        if c and os.path.exists(c) and os.path.getmtime(c) >= os.path.getmtime(p):
            z = np.load(c, allow_pickle=True); man = json.loads(str(z["_manifest"]))
            A = {k: z[k] for k in z.files if k != "_manifest"}
        else:
            man, A = load(p)
            if c:
                os.makedirs(cache, exist_ok=True)
                np.savez_compressed(c, _manifest=json.dumps(man), **A)
        out[rid] = (man, A)
    return out


def cell_key(rid):
    """'v0p50_mid_workspace_contour_r00' -> 'v0p50_mid_workspace_contour'"""
    return rid.rsplit("_r", 1)[0]


def held_out(man):
    return "HELD-OUT" in (man.get("notes") or "")

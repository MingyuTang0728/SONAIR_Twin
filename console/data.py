"""What the Data and Benchmark pages show: recorded sessions, the released benchmark-v1 runs, the scoring rules."""
from __future__ import annotations

import csv
import gzip
import json
import os

import numpy as np

from sonair import cell

ROOT = cell.ROOT
SESSIONS = os.path.join(ROOT, "runs_jetson")
BENCH = os.path.join(ROOT, "data", "benchmark_v1")


def list_sessions(root=SESSIONS):
    out = []
    if not os.path.isdir(root):
        return out
    for d in sorted(os.listdir(root), reverse=True):
        p = os.path.join(root, d)
        mp = os.path.join(p, "manifest.json")
        if not os.path.exists(mp):
            continue
        try:
            m = json.load(open(mp))
        except Exception:
            continue
        qc = {}
        if os.path.exists(os.path.join(p, "qc.json")):
            try:
                qc = json.load(open(os.path.join(p, "qc.json")))
            except Exception:
                pass
        res = m.get("result") or {}
        out.append(dict(run_id=d, created_utc=m.get("created_utc"), mock=bool(m.get("mock")), cell=m.get("cell"),
                        motion=m.get("motion"), payload=m.get("payload"), verdict=qc.get("verdict"),
                        reasons=(qc.get("reasons") or [])[:6], duration_s=(qc.get("rtde") or {}).get("duration_s"),
                        packets=res.get("robot_packets"), imu=res.get("imu_samples"),
                        exported=os.path.exists(os.path.join(ROOT, "exports", "platform", d + ".jsonl"))))
    return out


def bench_index():
    p = os.path.join(BENCH, "index.csv")
    if not os.path.exists(p):
        return []
    with open(p) as f:
        return list(csv.DictReader(f))


def _read_jsonl_gz(path):
    op = gzip.open if path.endswith(".gz") else open
    with op(path, "rt") as f:
        head = json.loads(f.readline())
        rows = [json.loads(l) for l in f if l.strip()]
    return head, rows


def preview_bench(run_id, max_points=800):
    for split in ("train", "test_inputs"):
        for ext in (".jsonl.gz", ".cmd.jsonl.gz"):
            p = os.path.join(BENCH, split, run_id + ext)
            if os.path.exists(p):
                head, rows = _read_jsonl_gz(p)
                step = max(1, len(rows) // max_points)
                rows = rows[::step]
                t = [r["t"] - rows[0]["t"] for r in rows]
                out = {"run_id": run_id, "split": split, "manifest": head.get("_manifest") or head.get("_input"), "t": t,
                       "cmd_elbow": [(r.get("target_q") or (r.get("cmd") or {}).get("q") or [None] * 3)[2] for r in rows]}
                if split == "train":
                    out["q_elbow"] = [(r.get("q") or [None] * 3)[2] for r in rows]
                    out["current_elbow"] = [(((r.get("sensors") or {}).get("ur_tcp") or {}).get("actual_current") or [None] * 3)[2] for r in rows]
                    out["gyro_norm"] = [float(np.linalg.norm(((r.get("imu") or {}).get("ind0") or {}).get("gyro") or [np.nan] * 3)) for r in rows]
                    out["gyro_norm"] = [None if not np.isfinite(x) else round(x, 5) for x in out["gyro_norm"]]
                return out
    return {"error": f"{run_id}: not in the released data"}


def preview_session(run_id, max_points=800, root=SESSIONS):
    import pandas as pd
    p = os.path.join(root, run_id, "rtde.csv")
    if not os.path.exists(p):
        return {"error": f"{run_id}: no rtde.csv"}
    df = pd.read_csv(p)
    step = max(1, len(df) // max_points)
    d = df.iloc[::step]
    t = (d["timestamp"] - d["timestamp"].iloc[0]).round(4).tolist()
    out = {"run_id": run_id, "split": "session", "t": t, "cmd_elbow": d["target_q_2"].round(6).tolist(),
           "q_elbow": d["actual_q_2"].round(6).tolist(), "current_elbow": d["actual_current_2"].round(4).tolist()}
    qc = os.path.join(root, run_id, "qc.json")
    if os.path.exists(qc):
        out["qc"] = json.load(open(qc))
    return out


def bench_info():
    """The numbers the Benchmark page shows, from the files in the repo (nothing is recomputed here)."""
    info = {}
    a = os.path.join(ROOT, "reports", "v1_real", "audit_v1.json")
    if os.path.exists(a):
        d = json.load(open(a))
        info["audit"] = dict(data=d.get("data"), quality=d.get("quality"), floors=d.get("floors"),
                             current=dict(S0_p95_A=d["current"].get("S0_p95_A"), S0_tail_p95_A=d["current"].get("S0_tail_p95_A"), paired=d["current"].get("paired"), ceiling=d["current"].get("ceiling"),
                                          maps=d["current"].get("maps"), board=d["current"].get("board")),
                             gyro=d.get("gyro"))
    lb = os.path.join(BENCH, "site", "leaderboard.json")
    if os.path.exists(lb):
        info["leaderboard"] = json.load(open(lb))
    try:
        from cell_platform import paths
        paths.add()
        import importlib
        task = importlib.import_module("sonair_benchmark.task")
        info["task_version"] = task.TASK_VERSION
        info["stress"] = task.STRESS
    except Exception as e:
        info["task_error"] = str(e)
    rel = os.path.join(BENCH, "release.json")
    if os.path.exists(rel):
        info["release"] = json.load(open(rel))
    return info


# ---------------------------------------------------------------------------------------------- run replay and channels
CHANNELS = {
    "q": "measured joints (rad)", "target_q": "commanded joints (rad)", "current": "joint currents (A)",
    "tcp": "tool position (m)", "gyro": "IMU angular rate (rad/s)", "accel": "IMU acceleration (m/s2)",
    "target_moment": "controller torque model (N m)", "qd": "measured joint velocities (rad/s)",
}


def run_series(kind, run_id, max_points=2500, root=SESSIONS):
    """Every channel of one run, decimated, on its own clock from 0: for the Data page charts and the 3D replay.
    Held-out benchmark runs carry their inputs only (commanded joints and the controller torque model)."""
    out = {"run_id": run_id, "kind": kind, "channels": {}}
    if kind == "session":
        import pandas as pd
        p = os.path.join(root, run_id, "rtde.csv")
        if not os.path.exists(p):
            return {"error": f"{run_id}: no rtde.csv"}
        df = pd.read_csv(p)
        step = max(1, len(df) // max_points)
        d = df.iloc[::step]
        t = d["timestamp"].to_numpy(); out["t"] = (t - t[0]).round(4).tolist()
        for ch, base in (("q", "actual_q"), ("target_q", "target_q"), ("current", "actual_current"), ("qd", "actual_qd"),
                         ("target_moment", "target_moment")):
            cols = [f"{base}_{i}" for i in range(6)]
            if all(c in d for c in cols):
                out["channels"][ch] = d[cols].round(6).to_numpy().T.tolist()
        cols = [f"actual_TCP_pose_{i}" for i in range(3)]
        if all(c in d for c in cols):
            out["channels"]["tcp"] = d[cols].round(6).to_numpy().T.tolist()
        for f in sorted(os.listdir(os.path.join(root, run_id))):
            if f.startswith("imu_") and f.endswith(".npz"):
                z = np.load(os.path.join(root, run_id, f))
                td = z["t_dev"]
                if len(td) > 10:
                    k = max(1, len(td) // max_points)
                    out.setdefault("imu", {})[f[4:-4]] = {"t_dev": (td[::k] - td[0]).round(5).tolist(),
                                                          "gyro": z["gyro"][::k].round(5).T.tolist(),
                                                          "accel": z["accel"][::k].round(4).T.tolist(),
                                                          "note": "IMU on its own clock from its first sample; the session's clock maps place it"}
        mp = os.path.join(root, run_id, "manifest.json")
        if os.path.exists(mp):
            out["manifest"] = json.load(open(mp))
        qp = os.path.join(root, run_id, "qc.json")
        if os.path.exists(qp):
            out["qc"] = json.load(open(qp))
        return out
    for split in ("train", "test_inputs"):
        for ext in (".jsonl.gz", ".cmd.jsonl.gz"):
            p = os.path.join(BENCH, split, run_id + ext)
            if not os.path.exists(p):
                continue
            head, rows = _read_jsonl_gz(p)
            step = max(1, len(rows) // max_points)
            rows = rows[::step]
            out["split"] = "train" if split == "train" else "test"
            out["manifest"] = head.get("_manifest") or head.get("_input")
            out["t"] = [round(r["t"] - rows[0]["t"], 4) for r in rows]

            def col(get, n):
                vals = []
                for r in rows:
                    try:
                        v = get(r)
                    except (KeyError, TypeError, IndexError):
                        v = None
                    vals.append(v if v is not None else [None] * n)
                return [[(None if x is None else round(float(x), 6)) for x in c] for c in zip(*vals)]
            out["channels"]["target_q"] = col(lambda r: r.get("target_q"), 6)
            out["channels"]["target_moment"] = col(lambda r: r.get("target_moment"), 6)
            if split == "train":
                out["channels"]["q"] = col(lambda r: r.get("q"), 6)
                out["channels"]["qd"] = col(lambda r: r.get("qd"), 6)
                out["channels"]["current"] = col(lambda r: r["sensors"]["ur_tcp"]["actual_current"], 6)
                out["channels"]["tcp"] = col(lambda r: r.get("tcp_pos"), 3)
                out["channels"]["gyro"] = col(lambda r: r["imu"]["ind0"]["gyro"], 3)
                out["channels"]["accel"] = col(lambda r: r["imu"]["ind0"]["accel"], 3)
            return out
    return {"error": f"{run_id}: not in the released data"}


# ---------------------------------------------------------------------------------------------- the scoreboard
SPEEDS = ["0.2", "0.4", "0.5", "0.6", "0.7", "0.9"]
ARMS = ["near_singular", "mid_workspace", "extended"]
TRAJS = ["contour", "point_to_point", "stop_start"]
# the UoN collection budget of the landing plan (6.3): hours of QC-PASS motion per category
BUDGET_H = [("E1 identification", 30), ("E1 payload step", 5), ("E2 condition grid", 60), ("E3 kit sequences", 60),
            ("Thermal and endurance", 120), ("Pilot and acceptance", 25)]
# proposal requirements and their state (landing plan 6.1); what the page lists, not something it measures
RELEASE = [("API keys, local validator, containerised evaluator, score card", "met"),
           ("Held-out split, robustness (missing, lag, drift), uncertainty calibration", "met"),
           ("Data, evaluation and task cards, SHA-256 checksums", "met"), ("Croissant and RAI metadata", "open"),
           ("Efficiency and abstention columns", "partial"), ("Two sites", "open"), ("Two or more public tasks", "open"),
           ("About 600 h of data (UoN 300 h, UCL 300 h)", "open"), ("DOI release (Zenodo)", "open")]


def _csv(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def bench_overview(sessions_root=SESSIONS):
    """Everything the Benchmark page draws, read from files in the repository and the local sessions."""
    out = {"speeds": SPEEDS, "arms": ARMS, "trajs": TRAJS}
    idx = bench_index()
    held = set()
    rel = os.path.join(BENCH, "release.json")
    if os.path.exists(rel):
        held = set(json.load(open(rel)).get("held_out_cell_list") or [])
    grid = {}
    for r in idx:
        v = f"{float(r['joint_vel']):.1f}"
        key = f"{v}|{r['arm_config']}|{r['traj_type']}"
        g = grid.setdefault(key, {"runs": 0, "by_session": {}, "held_out": r["cell"] in held, "seconds": 0.0})
        g["runs"] += 1
        g["by_session"][r["session"]] = g["by_session"].get(r["session"], 0) + 1
        g["seconds"] += float(r.get("duration_s") or 0)
    floors = {}
    for r in _csv(os.path.join(ROOT, "reports", "v1_real", "floors_per_cell.csv")):
        c = r["cell"]                                     # v0p20_extended_contour
        v = f"{int(c[1]) + int(c[3:5]) / 100:.1f}" if c.startswith("v0p") else None
        rest = c[6:]
        arm = next((a for a in ARMS if rest.startswith(a)), None)
        if v and arm:
            floors[f"{v}|{arm}|{rest[len(arm) + 1:]}"] = {k: round(float(r[k]), 4) for k in ("cur_p95", "cur_tail_p95", "tcp_p95_mm", "gyro_p95")}
    for k, f in floors.items():
        grid.setdefault(k, {"runs": 0, "by_session": {}, "held_out": False, "seconds": 0.0})["floor"] = f
    out["grid"] = grid
    out["by_speed"] = _csv(os.path.join(ROOT, "reports", "v1_real", "current_by_speed.csv"))
    heldout = _csv(os.path.join(ROOT, "reports", "v1_real", "current_scores_heldout.csv"))
    out["heldout_runs"] = [{k: (r[k] if k in ("run", "cell") else round(float(r[k]), 4)) for k in
                            ("run", "cell", "v", "S0_p95", "floor_p95", "CM_gcr", "CM_gcr_tail", "F1_gcr", "F1_gcr_tail", "B2_gcr",
                             "B2_gcr_tail", "B2F_gcr", "B2F_gcr_tail", "B2_s0map_gcr", "ceiling_gcr", "ceiling_gcr_tail") if k in r}
                           for r in heldout]
    q = _csv(os.path.join(ROOT, "reports", "v1_real", "quality_per_run.csv"))
    if q:
        imu_frac = sorted(float(r["imu_unique_frac"]) for r in q)
        out["quality"] = {"runs": len(q), "ctrl_period_anomalies": sum(int(r["ctrl_period_anomalies"]) for r in q),
                          "stale_rows": sum(int(r["stale_rows"]) for r in q),
                          "imu_unique_frac_median": imu_frac[len(imu_frac) // 2], "imu_unique_frac_min": imu_frac[0],
                          "elbow_track_rms_deg_median": sorted(float(r["elbow_track_rms_deg"]) for r in q)[len(q) // 2]}
    # hours: benchmark-v1 by session label, and this console's sessions that passed QC (mock ones apart)
    hours = {}
    for r in idx:
        hours.setdefault(r["session"], 0.0)
        hours[r["session"]] += float(r.get("duration_s") or 0) / 3600
    local = {"pass_h": 0.0, "warn_h": 0.0, "fail_h": 0.0, "mock_h": 0.0, "n": 0}
    for s in list_sessions(sessions_root):
        h = (s.get("duration_s") or 0) / 3600
        local["n"] += 1
        if s.get("mock"):
            local["mock_h"] += h
        elif s.get("verdict") == "PASS":
            local["pass_h"] += h
        elif s.get("verdict") == "WARN":
            local["warn_h"] += h
        else:
            local["fail_h"] += h
    out["hours"] = {"benchmark_v1": {k: round(v, 4) for k, v in hours.items()}, "console": {k: round(v, 4) if isinstance(v, float) else v for k, v in local.items()},
                    "budget": BUDGET_H, "target_uon_h": 300}
    out["release"] = RELEASE
    try:
        from cell_platform import paths
        paths.add()
        import importlib
        task = importlib.import_module("sonair_benchmark.task")
        ch = importlib.import_module("sonair_benchmark.channels")
        sc = importlib.import_module("sonair_benchmark.scoring")
        out["task"] = task.task_spec()
        out["channels"] = ch.CHANNELS
        out["tracks"] = getattr(sc, "TRACKS", None)
    except Exception as e:
        out["task_error"] = str(e)
    return out

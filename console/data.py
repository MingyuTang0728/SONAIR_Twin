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
                             current=dict(S0_p95_A=d["current"].get("S0_p95_A"), ceiling=d["current"].get("ceiling"),
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

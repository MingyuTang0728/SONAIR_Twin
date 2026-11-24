"""Run logs: one folder per execution of one command in one world, identical for real and simulated runs.

runs/<run_id>/meta.json      schema, source (real|mujoco|isaac|unity), variant, command id + sha256, payload, speed scale, ...
runs/<run_id>/joints.parquet 500 Hz rows, columns:
    t                      s, 0 = start of the command
    tq0..5, tqd0..5        target joint position / velocity the controller tracked (real: RTDE target_q, target_qd)
    q0..5, qd0..5          actual joint position / velocity
    tau0..5   (sim)        joint torque applied by the simulated controller, N m
    cur0..5   (real)       actual joint current, A          tau_ff0..5 (real) RTDE target_moment, N m
    tcp_x,y,z,rx,ry,rz     actual tool0 pose in B (m, rotation vector rad)
    speed_scaling, temp0..5 (real only), grip (0..1)
"""
import json, os, re, time
import numpy as np
import pandas as pd

SCHEMA = "sonair.run/0.1"
REQUIRED = ["t"] + [f"tq{i}" for i in range(6)] + [f"q{i}" for i in range(6)] + [f"qd{i}" for i in range(6)]
SOURCES = ("real", "mujoco", "isaac", "unity", "other")


def write(run_dir, meta, df):
    os.makedirs(run_dir, exist_ok=True)
    meta = dict(schema=SCHEMA, created_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), n=len(df),
                duration_s=float(df["t"].iloc[-1] - df["t"].iloc[0]), **meta)
    df.to_parquet(os.path.join(run_dir, "joints.parquet"), index=False)
    json.dump(meta, open(os.path.join(run_dir, "meta.json"), "w"), ensure_ascii=False, indent=1)
    return run_dir


def read(run_dir):
    return json.load(open(os.path.join(run_dir, "meta.json"), encoding="utf-8")), pd.read_parquet(os.path.join(run_dir, "joints.parquet"))


def validate(run_dir):
    """returns a list of problems (empty = valid)"""
    probs = []
    try:
        meta, df = read(run_dir)
    except Exception as e:
        return [f"cannot read: {e}"]
    if meta.get("schema") != SCHEMA: probs.append(f"schema {meta.get('schema')} != {SCHEMA}")
    if meta.get("source") not in SOURCES: probs.append(f"source must be one of {SOURCES}")
    for k in ("run_id", "command_id", "payload"):
        if k not in meta: probs.append(f"meta.{k} missing")
    miss = [c for c in REQUIRED if c not in df.columns]
    if miss: probs.append(f"missing columns {miss}")
    else:
        dt = np.diff(df["t"].to_numpy())
        if (dt <= 0).any(): probs.append("t is not strictly increasing")
        elif abs(np.median(dt) - 0.002) > 2e-4: probs.append(f"median step {np.median(dt) * 1000:.2f} ms, expected 2 ms (500 Hz)")
        if df[REQUIRED].isna().any().any(): probs.append("NaN in required columns")
    return probs


def target_from_run(run_dir):
    """the controller's own target trajectory from a real run, for replay in a simulator"""
    meta, df = read(run_dir)
    return df[["t"] + [f"tq{i}" for i in range(6)] + [f"tqd{i}" for i in range(6)]].copy(), meta


# ------------------------------------------------------------------ ur_rtde recording CSV -> run
_MAP = [(r"^timestamp$", "t"), (r"^targetq(\d)$", "tq\\1"), (r"^targetqd(\d)$", "tqd\\1"), (r"^targetqdd(\d)$", "tqdd\\1"),
        (r"^actualq(\d)$", "q\\1"), (r"^actualqd(\d)$", "qd\\1"), (r"^actualcurrent(\d)$", "cur\\1"), (r"^targetmoment(\d)$", "tau_ff\\1"),
        (r"^jointtemperatures(\d)$", "temp\\1"), (r"^speedscaling$", "speed_scaling"), (r"^robotmode$", "robot_mode"), (r"^safetymode$", "safety_mode")]
_TCP = ["tcp_x", "tcp_y", "tcp_z", "tcp_rx", "tcp_ry", "tcp_rz"]


def from_rtde_csv(csv_path):
    raw = pd.read_csv(csv_path)
    out = {}
    for col in raw.columns:
        k = re.sub(r"[^A-Za-z0-9]", "", col)
        m = re.match(r"^actualTCPpose(\d)$", k)
        if m: out[_TCP[int(m.group(1))]] = raw[col]; continue
        k = k.lower()
        for pat, rep in _MAP:
            if re.match(pat, k): out[re.sub(pat, rep, k)] = raw[col]; break
    df = pd.DataFrame(out)
    if "t" not in df: raise ValueError("no timestamp column in " + csv_path)
    df["t"] = df["t"] - df["t"].iloc[0]
    return df

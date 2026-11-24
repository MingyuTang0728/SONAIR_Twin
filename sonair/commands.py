"""Command files: the single source of truth that every world (real UR, MuJoCo, Isaac Sim, Unity) executes.

A command is a folder  commands/<id>/  with
    command.json     meta (id, kind, payload or payload_schedule, sha256 of the knots file, provenance, collision check)
    knots.parquet    t, q0..q5 (rad), grip (0 open .. 1 closed), capture (bool) at 50 Hz
load() expands the knots to 500 Hz (the UR real-time rate) with a clamped cubic spline, giving t, q*, qd*, qdd*,
so every machine reproduces the same stream from a small file. Simulators should replay the robot's *recorded* target_q when it exists
(see runlog.target_from_run), so trajectory interpolation on the controller never shows up as a dynamics gap.
"""
import hashlib, json, math, os, time
import numpy as np
import pandas as pd
from scipy.interpolate import CubicSpline
from . import cell

RATE = 500.0
QCOLS = [f"q{i}" for i in range(6)]; QDCOLS = [f"qd{i}" for i in range(6)]; QDDCOLS = [f"qdd{i}" for i in range(6)]


def make(cid, kind, t, q, qd, qdd, grip=None, capture=None, **meta):
    df = pd.DataFrame(np.column_stack([t, q, qd, qdd]), columns=["t"] + QCOLS + QDCOLS + QDDCOLS)
    df["grip"] = np.zeros(len(t)) if grip is None else grip
    df["capture"] = np.zeros(len(t), bool) if capture is None else capture
    m = dict(schema="sonair.command/0.1", id=cid, kind=kind, rate_hz=RATE, n=len(t), duration_s=float(t[-1]),
             peak_qd=np.abs(qd).max(0).round(4).tolist(), peak_qdd=np.abs(qdd).max(0).round(3).tolist(),
             created_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), **meta)
    return dict(meta=m, df=df)


KNOT_RATE = 50.0


def expand(knots):
    """50 Hz knots -> 500 Hz command samples (clamped cubic spline, zero velocity at both ends)"""
    t0 = knots["t"].to_numpy(); Q = knots[QCOLS].to_numpy()
    sp = CubicSpline(t0, Q, axis=0, bc_type="clamped")
    t = np.arange(0, t0[-1] + 1e-9, 1 / RATE)
    df = pd.DataFrame(np.column_stack([t, sp(t), sp(t, 1), sp(t, 2)]), columns=["t"] + QCOLS + QDCOLS + QDDCOLS)
    df["grip"] = np.interp(t, t0, knots["grip"]); df["capture"] = np.interp(t, t0, knots["capture"].astype(float)) > 0.5
    return df


def save(cmd, root=os.path.join(cell.ROOT, "commands")):
    """stores 50 Hz knots (small, exact recipe); load() expands them to 500 Hz the same way on every machine"""
    d = os.path.join(root, cmd["meta"]["id"]); os.makedirs(d, exist_ok=True)
    df = cmd["df"]; step = int(round(RATE / KNOT_RATE)); idx = list(range(0, len(df), step))
    if idx[-1] != len(df) - 1: idx.append(len(df) - 1)
    knots = df.iloc[idx][["t"] + QCOLS + ["grip", "capture"]].reset_index(drop=True)
    p = os.path.join(d, "knots.parquet"); knots.to_parquet(p, index=False, compression="zstd")
    cmd["meta"]["sha256_knots"] = hashlib.sha256(open(p, "rb").read()).hexdigest()
    cmd["meta"]["storage"] = "knots.parquet at 50 Hz; expand(): clamped cubic spline to 500 Hz"
    ex = expand(knots)
    cmd["meta"]["peak_qd"] = np.abs(ex[QDCOLS].to_numpy()).max(0).round(4).tolist()
    cmd["meta"]["peak_qdd"] = np.abs(ex[QDDCOLS].to_numpy()).max(0).round(3).tolist()
    json.dump(cmd["meta"], open(os.path.join(d, "command.json"), "w"), ensure_ascii=False, indent=1)
    return d


def load(path):
    d = path if os.path.isdir(path) else os.path.join(cell.ROOT, "commands", path)
    meta = json.load(open(os.path.join(d, "command.json"), encoding="utf-8"))
    meta["sha256"] = meta.get("sha256_knots")
    return dict(meta=meta, df=expand(pd.read_parquet(os.path.join(d, "knots.parquet"))))


def from_unity_csv(path, cid=None, payload_schedule=None):
    """50 Hz trajectory CSV exported with the Unity package -> 500 Hz command (clamped cubic spline)."""
    src = pd.read_csv(path, encoding="utf-8-sig")
    t0 = src["t_s"].to_numpy(); Q = src[["q1_base", "q2_shoulder", "q3_elbow", "q4_wrist1", "q5_wrist2", "q6_wrist3"]].to_numpy()
    keep = np.concatenate([[True], np.diff(t0) > 1e-9]); t0, Q = t0[keep], Q[keep]
    sp = CubicSpline(t0, Q, axis=0, bc_type="clamped")
    t = np.arange(0, t0[-1] + 1e-9, 1 / RATE)
    grip = np.interp(t, src["t_s"], src["gripper_closure_0to1"])
    phase = src["phase"].astype(str).to_numpy()
    idx = np.clip(np.searchsorted(src["t_s"].to_numpy(), t, side="right") - 1, 0, len(src) - 1)
    capture = np.array([("scan" in phase[i].lower()) or ("capture" in phase[i].lower()) for i in idx])
    cid = cid or "task_" + os.path.splitext(os.path.basename(path))[0]
    return make(cid, "task", t, sp(t), sp(t, 1), sp(t, 2), grip, capture, source=os.path.basename(path),
                payload_schedule=payload_schedule or [[0.0, "gripper_open"]])


def time_scale(cmd, s, cid=None):
    """s > 1 runs faster: t' = t/s, qd' = s qd, qdd' = s^2 qdd (then resampled to 500 Hz)."""
    df = cmd["df"]; t = df["t"].to_numpy() / s
    tn = np.arange(0, t[-1] + 1e-9, 1 / RATE)
    q = np.column_stack([np.interp(tn, t, df[c]) for c in QCOLS])
    qd = np.column_stack([np.interp(tn, t, df[c]) * s for c in QDCOLS])
    qdd = np.column_stack([np.interp(tn, t, df[c]) * s * s for c in QDDCOLS])
    grip = np.interp(tn, t, df["grip"]); cap = np.interp(tn, t, df["capture"].astype(float)) > 0.5
    m = {k: v for k, v in cmd["meta"].items() if k not in ("id", "kind", "rate_hz", "n", "duration_s", "peak_qd", "peak_qdd", "created_utc", "sha256", "schema")}
    if "payload_schedule" in m: m["payload_schedule"] = [[round(a / s, 4), k] for a, k in m["payload_schedule"]]
    return make(cid or f"{cmd['meta']['id']}_x{s:g}", cmd["meta"]["kind"], tn, q, qd, qdd, grip, cap, speed_scale=s, parent=cmd["meta"]["id"], **m)


# ------------------------------------------------------------------ excitation (finite Fourier series, Swevers et al.)
Q_CENTER = np.array([2.746, -2.123, 2.135, -1.583, -1.571, -1.966])   # tool0 ~ (0.30, 0.02, 0.42) m pointing down
DEV_LIMIT = np.array([0.55, 0.30, 0.45, 0.70, 0.70, 0.90])             # max deviation from the centre per joint (rad)


def _window(t, T, r):
    w = np.ones_like(t)
    for sel, x in ((t < r, t / r), (t > T - r, (T - t) / r)):
        x = np.clip(x, 0, 1); w[sel] = (x * x * x * (10 - 15 * x + 6 * x * x))[sel]
    return w


def excitation(level_qd, seed, period=10.0, harmonics=5, periods=2, ramp=3.0, center=Q_CENTER, cid=None, payload="gripper_open"):
    """Smooth multi-sine joint trajectory around `center`, scaled so the fastest joint reaches level_qd (rad/s)."""
    rng = np.random.default_rng(seed); wf = 2 * math.pi / period
    T = periods * period + 2 * ramp; t = np.arange(0, T + 1e-9, 1 / RATE)
    k = np.arange(1, harmonics + 1)
    F = np.zeros((len(t), 6))
    for j in range(6):
        a = rng.normal(size=harmonics) * k / harmonics; b = rng.normal(size=harmonics) * k / harmonics
        F[:, j] = (a[None] / (wf * k) * np.sin(wf * np.outer(t, k)) - b[None] / (wf * k) * np.cos(wf * np.outer(t, k))).sum(1)
    F -= F.mean(0); W = _window(t, T, ramp)[:, None]
    D = W * F
    dD = np.gradient(D, t, axis=0)
    scale = level_qd / np.abs(dD).max(0)                                # every joint peaks at level_qd ...
    scale = np.minimum(scale, DEV_LIMIT / np.abs(D).max(0))             # ... unless that breaks its deviation limit
    q = center + D * scale
    qd = np.gradient(q, t, axis=0); qdd = np.gradient(qd, t, axis=0)
    return make(cid or f"exc_v{level_qd:.1f}_s{seed}", "excitation", t, q, qd, qdd, None, None, level_qd=level_qd, seed=seed,
                period_s=period, harmonics=harmonics, payload=payload)

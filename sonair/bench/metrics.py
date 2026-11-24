"""Dynamics-gap metrics. Everything is computed on run logs (runlog format), so real and simulated runs are scored
by the same code.

tracking(df)      how well one world followed its own target: joint error, tool0 error, lag, per speed bin
gap(a, b)         difference between two worlds that executed the same command (e.g. MuJoCo vs real):
                  joint and tool0 differences after an optional time alignment, per speed bin
normalized gap    gap(sim, real) / gap(real, real repeat): 1.0 means "as close as the robot is to itself"
"""
import numpy as np
from .. import kin

BINS = [0.0, 0.3, 0.6, 0.9, np.inf]
BIN_NAMES = ["<0.3", "0.3-0.6", "0.6-0.9", ">0.9"]
J = range(6)


def _cols(df, p): return df[[f"{p}{i}" for i in J]].to_numpy()


def speed(df):
    """joint-space speed of the target: max_j |tqd_j| (rad/s)"""
    tqd = _cols(df, "tqd") if "tqd0" in df else np.gradient(_cols(df, "tq"), df["t"].to_numpy(), axis=0)
    return np.abs(tqd).max(1)


def _bins(v):
    return np.digitize(v, BINS[1:-1])


def _lag_ms(x, y, dt, maxlag=0.25):
    """lag of y behind x (ms) from the cross-correlation peak"""
    n = int(maxlag / dt)
    if np.std(x) < 1e-9 or np.std(y) < 1e-9: return 0.0
    mse = [np.mean((x[: len(x) - k] - y[k:]) ** 2) for k in range(n)]
    return round(1000 * dt * int(np.argmin(mse)), 1)


def tracking(df, stride=5):
    t = df["t"].to_numpy(); dt = float(np.median(np.diff(t)))
    q, tq = _cols(df, "q"), _cols(df, "tq"); e = q - tq
    P = kin.fk_pos(q[::stride]); PT = kin.fk_pos(tq[::stride]); et = np.linalg.norm(P - PT, axis=1) * 1000
    b = _bins(speed(df)); bs = b[::stride]
    out = dict(joint_rms_mrad=(np.sqrt((e ** 2).mean(0)) * 1000).round(3).tolist(), joint_max_mrad=(np.abs(e).max(0) * 1000).round(3).tolist(),
               tcp_rms_mm=round(float(np.sqrt((et ** 2).mean())), 3), tcp_max_mm=round(float(et.max()), 3),
               lag_ms=[_lag_ms(df[f"tqd{j}"].to_numpy() if f"tqd{j}" in df else np.gradient(tq[:, j], t), df[f"qd{j}"].to_numpy(), dt) for j in J],
               by_speed={})
    for k, name in enumerate(BIN_NAMES):
        s = bs == k
        if s.sum() >= 10: out["by_speed"][name] = dict(share=round(float(s.mean()), 3), tcp_rms_mm=round(float(np.sqrt((et[s] ** 2).mean())), 3))
    return out


def gap(a, b, align=False, stride=5, max_offset=0.05):
    """a, b: run DataFrames of the same command. Returns differences of actual motion (b resampled onto a's clock).
    align=False (default) when both share the command clock, e.g. a simulator replaying the robot's recorded target:
    a lag is then part of the gap. align=True estimates a constant clock offset (only for independently clocked logs)."""
    ta = a["t"].to_numpy(); tb = b["t"].to_numpy(); dt = float(np.median(np.diff(ta)))
    off = 0.0
    if align:   # estimate a constant clock offset from the joint-speed norm
        va = np.linalg.norm(_cols(a, "qd"), axis=1); vb = np.interp(ta, tb, np.linalg.norm(_cols(b, "qd"), axis=1))
        n = int(round(max_offset / dt))
        mse = [np.mean((va[max(0, -k): len(va) - max(0, k)] - vb[max(0, k): len(vb) - max(0, -k)]) ** 2) for k in range(-n, n + 1)]
        off = (int(np.argmin(mse)) - n) * dt
    qa = _cols(a, "q"); qb = np.column_stack([np.interp(ta + off, tb, b[f"q{j}"]) for j in J])
    d = qa - qb
    Pa = kin.fk_pos(qa[::stride]); Pb = kin.fk_pos(qb[::stride]); dp = np.linalg.norm(Pa - Pb, axis=1) * 1000
    bs = _bins(speed(a))[::stride]
    out = dict(clock_offset_ms=round(off * 1000, 2), joint_rms_mrad=(np.sqrt((d ** 2).mean(0)) * 1000).round(3).tolist(),
               tcp_rms_mm=round(float(np.sqrt((dp ** 2).mean())), 3), tcp_max_mm=round(float(dp.max()), 3), by_speed={})
    for k, name in enumerate(BIN_NAMES):
        s = bs == k
        if s.sum() >= 10: out["by_speed"][name] = dict(share=round(float(s.mean()), 3), tcp_rms_mm=round(float(np.sqrt((dp[s] ** 2).mean())), 3))
    return out


def normalized_gap(gap_sim_real, gap_real_real):
    """per speed bin: sim-real tool0 gap divided by the real-real repeatability gap (same bins)"""
    out = {}
    for k, v in gap_sim_real["by_speed"].items():
        r = gap_real_real["by_speed"].get(k)
        if r and r["tcp_rms_mm"] > 0: out[k] = round(v["tcp_rms_mm"] / r["tcp_rms_mm"], 2)
    tot = gap_real_real["tcp_rms_mm"]
    out["all"] = round(gap_sim_real["tcp_rms_mm"] / tot, 2) if tot > 0 else None
    return out

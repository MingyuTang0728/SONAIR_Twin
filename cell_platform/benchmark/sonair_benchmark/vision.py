"""
vision -- the tool pose seen by the tool camera, made into a benchmark target.

The cell writes vision_logs/<run_id>.vtp.jsonl during each run (vision_track.py
on the host): the board's pose in every frame, raw. This turns that into

    the tool position the camera saw, in the robot's base frame, on the run's
    clock -- the VISION channel's target

and says how far it can be trusted:

    LATENCY   a frame is stamped when it reaches the host, after exposure and
              USB transfer. The delay is found by sliding the seen positions
              against the encoders' positions while the arm moves (it is the
              shift that makes them agree best), per run, and the session's
              median is applied -- one run's estimate can lean on its own
              dynamics, the median of a session cannot.
    FLOOR     the jitter of the seen position while the arm is still. Nothing
              smaller than this is a gap the camera can see.
    GATE      the camera is worth scoring only where it sees something the
              robot's own encoders do not. The p95 of |seen - encoder TCP| over
              moving samples, against the floor: under 3x, the channel is
              reported as provisional and does not rank.

Reference for this channel: K0 (rigid kinematics of the commanded joints),
not S0. S0's position error is mostly its servo lagging the command, which
says nothing about the arm; K0 is the strongest trivial model, so a vision
score says how much an entry explains beyond kinematics.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

GATE_RATIO = 3.0
LATENCY_RANGE_S = (0.0, 0.20)
DEFAULT_LATENCY_S = 0.040


def _kin():
    import sys
    root = Path(__file__).resolve().parent.parent
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    import ur_kin
    return ur_kin


def _rotmat(rv):
    rv = np.asarray(rv, dtype=float)
    th = float(np.linalg.norm(rv))
    if th < 1e-12:
        return np.eye(3)
    k = rv / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def _inv(T):
    T = np.asarray(T, dtype=float)
    o = np.eye(4)
    o[:3, :3] = T[:3, :3].T
    o[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return o


def read_vtp(path) -> tuple[dict, list, dict]:
    """(header, frame rows, summary)."""
    head, rows, summ = {}, [], {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        d = json.loads(line)
        if "_vtp" in d:
            head = d["_vtp"]
        elif "_summary" in d:
            summ = d["_summary"]
        else:
            rows.append(d)
    return head, rows, summ


def seen_positions(head: dict, rows: list, cal: dict | None = None):
    """
    (t, P[n,3] metres, n_corners, rep_px) for the frames where the board was
    seen. `cal` (T_tcp_cam, T_base_target) overrides the calibration recorded
    in the file -- a later, better calibration applied to old runs.
    """
    c = cal or head
    Tbb, Ttc = np.asarray(c["T_base_target"], float), np.asarray(c["T_tcp_cam"], float)
    Ttc_inv = _inv(Ttc)
    t, P, n, rep = [], [], [], []
    for r in rows:
        if not r.get("rvec"):
            continue
        Tcb = np.eye(4)
        Tcb[:3, :3] = _rotmat(r["rvec"])
        Tcb[:3, 3] = r["tvec"]
        T = Tbb @ _inv(Tcb) @ Ttc_inv
        t.append(float(r["t"]))
        P.append(T[:3, 3])
        n.append(int(r.get("n", 0)))
        rep.append(float(r.get("rep_px", 0.0)))
    return np.asarray(t), np.asarray(P).reshape(-1, 3), np.asarray(n), np.asarray(rep)


def tcp_offset(runs, robot: str = "ur5e", max_samples: int = 2000) -> np.ndarray:
    """
    T_flange_tcp from recorded runs: the controller's TCP against nominal
    forward kinematics of its encoders, averaged. It carries the pendant's TCP
    setting and the mean of the arm's factory calibration against nominal.

    Fit it on TRAIN runs only. Fitted on a test run it would hand that run's
    own static offset -- part of its hidden vision target -- to every model.
    """
    k = _kin()
    dh = k.DH.get(robot, k.UR5E_DH)
    runs = runs if isinstance(runs, (list, tuple)) else [runs]
    S = [s for r in runs for s in r.samples if s.get("q") and s.get("tcp_pos") and s.get("tcp_rot")]
    if not S:
        raise ValueError("no samples with q, tcp_pos and tcp_rot to fit the TCP offset")
    S = S[:: max(1, len(S) // max_samples)]
    Ts = []
    for s in S:
        Tc = np.eye(4)
        Tc[:3, :3] = _rotmat(s["tcp_rot"])
        Tc[:3, 3] = s["tcp_pos"]
        Ts.append(_inv(k.fk(s["q"], dh)) @ Tc)
    out = np.eye(4)
    U, _, Vt = np.linalg.svd(np.mean([T[:3, :3] for T in Ts], axis=0))
    out[:3, :3] = U @ Vt
    out[:3, 3] = np.mean([T[:3, 3] for T in Ts], axis=0)
    return out


def tcp_offset_from_config(v):
    """A configured TCP: [x, y, z] or the pendant's [x, y, z, rx, ry, rz] (m, rad) -> 4x4, or None."""
    if not v:
        return None
    v = [float(x) for x in v]
    T = np.eye(4)
    T[:3, 3] = v[:3]
    if len(v) >= 6:
        T[:3, :3] = _rotmat(v[3:6])
    return T


def fk_points(qs, T_ft, robot: str = "ur5e") -> np.ndarray:
    k = _kin()
    dh = k.DH.get(robot, k.UR5E_DH)
    return np.asarray([(k.fk(q, dh) @ T_ft)[:3, 3] for q in qs])


def _series(real, key):
    t = np.asarray([float(s["t"]) for s in real.samples if s.get(key)])
    v = np.asarray([s[key] for s in real.samples if s.get(key)], float)
    return t, v


def encoder_tcp(real):
    """The controller's own TCP (forward kinematics of its encoders), on the run clock."""
    return _series(real, "tcp_pos")


def latency(t_v, P_v, t_e, P_e, speed_min=0.02) -> dict:
    """
    The delay L that best aligns seen positions (stamped t_v = t_true + L)
    with the encoders' positions, over moving samples. Grid search, 1 ms.
    """
    if len(t_v) < 20 or len(t_e) < 20:
        return {"ok": False, "error": "too few samples"}
    sp = np.linalg.norm(np.gradient(P_e, t_e, axis=0), axis=1)
    moving = np.interp(t_v, t_e, sp) > speed_min
    if moving.sum() < 15:
        return {"ok": False, "error": "the tool hardly moved while the board was seen"}
    best = None
    for L in np.arange(LATENCY_RANGE_S[0], LATENCY_RANGE_S[1] + 1e-9, 0.001):
        Q = np.column_stack([np.interp(t_v - L, t_e, P_e[:, i]) for i in range(3)])
        # the constant part (calibration) is not latency: remove it first
        d = (P_v - Q)[moving]
        d = d - d.mean(axis=0)
        rms = float(np.sqrt(np.mean(np.sum(d ** 2, axis=1))))
        if best is None or rms < best[1]:
            best = (float(L), rms)
    return {"ok": True, "latency_s": round(best[0], 4), "rms_mm": round(best[1] * 1000, 3),
            "moving_frames": int(moving.sum())}


def floor(t_v, P_v, t_e, P_e, still_speed=0.002, min_frames=10) -> dict:
    """Jitter of the seen position over frames where the encoders say the tool is still."""
    if len(t_v) < min_frames or len(t_e) < 5:
        return {"ok": False, "error": "too few frames"}
    sp = np.linalg.norm(np.gradient(P_e, t_e, axis=0), axis=1)
    still = np.interp(t_v, t_e, sp) < still_speed
    if still.sum() < min_frames:
        return {"ok": False, "error": "no still stretch with the board seen"}
    # still frames come in stretches (between moves); jitter about each stretch's mean
    idx = np.where(still)[0]
    groups, cur = [], [idx[0]]
    for i in idx[1:]:
        if i == cur[-1] + 1:
            cur.append(i)
        else:
            groups.append(cur)
            cur = [i]
    groups.append(cur)
    dev = np.concatenate([np.linalg.norm(P_v[g] - P_v[g].mean(axis=0), axis=1)
                          for g in groups if len(g) >= 3] or [np.array([])])
    if dev.size < min_frames:
        return {"ok": False, "error": "still stretches too short"}
    return {"ok": True, "frames": int(dev.size),
            "rms_mm": round(float(np.sqrt(np.mean(dev ** 2))) * 1000, 3),
            "p95_mm": round(float(np.percentile(dev, 95)) * 1000, 3)}


def run_vision(real, vtp_path, *, latency_s: float | None = None, cal: dict | None = None,
               robot: str = "ur5e") -> dict:
    """
    One run's vision target and its diagnostics:
      t (run clock, latency removed), P (m), the encoder residual (moving and
      all), coverage, latency estimate, floor.
    """
    head, rows, summ = read_vtp(vtp_path)
    if not head:
        return {"ok": False, "error": f"{Path(vtp_path).name}: no header (no frame had intrinsics)"}
    t_v, P_v, n, rep = seen_positions(head, rows, cal)
    t_e, P_e = encoder_tcp(real)
    out = {"ok": True, "run_id": real.manifest.run_id, "frames": len(rows),
           "seen": int(len(t_v)), "coverage": round(len(t_v) / max(1, len(rows)), 3),
           "handeye_calib": head.get("handeye_calib", "")}
    if len(t_v) < 10 or len(t_e) < 10:
        out.update(ok=False, error="the board was seen in fewer than 10 frames")
        return out
    est = latency(t_v, P_v, t_e, P_e)
    out["latency_est"] = est
    L = latency_s if latency_s is not None else (est["latency_s"] if est.get("ok") else DEFAULT_LATENCY_S)
    out["latency_used_s"] = L
    tt = t_v - L
    inside = (tt >= t_e[0]) & (tt <= t_e[-1])
    tt, P = tt[inside], P_v[inside]
    E = np.column_stack([np.interp(tt, t_e, P_e[:, i]) for i in range(3)])
    sp = np.linalg.norm(np.gradient(P_e, t_e, axis=0), axis=1)
    moving = np.interp(tt, t_e, sp) > 0.02
    d = np.linalg.norm(P - E, axis=1) * 1000
    out["encoder_residual_mm"] = {
        "all_p95": round(float(np.percentile(d, 95)), 3) if d.size else None,
        "moving_p95": round(float(np.percentile(d[moving], 95)), 3) if moving.sum() >= 5 else None,
        "median": round(float(np.median(d)), 3) if d.size else None}
    out["floor"] = floor(t_v, P_v, t_e, P_e)
    out["t"], out["P"] = tt, P
    return out


def gate(run_results: list[dict]) -> dict:
    """Does the camera see anything the encoders do not? Over every run of a release."""
    fl = [r["floor"]["p95_mm"] for r in run_results if (r.get("floor") or {}).get("ok")]
    mv = [r["encoder_residual_mm"]["moving_p95"] for r in run_results
          if (r.get("encoder_residual_mm") or {}).get("moving_p95") is not None]
    if not fl or not mv:
        return {"verdict": "unmeasured", "ranked": False,
                "note": "no run had both a still stretch and a moving stretch with the board seen"}
    f, g = float(np.median(fl)), float(np.median(mv))
    ratio = g / f if f > 1e-9 else float("inf")
    ok = ratio >= GATE_RATIO
    return {"verdict": "pass" if ok else "below floor", "ranked": ok,
            "floor_p95_mm": round(f, 3), "beyond_encoders_p95_mm": round(g, 3),
            "ratio": round(ratio, 2),
            "note": ("the camera sees the tool move in ways its encoders do not, well above "
                     "its own jitter" if ok else
                     "what the camera sees beyond the encoders is within a few times its own "
                     "jitter: reported, not ranked, until the floor is lower or the effect larger")}


def session_latency(run_results: list[dict]) -> float:
    xs = [r["latency_est"]["latency_s"] for r in run_results
          if (r.get("latency_est") or {}).get("ok")]
    return float(np.median(xs)) if xs else DEFAULT_LATENCY_S

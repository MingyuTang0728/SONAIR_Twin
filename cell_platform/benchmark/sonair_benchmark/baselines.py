"""
baselines -- the ladder every entry is read against, made the way an entry
is made: from the PUBLIC package only, written as a submission, scored by the
same evaluator.

    S0  the reference simulator, unchanged                       (0 by definition)
    CM  the controller's own torque model, mapped to current
    K0  the angular rate the commanded joints imply (no vibration), and the
        tool point they imply -- the reference of the vision track (0 there)
    F1  physics fixes: S0 + Coulomb/viscous friction, and K0's rate
    L1  learned: a ridge regression on friction, inertia and the joint's
        angle-locked ripple, with its own uncertainty; for the vision track,
        K0 plus a learned correction for pose (the arm's kinematic
        calibration) and load (deflection under torque and acceleration)

Which joint the effort track scores is read from each run's own header
("effort_joint"), never assumed.

A benchmark with no learned baseline cannot show that its problem is
learnable; one whose baselines are made differently from entries will
eventually disagree with itself.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

from .task import ELBOW, read_jsonl


def _fill(vals, dim=None):
    """None-holed series -> array, gaps filled from the nearest known sample."""
    n = len(vals)
    if dim is None:
        a = np.array([np.nan if v is None else float(v) for v in vals])
        ok = np.isfinite(a)
        if not ok.any():
            return np.zeros(n)
        idx = np.arange(n)
        return np.interp(idx, idx[ok], a[ok])
    return np.column_stack([_fill([None if v is None else v[k] for v in vals]) for k in range(dim)])


def _cols(rows, head):
    m = head["_input"].get("effort_map") or {"a": 1.0, "b": 0.0}
    t = np.array([r["t"] for r in rows])
    q = _fill([r["cmd"]["q"] for r in rows], 6)
    qd = _fill([r["cmd"]["qd"] for r in rows], 6)
    tc = _fill([r["cmd"]["tau_ctrl"] for r in rows], 6)
    tau = _fill([r["sim"]["tau"] for r in rows], 6)
    sg = _fill([r["sim"]["gyro"] for r in rows], 3)
    ej = int(head["_input"].get("effort_joint", ELBOW))
    qdd = np.gradient(qd[:, ej], t) if len(t) > 2 else np.zeros(len(t))
    return {"t": t, "q": q, "qd": qd, "qdd": qdd, "tau_ctrl": tc, "tau": tau, "ej": ej,
            "s0_cur": m["a"] * tau[:, ej] + m["b"], "s0_gyro": sg}


def _k0_gyro(c, head):
    root = Path(__file__).resolve().parent.parent
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    import imu_align
    R = head["_input"].get("imu_mount")
    if not R:
        return c["s0_gyro"]
    return imu_align.flange_omega(c["q"], c["qd"]) @ np.asarray(R, float)


def _ripple(c, N):
    ph = N * c["q"][:, c["ej"]]
    v = np.abs(c["qd"][:, c["ej"]])
    return np.column_stack([np.sin(ph) * v, np.cos(ph) * v, np.sin(ph) * v * v, np.cos(ph) * v * v])


def _l1_x(c, N):
    v = c["qd"][:, c["ej"]]
    return np.column_stack([c["s0_cur"], c["tau_ctrl"][:, c["ej"]], np.tanh(v / 0.01), v, c["qdd"],
                            _ripple(c, N), np.ones(len(v))])


def _ridge(X, y, lam=1e-3):
    A = X.T @ X + lam * np.eye(X.shape[1])
    return np.linalg.solve(A, X.T @ y)


_K0_CACHE: dict = {}


def _k0_vis(c, head):
    """The tool point the commanded joints imply (rigid kinematics), per sample; None without a TCP offset."""
    rid = head["_input"]["run_id"].split("~")[0]
    T_ft = head["_input"].get("tcp_offset")
    if not T_ft:
        return None
    if rid not in _K0_CACHE:
        root = Path(__file__).resolve().parent.parent
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))
        import ur_kin
        dh = ur_kin.DH.get(head["_input"].get("robot", "ur5e"), ur_kin.UR5E_DH)
        T = np.asarray(T_ft, float)
        _K0_CACHE[rid] = np.asarray([(ur_kin.fk(q, dh) @ T)[:3, 3] for q in c["q"]])
        if len(_K0_CACHE) > 64:
            _K0_CACHE.pop(next(iter(_K0_CACHE)))
    return _K0_CACHE[rid]


def _nearest(t, tq):
    i = np.clip(np.searchsorted(t, tq), 1, len(t) - 1)
    return np.where(np.abs(t[i - 1] - tq) <= np.abs(t[i] - tq), i - 1, i)


def _vis_x(c):
    """Features of the tool point's departure from rigid kinematics: pose, and load."""
    q = c["q"]
    pose = [np.ones(len(q))]
    for j in (0, 1, 2):
        pose += [np.sin(q[:, j]), np.cos(q[:, j])]
    pose += [np.sin(q[:, 1] + q[:, 2]), np.cos(q[:, 1] + q[:, 2])]
    load = [c["tau"][:, j] for j in (1, 2, 3)] + [c["qdd"], c["qd"][:, c["ej"]]]
    return np.column_stack(pose + load)


def load_train(task_dir):
    out = []
    for p in sorted((Path(task_dir) / "train").glob("*.input.jsonl")):
        head, rows = read_jsonl(p)
        _h, tgt = read_jsonl(p.with_name(p.name.replace(".input.", ".target.")))
        c = _cols(rows, head)
        y = np.array([np.nan if r["current"] is None else r["current"] for r in tgt])
        out.append((head, c, y))
    return out


def fit(task_dir, log=print) -> dict:
    """Every fitted constant the ladder needs, from the train split only."""
    tr = load_train(task_dir)
    if not tr:
        raise ValueError("no train runs in the package")
    ys = np.concatenate([y for _, _, y in tr])
    ok = np.isfinite(ys)
    Xur = np.concatenate([np.column_stack([c["tau_ctrl"][:, c["ej"]], np.ones(len(y))]) for _, c, y in tr])
    ur = np.linalg.lstsq(Xur[ok], ys[ok], rcond=None)[0]
    Xf = np.concatenate([np.column_stack([np.tanh(c["qd"][:, c["ej"]] / 0.01), c["qd"][:, c["ej"]]])
                         for _, c, _ in tr])
    r0 = np.concatenate([y - c["s0_cur"] for _, c, y in tr])
    f1 = np.linalg.lstsq(Xf[ok], r0[ok], rcond=None)[0]
    # the ripple's count per joint revolution, read from the training data
    best = (np.inf, 220)
    for N in range(190, 251):
        X = np.concatenate([_l1_x(c, N) for _, c, _ in tr])
        w = _ridge(X[ok], ys[ok])
        res = np.mean((ys[ok] - X[ok] @ w) ** 2)
        if res < best[0]:
            best = (res, N)
    N = best[1]
    X = np.concatenate([_l1_x(c, N) for _, c, _ in tr])
    w = _ridge(X[ok], ys[ok])
    res = np.abs(ys - X @ w)
    v = np.concatenate([np.abs(c["qd"][:, c["ej"]]) for _, c, _ in tr])
    a_ = np.concatenate([np.abs(c["qdd"]) for _, c, _ in tr])
    S = np.column_stack([np.ones(ok.sum()), v[ok], a_[ok]])
    s = np.linalg.lstsq(S, res[ok] * math.sqrt(math.pi / 2), rcond=None)[0]   # E|e| -> sigma
    out = {"ur": ur.tolist(), "f1": f1.tolist(), "ripple_N": N, "l1": w.tolist(),
           "l1_sigma": s.tolist()}
    Xv, Yv = [], []
    for p in sorted((Path(task_dir) / "train").glob("*.vis.json")):
        doc = json.loads(p.read_text(encoding="utf-8"))
        head, rows = read_jsonl(p.with_name(p.name.replace(".vis.json", ".input.jsonl")))
        c = _cols(rows, head)
        k0 = _k0_vis(c, head)
        if k0 is None or not doc.get("t"):
            continue
        tv = np.asarray(doc["t"], float)
        keep = (tv >= c["t"][0]) & (tv <= c["t"][-1])
        tv = tv[keep]
        # K0 interpolated to each camera frame's own time; the slowly varying
        # features from the nearest sample. Pairing a frame with the next
        # sample instead puts the tool up to 8 ms -- millimetres -- out.
        k0_at = np.column_stack([np.interp(tv, c["t"], k0[:, i]) for i in range(3)])
        idx = _nearest(c["t"], tv)
        Xv.append(_vis_x(c)[idx])
        Yv.append(np.asarray(doc["p"], float)[keep] - k0_at)
    if Xv:
        Xv, Yv = np.concatenate(Xv), np.concatenate(Yv)
        out["l1_vis"] = np.column_stack([_ridge(Xv, Yv[:, i], 1e-4) for i in range(3)]).tolist()
        log(f"vision correction fitted on {len(Xv)} camera frames")
    log(f"baselines fitted on {len(tr)} train runs: ripple {N} per revolution")
    return out


LADDER = {
    "S0": ("S0 · reference simulator, unchanged", "A", "The reference. Scores 0 by definition."),
    "CM": ("CM · controller torque model", "B",
           "The controller's own torque estimate (tau_ctrl), mapped to current."),
    "K0": ("K0 · rigid kinematics", "A",
           "The angular rate the commanded joints imply, with no dynamics; S0's current."),
    "F1": ("F1 · S0 + friction, kinematic rate", "B",
           "Two friction coefficients fitted on the train runs, plus K0's angular rate."),
    "L1": ("L1 · learned (ridge regression)", "B",
           "Friction, inertia, the controller's model and the joint's angle-locked ripple, "
           "fitted on the train runs; with its own uncertainty."),
}


def predict(name: str, head, rows, P) -> dict:
    c = _cols(rows, head)
    cur, gy, sigma = c["s0_cur"], c["s0_gyro"], None
    if name == "CM":
        cur = P["ur"][0] * c["tau_ctrl"][:, c["ej"]] + P["ur"][1]
    if name in ("K0", "F1", "L1"):
        gy = _k0_gyro(c, head)
    if name == "F1":
        v = c["qd"][:, c["ej"]]
        cur = c["s0_cur"] + P["f1"][0] * np.tanh(v / 0.01) + P["f1"][1] * v
    if name == "L1":
        cur = _l1_x(c, P["ripple_N"]) @ np.asarray(P["l1"])
        s = P["l1_sigma"]
        sig = s[0] + s[1] * np.abs(c["qd"][:, c["ej"]]) + s[2] * np.abs(c["qdd"])
        sigma = {"current": np.maximum(sig, 1e-3).round(5).tolist()}
    out = {"run_id": head["_input"]["run_id"], "t": c["t"].round(6).tolist(),
           "current": np.asarray(cur).round(5).tolist(),
           "gyro": np.asarray(gy).round(6).tolist()}
    if name in ("K0", "F1", "L1"):
        k0 = _k0_vis(c, head)
        if k0 is not None:
            vis = k0
            if name == "L1" and P.get("l1_vis"):
                vis = k0 + _vis_x(c) @ np.asarray(P["l1_vis"])
            out["vis"] = np.asarray(vis).round(6).tolist()
    if sigma:
        out["sigma"] = sigma
    return out


def write_submissions(task_dir, out_dir, log=print) -> dict:
    task_dir, out_dir = Path(task_dir), Path(out_dir)
    P = fit(task_dir, log)
    tests = sorted((task_dir / "test").glob("*.input.jsonl"))
    made = {}
    for key, (nm, track, desc) in LADDER.items():
        d = out_dir / key
        d.mkdir(parents=True, exist_ok=True)
        with (d / "predictions.jsonl").open("w", encoding="utf-8") as fh:
            for p in tests:
                head, rows = read_jsonl(p)
                fh.write(json.dumps(predict(key, head, rows, P), separators=(",", ":")) + "\n")
        (d / "meta.json").write_text(json.dumps({
            "name": nm, "team": "Organisers", "track": track, "description": desc,
            "kind": "baseline", "runtime_s_per_run": None, "params": None}, indent=1),
            encoding="utf-8")
        made[key] = str(d)
    (out_dir / "fitted.json").write_text(json.dumps(P, indent=1), encoding="utf-8")
    return made

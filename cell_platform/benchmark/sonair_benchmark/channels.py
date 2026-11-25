"""
channels -- scoring the gap where the physics is: joint effort and the IMU.

WHY NOT ONLY POSITION. A UR's own controller follows its commanded joint
trajectory to a few hundredths of a degree, so the tool-position gap against
the COMMANDED joints is almost entirely the reference simulator's servo, not
physics: copying the command onto a kinematic model (K0) closes ~98% of it.
Two channels carry what a simulator genuinely gets wrong:

  effort  (primary)   the elbow motor current. The reference simulator's
                      torque explains ~6% of how it varies; friction (Coulomb
                      0.70 A, viscous ~1 A/(rad/s)) and a ripple locked to the
                      joint angle (~220 cycles per revolution) make up most of
                      the rest. Stable run to run, learnable, absent from S0.
  gyro    (secondary) the tool's angular rate from the IMU, in the IMU's own
                      axes. A kinematic model closes only about half of it;
                      the rest is the arm's structural vibration.
  accel   (provisional) the IMU's acceleration. Reported, not ranked, until
                      the six-face accelerometer calibration is under 0.5%.

UNITS. A simulator outputs torque (N m); the robot reports current (A). The
two are tied by ONE linear map per joint, current = a * tau + b, fitted once
on the PUBLISHED cells between S0's torque and the real current and published
with the board. Every entry's torque goes through that same map, so the map
can neither help nor hurt one entry against another. A Track B entry may give
current directly.

SCORING, exactly as for position: per run, the p95 and median of the
per-sample error; GCR = 1 - err(entry) / err(S0) per run; the mean over runs
of the HELD-OUT cells is the score; anything fitted (the map, a baseline's
friction) is fitted on the PUBLISHED cells only.
"""
from __future__ import annotations

import json
import math
import statistics
from pathlib import Path

import numpy as np

from .site import effort_joint as _effort_joint
ELBOW = _effort_joint("ur5e")       # the scored joint, from the embodiment card
CHANNELS = {
    "effort": {"name": "Joint effort", "what": "elbow motor current", "unit": "A",
               "role": "primary"},
    "gyro": {"name": "Tool angular rate", "what": "IMU gyroscope, its own axes",
             "unit": "rad/s", "role": "secondary"},
    "accel": {"name": "Tool acceleration", "what": "IMU accelerometer, its own axes",
              "unit": "m/s^2", "role": "provisional"},
}


# ---------------------------------------------------------------------------
# series
# ---------------------------------------------------------------------------

def _rows(run, getter):
    t, y = [], []
    for s in run.samples:
        try:
            v = getter(s)
        except (KeyError, TypeError, IndexError):
            continue
        if v is None:
            continue
        t.append(float(s["t"]))
        y.append(v)
    return np.asarray(t), np.asarray(y, dtype=float)


def imu_own(run, key="gyro"):
    """The IMU on its OWN timestamps: the rows hold the latest IMU sample at each robot row (sample-and-hold), so the
    same sample repeats and about a quarter of the samples never appear. Keep each sample once, at its own time."""
    t, y, last = [], [], None
    for s in run.samples:
        imu = ((s.get("sensors") or {}).get("imu_ind0")) or ((s.get("imu") or {}).get("ind0")) or {}
        ti, v = imu.get("t"), imu.get(key)
        if ti is None or v is None or (last is not None and ti <= last):
            continue
        t.append(float(ti)); y.append(v); last = ti
    return np.asarray(t), np.asarray(y, dtype=float)


def real_series(run, ch, lag_s=None):
    """lag_s: the IMU's measured lag behind the robot (imu_cal lag_s). Given, the gyro target is the IMU on its own
    timestamps, shifted by the lag and interpolated onto the robot rows (sonair-task/3); None keeps the v2 rows."""
    if ch == "effort":
        return _rows(run, lambda s: [s["sensors"]["ur_tcp"]["actual_current"][ELBOW]])
    if ch == "gyro" and lag_s is not None:
        ti, g = imu_own(run, "gyro")
        tr = np.asarray([float(s["t"]) for s in run.samples])
        if len(ti) < 20:
            return np.zeros(0), np.zeros((0, 3))
        keep = (tr >= ti[0] - lag_s) & (tr <= ti[-1] - lag_s)
        return tr[keep], np.column_stack([np.interp(tr[keep], ti - lag_s, g[:, k]) for k in range(3)])
    if ch == "gyro":
        return _rows(run, lambda s: s["imu"]["ind0"]["gyro"])
    if ch == "accel":
        return _rows(run, lambda s: s["imu"]["ind0"]["accel"])
    raise ValueError(ch)


def sim_series(run, ch, emap=None):
    """A simulated run as a prediction; effort goes through the published map."""
    if ch == "effort":
        a, b = (emap or {}).get("a", 1.0), (emap or {}).get("b", 0.0)
        return _rows(run, lambda s: [a * s["tau"][ELBOW] + b] if s.get("tau") else None)
    if ch == "gyro":
        return _rows(run, lambda s: s["imu"]["ind0"]["gyro"])
    if ch == "accel":
        return _rows(run, lambda s: s["imu"]["ind0"]["accel"])
    raise ValueError(ch)


def _on(t_to, t_from, y_from):
    """y_from resampled onto t_to (linear, clamped to its own span)."""
    if len(t_from) < 2:
        return None
    return np.column_stack([np.interp(t_to, t_from, y_from[:, k])
                            for k in range(y_from.shape[1])])


def errors(real, pred, ch, lag_s=None):
    """Per-sample error magnitude, on the real run's own samples."""
    tr, yr = real_series(real, ch, lag_s)
    if pred is None or len(tr) < 20:
        return None
    tp, yp = pred
    yp = _on(tr, tp, yp)
    if yp is None:
        return None
    keep = (tr >= tp[0]) & (tr <= tp[-1])
    if keep.sum() < 20:
        return None
    return np.linalg.norm(yr[keep] - yp[keep], axis=1)


# ---------------------------------------------------------------------------
# fitting, on published cells only
# ---------------------------------------------------------------------------

def fit_effort_map(fit_pairs) -> dict:
    """current = a * tau + b at the elbow, least squares over the published runs.

    sonair-task/3: fitted SEPARATELY for every simulator entry (its own runs on the published cells), not once on S0
    and reused. On benchmark-v1 S0's torque is dominated by its servo lag and correlates NEGATIVELY with the real
    current, so the S0 map has the wrong physical sign (a = -0.089 A/N m; the controller's own model gives +0.085).
    Pushed through that map, a simulator with correct rigid-body dynamics scored GCR -0.12; with its own map, +0.04,
    and +0.63 with two friction terms, against 0.38 for S0 + friction. Two nuisance parameters per entry, fitted on
    public data only, treat every entry the same way, and S0's own score is unchanged."""
    xs, ys = [], []
    for real, sim in fit_pairs:
        tr, yr = real_series(real, "effort")
        ts, ys_ = _rows(sim, lambda s: [s["tau"][ELBOW]] if s.get("tau") else None)
        if len(tr) < 20 or len(ts) < 20:
            continue
        xs.append(np.interp(tr, ts, ys_[:, 0]))
        ys.append(yr[:, 0])
    if not xs:
        return {"ok": False, "error": "no simulated torques: replay S0 again"}
    x, y = np.concatenate(xs), np.concatenate(ys)
    A = np.column_stack([x, np.ones_like(x)])
    (a, b), *_ = np.linalg.lstsq(A, y, rcond=None)
    return {"ok": True, "joint": "elbow", "a": round(float(a), 6), "b": round(float(b), 6),
            "n_samples": int(len(x)),
            "means": "current (A) = a * simulated elbow torque (N m) + b, fitted on the "
                     "published cells between S0 and the real arm; applied to every entry"}


def _cmd(real, key):
    return _rows(real, lambda s: s[key])


def preds_ur_model(pairs, fit_pairs) -> dict:
    """The controller's own torque model (target_moment), mapped the same way."""
    xs, ys = [], []
    for real, _ in fit_pairs:
        t, m = _cmd(real, "target_moment")
        tr, yr = real_series(real, "effort")
        if len(t) < 20:
            continue
        xs.append(np.interp(tr, t, m[:, ELBOW]))
        ys.append(yr[:, 0])
    x, y = np.concatenate(xs), np.concatenate(ys)
    (a, b), *_ = np.linalg.lstsq(np.column_stack([x, np.ones_like(x)]), y, rcond=None)
    out = {}
    for real, _ in pairs:
        t, m = _cmd(real, "target_moment")
        if len(t) >= 20:
            out[real.manifest.run_id] = (t, (a * m[:, ELBOW] + b)[:, None])
    return out


def _friction_terms(real, t_on):
    t, qd = _cmd(real, "target_qd")
    v = np.interp(t_on, t, qd[:, ELBOW])
    return np.column_stack([np.tanh(v / 0.01), v])


def preds_s0_friction(pairs, fit_pairs, emap) -> dict:
    """
    S0 plus a Coulomb + viscous friction term on the COMMANDED elbow speed,
    the two coefficients fitted on the published cells. What a first,
    physically obvious fix achieves.
    """
    X, Y = [], []
    for real, sim in fit_pairs:
        tr, yr = real_series(real, "effort")
        p = sim_series(sim, "effort", emap)
        if len(p[0]) < 20:
            continue
        X.append(_friction_terms(real, tr))
        Y.append(yr[:, 0] - np.interp(tr, p[0], p[1][:, 0]))
    coef, *_ = np.linalg.lstsq(np.vstack(X), np.concatenate(Y), rcond=None)
    out = {}
    for real, sim in pairs:
        ts, ys = sim_series(sim, "effort", emap)
        if len(ts) < 20:
            continue
        out[real.manifest.run_id] = (ts, (ys[:, 0] + _friction_terms(real, ts) @ coef)[:, None])
    return out, {"coulomb_A": round(float(coef[0]), 4), "viscous_A_per_rad_s": round(float(coef[1]), 4)}


def preds_k0_gyro(pairs, cal_for) -> dict:
    """The flange's angular rate from the commanded joints, in the IMU's axes."""
    import sys
    root = Path(__file__).resolve().parent.parent
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    import imu_align
    out = {}
    for real, _ in pairs:
        cal = cal_for(real.manifest.run_id) if cal_for else None
        if not cal or not cal.get("R_flange_imu"):
            continue
        R = np.asarray(cal["R_flange_imu"], dtype=float)
        rows = [(float(s["t"]), s["target_q"], s["target_qd"]) for s in real.samples
                if s.get("target_q") and s.get("target_qd")]
        if len(rows) < 20:
            continue
        t = np.array([r[0] for r in rows])
        w = imu_align.flange_omega(np.array([r[1] for r in rows]), np.array([r[2] for r in rows]))
        out[real.manifest.run_id] = (t, w @ R)       # R^T w, row-wise
    return out


# ---------------------------------------------------------------------------
# scoring
# ---------------------------------------------------------------------------

def score_channel(pairs, preds: dict, ref: dict, ch: str, name: str, *,
                  held: set, track: str, kind: str = "submission",
                  description: str = "") -> dict:
    """One entry on one channel, scored on the held-out cells against S0."""
    runs = []
    for real, _ in pairs:
        if real.manifest.cell_key() not in held:
            continue
        rid = real.manifest.run_id
        e = errors(real, preds.get(rid), ch)
        e0 = errors(real, ref.get(rid), ch)
        if e is None or e0 is None:
            continue
        p95, p950 = float(np.percentile(e, 95)), float(np.percentile(e0, 95))
        med, med0 = float(np.median(e)), float(np.median(e0))
        runs.append({"cell": real.manifest.cell_key(),
                     "gcr_p95": 1.0 - p95 / p950 if p950 > 1e-9 else 0.0,
                     "gcr_median": 1.0 - med / med0 if med0 > 1e-9 else 0.0})
    entry = {"name": name, "track": track, "kind": kind, "description": description,
             "channel": ch, "n_runs": len(runs)}
    if not runs:
        entry.update(gcr_p95=0.0, gcr_median=0.0, n_cells=0, per_cell={},
                     notes="no scorable runs on this channel")
        return entry
    entry["gcr_p95"] = statistics.fmean(r["gcr_p95"] for r in runs)
    entry["gcr_median"] = statistics.fmean(r["gcr_median"] for r in runs)
    cells = {}
    for r in runs:
        cells.setdefault(r["cell"], []).append(r)
    entry["per_cell"] = {k: {"n": len(v), "held_out": True,
                             "gcr_p95": statistics.fmean(x["gcr_p95"] for x in v),
                             "gcr_median": statistics.fmean(x["gcr_median"] for x in v)}
                         for k, v in cells.items()}
    entry["n_cells"] = len(cells)
    worst = min(entry["per_cell"], key=lambda k: entry["per_cell"][k]["gcr_p95"])
    entry["worst_cell"], entry["worst_cell_gcr"] = worst, entry["per_cell"][worst]["gcr_p95"]
    return entry


def published_levels(pairs, ref: dict, ch: str, held: set) -> dict:
    """S0's raw error on the PUBLISHED cells, by elbow speed: what the gap looks like."""
    by_v = {}
    for real, _ in pairs:
        if real.manifest.cell_key() in held:
            continue
        e = errors(real, ref.get(real.manifest.run_id), ch)
        if e is None:
            continue
        by_v.setdefault(round(float(real.manifest.joint_vel), 2), []).append(
            (float(np.median(e)), float(np.percentile(e, 95))))
    return [{"v": v, "median": round(statistics.fmean(x[0] for x in xs), 4),
             "p95": round(statistics.fmean(x[1] for x in xs), 4), "runs": len(xs)}
            for v, xs in sorted(by_v.items())]


def load_submission(path, ch: str) -> dict:
    """
    A Track B prediction file for one channel, JSON Lines, one run a line:
      {"run_id": ..., "t": [...], "current": [...]}           (effort, amperes)
      {"run_id": ..., "t": [...], "gyro": [[x, y, z], ...]}   (rad/s, IMU axes)
    """
    key = {"effort": "current", "gyro": "gyro", "accel": "accel"}[ch]
    out = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        d = json.loads(line)
        if key not in d:
            continue
        y = np.asarray(d[key], dtype=float)
        out[d["run_id"]] = (np.asarray(d["t"], dtype=float), y[:, None] if y.ndim == 1 else y)
    return out


def score_all(pairs, held: set, fit_pairs, cal_for=None, k0_runs=None, log=print) -> dict:
    """
    Every channel's board: S0, and the baselines that show how hard it is.
    Returns {channel: {meta, map?, entries: [...], published: [...]}}.
    """
    ref = {ch: {real.manifest.run_id: sim_series(sim, ch) for real, sim in pairs}
           for ch in ("gyro", "accel")}
    emap = fit_effort_map(fit_pairs)
    boards = {}
    if emap.get("ok"):
        ref["effort"] = {real.manifest.run_id: sim_series(sim, "effort", emap)
                         for real, sim in pairs}
        ent = [score_channel(pairs, ref["effort"], ref["effort"], "effort",
                             "S0 · reference simulator, unchanged", held=held, track="A",
                             kind="baseline", description="the reference; 0 by definition")]
        ent.append(score_channel(pairs, preds_ur_model(pairs, fit_pairs), ref["effort"], "effort",
                                 "CM · controller torque model", held=held, track="B",
                                 kind="baseline",
                                 description="the controller's own torque estimate, mapped to "
                                             "current the same way"))
        fp, coef = preds_s0_friction(pairs, fit_pairs, emap)
        ent.append(score_channel(pairs, fp, ref["effort"], "effort",
                                 "F1 · S0 + Coulomb and viscous friction", held=held, track="B",
                                 kind="baseline",
                                 description=f"two friction coefficients fitted on the published "
                                             f"cells ({coef['coulomb_A']} A, "
                                             f"{coef['viscous_A_per_rad_s']} A per rad/s)"))
        boards["effort"] = {**CHANNELS["effort"], "map": emap, "friction_fit": coef,
                            "entries": ent,
                            "published": published_levels(pairs, ref["effort"], "effort", held)}
    else:
        log("effort track skipped: " + emap.get("error", ""))
    ent = [score_channel(pairs, ref["gyro"], ref["gyro"], "gyro",
                         "S0 · reference simulator, unchanged", held=held, track="A",
                         kind="baseline", description="the reference; 0 by definition")]
    if cal_for is not None:
        ent.append(score_channel(pairs, preds_k0_gyro(pairs, cal_for), ref["gyro"], "gyro",
                                 "K0 · rigid kinematics (joints = command)", held=held, track="A",
                                 kind="baseline",
                                 description="the flange's rate from the commanded joints, in the "
                                             "IMU's axes: everything but the arm's vibration"))
    boards["gyro"] = {**CHANNELS["gyro"], "entries": ent,
                      "published": published_levels(pairs, ref["gyro"], "gyro", held)}
    boards["accel"] = {**CHANNELS["accel"],
                       "entries": [score_channel(pairs, ref["accel"], ref["accel"], "accel",
                                                 "S0 · reference simulator, unchanged", held=held,
                                                 track="A", kind="baseline",
                                                 description="the reference; 0 by definition")],
                       "published": published_levels(pairs, ref["accel"], "accel", held),
                       "note": "provisional: not ranked until the accelerometer's six-face "
                               "calibration is under 0.5%"}
    for b in boards.values():
        b["entries"].sort(key=lambda e: e.get("gcr_p95", 0), reverse=True)
    return boards

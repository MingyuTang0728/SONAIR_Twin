"""Audit of the real benchmark-v1 Session 1 data (108 runs, 54 cells x 2 repeats) with the sonair-twin tools.

Answers four questions with numbers, not opinions:
  1. Data quality: are the robot, IMU and timing streams fit to score against?
  2. Floors: how far apart are two real repeats of the same cell (the real-real gap) on each scored channel?
  3. Simulators: on the primary track (elbow current) how do S0 (Menagerie as shipped), the sonair-twin B2
     (inverse-dynamics servo) and simple physics corrections score, and how close can anything get (ceiling)?
  4. Metric consistency: does the published scoring rule (one torque->current map fitted on S0) treat a better
     simulator fairly, and does the IMU timing used by the scorer match the IMU's own timestamps?

Run:  python -m sonair audit-v1 --runs <bench_runs dir> --cal <imu_cal.json> [--out reports/v1_real]
"""
import json, math, os, time
import numpy as np
import pandas as pd
import mujoco
from .. import cell, realdata as R
from ..sim import mujoco_twin as tw

E = R.ELBOW
DT = 0.008                       # benchmark-v1 RTDE period (125 Hz)
TAIL = 0.5                       # tail sample: |commanded elbow acceleration| >= 0.5 x run peak (sonair-task/2)
CARRIER = {"label": "carrier_v1", "mass_kg": 0.19, "cog_m": [-0.012, -0.037, 0.008],
           "inertia_kgm2": [[2e-4, 0, 0], [0, 2e-4, 0], [0, 0, 2e-4]]}    # measured mass and CoM (carrier.json); inertia nominal


# ------------------------------------------------------------------ helpers
def tail_mask(t, tqd):
    a = np.abs(np.gradient(tqd[:, E], t))
    return a >= TAIL * a.max() if a.max() > 1e-6 else np.zeros(len(t), bool)


def moving(tqd, thr=1e-3):
    return np.abs(tqd).max(axis=1) > thr


def p95(x): return float(np.percentile(x, 95)) if len(x) else float("nan")


def qc(man, A):
    t = A["t_ctrl"]; dt = np.diff(t)
    off = A["t_host"] - A["t_ctrl"]; off = off - np.nanmin(off)
    ti = A["t_imu"]; dur = t[-1] - t[0]
    uniq_imu = len(np.unique(ti[np.isfinite(ti)]))
    rate = np.nanmedian(A["imu_rate"])
    mv = moving(A["tqd"])
    trk = np.degrees(A["tq"][:, E] - A["q"][:, E])
    return dict(n=len(t), duration_s=round(float(dur), 3),
                ctrl_period_anomalies=int(np.sum(np.abs(dt - DT) > 1e-4)), stale_rows=int(A["stale"].sum()),
                robot_age_p95_ms=round(float(np.nanpercentile(A["age"], 95)) * 1e3, 2),
                robot_age_max_ms=round(float(np.nanmax(A["age"])) * 1e3, 1),
                host_jitter_p99_ms=round(float(np.percentile(off, 99)) * 1e3, 2),
                host_jitter_max_ms=round(float(off.max()) * 1e3, 1),
                imu_rate_hz=round(float(rate), 1),
                imu_unique_frac=round(uniq_imu / max(1.0, dur * rate), 3),
                elbow_track_rms_deg=round(float(np.sqrt(np.mean(trk[mv] ** 2))), 4),
                elbow_track_max_deg=round(float(np.abs(trk[mv]).max()), 4),
                speed_scaling_min_moving=round(float(np.nanmin(A["speed_scaling"][mv])), 3) if mv.any() else None)


def align(ta, ya, tb, yb, coarse=None, span=0.06, step=0.0005):
    """shift s such that yb(t + s) best matches ya(t) (MSE), searched around a coarse guess"""
    s0 = coarse if coarse is not None else 0.0
    best = (np.inf, s0)
    for s in np.arange(s0 - span, s0 + span + 1e-12, step):
        yi = np.column_stack([np.interp(ta + s, tb, yb[:, k]) for k in range(yb.shape[1])])
        keep = (ta + s >= tb[0]) & (ta + s <= tb[-1])
        e = np.mean((yi[keep] - ya[keep]) ** 2)
        if e < best[0]: best = (e, s)
    return best[1]


def motion_start(t, tqd):
    i = np.argmax(moving(tqd)); return t[i]


# ------------------------------------------------------------------ simulators
def s0_replay(tq, dt=DT, settle=0.5):
    """S0 exactly as the platform's sim_mujoco.py builds it: Menagerie UR5e as shipped, carrier mass added to
    wrist_3_link, target_q held for one sample period; returns actuator torque and joint state per sample"""
    m = mujoco.MjModel.from_xml_path(os.path.join(cell.ASSETS, "universal_robots_ur5e", "scene.xml"))
    m.body_mass[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "wrist_3_link")] += CARRIER["mass_kg"]
    d = mujoco.MjData(m); d.qpos[:6] = tq[0]; d.ctrl[:6] = tq[0]; mujoco.mj_forward(m, d)
    for _ in range(int(settle / m.opt.timestep)): mujoco.mj_step(m, d)
    n_sub = max(1, int(round(dt / m.opt.timestep)))
    tau = np.empty((len(tq), 6)); q = np.empty((len(tq), 6)); qd = np.empty((len(tq), 6))
    for k in range(len(tq)):
        d.ctrl[:6] = tq[k]
        for _ in range(n_sub): mujoco.mj_step(m, d)
        tau[k] = d.actuator_force[:6]; q[k] = d.qpos[:6]; qd[k] = d.qvel[:6]
    return tau, q, qd


def b2_replay(t, tq, tqd):
    df = pd.DataFrame({"t": t, **{f"tq{i}": tq[:, i] for i in range(6)}, **{f"tqd{i}": tqd[:, i] for i in range(6)}})
    _, out = tw.replay(df, payload_key=CARRIER, variant="B2_pd_invdyn", settle=0.5)
    return out[[f"tau{i}" for i in range(6)]].to_numpy(), out[[f"q{i}" for i in range(6)]].to_numpy(), out[[f"qd{i}" for i in range(6)]].to_numpy()


class Flange:
    """angular velocity of tool0 in tool0 axes, from joint positions and velocities (Menagerie kinematics)"""
    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(cell.UR5E_XML); self.d = mujoco.MjData(self.m)
        self.s = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, cell.SITE)
        self.jr = np.zeros((3, self.m.nv))

    def omega(self, Q, QD):
        out = np.empty((len(Q), 3))
        for k in range(len(Q)):
            self.d.qpos[:6] = Q[k]; mujoco.mj_kinematics(self.m, self.d); mujoco.mj_comPos(self.m, self.d)
            mujoco.mj_jacSite(self.m, self.d, None, self.jr, self.s)
            Rw = self.d.site_xmat[self.s].reshape(3, 3)
            out[k] = Rw.T @ (self.jr[:, :6] @ QD[k])
        return out


# ------------------------------------------------------------------ main analysis
def run(runs_dir, cal_path, out_dir, cache=None, log=print):
    t0 = time.time()
    D = R.load_dir(runs_dir, "*_r0[03].jsonl", cache=cache)
    cal = json.load(open(cal_path)); Rfi = np.asarray(cal["R_flange_imu"]); lag = float(cal.get("lag_s", 0.0))
    os.makedirs(out_dir, exist_ok=True)
    fl = Flange()
    per = {}
    for i, (rid, (man, A)) in enumerate(sorted(D.items())):
        t = A["t_ctrl"] - A["t_ctrl"][0]; tq, tqd = A["tq"], A["tqd"]
        s0_tau, s0_q, s0_qd = s0_replay(tq)
        b2_tau, _, _ = b2_replay(t, tq, tqd)
        k0_w = fl.omega(tq, tqd) @ Rfi                 # commanded flange rate in IMU axes (R^T w)
        s0_w = fl.omega(s0_q, s0_qd) @ Rfi
        per[rid] = dict(man=man, A=A, t=t, s0_tau=s0_tau, b2_tau=b2_tau, k0_w=k0_w, s0_w=s0_w,
                        tail=tail_mask(t, tqd), qc=qc(man, A), held=R.held_out(man))
        if i % 12 == 0: log(f"  replayed {i + 1}/{len(D)} ({time.time() - t0:.0f} s)")

    pub = [r for r in per.values() if not r["held"]]
    held = [r for r in per.values() if r["held"]]
    cur = lambda r: r["A"]["cur"][:, E]

    # ---- torque -> current maps, fitted on PUBLISHED cells only
    def fit_map(key):
        x = np.concatenate([r[key][:, E] for r in pub]); y = np.concatenate([cur(r) for r in pub])
        (a, b), *_ = np.linalg.lstsq(np.column_stack([x, np.ones_like(x)]), y, rcond=None); return float(a), float(b)
    maps = {"S0": fit_map("s0_tau"), "B2": fit_map("b2_tau")}
    for r in per.values(): r["tm"] = r["A"]["tm"]
    maps["CM"] = fit_map("tm")

    def fric(r):
        v = r["A"]["tqd"][:, E]; return np.column_stack([np.tanh(v / 0.01), v])

    def fit_fric(pred_key):
        X = np.vstack([fric(r) for r in pub]); Y = np.concatenate([cur(r) - r[pred_key] for r in pub])
        c, *_ = np.linalg.lstsq(X, Y, rcond=None); return c

    # predictions of the elbow current (A)
    a0, b0 = maps["S0"]; a2, b2 = maps["B2"]; ac, bc = maps["CM"]
    for r in per.values():
        r["p_S0"] = a0 * r["s0_tau"][:, E] + b0
        r["p_CM"] = ac * r["tm"][:, E] + bc
        r["p_B2_s0map"] = a0 * r["b2_tau"][:, E] + b0          # the published rule: S0's map applied to every entry
        r["p_B2"] = a2 * r["b2_tau"][:, E] + b2                # the same simulator with its own map
        r["p_B2_cmmap"] = ac * r["b2_tau"][:, E] + bc          # proposed rule: one fixed physical map (controller torque -> current)
    cF1 = fit_fric("p_S0"); cB2F = fit_fric("p_B2"); cB2Fc = fit_fric("p_B2_cmmap")
    for r in per.values():
        r["p_F1"] = r["p_S0"] + fric(r) @ cF1
        r["p_B2F"] = r["p_B2"] + fric(r) @ cB2F
        r["p_B2F_cmmap"] = r["p_B2_cmmap"] + fric(r) @ cB2Fc

    # ripple locked to the elbow angle: harmonic k per revolution, amplitude growing with speed
    def rip(r, k):
        qe = r["A"]["tq"][:, E]; v = np.abs(r["A"]["tqd"][:, E])
        s, c = np.sin(k * qe), np.cos(k * qe); return np.column_stack([s, c, s * v, c * v])
    best = None
    for k in np.arange(200.0, 241.0, 1.0):
        X = np.vstack([rip(r, k) for r in pub]); Y = np.concatenate([cur(r) - r["p_B2F"] for r in pub])
        c, res, *_ = np.linalg.lstsq(X, Y, rcond=None); sse = float(np.sum((Y - X @ c) ** 2))
        if best is None or sse < best[0]: best = (sse, k, c)
    k_rip, c_rip = best[1], best[2]
    # refine k to 0.05 resolution around the best integer
    for k in np.arange(k_rip - 1, k_rip + 1.0001, 0.05):
        X = np.vstack([rip(r, k) for r in pub]); Y = np.concatenate([cur(r) - r["p_B2F"] for r in pub])
        c, *_ = np.linalg.lstsq(X, Y, rcond=None); sse = float(np.sum((Y - X @ c) ** 2))
        if sse < best[0]: best = (sse, k, c)
    k_rip, c_rip = best[1], best[2]
    for r in per.values(): r["p_B2FR"] = r["p_B2F"] + rip(r, k_rip) @ c_rip

    # ---- real-real floor: r00 vs r03 of the same cell, aligned on the command
    floors = {}
    for rid, r in per.items():
        if not rid.endswith("_r00"): continue
        o = per.get(rid[:-4] + "_r03")
        if o is None: continue
        ta, tb = r["A"]["t_ctrl"], o["A"]["t_ctrl"]
        coarse = motion_start(tb, o["A"]["tqd"]) - motion_start(ta, r["A"]["tqd"])
        s = align(ta, r["A"]["tq"], tb, o["A"]["tq"], coarse=coarse)
        keep = (ta + s >= tb[0]) & (ta + s <= tb[-1])
        ib = np.interp(ta + s, tb, o["A"]["cur"][:, E])
        e_cur = np.abs(cur(r) - ib)
        tcp_b = np.column_stack([np.interp(ta + s, tb, o["A"]["tcp"][:, k]) for k in range(3)])
        e_tcp = np.linalg.norm(r["A"]["tcp"] - tcp_b, axis=1) * 1e3
        # gyro on the host clock with the IMU's own stamps (sample-and-hold rows removed)
        def gyro_own(x):
            ti = x["A"]["t_imu"]; g = x["A"]["gyro"]; ok = np.isfinite(ti) & np.r_[True, np.diff(ti) > 0]
            return ti[ok], g[ok]
        tia, ga = gyro_own(r); tib, gb = gyro_own(o)
        sh = (np.nanmedian(o["A"]["t_host"] - o["A"]["t_ctrl"]) - np.nanmedian(r["A"]["t_host"] - r["A"]["t_ctrl"])) + s
        gbi = np.column_stack([np.interp(tia + sh, tib, gb[:, k]) for k in range(3)])
        kg = (tia + sh >= tib[0]) & (tia + sh <= tib[-1])
        e_g = np.linalg.norm(ga[kg] - gbi[kg], axis=1)
        tail = r["tail"] & keep
        floors[R.cell_key(rid)] = dict(shift_s=round(float(s), 4),
                                       cur_p95=p95(e_cur[keep]), cur_med=float(np.median(e_cur[keep])),
                                       cur_tail_p95=p95(e_cur[tail]), tcp_p95_mm=p95(e_tcp[keep]),
                                       gyro_p95=p95(e_g), gyro_med=float(np.median(e_g)))

    # ---- scoring on held-out runs: p95 of |error| per run, GCR vs S0, mean over runs
    names = {"S0": "S0 Menagerie as shipped (reference)", "CM": "CM controller torque model",
             "F1": "F1 S0 + Coulomb/viscous friction", "B2_s0map": "B2 twin, scored with S0's map (published rule)",
             "B2": "B2 twin (inverse-dynamics servo), own map", "B2_cmmap": "B2 twin, fixed physical map (proposed rule)",
             "B2F": "B2 + Coulomb/viscous friction", "B2F_cmmap": "B2 + friction, fixed physical map (proposed rule)",
             "B2FR": f"B2 + friction + angle-locked ripple (k={k_rip:.2f}/rev)"}
    rows = []
    for r in held:
        rid = r["man"]["run_id"]; y = cur(r); tl = r["tail"]; ck = R.cell_key(rid)
        e0 = np.abs(y - r["p_S0"])
        row = dict(run=rid, cell=ck, v=r["man"]["joint_vel"], S0_p95=p95(e0), S0_tail_p95=p95(e0[tl]),
                   floor_p95=floors.get(ck, {}).get("cur_p95"), floor_tail_p95=floors.get(ck, {}).get("cur_tail_p95"))
        for key in names:
            e = np.abs(y - r["p_" + key])
            row[f"{key}_gcr"] = 1 - p95(e) / p95(e0); row[f"{key}_gcr_tail"] = 1 - p95(e[tl]) / p95(e0[tl])
        row["ceiling_gcr"] = 1 - row["floor_p95"] / row["S0_p95"] if row["floor_p95"] else None
        row["ceiling_gcr_tail"] = 1 - row["floor_tail_p95"] / row["S0_tail_p95"] if row["floor_tail_p95"] else None
        rows.append(row)
    sc = pd.DataFrame(rows)
    board = []
    rng = np.random.default_rng(0); B = rng.integers(0, len(sc), (2000, len(sc)))     # bootstrap over held-out runs
    for key, nm in names.items():
        v, vt = sc[f"{key}_gcr"].to_numpy(), sc[f"{key}_gcr_tail"].to_numpy()
        ci = np.percentile(v[B].mean(axis=1), [2.5, 97.5]); cit = np.percentile(vt[B].mean(axis=1), [2.5, 97.5])
        board.append(dict(entry=key, name=nm, gcr_p95=round(float(v.mean()), 3), ci95=[round(float(x), 3) for x in ci],
                          gcr_p95_tail=round(float(vt.mean()), 3), ci95_tail=[round(float(x), 3) for x in cit]))
    d = (sc["B2F_gcr"] - sc["F1_gcr"]).to_numpy(); dci = np.percentile(d[B].mean(axis=1), [2.5, 97.5])
    paired = dict(B2F_minus_F1=round(float(d.mean()), 3), ci95=[round(float(x), 3) for x in dci], runs_B2F_better=int((d > 0).sum()), runs=len(d))
    # ---- proposed rule: ONE fixed drive map, a property of the real robot, fitted on still samples of the published
    # cells between the controller's own model torque and the measured current; no entry's output is used to fit it
    xs, ys = [], []
    for r in pub:
        still = ~moving(r["A"]["tqd"]); xs.append(r["tm"][still, E]); ys.append(cur(r)[still])
    x, y = np.concatenate(xs), np.concatenate(ys)
    (kd, bd), *_ = np.linalg.lstsq(np.column_stack([x, np.ones_like(x)]), y, rcond=None)
    drive = dict(a=round(float(kd), 5), b=round(float(bd), 4), n_still=int(len(x)),
                 rmse_A=round(float(np.sqrt(np.mean((y - kd * x - bd) ** 2))), 4))
    for r in per.values():
        r["d_S0"] = kd * r["s0_tau"][:, E] + bd; r["d_B2"] = kd * r["b2_tau"][:, E] + bd; r["d_CM"] = kd * r["tm"][:, E] + bd
    cdF1, cdB2F = fit_fric("d_S0"), fit_fric("d_B2")
    for r in per.values():
        r["d_F1"] = r["d_S0"] + fric(r) @ cdF1; r["d_B2F"] = r["d_B2"] + fric(r) @ cdB2F
    best = None
    for k in np.arange(k_rip - 1, k_rip + 1.0001, 0.05):
        X = np.vstack([rip(r, k) for r in pub]); Y = np.concatenate([cur(r) - r["d_B2F"] for r in pub])
        c, *_ = np.linalg.lstsq(X, Y, rcond=None); sse = float(np.sum((Y - X @ c) ** 2))
        if best is None or sse < best[0]: best = (sse, k, c)
    for r in per.values(): r["d_B2FR"] = r["d_B2F"] + rip(r, best[1]) @ best[2]
    board_drive = []
    for key in ("S0", "CM", "F1", "B2", "B2F", "B2FR"):
        g_all, g_tail = [], []
        for r in held:
            y = cur(r); tl = r["tail"]; e0 = np.abs(y - r["d_S0"]); e = np.abs(y - r["d_" + key])
            g_all.append(1 - p95(e) / p95(e0)); g_tail.append(1 - p95(e[tl]) / p95(e0[tl]))
        board_drive.append(dict(entry=key, gcr_p95=round(float(np.mean(g_all)), 3), gcr_p95_tail=round(float(np.mean(g_tail)), 3)))
    S0d = [p95(np.abs(cur(r) - r["d_S0"])) for r in held]; S0dt = [p95(np.abs(cur(r) - r["d_S0"])[r["tail"]]) for r in held]
    cl = [1 - floors[R.cell_key(r["man"]["run_id"])]["cur_p95"] / a for r, a in zip(held, S0d)]
    clt = [1 - floors[R.cell_key(r["man"]["run_id"])]["cur_tail_p95"] / a for r, a in zip(held, S0dt)]
    drive_rule = dict(map=drive, board=board_drive, S0_p95_A=round(float(np.mean(S0d)), 4),
                      ceiling=dict(gcr_p95=round(float(np.mean(cl)), 3), gcr_p95_tail=round(float(np.mean(clt)), 3)),
                      ripple_cycles_per_rev=round(float(best[1]), 2))

    by_v = sc.groupby("v").agg(S0_p95=("S0_p95", "mean"), floor_p95=("floor_p95", "mean"), B2F_gcr=("B2F_gcr", "mean"),
                               F1_gcr=("F1_gcr", "mean"), ceiling=("ceiling_gcr", "mean"), runs=("run", "count")).round(3)
    by_v.to_csv(os.path.join(out_dir, "current_by_speed.csv"))
    ceiling = dict(gcr_p95=round(float(sc["ceiling_gcr"].mean()), 3), gcr_p95_tail=round(float(sc["ceiling_gcr_tail"].mean()), 3))

    # ---- gyro track: K0 (commanded kinematics) vs S0, two timing conventions
    def gyro_err(r, pred, mode):
        A = r["A"]
        if mode == "rows":                # what the platform scorer does: IMU sample-and-hold on the robot rows
            return np.linalg.norm(A["gyro"] - pred, axis=1)
        if mode == "rows_task3":          # the task/3 patch: IMU on its own stamps, minus the lag, onto the robot rows
            ti = A["t_imu"]; ok = np.isfinite(ti) & np.r_[True, np.diff(ti) > 0]; tr = A["t_host"]
            keep = (tr >= ti[ok][0] - lag) & (tr <= ti[ok][-1] - lag)
            tg = np.column_stack([np.interp(tr[keep], ti[ok] - lag, A["gyro"][ok, k]) for k in range(3)])
            return np.linalg.norm(tg - pred[keep], axis=1)
        ti = A["t_imu"]; ok = np.isfinite(ti) & np.r_[True, np.diff(ti) > 0]
        th = A["t_ctrl"] + np.nanmedian(A["t_host"] - A["t_ctrl"]) + lag   # command time on the host clock + IMU lag
        pi = np.column_stack([np.interp(ti[ok], th, pred[:, k]) for k in range(3)])
        return np.linalg.norm(A["gyro"][ok] - pi, axis=1)
    g = {}
    for mode in ("rows", "rows_task3", "own_stamps"):
        k0 = [1 - p95(gyro_err(r, r["k0_w"], mode)) / p95(gyro_err(r, r["s0_w"], mode)) for r in held]
        g[mode] = dict(K0_gcr_p95=round(float(np.mean(k0)), 3),
                       K0_p95_rad_s=round(float(np.mean([p95(gyro_err(r, r["k0_w"], mode)) for r in held])), 4),
                       S0_p95_rad_s=round(float(np.mean([p95(gyro_err(r, r["s0_w"], mode)) for r in held])), 4))
    fl_g = [floors[R.cell_key(r["man"]["run_id"])]["gyro_p95"] for r in held if R.cell_key(r["man"]["run_id"]) in floors]
    g["real_real_p95_rad_s"] = round(float(np.mean(fl_g)), 4)

    q = pd.DataFrame([dict(run=k, **v["qc"]) for k, v in per.items()])
    fl = pd.DataFrame([dict(cell=k, **v) for k, v in floors.items()])
    summary = dict(
        data=dict(runs=len(per), cells=len(floors), held_out_runs=len(held), samples=int(q["n"].sum()),
                  hours=round(float(q["duration_s"].sum()) / 3600, 3)),
        quality=dict(ctrl_period_anomalies=int(q["ctrl_period_anomalies"].sum()), stale_rows=int(q["stale_rows"].sum()),
                     robot_age_p95_ms=round(float(q["robot_age_p95_ms"].median()), 2), robot_age_max_ms=float(q["robot_age_max_ms"].max()),
                     host_jitter_p99_ms=round(float(q["host_jitter_p99_ms"].median()), 2), host_jitter_max_ms=float(q["host_jitter_max_ms"].max()),
                     imu_rate_hz=float(q["imu_rate_hz"].median()), imu_unique_frac_median=float(q["imu_unique_frac"].median()),
                     imu_unique_frac_min=float(q["imu_unique_frac"].min()),
                     elbow_track_rms_deg=round(float(q["elbow_track_rms_deg"].median()), 4),
                     speed_scaling_min_moving=float(q["speed_scaling_min_moving"].min())),
        floors=dict(current_p95_A=round(float(fl["cur_p95"].mean()), 4), current_tail_p95_A=round(float(fl["cur_tail_p95"].mean()), 4),
                    current_median_A=round(float(fl["cur_med"].mean()), 4), tcp_p95_mm=round(float(fl["tcp_p95_mm"].mean()), 3),
                    tcp_p95_mm_max=round(float(fl["tcp_p95_mm"].max()), 3), gyro_p95_rad_s=round(float(fl["gyro_p95"].mean()), 4)),
        current=dict(maps={k: dict(a=round(v[0], 5), b=round(v[1], 4)) for k, v in maps.items()},
                     friction_F1=dict(coulomb_A=round(float(cF1[0]), 3), viscous_A_per_rad_s=round(float(cF1[1]), 3)),
                     friction_B2F=dict(coulomb_A=round(float(cB2F[0]), 3), viscous_A_per_rad_s=round(float(cB2F[1]), 3)),
                     ripple=dict(cycles_per_rev=round(float(k_rip), 2), coef=[round(float(c), 4) for c in c_rip]),
                     board=board, paired=paired, ceiling=ceiling, drive_map_rule=drive_rule,
                     S0_p95_A=round(float(sc["S0_p95"].mean()), 4), S0_tail_p95_A=round(float(sc["S0_tail_p95"].mean()), 4)),
        gyro=g, runtime_s=round(time.time() - t0, 1))
    with open(os.path.join(out_dir, "audit_v1.json"), "w") as f: json.dump(summary, f, indent=2)
    q.to_csv(os.path.join(out_dir, "quality_per_run.csv"), index=False)
    fl.to_csv(os.path.join(out_dir, "floors_per_cell.csv"), index=False)
    sc.to_csv(os.path.join(out_dir, "current_scores_heldout.csv"), index=False)
    _figures(per, sc, summary, out_dir)
    return summary


def _figures(per, sc, summary, out_dir):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    ink, muted, grid = "#1f2328", "#59636e", "#d8dee4"
    col = {"real": "#1f2328", "S0": "#c2410c", "B2": "#2563eb", "B2FR": "#15803d"}
    rid = "v0p50_mid_workspace_contour_r00" if "v0p50_mid_workspace_contour_r00" in per else sorted(per)[0]
    r = per[rid]; t = r["t"]
    fig, ax = plt.subplots(figsize=(9, 3.6))
    ax.plot(t, r["A"]["cur"][:, E], color=col["real"], lw=1.6, label="real elbow current")
    ax.plot(t, r["p_S0"], color=col["S0"], lw=1.2, label="S0 (Menagerie as shipped)")
    ax.plot(t, r["p_B2"], color=col["B2"], lw=1.2, label="B2 twin")
    ax.plot(t, r["p_B2FR"], color=col["B2FR"], lw=1.2, label="B2 + friction + ripple")
    ax.set_xlabel("time (s)"); ax.set_ylabel("current (A)"); ax.grid(color=grid, lw=0.6)
    ax.set_title(f"{rid} (held-out cell)", color=ink, fontsize=10, loc="left")
    ax.legend(frameon=False, fontsize=8, ncol=2); [s.set_visible(False) for s in (ax.spines["top"], ax.spines["right"])]
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, "fig_current_example.png"), dpi=160); plt.close(fig)

    b = summary["current"]["board"]; c = summary["current"]["ceiling"]
    labels = [x["entry"] for x in b] + ["ceiling"]
    v_all = [x["gcr_p95"] for x in b] + [c["gcr_p95"]]; v_tail = [x["gcr_p95_tail"] for x in b] + [c["gcr_p95_tail"]]
    y = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=(8, 3.8))
    ax.barh(y - 0.18, v_all, 0.34, color="#2563eb", label="all samples")
    ax.barh(y + 0.18, v_tail, 0.34, color="#93c5fd", label="tail samples (headline)")
    ax.set_yticks(y); ax.set_yticklabels(labels, fontsize=8); ax.invert_yaxis(); ax.axvline(0, color=muted, lw=0.8)
    ax.set_xlabel("GCR p95 on held-out runs (1 = perfect, 0 = S0)"); ax.grid(axis="x", color=grid, lw=0.6)
    for yy, va, vt in zip(y, v_all, v_tail):
        ax.text(va + 0.01, yy - 0.18, f"{va:.2f}", va="center", fontsize=7, color=ink)
        ax.text(vt + 0.01, yy + 0.18, f"{vt:.2f}", va="center", fontsize=7, color=ink)
    ax.legend(frameon=False, fontsize=8, loc="upper right"); ax.set_xlim(-0.25, 1.0); [s.set_visible(False) for s in (ax.spines["top"], ax.spines["right"])]
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, "fig_current_board.png"), dpi=160); plt.close(fig)

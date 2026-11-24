"""Session QC: is this run fit to be scored? Every check is a number with a threshold, written to qc.json.

FAIL blocks the run from the dataset; WARN keeps it but flags it in the manifest. Thresholds come from what the
benchmark-v1 data showed is achievable (0 controller period anomalies, 0 stale rows, elbow tracking 0.017 deg RMS) and
from what the metrics need (the IMU must be on its own clock with every sample kept).
"""
import json, os
import numpy as np
import pandas as pd
from .clock import ClockMap
from .. import realdata

GATES = dict(rtde_missing_frac=0.001, rtde_max_gap_ms=20.0, clock_delay_p95_ms=2.0, clock_drift_ppm=200.0,
             imu_rate_err=0.02, imu_gap_factor=3.0, imu_coverage=0.99, imu_drift_ppm=1000.0, imu_robot_corr=0.95, imu_lag_ms=5.0)


def _rtde(run_dir):
    df = realdata_rtde(os.path.join(run_dir, "rtde.csv"))
    return df


def realdata_rtde(path):
    raw = pd.read_csv(path)
    raw.columns = [c.strip() for c in raw.columns]
    return raw


def _cols(df, base):
    return df[[f"{base}_{i}" for i in range(6)]].to_numpy()


def audit(run_dir, imu_cal=None, write=True):
    reasons, warn, out = [], [], {}
    df = _rtde(run_dir)
    t = df["timestamp"].to_numpy(); dt = np.diff(t)
    missing = np.clip(np.round(dt / 0.002) - 1, 0, None).sum()
    out["rtde"] = dict(n=len(t), duration_s=round(float(t[-1] - t[0]), 3), median_period_ms=round(float(np.median(dt)) * 1e3, 4),
                       missing_samples=int(missing), max_gap_ms=round(float(dt.max()) * 1e3, 2),
                       non_monotonic=int((dt <= 0).sum()))
    if out["rtde"]["non_monotonic"]: reasons.append("RTDE timestamps not increasing")
    if missing / max(1, len(t)) > GATES["rtde_missing_frac"]: reasons.append(f"RTDE lost {missing:.0f} samples")
    if out["rtde"]["max_gap_ms"] > GATES["rtde_max_gap_ms"]: reasons.append(f"RTDE gap of {out['rtde']['max_gap_ms']} ms")
    tqd = _cols(df, "target_qd"); mv = np.abs(tqd).max(axis=1) > 1e-3
    if "speed_scaling" in df and mv.any():
        ss = float(df["speed_scaling"].to_numpy()[mv].min()); out["rtde"]["speed_scaling_min_moving"] = ss
        if ss < 0.999: reasons.append(f"speed scaling fell to {ss:.3f} while moving: the trajectory was not executed as commanded")
    for k, want in (("safety_mode", 1), ("robot_mode", 7)):
        if k in df and (df[k].to_numpy() != want).any(): reasons.append(f"{k} left {want} during the run")
    if mv.any():
        e = np.degrees(np.abs(_cols(df, "target_q")[:, 2] - _cols(df, "actual_q")[:, 2]))[mv]
        out["rtde"]["elbow_track_rms_deg"] = round(float(np.sqrt(np.mean(e ** 2))), 4)

    ck = np.load(os.path.join(run_dir, "rtde_clock.npz"))
    cm_ = ClockMap().fit(ck["ctrl"], ck["host"]); out["rtde_clock"] = cm_.summary()
    if cm_.resid_p95 * 1e3 > GATES["clock_delay_p95_ms"]: warn.append(f"RTDE receive delay p95 {cm_.resid_p95 * 1e3:.2f} ms above the envelope")
    if abs(cm_.drift_ppm()) > GATES["clock_drift_ppm"]: reasons.append(f"controller/host drift {cm_.drift_ppm():.0f} ppm: clock pairs are wrong")
    t_host = cm_.to_host(t)

    out["imu"] = {}
    for f in sorted(os.listdir(run_dir)):
        if not (f.startswith("imu_") and f.endswith(".npz")): continue
        name = f[4:-4]; z = np.load(os.path.join(run_dir, f))
        td, g = z["t_dev"], z["gyro"]; r = {}
        if len(td) < 20: reasons.append(f"IMU {name}: no data"); continue
        d = np.diff(td); man = json.load(open(os.path.join(run_dir, "manifest.json")))
        odr = float(man.get("imu_odr") or 1.0 / np.median(d))
        r.update(n=int(len(td)), rate_hz=round(float(1.0 / np.median(d)), 2), non_monotonic=int((d <= 0).sum()),
                 gaps=int((d > GATES["imu_gap_factor"] / odr).sum()))
        if r["non_monotonic"]: reasons.append(f"IMU {name}: timestamps not increasing")
        if abs(r["rate_hz"] / odr - 1) > GATES["imu_rate_err"]: warn.append(f"IMU {name}: rate {r['rate_hz']} Hz vs ODR {odr:g}")
        ok = np.isfinite(z["burst_dev"])
        icm = ClockMap().fit(z["burst_dev"][ok], z["burst_host"][ok]); r["clock"] = icm.summary()
        if abs(icm.drift_ppm()) > GATES["imu_drift_ppm"]: reasons.append(f"IMU {name}: drift {icm.drift_ppm():.0f} ppm, check the timestamp tick")
        th = icm.to_host(td)
        cov = (min(th[-1], t_host[-1]) - max(th[0], t_host[0])) / (t_host[-1] - t_host[0]); r["coverage"] = round(float(cov), 4)
        if cov < GATES["imu_coverage"]: reasons.append(f"IMU {name}: covers {cov:.1%} of the run")
        # consistency with the robot: |angular rate| of the IMU against the flange rate from the measured joints
        from ..bench.v1_audit import Flange
        q, qd = _cols(df, "actual_q"), _cols(df, "actual_qd")
        step = max(1, len(q) // 3000)
        wr = np.linalg.norm(Flange().omega(q[::step], qd[::step]), axis=1); trr = t_host[::step]
        wi = np.linalg.norm(g, axis=1)
        best = (-2, 0.0)
        for lag in np.arange(-0.03, 0.0305, 0.0005):
            x = np.interp(trr, th + lag, wi); c = np.corrcoef(x, wr)[0, 1] if np.std(wr) > 1e-6 else np.nan
            if np.isfinite(c) and c > best[0]: best = (c, lag)
        r["robot_corr"], r["lag_ms"] = round(float(best[0]), 4), round(float(best[1]) * 1e3, 2)
        if np.std(wr) > 1e-3:
            if best[0] < GATES["imu_robot_corr"]: reasons.append(f"IMU {name}: |rate| correlates only {best[0]:.3f} with the robot")
            if abs(best[1]) * 1e3 > GATES["imu_lag_ms"]: warn.append(f"IMU {name}: residual lag {best[1] * 1e3:.1f} ms after the clock map")
        out["imu"][name] = r
    out["verdict"] = "FAIL" if reasons else ("WARN" if warn else "PASS")
    out["reasons"] = reasons + warn
    out["gates"] = GATES
    if write: json.dump(out, open(os.path.join(run_dir, "qc.json"), "w"), indent=1)
    return out

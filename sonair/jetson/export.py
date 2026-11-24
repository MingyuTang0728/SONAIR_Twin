"""Session -> the two run formats the project scores.

to_platform_v1  the existing cell platform's run file (schema "sonair-run-1", JSON Lines, 125 Hz), so a Jetson run
                drops into bench_runs/ and goes through the same publish, task and evaluator as the 270 runs already
                recorded. 125 Hz rows = every 4th controller sample, which is exactly what RTDE at 125 Hz delivers,
                so old and new runs are on the same grid. Row time "t" is the controller time mapped to the host clock.
                The IMU is interpolated at the row times from its OWN timestamps (mapped to the host clock) instead of
                the latest-sample hold the old recorder used, and the full-rate IMU stream is kept next to the run file
                (<run_id>.imu.npz) for scorers that use it.
to_twin_run     sonair-twin's sonair.run/0.1 at 500 Hz (runlog), for the MuJoCo replay and the dynamics metrics.
"""
import json, os
import numpy as np
import pandas as pd
from .clock import ClockMap
from .qc import realdata_rtde, _cols
from .. import runlog


def _maps(run_dir):
    ck = np.load(os.path.join(run_dir, "rtde_clock.npz"))
    rmap = ClockMap().fit(ck["ctrl"], ck["host"])
    imus = {}
    for f in sorted(os.listdir(run_dir)):
        if f.startswith("imu_") and f.endswith(".npz"):
            z = np.load(os.path.join(run_dir, f)); ok = np.isfinite(z["burst_dev"])
            im = ClockMap().fit(z["burst_dev"][ok], z["burst_host"][ok])
            imus[f[4:-4]] = dict(t=im.to_host(z["t_dev"]), gyro=z["gyro"], accel=z["accel"], map=im.summary())
    return rmap, imus


def to_platform_v1(run_dir, out_dir, imu_name=None, lag_s=0.0, R_flange_imu=None, carrier_mass_kg=0.0, carrier_com_m=(0, 0, 0)):
    man = json.load(open(os.path.join(run_dir, "manifest.json")))
    df = realdata_rtde(os.path.join(run_dir, "rtde.csv"))
    rmap, imus = _maps(run_dir)
    rows = df.iloc[::4].reset_index(drop=True)                      # 500 Hz -> 125 Hz on the controller's own grid
    t_ctrl = rows["timestamp"].to_numpy(); t_host = rmap.to_host(t_ctrl)
    imu = imus.get(imu_name) if imu_name else (next(iter(imus.values())) if imus else None)
    c = man.get("cell") or {}
    run_id = man["run_id"]
    head = {"_manifest": dict(run_id=run_id, side="real", calib_version=(man.get("calib") or {}).get("imu") or "none",
                              joint_vel=c.get("joint_vel") or 0.0, arm_config=c.get("arm_config") or "unspecified",
                              traj_type=c.get("traj_type") or "unspecified", repeat_idx=int(c.get("repeat_idx") or 0),
                              carrier_id=str(man.get("payload") or "unknown"), carrier_mass_kg=carrier_mass_kg, carrier_com_m=list(carrier_com_m),
                              sample_rate_hz=125.0, started_utc=man.get("created_utc", ""), operator=man.get("operator") or "",
                              notes=f"Jetson session {run_id}; IMU from its own timestamps (clock map), lag {lag_s * 1e3:.1f} ms applied",
                              schema="sonair-run-1", experiment=man.get("experiment") or "E2", robot="ur5e", site=man.get("site") or "UoN",
                              unit=man.get("unit") or "")}
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, run_id + ".jsonl")
    tq, tqd, q, qd = _cols(rows, "target_q"), _cols(rows, "target_qd"), _cols(rows, "actual_q"), _cols(rows, "actual_qd")
    tm, cur, tcp = _cols(rows, "target_moment"), _cols(rows, "actual_current"), rows[[f"actual_TCP_pose_{i}" for i in range(6)]].to_numpy()
    if imu is not None:
        ti = imu["t"] - lag_s
        g = np.column_stack([np.interp(t_host, ti, imu["gyro"][:, k], left=np.nan, right=np.nan) for k in range(3)])
        acc = np.column_stack([np.interp(t_host, ti, imu["accel"][:, k], left=np.nan, right=np.nan) for k in range(3)])
    r6 = lambda v: [round(float(x), 6) for x in v]
    with open(path, "w") as fh:
        fh.write(json.dumps(head) + "\n")
        for k in range(len(rows)):
            s = {"t": round(float(t_host[k]), 6), "q": r6(q[k]), "qd": r6(qd[k]), "tcp_pos": r6(tcp[k, :3]), "tcp_rot": r6(tcp[k, 3:]),
                 "target_q": r6(tq[k]), "target_qd": r6(tqd[k]), "target_moment": [round(float(x), 4) for x in tm[k]],
                 "speed_scaling": float(rows["speed_scaling"].iloc[k]) if "speed_scaling" in rows else None, "robot_age_s": 0.0,
                 "aux": {"controller_t": round(float(t_ctrl[k]), 6)},
                 "sensors": {"ur_tcp": {"actual_q": r6(q[k]), "actual_qd": r6(qd[k]), "actual_TCP_pose": r6(tcp[k]), "actual_current": r6(cur[k])}}}
            if imu is not None and np.isfinite(g[k]).all():
                s["imu"] = {"ind0": {"gyro": r6(g[k]), "accel": r6(acc[k]), "t": round(float(t_host[k]), 6), "rate_hz": None}}
            fh.write(json.dumps(s, separators=(",", ":")) + "\n")
    if imu is not None:
        np.savez_compressed(os.path.join(out_dir, run_id + ".imu.npz"), t_host=imu["t"], gyro=imu["gyro"], accel=imu["accel"],
                            lag_s=lag_s, clock=json.dumps(imu["map"]))
    return path


def to_twin_run(run_dir, runs_root):
    man = json.load(open(os.path.join(run_dir, "manifest.json")))
    raw = runlog.from_rtde_csv(os.path.join(run_dir, "rtde.csv"))
    meta = dict(run_id=man["run_id"], source="real", variant="UR5e", command_id=(man.get("motion") or {}).get("command_id") or "passive",
                payload=man.get("payload") or "unknown", site=man.get("site"), session=os.path.basename(run_dir))
    return runlog.write(os.path.join(runs_root, man["run_id"]), meta, raw)

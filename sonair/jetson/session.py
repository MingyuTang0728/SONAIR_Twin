"""One recorded run on the Jetson: every stream in its own file, one manifest, then QC and export.

    runs_jetson/<run_id>/
      manifest.json        who, where, what was commanded, payload, calibration versions, streams, software versions
      rtde.csv             ur_rtde's own 500 Hz recording (controller timestamp + VARS)
      rtde_clock.npz       (controller timestamp, host monotonic time) pairs, ~100 Hz, for the clock map
      imu_<name>.npz       t_dev (sensor timestamp, s), accel (m/s^2), gyro (rad/s), burst_dev / burst_host (read times)
      qc.json              written by sonair.jetson.qc

Real cell:   python -m sonair jetson-record --robot 192.168.1.10 --imu tool:1:0x6b --cmd <command id> --speed 0.5 \
                 --cell 0.50/mid_workspace/contour --rep 0 --unity 192.168.1.50:5005
             (motion through sonair.real.executor; or --passive to record a program started on the pendant)
No hardware: python -m sonair jetson-record --mock --cmd <command id>   (simulated robot and IMU, same files)
"""
import json, os, platform, socket, threading, time
import numpy as np
import pandas as pd
from .. import cell, commands as cm, kin
from . import lsm6dsv16x as imu_dev

RTDE_HZ = 500.0
VARS = ["timestamp", "target_q", "target_qd", "target_qdd", "target_moment", "target_current", "actual_q", "actual_qd",
        "actual_current", "joint_control_output", "actual_TCP_pose", "target_TCP_pose", "joint_temperatures", "speed_scaling",
        "robot_mode", "safety_mode", "runtime_state"]


def _versions():
    v = dict(python=platform.python_version(), machine=platform.machine(), host=socket.gethostname())
    for mod in ("numpy", "mujoco", "rtde_receive", "smbus2", "pyrealsense2"):
        try: v[mod] = getattr(__import__(mod), "__version__", "present")
        except Exception: v[mod] = None
    return v


def manifest(run_id, **kw):
    return dict(schema="sonair.session/0.1", run_id=run_id, created_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                versions=_versions(), **kw)


# ------------------------------------------------------------------ Unity forwarding (display only)
class UnityLink:
    def __init__(self, target):
        host, _, port = target.rpartition(":")
        self.addr = (host or "127.0.0.1", int(port)); self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, q, tq, closure=None, status=None):
        cs = "NaN" if closure is None else f"{closure:.4f}"
        try:
            self.s.sendto((",".join(f"{v:.6f}" for v in q) + "," + cs + "," + ",".join(f"{v:.6f}" for v in tq)).encode(), self.addr)
            if status: self.s.sendto(("#" + status).encode(), self.addr)
        except OSError:
            pass


# ------------------------------------------------------------------ real hardware
class ImuThread(threading.Thread):
    def __init__(self, name, bus, addr, odr):
        super().__init__(daemon=True, name=f"imu-{name}")
        self.dev = imu_dev.LSM6DSV16X(bus, addr, odr)
        self.name_, self.chunks, self.reads, self.stop_ = name, [], [], threading.Event()

    def run(self):
        while not self.stop_.is_set():
            raw, th = self.dev.read()
            if raw:
                self.chunks.append(raw); self.reads.append((len(self.chunks) - 1, th))
            time.sleep(0.005)

    def save(self, run_dir):
        return save_imu(run_dir, self.name_, self.chunks, [r[1] for r in self.reads], self.dev.acc_fs_g, self.dev.gyr_fs_dps)


def save_imu(run_dir, name, chunks, read_host, acc_fs_g=8, gyr_fs_dps=2000):
    t_all, a_all, g_all, b_dev = [], [], [], []
    for raw in chunks:
        t, a, g, _ = imu_dev.decode_fifo(raw, acc_fs_g, gyr_fs_dps)
        t_all.append(t); a_all.append(a); g_all.append(g); b_dev.append(t[-1] if len(t) else np.nan)
    t = np.concatenate(t_all) if t_all else np.zeros(0)
    # the device counter may wrap between chunks: unwrap on the concatenated stream
    period = 2 ** 32 * imu_dev.TS_LSB_NOMINAL_S
    jumps = np.r_[0, np.cumsum(np.diff(t) < -0.5 * period)] if len(t) else np.zeros(0)
    t = t + jumps * period
    np.savez_compressed(os.path.join(run_dir, f"imu_{name}.npz"), t_dev=t, accel=np.concatenate(a_all) if a_all else np.zeros((0, 3)),
                        gyro=np.concatenate(g_all) if g_all else np.zeros((0, 3)), burst_dev=np.array(b_dev), burst_host=np.array(read_host))
    return len(t)


def record_real(a, run_id, run_dir):
    """Hardware path. Motion: sonair.real.executor (servoJ of a checked command) or --passive (pendant program)."""
    import rtde_receive
    rr = rtde_receive.RTDEReceiveInterface(a.robot, RTDE_HZ, VARS)
    imus = []
    for spec in a.imu or []:
        name, bus, addr = spec.split(":"); imus.append(ImuThread(name, int(bus), int(addr, 16), a.imu_odr))
    link = UnityLink(a.unity) if a.unity else None
    csv = os.path.join(run_dir, "rtde.csv")
    for th in imus: th.start()
    rr.startFileRecording(csv, VARS)
    pairs, stop = [], threading.Event()

    def poll():                                        # clock pairs at ~100 Hz, Unity at ~50 Hz
        k = 0
        while not stop.is_set():
            pairs.append((rr.getTimestamp(), time.monotonic()))
            if link and k % 2 == 0: link.send(rr.getActualQ(), rr.getTargetQ(), status=f"recording {run_id}" if k % 100 == 0 else None)
            k += 1; time.sleep(0.01)
    pt = threading.Thread(target=poll, daemon=True); pt.start()
    t0 = time.time(); aborted = ""
    try:
        if a.passive:
            print("recording (passive): start the program on the pendant; Ctrl+C to stop")
            while a.duration <= 0 or time.time() - t0 < a.duration: time.sleep(0.2)
        else:
            import subprocess, sys
            cmdline = [sys.executable, "-m", "sonair.real.executor", "--robot", a.robot, "--cmd", a.cmd, "--speed", str(a.speed),
                       "--max-qd", str(a.max_qd), "--run-id", run_id + "_exec"] + (["--yes"] if a.yes else [])
            subprocess.run(cmdline, check=False)
    except KeyboardInterrupt:
        aborted = "Ctrl+C"
    finally:
        stop.set(); time.sleep(0.05); rr.stopFileRecording()
        for th in imus: th.stop_.set()
        for th in imus: th.join(1.0)
    np.savez_compressed(os.path.join(run_dir, "rtde_clock.npz"), ctrl=np.array([p[0] for p in pairs]), host=np.array([p[1] for p in pairs]))
    n_imu = {th.name_: th.save(run_dir) for th in imus}
    return dict(aborted=aborted or None, imu_samples=n_imu)


# ------------------------------------------------------------------ mock hardware (same files, no devices)
def record_mock(a, run_id, run_dir, seed=0):
    """A simulated session: controller at 500 Hz following the command with a 4 ms delay and first-order lag, an IMU
    on the tool whose FIFO stamps run on their own drifting clock, bursty reads, and host receive jitter."""
    rng = np.random.default_rng(seed)
    c = cm.load(a.cmd)
    if abs(a.speed - 1.0) > 1e-9: c = cm.time_scale(c, a.speed, cid=a.cmd)
    df = c["df"]; Q = df[cm.QCOLS].to_numpy(); QD = df[cm.QDCOLS].to_numpy(); n = len(Q)
    ctrl0 = 3456.789; tc = ctrl0 + np.arange(n) * 0.002
    q = np.empty_like(Q); q[0] = Q[0]; tau_lag = 0.008
    for k in range(1, n):
        src = Q[max(0, k - 2)]; q[k] = q[k - 1] + (src - q[k - 1]) * (0.002 / tau_lag)
    qd = np.gradient(q, 0.002, axis=0)
    cur = 0.09 * (np.sin(q[:, 1]) * 20 + np.sin(q[:, 1] + q[:, 2]) * 8)[:, None] * np.ones((1, 6)) + 0.7 * np.tanh(qd / 0.01) + rng.normal(0, 0.03, (n, 6))
    T = [kin.fk(x) for x in q]
    cols = {"timestamp": tc}
    for i in range(6):
        cols[f"target_q_{i}"] = Q[:, i]; cols[f"target_qd_{i}"] = QD[:, i]; cols[f"actual_q_{i}"] = q[:, i]; cols[f"actual_qd_{i}"] = qd[:, i]
        cols[f"actual_current_{i}"] = cur[:, i]; cols[f"target_moment_{i}"] = cur[:, i] / 0.09
    for i in range(3): cols[f"actual_TCP_pose_{i}"] = [Tk[i, 3] for Tk in T]
    for i, v in enumerate(np.array([cell.rotvec(Tk[:3, :3]) for Tk in T]).T): cols[f"actual_TCP_pose_{i + 3}"] = v
    cols["speed_scaling"] = np.ones(n); cols["robot_mode"] = np.full(n, 7); cols["safety_mode"] = np.full(n, 1)
    pd.DataFrame(cols).to_csv(os.path.join(run_dir, "rtde.csv"), index=False)
    host0 = 1000.0                                                    # host monotonic at the first controller sample
    ts = np.arange(0, n, 5)                                           # clock pairs at 100 Hz with exponential receive delay
    np.savez_compressed(os.path.join(run_dir, "rtde_clock.npz"), ctrl=tc[ts],
                        host=host0 + (tc[ts] - ctrl0) * (1 + 15e-6) + 0.0003 + rng.exponential(0.0004, len(ts)))
    # tool IMU: angular rate of the flange from the actual motion (rotated into the IMU axes), gravity + motion in accel
    from ..bench.v1_audit import Flange
    fl = Flange(); w_fl = fl.omega(q, qd)
    R_fi = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1.0]])           # IMU mounted -90 deg about the flange z axis
    t_true = host0 + (tc - ctrl0) * (1 + 15e-6)
    w_imu = w_fl @ R_fi
    g_imu = np.array([(Tk[:3, :3] @ R_fi).T @ np.array([0, 0, 9.80665]) for Tk in T])
    f_w = lambda th: np.array([np.interp(th, t_true, w_imu[:, k]) for k in range(3)])
    f_a = lambda th: np.array([np.interp(th, t_true, g_imu[:, k]) for k in range(3)])
    dev = imu_dev.MockIMU(f_w, f_a, odr=a.imu_odr, seed=seed + 1, group_delay_s=0.0015)
    chunks, reads, th = [], [], t_true[0]
    while th < t_true[-1]:
        th += float(rng.choice([0.004, 0.006, 0.010, 0.030], p=[0.6, 0.3, 0.09, 0.01]))
        raw, t_read = dev.read_at(th)
        if raw: chunks.append(raw); reads.append(t_read)
    nimu = save_imu(run_dir, "tool", chunks, reads)
    truth = dict(R_flange_imu=R_fi.tolist(), ctrl_to_host_drift_ppm=15.0, imu_drift_ppm=40.0, imu_odr=a.imu_odr, imu_group_delay_ms=1.5)
    json.dump(truth, open(os.path.join(run_dir, "mock_truth.json"), "w"), indent=1)
    return dict(aborted=None, imu_samples={"tool": nimu}, mock=True)


def record(a):
    run_id = a.run_id or f"{'mock' if a.mock else a.site}_{a.cmd or 'passive'}_x{a.speed:g}_r{a.rep}_{time.strftime('%Y%m%d_%H%M%S')}"
    run_dir = os.path.join(a.out, run_id); os.makedirs(run_dir, exist_ok=True)
    cv = (a.cell or "").split("/")
    man = manifest(run_id, site=a.site, unit=a.unit, operator=a.operator, experiment=a.experiment,
                   cell=dict(joint_vel=float(cv[0]) if len(cv) == 3 else None, arm_config=cv[1] if len(cv) == 3 else None,
                             traj_type=cv[2] if len(cv) == 3 else None, repeat_idx=a.rep),
                   motion=dict(source="passive pendant program" if a.passive else "sonair.real.executor servoJ" if not a.mock else "mock",
                               command_id=a.cmd, speed_scale=a.speed),
                   payload=a.payload, calib=dict(imu=a.imu_cal, handeye=a.handeye_cal), rtde_hz=RTDE_HZ,
                   imus=a.imu or (["tool:mock"] if a.mock else []), imu_odr=a.imu_odr, notes=a.notes, mock=bool(a.mock))
    json.dump(man, open(os.path.join(run_dir, "manifest.json"), "w"), indent=1)
    res = record_mock(a, run_id, run_dir) if a.mock else record_real(a, run_id, run_dir)
    man.update(result=res); json.dump(man, open(os.path.join(run_dir, "manifest.json"), "w"), indent=1)
    from .qc import audit
    q = audit(run_dir, imu_cal=a.imu_cal)
    print(f"{run_id}: QC {q['verdict']}" + ("".join(f"\n  - {r}" for r in q["reasons"]) if q["reasons"] else ""))
    return run_dir

"""Sessions recorded from the console, in the Jetson session format (sonair.session/0.1).

The robot side does NOT open a second robot connection: it subscribes to the packets of the console's own robot
link (ur_bridge_ext.UR, unchanged), which at 500 Hz is every controller cycle. Each row keeps the controller's own
timestamp; the host receive time of every packet (stamped in the reader process at the socket) gives the clock pairs.

    runs_jetson/<run_id>/
      manifest.json      what, where, how it moved, payload, calibration, console version, mock flag
      rtde.csv           one row per controller packet: timestamp, target/actual joints, currents, TCP, modes
      rtde_clock.npz     (controller timestamp, host monotonic) per packet, for sonair.jetson.clock.ClockMap
      imu_<name>.npz     sensor timestamps + samples + read times (sonair.jetson.session.save_imu)
      qc.json            sonair.jetson.qc.audit plus the console's own link checks

The files are the ones `python -m sonair jetson-record` writes, so QC and export are the same code.
The host clock is time.perf_counter() in the robot reader and time.monotonic() in the LSM6DSV16X driver: on Linux
(the Jetson) both read CLOCK_MONOTONIC, which is what makes them comparable.
"""
from __future__ import annotations

import json
import os
import threading
import time
from collections import deque

import numpy as np
import pandas as pd

from sonair import cell
from sonair.jetson import lsm6dsv16x as imu_dev
from sonair.jetson.session import manifest as session_manifest, save_imu

VEC6 = ["target_q", "target_qd", "target_qdd", "target_moment", "target_current", "actual_q", "actual_qd",
        "actual_current", "joint_control_output", "actual_TCP_pose", "target_TCP_pose", "joint_temperatures"]
SCALARS = ["speed_scaling", "robot_mode", "safety_mode", "runtime_state"]
CONSOLE_VERSION = "sonair-console/1.0"
R_FLANGE_IMU_MOCK = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1.0]])      # the mock IMU: -90 deg about flange z


# ---------------------------------------------------------------------------------------------- robot history
class RobotHistory:
    """The last few seconds of measured joints on the host clock. Feeds the mock IMU and the live checks."""

    def __init__(self, seconds=20.0, hz=500.0):
        self.n = int(seconds * hz)
        self.buf = deque(maxlen=self.n)
        self.lock = threading.Lock()
        self.off = None                         # lower envelope of (host receive - controller time), creeping up slowly

    def push(self, st):
        q, qd, m, ts = st.get("actual_q"), st.get("actual_qd"), st.get("_mono"), st.get("timestamp")
        if q is None or qd is None or m is None:
            return
        if ts is not None:                      # time of the sample on the host clock, without the transport jitter
            o = float(m) - float(ts)
            self.off = o if self.off is None or o < self.off else self.off + 2e-6
            m = float(ts) + self.off
        with self.lock:
            self.buf.append((float(m), tuple(q), tuple(qd)))

    def arrays(self, last=None):
        with self.lock:
            b = list(self.buf)[-last:] if last else list(self.buf)
        if not b:
            return np.zeros(0), np.zeros((0, 6)), np.zeros((0, 6))
        t = np.array([x[0] for x in b]); return t, np.array([x[1] for x in b]), np.array([x[2] for x in b])

    def latest_t(self):
        with self.lock:
            return self.buf[-1][0] if self.buf else None


# ---------------------------------------------------------------------------------------------- IMU sources
class ImuSource(threading.Thread):
    """One inertial unit read in its own thread. Keeps the latest decoded sample for the display, and while a
    session is capturing, every FIFO burst with its host read time."""

    def __init__(self, name, odr):
        super().__init__(daemon=True, name=f"imu-{name}")
        self.name_, self.odr = name, odr
        self.stop_ = threading.Event()
        self.capture = None                     # list of (raw, t_read) while a session records
        self.cap_lock = threading.Lock()
        self.latest = {}
        self.samples = 0
        self.t_start = None
        self.error = ""
        self.acc_fs_g, self.gyr_fs_dps = 8, 2000
        self.recent = deque(maxlen=600)         # (t_dev, |gyro|) for the live chart

    def _read(self):                            # -> (raw bytes, host read time)
        raise NotImplementedError

    def run(self):
        self.t_start = time.perf_counter()
        while not self.stop_.is_set():
            try:
                raw, th = self._read()
            except Exception as e:              # a sensor that stops answering is reported, not fatal
                self.error = str(e); time.sleep(0.2); continue
            if raw:
                with self.cap_lock:
                    if self.capture is not None:
                        self.capture.append((raw, th))
                try:
                    t, a, g, _ = imu_dev.decode_fifo(raw, self.acc_fs_g, self.gyr_fs_dps)
                    if len(t):
                        self.samples += len(t)
                        self.latest = {"t_dev": float(t[-1]), "accel": a[-1].round(4).tolist(), "gyro": g[-1].round(5).tolist(),
                                       "t_host": th}
                        for k in range(0, len(t), max(1, len(t) // 4)):
                            self.recent.append((float(t[k]), g[k].round(5).tolist(), a[k].round(4).tolist()))
                except Exception as e:
                    self.error = f"decode: {e}"
            time.sleep(0.005)

    def status(self):
        el = (time.perf_counter() - self.t_start) if self.t_start else 0
        return {"name": self.name_, "odr": self.odr, "kind": self.kind, "alive": self.is_alive(), "error": self.error,
                "samples": self.samples, "rate_hz": round(self.samples / el, 1) if el > 1 else None, "latest": self.latest}

    def begin_capture(self):
        with self.cap_lock:
            self.capture = []

    def end_capture(self):
        with self.cap_lock:
            cap, self.capture = self.capture or [], None
        return cap


class LsmImu(ImuSource):
    kind = "lsm6dsv16x"

    def __init__(self, name, bus, addr, odr):
        super().__init__(name, odr)
        self.dev = imu_dev.LSM6DSV16X(bus, addr, odr)
        self.acc_fs_g, self.gyr_fs_dps = self.dev.acc_fs_g, self.dev.gyr_fs_dps

    def _read(self):
        return self.dev.read()


class MockImu(ImuSource):
    """The sonair.jetson MockIMU (own drifting clock, FIFO bytes, bursty reads) driven by the measured joints the
    console receives, so a mock session has an IMU that agrees with the robot the way a real one should."""
    kind = "mock"

    def __init__(self, name, odr, history: RobotHistory):
        super().__init__(name, odr)
        self.hist = history
        from sonair.bench.v1_audit import Flange
        self.fl = Flange()
        self._cache = (None, None)
        self.dev = imu_dev.MockIMU(self._omega, self._accel, odr=odr, seed=7, group_delay_s=0.0015)

    def _robot_at(self, t):
        T, Q, QD = self._cache_arrays()
        if len(T) < 2:
            return np.array(cell_home()), np.zeros(6)
        return (np.array([np.interp(t, T, Q[:, k]) for k in range(6)]), np.array([np.interp(t, T, QD[:, k]) for k in range(6)]))

    def _cache_arrays(self):
        now = time.perf_counter()
        if self._cache[0] is None or now - self._cache[0] > 0.02:
            T, Q, QD = self.hist.arrays(last=1000)          # the last 2 s is all the IMU ever asks about
            self._cache = (now, (T, Q, QD))
        return self._cache[1]

    def _omega(self, t):
        q, qd = self._robot_at(t)
        return (self.fl.omega(q[None], qd[None])[0]) @ R_FLANGE_IMU_MOCK

    def _accel(self, t):
        from sonair import kin
        q, _ = self._robot_at(t)
        R = kin.fk(q)[:3, :3] @ R_FLANGE_IMU_MOCK
        return R.T @ np.array([0, 0, 9.80665])

    def _read(self):
        lt = self.hist.latest_t()
        if lt is None:
            time.sleep(0.05); return b"", time.perf_counter()
        return self.dev.read_at(lt - 0.01)      # a little behind the newest robot packet, so the motion is known


def cell_home():
    return [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]


class ImuHub:
    def __init__(self, history):
        self.hist = history
        self.units: dict[str, ImuSource] = {}

    def start(self, units, odr=480, mock=False):
        self.stop()
        errs = []
        for spec in units:
            try:
                if mock or spec.get("kind") == "mock":
                    src = MockImu(spec.get("name", "tool"), odr, self.hist)
                else:
                    src = LsmImu(spec["name"], int(spec.get("bus", 1)), int(str(spec.get("addr", "0x6b")), 16), odr)
                src.start(); self.units[src.name_] = src
            except Exception as e:
                errs.append(f"{spec.get('name')}: {e}")
        return {"ok": not errs and bool(self.units), "errors": errs, "units": list(self.units)}

    def stop(self):
        for u in self.units.values():
            u.stop_.set()
        for u in self.units.values():
            u.join(1.0)
        self.units = {}

    def status(self):
        return {n: u.status() for n, u in self.units.items()}


# ---------------------------------------------------------------------------------------------- the session
class Session:
    def __init__(self, root, run_id, meta, ur_service, imu_hub: ImuHub, mock=False):
        self.root, self.run_id, self.meta, self.UR, self.imus, self.mock = root, run_id, meta, ur_service, imu_hub, mock
        self.dir = os.path.join(root, run_id)
        self.rows = []
        self.sources = {}
        self.lock = threading.Lock()
        self.t0 = None
        self.state = "idle"
        self.qc = None
        self.exports = {}
        self.last_ts = None

    # the subscriber: as cheap as possible, it runs on the robot reader's drain thread
    def _sink(self, st):
        ts = st.get("timestamp")
        if ts is None or ts == self.last_ts:
            return
        self.last_ts = ts
        src = st.get("_source", "?")
        self.sources[src] = self.sources.get(src, 0) + 1
        row = [ts]
        for k in VEC6:
            v = st.get(k)
            row.extend(v if v is not None and len(v) == 6 else [np.nan] * 6)
        for k in SCALARS:
            v = st.get(k); row.append(np.nan if v is None else v)
        row.append(st.get("_mono", np.nan))
        with self.lock:
            self.rows.append(row)

    def start(self):
        os.makedirs(self.dir, exist_ok=True)
        self.t0 = time.time()
        for u in self.imus.units.values():
            u.begin_capture()
        self.UR.subscribe(self._sink)
        self.state = "recording"
        json.dump(self._manifest(), open(os.path.join(self.dir, "manifest.json"), "w"), indent=1)
        return {"ok": True, "run_id": self.run_id, "dir": self.dir}

    def n_rows(self):
        with self.lock:
            return len(self.rows)

    def _manifest(self, **extra):
        m = self.meta
        cv = (m.get("cell") or "").split("/")
        cell_d = dict(joint_vel=float(cv[0]) if len(cv) == 3 and cv[0] else None, arm_config=cv[1] if len(cv) == 3 else None,
                      traj_type=cv[2] if len(cv) == 3 else None, repeat_idx=int(m.get("rep") or 0))
        return session_manifest(self.run_id, site=m.get("site", "UoN"), unit=m.get("unit", "UR5e-UoN-1"), operator=m.get("operator", ""),
                                experiment=m.get("experiment", "E2"), cell=cell_d,
                                motion=dict(source=m.get("motion_source", "none"), command_id=m.get("cmd"), speed_scale=m.get("speed")),
                                payload=m.get("payload"), calib=dict(imu=m.get("imu_cal"), handeye=m.get("handeye_cal")), rtde_hz=500.0,
                                imus=[f"{n}:{u.kind}" for n, u in self.imus.units.items()],
                                imu_odr=next(iter(self.imus.units.values())).odr if self.imus.units else None,
                                notes=m.get("notes", ""), mock=bool(self.mock), recorder=CONSOLE_VERSION,
                                robot_link="ur_bridge_ext.UR telemetry subscription (unchanged platform link)", **extra)

    def stop(self):
        if self.state != "recording":
            return {"ok": False, "error": "not recording"}
        self.UR.unsubscribe(self._sink)
        self.state = "writing"
        caps = {n: u.end_capture() for n, u in self.imus.units.items()}
        with self.lock:
            rows, self.rows = self.rows, []
        cols = ["timestamp"] + [f"{k}_{i}" for k in VEC6 for i in range(6)] + SCALARS + ["host_mono"]
        df = pd.DataFrame(rows, columns=cols)
        if len(df) < 50:
            self.state = "failed"
            res = {"ok": False, "error": f"only {len(df)} robot packets: is the robot link up?"}
            json.dump(self._manifest(result=res), open(os.path.join(self.dir, "manifest.json"), "w"), indent=1)
            return res
        df.drop(columns=["host_mono"]).to_csv(os.path.join(self.dir, "rtde.csv"), index=False)
        np.savez_compressed(os.path.join(self.dir, "rtde_clock.npz"), ctrl=df["timestamp"].to_numpy(float), host=df["host_mono"].to_numpy(float))
        n_imu = {}
        for n, cap in caps.items():
            u = self.imus.units.get(n)
            n_imu[n] = save_imu(self.dir, n, [c[0] for c in cap], [c[1] for c in cap], u.acc_fs_g, u.gyr_fs_dps)
        res = dict(aborted=self.meta.get("aborted"), imu_samples=n_imu, robot_packets=len(df), packet_sources=dict(self.sources),
                   wall_duration_s=round(time.time() - self.t0, 2), mock=bool(self.mock))
        if self.mock:
            res["mock_truth"] = dict(R_flange_imu=R_FLANGE_IMU_MOCK.tolist(), imu_group_delay_ms=1.5)
        json.dump(self._manifest(result=res), open(os.path.join(self.dir, "manifest.json"), "w"), indent=1)
        self.qc = run_qc(self.dir)
        self.state = "done"
        return {"ok": True, "run_id": self.run_id, "qc": self.qc, "result": res}

    def status(self):
        return {"run_id": self.run_id, "state": self.state, "rows": self.n_rows() if self.state == "recording" else None,
                "elapsed_s": round(time.time() - self.t0, 1) if self.t0 else 0, "sources": dict(self.sources),
                "qc": (self.qc or {}).get("verdict"), "reasons": (self.qc or {}).get("reasons"), "mock": self.mock,
                "meta": {k: self.meta.get(k) for k in ("cell", "rep", "cmd", "speed", "motion_source", "payload")}}


def run_qc(run_dir):
    """sonair.jetson.qc.audit, then the console's link checks on top (RTDE only, 500 Hz, mock never benchmark-grade)."""
    from sonair.jetson.qc import audit
    q = audit(run_dir, write=False)
    man = json.load(open(os.path.join(run_dir, "manifest.json")))
    res = man.get("result") or {}
    fails, warns = [], []
    srcs = res.get("packet_sources") or {}
    other = sum(v for k, v in srcs.items() if k != "rtde")
    if other:
        fails.append(f"{other} robot packets came from the 30003 fallback, not RTDE: not benchmark-grade (RTDE only)")
    med = (q.get("rtde") or {}).get("median_period_ms")
    if med and abs(med - 2.0) > 0.05:
        fails.append(f"robot link at {1000 / med:.0f} Hz, the session format needs 500 Hz: restart the robot link at 500 Hz")
    if man.get("mock"):
        warns.append("mock session (simulated controller and IMU): never part of a dataset")
    q["console_checks"] = dict(packet_sources=srcs, fails=fails, warns=warns)
    q["reasons"] = fails + q.get("reasons", []) + warns
    if fails:
        q["verdict"] = "FAIL"
    elif warns and q["verdict"] == "PASS":
        q["verdict"] = "WARN"
    json.dump(q, open(os.path.join(run_dir, "qc.json"), "w"), indent=1)
    return q


def export(run_dir, platform_out, twin_out, lag_s=None):
    """Session -> sonair-run-1 (125 Hz, the platform's run file) and sonair.run/0.1 (500 Hz). The IMU lag applied is
    the one QC measured unless given."""
    from sonair.jetson.export import to_platform_v1, to_twin_run
    qc = json.load(open(os.path.join(run_dir, "qc.json"))) if os.path.exists(os.path.join(run_dir, "qc.json")) else {}
    man = json.load(open(os.path.join(run_dir, "manifest.json")))
    if lag_s is None:
        lags = [v.get("lag_ms") for v in (qc.get("imu") or {}).values() if v.get("lag_ms") is not None]
        lag_s = (lags[0] / 1e3) if lags else 0.0
    R = ((man.get("result") or {}).get("mock_truth") or {}).get("R_flange_imu")
    carrier = man.get("carrier") or {}
    p1 = to_platform_v1(run_dir, platform_out, lag_s=lag_s, R_flange_imu=R, carrier_mass_kg=carrier.get("mass_kg", 0.0),
                        carrier_com_m=carrier.get("com_m", (0, 0, 0)))
    p2 = to_twin_run(run_dir, twin_out)
    return {"platform_run": p1, "twin_run": p2 if isinstance(p2, str) else str(p2), "lag_ms": round(lag_s * 1e3, 2)}

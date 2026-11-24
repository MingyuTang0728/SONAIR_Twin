"""Execute one SONAIR command on the real UR5e (or URSim) with servoJ at 500 Hz and record RTDE at 500 Hz.

    python -m sonair.real.executor --robot 192.168.1.10 --cmd exc_gripper_open_v0.3_s1 --speed 0.5 --rep 1
    python -m sonair.real.executor --mock --cmd exc_gripper_open_v0.3_s1          (no robot: tests the whole chain)

What it does
  1. loads the command (50 Hz knots -> 500 Hz), applies --speed (time scaling, <= 1.0 unless --allow-full-speed)
  2. refuses to run if the scaled peak joint speed exceeds --max-qd (default 0.6 rad/s), if the command failed the
     collision check, or if the robot is not in normal safety mode; asks you to type "yes"
  3. moveJ slowly to the first sample, starts RTDE recording, streams servoJ every 2 ms, holds the last sample
  4. stops servo mode, stops recording and writes runs/<run_id>/ in the SONAIR run format (source = real)
Ctrl+C at any time: servoStop + stopScript, the partial run is kept and marked aborted.

The simulators later replay the robot's recorded target_q (python -m sonair sim --target-run runs/<run_id>),
so servoJ filtering and PC timing jitter never show up as a sim-to-real gap.
"""
import argparse, json, math, os, sys, tempfile, time
import numpy as np
from .. import cell, commands as cm, runlog

VARS = ["timestamp", "target_q", "target_qd", "target_qdd", "target_current", "target_moment", "actual_q", "actual_qd",
        "actual_current", "joint_control_output", "actual_TCP_pose", "target_TCP_pose", "joint_temperatures", "speed_scaling",
        "robot_mode", "safety_mode", "actual_execution_time"]


# ------------------------------------------------------------------ mock robot (same method names as ur_rtde)
class _MockRobot:
    """first-order joint servo with a 4 ms delay; writes a ur_rtde-style CSV so the whole chain can be tested"""
    def __init__(self):
        self.q = None; self.qd = np.zeros(6); self.t = 0.0; self.target = None; self.buf = []; self.rec = None; self.hist = []
    def getSafetyMode(self): return 1
    def getRobotMode(self): return 7
    def isProtectiveStopped(self): return False
    def isEmergencyStopped(self): return False
    def getActualQ(self): return list(self.q)
    def getTimestamp(self): return self.t
    def moveJ(self, q, speed=0.3, acc=0.5): self.q = np.array(q, float); self.target = self.q.copy(); return True
    def startFileRecording(self, path, variables=None): self.rec = path; self.buf = []; return True
    def stopFileRecording(self):
        cols = ["timestamp"] + [f"target_q_{i}" for i in range(6)] + [f"target_qd_{i}" for i in range(6)] + [f"actual_q_{i}" for i in range(6)] \
               + [f"actual_qd_{i}" for i in range(6)] + [f"actual_TCP_pose_{i}" for i in range(6)] + ["speed_scaling"]
        with open(self.rec, "w") as f:
            f.write(",".join(cols) + "\n")
            for r in self.buf: f.write(",".join(f"{v:.9g}" for v in r) + "\n")
        return True
    def initPeriod(self): return time.perf_counter()
    def waitPeriod(self, t0): pass
    def servoJ(self, q, v, a, dt, look, gain):
        q = np.array(q, float); tqd = (q - self.target) / dt; self.target = q; self.hist.append(q)
        delayed = self.hist[-3] if len(self.hist) >= 3 else self.hist[0]                 # 4 ms transport delay
        qd_new = (delayed - self.q) / 0.008; self.q = self.q + qd_new * dt; self.qd = qd_new; self.t += dt
        if self.rec:
            from .. import kin
            T = kin.fk(self.q); self.buf.append([self.t, *q, *tqd, *self.q, *self.qd, *T[:3, 3], *cell.rotvec(T[:3, :3]), 1.0])
        return True
    def servoStop(self, a=10.0): return True
    def stopScript(self): pass
    def isProgramRunning(self): return True
    def disconnect(self): pass


def _connect(ip, mock):
    if mock:
        r = _MockRobot(); return r, r
    try:
        import rtde_control, rtde_receive
    except ImportError:
        sys.exit("ur_rtde is not installed: pip install ur_rtde")
    rr = rtde_receive.RTDEReceiveInterface(ip, 500.0)
    rc = rtde_control.RTDEControlInterface(ip, 500.0)
    return rc, rr


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", help="UR controller (or URSim) IP")
    ap.add_argument("--mock", action="store_true", help="no robot: simulated servo, tests the chain end to end")
    ap.add_argument("--cmd", required=True, help="command id under commands/ (run `python -m sonair commands` first)")
    ap.add_argument("--speed", type=float, default=0.5, help="time scaling, 0.5 = half speed (default)")
    ap.add_argument("--allow-full-speed", action="store_true")
    ap.add_argument("--max-qd", type=float, default=0.6, help="refuse if the scaled peak joint speed exceeds this (rad/s)")
    ap.add_argument("--lookahead", type=float, default=0.1); ap.add_argument("--gain", type=float, default=300)
    ap.add_argument("--rep", type=int, default=1); ap.add_argument("--run-id"); ap.add_argument("--site", default="UoN")
    ap.add_argument("--robot-id", default="UR5e-UoN-1", help="your label for this robot (serial number ideally)")
    ap.add_argument("--payload-set-on-pendant", default="", help="what you entered on the pendant, e.g. 'camtool 1.265 kg (0,-4.5,63.4) mm'")
    ap.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    a = ap.parse_args()
    if not a.mock and not a.robot: sys.exit("--robot IP or --mock is required")
    if a.speed > 1.0 and not a.allow_full_speed: sys.exit("--speed > 1.0 needs --allow-full-speed")

    c = cm.load(a.cmd)
    if abs(a.speed - 1.0) > 1e-9: c = cm.time_scale(c, a.speed, cid=a.cmd)
    df = c["df"]; Q = df[cm.QCOLS].to_numpy(); dt = 1.0 / cm.RATE
    peak = float(np.abs(df[cm.QDCOLS].to_numpy()).max())
    col = c["meta"].get("collision_check", "")
    problems = []
    if peak > a.max_qd + 0.005: problems.append(f"peak joint speed {peak:.2f} rad/s > --max-qd {a.max_qd}")
    if isinstance(col, dict): problems.append(f"command failed the collision check: {col}")
    lim = np.array([2 * math.pi] * 6); lim[2] = math.pi
    if (np.abs(Q) > lim).any(): problems.append("joint target outside the UR5e range")
    print(f"command {a.cmd}: {len(Q)} samples, {df['t'].iloc[-1]:.1f} s at speed x{a.speed}, peak {peak:.2f} rad/s, payload {c['meta'].get('payload') or c['meta'].get('payload_schedule')}")
    if problems: sys.exit("REFUSED: " + "; ".join(problems))

    rc, rr = _connect(a.robot, a.mock)
    if rr.getSafetyMode() != 1 or rr.getRobotMode() != 7:
        sys.exit(f"robot not ready: safety mode {rr.getSafetyMode()} (want 1 normal), robot mode {rr.getRobotMode()} (want 7 running)")
    if not a.yes and not a.mock:
        print("Check: pendant payload set, speed slider at 100 % (the scaling is done here), area clear, hand on the E-stop.")
        if input('type "yes" to move the robot: ').strip() != "yes": sys.exit("cancelled")

    run_id = a.run_id or f"real_{a.cmd}_x{a.speed:g}_r{a.rep}_{time.strftime('%Y%m%d_%H%M%S')}"
    csv = os.path.join(tempfile.gettempdir(), run_id + "_rtde.csv")
    aborted = ""
    try:
        rc.moveJ(list(Q[0]), 0.3, 0.5)                                  # slow approach to the first sample
        q_now = np.array(rr.getActualQ())
        if np.abs(q_now - Q[0]).max() > 0.01: raise RuntimeError(f"not at the start pose (error {np.abs(q_now - Q[0]).max():.3f} rad)")
        rr.startFileRecording(csv, VARS) if not a.mock else rr.startFileRecording(csv)
        time.sleep(0.2 if not a.mock else 0)
        t_start_rtde = rr.getTimestamp()
        for k in range(len(Q) + int(0.5 / dt)):                         # + 0.5 s holding the last sample
            t0 = rc.initPeriod()
            rc.servoJ(list(Q[min(k, len(Q) - 1)]), 0.0, 0.0, dt, a.lookahead, a.gain)
            if k % 50 == 0 and (rr.isProtectiveStopped() or rr.isEmergencyStopped() or rr.getSafetyMode() != 1):
                raise RuntimeError("protective or emergency stop")
            rc.waitPeriod(t0)
    except KeyboardInterrupt:
        aborted = "Ctrl+C"
    except Exception as e:
        aborted = str(e)
    finally:
        try: rc.servoStop(); rc.stopScript()
        except Exception: pass
        time.sleep(0.3 if not a.mock else 0)
        try: rr.stopFileRecording()
        except Exception: pass
    if not os.path.exists(csv): sys.exit("no recording written" + (f" ({aborted})" if aborted else ""))

    raw = runlog.from_rtde_csv(csv)                     # t starts at the first recorded sample
    import pandas as pd
    t_abs = pd.read_csv(csv, usecols=[0]).iloc[:, 0].to_numpy()
    raw["t"] = t_abs - t_start_rtde                     # 0 = first servoJ sample
    raw = raw[raw["t"] >= -1e-9].reset_index(drop=True)
    raw["grip"] = np.interp(raw["t"], df["t"], df["grip"])
    meta = dict(run_id=run_id, source="real", variant="UR5e_servoJ", command_id=a.cmd, command_sha256=c["meta"].get("sha256"),
                speed_scale=a.speed, payload=c["meta"].get("payload", "schedule"), payload_schedule=c["meta"].get("payload_schedule"),
                site=a.site, robot=dict(id=a.robot_id, ip=a.robot or "mock", mock=a.mock), servoj=dict(lookahead=a.lookahead, gain=a.gain),
                payload_set_on_pendant=a.payload_set_on_pendant, rep=a.rep, aborted=aborted or None)
    d = runlog.write(os.path.join(cell.ROOT, "runs", run_id), meta, raw)
    print(("ABORTED: " + aborted + "\n" if aborted else "") + f"wrote {d}  ({len(raw)} samples)  problems: {runlog.validate(d) or 'none'}")


if __name__ == "__main__":
    main()

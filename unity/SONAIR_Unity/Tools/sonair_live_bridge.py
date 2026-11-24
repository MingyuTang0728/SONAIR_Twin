#!/usr/bin/env python3
"""SONAIR live bridge: stream the UR5e state to the Unity twin over UDP (read only, nothing is commanded).

Packet sent to Unity (ASCII, one per datagram):
    q1..q6,closure,t1..t6
      q        actual joints, rad (UR order: base, shoulder, elbow, wrist 1, wrist 2, wrist 3)
      closure  gripper 0 = open (85 mm) .. 1 = closed, or NaN if not read
      t        commanded joints (RTDE target_q), rad: Unity draws them as a translucent ghost arm
    #text      a status line shown in the Unity panel (sent once per second)

On the cell the Jetson recorder already forwards the same packet (python -m sonair jetson-record ... --unity HOST:PORT);
use this script only when the recorder is not running.

Real robot:
    pip install ur_rtde
    python sonair_live_bridge.py --robot 192.168.1.10 --unity 192.168.1.50:5005 [--gripper]
Without a robot (replays a pre-computed trajectory in real time, looping; the ghost lags it by --demo-lag seconds):
    python sonair_live_bridge.py --demo ../SONAIR_Trajectories_CSV/B_S2.csv --unity 127.0.0.1:5005
"""
import argparse, csv, math, socket, sys, time

NAN = float("nan")


def parse_target(s):
    host, _, port = s.rpartition(":")
    return (host or "127.0.0.1", int(port))


def packet(q, c, tq):
    cs = "NaN" if c is None or (isinstance(c, float) and math.isnan(c)) else f"{c:.4f}"
    return ",".join(f"{v:.6f}" for v in q) + "," + cs + "," + ",".join(f"{v:.6f}" for v in tq)


def send_loop(sock, target, source, hz, label):
    period = 1.0 / hz
    n = 0
    t_next = time.perf_counter(); t_status = 0.0; t0 = time.perf_counter()
    try:
        for q, c, tq in source:
            sock.sendto(packet(q, c, tq).encode("ascii"), target)
            n += 1
            now = time.perf_counter()
            if now - t_status > 1.0:
                t_status = now
                sock.sendto(f"#{label} | {n / max(1e-6, now - t0):.0f} Hz | {n} packets".encode("ascii"), target)
                print(f"\rsent {n:7d}  q = [{', '.join(f'{v:+.3f}' for v in q)}]   ", end="", flush=True)
            t_next += period
            dt = t_next - time.perf_counter()
            if dt > 0: time.sleep(dt)
            else: t_next = time.perf_counter()
    except KeyboardInterrupt:
        print("\nstopped")


def demo_source(path, hz, speed, lag):
    rows = []
    with open(path, encoding="utf-8-sig") as f:
        for r in csv.DictReader(f):
            q = [float(r[k]) for k in ("q1_base", "q2_shoulder", "q3_elbow", "q4_wrist1", "q5_wrist2", "q6_wrist3")]
            rows.append((float(r["t_s"]), q, float(r["gripper_closure_0to1"])))
    if not rows: sys.exit("empty CSV")
    T = rows[-1][0]

    def at(t):
        t = min(max(t, 0.0), T); i = 0
        lo, hi = 0, len(rows) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if rows[mid][0] <= t: lo = mid
            else: hi = mid
        t0, q0, c0 = rows[lo]; t1, q1, c1 = rows[hi]
        f = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
        return [a + (b - a) * f for a, b in zip(q0, q1)], c0 + (c1 - c0) * f

    t = 0.0
    while True:
        tq, c = at(t)                 # the command
        q, _ = at(t - lag)            # the "actual" arm, lagging the command (an illustration only)
        yield q, c, tq
        t += speed / hz
        if t > T: t = 0.0


class Gripper:
    """Reads the 2F gripper position through its URCap socket (robot port 63352)."""
    def __init__(self, ip):
        self.s = socket.create_connection((ip, 63352), timeout=1.0)
        self.last = None

    def closure(self):
        try:
            self.s.sendall(b"GET POS\n")
            data = self.s.recv(64).decode("ascii", "ignore").split()
            if len(data) >= 2 and data[0] == "POS": self.last = max(0.0, min(1.0, int(data[1]) / 255.0))
        except (socket.timeout, ValueError, OSError):
            pass
        return self.last


def robot_source(ip, use_gripper):
    try:
        import rtde_receive
    except ImportError:
        sys.exit("ur_rtde is not installed: pip install ur_rtde")
    rr = rtde_receive.RTDEReceiveInterface(ip)
    grip = Gripper(ip) if use_gripper else None
    k = 0
    while True:
        q, tq = rr.getActualQ(), rr.getTargetQ()
        c = None
        if grip is not None: c = grip.closure() if k % 6 == 0 else grip.last
        k += 1
        yield list(q), c, list(tq)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--unity", default="127.0.0.1:5005", help="Unity host:port (default 127.0.0.1:5005)")
    ap.add_argument("--hz", type=float, default=60.0)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--robot", help="UR controller IP (RTDE, read only)")
    g.add_argument("--demo", help="trajectory CSV from SONAIR_Trajectories_CSV")
    ap.add_argument("--gripper", action="store_true", help="also read the gripper position (URCap socket 63352)")
    ap.add_argument("--speed", type=float, default=1.0, help="demo playback speed")
    ap.add_argument("--demo-lag", type=float, default=0.15, help="demo only: how far the shown arm lags the command (s)")
    a = ap.parse_args()
    target = parse_target(a.unity)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"sending to {target[0]}:{target[1]} at {a.hz:.0f} Hz, Ctrl+C to stop")
    src = demo_source(a.demo, a.hz, a.speed, a.demo_lag) if a.demo else robot_source(a.robot, a.gripper)
    send_loop(sock, target, src, a.hz, "demo " + a.demo.split("/")[-1] if a.demo else "robot " + a.robot)


if __name__ == "__main__":
    main()

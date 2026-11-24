"""Read-only RTDE recorder (500 Hz). Use it to log anything you run from the pendant, e.g. your existing arc scans.
Nothing is sent to the robot.

    python -m sonair.real.rtde_logger --robot 192.168.1.10 --run-id arcscan_2026-11-25_r1 --payload camtool
    (Ctrl+C to stop; or --duration 60)
"""
import argparse, os, sys, tempfile, time
from .. import cell, runlog
from .executor import VARS


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", required=True); ap.add_argument("--run-id", required=True)
    ap.add_argument("--duration", type=float, default=0, help="seconds (0 = until Ctrl+C)")
    ap.add_argument("--cmd", default="pendant_program", help="what was executed (free text if not a SONAIR command)")
    ap.add_argument("--payload", default="unknown"); ap.add_argument("--site", default="UoN"); ap.add_argument("--notes", default="")
    a = ap.parse_args()
    try:
        import rtde_receive
    except ImportError:
        sys.exit("ur_rtde is not installed: pip install ur_rtde")
    rr = rtde_receive.RTDEReceiveInterface(a.robot, 500.0, VARS)
    csv = os.path.join(tempfile.gettempdir(), a.run_id + "_rtde.csv")
    rr.startFileRecording(csv, VARS)
    print("recording", csv, "... Ctrl+C to stop")
    t0 = time.time()
    try:
        while a.duration <= 0 or time.time() - t0 < a.duration: time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    rr.stopFileRecording(); rr.disconnect()
    df = runlog.from_rtde_csv(csv)
    meta = dict(run_id=a.run_id, source="real", variant="UR5e_pendant", command_id=a.cmd, payload=a.payload, site=a.site, notes=a.notes)
    d = runlog.write(os.path.join(cell.ROOT, "runs", a.run_id), meta, df)
    print("wrote", d, "problems:", runlog.validate(d) or "none")


if __name__ == "__main__":
    main()

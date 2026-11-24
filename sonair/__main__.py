"""SONAIR twin command line.   python -m sonair <command> ...

  payload                       lumped end-effector payloads -> config/payloads.json (also the pendant payload values)
  commands                      build the M1 command set (6 tasks x 2 speeds + 22 excitation runs), collision-checked
  sim   --cmd ID|all --variant B0_kinematic|B1_menagerie_pd|B2_pd_invdyn|all [--payload KEY] [--target-run RUN]
  track RUN                     tracking metrics of one run
  gap   RUN_A RUN_B [--repeat RUN_R1 RUN_R2]   gap between two runs of the same command (+ normalised gap)
  validate RUN [RUN ...]        check run folders against the run format
  fk    q1 q2 q3 q4 q5 q6       tool0 pose in B (mm, rotation vector) for a pendant cross-check
  rtde2run CSV --cmd ID --run-id ID [--payload KEY]   convert a ur_rtde recording into a run
  report-m1                     figures + reports/m1_summary.json from the simulated runs
"""
import argparse, glob, json, os, sys
import numpy as np
from . import cell, commands as cm, runlog, payload as pl

RUNS = os.path.join(cell.ROOT, "runs")
EXC_PLAN = [("gripper_open", None, "open", (0.3, 0.6, 0.9, 1.2), "centre"),
            ("camtool", "camtool", "closed", (0.3, 0.6, 0.9, 1.2), "high"),
            ("s2", "s2", "closed", (0.3, 0.6, 0.9), "high")]


def cmd_payload(a):
    out = pl.write()
    print(f"{'config':14s}{'mass kg':>9s}   CoG in tool0 (mm)            label")
    for k, v in out.items():
        print(f"{k:14s}{v['mass_kg']:9.3f}   ({', '.join(f'{x * 1000:6.1f}' for x in v['cog_m'])})   {v['label']}")


def _check(cmd, held, grip, step=25):
    from . import collision
    ck = collision.Checker(held=held, gripper=grip, on_riser=None if held in ("s2", "s4", "paddle") else "s2")
    Q = cmd["df"][cm.QCOLS].to_numpy(); worst = {}
    for i in range(0, len(Q), step):
        for k, d in ck.check(Q[i]).items(): worst[f"{k[0]}->{k[1]}"] = max(worst.get(f"{k[0]}->{k[1]}", 0), d)
    return worst


def cmd_commands(a):
    from . import kin
    index = []
    sched = json.load(open(os.path.join(cell.ASSETS, "unity_trajectories", "payload_schedules.json")))
    for p in sorted(glob.glob(os.path.join(cell.ASSETS, "unity_trajectories", "*.csv"))):
        key = os.path.splitext(os.path.basename(p))[0]
        base = cm.from_unity_csv(p, payload_schedule=sched.get(key))
        for s in (1.0, 0.5):
            c = base if s == 1.0 else cm.time_scale(base, s)
            c["meta"]["collision_check"] = "checked in the Unity exporter (task trajectories)"
            cm.save(c); index.append(dict(id=c["meta"]["id"], kind="task", duration_s=c["meta"]["duration_s"], peak_qd=max(c["meta"]["peak_qd"])))
            print(f"{c['meta']['id']:22s} {c['meta']['duration_s']:6.1f} s  peak {max(c['meta']['peak_qd']):.2f} rad/s")
    T = kin.fk(cm.Q_CENTER); T[2, 3] = 0.62
    q_high, _ = kin.ik(T, cm.Q_CENTER)
    for pay, held, grip, levels, centre in EXC_PLAN:
        for lv in levels:
            for seed in (1, 2):
                c = cm.excitation(lv, seed, center=cm.Q_CENTER if centre == "centre" else q_high, cid=f"exc_{pay}_v{lv:.1f}_s{seed}", payload=pay)
                c["df"]["grip"] = 0.0 if grip == "open" else 0.681
                hits = _check(c, held, grip)
                c["meta"]["split"] = "train" if seed == 1 else "test"
                c["meta"]["collision_check"] = "pass" if not hits else hits
                cm.save(c)
                index.append(dict(id=c["meta"]["id"], kind="excitation", payload=pay, level_qd=lv, split=c["meta"]["split"],
                                  duration_s=c["meta"]["duration_s"], collision=c["meta"]["collision_check"]))
                print(f"{c['meta']['id']:28s} {c['meta']['duration_s']:5.1f} s  peak {np.round(c['meta']['peak_qd'], 2).tolist()}  collision: {c['meta']['collision_check']}")
    json.dump(index, open(os.path.join(cell.ROOT, "commands", "index.json"), "w"), ensure_ascii=False, indent=1)


def _cmd_ids(sel):
    if sel != "all": return [sel]
    return [os.path.basename(os.path.dirname(p)) for p in sorted(glob.glob(os.path.join(cell.ROOT, "commands", "*", "command.json")))]


def cmd_sim(a):
    from .sim import mujoco_twin as tw
    variants = tw.VARIANTS if a.variant == "all" else [a.variant]
    for cid in _cmd_ids(a.cmd):
        c = cm.load(cid)
        target = c
        if a.target_run:                                  # replay the robot's recorded target instead of the command
            target, _ = runlog.target_from_run(a.target_run); target = dict(meta=c["meta"], df=target)
        for v in variants:
            meta, df = tw.replay(target, a.payload, v)
            rid = f"{cid}__{v}" + ("__realtarget" if a.target_run else "")
            meta.update(run_id=rid, command_id=cid, command_sha256=c["meta"].get("sha256"), speed_scale=c["meta"].get("speed_scale", 1.0))
            df = df.astype({k: "float32" for k in df.columns if k not in ("t", "payload")})
            runlog.write(os.path.join(RUNS, rid), meta, df)
            print("wrote", rid)


def cmd_track(a):
    from .bench import metrics as M
    _, df = runlog.read(a.run); print(json.dumps(M.tracking(df), ensure_ascii=False, indent=1))


def cmd_gap(a):
    from .bench import metrics as M
    _, A = runlog.read(a.run_a); _, B = runlog.read(a.run_b); g = M.gap(A, B)
    out = dict(gap=g)
    if a.repeat:
        _, R1 = runlog.read(a.repeat[0]); _, R2 = runlog.read(a.repeat[1]); r = M.gap(R1, R2)
        out.update(repeatability=r, normalized=M.normalized_gap(g, r))
    print(json.dumps(out, ensure_ascii=False, indent=1))


def cmd_validate(a):
    bad = 0
    for r in a.runs:
        p = runlog.validate(r); bad += bool(p); print(("OK   " if not p else "FAIL ") + r + ("" if not p else "  " + "; ".join(p)))
    sys.exit(1 if bad else 0)


def cmd_fk(a):
    from . import kin
    T = kin.fk(np.array(a.q, float)); rv = cell.rotvec(T[:3, :3])
    print(f"tool0 in B:  X {T[0, 3] * 1000:.1f}  Y {T[1, 3] * 1000:.1f}  Z {T[2, 3] * 1000:.1f} mm   RX {rv[0]:.4f}  RY {rv[1]:.4f}  RZ {rv[2]:.4f} rad")


def cmd_rtde2run(a):
    df = runlog.from_rtde_csv(a.csv)
    c = cm.load(a.cmd)
    meta = dict(run_id=a.run_id, source="real", variant="UR5e", command_id=a.cmd, command_sha256=c["meta"].get("sha256"),
                payload=a.payload or c["meta"].get("payload", "schedule"), site=a.site, notes=a.notes)
    runlog.write(os.path.join(RUNS, a.run_id), meta, df)
    print("wrote", a.run_id, "problems:", runlog.validate(os.path.join(RUNS, a.run_id)) or "none")


def cmd_audit_v1(a):
    from .bench import v1_audit
    print(json.dumps(v1_audit.run(a.runs, a.cal, a.out, cache=a.cache), indent=1))


def cmd_jetson_record(a):
    if not a.mock and not a.robot: sys.exit("--robot IP, or --mock")
    if not a.passive and not a.cmd: sys.exit("--cmd <command id>, or --passive to record a pendant program")
    from .jetson import session
    session.record(a)


def cmd_session_export(a):
    from .jetson import export
    p = export.to_platform_v1(a.run_dir, a.platform_out, a.imu, a.lag_ms / 1000.0, carrier_mass_kg=a.carrier_mass_kg)
    d = export.to_twin_run(a.run_dir, os.path.join(cell.ROOT, "runs"))
    print("platform run:", p); print("twin run:", d, runlog.validate(d) or "valid")


def cmd_imu_probe(a):
    import time as _t
    import numpy as np
    from .jetson import lsm6dsv16x as L
    from .jetson.clock import ClockMap
    dev = L.LSM6DSV16X(a.bus, int(a.addr, 16), a.odr)
    chunks, reads, t0 = [], [], _t.monotonic()
    while _t.monotonic() - t0 < a.seconds:
        raw, th = dev.read()
        if raw: chunks.append(raw); reads.append(th)
        _t.sleep(0.005)
    ts, gs, bd = [], [], []
    for raw in chunks:
        t, acc, g, _ = L.decode_fifo(raw); ts.append(t); gs.append(g); bd.append(t[-1] if len(t) else np.nan)
    t, g = np.concatenate(ts), np.concatenate(gs); ok = np.isfinite(bd)
    cmap = ClockMap().fit(np.array(bd)[ok], np.array(reads)[ok])
    print(json.dumps(dict(samples=len(t), rate_hz=round(1 / np.median(np.diff(t)), 2), gyro_std_dps=np.degrees(g.std(axis=0)).round(4).tolist(),
                          clock=cmap.summary(), note="keep the sensor still; drift is the tick error + crystal (expect tens of ppm)"), indent=1))


def main():
    ap = argparse.ArgumentParser(prog="python -m sonair", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd_name", required=True)
    sp.add_parser("payload").set_defaults(f=cmd_payload)
    sp.add_parser("commands").set_defaults(f=cmd_commands)
    s = sp.add_parser("sim"); s.add_argument("--cmd", default="all"); s.add_argument("--variant", default="all"); s.add_argument("--payload")
    s.add_argument("--target-run"); s.set_defaults(f=cmd_sim)
    s = sp.add_parser("track"); s.add_argument("run"); s.set_defaults(f=cmd_track)
    s = sp.add_parser("gap"); s.add_argument("run_a"); s.add_argument("run_b"); s.add_argument("--repeat", nargs=2); s.set_defaults(f=cmd_gap)
    s = sp.add_parser("validate"); s.add_argument("runs", nargs="+"); s.set_defaults(f=cmd_validate)
    s = sp.add_parser("fk"); s.add_argument("q", nargs=6, type=float); s.set_defaults(f=cmd_fk)
    s = sp.add_parser("rtde2run"); s.add_argument("csv"); s.add_argument("--cmd", required=True); s.add_argument("--run-id", required=True)
    s.add_argument("--payload"); s.add_argument("--site", default="UoN"); s.add_argument("--notes", default=""); s.set_defaults(f=cmd_rtde2run)
    s = sp.add_parser("report-m1"); s.set_defaults(f=lambda a: __import__("sonair.bench.report_m1", fromlist=["main"]).main())
    # ---- real data audit (benchmark-v1 runs recorded by the cell platform)
    s = sp.add_parser("audit-v1", help="audit real sonair-run-1 runs: quality, floors, simulators, metric consistency")
    s.add_argument("--runs", required=True); s.add_argument("--cal", required=True); s.add_argument("--out", default=os.path.join(cell.ROOT, "reports", "v1_real"))
    s.add_argument("--cache", default=None); s.set_defaults(f=cmd_audit_v1)
    # ---- Jetson acquisition
    s = sp.add_parser("jetson-record", help="record one run on the Jetson (RTDE 500 Hz + IMU FIFO + clock pairs); --mock without hardware")
    s.add_argument("--robot"); s.add_argument("--mock", action="store_true"); s.add_argument("--passive", action="store_true")
    s.add_argument("--cmd"); s.add_argument("--speed", type=float, default=0.5); s.add_argument("--max-qd", type=float, default=0.6)
    s.add_argument("--imu", action="append", help="name:bus:addr, e.g. tool:1:0x6b (repeat for more IMUs)"); s.add_argument("--imu-odr", type=int, default=480)
    s.add_argument("--cell", default="", help="joint_vel/arm_config/traj_type, e.g. 0.50/mid_workspace/contour"); s.add_argument("--rep", type=int, default=0)
    s.add_argument("--experiment", default="E2"); s.add_argument("--payload", default="gripper_open"); s.add_argument("--site", default="UoN")
    s.add_argument("--unit", default="UR5e-UoN-1"); s.add_argument("--operator", default=""); s.add_argument("--imu-cal", default=None)
    s.add_argument("--handeye-cal", default=None); s.add_argument("--unity", default=None, help="HOST:PORT of the Unity twin (display only)")
    s.add_argument("--duration", type=float, default=0); s.add_argument("--notes", default=""); s.add_argument("--run-id")
    s.add_argument("--yes", action="store_true"); s.add_argument("--out", default=os.path.join(cell.ROOT, "runs_jetson")); s.set_defaults(f=cmd_jetson_record)
    s = sp.add_parser("session-qc"); s.add_argument("run_dir"); s.set_defaults(f=lambda a: print(json.dumps(__import__("sonair.jetson.qc", fromlist=["audit"]).audit(a.run_dir), indent=1)))
    s = sp.add_parser("session-export", help="session -> platform run file (sonair-run-1, 125 Hz) and sonair-twin run (500 Hz)")
    s.add_argument("run_dir"); s.add_argument("--platform-out", required=True); s.add_argument("--imu", default=None)
    s.add_argument("--lag-ms", type=float, default=0.0); s.add_argument("--carrier-mass-kg", type=float, default=0.0); s.set_defaults(f=cmd_session_export)
    s = sp.add_parser("jetson-imu-probe", help="bench check of one LSM6DSV16X: WHO_AM_I, rate, noise, clock drift")
    s.add_argument("--bus", type=int, default=1); s.add_argument("--addr", default="0x6b"); s.add_argument("--odr", type=int, default=480)
    s.add_argument("--seconds", type=float, default=5.0); s.set_defaults(f=cmd_imu_probe)
    a = ap.parse_args(); a.f(a)


if __name__ == "__main__":
    main()

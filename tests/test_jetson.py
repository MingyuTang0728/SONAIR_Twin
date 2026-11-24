"""Acquisition chain without hardware: clock map, IMU FIFO codec, mock session QC, export to both run formats."""
import json, os, sys, tempfile, types
import numpy as np
from sonair.jetson.clock import ClockMap
from sonair.jetson import lsm6dsv16x as L


def test_clock_map_recovers_offset_and_drift():
    rng = np.random.default_rng(1)
    d = np.sort(rng.uniform(0, 600, 5000)); h = 50.0 + d * (1 + 37e-6) + 0.0004 + rng.exponential(0.002, d.size)
    m = ClockMap().fit(d, h)
    assert abs(m.drift_ppm() - 37) < 1.0
    assert abs(np.median(m.to_host(d) - (50.0 + d * (1 + 37e-6))) - 0.0004) < 2e-4   # lands on the minimum delay


def test_fifo_codec_roundtrip():
    t = 10.0 + np.arange(200) / 480.0; a = np.tile([0.1, -0.2, 9.8], (200, 1)); g = np.tile([0.01, 0.5, -1.0], (200, 1))
    raw = L.encode_fifo(t, a, g)
    t2, a2, g2, c = L.decode_fifo(raw)
    assert len(t2) == 200 and c["orphan"] == 0
    assert np.abs(t2 - t).max() < L.TS_LSB_NOMINAL_S
    assert np.abs(a2 - a).max() < 0.01 and np.abs(g2 - g).max() < np.radians(0.07)


def _args(out, rid):
    return types.SimpleNamespace(mock=True, robot=None, passive=False, cmd="exc_gripper_open_v0.3_s1", speed=1.0, max_qd=0.6,
                                 imu=None, imu_odr=480, cell="0.30/mid_workspace/contour", rep=0, experiment="E2", payload="gripper_open",
                                 site="UoN", unit="mock", operator="test", imu_cal=None, handeye_cal=None, unity=None, duration=0,
                                 notes="", run_id=rid, yes=True, out=out)


def test_mock_session_qc_and_exports():
    from sonair.jetson import session, export
    from sonair import runlog
    out = tempfile.mkdtemp()
    d = session.record(_args(out, "t_mock"))
    q = json.load(open(os.path.join(d, "qc.json")))
    assert q["verdict"] == "PASS", q["reasons"]
    assert abs(q["rtde_clock"]["drift_ppm"] - 15) < 3 and q["imu"]["tool"]["coverage"] > 0.99
    assert abs(q["imu"]["tool"]["lag_ms"] + 1.5) <= 0.5          # the mock IMU's 1.5 ms filter delay is found
    p = export.to_platform_v1(d, os.path.join(out, "platform"), lag_s=0.0015)
    rows = [json.loads(l) for l in open(p)]
    assert rows[0]["_manifest"]["schema"] == "sonair-run-1"
    t = np.array([r["t"] for r in rows[1:]]); assert abs(1 / np.median(np.diff(t)) - 125) < 0.01
    assert sum("imu" in r for r in rows[1:]) > 0.99 * (len(rows) - 1)
    rd = export.to_twin_run(d, os.path.join(out, "runs"))
    assert runlog.validate(rd) == []

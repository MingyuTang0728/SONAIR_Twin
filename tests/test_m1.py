"""python -m pytest -q tests   (about 20 s)"""
import os, sys, numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from sonair import payload, commands as cm, runlog, kin, cell
from sonair.sim import mujoco_twin as tw
from sonair.bench import metrics as M


def test_fk_matches_ur_nominal():
    p = kin.fk(np.zeros(6))[:3, 3] * 1000
    assert np.allclose(p, [-817.2, -233.0, 62.8], atol=3.0)         # UR5e nominal tool0 at q = 0


def test_payloads_physical():
    P = payload.payloads()
    assert 1.0 < P["gripper_open"]["mass_kg"] < 1.3
    assert P["s2"]["cog_m"][2] > P["camtool"]["cog_m"][2] > P["gripper_open"]["cog_m"][2]
    for v in P.values(): assert np.all(np.linalg.eigvalsh(np.array(v["inertia_kgm2"])) > 0)


def test_command_roundtrip(tmp_path):
    c = cm.excitation(0.6, 7); cm.save(c, root=str(tmp_path)); c2 = cm.load(str(tmp_path / c["meta"]["id"]))
    assert np.abs(c2["df"][cm.QCOLS].to_numpy() - c["df"][cm.QCOLS].to_numpy()).max() < 1e-6
    assert abs(max(c2["meta"]["peak_qd"]) - 0.6) < 0.02


def test_twin_tracks_and_logs(tmp_path):
    c = cm.excitation(0.6, 3)
    meta, df = tw.replay(c, "camtool", "B2_pd_invdyn")
    assert M.tracking(df)["tcp_rms_mm"] < 1.0
    meta.update(run_id="t", command_id="x"); d = runlog.write(str(tmp_path / "t"), meta, df)
    assert runlog.validate(d) == []


def test_gap_alignment():
    c = cm.excitation(0.6, 4); _, a = tw.replay(c, "gripper_open", "B0_kinematic")
    b = a.copy(); b["t"] = b["t"] + 0.010
    assert abs(M.gap(a, b, align=True)["clock_offset_ms"] - 10.0) <= 2.0

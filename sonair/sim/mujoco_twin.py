"""MuJoCo dynamic twin of the UR5e + lumped end-effector payload.

Variants (baselines of the dynamics-gap benchmark):
  B0_kinematic    actual = target (what the Unity package does; zero dynamics)
  B1_menagerie_pd PD exactly as shipped in MuJoCo Menagerie (kp 2000/500, kv 400/100), no gravity compensation
  B2_pd_invdyn    inverse-dynamics feedforward on the target + the same PD on the error (UR-like model-based servo)
Later (M3): B3_identified = B2 with friction, armature, gains and delay fitted to real runs.

The controller updates at 500 Hz (zero-order hold) and the physics runs at 2 kHz.
"""
import math
import numpy as np
import pandas as pd
import mujoco
from .. import cell, payload as pl, commands as cm

KP = np.array([2000.0] * 3 + [500.0] * 3)
KV = np.array([400.0] * 3 + [100.0] * 3)
VARIANTS = ("B0_kinematic", "B1_menagerie_pd", "B2_pd_invdyn")
PHYS_DT = 0.0005


def build(payload_key):
    """payload_key: a key of config/payloads.json, or a payload dict {mass_kg, cog_m, inertia_kgm2}"""
    P = payload_key if isinstance(payload_key, dict) else pl.payloads()[payload_key]
    s = mujoco.MjSpec.from_file(cell.UR5E_XML)
    for a in list(s.actuators): s.delete(a)
    for k in list(s.keys): s.delete(k)
    site = s.site(cell.SITE); q = np.array(site.quat, float); q /= np.linalg.norm(q)
    b = site.parent.add_body(name="ee_payload", pos=site.pos, quat=q)
    b.explicitinertial = True; b.mass = P["mass_kg"]; b.ipos = P["cog_m"]
    I = np.array(P["inertia_kgm2"]); b.fullinertia = [I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]]
    s.option.timestep = PHYS_DT; s.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    # no contacts: the Menagerie collision capsules are coarse and touch each other when the elbow is folded,
    # which a real UR5e does not; environment collisions are checked separately (sonair.collision)
    s.option.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    m = s.compile()
    return m, P


def _target_arrays(target):
    """accepts a command dict or a target DataFrame (columns t, tq*, tqd*[, tqdd*] or t, q*, qd*[, qdd*])"""
    df = target["df"] if isinstance(target, dict) else target
    pre = "tq" if "tq0" in df.columns else "q"; pred = "tqd" if pre == "tq" else "qd"; predd = "tqdd" if pre == "tq" else "qdd"
    t = df["t"].to_numpy(); Q = df[[f"{pre}{i}" for i in range(6)]].to_numpy()
    QD = df[[f"{pred}{i}" for i in range(6)]].to_numpy() if f"{pred}0" in df else np.gradient(Q, t, axis=0)
    QDD = df[[f"{predd}{i}" for i in range(6)]].to_numpy() if f"{predd}0" in df else np.gradient(QD, t, axis=0)
    grip = df["grip"].to_numpy() if "grip" in df else np.zeros(len(t))
    return t, Q, QD, QDD, grip


def _inertial(m):
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "ee_payload")
    return b, (m.body_mass[b].copy(), m.body_ipos[b].copy(), m.body_iquat[b].copy(), m.body_inertia[b].copy())


def replay(target, payload_key=None, variant="B2_pd_invdyn", settle=1.5):
    """payload_key: fixed payload, or None to use the command's payload / payload_schedule ([[t, key], ...])"""
    assert variant in VARIANTS, variant
    t, Q, QD, QDD, grip = _target_arrays(target)
    meta_c = target["meta"] if isinstance(target, dict) else {}
    if isinstance(payload_key, dict):                 # one explicit payload (e.g. a measured carrier), no schedule
        sched = [[0.0, payload_key.get("label", "custom")]]; m, P = build(payload_key); inert = {}
    else:
        sched = [[0.0, payload_key]] if payload_key else meta_c.get("payload_schedule") or [[0.0, meta_c.get("payload", "gripper_open")]]
        m, P = build(sched[0][1])
        inert = {k: _inertial(build(k)[0])[1] for k in {s_[1] for s_ in sched}}
    d = mujoco.MjData(m); d_id = mujoco.MjData(m)
    eb, _ = _inertial(m)
    switch = [(int(round(ts / (t[1] - t[0]))), k) for ts, k in sched[1:]]
    site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, cell.SITE)
    n_sub = int(round((t[1] - t[0]) / PHYS_DT)) if len(t) > 1 else 4
    d.qpos[:] = Q[0]; d.qvel[:] = QD[0]; mujoco.mj_forward(m, d)

    def feedforward(qt, qdt, qddt):
        if variant == "B1_menagerie_pd": return np.zeros(6)
        d_id.qpos[:] = qt; d_id.qvel[:] = qdt; d_id.qacc[:] = qddt; mujoco.mj_inverse(m, d_id)
        return d_id.qfrc_inverse.copy()

    def servo(ff, qt, qdt, n):
        """joint servo at the physics rate (2 kHz), target held for one 2 ms control period; returns mean torque"""
        acc = np.zeros(6)
        for _ in range(n):
            if variant == "B1_menagerie_pd": tau = KP * (qt - d.qpos) - KV * d.qvel
            else: tau = ff + KP * (qt - d.qpos) + KV * (qdt - d.qvel)
            tau = np.clip(tau, -cell.TORQUE_LIMIT, cell.TORQUE_LIMIT); d.qfrc_applied[:] = tau; acc += tau
            mujoco.mj_step(m, d)
        return acc / n

    if variant != "B0_kinematic":                     # robot at rest under its own controller before the command starts
        ff0 = feedforward(Q[0], np.zeros(6), np.zeros(6))
        for _ in range(int(settle / PHYS_DT / n_sub)): servo(ff0, Q[0], np.zeros(6), n_sub)
    rows = np.empty((len(t), 6 * 5 + 6)); pay = np.empty(len(t), object); cur = sched[0][1]
    for k in range(len(t)):
        for ks, key in switch:
            if ks == k:
                cur = key; m.body_mass[eb], m.body_ipos[eb], m.body_iquat[eb], m.body_inertia[eb] = inert[key]
                mujoco.mj_setConst(m, mujoco.MjData(m))      # scratch data: mj_setConst resets qpos of the data it gets
        pay[k] = cur
        if variant == "B0_kinematic":
            d.qpos[:] = Q[k]; d.qvel[:] = QD[k]; mujoco.mj_kinematics(m, d); tau = np.zeros(6)
        else:
            mujoco.mj_kinematics(m, d)
        T = cell.Tm(d.site_xmat[site].reshape(3, 3), d.site_xpos[site])
        rows[k, :12] = np.concatenate([Q[k], QD[k]]); rows[k, 12:24] = np.concatenate([d.qpos, d.qvel])
        rows[k, 30:33] = T[:3, 3]; rows[k, 33:36] = cell.rotvec(T[:3, :3])
        if variant != "B0_kinematic":
            tau = servo(feedforward(Q[k], QD[k], QDD[k]), Q[k], QD[k], n_sub)
        rows[k, 24:30] = tau
    cols = ([f"tq{i}" for i in range(6)] + [f"tqd{i}" for i in range(6)] + [f"q{i}" for i in range(6)] + [f"qd{i}" for i in range(6)]
            + [f"tau{i}" for i in range(6)] + ["tcp_x", "tcp_y", "tcp_z", "tcp_rx", "tcp_ry", "tcp_rz"])
    df = pd.DataFrame(rows, columns=cols); df.insert(0, "t", t - t[0]); df["grip"] = grip; df["payload"] = pay.astype(str)
    meta = dict(source="mujoco" if variant != "B0_kinematic" else "unity", variant=variant, payload=sched[0][1] if len(sched) == 1 else "schedule",
                payload_schedule=sched, command_id=meta_c.get("id"), command_sha256=meta_c.get("sha256"), mujoco_version=mujoco.__version__,
                physics_dt_s=PHYS_DT, control_rate_hz=round(1 / (t[1] - t[0])), kp=KP.tolist(), kv=KV.tolist())
    return meta, df

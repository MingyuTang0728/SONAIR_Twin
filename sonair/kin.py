"""UR5e forward / inverse kinematics on the MuJoCo model (tool0 = attachment_site)."""
import numpy as np
import mujoco
from . import cell

_M = mujoco.MjModel.from_xml_path(cell.UR5E_XML); _D = mujoco.MjData(_M)
_S = mujoco.mj_name2id(_M, mujoco.mjtObj.mjOBJ_SITE, cell.SITE)


def fk(q):
    _D.qpos[:] = q; mujoco.mj_kinematics(_M, _D)
    return cell.Tm(_D.site_xmat[_S].reshape(3, 3).copy(), _D.site_xpos[_S].copy())


def fk_pos(Q):
    """tool0 positions (N,3) for joint samples Q (N,6)"""
    out = np.empty((len(Q), 3))
    for i, q in enumerate(Q):
        _D.qpos[:] = q; mujoco.mj_kinematics(_M, _D); out[i] = _D.site_xpos[_S]
    return out


def ik(T, q0, iters=300):
    q = np.array(q0, float)
    for _ in range(iters):
        _D.qpos[:] = q; mujoco.mj_forward(_M, _D)
        ep = T[:3, 3] - _D.site_xpos[_S]; Re = T[:3, :3] @ _D.site_xmat[_S].reshape(3, 3).T
        er = 0.5 * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
        e = np.concatenate([ep, er])
        if np.linalg.norm(ep) < 1e-6 and np.linalg.norm(er) < 1e-5: break
        jp = np.zeros((3, 6)); jr = np.zeros((3, 6)); mujoco.mj_jacSite(_M, _D, jp, jr, _S)
        J = np.vstack([jp, jr]); q = q + J.T @ np.linalg.solve(J @ J.T + 1e-5 * np.eye(6), e)
    return q, float(np.linalg.norm(ep))

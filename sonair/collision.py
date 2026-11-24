"""Coarse collision check: sampled surface points of the arm, gripper, collar and held object against
axis-aligned boxes of the cell (table, riser, profile column, dock, Jetson tray, objects on the riser).
No self-collision. Good enough to reject bad trajectories before they reach URSim or the robot."""
import os
import numpy as np
import mujoco
from . import cell

_P = np.load(os.path.join(cell.ASSETS, "collision_points.npz"))
_M = mujoco.MjModel.from_xml_path(cell.UR5E_XML); _D = mujoco.MjData(_M)
_SITE = mujoco.mj_name2id(_M, mujoco.mjtObj.mjOBJ_SITE, cell.SITE)
_LINKS = [(k, mujoco.mj_name2id(_M, mujoco.mjtObj.mjOBJ_BODY, k[3:])) for k in _P.files if k.startswith("ur_")]


def site_pose(q):
    _D.qpos[:] = q; mujoco.mj_kinematics(_M, _D)
    return cell.Tm(_D.site_xmat[_SITE].reshape(3, 3).copy(), _D.site_xpos[_SITE].copy())


def _pen(P, lo, hi, m=0.002):
    inside = np.all((P > lo + m) & (P < hi - m), axis=1)
    if not inside.any(): return 0.0
    Q = P[inside]; return float(np.minimum(Q - lo, hi - Q).min(1).max())


class Checker:
    def __init__(self, held=None, gripper="open", on_riser="s2", ignore=()):
        self.held = held
        self.grip = _P["gripper_closed_27mm" if gripper != "open" else "gripper_open"]
        self.boxes = dict(cell.STATIC_BOXES)
        if held == "camtool": self.boxes.pop("camtool_docked")
        if on_riser and on_riser != held: self.boxes["obj_" + on_riser] = cell.world_box(cell.T_W_FIX, *cell.OBJ_BOX[on_riser])
        for k in ignore: self.boxes.pop(k, None)

    def check(self, q):
        """returns {(part, obstacle): penetration_mm} for every intersection"""
        T = site_pose(q); hits = {}
        groups = {k[3:]: cell.xf(_P[k], cell.Tm(_D.xmat[b].reshape(3, 3), _D.xpos[b])) for k, b in _LINKS}
        groups["gripper"] = cell.xf(self.grip, T); groups["collar"] = cell.xf(_P["collar"], T)
        if self.held: groups["held_" + self.held] = cell.xf(_P["obj_" + self.held], T @ cell.GRASP[self.held])
        for g, P in groups.items():
            for name, (lo, hi) in self.boxes.items():
                d = _pen(P, lo, hi)
                if d > 0: hits[(g, name)] = round(d * 1000, 1)
        return hits

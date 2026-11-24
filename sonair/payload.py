"""Lumped end-effector payloads (everything rigidly attached to tool0) for each experiment configuration.

Output per configuration: mass (kg), centre of gravity in tool0 (m) and inertia about the CoG in tool0 axes (kg m^2).
These are the numbers to enter as the payload on the pendant, and the inertial of the 'ee_payload' body in the twins.
"""
import json, os
import numpy as np
from . import cell

CFG = os.path.join(cell.ROOT, "config")


def _box_inertia(m, dx, dy, dz):
    return m / 12.0 * np.diag([dy * dy + dz * dz, dx * dx + dz * dz, dx * dx + dy * dy])


class Part:
    def __init__(self, name, mass, com, inertia, T=np.eye(4)):
        """mass kg, com (3,) m and inertia (3x3, kg m^2, about com) in the part frame; T = tool0 <- part frame"""
        self.name, self.m = name, float(mass)
        self.c = cell.xf(np.asarray(com, float)[None], T)[0]
        R = T[:3, :3]; self.I = R @ np.asarray(inertia, float) @ R.T


def lump(parts):
    M = sum(p.m for p in parts); c = sum(p.m * p.c for p in parts) / M
    I = np.zeros((3, 3))
    for p in parts:
        d = p.c - c; I += p.I + p.m * (d @ d * np.eye(3) - np.outer(d, d))
    return M, c, I


def load_parts():
    masses = json.load(open(os.path.join(CFG, "masses.json")))
    cad = json.load(open(os.path.join(cell.ASSETS, "kit_frames_masses.json")))["mass_properties_CAD"]
    grip = json.load(open(os.path.join(CFG, "gripper_2f85_inertial.json")))
    b = masses["bought"]
    P = {}

    def printed(key, T=np.eye(4)):
        e = masses["printed"][key]; c = cad[e["cad_key"]]; m = e["mass_g"] / 1000.0; k = m / (c["mass_if_solid_g"] / 1000.0)
        I = np.array(c["inertia_at_com_if_solid_g_mm2"]) * 1e-9 * k          # g mm^2 -> kg m^2, scaled with mass
        return Part(key, m, np.array(c["com_mm"]) / 1000.0, I, T)

    for state in ("open", "closed_27mm"):
        g = grip[state]; s = b["gripper_2f85_g"] / 1000.0 / g["mass_kg"]
        P["gripper_" + state] = Part("gripper_" + state, g["mass_kg"] * s, g["com_m"], np.array(g["inertia_kgm2"]) * s)
    imu = b["imu_board_g"] / 1000.0
    P["collar"] = [printed("P05a_collar"), printed("P05b_collar"),
                   Part("D435i", b["D435i_g"] / 1000.0, (0.0, -0.060, 0.0325), _box_inertia(b["D435i_g"] / 1000.0, 0.090, 0.025, 0.025)),
                   Part("IMU_F", imu, (0.0, 0.04615, 0.02975), np.zeros((3, 3))),
                   Part("fasteners", b["fasteners_and_cables_on_tool0_g"] / 1000.0, (0.0, 0.0, 0.030), np.zeros((3, 3)))]
    balls = [Part("ball", b["steel_ball_g"] / 1000.0, np.array(bc) / 1000.0, np.zeros((3, 3))) for bc in cell.FR["P01_camera_tool"]["ball_centres"]]
    P["camtool"] = [printed("P01_camtool"), Part("D405", b["D405_g"] / 1000.0, (0, 0, 0.1915), _box_inertia(b["D405_g"] / 1000.0, 0.042, 0.042, 0.023)),
                    Part("IMU_C1", imu, (0.03785, 0, 0.20133), np.zeros((3, 3))), Part("IMU_C2", imu, (-0.03785, 0, 0.20133), np.zeros((3, 3)))] + balls
    imu_p = lambda: Part("IMU_P", imu, (0.0, -0.121, 0.0132), np.zeros((3, 3)), cell.T_FS)
    P["s2"] = [printed("P06_S2", cell.T_FS), imu_p()]
    P["s4"] = [printed("P07_S4", cell.T_FS), imu_p()]
    P["paddle"] = [printed("P08_paddle", cell.T_FS)]
    return P


CONFIGS = {
    "gripper_open": ("gripper_open", None, "Gripper open, nothing in hand (collar, D435i and IMU-F included)"),
    "camtool":      ("gripper_closed_27mm", "camtool", "Mode B: holding the camera tool (P01 + D405 + 2 IMUs)"),
    "s2":           ("gripper_closed_27mm", "s2", "Mode A: holding specimen S2"),
    "s4":           ("gripper_closed_27mm", "s4", "Mode A: holding specimen S4"),
    "paddle":       ("gripper_closed_27mm", "paddle", "Calibration A: holding the paddle"),
}


def payloads():
    P = load_parts(); out = {}
    for key, (g, held, label) in CONFIGS.items():
        parts = [P[g]] + P["collar"] + (P[held] if held else [])
        M, c, I = lump(parts)
        out[key] = dict(label=label, mass_kg=round(M, 4), cog_m=[round(v, 5) for v in c], inertia_kgm2=np.round(I, 7).tolist())
    return out


def write(path=os.path.join(CFG, "payloads.json")):
    out = payloads()
    json.dump(dict(_notes="tool0 (flange) frame; on the pendant enter mass_kg and cog_m (in mm); printed parts from config/masses.json, re-run `python -m sonair payload` after weighing",
                   **out), open(path, "w"), ensure_ascii=False, indent=1)
    return out

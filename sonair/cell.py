"""SONAIR two-mode inspection cell: frames, layout and small math helpers.

Conventions (shared by every simulator and by the real robot logs):
  * World frame = UR controller base frame B (z up, metres). MuJoCo Menagerie's world matches B
    (q = 0 puts tool0 at (-0.817, -0.234, 0.063)).
  * tool0 = flange frame F = MuJoCo site "attachment_site" (TCP offset 0).
  * Joint order = pendant order: base, shoulder, elbow, wrist 1, wrist 2, wrist 3; radians.
  * Kit parts keep the frames of the print package (mm in the CAD, metres here).
"""
import json, math, os
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ASSETS = os.path.join(ROOT, "assets")
UR5E_XML = os.environ.get("SONAIR_UR5E_XML", os.path.join(ASSETS, "universal_robots_ur5e", "ur5e.xml"))
SITE = "attachment_site"
JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
TORQUE_LIMIT = np.array([150.0, 150.0, 150.0, 28.0, 28.0, 28.0])   # UR5e joint torque limits (N m)
QD_LIMIT = np.full(6, math.pi)                                       # UR5e joint speed limit (rad/s)


# ------------------------------------------------------------------ math
def Tm(R=np.eye(3), t=(0, 0, 0)):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t; return T
def Rx(a): c, s = math.cos(a), math.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
def Ry(a): c, s = math.cos(a), math.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
def Rz(a): c, s = math.cos(a), math.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
def inv(T):
    Ti = np.eye(4); Ti[:3, :3] = T[:3, :3].T; Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]; return Ti
def mm(T):
    T2 = np.array(T, float).copy(); T2[:3, 3] /= 1000.0; return T2
def xf(P, T): return np.asarray(P) @ T[:3, :3].T + T[:3, 3]
def rotvec(R):
    """rotation matrix -> UR rotation vector (rad)"""
    c = np.clip((np.trace(R) - 1) / 2, -1, 1); a = math.acos(c)
    if a < 1e-9: return np.zeros(3)
    if math.pi - a < 1e-6:
        w, V = np.linalg.eigh(R); v = V[:, np.argmax(w)]; return v * a
    return a / (2 * math.sin(a)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


# ------------------------------------------------------------------ cell layout (same numbers as the Unity package)
FR = json.load(open(os.path.join(ASSETS, "kit_frames_raw.json")))["frames"]
R_BASE = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], float)
LENS_DOCK = np.array([0.66, -0.27, 0.31])
R_DOCK = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)
T_W_DOCK = Tm(R_DOCK, LENS_DOCK - R_DOCK @ np.array([0, 0, 0.203]))
T_W_FIX = Tm(R_BASE, (0.40, 0.25, 0.070))
T_FS = mm(FR["P06_specimen_S2_R150"]["T_F_S_ModeA"])        # tool0 <- specimen/paddle when held by the handle
PARK = {"s2": Tm(R_BASE, (0.45, 0.50, 0.0)), "s4": Tm(R_BASE, (0.17, 0.47, 0.0)), "paddle": Tm(np.eye(3), (0.30, -0.45, 0.0))}
PROFILE_XY = (T_W_DOCK @ np.array([0, 0.107, 0, 1.0]))[:2]
T_W_JETSON = Tm(np.eye(3), (-0.25, -0.18, 0.0))
GRASP = {"camtool": np.eye(4), "s2": T_FS, "s4": T_FS, "paddle": T_FS}

def world_box(T, lo, hi):
    C = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]); W = xf(C, T)
    return W.min(0), W.max(0)

STATIC_BOXES = {
    "table": (np.array([-0.375, -0.625, -0.03]), np.array([0.875, 0.625, 0.0])),
    "riser": world_box(T_W_FIX, (-0.119, -0.075, -0.070), (0.115, 0.075, 0.0)),
    "profile_foot": (np.array([PROFILE_XY[0] - 0.05, PROFILE_XY[1] - 0.05, 0.0]), np.array([PROFILE_XY[0] + 0.05, PROFILE_XY[1] + 0.05, 0.042])),
    "profile": (np.array([PROFILE_XY[0] - 0.015, PROFILE_XY[1] - 0.015, 0.0]), np.array([PROFILE_XY[0] + 0.015, PROFILE_XY[1] + 0.015, 0.444])),
    "dock": world_box(T_W_DOCK, (-0.050, -0.050, 0.1676), (0.050, 0.094, 0.2176)),
    "camtool_docked": world_box(T_W_DOCK, (-0.052, -0.053, 0.132), (0.052, 0.046, 0.2063)),
    "jetson": world_box(T_W_JETSON, (-0.090, -0.115, 0.0), (0.090, 0.115, 0.045)),
}
OBJ_BOX = {"s2": ((-0.115, -0.121, 0.0), (0.115, 0.075, 0.036)), "s4": ((-0.115, -0.121, 0.0), (0.115, 0.075, 0.048)),
           "paddle": ((-0.095, -0.121, 0.0), (0.095, 0.045, 0.036))}

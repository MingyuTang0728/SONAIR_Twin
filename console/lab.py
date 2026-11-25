"""The Unity cell and lab, served to the browser's 3D view, and the same rig's kinematics in Python.

The console does not copy the Unity package: it reads the scene data the Unity project builds itself from
(unity/SONAIR_Unity/Assets/SONAIR/Resources/SONAIR): scene.json (UR5e + 2F-85 rig, kit parts, sensors, frames),
lab.json (the room), sequences.json (the six kit sequences) and the binary meshes and textures. The browser builds
the same rig in three.js, so the arm, the gripper and every part are drawn from the same numbers as in Unity.

Unity coordinates are left-handed (x right, y up, z forward). The robot base frame B (z up) is
x_B = z_u, y_B = -x_u, z_B = y_u, i.e. p_B = P p_u. A frame drawn in the robot convention has R_B = P R_u P^T.
"""
from __future__ import annotations

import gzip
import json
import math
import os
import re
import threading

import numpy as np

from sonair import cell

RES = os.path.join(cell.ROOT, "unity", "SONAIR_Unity", "Assets", "SONAIR", "Resources", "SONAIR")
P = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
_NAME = re.compile(r"^[A-Za-z0-9_]{1,80}$")
_cache: dict[str, tuple[bytes, bytes]] = {}
_lock = threading.Lock()


# ---------------------------------------------------------------------------------------------- serving
def asset(rel: str):
    """(raw bytes, gzip bytes, content type) for an allowed path under the Unity Resources folder, else None.

    Allowed: scene.json, lab.json, sequences.json, Meshes/<name>.bytes, Textures/<name>.bytes. Nothing else."""
    rel = rel.strip("/")
    if rel in ("scene.json", "lab.json", "sequences.json"):
        p, ctype = os.path.join(RES, rel), "application/json"
    else:
        m = re.match(r"^(Meshes|Textures)/([^/]+)\.bytes$", rel)
        if not m or not _NAME.match(m.group(2)):
            return None
        p = os.path.join(RES, m.group(1), m.group(2) + ".bytes")
        ctype = "image/png" if m.group(1) == "Textures" else "application/octet-stream"
    if not os.path.isfile(p):
        return None
    with _lock:
        hit = _cache.get(p)
    if hit is None:
        raw = open(p, "rb").read()
        gz = gzip.compress(raw, 6) if ctype != "image/png" else b""
        hit = (raw, gz)
        with _lock:
            _cache[p] = hit
    return hit[0], hit[1], ctype


def available() -> bool:
    return os.path.isfile(os.path.join(RES, "scene.json"))


# ---------------------------------------------------------------------------------------------- the rig in Python
def _qmat(q):
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _axis_angle(axis, a):
    k = np.asarray(axis, float)
    k = k / (np.linalg.norm(k) or 1.0)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


_scene = None


def scene():
    global _scene
    if _scene is None:
        _scene = json.load(open(os.path.join(RES, "scene.json")))
    return _scene


def unity_world(q, sc=None):
    """World matrices (Unity numbers, 4x4) of every rig body for arm joints q, as SonairRig.ApplyJoint builds them."""
    sc = sc or scene()
    W = []
    for i, b in enumerate(sc["bodies"]):
        R = _qmat(b["rot"])
        t = np.asarray(b["pos"], float)
        if 0 <= b["joint"] < 6:
            r = _axis_angle(b["axis"], float(q[b["joint"]]))
            piv = np.asarray(b["pivot"], float)
            t = t + R @ (piv - r @ piv)
            R = R @ r
        L = np.eye(4); L[:3, :3] = R; L[:3, 3] = t
        W.append(L if b["parent"] < 0 else W[b["parent"]] @ L)
    return W


def to_B(W_u):
    """A Unity world matrix as a robot-convention frame in B (R_B = P R P^T, t_B = P t)."""
    T = np.eye(4)
    T[:3, :3] = P @ W_u[:3, :3] @ P.T
    T[:3, 3] = P @ W_u[:3, 3]
    return T


def tool0_B(q):
    sc = scene()
    return to_B(unity_world(q, sc)[sc["tool0"]])


def optical_B(W_u_sensor):
    """Optical frame (x right, y down, z forward) in B of a scene sensor. The sensor transforms in scene.json ARE the
    optical frames, written in the robot convention like every frame of the scene (x_r = +z_u, y_r = -x_u,
    z_r = +y_u; SonairSensors.cs puts the Unity camera under it looking along +y_u), so this is to_B."""
    return to_B(W_u_sensor)


def sensor(name):
    for s in scene()["sensors"]:
        if s["name"] == name:
            return s
    raise KeyError(name)


def _local(pos, rot):
    L = np.eye(4); L[:3, :3] = _qmat(rot); L[:3, 3] = pos
    return L


def tool0_to_optical(name="G_D435i"):
    """Constant flange -> optical transform of a camera carried on tool0 (the D435i on the collar)."""
    s = sensor(name)
    if s["parent"] != "tool0":
        raise ValueError(f"{name} is not on tool0")
    q = [0.3, -1.2, 1.4, -1.7, -1.5, 0.4]
    W = unity_world(q)
    Wt = W[scene()["tool0"]]
    Ws = Wt @ _local(s["pos"], s["rot"])
    return np.linalg.inv(to_B(Wt)) @ optical_B(Ws)


def docked_optical(name="A_D405"):
    """Optical frame in B of a camera inside a part that is not held (the D405 in the docked camera tool)."""
    s = sensor(name)
    obj = next(o for o in scene()["objects"] if o["name"] == s["parent"])
    Wo = _local(obj["pos"], obj["rot"])
    return optical_B(Wo @ _local(s["pos"], s["rot"]))


def intrinsics(name, width=640):
    """Pinhole intrinsics from the sensor's field of view in the scene (the mock camera's; a real camera reports its
    own). Square pixels: the height follows from the two fields of view."""
    s = sensor(name)
    th, tv = math.tan(math.radians(s["hfov"]) / 2), math.tan(math.radians(s["vfov"]) / 2)
    height = int(round(width * tv / th / 2)) * 2
    fx = fy = (width / 2) / th
    return {"width": width, "height": height, "fx": fx, "fy": fy, "cx": width / 2 - 0.5, "cy": height / 2 - 0.5,
            "coeffs": [0, 0, 0, 0, 0], "model": "pinhole (from the scene field of view)"}


def sequences_summary():
    """The six kit sequences without their joint arrays: what the Grasp page lists."""
    p = os.path.join(RES, "sequences.json")
    if not os.path.exists(p):
        return []
    out = []
    for s in json.load(open(p))["sequences"]:
        out.append({k: s.get(k) for k in ("key", "title", "mode", "desc", "duration", "fps", "phaseT", "phaseLabel", "scans")})
    return out

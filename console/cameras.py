"""The two RealSense cameras of the kit, for the Cameras and Calibrate pages.

    D405   in the camera tool (P01): fixed in the dock for Mode A, carried by the gripper for Mode B (scan camera A)
    D435i  on the gripper collar (P05), on tool0 all the time (localisation camera G)

Real cameras are read with pyrealsense2 (colour and depth, the camera's own frame timestamps and frame numbers,
exposure when the firmware reports it). In --mock mode each camera is simulated from the same scene the 3D view
draws: its pose comes from the rig (scene.json) and the measured joints, its field of view from the scene, and what
it sees is the table, the cell floor and the ChArUco paddle (the Unity board texture, a valid DICT_5X5_100 9 x 6
board). The mock images are rendered, not photographed: good enough to exercise detection and the hand-eye solve
with a known answer, nothing more. Camera frames are not part of a recorded session yet (milestone M4).

Frames reach the browser as JPEG (or PNG when no encoder is installed) over plain HTTP from the agent's port:
/cam/<name>/color.jpg and /cam/<name>/depth.jpg. A camera renders or decodes only while somebody looks at it.
"""
from __future__ import annotations

import io
import json
import math
import os
import struct
import threading
import time
import zlib

import numpy as np

from . import lab

ROOT = lab.cell.ROOT
SNAP_DIR = os.path.join(ROOT, "captures", "camera")
CAMS = {"D405": {"sensor": "A_D405", "label": "D405, camera tool (scan camera A)", "mount": "dock / gripper"},
        "D435i": {"sensor": "G_D435i", "label": "D435i, gripper collar (localisation camera G)", "mount": "tool0"}}


# ---------------------------------------------------------------------------------------------- encoding
def encode(rgb, quality=80):
    """(bytes, content type) for an RGB uint8 image: JPEG through OpenCV or Pillow, else PNG with zlib only."""
    try:
        import cv2
        ok, buf = cv2.imencode(".jpg", rgb[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, quality])
        if ok:
            return buf.tobytes(), "image/jpeg"
    except Exception:
        pass
    try:
        from PIL import Image
        b = io.BytesIO()
        Image.fromarray(rgb).save(b, "JPEG", quality=quality)
        return b.getvalue(), "image/jpeg"
    except Exception:
        pass
    return png(rgb), "image/png"


def png(rgb):
    h, w = rgb.shape[:2]
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))
    def chunk(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)) + \
        chunk(b"IDAT", zlib.compress(raw, 3)) + chunk(b"IEND", b"")


def colorize_depth(z, lo=0.1, hi=2.0):
    """Depth (m) to a perceptual blue-to-yellow ramp; zero depth (no data) is black."""
    t = np.clip((z - lo) / (hi - lo), 0, 1)
    stops = np.array([[0.05, 0.03, 0.25], [0.13, 0.35, 0.62], [0.17, 0.65, 0.53], [0.66, 0.82, 0.24], [0.99, 0.91, 0.15]])
    x = t * (len(stops) - 1)
    i = np.clip(x.astype(int), 0, len(stops) - 2)
    f = (x - i)[..., None]
    rgb = stops[i] * (1 - f) + stops[i + 1] * f
    rgb[z <= 0] = 0
    return (rgb * 255).astype(np.uint8)


# ---------------------------------------------------------------------------------------------- one camera
class Camera(threading.Thread):
    kind = "none"

    def __init__(self, name, fps=10.0):
        super().__init__(daemon=True, name=f"cam-{name}")
        self.cam_name, self.fps = name, fps
        self.lock = threading.Lock()
        self.stop_ = threading.Event()
        self.wanted = 0.0                   # monotonic time of the last request: idle after 3 s without one
        self.color = None                   # RGB uint8
        self.depth = None                   # metres, float32
        self.jpg = {}
        self.meta = {}
        self.frames = 0
        self.t_frames = deque_times()
        self.error = ""
        self.intr = None
        self.pose = None                    # optical frame in B when known (mock), 4x4

    def want(self):
        self.wanted = time.monotonic()

    def active(self):
        return time.monotonic() - self.wanted < 3.0

    def grab(self):                         # -> (rgb, depth, meta) or None
        raise NotImplementedError

    def run(self):
        while not self.stop_.is_set():
            if not self.active():
                time.sleep(0.1)
                continue
            t0 = time.monotonic()
            try:
                got = self.grab()
            except Exception as e:
                self.error = f"{type(e).__name__}: {e}"
                time.sleep(0.5)
                continue
            if got is None:
                continue
            rgb, depth, meta = got
            jc, ct = encode(rgb)
            jd = encode(colorize_depth(depth))[0] if depth is not None else None
            with self.lock:
                self.color, self.depth, self.meta = rgb, depth, meta
                self.jpg = {"color": (jc, ct), "depth": (jd, ct) if jd else None}
                self.frames += 1
                self.t_frames.add(time.monotonic())
            self.error = ""
            dt = 1.0 / self.fps - (time.monotonic() - t0)
            if dt > 0 and self.kind == "mock":
                time.sleep(dt)

    def frame(self, stream="color"):
        self.want()
        with self.lock:
            return self.jpg.get(stream)

    def latest_bgr(self):
        with self.lock:
            return None if self.color is None else self.color[:, :, ::-1].copy()

    def still_frame(self):
        """(BGR image, intrinsics) for a calibration capture: the latest frame of a real camera."""
        return self.latest_bgr(), self.intr

    def depth_at(self, u, v):
        with self.lock:
            d = self.depth
        if d is None:
            return None
        u, v = int(u), int(v)
        if not (0 <= v < d.shape[0] and 0 <= u < d.shape[1]):
            return None
        patch = d[max(0, v - 2):v + 3, max(0, u - 2):u + 3]
        good = patch[patch > 0]
        return float(np.median(good)) if good.size else 0.0

    def status(self):
        with self.lock:
            meta = dict(self.meta)
            shape = None if self.color is None else list(self.color.shape[:2])
        return {"name": self.cam_name, "kind": self.kind, "label": CAMS.get(self.cam_name, {}).get("label", self.cam_name),
                "streaming": self.active() and self.frames > 0, "fps": self.t_frames.rate(), "frames": self.frames,
                "size": shape and [shape[1], shape[0]], "intrinsics": self.intr, "error": self.error, "meta": meta,
                "pose_B": None if self.pose is None else np.round(self.pose, 5).tolist()}

    def snapshot(self):
        """The colour image (PNG) and a JSON with its intrinsics and pose, as the Unity package saves its views."""
        with self.lock:
            rgb, meta = self.color, dict(self.meta)
        if rgb is None:
            return {"ok": False, "error": "no frame yet: open the camera view first"}
        os.makedirs(SNAP_DIR, exist_ok=True)
        stem = f"{time.strftime('%Y%m%d_%H%M%S')}_{self.cam_name}"
        open(os.path.join(SNAP_DIR, stem + ".png"), "wb").write(png(rgb))
        info = {"camera": self.cam_name, "kind": self.kind, "intrinsics": self.intr, "meta": meta,
                "T_B_optical": None if self.pose is None else self.pose.tolist(),
                "note": "optical frame: x right, y down, z forward (OpenCV); B = robot base frame, metres"}
        json.dump(info, open(os.path.join(SNAP_DIR, stem + ".json"), "w"), indent=1)
        return {"ok": True, "file": f"camera/{stem}.png", "json": f"camera/{stem}.json"}


class deque_times:
    def __init__(self, n=30):
        from collections import deque
        self.d = deque(maxlen=n)

    def add(self, t):
        self.d.append(t)

    def rate(self):
        if len(self.d) < 2 or time.monotonic() - self.d[-1] > 2.0:
            return 0.0
        return round((len(self.d) - 1) / max(1e-6, self.d[-1] - self.d[0]), 1)


# ---------------------------------------------------------------------------------------------- the mock camera
_board = None


def board():
    """The ChArUco paddle as the scene draws it: corners in B and its texture (grey, uint8)."""
    global _board
    if _board is None:
        quad = mesh_vertices("paddle_tag_charuco")
        obj = next(o for o in lab.scene()["objects"] if o["name"] == "paddle")
        W = lab._local(obj["pos"], obj["rot"])
        pts = (lab.P @ ((W[:3, :3] @ quad.T).T + W[:3, 3]).T).T
        tex = None
        try:
            from PIL import Image
            img = Image.open(os.path.join(lab.RES, "Textures", "charuco.bytes")).convert("L").resize((1800, 1200), Image.LANCZOS)
            tex = np.asarray(img)
        except Exception:
            pass
        if tex is None:                       # no image decoder: a plain 9 x 6 chessboard of the same size
            yy, xx = np.mgrid[0:1200, 0:1800]
            tex = (((xx // 200) + (yy // 200)) % 2 * 255).astype(np.uint8)
        _board = {"corners": pts, "tex": tex, "z": float(pts[:, 2].mean()),
                  "x0": float(pts[:, 0].min()), "x1": float(pts[:, 0].max()), "y0": float(pts[:, 1].min()), "y1": float(pts[:, 1].max())}
    return _board


def mesh_vertices(name):
    b = open(os.path.join(lab.RES, "Meshes", name + ".bytes"), "rb").read()
    nv = struct.unpack("<i", b[4:8])[0]
    return np.frombuffer(b, np.float32, nv * 3, 16).reshape(-1, 3).astype(float)


class MockCamera(Camera):
    kind = "mock"

    def __init__(self, name, q_fn, width=640, fps=8.0):
        super().__init__(name, fps)
        self.sensor = CAMS[name]["sensor"]
        self.q_fn = q_fn
        self.intr = lab.intrinsics(self.sensor, width)
        s = lab.sensor(self.sensor)
        self.on_tool = s["parent"] == "tool0"
        self.T_tool_opt = lab.tool0_to_optical(self.sensor) if self.on_tool else None
        self.T_fixed = None if self.on_tool else lab.docked_optical(self.sensor)
        self.rng = np.random.default_rng(3)
        self.rays = self._rays(self.intr, 1)
        self.n = 0

    @staticmethod
    def _rays(intr, ss):
        """Ray directions (z = 1) for every sub-pixel of an ss x ss supersampled image with intrinsics intr."""
        W, H = intr["width"] * ss, intr["height"] * ss
        u, v = np.meshgrid((np.arange(W) + 0.5) / ss - 0.5, (np.arange(H) + 0.5) / ss - 0.5)
        return np.stack([(u - intr["cx"]) / intr["fx"], (v - intr["cy"]) / intr["fy"], np.ones_like(u, float)], -1)

    def still_frame(self, width=1280, ss=2):
        """A calibration capture: rendered at full resolution with 2 x 2 supersampling, for the pose the arm is at."""
        T = self.current_pose()
        if T is None:
            return None, None
        intr = lab.intrinsics(self.sensor, width)
        g, _ = self._render(T, self._rays(intr, ss))
        H, W = intr["height"], intr["width"]
        g = g.reshape(H, ss, W, ss).mean(axis=(1, 3))
        g = np.clip(g + self.rng.normal(0, 0.008, g.shape), 0, 1)
        img = (g * 255).astype(np.uint8)
        return np.stack([img, img, img], -1), intr

    def current_pose(self):
        if self.on_tool:
            q = self.q_fn()
            if q is None:
                return None
            return lab.tool0_B(q) @ self.T_tool_opt
        return self.T_fixed

    def grab(self):
        T = self.current_pose()
        if T is None:
            time.sleep(0.2)
            return None
        self.pose = T
        img, depth = self._render(T, self.rays)
        H, W = img.shape
        # light: a soft vignette and sensor noise
        yy, xx = np.mgrid[0:H, 0:W]
        vig = 1 - 0.18 * (((xx - W / 2) / W) ** 2 + ((yy - H / 2) / H) ** 2) * 2
        g = np.clip(img * vig + self.rng.normal(0, 0.012, img.shape), 0, 1)
        rgb = np.stack([g * 0.98, g, g * 1.02], -1)
        rgb = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)
        depth = np.where(depth > 0.07, depth + self.rng.normal(0, 0.0006, depth.shape) * depth ** 2 * 10, 0).astype(np.float32)
        self.n += 1
        meta = {"frame_number": self.n, "timestamp_domain": "mock (host clock)", "t_host": time.monotonic(),
                "exposure_us": None, "source": "rendered from the scene"}
        return rgb, depth, meta

    def _render(self, T, rays):
        R, o = T[:3, :3], T[:3, 3]
        d = rays @ R.T                                       # ray directions in B (z of the optical frame = 1)
        H, W = d.shape[:2]
        img = np.empty((H, W), float)
        depth = np.zeros((H, W), np.float32)
        img[:] = 0.30 + 0.10 * np.clip(d[..., 2], -1, 1)     # far wall / room
        dz = d[..., 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            # floor of the cell, under the table
            s_f = (-0.765 - o[2]) / dz
            hit = (s_f > 0) & np.isfinite(s_f)
            img[hit] = 0.42
            depth[hit] = np.where(s_f[hit] < 6, s_f[hit], 0)
            # table top: aluminium plate with a 100 mm grid
            s_t = (0.0 - o[2]) / dz
            px, py = o[0] + s_t * d[..., 0], o[1] + s_t * d[..., 1]
            hit = (s_t > 0) & (px > -0.375) & (px < 0.875) & (py > -0.625) & (py < 0.625)
            grid = (np.abs(((px + 0.05) % 0.1) - 0.05) < 0.0012) | (np.abs(((py + 0.05) % 0.1) - 0.05) < 0.0012)
            img[hit] = np.where(grid[hit], 0.52, 0.66)
            depth[hit] = s_t[hit]
            # the ChArUco paddle (its board face; the paddle body around it is a lighter grey rim)
            b = board()
            s_b = (b["z"] - o[2]) / dz
            bx, by = o[0] + s_b * d[..., 0], o[1] + s_b * d[..., 1]
            rim = (s_b > 0) & (bx > b["x0"] - 0.012) & (bx < b["x1"] + 0.012) & (by > b["y0"] - 0.012) & (by < b["y1"] + 0.03)
            img[rim] = 0.80
            depth[rim] = s_b[rim]
            on = (s_b > 0) & (bx >= b["x0"]) & (bx < b["x1"]) & (by >= b["y0"]) & (by < b["y1"])
            th, tw = b["tex"].shape
            col = ((bx[on] - b["x0"]) / (b["x1"] - b["x0"]) * tw).astype(int).clip(0, tw - 1)
            row = ((1 - (by[on] - b["y0"]) / (b["y1"] - b["y0"])) * th).astype(int).clip(0, th - 1)
            img[on] = 0.08 + 0.84 * b["tex"][row, col] / 255.0
        return img, depth


# ---------------------------------------------------------------------------------------------- RealSense
class RealSenseCamera(Camera):
    """pyrealsense2, colour and depth aligned to nothing (each with its own intrinsics), global time on. Not run here:
    no RealSense hardware in the development environment."""
    kind = "realsense"

    def __init__(self, name, serial, width=640, height=480, fps=30):
        super().__init__(name, fps)
        import pyrealsense2 as rs
        self.rs = rs
        self.serial, self.size = serial, (width, height)
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        prof = self.pipe.start(cfg)
        dev = prof.get_device()
        for s in dev.query_sensors():
            if s.supports(rs.option.global_time_enabled):
                s.set_option(rs.option.global_time_enabled, 1)
        self.depth_scale = dev.first_depth_sensor().get_depth_scale()
        ci = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.intr = {"width": ci.width, "height": ci.height, "fx": ci.fx, "fy": ci.fy, "cx": ci.ppx, "cy": ci.ppy,
                     "coeffs": list(ci.coeffs), "model": str(ci.model)}

    def grab(self):
        rs = self.rs
        fs = self.pipe.wait_for_frames(2000)
        c, d = fs.get_color_frame(), fs.get_depth_frame()
        if not c:
            return None
        rgb = np.asanyarray(c.get_data()).copy()
        depth = (np.asanyarray(d.get_data()).astype(np.float32) * self.depth_scale) if d else None
        meta = {"frame_number": c.get_frame_number(), "timestamp_ms": c.get_timestamp(),
                "timestamp_domain": str(c.get_frame_timestamp_domain()), "t_host": time.monotonic()}
        if c.supports_frame_metadata(rs.frame_metadata_value.actual_exposure):
            meta["exposure_us"] = c.get_frame_metadata(rs.frame_metadata_value.actual_exposure)
        if c.supports_frame_metadata(rs.frame_metadata_value.sensor_timestamp):
            meta["sensor_timestamp_us"] = c.get_frame_metadata(rs.frame_metadata_value.sensor_timestamp)
        return rgb, depth, meta

    def stop(self):
        self.stop_.set()
        try:
            self.pipe.stop()
        except Exception:
            pass


# ---------------------------------------------------------------------------------------------- the hub
class CameraHub:
    def __init__(self):
        self.cams: dict[str, Camera] = {}
        self.note = ""

    def start(self, mock, q_fn):
        self.stop()
        if mock:
            for n in CAMS:
                try:
                    c = MockCamera(n, q_fn); c.start(); self.cams[n] = c
                except Exception as e:
                    self.note = f"mock camera {n}: {e}"
            return {"ok": bool(self.cams), "cameras": list(self.cams), "note": self.note}
        try:
            import pyrealsense2 as rs
        except Exception as e:
            self.note = f"pyrealsense2 not importable ({e}): no camera streams on this machine"
            return {"ok": False, "error": self.note}
        found = []
        for dev in rs.context().query_devices():
            nm, sn = dev.get_info(rs.camera_info.name), dev.get_info(rs.camera_info.serial_number)
            key = "D405" if "D405" in nm else "D435i" if "D435" in nm else None
            if key and key not in self.cams:
                try:
                    c = RealSenseCamera(key, sn); c.start(); self.cams[key] = c; found.append(f"{key} ({sn})")
                except Exception as e:
                    self.note += f"{key}: {e}; "
        if not found:
            self.note = self.note or "no RealSense camera found (USB 3 port, librealsense installed?)"
        return {"ok": bool(found), "cameras": found, "note": self.note}

    def stop(self):
        for c in self.cams.values():
            c.stop_.set()
            if hasattr(c, "stop"):
                try:
                    c.stop()
                except Exception:
                    pass
        self.cams = {}

    def get(self, name):
        return self.cams.get(name)

    def status(self):
        return {n: c.status() for n, c in self.cams.items()}

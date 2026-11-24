"""Generates the realistic lab around the SONAIR cell as data: Resources/SONAIR/lab.json + procedural textures.

The same file is consumed by the Unity runtime (SonairLab.cs) and by the Blender preview (verify_blender.py LAB=1),
so what is checked here is what Unity builds. Layout is written in the robot base frame B (metres, z up, floor at
z = -0.755, the table top at z = 0) and converted to Unity coordinates u = (-y_B, z_B, x_B) on output.
"""
import io, json, math, os
import numpy as np
from PIL import Image, ImageDraw, ImageFont

RES = "/home/claude/unity/SONAIR_Unity/Assets/SONAIR/Resources/SONAIR"
FZ = -0.755                      # floor height in B
H = 3.2                          # room height
CZ = FZ + H
X0, X1, Y0, Y1 = -3.2, 5.2, -3.6, 3.4   # inner faces of the walls
WT = 0.12                        # wall thickness
rng = np.random.default_rng(7)

MATS, PRIMS, LIGHTS = [], [], []


def mat(name, color, metallic=0.0, smooth=0.3, texture="", emission=None):
    MATS.append(dict(name=name, color=[float(c) for c in (list(color) + [1.0])[:4]], metallic=metallic, smoothness=smooth,
                     texture=texture, emission=[float(e) for e in emission] if emission else []))


def u_pos(p): return [round(-p[1], 5), round(p[2], 5), round(p[0], 5)]
def u_size(s): return [round(s[1], 5), round(s[2], 5), round(s[0], 5)]


def u_yaw(deg):
    """rotation about z_B by deg (right-handed) as a Unity quaternion (x, y, z, w)"""
    a = math.radians(deg) / 2; qb = (0.0, 0.0, math.sin(a), math.cos(a))
    return [round(qb[1], 6), round(-qb[2], 6), round(-qb[0], 6), round(qb[3], 6)]


def box(name, m, c, s, yaw=0.0, uv=(1.0, 1.0), fit=False, shadow=True):
    """c: centre in B, s: size along B axes (x, y, z); uv: metres per texture repeat (u, v)"""
    PRIMS.append(dict(type="box", name=name, mat=m, pos=u_pos(c), rot=u_yaw(yaw), size=u_size(s),
                      uv=[float(uv[0]), float(uv[1])], fit=fit, shadow=shadow))


def cyl(name, m, c, r, h, uv=(1.0, 1.0), shadow=True):
    """vertical cylinder: centre c in B, radius r, height h"""
    PRIMS.append(dict(type="cyl", name=name, mat=m, pos=u_pos(c), rot=[0, 0, 0, 1], size=[2 * r, h, 2 * r],
                      uv=[float(uv[0]), float(uv[1])], fit=False, shadow=shadow))


def light(kind, p, color=(1.0, 0.97, 0.92), intensity=1.0, rng_=7.0, angle=110.0, shadows=False, pitch=90.0):
    LIGHTS.append(dict(type=kind, pos=u_pos(p), rot=[float(pitch), 0.0, 0.0], color=list(color), intensity=intensity,
                       range=rng_, angle=angle, shadows=shadows))


# ------------------------------------------------------------------ procedural, tileable textures
def periodic_noise(n, scale, seed):
    """tileable smooth noise in [0, 1] by filtering white noise in the Fourier domain"""
    r = np.random.default_rng(seed); w = r.standard_normal((n, n))
    f = np.fft.fftfreq(n)[:, None] ** 2 + np.fft.fftfreq(n)[None, :] ** 2
    g = np.real(np.fft.ifft2(np.fft.fft2(w) * np.exp(-f * (n / scale) ** 2)))
    return (g - g.min()) / (np.ptp(g) + 1e-9)


def save_tex(key, img):
    b = io.BytesIO(); img.save(b, "PNG"); open(os.path.join(RES, "Textures", key + ".bytes"), "wb").write(b.getvalue())


def tex_epoxy():
    n = 512; base = 0.80 + 0.20 * periodic_noise(n, 40, 1)            # broad mottling
    img = np.stack([base] * 3, -1) * np.array([0.98, 1.0, 1.02])
    k = int(n * n * 0.035)                                            # chips of the flake epoxy
    ys, xs = rng.integers(0, n, k), rng.integers(0, n, k)
    tone = rng.choice([0.55, 0.7, 1.18, 1.3], k)
    for y, x, t in zip(ys, xs, tone): img[y, x] *= t; img[(y + 1) % n, x] *= 0.5 + 0.5 * t
    img *= 0.97 + 0.03 * periodic_noise(n, 4, 2)[..., None]
    save_tex("lab_epoxy", Image.fromarray(np.clip(img * 255, 0, 255).astype(np.uint8)))


def tex_wall():
    n = 256; v = 0.95 + 0.05 * periodic_noise(n, 6, 3); v = v * (0.985 + 0.015 * periodic_noise(n, 60, 4))
    save_tex("lab_wall", Image.fromarray(np.clip(np.stack([v] * 3, -1) * 255, 0, 255).astype(np.uint8)))


def tex_ceiling():
    n = 512; v = 0.93 + 0.05 * periodic_noise(n, 3, 5); img = np.stack([v] * 3, -1)
    holes = rng.integers(0, n, (int(n * n * 0.01), 2)); img[holes[:, 0], holes[:, 1]] *= 0.82   # mineral fibre pores
    for k in (0, n // 2):                                               # T-bar grid: two 600 mm tiles per repeat
        img[k:k + 5, :] = [0.86, 0.86, 0.87]; img[:, k:k + 5] = [0.86, 0.86, 0.87]
        img[k + 5, :] *= 0.9; img[:, k + 5] *= 0.9
    save_tex("lab_ceiling", Image.fromarray(np.clip(img * 255, 0, 255).astype(np.uint8)))


def tex_hazard():
    w, h = 256, 64; y, x = np.mgrid[0:h, 0:w]
    stripe = ((x + y) // 32) % 2 == 0                                   # 45 deg, period 64 px divides 256: tileable
    img = np.where(stripe[..., None], [0.96, 0.78, 0.07], [0.06, 0.06, 0.06])
    img *= (0.94 + 0.06 * periodic_noise(256, 8, 6)[:h, :w])[..., None]   # scuffed tape
    save_tex("lab_hazard", Image.fromarray(np.clip(img * 255, 0, 255).astype(np.uint8)))


def font(sz, mono=False):
    p = "/usr/share/fonts/truetype/dejavu/" + ("DejaVuSansMono.ttf" if mono else "DejaVuSans.ttf")
    return ImageFont.truetype(p, sz)


def tex_whiteboard():
    W, Hh = 1024, 512; im = Image.new("RGB", (W, Hh), (246, 247, 246)); d = ImageDraw.Draw(im)
    blue, red, black = (40, 70, 160), (170, 40, 40), (40, 40, 45)
    for i, (t, x) in enumerate((("command q*", 60), ("sim", 300), ("real UR", 540))):
        d.rounded_rectangle([x, 90, x + 170, 170], 14, outline=blue, width=5); d.text((x + 22, 112), t, fill=blue, font=font(30))
    d.line([230, 130, 300, 130], fill=blue, width=5); d.line([470, 130, 540, 130], fill=blue, width=5)
    d.text((300, 210), "gap = |sim - real|", fill=red, font=font(34))
    d.text((300, 262), "GCR = 1 - e / e(S0)", fill=red, font=font(34))
    d.line([760, 430, 760, 90], fill=black, width=4); d.line([760, 430, 990, 430], fill=black, width=4)
    xs = np.linspace(0, 1, 60); ys = 0.5 + 0.35 * np.sin(xs * 9) * np.exp(-2 * xs)
    d.line([(770 + 215 * a, 420 - 300 * b) for a, b in zip(xs, ys)], fill=red, width=4)
    d.text((780, 60), "elbow current", fill=black, font=font(24))
    d.text((60, 380), "to do: E1 payload step, 6-face IMU cal, ChArUco check", fill=black, font=font(26))
    for k in range(3): d.line([60, 440 + 18 * k, 600 - 90 * k, 440 + 18 * k], fill=(205, 208, 212), width=2)
    save_tex("lab_whiteboard", im)


def tex_screen(key, kind):
    W, Hh = 640, 400; im = Image.new("RGB", (W, Hh), (18, 22, 28)); d = ImageDraw.Draw(im)
    d.rectangle([0, 0, W, 34], fill=(32, 38, 48)); d.text((14, 6), "SONAIR cell  |  recording" if kind == 0 else "sonair-twin  |  replay", fill=(210, 220, 230), font=font(20))
    for r, col in enumerate(((90, 170, 240), (240, 150, 70), (120, 210, 140))):
        y0 = 60 + r * 110; d.rectangle([20, y0, W - 20, y0 + 96], outline=(52, 60, 72), width=2)
        x = np.linspace(0, 1, 200); y = 0.5 + 0.38 * np.sin(2 * np.pi * (2 + r) * x + r) * (0.6 + 0.4 * np.cos(9 * x))
        d.line([(28 + (W - 56) * a, y0 + 90 - 84 * b) for a, b in zip(x, y)], fill=col, width=3)
    save_tex(key, im)


# ------------------------------------------------------------------ materials
mat("lab_floor", (0.60, 0.61, 0.62), 0.0, 0.62, "lab_epoxy")
mat("lab_wall", (0.88, 0.88, 0.86), 0.0, 0.06, "lab_wall")
mat("lab_dado", (0.55, 0.61, 0.67), 0.0, 0.12, "lab_wall")
mat("lab_skirting", (0.22, 0.23, 0.25), 0.0, 0.3)
mat("lab_ceiling", (0.95, 0.95, 0.95), 0.0, 0.05, "lab_ceiling")
mat("lab_led", (1.0, 1.0, 1.0), 0.0, 0.2, "", (1.0, 0.98, 0.94))
mat("lab_window", (0.70, 0.80, 0.92), 0.0, 0.9, "", (0.62, 0.72, 0.86))
mat("lab_frame", (0.88, 0.89, 0.90), 0.3, 0.45)
mat("lab_hazard", (1.0, 1.0, 1.0), 0.0, 0.35, "lab_hazard")
mat("lab_walkway", (0.16, 0.52, 0.28), 0.0, 0.35)
mat("lab_profile", (0.84, 0.85, 0.86), 0.5, 0.45)
mat("lab_polycarb", (0.86, 0.92, 0.97, 0.14), 0.0, 0.92)
mat("lab_cabinet", (0.73, 0.75, 0.77), 0.35, 0.38)
mat("lab_cabinet_dark", (0.30, 0.32, 0.34), 0.3, 0.35)
mat("lab_benchtop", (0.52, 0.60, 0.66), 0.0, 0.25)
mat("lab_steel", (0.30, 0.31, 0.33), 0.6, 0.45)
mat("lab_plastic", (0.07, 0.07, 0.08), 0.0, 0.45)
mat("lab_board", (1.0, 1.0, 1.0), 0.0, 0.7, "lab_whiteboard")
mat("lab_screen0", (0.0, 0.0, 0.0), 0.0, 0.8, "lab_screen0", (0.9, 0.9, 0.9))
mat("lab_screen1", (0.0, 0.0, 0.0), 0.0, 0.8, "lab_screen1", (0.9, 0.9, 0.9))
mat("lab_red", (0.78, 0.07, 0.05), 0.1, 0.5)
mat("lab_yellow", (0.96, 0.76, 0.06), 0.0, 0.4)
mat("lab_exit", (0.05, 0.6, 0.25), 0.0, 0.5, "", (0.1, 0.9, 0.35))
mat("lab_door", (0.42, 0.45, 0.49), 0.1, 0.35)
mat("lab_cardboard", (0.66, 0.52, 0.36), 0.0, 0.15)
mat("lab_orange_print", (0.93, 0.47, 0.12), 0.0, 0.35)

# ------------------------------------------------------------------ shell
box("floor", "lab_floor", ((X0 + X1) / 2, (Y0 + Y1) / 2, FZ - 0.01), (X1 - X0, Y1 - Y0, 0.02), uv=(2.0, 2.0))
box("ceiling", "lab_ceiling", ((X0 + X1) / 2, (Y0 + Y1) / 2, CZ + 0.01), (X1 - X0, Y1 - Y0, 0.02), uv=(1.2, 1.2), shadow=False)


def wall_x(x, y_from, y_to, z_from, z_to, name, m="lab_wall", depth=WT, outward=-1):
    """wall slab on the plane x = const (inner face at x), spanning y and z"""
    box(name, m, (x + outward * depth / 2, (y_from + y_to) / 2, (z_from + z_to) / 2), (depth, y_to - y_from, z_to - z_from), uv=(2.5, 2.5))


def wall_y(y, x_from, x_to, z_from, z_to, name, m="lab_wall", depth=WT, outward=1):
    box(name, m, ((x_from + x_to) / 2, y + outward * depth / 2, (z_from + z_to) / 2), (x_to - x_from, depth, z_to - z_from), uv=(2.5, 2.5))


# left wall y = Y1 with three windows
SILL, HEAD = FZ + 0.95, FZ + 2.35
WIN = [(-1.5, 1.7), (0.75, 1.7), (3.0, 1.7)]
wall_y(Y1, X0 - WT, X1 + WT, FZ, SILL, "wall_left_low")
wall_y(Y1, X0 - WT, X1 + WT, HEAD, CZ, "wall_left_high")
edges = [X0 - WT] + [v for c, w in WIN for v in (c - w / 2, c + w / 2)] + [X1 + WT]
for i in range(0, len(edges), 2): wall_y(Y1, edges[i], edges[i + 1], SILL, HEAD, f"wall_left_pier{i // 2}")
for k, (c, w) in enumerate(WIN):
    box(f"window{k}_glass", "lab_window", (c, Y1 + 0.06, (SILL + HEAD) / 2), (w, 0.01, HEAD - SILL), shadow=False)
    for dx in (-w / 2, 0.0, w / 2): box(f"window{k}_mullion", "lab_frame", (c + dx, Y1 + 0.04, (SILL + HEAD) / 2), (0.05, 0.06, HEAD - SILL))
    for z in (SILL, (SILL + HEAD) / 2 + 0.25, HEAD): box(f"window{k}_transom", "lab_frame", (c, Y1 + 0.04, z), (w, 0.06, 0.05))
    box(f"window{k}_sill", "lab_frame", (c, Y1 - 0.06, SILL - 0.015), (w + 0.1, 0.16, 0.03))
# right wall y = Y0, plain, with a cable tray
wall_y(Y0, X0 - WT, X1 + WT, FZ, CZ, "wall_right", outward=-1)
box("cable_tray", "lab_profile", ((X0 + X1) / 2, Y0 + 0.2, FZ + 2.65), (X1 - X0 - 0.4, 0.3, 0.06))
# back wall x = X0 with a door
DY0, DY1, DH = -3.05, -2.05, FZ + 2.1
wall_x(X0, Y0 - WT, DY0, FZ, CZ, "wall_back_a"); wall_x(X0, DY1, Y1 + WT, FZ, CZ, "wall_back_b")
wall_x(X0, DY0, DY1, DH, CZ, "wall_back_over_door")
box("door", "lab_door", (X0 - 0.03, (DY0 + DY1) / 2, (FZ + DH) / 2), (0.05, DY1 - DY0 - 0.04, DH - FZ - 0.02))
box("door_window", "lab_window", (X0 + 0.0, (DY0 + DY1) / 2 + 0.18, FZ + 1.55), (0.01, 0.22, 0.6), shadow=False)
box("door_handle", "lab_profile", (X0 + 0.05, DY1 - 0.15, FZ + 1.05), (0.05, 0.14, 0.03))
for dy in (DY0, DY1): box("door_frame", "lab_frame", (X0 + 0.01, dy, (FZ + DH) / 2), (0.06, 0.06, DH - FZ))
box("exit_sign", "lab_exit", (X0 + 0.03, (DY0 + DY1) / 2, DH + 0.18), (0.03, 0.36, 0.14), shadow=False)
# front wall x = X1
wall_x(X1, Y0 - WT, Y1 + WT, FZ, CZ, "wall_front", outward=1)
# dado band and skirting on every wall
for nm, (a, b, axis, v, s) in {"left": (X0, X1, "y", Y1, -1), "right": (X0, X1, "y", Y0, 1), "back": (Y0, Y1, "x", X0, 1), "front": (Y0, Y1, "x", X1, -1)}.items():
    if axis == "y":
        box(f"skirting_{nm}", "lab_skirting", ((a + b) / 2, v + s * 0.006, FZ + 0.05), (b - a, 0.012, 0.10))
    else:
        box(f"skirting_{nm}", "lab_skirting", (v + s * 0.006, (a + b) / 2, FZ + 0.05), (0.012, b - a, 0.10))
box("dado_right", "lab_dado", ((X0 + X1) / 2, Y0 + 0.004, FZ + 0.6), (X1 - X0, 0.008, 1.0))
box("dado_front", "lab_dado", (X1 - 0.004, (Y0 + Y1) / 2, FZ + 0.6), (0.008, Y1 - Y0, 1.0))

# ------------------------------------------------------------------ ceiling LED panels and lights
for i, x in enumerate((-1.9, 0.35, 2.6, 4.5)):
    for j, y in enumerate((-2.1, 0.15, 2.3)):
        box(f"led_{i}{j}", "lab_led", (x, y, CZ - 0.008), (1.18, 0.58, 0.012), shadow=False)
        main = abs(x - 0.35) < 0.1 and abs(y - 0.15) < 0.1
        light("Point", (x, y, CZ - 0.25), intensity=1.6 if main else 1.05, rng_=7.5, shadows=main)

# ------------------------------------------------------------------ the robot cell: floor marking, fence, controller, e-stop
CX0, CX1, CY0, CY1 = -1.0, 1.55, -1.3, 1.35
TW = 0.05
for nm, c, s in (("tape_n", ((CX0 + CX1) / 2, CY1, FZ + 0.001), (CX1 - CX0 + TW, TW, 0.002)),
                 ("tape_s", ((CX0 + CX1) / 2, CY0, FZ + 0.001), (CX1 - CX0 + TW, TW, 0.002)),
                 ("tape_w", (CX0, (CY0 + CY1) / 2, FZ + 0.001), (TW, CY1 - CY0, 0.002)),
                 ("tape_e", (CX1, (CY0 + CY1) / 2, FZ + 0.001), (TW, CY1 - CY0, 0.002))):
    box(nm, "lab_hazard", c, s, uv=(0.2, 0.05), shadow=False)
for nm, c, s in (("walkway_a", (3.4, -0.2, FZ + 0.001), (0.06, 5.2, 0.002)), ("walkway_b", (2.0, -0.2, FZ + 0.001), (0.06, 5.2, 0.002))):
    box(nm, "lab_walkway", c, s, shadow=False)
# guard fence on the two far sides (aluminium profile + polycarbonate)
FH = 1.9


def fence(x0, y0, x1, y1, tag):
    L = math.hypot(x1 - x0, y1 - y0); n = max(1, int(round(L / 1.0)))
    for k in range(n + 1):
        x, y = x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n
        box(f"fence_{tag}_post{k}", "lab_profile", (x, y, FZ + FH / 2), (0.045, 0.045, FH))
        box(f"fence_{tag}_foot{k}", "lab_steel", (x, y, FZ + 0.004), (0.12, 0.12, 0.008))
    along_x = abs(x1 - x0) > abs(y1 - y0)
    for z in (FZ + 0.2, FZ + 1.0, FZ + FH - 0.02):
        box(f"fence_{tag}_rail", "lab_profile", ((x0 + x1) / 2, (y0 + y1) / 2, z), (L, 0.04, 0.04) if along_x else (0.04, L, 0.04))
    box(f"fence_{tag}_panel", "lab_polycarb", ((x0 + x1) / 2, (y0 + y1) / 2, FZ + 1.05), (L, 0.006, 1.7) if along_x else (0.006, L, 1.7), shadow=False)


fence(CX0 - 0.15, CY0 - 0.15, CX0 - 0.15, CY1 + 0.15, "back")
fence(CX0 - 0.15, CY1 + 0.15, CX1 + 0.15, CY1 + 0.15, "left")
# robot controller cabinet with a teach pendant hung on its side (generic, no branding)
cx, cy = -0.35, -1.78
box("controller", "lab_cabinet", (cx, cy, FZ + 0.31), (0.46, 0.47, 0.58))
box("controller_door_seam", "lab_cabinet_dark", (cx + 0.232, cy, FZ + 0.31), (0.004, 0.44, 0.54), shadow=False)
box("controller_vent", "lab_cabinet_dark", (cx, cy - 0.236, FZ + 0.42), (0.30, 0.004, 0.12), shadow=False)
for dx in (-0.18, 0.18):
    for dy in (-0.18, 0.18): cyl("controller_castor", "lab_plastic", (cx + dx, cy + dy, FZ + 0.012), 0.022, 0.024)
box("pendant", "lab_plastic", (cx + 0.26, cy + 0.05, FZ + 0.42), (0.05, 0.30, 0.22))
box("pendant_screen", "lab_screen1", (cx + 0.287, cy + 0.05, FZ + 0.43), (0.004, 0.24, 0.15), fit=True, shadow=False)
box("pendant_estop", "lab_red", (cx + 0.29, cy + 0.17, FZ + 0.50), (0.02, 0.035, 0.035))
box("conduit", "lab_cabinet_dark", (cx, (cy + 0.24 - 0.58) / 2, FZ + 0.012), (0.05, abs(cy + 0.24 + 0.58), 0.024), shadow=False)
# emergency-stop pillar at the cell entry
ex, ey = CX1 + 0.18, CY0 - 0.18
box("estop_pillar", "lab_yellow", (ex, ey, FZ + 0.55), (0.10, 0.10, 1.1))
cyl("estop_button", "lab_red", (ex, ey, FZ + 1.125), 0.03, 0.05)
box("estop_base", "lab_steel", (ex, ey, FZ + 0.005), (0.25, 0.25, 0.01))

# ------------------------------------------------------------------ benches under the windows
def bench(x0, x1, y_back, depth=0.75, h=0.9, tag="b"):
    yc = y_back - depth / 2
    box(f"bench_{tag}_top", "lab_benchtop", ((x0 + x1) / 2, yc, FZ + h - 0.015), (x1 - x0, depth, 0.03))
    for x in (x0 + 0.04, x1 - 0.04):
        for y in (yc - depth / 2 + 0.04, yc + depth / 2 - 0.04):
            box(f"bench_{tag}_leg", "lab_steel", (x, y, FZ + (h - 0.03) / 2), (0.04, 0.04, h - 0.03))
    box(f"bench_{tag}_rail", "lab_steel", ((x0 + x1) / 2, yc + depth / 2 - 0.04, FZ + 0.15), (x1 - x0, 0.03, 0.03))
    box(f"bench_{tag}_shelf", "lab_steel", ((x0 + x1) / 2, yc, FZ + 0.18), (x1 - x0 - 0.1, depth - 0.1, 0.015))
    box(f"bench_{tag}_back", "lab_benchtop", ((x0 + x1) / 2, y_back - 0.01, FZ + h + 0.25), (x1 - x0, 0.02, 0.5))
    box(f"bench_{tag}_shelfup", "lab_benchtop", ((x0 + x1) / 2, y_back - 0.15, FZ + h + 0.48), (x1 - x0, 0.28, 0.025))
    return FZ + h


tz = bench(-2.4, -0.1, Y1 - 0.02, tag="w1")
box("scope", "lab_cabinet_dark", (-2.0, Y1 - 0.35, tz + 0.10), (0.36, 0.20, 0.20))
box("scope_screen", "lab_screen0", (-2.04, Y1 - 0.452, tz + 0.11), (0.18, 0.004, 0.12), fit=True, shadow=False)
box("psu", "lab_cabinet", (-1.5, Y1 - 0.34, tz + 0.07), (0.24, 0.26, 0.14))
box("psu_face", "lab_plastic", (-1.5, Y1 - 0.472, tz + 0.07), (0.22, 0.004, 0.12), shadow=False)
box("parts_bin_a", "lab_red", (-0.9, Y1 - 0.25, tz + 0.06), (0.30, 0.20, 0.12))
box("parts_bin_b", "lab_yellow", (-0.55, Y1 - 0.25, tz + 0.06), (0.30, 0.20, 0.12))
box("carton", "lab_cardboard", (-1.2, Y1 - 0.15, tz + 0.48 + 0.12), (0.40, 0.28, 0.22))
tz2 = bench(0.6, 3.6, Y1 - 0.02, tag="w2")
# a desktop 3D printer (the kit parts are printed in the lab)
px, py = 1.3, Y1 - 0.42
for dx in (-0.22, 0.22):
    for dy in (-0.2, 0.2): box("printer_post", "lab_steel", (px + dx, py + dy, tz2 + 0.25), (0.025, 0.025, 0.5))
for z in (tz2 + 0.02, tz2 + 0.49):
    box("printer_frame", "lab_steel", (px, py - 0.2, z), (0.47, 0.025, 0.025)); box("printer_frame", "lab_steel", (px, py + 0.2, z), (0.47, 0.025, 0.025))
box("printer_bed", "lab_cabinet_dark", (px, py, tz2 + 0.10), (0.36, 0.34, 0.01))
box("printer_part", "lab_orange_print", (px + 0.05, py, tz2 + 0.13), (0.09, 0.06, 0.05))
box("printer_gantry", "lab_profile", (px, py, tz2 + 0.36), (0.44, 0.03, 0.03))
box("spool", "lab_orange_print", (px + 0.33, py + 0.1, tz2 + 0.12), (0.07, 0.20, 0.20))
box("laptop_base", "lab_cabinet_dark", (2.7, Y1 - 0.4, tz2 + 0.01), (0.34, 0.24, 0.02))
box("laptop_lid", "lab_screen0", (2.7, Y1 - 0.30, tz2 + 0.13), (0.34, 0.01, 0.22), yaw=0.0, fit=True)

# ------------------------------------------------------------------ back wall: whiteboard and cabinets
box("whiteboard", "lab_board", (X0 + 0.02, -0.35, FZ + 1.55), (0.02, 2.4, 1.2), fit=True)
box("whiteboard_frame_top", "lab_profile", (X0 + 0.03, -0.35, FZ + 2.16), (0.03, 2.44, 0.03))
box("whiteboard_frame_bot", "lab_profile", (X0 + 0.03, -0.35, FZ + 0.94), (0.03, 2.44, 0.03))
box("whiteboard_tray", "lab_profile", (X0 + 0.07, -0.35, FZ + 0.93), (0.08, 1.2, 0.02))
for k, y in enumerate((1.15, 1.75, 2.35)):
    box(f"cabinet{k}", "lab_cabinet", (X0 + 0.26, y, FZ + 1.0), (0.5, 0.58, 2.0))
    box(f"cabinet{k}_seam", "lab_cabinet_dark", (X0 + 0.512, y, FZ + 1.0), (0.004, 0.006, 1.94), shadow=False)
    box(f"cabinet{k}_handle", "lab_steel", (X0 + 0.53, y - 0.04, FZ + 1.05), (0.02, 0.012, 0.22))
    box(f"cabinet{k}_plinth", "lab_cabinet_dark", (X0 + 0.26, y, FZ + 0.03), (0.48, 0.56, 0.06))
cyl("extinguisher", "lab_red", (X0 + 0.15, DY1 + 0.25, FZ + 0.33), 0.085, 0.56)
box("extinguisher_bracket", "lab_steel", (X0 + 0.02, DY1 + 0.25, FZ + 0.55), (0.02, 0.08, 0.12))

# ------------------------------------------------------------------ operator desk facing the cell (the Jetson display)
dx0, dy0 = 2.55, -2.65
box("desk_top", "lab_benchtop", (dx0, dy0, FZ + 0.735), (0.8, 1.6, 0.03))
for sx in (-0.36, 0.36):
    for sy in (-0.76, 0.76): box("desk_leg", "lab_steel", (dx0 + sx, dy0 + sy, FZ + 0.36), (0.04, 0.04, 0.72))
for k, yy in enumerate((dy0 - 0.33, dy0 + 0.33)):
    box(f"monitor{k}", "lab_plastic", (dx0 - 0.18, yy, FZ + 1.05), (0.03, 0.56, 0.34), yaw=-12 * (1 - 2 * k))
    box(f"monitor{k}_screen", f"lab_screen{k}", (dx0 - 0.162, yy, FZ + 1.055), (0.004, 0.53, 0.31), yaw=-12 * (1 - 2 * k), fit=True, shadow=False)
    box(f"monitor{k}_stand", "lab_plastic", (dx0 - 0.21, yy, FZ + 0.82), (0.03, 0.06, 0.16))
    box(f"monitor{k}_foot", "lab_plastic", (dx0 - 0.2, yy, FZ + 0.755), (0.18, 0.22, 0.012))
box("keyboard", "lab_plastic", (dx0 + 0.12, dy0, FZ + 0.758), (0.15, 0.44, 0.015))
box("tower", "lab_cabinet_dark", (dx0 - 0.15, dy0 + 0.95, FZ + 0.23), (0.45, 0.2, 0.44))
# office chair: the operator faces the cell and the screens
chx, chy = dx0 + 0.62, dy0
cyl("chair_seat", "lab_plastic", (chx, chy, FZ + 0.47), 0.24, 0.07)
cyl("chair_post", "lab_steel", (chx, chy, FZ + 0.26), 0.025, 0.36)
for a in range(5):
    th = 2 * math.pi * a / 5
    box("chair_leg", "lab_plastic", (chx + 0.14 * math.cos(th), chy + 0.14 * math.sin(th), FZ + 0.06), (0.30, 0.04, 0.03), yaw=math.degrees(th))
box("chair_back", "lab_plastic", (chx + 0.25, chy, FZ + 0.78), (0.05, 0.42, 0.5))
# parts trolley near the cell
tx_, ty_ = 1.95, 1.75
for z in (FZ + 0.15, FZ + 0.85):
    box("trolley_shelf", "lab_cabinet_dark", (tx_, ty_, z), (0.6, 0.4, 0.02))
for sx in (-0.28, 0.28):
    for sy in (-0.18, 0.18): box("trolley_post", "lab_profile", (tx_ + sx, ty_ + sy, FZ + 0.48), (0.025, 0.025, 0.9))
box("trolley_crate", "lab_cardboard", (tx_, ty_, FZ + 0.96), (0.36, 0.28, 0.2))

LAB = dict(version="1.0", note="Realistic lab around the SONAIR cell. Unity coordinates, metres. Generated by make_lab.py; edit there, not here.",
           materials=MATS, prims=PRIMS, lights=LIGHTS,
           ambientSky=[0.62, 0.66, 0.72], ambientEquator=[0.46, 0.47, 0.48], ambientGround=[0.28, 0.27, 0.26],
           hide=["floor"], keyLightEuler=[62.0, -30.0, 0.0], keyLightIntensity=0.55)

if __name__ == "__main__":
    tex_epoxy(); tex_wall(); tex_ceiling(); tex_hazard(); tex_whiteboard(); tex_screen("lab_screen0", 0); tex_screen("lab_screen1", 1)
    json.dump(LAB, open(os.path.join(RES, "lab.json"), "w"), indent=0)
    print(f"lab.json: {len(PRIMS)} primitives, {len(LIGHTS)} lights, {len(MATS)} materials")

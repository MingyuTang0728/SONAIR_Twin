# Blender check of the Unity data: rebuilds the scene from scene.json / sequences.json / Meshes/*.bytes
# using the same rules the C# runtime uses, converts Unity -> Blender and renders frames.
# blender -b -P verify_blender.py -- <SEQ> <t1,t2,...> <out_prefix> [cam: ov|A|G]
import bpy, json, struct, sys, math, os
import numpy as np
from mathutils import Matrix, Vector

argv = sys.argv[sys.argv.index("--") + 1:]
SEQ, TIMES, OUT = argv[0], [float(x) for x in argv[1].split(",")], argv[2]
CAM = argv[3] if len(argv) > 3 else "ov"
RES = "/home/claude/unity/SONAIR_Unity/Assets/SONAIR/Resources/SONAIR"
SC = json.load(open(RES + "/scene.json")); SQ = {s["key"]: s for s in json.load(open(RES + "/sequences.json"))["sequences"]}[SEQ]
M = np.array([[0, -1, 0], [0, 0, 1], [1, 0, 0]], float)

# ---------------- Unity math (same as the C# runtime)
def qmul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])
def qrot(q, v):
    x, y, z, w = q; u = np.array([x, y, z]); v = np.asarray(v, float); return v + 2 * np.cross(u, np.cross(u, v) + w * v)
def aa(rad, axis):
    a = np.asarray(axis, float); a /= np.linalg.norm(a); s = math.sin(rad / 2); return np.array([*(a * s), math.cos(rad / 2)])
def q2R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
def look_rotation(fwd, up):
    f = np.asarray(fwd, float); f /= np.linalg.norm(f); r = np.cross(up, f); r /= np.linalg.norm(r); u = np.cross(f, r)
    return np.column_stack([r, u, f])   # Unity LookRotation as a matrix (columns = local x, y, z in world)

def lerp_arr(arr, n, t, fps):
    a = np.array(arr).reshape(-1, n); x = t * fps; i = int(min(max(math.floor(x), 0), len(a) - 1)); j = min(i + 1, len(a) - 1); f = x - i
    return a[i] * (1 - f) + a[j] * f

def state(t):
    q = lerp_arr(SQ["q"], 6, t, SQ["fps"]); c = float(np.array(SQ["grip"])[min(int(round(t * SQ["fps"])), len(SQ["grip"]) - 1)])
    G = SC["gripper"]; C = np.array(G["closure"]); GQ = np.array(G["q"]).reshape(len(C), G["nq"])
    qg = np.array([np.interp(c, C, GQ[:, k]) for k in range(G["nq"])])
    qall = np.zeros(len(SC["joints"])); qall[:6] = q; qall[6:6 + len(qg)] = qg
    W = []
    for b in SC["bodies"]:
        lr, lp = np.array(b["rot"]), np.array(b["pos"])
        if b["joint"] >= 0:
            r = aa(qall[b["joint"]], b["axis"]); pv = np.array(b["pivot"]); lp = lp + qrot(lr, pv - qrot(r, pv)); lr = qmul(lr, r)
        if b["parent"] < 0: W.append((lr, lp))
        else:
            pr, pp = W[b["parent"]]; W.append((qmul(pr, lr), pp + qrot(pr, lp)))
    tool = W[SC["tool0"]]
    obj = {}
    for o in SC["objects"]:
        if o["parent"] == "tool0": pr, pp = tool; obj[o["name"]] = (qmul(pr, o["rot"]), pp + qrot(pr, o["pos"])); continue
        obj[o["name"]] = (np.array(o["rot"]), np.array(o["pos"]))
    cur = {}
    for e in SQ["tracks"]:
        if e["t"] <= t + 1e-9: cur[e["obj"]] = e
    for n, e in cur.items():
        if e["parent"] == "world": obj[n] = (np.array(e["rot"]), np.array(e["pos"]))
        else: pr, pp = tool; obj[n] = (qmul(pr, e["rot"]), pp + qrot(pr, e["pos"]))
    return W, obj, c

def u2b(rot, pos):
    R = M.T @ q2R(rot) @ M; p = M.T @ np.asarray(pos); T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p; return Matrix(T.tolist())

# ---------------- build meshes
def load_mesh(name):
    b = open(f"{RES}/Meshes/{name}.bytes", "rb").read(); nv, ni, fl = struct.unpack("<III", b[4:16]); o = 16
    V = np.frombuffer(b, "<f4", nv * 3, o).reshape(-1, 3); o += nv * 12
    o += nv * 12
    if fl & 2: UV = np.frombuffer(b, "<f4", nv * 2, o).reshape(-1, 2); o += nv * 8
    else: UV = None
    I = np.frombuffer(b, "<u4", ni, o).reshape(-1, 3)
    Vb = V.astype(float) @ M; Fb = I[:, [0, 2, 1]]      # Unity -> Blender, winding back
    me = bpy.data.meshes.new(name); me.from_pydata(Vb.tolist(), [], Fb.tolist()); me.update()
    if UV is not None:
        uvl = me.uv_layers.new()
        for li, loop in enumerate(me.loops): uvl.data[li].uv = UV[loop.vertex_index]
    for p in me.polygons: p.use_smooth = UV is None
    return me
MATS = {}
for m in SC["materials"]:
    mt = bpy.data.materials.new(m["name"]); mt.use_nodes = True; bs = mt.node_tree.nodes["Principled BSDF"]
    c = m["color"]; bs.inputs["Base Color"].default_value = (c[0] ** 2.2, c[1] ** 2.2, c[2] ** 2.2, 1)
    bs.inputs["Metallic"].default_value = m["metallic"]; bs.inputs["Roughness"].default_value = 1 - m["smoothness"]
    if c[3] < 1: bs.inputs["Alpha"].default_value = c[3]
    if m["texture"]:
        tx = mt.node_tree.nodes.new("ShaderNodeTexImage"); p = f"/tmp/{m['texture']}.png"
        open(p, "wb").write(open(f"{RES}/Textures/{m['texture']}.bytes", "rb").read())
        tx.image = bpy.data.images.load(p); tx.interpolation = "Closest"; mt.node_tree.links.new(tx.outputs["Color"], bs.inputs["Base Color"])
    MATS[m["name"]] = mt
def mkobj(name, visuals):
    root = bpy.data.objects.new(name, None); bpy.context.scene.collection.objects.link(root)
    for v in visuals:
        ob = bpy.data.objects.new(v["mesh"], load_mesh(v["mesh"])); ob.data.materials.append(MATS[v["mat"]])
        bpy.context.scene.collection.objects.link(ob); ob.parent = root
    return root
for o in bpy.data.objects: bpy.data.objects.remove(o)
BOB = [mkobj("b_" + b["name"], b["visuals"]) for b in SC["bodies"]]
OOB = {o["name"]: mkobj("o_" + o["name"], o["visuals"]) for o in SC["objects"]}

# ---------------- optional realistic lab (same data and the same mesh/UV rules as SonairLab.cs)
LAB = json.load(open(RES + "/lab.json")) if os.environ.get("LAB") else None
NRM = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
def lab_box(s, uvm, fit):
    V, U, I = [], [], []
    h = 0.5 * np.asarray(s, float)
    for n in NRM:
        n = np.array(n, float)
        if abs(n[1]) > 0.5:
            a, b = (np.array([1., 0, 0]), np.array([0, 0, 1.])) if s[0] >= s[2] else (np.array([0, 0, 1.]), np.array([1., 0, 0]))
        else:
            a = np.cross([0, 1, 0], -n); b = np.array([0, 1., 0])
        la, lb = abs(a @ s), abs(b @ s); c0 = n * h; o = len(V)
        for kx, ky in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            V.append(c0 + a * 0.5 * la * kx + b * 0.5 * lb * ky)
            U.append((0.5 * (kx + 1), 0.5 * (ky + 1)) if fit else (0.5 * (kx + 1) * la / max(1e-4, uvm[0]), 0.5 * (ky + 1) * lb / max(1e-4, uvm[1])))
        cr = np.cross(V[o + 1] - V[o], V[o + 2] - V[o])
        I += [[o, o + 1, o + 2], [o, o + 2, o + 3]] if cr @ n > 0 else [[o, o + 2, o + 1], [o, o + 3, o + 2]]
    return np.array(V), np.array(U), np.array(I)
def lab_cyl(s, seg=32):
    r, hh = 0.5 * s[0], 0.5 * s[1]; V, U, I = [], [], []
    for i in range(seg + 1):
        a = 2 * math.pi * i / seg; d = np.array([math.cos(a), 0, math.sin(a)])
        V += [d * r - [0, hh, 0], d * r + [0, hh, 0]]; U += [(i / seg, 0), (i / seg, 1)]
    def tri(a, b, c, nout):
        cr = np.cross(V[b] - V[a], V[c] - V[a]); I.append([a, b, c] if cr @ nout > 0 else [a, c, b])
    for i in range(seg):
        o = 2 * i; d = V[o] * [1, 0, 1]; tri(o, o + 2, o + 1, d); tri(o + 1, o + 2, o + 3, d)
    for cap in (1, -1):
        n = np.array([0, cap, 0.]); c = len(V); V.append(n * hh); U.append((0.5, 0.5))
        for i in range(seg + 1):
            a = 2 * math.pi * i / seg; d = np.array([math.cos(a), 0, math.sin(a)]); V.append(d * r + n * hh); U.append((0.5 + 0.5 * d[0], 0.5 + 0.5 * d[2]))
        for i in range(seg): tri(c, c + 1 + i, c + 2 + i, n)
    return np.array(V), np.array(U), np.array(I)
if LAB:
    LMATS = {}
    for m in LAB["materials"]:
        mt = bpy.data.materials.new("lab_" + m["name"]); mt.use_nodes = True; bs = mt.node_tree.nodes["Principled BSDF"]
        c = m["color"]; bs.inputs["Base Color"].default_value = (c[0] ** 2.2, c[1] ** 2.2, c[2] ** 2.2, 1)
        bs.inputs["Metallic"].default_value = m["metallic"]; bs.inputs["Roughness"].default_value = 1 - m["smoothness"]
        if c[3] < 1: bs.inputs["Alpha"].default_value = c[3]; bs.inputs["Transmission Weight"].default_value = 0.9
        tx = None
        if m["texture"]:
            tx = mt.node_tree.nodes.new("ShaderNodeTexImage"); p = f"/tmp/{m['texture']}.png"
            open(p, "wb").write(open(f"{RES}/Textures/{m['texture']}.bytes", "rb").read()); tx.image = bpy.data.images.load(p)
            mix = mt.node_tree.nodes.new("ShaderNodeMix"); mix.data_type = "RGBA"; mix.blend_type = "MULTIPLY"; mix.inputs[0].default_value = 1.0
            mix.inputs[6].default_value = (c[0] ** 2.2, c[1] ** 2.2, c[2] ** 2.2, 1)
            mt.node_tree.links.new(tx.outputs["Color"], mix.inputs[7]); mt.node_tree.links.new(mix.outputs[2], bs.inputs["Base Color"])
        e = m.get("emission") or []
        if e and sum(e) > 0:
            bs.inputs["Emission Color"].default_value = (e[0], e[1], e[2], 1); bs.inputs["Emission Strength"].default_value = 3.0
            if tx is not None: mt.node_tree.links.new(tx.outputs["Color"], bs.inputs["Emission Color"])
        LMATS[m["name"]] = mt
    for pr in LAB["prims"]:
        V, UV, I = lab_box(pr["size"], pr["uv"], pr["fit"]) if pr["type"] == "box" else lab_cyl(pr["size"])
        me = bpy.data.meshes.new(pr["name"]); me.from_pydata((V @ M).tolist(), [], I[:, [0, 2, 1]].tolist()); me.update()
        uvl = me.uv_layers.new()
        for li, loop in enumerate(me.loops): uvl.data[li].uv = UV[loop.vertex_index]
        ob = bpy.data.objects.new(pr["name"], me); ob.data.materials.append(LMATS[pr["mat"]]); bpy.context.scene.collection.objects.link(ob)
        ob.matrix_world = u2b(pr["rot"], pr["pos"])
    for h in LAB["hide"]:
        if h in OOB:
            for ch in OOB[h].children: ch.hide_render = True
    for k, l in enumerate(LAB["lights"]):
        ld = bpy.data.lights.new(f"ll{k}", "POINT"); ld.energy = 95.0 * l["intensity"]; ld.shadow_soft_size = 0.35
        ld.color = l["color"]; lo = bpy.data.objects.new(f"ll{k}", ld); bpy.context.scene.collection.objects.link(lo)
        lo.location = Vector((M.T @ np.asarray(l["pos"])).tolist())

sc = bpy.context.scene; sc.render.engine = "CYCLES"; sc.cycles.samples = 24; sc.cycles.device = "CPU"
sc.render.resolution_x, sc.render.resolution_y = 960, 600
w = bpy.data.worlds.new("w"); sc.world = w; w.use_nodes = True; w.node_tree.nodes["Background"].inputs[1].default_value = 0.8
sun = bpy.data.lights.new("sun", "SUN"); sun.energy = 3.0 if not LAB else 0.8; so = bpy.data.objects.new("sun", sun); sc.collection.objects.link(so)
so.rotation_euler = (0.6, 0.2, 0.8)
if LAB:
    w.node_tree.nodes["Background"].inputs[1].default_value = 0.15; sc.cycles.samples = 64
    try: sc.cycles.use_denoising = True
    except Exception: pass
cd = bpy.data.cameras.new("cam"); co = bpy.data.objects.new("cam", cd); sc.collection.objects.link(co); sc.camera = co

def place_cam_ov():
    cd.lens = 32; loc = Vector((1.25, -1.05, 0.85)); tgt = Vector((0.42, -0.02, 0.18))
    if os.environ.get("VIEW"):
        v = [float(x) for x in os.environ["VIEW"].split(",")]; loc = Vector(v[:3]); tgt = Vector(v[3:6]); cd.lens = v[6]
    co.location = loc; co.rotation_euler = (tgt - loc).to_track_quat("-Z", "Y").to_euler()
def place_cam_sensor(sname, W, obj):
    s = [x for x in SC["sensors"] if x["name"] == sname][0]
    pr, pp = (W[SC["tool0"]] if s["parent"] == "tool0" else obj[s["parent"]])
    rot = qmul(pr, s["rot"]); pos = pp + qrot(pr, s["pos"])
    # C#: Camera child of the optical frame with localRotation = LookRotation(Vector3.up, Vector3.right)
    Rl = look_rotation([0, 1, 0], [1, 0, 0]); Rw = q2R(rot) @ Rl
    f_b = M.T @ Rw[:, 2]; up_b = M.T @ Rw[:, 1]; right_b = np.cross(f_b, up_b)
    Rb = np.column_stack([right_b, up_b, -f_b]); T = np.eye(4); T[:3, :3] = Rb; T[:3, 3] = M.T @ pos
    co.matrix_world = Matrix(T.tolist()); cd.sensor_fit = "VERTICAL"; cd.angle_y = math.radians(s["vfov"]); cd.clip_start = 0.01
    sc.render.resolution_x, sc.render.resolution_y = s["width"] * 2, s["height"] * 2

for t in TIMES:
    W, obj, c = state(t)
    for b, ob in zip(SC["bodies"], BOB): ob.matrix_world = u2b(*W[SC["bodies"].index(b)])
    for n, ob in OOB.items(): ob.matrix_world = u2b(*obj[n])
    if CAM == "ov": place_cam_ov()
    else: place_cam_sensor("A_D405" if CAM == "A" else "G_D435i", W, obj)
    sc.render.filepath = f"{OUT}_{t:05.1f}.png"; bpy.ops.render.render(write_still=True)
    lab = [l for tt, l in zip(SQ["phaseT"], SQ["phaseLabel"]) if tt <= t][-1]
    print(f"RENDERED {sc.render.filepath} t={t} grip={c:.3f} phase={lab}")

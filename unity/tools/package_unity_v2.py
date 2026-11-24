#!/usr/bin/env python3
"""Write .meta files, build SONAIR_Unity_v2.unitypackage and the delivery zip (English-only package)."""
import hashlib, io, os, re, shutil, tarfile, zipfile, time

ROOT = "/home/claude/unity/SONAIR_Unity"
ASSETS = ROOT + "/Assets"
OUT = "/home/claude/unity/dist/SONAIR_Unity_v2"
CJK = re.compile("[\u4e00-\u9fff]")


def guid(rel): return hashlib.md5(("SONAIR-unity-v1/" + rel).encode()).hexdigest()   # same seed as v1: importing v2 updates v1 files in place


def meta_text(rel, is_dir):
    g = guid(rel)
    if is_dir:
        return f"fileFormatVersion: 2\nguid: {g}\nfolderAsset: yes\nDefaultImporter:\n  externalObjects: {{}}\n  userData: \n  assetBundleName: \n  assetBundleVariant: \n"
    if rel.endswith(".cs"):
        return (f"fileFormatVersion: 2\nguid: {g}\nMonoImporter:\n  externalObjects: {{}}\n  serializedVersion: 2\n  defaultReferences: []\n"
                f"  executionOrder: 0\n  icon: {{instanceID: 0}}\n  userData: \n  assetBundleName: \n  assetBundleVariant: \n")
    if rel.endswith((".json", ".bytes", ".txt", ".csv")):
        return f"fileFormatVersion: 2\nguid: {g}\nTextScriptImporter:\n  externalObjects: {{}}\n  userData: \n  assetBundleName: \n  assetBundleVariant: \n"
    return f"fileFormatVersion: 2\nguid: {g}\nDefaultImporter:\n  externalObjects: {{}}\n  userData: \n  assetBundleName: \n  assetBundleVariant: \n"


entries = []
for dirpath, dirnames, filenames in os.walk(ASSETS + "/SONAIR"):
    dirnames.sort()
    rel_dir = os.path.relpath(dirpath, ROOT).replace(os.sep, "/")
    entries.append((rel_dir, dirpath, True))
    for f in sorted(filenames):
        if f.endswith(".meta"): continue
        entries.append((rel_dir + "/" + f, os.path.join(dirpath, f), False))
for rel, ab, is_dir in entries:
    open(ab.rstrip("/") + ".meta", "w").write(meta_text(rel, is_dir))
    if not is_dir and rel.endswith((".cs", ".json", ".csv", ".txt", ".py")):
        assert not CJK.search(open(ab, encoding="utf-8").read()), f"non-English text in {rel}"
for p in [ROOT + "/Tools/sonair_live_bridge.py"] + [os.path.join(ROOT, "SONAIR_Trajectories_CSV", f) for f in os.listdir(ROOT + "/SONAIR_Trajectories_CSV")]:
    assert not CJK.search(open(p, encoding="utf-8").read()), p
print("meta files:", len(entries))

if os.path.isdir(OUT): shutil.rmtree(OUT)
os.makedirs(OUT)
pkg = OUT + "/SONAIR_Unity_v2.unitypackage"


def add_bytes(tar, name, data):
    ti = tarfile.TarInfo(name); ti.size = len(data); ti.mtime = int(time.time()); ti.mode = 0o644
    tar.addfile(ti, io.BytesIO(data))


with tarfile.open(pkg, "w:gz") as tar:
    for rel, ab, is_dir in entries:
        g = guid(rel)
        ti = tarfile.TarInfo(g); ti.type = tarfile.DIRTYPE; ti.mode = 0o755; ti.mtime = int(time.time()); tar.addfile(ti)
        add_bytes(tar, g + "/pathname", rel.encode("utf-8"))
        add_bytes(tar, g + "/asset.meta", meta_text(rel, is_dir).encode("utf-8"))
        if not is_dir: add_bytes(tar, g + "/asset", open(ab, "rb").read())
print("unitypackage:", os.path.getsize(pkg) // 1024, "KB")

shutil.copytree(ROOT, OUT + "/SONAIR_Unity", ignore=shutil.ignore_patterns("Library", "Temp", "Logs", "obj"))
os.makedirs(OUT + "/Source")
for f in ("make_lab.py", "verify_blender.py", "package_unity_v2.py"): shutil.copy("/home/claude/unity/" + f, OUT + "/Source/")
os.makedirs(OUT + "/Previews")
for src, dst in (("lab_room.png", "lab_room_operator_corner.png"), ("lab_cell.png", "lab_cell_default_view.png"), ("lab_fence.png", "lab_cell_from_the_windows.png")):
    shutil.copy("/home/claude/unity/vr/" + src, OUT + "/Previews/" + dst)
open(OUT + "/README.txt", "w", encoding="utf-8").write("""SONAIR cell twin for Unity, v2.0 (English)

What it is
  A viewer and authoring tool for the SONAIR two-mode inspection cell (UR5e, 2F-85 gripper, camera tool, specimens),
  inside a realistic lab. Unity does not command the robot and is not the reference simulator of the benchmark;
  the dynamics reference is MuJoCo (sonair-twin). Unity is used to author and check sequences, to mirror the real arm
  live (with the commanded arm as a ghost), to render camera views with exact poses, and for XR.

Quick start
  1. Unity Hub > Add > Add project from disk > SONAIR_Unity (Unity 2021.3 LTS or newer, or Unity 6).
  2. On first open the demo scene Assets/SONAIR/SONAIR_Demo.unity is created and opened.
     If not: menu SONAIR > 1 Create and Open Demo Scene. This also sets the colour space to Linear.
  3. Press Play. Keys: Space play/pause, 1..6 sequences, L lab on/off, G ghost, F frames, P camera insets,
     C save camera images with poses, V cycle views, Tab hide the panel.
  Existing project: Assets > Import Package > Custom Package > SONAIR_Unity_v2.unitypackage, then SONAIR > 1.

Key features (everything else was removed)
  Sequence  the six pre-computed sequences: Mode A and Mode B on S2 and S4, two hand-eye calibrations
  Manual    joint and gripper sliders, key poses, object in the hand
  Live      mirrors the real arm from UDP. Packet: q1..q6,closure,t1..t6 (rad); t = RTDE target_q, drawn as a
            translucent ghost, with the tracking error in degrees and millimetres. "#text" lines are shown as status.
            Senders: the Jetson recorder (python -m sonair jetson-record ... --unity HOST:5005) or
            SONAIR_Unity/Tools/sonair_live_bridge.py (--demo SONAIR_Trajectories_CSV/B_S2.csv works without a robot).
  Cameras   D405 and D435i views, saved as PNG + JSON (optical frame and tool0 poses in the robot base frame B,
            pinhole intrinsics, joint state)
  Lab       room, epoxy floor with the cell's hazard marking, guard fence, controller cabinet, e-stop, benches,
            whiteboard, cabinets, operator desk, LED ceiling panels. Built at Play from Resources/SONAIR/lab.json.

Editing the lab
  Source/make_lab.py writes lab.json and the procedural textures. Source/verify_blender.py (LAB=1) renders the same data
  in Blender; the images in Previews/ were made that way.

Frames
  Robot base frame B (UR controller base, metres, z up) to Unity: x_B = z_u, y_B = -x_u, z_B = y_u.
""")
zp = os.path.dirname(OUT) + "/SONAIR_Unity_v2.zip"
if os.path.exists(zp): os.remove(zp)
with zipfile.ZipFile(zp, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as z:
    for dirpath, dirnames, filenames in os.walk(OUT):
        for f in filenames:
            ab = os.path.join(dirpath, f); z.write(ab, os.path.relpath(ab, os.path.dirname(OUT)))
print("zip:", zp, os.path.getsize(zp) // 1024, "KB")

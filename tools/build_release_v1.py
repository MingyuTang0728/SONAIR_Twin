"""Build data/benchmark_v1/ (the public part of the benchmark-v1 real data) from the cell platform folder.

    python tools/build_release_v1.py --src "<SONAIRCrossite33 folder>" [--out data/benchmark_v1]

What goes in, and what does not:
  train/<run_id>.jsonl.gz         every run of the 36 PUBLISHED cells, the original file byte for byte, gzip
  test_inputs/<run_id>.cmd.jsonl.gz   the 18 HELD-OUT cells as inputs only: per row t, controller time,
                                  target_q, target_qd, target_moment (the task's cmd channels). Measured joints,
                                  currents, tool pose, IMU and camera channels are withheld: they are the hidden
                                  targets of the benchmark and must never be in a public repository.
  index.csv                       one row per run: split, session, cell, rows, duration, SHA-256 of the original
                                  file and of the released file
  calib/ campaign/ site/ cards/ embodiments/ carrier.json site.json   platform metadata (paths to raw logs removed)
  analysis/                       per-run analysis tables, published cells only
  release.json                    counts, sessions, what is withheld and why

Not copied at all: raw ur_logs/ and imu_logs/ (they cover held-out periods), videos, API keys (sonair_keys.json).
"""
from __future__ import annotations

import argparse
import csv
import glob
import gzip
import hashlib
import io
import json
import os
import re
import shutil
import time

# keyed by the "campaign session N" in each manifest's notes (the platform's own numbering)
SESSIONS = {"0": ("benchmark-v1", "campaign session 0: the benchmark-v1 runs scored on the benchmark site, repeats r00 and r03"),
            "1": ("trial-1", "campaign session 1: trial session, repeats r01 and r04"),
            "2": ("trial-2", "campaign session 2: trial session, repeat r02")}
INPUT_KEYS = ("t", "target_q", "target_qd", "target_moment")


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def gz_write(path, text_lines=None, data=None):
    """Deterministic gzip (mtime 0) so a rebuild gives byte-identical files."""
    with open(path, "wb") as raw:
        with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0, compresslevel=9) as g:
            if data is not None:
                g.write(data)
            for line in text_lines or ():
                g.write(line.encode("utf-8"))


def scrub_paths(obj):
    """Drop absolute paths to raw logs on the lab PC from metadata files."""
    if isinstance(obj, dict):
        return {k: scrub_paths(v) for k, v in obj.items() if not (isinstance(v, str) and re.match(r"^[A-Za-z]:\\\\|^[A-Za-z]:\\|^/(Users|home|Volumes)/", v))}
    if isinstance(obj, list):
        return [scrub_paths(v) for v in obj]
    return obj


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--src", required=True)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data", "benchmark_v1"))
    a = ap.parse_args(argv)
    src, out = a.src, a.out
    readme = open(os.path.join(out, "README.md"), encoding="utf-8").read() if os.path.exists(os.path.join(out, "README.md")) else None
    if os.path.exists(out):
        shutil.rmtree(out)
    for d in ("train", "test_inputs"):
        os.makedirs(os.path.join(out, d))

    index = []
    for p in sorted(glob.glob(os.path.join(src, "bench_runs", "*.jsonl"))):
        rid = os.path.basename(p)[:-6]
        with open(p, encoding="utf-8") as f:
            head = f.readline()
            man = json.loads(head)["_manifest"]
            rows = [l for l in f if l.strip()]
        notes = man.get("notes") or ""
        held = "HELD-OUT" in notes
        m = re.search(r"campaign session (\d+)", notes)
        ses, _ = SESSIONS.get(m.group(1) if m else "", ("unknown", ""))
        t0 = json.loads(rows[0])["t"]; t1 = json.loads(rows[-1])["t"]
        if not held:
            dst = os.path.join(out, "train", rid + ".jsonl.gz")
            gz_write(dst, data=open(p, "rb").read())       # the original bytes, unchanged
        else:
            dst = os.path.join(out, "test_inputs", rid + ".cmd.jsonl.gz")
            inp = {k: v for k, v in man.items()}
            inp["withheld"] = "measured channels (q, qd, tcp, currents, IMU, cameras) are the hidden targets"
            lines = [json.dumps({"_input": inp}) + "\n"]
            for r in rows:
                s = json.loads(r)
                o = {k: s.get(k) for k in INPUT_KEYS}
                o["controller_t"] = (s.get("aux") or {}).get("controller_t")
                lines.append(json.dumps(o, separators=(",", ":")) + "\n")
            gz_write(dst, lines)
        index.append(dict(run_id=rid, split="test" if held else "train", session=ses, repeat_idx=man.get("repeat_idx"),
                          joint_vel=man.get("joint_vel"), arm_config=man.get("arm_config"), traj_type=man.get("traj_type"),
                          cell=f"{man.get('joint_vel'):.3f}|{man.get('arm_config')}|{man.get('traj_type')}", rows=len(rows),
                          duration_s=round(t1 - t0, 3), rate_hz=man.get("sample_rate_hz"), started_utc=man.get("started_utc"),
                          carrier_id=man.get("carrier_id"), file=os.path.relpath(dst, out).replace(os.sep, "/"),
                          sha256_original=sha(p), sha256_released=sha(dst)))

    with open(os.path.join(out, "index.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(index[0])); w.writeheader(); w.writerows(index)

    # platform metadata
    def cp_json(rel, dst_rel=None):
        s = os.path.join(src, rel)
        if os.path.exists(s):
            d = os.path.join(out, dst_rel or rel); os.makedirs(os.path.dirname(d), exist_ok=True)
            json.dump(scrub_paths(json.load(open(s, encoding="utf-8"))), open(d, "w"), indent=1)
    for rel in ("calib/imu_cal.json", "calib/quat_convention.json", "calib/sessions/session_1.json", "calib/sessions/session_2.json",
                "calib/sessions/session_3.json", "campaign/state.json", "site/leaderboard.json", "site/status.json",
                "embodiments/ur5e.json", "carrier.json", "site.json"):
        cp_json(rel)
    cp_json("accel_cal.json", "calib/accel_cal.json")
    for c in glob.glob(os.path.join(src, "cards", "*.md")):            # house style: no em dash in any text
        os.makedirs(os.path.join(out, "cards"), exist_ok=True)
        txt = open(c, encoding="utf-8").read().replace(" \u2014 ", " - ").replace("\u2014", " - ")
        open(os.path.join(out, "cards", os.path.basename(c)), "w", encoding="utf-8").write(txt)

    # analysis tables: published cells only
    pub = {r["run_id"] for r in index if r["split"] == "train"}
    for t in glob.glob(os.path.join(src, "analysis", "benchmark_v1_S1", "*.csv")):
        rows = list(csv.DictReader(open(t, encoding="utf-8")))
        if not rows:
            continue
        key = next((k for k in rows[0] if k in ("run_id", "run", "id")), None)
        keep = [r for r in rows if key is None or r[key] in pub]
        if key is None:
            continue                                   # a table we cannot filter by run is left out
        os.makedirs(os.path.join(out, "analysis"), exist_ok=True)
        with open(os.path.join(out, "analysis", os.path.basename(t)), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(keep)

    n_tr = sum(r["split"] == "train" for r in index); n_te = len(index) - n_tr
    cells_tr = sorted({r["cell"] for r in index if r["split"] == "train"}); cells_te = sorted({r["cell"] for r in index if r["split"] == "test"})
    by_ses = {}
    for r in index:
        b = by_ses.setdefault(r["session"], {"train": 0, "test": 0, "first_utc": r["started_utc"], "last_utc": r["started_utc"]})
        b[r["split"]] += 1
        b["first_utc"] = min(b["first_utc"], r["started_utc"]); b["last_utc"] = max(b["last_utc"], r["started_utc"])
    rel = dict(name="SONAIR benchmark-v1 real runs (public part)", schema="sonair-run-1", built_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
               source="cell platform folder SONAIRCrossite33/bench_runs (originals unchanged; SHA-256 of each in index.csv)",
               robot="UR5e", site="UoN", rate_hz=125.0, runs=len(index), train_runs=n_tr, test_input_runs=n_te,
               published_cells=len(cells_tr), held_out_cells=len(cells_te), held_out_cell_list=cells_te,
               sessions={k: dict(label=v[1], runs=by_ses.get(v[0], {})) for k, v in ((s[0], s) for s in SESSIONS.values())},
               benchmark_v1_official="session benchmark-v1 only (108 runs: 72 published, 36 held-out), as on the benchmark site; trial-1 and trial-2 are kept for repeatability studies",
               withheld=["measured channels of held-out runs (hidden targets)", "raw ur_logs and imu_logs (cover held-out periods)",
                         "camera data", "API keys and submission secrets"],
               test_input_channels=list(INPUT_KEYS) + ["controller_t"])
    json.dump(rel, open(os.path.join(out, "release.json"), "w"), indent=1)
    if readme:                                          # the hand-written README survives a rebuild
        open(os.path.join(out, "README.md"), "w", encoding="utf-8").write(readme)
    size = sum(os.path.getsize(os.path.join(dp, f)) for dp, _, fs in os.walk(out) for f in fs)
    print(f"{len(index)} runs: {n_tr} train ({len(cells_tr)} cells), {n_te} test inputs ({len(cells_te)} cells); {size / 1e6:.1f} MB in {out}")


if __name__ == "__main__":
    main()

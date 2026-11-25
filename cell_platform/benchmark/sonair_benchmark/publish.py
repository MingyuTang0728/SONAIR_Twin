"""
publish -- from a finished session to the benchmark site, in one step.

After a campaign session the cell has new accepted runs. Getting them onto
the benchmark page used to be four commands typed in the right order with the
right folders: replay S0, measure the gap, score the leaderboard, copy the
site. This does exactly those, the same way the CLI does, and nothing else:

  1. ACCEPTED RUNS ONLY. A run counts when the campaign read it back and
     marked it done (campaign/state.json). Rehearsals, superseded files,
     rejected runs, E1 training runs and user intake data are left out.
  2. S0, INCREMENTALLY. Each accepted run is replayed through the reference
     simulation once, into results/sim_s0/, with the IMU calibration and the
     pendant's tool offset the scored replay uses. A run already replayed
     (and not changed since) is not replayed again.
  3. THE PRIVATE NUMBERS stay in results/: the full gap over every cell.
  4. THE PUBLIC SITE is built in site_build/: benchmark.html, the leaderboard,
     and a gap file with the HELD-OUT CELLS REMOVED. A held-out cell's raw S0
     error is information about real data teams are not meant to see, so it
     never leaves this PC. The leaderboard carries only GCR, as designed.
  5. EVERY SITE. Runs imported from another cell (sites/<SITE>/, written by
     `site-import`) are published with the home site's, each with its own
     site's calibrations; the task package puts every run of a site that is
     not the training site in the cross-site test split.
  6. THE TOOL CAMERA. Where a run has vision_logs/<run>.vtp.jsonl, the tool
     position the camera saw becomes that run's vision target (latency and
     floor measured per site; the gate says whether it ranks).
  7. PUSHED only where publish.json says, and only there:
         {"target": "folder", "folder": "\\\\server\\share\\sonair-site"}
         {"target": "git", "repo": "D:/sites/sonair-site", "branch": "gh-pages"}
     With no publish.json, or "target": "none", the site is built and left
     on this PC. Nothing is ever sent anywhere unconfigured.

    python -m sonair_benchmark publish            # build (and push if configured)
    python -m sonair_benchmark publish --dry-run  # say what would happen
"""
from __future__ import annotations

import json
import shutil

import numpy as np
import subprocess
import sys
import time
from pathlib import Path

from .campaign import plan_campaign, split_cells
from .metrics import aggregate_by_cell, gap_between, gate_c
from .schema import read_dataset, pair_runs, unpaired
from .scoring import (baseline_constant_offset, score_submission,
                      submission_from_sim_runs, write_leaderboard)

CONFIG = Path("publish.json")
SIM_DIR = Path("results") / "sim_s0"
S0_FORMAT = "s0-2-tau"
BUILD = Path("site_build")
STATUS = Path("results") / "publish_status.json"


def load_config(path=CONFIG) -> dict:
    try:
        cfg = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        cfg = {}
    cfg.setdefault("target", "none")
    cfg.setdefault("auto_after_session", False)
    return cfg


def save_config(cfg: dict, path=CONFIG) -> dict:
    allowed = {"target", "folder", "repo", "branch", "auto_after_session",
               "tcp_offset"}
    clean = {k: v for k, v in cfg.items() if k in allowed}
    if clean.get("target") not in ("none", "folder", "git"):
        raise ValueError("target must be none, folder or git")
    Path(path).write_text(json.dumps(clean, indent=2), encoding="utf-8")
    return load_config(path)


def _accepted(runs_dir: Path, state_path: Path) -> tuple[list, list]:
    """(accepted E2 real runs, reasons for everything left out)."""
    st = {}
    try:
        st = json.loads(Path(state_path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        pass
    done = set(st.get("done", {}))
    keep, why = [], []
    for r in read_dataset(runs_dir, side="real"):
        m = r.manifest
        rid = m.run_id
        if str(m.notes).startswith("SIMULATED CELL"):
            continue
        if m.experiment != "E2":
            why.append(f"{rid}: {m.experiment}, not a scored run")
            continue
        if rid not in done:
            why.append(f"{rid}: not marked done by the campaign")
            continue
        keep.append(r)
    return keep, why


def session_cals(root: Path, state: dict):
    """
    run_id -> the IMU calibration its session was recorded under.

    A carrier refit moves the IMU (about 1 deg on the pilot campaign), so one
    calibration file cannot serve every session. In order of preference: the
    snapshot the session took when it started; calib/sessions/session_<n>.json;
    the current calib/imu_cal.json.
    """
    try:
        import imu_align
    except Exception:       # noqa: BLE001
        return lambda rid: None
    current = imu_align.load(root / "calib" / "imu_cal.json")
    by_sess = {}
    for k, sess in (state.get("sessions") or {}).items():
        snap = sess.get("imu_cal")
        f = root / "calib" / "sessions" / f"session_{int(k) + 1}.json"
        if snap and snap.get("R_flange_imu"):
            by_sess[int(k)] = {**snap, "ok": True}
        elif f.exists():
            by_sess[int(k)] = imu_align.load(f)
    done = state.get("done") or {}

    def cal_for(rid):
        s = (done.get(rid) or {}).get("session")
        if s is not None and by_sess.get(int(s)):
            return by_sess[int(s)]
        return current
    return cal_for


def _replay_missing(runs, sim_dir: Path, log, tcp_offset=None,
                    runs_dir: Path | None = None, cal_for=lambda rid: None) -> dict:
    """S0 for every accepted run that has none yet, or an out-of-date one."""
    root = Path(__file__).resolve().parent.parent
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    import sim_mujoco
    ok, why = sim_mujoco.available()
    if not ok:
        return {"ok": False, "error": f"MuJoCo is not available: {why}"}
    import twin
    men = next((d for d in twin.menagerie_dirs()
                if (d / sim_mujoco.MODEL_DIR / "scene.xml").exists()), None)
    if men is None:
        return {"ok": False, "error": "the MuJoCo model library is not "
                "installed; run install_sim.py once"}
    sim_dir.mkdir(parents=True, exist_ok=True)
    index_p = sim_dir / "_index.json"
    try:
        index = json.loads(index_p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        index = {}
    made = failed = 0
    errors = []
    for r in runs:
        rid = r.manifest.run_id
        cal = cal_for(rid)
        # the calibration AND the replay format: a cache made before the
        # simulated torques were written is out of date for the effort track
        cal_id = ((cal or {}).get("made_at") or "none") + "|" + S0_FORMAT
        src = (runs_dir / f"{rid}.jsonl") if runs_dir else Path("")
        dst = sim_dir / f"{rid}.jsonl"
        if dst.exists() and (not src.exists()
                             or dst.stat().st_mtime >= src.stat().st_mtime) \
                and index.get(rid, "none") == cal_id:
            continue
        res = sim_mujoco.replay(r, sim_dir, men, imu_cal=cal,
                                tcp_offset=tcp_offset)
        if res.get("ok"):
            index[rid] = cal_id
        if res.get("ok"):
            made += 1
        else:
            failed += 1
            errors.append(f"{rid}: {res.get('error')}")
    index_p.write_text(json.dumps(index, indent=1), encoding="utf-8")
    if made or failed:
        log(f"S0 replayed for {made} new run(s)"
            + (f"; {failed} failed" if failed else ""))
    return {"ok": not failed, "made": made, "failed": failed, "errors": errors[:10]}


K0_DIR = Path("results") / "sim_k0"


def _kinematic_baseline(runs, out_dir: Path, runs_dir: Path, log) -> dict:
    """
    K0: the commanded joints put straight onto the model, no dynamics at all.

    It is on the ladder because it is the first thing anyone would try and
    because of what it scores. A UR's own controller tracks its setpoint to a
    few hundredths of a degree, so the tool-position gap measured against the
    COMMANDED joints is almost entirely the reference simulator's servo, not
    physics. If K0 closes nearly all of it, the position channel alone is not
    a benchmark, and the board says so in public rather than letting an entry
    win by doing nothing.
    """
    root = Path(__file__).resolve().parent.parent
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    try:
        import sim_mujoco
        import twin
        from .schema import RunManifest, RunWriter, Sample
    except Exception as e:                          # noqa: BLE001
        return {"ok": False, "error": str(e)}
    if not sim_mujoco.available()[0]:
        return {"ok": False, "error": "MuJoCo is not available"}
    men = next((d for d in twin.menagerie_dirs()
                if (d / sim_mujoco.MODEL_DIR / "scene.xml").exists()), None)
    if men is None:
        return {"ok": False, "error": "no MuJoCo model library"}
    out_dir.mkdir(parents=True, exist_ok=True)
    arm = None
    made = 0
    for r in runs:
        m = r.manifest
        dst = out_dir / f"{m.run_id}.jsonl"
        src = runs_dir / f"{m.run_id}.jsonl"
        if dst.exists() and (not src.exists() or dst.stat().st_mtime >= src.stat().st_mtime):
            continue
        if arm is None:
            arm = sim_mujoco.Arm(sim_mujoco.ensure_model(men), m.carrier_mass_kg)
        man = RunManifest(**{**m.__dict__, "side": "sim",
                             "notes": "K0 rigid kinematics: joints = commanded "
                                      f"target_q; replayed from {m.run_id}"})
        with RunWriter(dst, man) as w:
            for smp in r.samples:
                q = smp.get("target_q")
                if not q:
                    continue
                for i, a in enumerate(arm.qadr):
                    arm.d.qpos[a] = float(q[i])
                pos = arm.tcp_position()
                w.write(Sample(t=smp["t"], q=list(q), target_q=list(q),
                               tcp_pos=pos,
                               tcp_rot=sim_mujoco._quat_to_rotvec(arm.read("tcp_quat"))))
        made += 1
    if made:
        log(f"K0 kinematic baseline written for {made} new run(s)")
    return {"ok": True, "made": made}


def _charts(pairs, reports, held, entries, sessions_of, k0_dir: Path) -> dict:
    """
    The numbers behind the site's pictures, PUBLISHED CELLS ONLY.

    Everything here is drawn from runs whose real data is public anyway, so a
    chart can never show a held-out cell's real behaviour by the back door.
    """
    import math
    import statistics as stt
    pub = [(p, rep) for p, rep in zip(pairs, reports)
           if p[0].manifest.cell_key() not in held]
    if not pub:
        return {}

    def q(xs, f):
        xs = sorted(xs)
        if not xs:
            return None
        k = (len(xs) - 1) * f
        lo, hi = math.floor(k), math.ceil(k)
        return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)

    by_v, by_cv, by_s = {}, {}, {}
    errs = []
    secs = 0.0
    for (real, sim), rp in pub:
        m = real.manifest
        v = round(float(m.joint_vel), 2)
        by_v.setdefault(v, []).append(rp)
        by_cv.setdefault((m.arm_config, v), []).append(rp)
        by_s.setdefault(sessions_of(m.run_id), []).append(rp)
        n = min(len(real.samples), len(sim.samples))
        for a, b in zip(real.samples[:n:4], sim.samples[:n:4]):
            if "tcp_pos" in a and "tcp_pos" in b:
                errs.append(1000.0 * math.dist(a["tcp_pos"], b["tcp_pos"]))
        if real.samples:
            secs += float(real.samples[-1]["t"]) - float(real.samples[0]["t"])

    def summ(rs):
        return {"median_mm": round(stt.median(r.pos_median_mm for r in rs), 2),
                "p95_mm": round(stt.mean(r.pos_p95_mm for r in rs), 2), "runs": len(rs)}

    hi = q(errs, 0.995) or 1.0
    width = max(1.0, round(hi / 40.0))
    bins = [0] * (int(hi // width) + 1)
    for e in errs:
        bins[min(len(bins) - 1, int(e // width))] += 1

    # one run to look at: the fastest published point-to-point move
    ex = sorted(pub, key=lambda x: (x[0][0].manifest.traj_type != "point_to_point",
                                    -x[0][0].manifest.joint_vel, x[0][0].manifest.run_id))[0]
    (real, sim), _ = ex
    k0 = {}
    kp = k0_dir / f"{real.manifest.run_id}.jsonl"
    if kp.exists():
        from .schema import read_run
        k0 = {round(float(x["t"]), 4): x["tcp_pos"] for x in read_run(kp).samples}
    n = min(len(real.samples), len(sim.samples))
    step = max(1, n // 300)
    t0 = float(real.samples[0]["t"])
    trace = []
    for a, b in zip(real.samples[:n:step], sim.samples[:n:step]):
        row = [round(float(a["t"]) - t0, 3),
               round(math.degrees(a["q"][2]), 2), round(math.degrees(b["q"][2]), 2),
               round(1000.0 * math.dist(a["tcp_pos"], b["tcp_pos"]), 2)]
        kpos = k0.get(round(float(a["t"]), 4))
        row.append(round(1000.0 * math.dist(a["tcp_pos"], kpos), 2) if kpos else None)
        trace.append(row)

    return {
        "note": "published condition cells only; held-out cells are never drawn",
        # which cells are held out is public (their commands are released);
        # the site uses it to keep live-preview points of those cells off its charts
        "held_out_cells": sorted(held),
        "kpi": {"runs": len(pub), "cells": len({p[0].manifest.cell_key() for p, _ in pub}),
                "minutes_of_motion": round(secs / 60.0, 1),
                "samples": sum(len(p[0].samples) for p, _ in pub),
                "median_gap_mm": round(stt.median(errs), 1) if errs else None,
                "p95_gap_mm": round(q(errs, 0.95), 1) if errs else None},
        "by_speed": [{"v": v, **summ(rs)} for v, rs in sorted(by_v.items())],
        "by_config_speed": [{"config": c, "v": v, **summ(rs)}
                            for (c, v), rs in sorted(by_cv.items())],
        "by_session": [{"session": s_, **summ(rs)} for s_, rs in sorted(by_s.items())
                       if s_ is not None],
        "histogram": {"bin_mm": width, "counts": bins},
        "example": {"run_id": real.manifest.run_id, "cell": real.manifest.cell_key(),
                    "columns": ["t_s", "real_elbow_deg", "s0_elbow_deg",
                                "s0_gap_mm", "k0_gap_mm"],
                    "rows": trace},
        "baselines": [{"name": e.name, "track": e.track, "gcr_p95": round(e.gcr_p95, 3),
                       "gcr_median": round(e.gcr_median, 3)} for e in entries],
    }


def _public_gap(doc: dict, held: set) -> dict:
    """The gap file with every held-out cell taken out."""
    pub = dict(doc)
    pub["cell_map"] = {k: v for k, v in doc.get("cell_map", {}).items()
                       if k not in held}
    pub["runs"] = [r for r in doc.get("runs", [])
                   if r.get("cell_key") not in held]
    if pub["runs"]:
        pub["overall_pos_median_mm"] = (sum(r["pos_median_mm"] for r in pub["runs"])
                                        / len(pub["runs"]))
    pub["note"] = ("held-out cells are not in this file: their raw gap is "
                   "information about real data the benchmark keeps private")
    pub["n_pairs"] = len(pub["runs"])
    return pub


TASK_DIR = Path("results") / "task_v2"


def _zip(src: Path, dst: Path, only=None) -> int:
    import zipfile
    dst.parent.mkdir(parents=True, exist_ok=True)
    n = 0
    with zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for p in sorted(src.rglob("*")):
            if p.is_file() and (only is None or only(p)):
                z.write(p, Path(src.name) / p.relative_to(src))
                n += 1
    return n


def _vision_targets(root: Path, pairs, site_dir: dict, log) -> dict:
    """
    The vision target of every run the tool camera tracked: latency measured
    per site (the median over its runs), then each run's seen positions.
    """
    from . import vision as vz
    from .sites import site_of
    paths = {}
    for real, _sim in pairs:
        rid = real.manifest.run_id
        s_ = site_of(rid)
        base = site_dir[s_] if s_ in site_dir else root
        f = base / "vision_logs" / f"{rid}.vtp.jsonl"
        if f.exists():
            paths[rid] = (real, f)
    if not paths:
        return {"by_run": {}, "gate": None}
    first = {}
    for rid, (real, f) in paths.items():
        try:
            first[rid] = vz.run_vision(real, f)
        except Exception as e:                      # noqa: BLE001
            log(f"vision: {rid} skipped ({type(e).__name__}: {e})")
    lat = {}
    for rid, r in first.items():
        lat.setdefault(site_of(rid) or "home", []).append(r)
    lat = {k: vz.session_latency(v) for k, v in lat.items()}
    out = {}
    for rid, (real, f) in paths.items():
        if rid not in first:
            continue
        r = vz.run_vision(real, f, latency_s=lat[site_of(rid) or "home"])
        if r.get("ok"):
            out[rid] = r
    gate = vz.gate(list(out.values()))
    gate["latency_s"] = {k: round(v, 4) for k, v in lat.items()}
    log(f"tool camera: {len(out)} runs tracked; gate {gate['verdict']}"
        + (f" (beyond encoders {gate.get('beyond_encoders_p95_mm')} mm, floor "
           f"{gate.get('floor_p95_mm')} mm)" if gate.get("floor_p95_mm") else ""))
    return {"by_run": out, "gate": gate}


def _tcp_offsets(root: Path, pairs, held, home_site: dict, site_dir: dict, log):
    """
    The flange-to-tool transform every run's input carries (for the vision
    track's K0). Configured where the site says it (site.json "tcp_offset",
    the pendant's TCP), otherwise fitted on the home site's TRAIN runs only --
    never on a test run, whose own encoders would leak its target.
    """
    from . import vision as vz
    from .sites import site_of
    import json as _json

    def configured(base: Path):
        try:
            return vz.tcp_offset_from_config(_json.loads((base / "site.json").read_text(
                encoding="utf-8")).get("tcp_offset"))
        except (OSError, ValueError):
            return None
    home = configured(root)
    if home is None:
        train = [r for r, _s in pairs if not site_of(r.manifest.run_id)
                 and r.manifest.cell_key() not in held][:24]
        try:
            home = vz.tcp_offset(train) if train else None
        except Exception as e:                      # noqa: BLE001
            log(f"tool offset not fitted: {e}")
            home = None
    per_site = {name: configured(d) for name, d in site_dir.items()}

    def tcp_for(real):
        s_ = site_of(real.manifest.run_id)
        return per_site.get(s_) if per_site.get(s_) is not None else home
    return tcp_for


def run_task_board(root: Path, pairs, held, lb, cal_for, build: Path, log=print, *,
                   vision_for=None, tcp_offset_for=None, train_sites=None,
                   vision_gate=None) -> dict:
    """Package, baselines, evaluation, scorecards, downloads: the benchmark of record."""
    from . import baselines as bl, evaluate as ev, task as tk
    emap = ((lb.get("channels") or {}).get("effort") or {}).get("map") or {}
    pub, priv = root / TASK_DIR / "public", root / TASK_DIR / "private"
    pkg = tk.build_package(pairs, held, pub, priv, effort_map=emap, cal_for=cal_for,
                           vision_for=vision_for, tcp_offset_for=tcp_offset_for,
                           train_sites=train_sites, log=log)
    subs = bl.write_submissions(pub, root / TASK_DIR / "baselines", log=log)
    rows = []
    for key, d in subs.items():
        card = ev.evaluate(d, pub, priv)
        ev.write_scorecard(card, Path(d) / "card")
        if not card.get("ok"):
            log(f"baseline {key} failed: {card.get('problems')}")
            continue
        dst = build / "site" / "scorecards" / f"{key}.html"
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(Path(d) / "card" / "scorecard.html", dst)
        t = card["tracks"]
        cs = (card.get("cross_site") or {}).get("tracks") or {}
        rows.append({"key": key, **card["entry"], "headline": card["headline"]["value"],
                     "effort_tail": t["current"]["tail"]["gcr_p95"],
                     "effort_all": t["current"]["all"]["gcr_p95"],
                     "gyro_tail": t["gyro"]["tail"]["gcr_p95"],
                     "gyro_all": t["gyro"]["all"]["gcr_p95"],
                     "vis_tail": (t.get("vis") or {}).get("tail", {}).get("gcr_p95"),
                     "vis_all": (t.get("vis") or {}).get("all", {}).get("gcr_p95"),
                     "cross_effort_tail": (cs.get("current") or {}).get("tail", {}).get("gcr_p95"),
                     "cross_gyro_tail": (cs.get("gyro") or {}).get("tail", {}).get("gcr_p95"),
                     "robust_drop": t["current"]["robustness"]["drop"],
                     "calibration": (t["current"].get("calibration") or {}).get("coverage_90"),
                     "scorecard": f"site/scorecards/{key}.html", "runs": card["runs_scored"],
                     "cross_runs": (card.get("cross_site") or {}).get("runs", 0)})
    rows.sort(key=lambda r: (r["headline"] is None, -(r["headline"] or 0)))
    # downloads: the whole public package, and a small examples pack
    dl = build / "downloads"
    _zip(pub, dl / "sonair_task_v2_public.zip")
    ex_ids = sorted(p.name.split(".")[0] for p in (pub / "train").glob("*.input.jsonl"))[::12][:6]
    _zip(pub, dl / "sonair_task_v2_examples.zip",
         only=lambda p: p.parent.name != "train" and p.parent.name != "test"
         or p.name.split(".")[0] in ex_ids)
    starter = Path(__file__).resolve().parent.parent / "starter_kit"
    if starter.exists():
        _zip(starter, dl / "sonair_starter_kit.zip")
    spec = json.loads((pub / "task.json").read_text(encoding="utf-8"))
    spec.pop("index", None)
    log(f"benchmark board: {len(rows)} entries scored; headline best "
        f"{rows[0]['headline']:+.3f} ({rows[0]['key']})" if rows else "benchmark board: empty")
    return {"task": spec, "entries": rows, "examples": ex_ids,
            "package": {k: pkg.get(k) for k in ("train", "test", "cross_site", "vision_runs")},
            "vision": vision_gate,
            "downloads": {"public": "downloads/sonair_task_v2_public.zip",
                          "examples": "downloads/sonair_task_v2_examples.zip",
                          "starter": "downloads/sonair_starter_kit.zip"}}


def public_live(doc: dict, held: set | None = None) -> dict:
    """The live score with held-out cells removed: safe to put on the site."""
    if held is None:
        held = split_cells(plan_campaign())[1]
    out = _public_live(doc, held)
    # the trace of the last minute carries no cell, but a held-out run's
    # stretch of it is that run's real behaviour: blank those samples
    h = dict(doc.get("history") or {})
    rec_ids = [r.get("run_id") for r in (doc.get("session") or {}).get("runs", [])
               if r.get("cell") in held]
    cur = doc.get("recording") or {}
    cells = h.get("cells") or []
    if cells:
        h["series"] = {k: [None if c in held else v for v, c in zip(vals, cells)]
                       for k, vals in (h.get("series") or {}).items()}
        h["cells"] = ["held-out" if c in held else c for c in cells]
    if cur.get("cell") in held:
        out["recording"] = {"run_id": "(a held-out cell)", "n": cur.get("n")}
    out["history"] = h
    out["live"] = doc.get("live", {}) if cur.get("cell") not in held else {}
    ch = json.loads(json.dumps(doc.get("channels") or {}))
    for name, hh in (ch.get("history") or {}).items():
        cl = hh.get("cells") or []
        hh["series"] = {k: [None if c in held else v for v, c in zip(vals, cl)]
                        for k, vals in (hh.get("series") or {}).items()}
        hh["cells"] = ["held-out" if c in held else c for c in cl]
    if cur.get("cell") in held:
        ch["live"] = {}
    out["channels"] = ch
    out["held_out_runs_hidden"] = len(rec_ids)
    return out


def _public_live(doc: dict, held: set) -> dict:
    out = dict(doc)
    sess = dict(out.get("session") or {})
    sess["runs"] = [r for r in sess.get("runs", []) if r.get("cell") not in held]
    sess["cells"] = {k: v for k, v in (sess.get("cells") or {}).items()
                     if k not in held}
    sess.pop("overall", None)
    out["session"] = sess
    out["live"] = {}
    return out


def _push(cfg: dict, build: Path, log) -> dict:
    target = cfg.get("target", "none")
    if target == "none":
        return {"ok": True, "pushed": False,
                "detail": "built on this PC only (no publish target set)"}
    if target == "folder":
        dest = Path(cfg.get("folder") or "")
        if not str(dest).strip():
            return {"ok": False, "error": "publish.json has target folder but "
                                          "no folder"}
        tmp = dest.with_name(dest.name + ".incoming")
        old = dest.with_name(dest.name + ".previous")
        try:
            if tmp.exists():
                shutil.rmtree(tmp)
            shutil.copytree(build, tmp)
            # swap in one rename, so a reader never sees half a site
            if dest.exists():
                if old.exists():
                    shutil.rmtree(old)
                dest.rename(old)
            tmp.rename(dest)
        except OSError as e:
            return {"ok": False, "error": f"could not write {dest}: {e}"}
        log(f"site copied to {dest}")
        return {"ok": True, "pushed": True, "detail": f"copied to {dest}"}
    if target == "git":
        repo = Path(cfg.get("repo") or "")
        branch = cfg.get("branch") or "gh-pages"
        if not (repo / ".git").exists():
            return {"ok": False, "error": f"{repo} is not a git clone; clone the "
                                          f"site repository there first"}

        def git(*a):
            return subprocess.run(["git", "-C", str(repo), *a], capture_output=True,
                                  text=True, timeout=120)
        r = git("checkout", branch)
        if r.returncode != 0:
            return {"ok": False, "error": f"git checkout {branch}: {r.stderr.strip()}"}
        for item in build.iterdir():
            dst = repo / item.name
            if dst.is_dir():
                shutil.rmtree(dst)
            elif dst.exists():
                dst.unlink()
            (shutil.copytree if item.is_dir() else shutil.copy2)(item, dst)
        git("add", "-A")
        msg = "SONAIR site update " + time.strftime("%Y-%m-%d %H:%M UTC", time.gmtime())
        r = git("commit", "-m", msg)
        if r.returncode != 0 and "nothing to commit" not in (r.stdout + r.stderr):
            return {"ok": False, "error": f"git commit: {r.stderr.strip() or r.stdout.strip()}"}
        r = git("push", "origin", branch)
        if r.returncode != 0:
            return {"ok": False, "error": f"git push: {r.stderr.strip()}"}
        log(f"site pushed to {branch}")
        return {"ok": True, "pushed": True, "detail": f"pushed to {branch}"}
    return {"ok": False, "error": f"unknown publish target {target!r}"}


def publish(root=".", runs_dir="bench_runs", state_path="campaign/state.json",
            dry_run: bool = False, push: bool = True, log=print) -> dict:
    root = Path(root)
    runs_dir, state_path = root / runs_dir, root / state_path
    cfg = load_config(root / CONFIG)
    t0 = time.time()
    runs, left_out = _accepted(runs_dir, state_path)
    from .site import load_site
    from .sites import list_sites, site_of
    home_site = load_site(root)
    sites = list_sites(root)
    site_dir = {s["site"]: Path(s["path"]) for s in sites}
    for s_ in sites:
        sr, sl = _accepted(site_dir[s_["site"]] / "bench_runs",
                           site_dir[s_["site"]] / "campaign" / "state.json")
        runs += sr
        left_out += [f"{s_['site']}: {x}" for x in sl]
    held = split_cells(plan_campaign())[1]
    plan = {"accepted_runs": len(runs), "left_out": len(left_out),
            "held_out_cells": len(held), "target": cfg["target"]}
    if dry_run:
        return {"ok": True, "dry_run": True, **plan,
                "left_out_examples": left_out[:8]}
    if not runs:
        return {"ok": False, "error": "no accepted benchmark runs to publish", **plan}

    sim_dir = root / SIM_DIR
    try:
        _state = json.loads(state_path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        _state = {}
    home_cal = session_cals(root, _state)
    site_cal = {}
    for name, d in site_dir.items():
        try:
            site_cal[name] = session_cals(d, json.loads((d / "campaign" / "state.json")
                                                         .read_text(encoding="utf-8")))
        except (OSError, ValueError):
            site_cal[name] = lambda rid: None

    def cal_for(rid):
        s_ = site_of(rid)
        return site_cal[s_](rid) if s_ in site_cal else home_cal(rid)
    rep = _replay_missing(runs, sim_dir, log, tcp_offset=cfg.get("tcp_offset"),
                          runs_dir=runs_dir, cal_for=cal_for)
    if not rep.get("ok") and not any(sim_dir.glob("*.jsonl")):
        return {"ok": False, "error": rep.get("error") or "; ".join(rep.get("errors", [])),
                **plan}

    accepted_ids = {r.manifest.run_id for r in runs}
    sims = [s for s in read_dataset(sim_dir, side="sim")
            if s.manifest.run_id in accepted_ids]
    all_pairs = pair_runs(runs, sims)
    lonely_real, _ = unpaired(runs, sims)
    # Everything below that is PUBLIC -- the position board, the channel
    # boards, the charts, the gap file -- is built from the home site's runs
    # only. Another site's runs are all test data, and they share their cell
    # names with the home site's published cells: let them in here and their
    # real numbers would be published as if they were training data. They
    # enter the task package, and only there, as the cross-site test split.
    pairs = [p_ for p_ in all_pairs if not site_of(p_[0].manifest.run_id)]
    if not pairs:
        return {"ok": False, "error": "no home-site run could be paired with its S0 replay",
                **plan}

    # the private picture: every cell
    reports = [gap_between(r, s) for r, s in pairs]
    cell_map = aggregate_by_cell(reports)
    gc = gate_c(cell_map)
    overall = sum(x.pos_median_mm for x in reports) / len(reports)
    gap_doc = {"n_pairs": len(pairs), "overall_pos_median_mm": overall,
               "gate_b": None, "gate_c": gc, "cell_map": cell_map,
               "runs": [x.summary() for x in reports], "measurement_floor": None}
    (root / "results").mkdir(exist_ok=True)
    (root / "results" / "gap.json").write_text(json.dumps(gap_doc, indent=2),
                                                encoding="utf-8")

    # the leaderboard: S0 and B1, fitted on published cells only
    fit = [p for p in pairs if p[0].manifest.cell_key() not in held]
    entries = [
        score_submission(pairs, None, "S0 · reference simulator, unchanged",
                         held_out_cells=held, track="A", kind="baseline",
                         description="the reference simulation; GCR 0 by definition"),
        score_submission(pairs, baseline_constant_offset(pairs, fit),
                         "B1 · one constant offset", held_out_cells=held,
                         track="B", kind="baseline",
                         description="one XYZ offset fitted on the published cells"),
    ]
    k0 = _kinematic_baseline(runs, root / K0_DIR, runs_dir, log)
    if k0.get("ok"):
        k0_runs = [r for r in read_dataset(root / K0_DIR, side="sim")
                   if r.manifest.run_id in accepted_ids]
        if k0_runs:
            entries.append(score_submission(
                pairs, submission_from_sim_runs(k0_runs, pairs),
                "K0 · rigid kinematics (joints = command)", held_out_cells=held,
                track="A", kind="baseline",
                description="no dynamics: the commanded joints put straight "
                            "onto the model. What it scores is how much of the "
                            "position gap is the reference servo, not physics"))
    sessions: dict[str, int] = {}
    try:
        st = json.loads(state_path.read_text(encoding="utf-8"))
        for rid in accepted_ids:
            s = (st.get("done", {}).get(rid) or {}).get("session")
            if s is not None:
                sessions[f"Session {int(s) + 1}"] = sessions.get(f"Session {int(s) + 1}", 0) + 1
    except (OSError, ValueError):
        pass
    by_site: dict[str, int] = {}
    for p_ in all_pairs:
        by_site[p_[0].manifest.site] = by_site.get(p_[0].manifest.site, 0) + 1
    dataset = {"runs": {"E2": len(pairs)},
               "cells": len({p[0].manifest.cell_key() for p in pairs}),
               "held_out_cells": len(held),
               "rate_hz": max((p[0].manifest.sample_rate_hz for p in pairs), default=0),
               "sessions": dict(sorted(sessions.items())),
               "sites": dict(sorted(by_site.items())), "home_site": home_site["site"]}

    build = root / BUILD
    if build.exists():
        shutil.rmtree(build)
    (build / "site").mkdir(parents=True)
    (build / "results").mkdir()
    lb = write_leaderboard(build / "site" / "leaderboard.json", entries,
                           gate_c_result=gc, dataset=dataset)
    # score_submission keeps each cell's raw S0 and entry error in mm beside
    # its GCR. For a held-out cell that raw error is exactly what must not be
    # public, so the public file keeps the ratio and drops the millimetres.
    for e in lb.get("entries", []):
        for cell, row in (e.get("per_cell") or {}).items():
            if cell in held:
                for k in [k for k in row if k.endswith("_mm")]:
                    row.pop(k)
                row["held_out"] = True
    # THE TRACKS WHERE THE PHYSICS IS. Position stays on the board as a
    # kinematics check (K0 shows why it cannot be the headline); effort is the
    # primary track and the IMU's angular rate the second.
    from . import channels as chn
    try:
        lb["channels"] = chn.score_all(pairs, held, fit, cal_for=cal_for, log=log)
        lb["track_order"] = ["effort", "gyro", "position"]
        lb["position_role"] = ("kinematics check: copying the commanded joints (K0) "
                               "closes most of this gap, so it does not rank entries")
    except Exception as e:                          # noqa: BLE001
        log(f"channel scoring failed: {type(e).__name__}: {e}")
    (build / "site" / "leaderboard.json").write_text(json.dumps(lb, indent=2),
                                                     encoding="utf-8")
    try:
        _st = json.loads(state_path.read_text(encoding="utf-8")).get("done", {})
    except (OSError, ValueError):
        _st = {}

    def _sess(rid):
        s_ = (_st.get(rid) or {}).get("session")
        return None if s_ is None else int(s_) + 1
    charts = _charts(pairs, reports, held, entries, _sess, root / K0_DIR)
    # the channels, drawn: S0's raw gap by speed on the published cells, the
    # scores, and one run's elbow current against what each model predicts
    try:
        boards = lb.get("channels") or {}
        charts["channels"] = {k: {"name": b["name"], "unit": b["unit"], "role": b["role"],
                                  "published": b.get("published", []),
                                  "scores": [{"name": e["name"], "gcr_p95": round(e["gcr_p95"], 3)}
                                             for e in b["entries"]]}
                              for k, b in boards.items()}
        ex_id = charts["example"]["run_id"]
        ex = next(p_ for p_ in pairs if p_[0].manifest.run_id == ex_id)
        emap = boards["effort"]["map"]
        tr, yr = chn.real_series(ex[0], "effort")
        ts, ys = chn.sim_series(ex[1], "effort", emap)
        ur = chn.preds_ur_model([ex], fit).get(ex_id)
        f1, _ = chn.preds_s0_friction([ex], fit, emap)
        f1 = f1.get(ex_id)
        step = max(1, len(tr) // 400)
        t0 = tr[0]
        from .task import input_rows, tail_mask
        irows = input_rows(*ex)
        tm = dict(zip([r["t"] for r in irows], tail_mask(irows)))
        charts["effort_example"] = {
            "run_id": ex_id,
            "columns": ["t_s", "real_A", "s0_A", "ur_model_A", "s0_friction_A", "tail"],
            "rows": [[round(float(t - t0), 3), round(float(y), 3),
                      round(float(np.interp(t, ts, ys[:, 0])), 3),
                      round(float(np.interp(t, ur[0], ur[1][:, 0])), 3) if ur else None,
                      round(float(np.interp(t, f1[0], f1[1][:, 0])), 3) if f1 else None,
                      1 if tm.get(round(float(t), 6)) else 0]
                     for t, y in zip(tr[::step], yr[::step, 0])]}
    except Exception as e:                          # noqa: BLE001
        log(f"channel charts skipped: {type(e).__name__}: {e}")
    (build / "site" / "charts.json").write_text(json.dumps(charts), encoding="utf-8")
    (build / "results" / "gap.json").write_text(
        json.dumps(_public_gap(gap_doc, held), indent=2), encoding="utf-8")
    live = root / "results" / "live_score.json"
    if live.exists():
        try:
            (build / "results" / "live_score.json").write_text(json.dumps(
                _public_live(json.loads(live.read_text(encoding="utf-8")), held),
                indent=2), encoding="utf-8")
        except (OSError, ValueError):
            pass
    # THE BENCHMARK OF RECORD: the task package (public inputs, hidden targets),
    # the baseline ladder made from the public package exactly as an entry is,
    # and every one scored by the evaluator. The board the site leads with.
    # THE TOOL CAMERA: each run's seen tool position, where it was tracked
    vis = _vision_targets(root, all_pairs, site_dir, log)
    tcp_for = _tcp_offsets(root, all_pairs, held, home_site, site_dir, log)
    if vis["gate"]:
        dataset["vision"] = {k: vis["gate"].get(k) for k in
                             ("verdict", "ranked", "floor_p95_mm", "beyond_encoders_p95_mm", "ratio")}
        dataset["vision"]["runs"] = len(vis["by_run"])
    try:
        lb["board"] = run_task_board(root, all_pairs, held, lb, cal_for, build, log,
                                     vision_for=lambda real: vis["by_run"].get(real.manifest.run_id),
                                     tcp_offset_for=tcp_for,
                                     train_sites=[home_site["site"]], vision_gate=vis["gate"])
        from .cards import write_cards
        names = write_cards(build / "cards", lb["board"]["task"], dataset, lb["board"])
        write_cards(root / "docs" / "cards", lb["board"]["task"], dataset, lb["board"])
        lb["board"]["cards"] = [f"cards/{n}" for n in names]
        (build / "site" / "leaderboard.json").write_text(json.dumps(lb, indent=2),
                                                         encoding="utf-8")
    except Exception as e:                          # noqa: BLE001
        log(f"task board skipped: {type(e).__name__}: {e}")
    page = root / "benchmark.html"
    if page.exists():
        shutil.copy2(page, build / "benchmark.html")
        shutil.copy2(page, build / "index.html")
    status = {"updated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
              "pairs": len(pairs), "unpaired": len(lonely_real),
              "overall_pos_median_mm": round(overall, 3),
              "gate_c": gc.get("verdict"), "sessions": dataset["sessions"],
              "s0_replay": {k: rep.get(k) for k in ("made", "failed", "errors")}}
    (build / "site" / "status.json").write_text(json.dumps(
        {k: v for k, v in status.items() if k != "s0_replay"}, indent=2),
        encoding="utf-8")
    # The same public files beside the operator's own copy of the page, so
    # benchmark.html opened from the project folder shows the board too (it
    # also finds the PRIVATE full gap map there, which is the operator's).
    (root / "site").mkdir(exist_ok=True)
    for f in ("leaderboard.json", "status.json", "charts.json"):
        shutil.copy2(build / "site" / f, root / "site" / f)
    for d in ("site/scorecards", "downloads", "cards"):
        if (build / d).exists():
            if (root / d).exists():
                shutil.rmtree(root / d)
            shutil.copytree(build / d, root / d)

    pushed = _push(cfg, build, log) if push else {"ok": True, "pushed": False,
                                                    "detail": "push skipped"}
    status.update({"push": pushed, "seconds": round(time.time() - t0, 1),
                   "build": str(build.resolve())})
    (root / STATUS).write_text(json.dumps(status, indent=2), encoding="utf-8")
    log(f"published {len(pairs)} runs: overall median gap {overall:.2f} mm, "
        f"Gate C {gc.get('verdict')}; {pushed.get('detail') or pushed.get('error')}")
    return {"ok": bool(pushed.get("ok")), **plan, **status,
            "error": pushed.get("error", "")}


def last_status(root=".") -> dict:
    try:
        return json.loads((Path(root) / STATUS).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}

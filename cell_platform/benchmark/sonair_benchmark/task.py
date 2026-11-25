"""
task -- the SONAIR benchmark task, defined once: what goes in, what comes out,
how it is split, and the package a team downloads.

THE QUESTION. Given what a robot was commanded and what the reference
simulator says will happen, predict what the real robot's sensors will read --
above all in the rare, demanding moments. Put another way: turn simulated data
into real data. A model that can do that in the long tail is a model whose
synthetic data can stand in for the real examples of rare, critical events
that are too few to learn from.

INPUT, per run, per sample at the controller's 125 Hz:
    cmd   q[6], qd[6] (rad, rad/s)   the commanded joint trajectory
          tau_ctrl[6] (N m)          the controller's own torque model
    sim   q[6], tcp[3] (m)           the reference simulator S0, same command
          tau[6] (N m)               S0's joint torques
          gyro[3] (rad/s)            S0's IMU, in the real IMU's axes
          accel[3] (m/s^2)           likewise
  plus, once per run: the condition (elbow speed, pose, trajectory, payload),
  the IMU's measured mounting, and the published effort map.

OUTPUT, per run, on exactly the input's timestamps:
    current       the real elbow motor current, A          (primary track)
    gyro[3]       the real IMU angular rate, rad/s         (second track)
    vis[3]        optional: the tool position the tool camera SEES, m, base
                  frame -- scored at the camera's own frame times, against
                  K0; reported, and ranked only once the vision gate passes
    accel[3]      optional; provisional, not ranked
    sigma         optional per-sample standard deviations, for calibration

SITES. The task is recorded at more than one cell with the same robot model.
Runs from the training site(s) split by condition cell as above; every run
from any other site is TEST, whatever its cell -- the cross-site split, which
asks whether a model learned the robot model or only one robot unit.

SPLITS. train: the published condition cells, input AND target. test: the
held-out cells, input only; their targets stay with the evaluator. Within the
test runs, the TAIL is the transient moments -- starts, stops, reversals,
where the commanded elbow acceleration is at least half its peak -- the
rare, high-load samples where the gap is largest and examples are fewest.
Out-of-distribution runs (E3: unseen payload, multi-joint path) count as tail
in full when they exist. Every test run also comes in three STRESSED copies
whose input is degraded (missing stretches, 40 ms lag, a 2 deg IMU and 3%
torque calibration drift); the target is unchanged.
"""
from __future__ import annotations

import hashlib
import json
import math
import shutil
import time
from pathlib import Path

import numpy as np

TASK_VERSION = "sonair-task/2"
ELBOW = 2                          # default; each run carries its own "effort_joint"
TAIL_ACCEL_FRACTION = 0.5          # |cmd qdd| >= this x run peak  -> tail sample
STRESS = {
    "missing": "three stretches of the simulated input (20% of samples) are absent",
    "lag": "the simulated input arrives 40 ms late",
    "drift": "the IMU calibration is off by 2 deg about x and S0's torque by +3%",
}
CHANNELS_OUT = ("current", "gyro", "accel", "vis")


# ---------------------------------------------------------------------------
# building one run's input and target
# ---------------------------------------------------------------------------

def _f(v, nd=5):
    return None if v is None else [round(float(x), nd) for x in v]


def input_rows(real, sim) -> list[dict]:
    n = min(len(real.samples), len(sim.samples))
    out = []
    for a, b in zip(real.samples[:n], sim.samples[:n]):
        if not a.get("target_q"):
            continue
        si = (b.get("imu") or {}).get("ind0") or {}
        out.append({"t": round(float(a["t"]), 6),
                    "cmd": {"q": _f(a["target_q"], 6), "qd": _f(a.get("target_qd"), 6),
                            "tau_ctrl": _f(a.get("target_moment"), 4)},
                    "sim": {"q": _f(b.get("q"), 6), "tcp": _f(b.get("tcp_pos"), 6),
                            "tau": _f(b.get("tau"), 4), "gyro": _f(si.get("gyro"), 5),
                            "accel": _f(si.get("accel"), 4)}})
    return out


def target_rows(real, ts, effort_joint: int = ELBOW, imu_lag_s=None) -> list[dict]:
    """imu_lag_s given (sonair-task/3): the gyro and accel targets are the IMU on its own timestamps, shifted by the
    measured lag and interpolated at the row times, instead of the latest sample held on each row."""
    by_t = {round(float(s["t"]), 6): s for s in real.samples}
    own = {}
    if imu_lag_s is not None:
        from .channels import imu_own
        for key in ("gyro", "accel"):
            ti, y = imu_own(real, key)
            if len(ti) >= 20:
                own[key] = (ti - imu_lag_s, y)
    out = []
    for t in ts:
        s = by_t.get(t) or {}
        imu = dict((s.get("imu") or {}).get("ind0") or {})
        for key, (ti, y) in own.items():
            imu[key] = [float(np.interp(t, ti, y[:, k])) for k in range(3)] if ti[0] <= t <= ti[-1] else None
        cur = ((s.get("sensors") or {}).get("ur_tcp") or {}).get("actual_current")
        out.append({"t": t, "current": None if not cur else round(float(cur[effort_joint]), 4),
                    "gyro": _f(imu.get("gyro"), 5), "accel": _f(imu.get("accel"), 4),
                    "tcp": _f(s.get("tcp_pos"), 6)})
    return out


def tail_mask(rows: list[dict], experiment: str = "E2", effort_joint: int = ELBOW) -> list[bool]:
    """The long-tail samples of one run: its transients (or all of an E3 run)."""
    if experiment == "E3":
        return [True] * len(rows)
    t = np.array([r["t"] for r in rows])
    qd = np.array([(r["cmd"]["qd"] or [0] * 6)[effort_joint] for r in rows])
    if len(t) < 3:
        return [False] * len(rows)
    qdd = np.abs(np.gradient(qd, t))
    peak = qdd.max()
    if peak <= 1e-6:
        return [False] * len(rows)
    return [bool(x) for x in (qdd >= TAIL_ACCEL_FRACTION * peak)]


def stressed(rows: list[dict], kind: str, seed: int) -> list[dict]:
    """A degraded copy of a run's INPUT. The evaluator scores it against the clean target."""
    rng = np.random.default_rng(seed)
    out = json.loads(json.dumps(rows))
    n = len(out)
    if kind == "missing":
        for _ in range(3):
            w = max(1, int(0.2 * n / 3))
            s0 = int(rng.integers(0, max(1, n - w)))
            for r in out[s0:s0 + w]:
                r["sim"] = {k: None for k in r["sim"]}
    elif kind == "lag":
        k = 5                                   # 5 samples at 125 Hz = 40 ms
        sims = [r["sim"] for r in out]
        for i, r in enumerate(out):
            r["sim"] = sims[max(0, i - k)]
    elif kind == "drift":
        a = math.radians(2.0)
        R = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
        for r in out:
            s = r["sim"]
            for key in ("gyro", "accel"):
                if s.get(key):
                    s[key] = [round(float(v), 5) for v in R @ np.asarray(s[key])]
            if s.get("tau"):
                s["tau"] = [round(1.03 * float(v), 4) for v in s["tau"]]
    else:
        raise ValueError(kind)
    return out


def _write_jsonl(path: Path, head: dict, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        fh.write(json.dumps(head) + "\n")
        for r in rows:
            fh.write(json.dumps(r, separators=(",", ":")) + "\n")


def read_jsonl(path: Path):
    lines = Path(path).read_text(encoding="utf-8").splitlines()
    return json.loads(lines[0]), [json.loads(x) for x in lines[1:] if x.strip()]


def _sha(p: Path) -> str:
    h = hashlib.sha256()
    with p.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


# ---------------------------------------------------------------------------
# the package
# ---------------------------------------------------------------------------

def build_package(pairs, held: set, public_dir, private_dir, *, effort_map: dict,
                  cal_for=None, version: str = "v2", vision_for=None,
                  tcp_offset_for=None, train_sites=None, log=print) -> dict:
    """
    public_dir: what a team downloads -- train inputs with targets, test inputs.
    private_dir: what only the evaluator holds -- test targets and the tail masks.
    Both are rebuilt from scratch, so a stale file can never leak in or survive.
    """
    pub, priv = Path(public_dir), Path(private_dir)
    for d in (pub, priv):
        if d.exists():
            shutil.rmtree(d)
    index = {"task": TASK_VERSION, "version": version, "train": [], "test": []}
    answers = {}
    from .site import effort_joint as _ej
    home = sorted({p[0].manifest.site for p in pairs})
    train_sites = set(train_sites or home[:1] or ["UoN"])
    index["train_sites"] = sorted(train_sites)
    n_vis = 0
    for real, sim in pairs:
        m = real.manifest
        rid = m.run_id
        cell = m.cell_key()
        rows = input_rows(real, sim)
        if len(rows) < 50:
            continue
        ej = _ej(m.robot)
        cal = cal_for(rid) if cal_for else None
        T_ft = tcp_offset_for(real) if tcp_offset_for else None
        cross = m.site not in train_sites
        head = {"_input": {"task": TASK_VERSION, "run_id": rid,
                           "condition": {"joint_vel": m.joint_vel, "arm_config": m.arm_config,
                                         "traj_type": m.traj_type, "experiment": m.experiment,
                                         "payload_kg": m.carrier_mass_kg},
                           "site": m.site, "robot": m.robot, "effort_joint": ej,
                           "imu_mount": (cal or {}).get("R_flange_imu"),
                           "tcp_offset": None if T_ft is None else np.round(T_ft, 6).tolist(),
                           "effort_map": {k: effort_map.get(k) for k in ("a", "b")},
                           "rate_hz": m.sample_rate_hz}}
        tgt = target_rows(real, [r["t"] for r in rows], ej)
        vis = vision_for(real) if vision_for else None
        vis_doc = None
        if vis is not None and len(vis.get("t", [])):
            vis_doc = {"run_id": rid, "t": [round(float(x), 6) for x in vis["t"]],
                       "p": np.round(np.asarray(vis["P"]), 6).tolist(),
                       "latency_s": vis.get("latency_used_s")}
            n_vis += 1
        if cell not in held and m.experiment != "E3" and not cross:
            head["_input"]["split"] = "train"
            _write_jsonl(pub / "train" / f"{rid}.input.jsonl", head, rows)
            _write_jsonl(pub / "train" / f"{rid}.target.jsonl", {"_target": {"run_id": rid}}, tgt)
            if vis_doc:
                (pub / "train" / f"{rid}.vis.json").write_text(json.dumps(vis_doc), encoding="utf-8")
            index["train"].append({"run_id": rid, "cell": cell, "site": m.site})
            continue
        mask = tail_mask(rows, m.experiment, ej)
        variants = [(rid, None, rows)] + [
            (f"{rid}~{k}", k, stressed(rows, k, int(_sha_str(rid + k), 16) % (2 ** 31)))
            for k in STRESS]
        for vid, kind, vrows in variants:
            h = json.loads(json.dumps(head))
            h["_input"].update(run_id=vid, split="test", stress=kind)
            _write_jsonl(pub / "test" / f"{vid}.input.jsonl", h, vrows)
            index["test"].append({"run_id": vid, "base": rid, "stress": kind})
        answers[rid] = {"cell": cell, "experiment": m.experiment, "tail": mask,
                        "site": m.site, "cross_site": cross}
        _write_jsonl(priv / "targets" / f"{rid}.target.jsonl", {"_target": {"run_id": rid}}, tgt)
        if vis_doc:
            (priv / "targets" / f"{rid}.vis.json").write_text(json.dumps(vis_doc), encoding="utf-8")
    (priv).mkdir(parents=True, exist_ok=True)
    (priv / "answers.json").write_text(json.dumps(answers), encoding="utf-8")
    pub.mkdir(parents=True, exist_ok=True)
    spec = task_spec(version)
    (pub / "task.json").write_text(json.dumps({**spec, "index": index}, indent=1), encoding="utf-8")
    (pub / "TASK.md").write_text(task_readme(spec, index), encoding="utf-8")
    sums = {str(p.relative_to(pub)).replace("\\", "/"): _sha(p)
            for p in sorted(pub.rglob("*")) if p.is_file()}
    (pub / "SHA256SUMS.json").write_text(json.dumps(sums, indent=1), encoding="utf-8")
    n_cross = sum(1 for a in answers.values() if a["cross_site"])
    log(f"task package: {len(index['train'])} train runs, "
        f"{len(answers)} test runs x {1 + len(STRESS)} inputs"
        + (f" ({n_cross} from other sites)" if n_cross else "")
        + (f"; tool camera on {n_vis} runs" if n_vis else ""))
    return {"train": len(index["train"]), "test": len(answers), "cross_site": n_cross,
            "vision_runs": n_vis,
            "test_inputs": len(index["test"]), "public": str(pub), "private": str(priv)}


def _sha_str(s: str) -> str:
    return hashlib.sha256(s.encode()).hexdigest()[:8]


def task_spec(version="v2") -> dict:
    return {
        "task": TASK_VERSION, "version": version,
        "question": "Given a robot's command and its simulation, predict what the real "
                    "robot's sensors will read -- above all in the rare, demanding moments.",
        "input": {"cmd": {"q": "rad[6]", "qd": "rad/s[6]", "tau_ctrl": "N m[6]"},
                  "sim": {"q": "rad[6]", "tcp": "m[3]", "tau": "N m[6]",
                          "gyro": "rad/s[3], IMU axes", "accel": "m/s^2[3], IMU axes"},
                  "per_run": ["condition", "site", "robot", "effort_joint", "imu_mount",
                              "tcp_offset", "effort_map", "rate_hz"]},
        "output": {"current": "A, elbow (primary track)",
                   "gyro": "rad/s[3], IMU axes (second track)",
                   "vis": "m[3], base frame, optional: the tool position the tool camera sees "
                          "(scored at camera frame times against K0)",
                   "accel": "m/s^2[3], optional, provisional",
                   "sigma": "optional {channel: per-sample standard deviation}"},
        "splits": {"train": "published condition cells of the training site(s), input and target",
                   "test": "held-out condition cells, input only",
                   "cross_site": "every run from a site not in training, input only",
                   "tail": f"test samples where |commanded elbow acceleration| >= "
                           f"{TAIL_ACCEL_FRACTION} x its peak in the run (starts, stops, "
                           f"reversals); E3 runs in full",
                   "stress": STRESS},
        "headline": "Effort Tail GCR-p95: the share of S0's worst-moment error in the "
                    "elbow current that an entry removes, on the long-tail samples of "
                    "the held-out cells",
        "metric": "GCR = 1 - err(entry) / err(ref), per run, at the 95th percentile "
                  "(and median) of the per-sample error; the mean over runs",
        "reference": {"current": "S0", "gyro": "S0", "vis": "K0 (rigid kinematics of the command)"},
    }


def task_readme(spec: dict, index: dict) -> str:
    sites = ", ".join(index.get("train_sites") or []) or "the home site"
    return f"""# SONAIR task {spec['version']}

{spec['question']}

## Files

- `train/<run>.input.jsonl` and `train/<run>.target.jsonl` -- {len(index['train'])} runs from the published cells of the training site ({sites}): learn here.
- `train/<run>.vis.json` -- where the run has one: the tool position the tool camera saw, `{{"t": [...], "p": [[x, y, z], ...]}}` (m, base frame, camera frame times on the run clock).
- `test/<run>.input.jsonl` -- {len(index['test'])} inputs: the held-out cells, and every run recorded at another site (each run clean and in three stressed copies). Their targets are kept by the evaluator.
- `task.json` -- this specification, machine-readable, with the index of every run.
- `SHA256SUMS.json` -- a checksum for every file.

Line 1 of an input file is `{{"_input": {{...}}}}`: run id, condition, site, robot, `effort_joint` (the joint whose current is scored), IMU mounting, `tcp_offset` (flange to tool point, 4x4), effort map, split, stress. Every later line is one sample:

```
{{"t": 12.345678, "cmd": {{"q": [6], "qd": [6], "tau_ctrl": [6]}},
 "sim": {{"q": [6], "tcp": [3], "tau": [6], "gyro": [3], "accel": [3]}}}}
```

A target line: `{{"t": ..., "current": A, "gyro": [3], "accel": [3], "tcp": [3]}}`.

## What to submit

One `predictions.jsonl`, one line per TEST input (stressed copies included):

```
{{"run_id": "v0p90_extended_point_to_point_r00~lag", "t": [...], "current": [...],
 "gyro": [[x, y, z], ...], "vis": [[x, y, z], ...], "sigma": {{"current": [...], "gyro": [...]}}}}
```

`t` must be the input's own timestamps. `vis` and `sigma` are optional; `vis` is
interpolated to the camera's frame times by the evaluator. Plus a `meta.json`:
`{{"name", "team", "description", "runtime_s_per_run", "params"}}`.
Check it with `python -m sonair_benchmark validate <submission> --task <this folder>`.

## How it is scored

{spec['metric']}. References: {", ".join(f"{k} against {v}" for k, v in spec.get("reference", {}).items())}.
Headline: {spec['headline']}. Also reported: the same on all test samples, the
angular-rate and vision tracks, the cross-site split separately, robustness
(the change in GCR on the stressed inputs), calibration (coverage of your
stated 90% interval, if you give `sigma`) and your stated cost.
"""

"""
evaluate -- the pre-flight validator a team runs at home, and the evaluator
that holds the hidden targets.

    python -m sonair_benchmark validate my_submission/ --task <public package>
    python -m sonair_benchmark evaluate my_submission/ --task <public> --answers <private>

A submission is a folder (or a .zip) holding predictions.jsonl and meta.json,
as the package's TASK.md describes. The validator needs only the public
package, so a submission that passes it at home will not be rejected for its
form. The evaluator scores it on the hidden targets and writes a scorecard,
as JSON and as a page a person can read.

Every number is relative to a reference computed HERE from the same public
inputs by the same code, so the reference can never drift from what entries
are scored against: S0 for the joint current and the angular rate; K0 (rigid
kinematics of the commanded joints) for the tool position the camera sees,
because S0's position error is mostly its servo lagging the command.

Runs from a site the package did not train on are scored separately (the
cross-site split): the headline is the training site's held-out cells; the
cross-site numbers say whether a model learned the robot model or one robot.
"""
from __future__ import annotations

import json
import statistics
import sys
import tempfile
import time
import zipfile
from pathlib import Path

import numpy as np

from .task import STRESS, read_jsonl

RANKED = ("current", "gyro")
OPTIONAL = ("vis",)
SCORED = RANKED + OPTIONAL
UNITS = {"current": "A", "gyro": "rad/s", "accel": "m/s^2", "vis": "m"}
TRACK_NAME = {"current": "Joint effort", "gyro": "Tool angular rate",
              "accel": "Tool acceleration", "vis": "Tool position, seen by the camera"}
REFERENCE = {"current": "S0", "gyro": "S0", "vis": "K0"}
Z90 = 1.6449


# ---------------------------------------------------------------------------
# reading
# ---------------------------------------------------------------------------

def open_submission(path) -> Path:
    p = Path(path)
    if p.is_file() and p.suffix.lower() == ".zip":
        d = Path(tempfile.mkdtemp(prefix="sonair_sub_"))
        with zipfile.ZipFile(p) as z:
            for n in z.namelist():
                if n.startswith("/") or ".." in Path(n).parts:
                    raise ValueError(f"unsafe path in the zip: {n}")
            z.extractall(d)
        inner = [c for c in d.rglob("predictions.jsonl")]
        return inner[0].parent if inner else d
    return p


def _inputs(task_dir: Path):
    spec = json.loads((task_dir / "task.json").read_text(encoding="utf-8"))
    return spec, {e["run_id"]: e for e in spec["index"]["test"]}


def _ej(head) -> int:
    return int(head["_input"].get("effort_joint", 2))


def s0_prediction(rows: list[dict], head: dict) -> dict:
    """The reference, from the public input alone."""
    m = head["_input"].get("effort_map") or {"a": 1.0, "b": 0.0}
    j = _ej(head)
    cur, gy = [], []
    for r in rows:
        s = r["sim"]
        cur.append(None if not s.get("tau") else m["a"] * s["tau"][j] + m["b"])
        gy.append(s.get("gyro"))
    return {"t": [r["t"] for r in rows], "current": cur, "gyro": gy}


def _kin():
    root = Path(__file__).resolve().parent.parent
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    import ur_kin
    return ur_kin


def k0_tool_points(rows: list[dict], head: dict):
    """K0 for the vision track: the commanded joints through rigid kinematics to the tool point."""
    T_ft = head["_input"].get("tcp_offset")
    if not T_ft:
        return None
    k = _kin()
    dh = k.DH.get(head["_input"].get("robot", "ur5e"), k.UR5E_DH)
    T_ft = np.asarray(T_ft, float)
    out = []
    for r in rows:
        q = (r.get("cmd") or {}).get("q")
        out.append(None if not q else (k.fk(q, dh) @ T_ft)[:3, 3].tolist())
    return out


# ---------------------------------------------------------------------------
# the validator
# ---------------------------------------------------------------------------

def _check_series(rid, ch, v, n, problems, required=True):
    if v is None:
        if required:
            problems.append(f"{rid}: no {ch!r}")
        return
    if len(v) != n:
        problems.append(f"{rid}: {ch} has {len(v)} values for {n} samples")
        return
    arr = np.asarray([x if x is not None else np.nan for x in v], dtype=float)
    if ch in ("gyro", "vis") and (arr.ndim != 2 or arr.shape[1] != 3):
        problems.append(f"{rid}: {ch} must be [[x, y, z], ...]")
        return
    if not np.all(np.isfinite(arr)):
        problems.append(f"{rid}: {ch} contains null, NaN or infinity")


def validate(sub_path, task_dir) -> dict:
    task_dir = Path(task_dir)
    problems, warnings = [], []
    try:
        sub = open_submission(sub_path)
    except Exception as e:                          # noqa: BLE001
        return {"ok": False, "problems": [f"cannot open the submission: {e}"], "warnings": []}
    pf, mf = sub / "predictions.jsonl", sub / "meta.json"
    if not pf.exists():
        return {"ok": False, "problems": ["predictions.jsonl is missing"], "warnings": []}
    meta = {}
    if not mf.exists():
        problems.append("meta.json is missing")
    else:
        try:
            meta = json.loads(mf.read_text(encoding="utf-8"))
        except ValueError as e:
            problems.append(f"meta.json is not valid JSON: {e}")
        for k in ("name", "team", "description"):
            if not meta.get(k):
                problems.append(f"meta.json has no {k!r}")
    _spec, wanted = _inputs(task_dir)
    seen = set()
    for ln, line in enumerate(pf.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            d = json.loads(line)
        except ValueError as e:
            problems.append(f"line {ln}: not JSON ({e})")
            continue
        rid = d.get("run_id")
        if rid not in wanted:
            warnings.append(f"line {ln}: {rid!r} is not a test input; ignored")
            continue
        if rid in seen:
            problems.append(f"{rid}: predicted twice")
        seen.add(rid)
        head, rows = read_jsonl(task_dir / "test" / f"{rid}.input.jsonl")
        ts = [r["t"] for r in rows]
        t = d.get("t") or []
        if len(t) != len(ts) or any(abs(a - b) > 1e-5 for a, b in zip(t, ts)):
            problems.append(f"{rid}: t must be the input's {len(ts)} timestamps")
            continue
        for ch in RANKED:
            _check_series(rid, ch, d.get(ch), len(ts), problems)
        for ch in OPTIONAL:
            _check_series(rid, ch, d.get(ch), len(ts), problems, required=False)
        for ch, sg in (d.get("sigma") or {}).items():
            a = np.asarray(sg, dtype=float)
            if len(a) != len(ts) or not np.all(np.isfinite(a)) or np.any(a <= 0):
                problems.append(f"{rid}: sigma.{ch} must be {len(ts)} positive numbers")
    missing = sorted(set(wanted) - seen)
    if missing:
        problems.append(f"{len(missing)} test inputs have no prediction, e.g. {missing[:3]}")
    return {"ok": not problems, "problems": problems[:50], "warnings": warnings[:20],
            "predicted": len(seen), "expected": len(wanted), "meta": meta}


# ---------------------------------------------------------------------------
# the evaluator
# ---------------------------------------------------------------------------

def _err(pred, target, ch):
    """Per-sample error magnitude; NaN where either side is absent."""
    out = []
    for p, y in zip(pred, target):
        if p is None or y is None:
            out.append(np.nan)
        elif ch == "current":
            out.append(abs(float(p) - float(y)))
        else:
            out.append(float(np.linalg.norm(np.asarray(p, float) - np.asarray(y, float))))
    return np.asarray(out)


def _at(ts, series, tq):
    """A per-sample [x,y,z] series (None where absent) linearly interpolated to times tq."""
    ts = np.asarray(ts, float)
    tq = np.asarray(tq, float)
    arr = np.asarray([s if s is not None else [np.nan] * 3 for s in series], float).reshape(-1, 3)
    out = np.full((len(tq), 3), np.nan)
    ok = np.all(np.isfinite(arr), axis=1)
    if ok.sum() < 2 or not len(tq):
        return out
    tk = ts[ok]
    for i in range(3):
        out[:, i] = np.interp(tq, tk, arr[ok, i])
    # never bridge a stretch the prediction left empty
    dt = float(np.median(np.diff(ts))) if len(ts) > 1 else 0.008
    j = np.clip(np.searchsorted(tk, tq), 1, len(tk) - 1)
    gap = tk[j] - tk[j - 1]
    out[gap > 3 * dt] = np.nan
    return out


def _vis_errors(pred_vis, ref_pts, ts, vis_doc):
    """(err entry, err K0, sample index of each camera frame) at the camera's frame times."""
    tv = np.asarray(vis_doc["t"], float)
    P = np.asarray(vis_doc["p"], float).reshape(-1, 3)
    inside = (tv >= ts[0]) & (tv <= ts[-1])
    tv, P = tv[inside], P[inside]
    e = np.linalg.norm(_at(ts, pred_vis, tv) - P, axis=1)
    e0 = np.linalg.norm(_at(ts, ref_pts, tv) - P, axis=1)
    i = np.clip(np.searchsorted(ts, tv), 1, len(ts) - 1)
    idx = np.where(np.abs(ts[i - 1] - tv) <= np.abs(ts[i] - tv), i - 1, i)   # nearest sample
    return e, e0, idx


def _gcr(e, e0, mask=None):
    keep = np.isfinite(e) & np.isfinite(e0)
    if mask is not None:
        keep &= mask
    if keep.sum() < 10:
        return None
    a, b = e[keep], e0[keep]
    p, p0 = np.percentile(a, 95), np.percentile(b, 95)
    m, m0 = np.median(a), np.median(b)
    return {"gcr_p95": 1 - p / p0 if p0 > 1e-12 else 0.0,
            "gcr_median": 1 - m / m0 if m0 > 1e-12 else 0.0,
            "p95": float(p), "median": float(m), "ref_p95": float(p0), "n": int(keep.sum())}


def _mean(xs):
    xs = [x for x in xs if x is not None]
    return float(statistics.fmean(xs)) if xs else None


def _score_run(rid, a, preds, task_dir, answers_dir, cov, sharp):
    _h, tgt = read_jsonl(answers_dir / "targets" / f"{rid}.target.jsonl")
    head, rows = read_jsonl(task_dir / "test" / f"{rid}.input.jsonl")
    ts = np.asarray([r["t"] for r in rows], float)
    ref = s0_prediction(rows, head)
    mask = np.asarray(a["tail"], bool)
    row = {"run_id": rid, "cell": a["cell"], "experiment": a.get("experiment", "E2"),
           "site": a.get("site", ""), "cross_site": bool(a.get("cross_site"))}
    p = preds.get(rid) or {}
    for ch in RANKED:
        y = [r.get(ch) for r in tgt]
        e0 = _err(ref[ch], y, ch)
        if p.get(ch) is None:
            continue
        e = _err(p[ch], y, ch)
        row[ch] = {"all": _gcr(e, e0), "tail": _gcr(e, e0, mask)}
        sg = (p.get("sigma") or {}).get(ch)
        if sg is not None:
            sg = np.asarray(sg, float)
            ok = np.isfinite(e)
            cov[ch] += list(e[ok] <= Z90 * sg[ok])
            sharp[ch] += list(sg[ok])
        for kind in STRESS:
            ps = preds.get(f"{rid}~{kind}") or {}
            if ps.get(ch) is not None:
                row[ch].setdefault("stress", {})[kind] = _gcr(_err(ps[ch], y, ch), e0)
    vf = answers_dir / "targets" / f"{rid}.vis.json"
    if vf.exists() and p.get("vis") is not None:
        vis_doc = json.loads(vf.read_text(encoding="utf-8"))
        k0 = k0_tool_points(rows, head)
        if k0 is not None:
            e, e0, idx = _vis_errors(p["vis"], k0, ts, vis_doc)
            row["vis"] = {"all": _gcr(e, e0), "tail": _gcr(e, e0, mask[idx]),
                          "frames": int(len(e)), "tail_frames": int(mask[idx].sum())}
            for kind in STRESS:
                ps = preds.get(f"{rid}~{kind}") or {}
                if ps.get("vis") is not None:
                    es, _e0, _i = _vis_errors(ps["vis"], k0, ts, vis_doc)
                    row["vis"].setdefault("stress", {})[kind] = _gcr(es, e0)
    return row


def _tracks(per_run, cov=None, sharp=None):
    def agg(ch, part, key="gcr_p95"):
        return _mean([r[ch][part][key] if (r.get(ch) or {}).get(part) else None for r in per_run])

    tracks = {}
    for ch in SCORED:
        if not any(r.get(ch) for r in per_run):
            continue
        clean = agg(ch, "all")
        stress = {kind: _mean([((r.get(ch) or {}).get("stress") or {}).get(kind, {}).get("gcr_p95")
                               if ((r.get(ch) or {}).get("stress") or {}).get(kind) else None
                               for r in per_run]) for kind in STRESS}
        t = {"name": TRACK_NAME[ch], "unit": UNITS[ch], "reference": REFERENCE[ch],
             "runs": sum(1 for r in per_run if r.get(ch)),
             "tail": {"gcr_p95": agg(ch, "tail"), "gcr_median": agg(ch, "tail", "gcr_median")},
             "all": {"gcr_p95": clean, "gcr_median": agg(ch, "all", "gcr_median")},
             "robustness": {"by_stress": stress,
                            "drop": (None if clean is None or not any(v is not None for v in stress.values())
                                     else _mean([v for v in stress.values()]) - clean)},
             "calibration": None}
        if cov is not None and cov.get(ch):
            t["calibration"] = {"coverage_90": float(np.mean(cov[ch])),
                                "error": abs(float(np.mean(cov[ch])) - 0.9),
                                "mean_sigma": float(np.mean(sharp[ch]))}
        if ch == "vis":
            t["frames"] = int(sum((r.get("vis") or {}).get("frames", 0) for r in per_run))
            t["tail_frames"] = int(sum((r.get("vis") or {}).get("tail_frames", 0) for r in per_run))
        tracks[ch] = t
    return tracks


def evaluate(sub_path, task_dir, answers_dir, *, check: bool = True) -> dict:
    task_dir, answers_dir = Path(task_dir), Path(answers_dir)
    t0 = time.time()
    if check:
        v = validate(sub_path, task_dir)
        if not v["ok"]:
            return {"ok": False, "stage": "validate", **v}
        meta = v["meta"]
    else:
        meta = {}
    sub = open_submission(sub_path)
    if not meta and (sub / "meta.json").exists():
        meta = json.loads((sub / "meta.json").read_text(encoding="utf-8"))
    preds = {}
    for line in (sub / "predictions.jsonl").read_text(encoding="utf-8").splitlines():
        if line.strip():
            d = json.loads(line)
            preds[d["run_id"]] = d
    answers = json.loads((answers_dir / "answers.json").read_text(encoding="utf-8"))

    cov, sharp = {ch: [] for ch in SCORED}, {ch: [] for ch in SCORED}
    per_run = [_score_run(rid, a, preds, task_dir, answers_dir, cov, sharp)
               for rid, a in sorted(answers.items())]
    home = [r for r in per_run if not r["cross_site"]]
    away = [r for r in per_run if r["cross_site"]]
    # calibration coverage is pooled over every scored run, home and away
    tracks = _tracks(home or per_run, cov, sharp)
    cross = None
    if away:
        ct = _tracks(away)
        cross = {"runs": len(away), "sites": sorted({r["site"] for r in away}),
                 "tracks": {ch: {k: ct[ch][k] for k in ("tail", "all", "runs")} for ch in ct}}
    cells = {}
    for r in per_run:
        key = r["cell"] if not r["cross_site"] else f"{r['site']}:{r['cell']}"
        c = cells.setdefault(key, {"n": 0})
        c["n"] += 1
        for ch in SCORED:
            if r.get(ch) and r[ch].get("tail"):
                c.setdefault(ch, []).append(r[ch]["tail"]["gcr_p95"])
    per_cell = {k: {"n": v["n"], **{ch: round(_mean(v[ch]), 4) for ch in SCORED if v.get(ch)}}
                for k, v in sorted(cells.items())}
    headline = (tracks.get("current") or {}).get("tail", {}).get("gcr_p95")
    return {"ok": True, "scorecard": "sonair-scorecard/2",
            "entry": {k: meta.get(k) for k in ("name", "team", "description",
                                                "runtime_s_per_run", "params", "kind")},
            "headline": {"name": "Effort Tail GCR-p95", "value": headline},
            "tracks": tracks, "cross_site": cross, "per_cell": per_cell,
            "runs_scored": len(home or per_run),
            "evaluated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "evaluator_seconds": round(time.time() - t0, 1)}


# ---------------------------------------------------------------------------
# the scorecard a person reads
# ---------------------------------------------------------------------------

def _f(v, nd=3):
    return " - " if v is None else f"{v:+.{nd}f}" if nd else str(v)


def scorecard_html(card: dict) -> str:
    e = card.get("entry") or {}
    rows = ""
    for ch, t in card["tracks"].items():
        rb = t["robustness"]["by_stress"]
        cal = t.get("calibration")
        covtxt = " - " if not cal else f"{100 * cal['coverage_90']:.0f}%"
        rows += (f"<tr><td><b>{t['name']}</b><br><small>{t['unit']} &middot; vs {t.get('reference', 'S0')}</small></td>"
                 f"<td class=n><b>{_f(t['tail']['gcr_p95'])}</b></td><td class=n>{_f(t['all']['gcr_p95'])}</td>"
                 f"<td class=n>{_f(t['all']['gcr_median'])}</td>"
                 + "".join(f"<td class=n>{_f(rb.get(k))}</td>" for k in STRESS)
                 + f"<td class=n>{covtxt}</td></tr>")
    chs = [ch for ch in SCORED if any(ch in v for v in card["per_cell"].values())]
    cells = "".join(f"<tr><td><code>{k}</code></td><td class=n>{v['n']}</td>"
                    + "".join(f"<td class=n>{_f(v.get(ch))}</td>" for ch in chs) + "</tr>"
                    for k, v in card["per_cell"].items())
    cs = card.get("cross_site")
    cross = ""
    if cs:
        cross = ("<div class=\"card\"><b>Other sites</b> <span class=\"muted\">(" + ", ".join(cs["sites"])
                 + f", {cs['runs']} runs: every run from a site the package did not train on)</span>"
                 "<table><thead><tr><th>Track</th><th class=n>Tail GCR p95</th><th class=n>All GCR p95</th></tr></thead><tbody>"
                 + "".join(f"<tr><td>{TRACK_NAME[ch]}</td><td class=n>{_f(v['tail']['gcr_p95'])}</td>"
                           f"<td class=n>{_f(v['all']['gcr_p95'])}</td></tr>" for ch, v in cs["tracks"].items())
                 + "</tbody></table></div>")
    hv = card["headline"]["value"]
    return f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SONAIR scorecard</title><style>
:root{{--bg:#f6f7f9;--panel:#fff;--ink:#141a21;--muted:#56606c;--line:#dde1e7;--accent:#1f5f9e}}
@media (prefers-color-scheme:dark){{:root{{--bg:#101316;--panel:#171b1f;--ink:#e4e8ec;--muted:#a1aab3;--line:#2a3037;--accent:#6aa6e0}}}}
body{{margin:0;background:var(--bg);color:var(--ink);font:15px/1.5 "Segoe UI",system-ui,sans-serif}}
.wrap{{max-width:980px;margin:0 auto;padding:28px 16px}}
h1{{font:700 26px "Segoe UI",sans-serif;margin:0}} .muted{{color:var(--muted)}}
.big{{font:650 52px Consolas,monospace;color:var(--accent)}}
.card{{background:var(--panel);border:1px solid var(--line);border-radius:4px;padding:16px 18px;margin:16px 0;overflow-x:auto}}
table{{border-collapse:collapse;width:100%;font-size:14px}} td,th{{padding:7px 9px;border-bottom:1px solid var(--line);text-align:left}}
th{{font-size:11px;text-transform:uppercase;letter-spacing:.08em;color:var(--muted)}} .n{{text-align:right;font-family:Consolas,monospace}}
</style></head><body><div class="wrap">
<p class="muted">SONAIR &middot; scorecard &middot; {card['evaluated_utc']}</p>
<h1>{e.get('name') or 'Entry'}</h1><p class="muted">{e.get('team') or ''} &middot; {e.get('description') or ''}</p>
<div class="card"><div class="muted">Headline &middot; {card['headline']['name']}</div>
<div class="big">{' - ' if hv is None else f'{hv:+.3f}'}</div>
<div class="muted">The share of the reference simulator's worst-moment error in the joint current that this entry removes, on the long-tail samples of the held-out cells. 1 is perfect, 0 is no better than the reference, negative is worse. {card['runs_scored']} held-out runs.</div></div>
<div class="card"><table><thead><tr><th>Track</th><th class=n>Tail GCR p95</th><th class=n>All GCR p95</th><th class=n>All GCR median</th>
{''.join(f'<th class=n>Stress: {k}</th>' for k in STRESS)}<th class=n>90% interval covers</th></tr></thead><tbody>{rows}</tbody></table>
<p class="muted" style="font-size:13px">Stress columns: the same score when the simulated input is degraded ({'; '.join(f'{k}: {v}' for k, v in STRESS.items())}). Interval coverage: how often the truth fell inside the entry's own 90% interval (90% is perfectly calibrated); only if the entry gave sigma. The camera-seen tool position is scored at the camera's frame times against K0, the commanded joints through rigid kinematics.</p></div>
{cross}
<div class="card"><table><thead><tr><th>Held-out cell</th><th class=n>Runs</th>{''.join(f'<th class=n>{TRACK_NAME[ch]} tail</th>' for ch in chs)}</tr></thead><tbody>{cells}</tbody></table></div>
<p class="muted" style="font-size:13px">Stated cost: {e.get('runtime_s_per_run') or ' - '} s per run, {e.get('params') or ' - '} parameters. Evaluated in {card['evaluator_seconds']} s.</p>
</div></body></html>"""


def write_scorecard(card: dict, out_dir) -> dict:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / "scorecard.json").write_text(json.dumps(card, indent=1), encoding="utf-8")
    if card.get("ok"):
        (out / "scorecard.html").write_text(scorecard_html(card), encoding="utf-8")
    return {"json": str(out / "scorecard.json"),
            "html": str(out / "scorecard.html") if card.get("ok") else None}

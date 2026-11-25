"""
sites -- one benchmark, recorded at more than one cell.

A second site (UCL) runs this same platform on its own UR5e with its own
site.json. Its runs reach the benchmark as a SITE BUNDLE: one zip, made on the
recording PC, checked on the publishing PC.

    python -m sonair_benchmark site-export --out UCL_bundle.zip      # at UCL
    python -m sonair_benchmark site-import UCL_bundle.zip            # at UoN
    python -m sonair_benchmark site-list

What a bundle carries, and why each part:

    runs        every run the campaign accepted (read back, marked done) --
                the same rule the home site's publish uses, so a site cannot
                ship a run its own platform rejected
    vision      the tool camera's per-run board poses, where recorded
    calib       the per-session IMU calibrations and the hand-eye calibration
                in force -- a run without its calibration cannot be replayed
                the way the home site's runs are
    state       the campaign state (which run belongs to which session)
    site.json   who recorded it: site, robot unit, cell
    MANIFEST    a SHA-256 for every file; import refuses a bundle whose files
                do not match, and refuses a bundle claiming to be this site

On import, every run is renamed "<SITE>__<run_id>" (manifest and file), its
manifest says which site and unit recorded it, and `origin_run_id` keeps the
name it was recorded under. Two sites run the same campaign plan, so their run
ids are the same strings; without the prefix the second site's runs would
silently overwrite the first's.

The bundle replaces any earlier bundle from the same site: sites send their
whole accepted dataset each time, so an import is always a complete picture.
"""
from __future__ import annotations

import hashlib
import json
import shutil
import tempfile
import time
import zipfile
from pathlib import Path

from .site import clean_id, load_site

SCHEMA = "sonair-site-bundle/1"
SITES_DIR = Path("sites")
SEP = "__"


def _sha(p: Path) -> str:
    h = hashlib.sha256()
    with Path(p).open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _done(state_path: Path) -> dict:
    try:
        return json.loads(Path(state_path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


# ---------------------------------------------------------------------------
# export, at the recording site
# ---------------------------------------------------------------------------

def export_bundle(root=".", out=None, log=print) -> dict:
    root = Path(root)
    site = load_site(root)
    state = _done(root / "campaign" / "state.json")
    done = state.get("done") or {}
    runs_dir = root / "bench_runs"
    files: dict[str, Path] = {}
    runs = []
    for rid in sorted(done):
        src = runs_dir / f"{rid}.jsonl"
        if not src.exists():
            continue
        files[f"runs/{rid}.jsonl"] = src
        runs.append(rid)
        vtp = root / "vision_logs" / f"{rid}.vtp.jsonl"
        if vtp.exists():
            files[f"vision/{rid}.vtp.jsonl"] = vtp
    if not runs:
        return {"ok": False, "error": "no accepted runs to export (campaign/state.json marks none done)"}
    for p in sorted((root / "calib" / "sessions").glob("*.json")):
        files[f"calib/sessions/{p.name}"] = p
    for rel in ("calib/imu_cal.json", "calibration/handeye.json", "carrier.json", "site.json"):
        if (root / rel).exists():
            files[rel] = root / rel
    keep_state = {"done": {rid: done[rid] for rid in runs},
                  "sessions": state.get("sessions") or {}}
    out = Path(out or root / f"{site['site']}_bundle_{time.strftime('%Y%m%dT%H%M%S')}.zip")
    manifest = {"schema": SCHEMA, "site": site["site"], "unit": site.get("unit", ""),
                "robot": site.get("robot", "ur5e"), "cell": site.get("cell", ""),
                "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "runs": runs, "files": {}}
    state_bytes = json.dumps(keep_state, indent=1).encode("utf-8")
    manifest["files"]["campaign/state.json"] = hashlib.sha256(state_bytes).hexdigest()
    for arc, src in files.items():
        manifest["files"][arc] = _sha(src)
    out.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
        for arc, src in files.items():
            z.write(src, arc)
        z.writestr("campaign/state.json", state_bytes)
        z.writestr("MANIFEST.json", json.dumps(manifest, indent=1))
    log(f"site bundle {out.name}: {site['site']}, {len(runs)} runs, "
        f"{sum(1 for a in files if a.startswith('vision/'))} with tool-camera tracking")
    return {"ok": True, "path": str(out), "site": site["site"], "runs": len(runs),
            "bytes": out.stat().st_size}


# ---------------------------------------------------------------------------
# import, at the publishing site
# ---------------------------------------------------------------------------

def _rename_run_file(src: Path, dst: Path, site: str, unit: str, new_id: str) -> None:
    """Copy a run file, rewriting its manifest line: new id, the site that recorded it."""
    with src.open("r", encoding="utf-8") as fi, dst.open("w", encoding="utf-8") as fo:
        first = fi.readline()
        doc = json.loads(first)
        m = doc["_manifest"]
        m["origin_run_id"] = m.get("origin_run_id") or m["run_id"]
        m["run_id"] = new_id
        m["site"] = site
        if unit:
            m["unit"] = unit
        fo.write(json.dumps(doc) + "\n")
        shutil.copyfileobj(fi, fo)


def import_bundle(bundle, root=".", log=print) -> dict:
    root = Path(root)
    bundle = Path(bundle)
    home = load_site(root)["site"]
    tmp = Path(tempfile.mkdtemp(prefix="sonair_bundle_"))
    try:
        with zipfile.ZipFile(bundle) as z:
            for n in z.namelist():
                if n.startswith(("/", "\\")) or ".." in Path(n).parts or ":" in n:
                    return {"ok": False, "error": f"unsafe path in the bundle: {n}"}
            z.extractall(tmp)
        try:
            man = json.loads((tmp / "MANIFEST.json").read_text(encoding="utf-8"))
        except (OSError, ValueError) as e:
            return {"ok": False, "error": f"no readable MANIFEST.json ({e})"}
        if man.get("schema") != SCHEMA:
            return {"ok": False, "error": f"not a site bundle (schema {man.get('schema')!r})"}
        site = clean_id(man.get("site"))
        if not site:
            return {"ok": False, "error": "the bundle names no site"}
        if site == home:
            return {"ok": False, "error": f"this bundle is from {site}, which is THIS site: "
                                          "its runs are already here"}
        bad = [arc for arc, h in (man.get("files") or {}).items()
               if not (tmp / arc).exists() or _sha(tmp / arc) != h]
        extra = [str(p.relative_to(tmp)).replace("\\", "/") for p in tmp.rglob("*")
                 if p.is_file() and p.name != "MANIFEST.json"
                 and str(p.relative_to(tmp)).replace("\\", "/") not in (man.get("files") or {})]
        if bad or extra:
            return {"ok": False, "error": "the bundle does not match its manifest -- "
                    + (f"{len(bad)} file(s) altered or missing, e.g. {bad[:3]}" if bad else "")
                    + (f"; {len(extra)} unlisted file(s), e.g. {extra[:3]}" if extra else "")}
        unit = str(man.get("unit") or "")
        stage = root / SITES_DIR / f".{site}.importing"
        if stage.exists():
            shutil.rmtree(stage)
        (stage / "bench_runs").mkdir(parents=True)
        (stage / "vision_logs").mkdir()
        (stage / "campaign").mkdir()
        rename = {}
        for rid in man.get("runs") or []:
            src = tmp / "runs" / f"{rid}.jsonl"
            if not src.exists():
                continue
            new_id = f"{site}{SEP}{rid}"
            rename[rid] = new_id
            _rename_run_file(src, stage / "bench_runs" / f"{new_id}.jsonl", site, unit, new_id)
            v = tmp / "vision" / f"{rid}.vtp.jsonl"
            if v.exists():
                shutil.copy2(v, stage / "vision_logs" / f"{new_id}.vtp.jsonl")
        st = json.loads((tmp / "campaign" / "state.json").read_text(encoding="utf-8"))
        st["done"] = {rename[r]: v for r, v in (st.get("done") or {}).items() if r in rename}
        (stage / "campaign" / "state.json").write_text(json.dumps(st, indent=1), encoding="utf-8")
        for sub in ("calib", "calibration"):
            if (tmp / sub).exists():
                shutil.copytree(tmp / sub, stage / sub)
        for f in ("site.json", "carrier.json"):
            if (tmp / f).exists():
                shutil.copy2(tmp / f, stage / f)
        (stage / "BUNDLE.json").write_text(json.dumps(
            {**{k: v for k, v in man.items() if k != "files"},
             "imported_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
             "bundle_file": bundle.name, "bundle_sha256": _sha(bundle)}, indent=1),
            encoding="utf-8")
        dest = root / SITES_DIR / site
        if dest.exists():
            old = root / SITES_DIR / f".{site}.replaced.{time.strftime('%Y%m%dT%H%M%S')}"
            dest.rename(old)
            shutil.rmtree(old, ignore_errors=True)
        stage.rename(dest)
        log(f"imported {site}: {len(rename)} runs"
            + (f", unit {unit}" if unit else ""))
        return {"ok": True, "site": site, "unit": unit, "runs": len(rename),
                "path": str(dest)}
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def list_sites(root=".") -> list[dict]:
    """Every imported site, with what its bundle said."""
    out = []
    for d in sorted((Path(root) / SITES_DIR).glob("*")):
        if not d.is_dir() or d.name.startswith("."):
            continue
        try:
            b = json.loads((d / "BUNDLE.json").read_text(encoding="utf-8"))
        except (OSError, ValueError):
            b = {}
        out.append({"site": d.name, "unit": b.get("unit", ""), "runs": len(b.get("runs") or []),
                    "created_utc": b.get("created_utc", ""), "imported_utc": b.get("imported_utc", ""),
                    "path": str(d)})
    return out


def site_of(run_id: str) -> str | None:
    """The importing site's prefix of a run id, or None for a home-site run."""
    return run_id.split(SEP, 1)[0] if SEP in run_id else None

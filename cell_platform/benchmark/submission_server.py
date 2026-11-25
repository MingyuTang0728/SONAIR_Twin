"""
submission_server.py -- the SONAIR evaluator, behind an API key.

The hidden half of the benchmark lives here and nowhere else: the targets of
the held-out cells. A team sends a zip; this checks its key, checks the zip
with the same pre-flight validator the team ran at home, scores it on the
hidden targets, writes a scorecard and puts it on the leaderboard. The team
gets the scorecard back; nobody ever gets the targets.

    python submission_server.py --port 8090 \\
        --task results/task_v2/public --answers results/task_v2/private

    # a team, with a key issued by:  python sonair_keygen.py issue --institution UCL --role team
    curl -H "Authorization: Bearer sonair_team_..." -H "Content-Type: application/zip" \\
         --data-binary @my_submission.zip http://<host>:8090/api/submit

Only keys of role "team" are accepted, and a team key opens nothing else: the
relay and the cell refuse it. Each key may score SUBMISSIONS_PER_DAY entries a
day -- a hidden test set that can be queried without limit stops being hidden.

Endpoints
    POST /api/submit           zip body -> {id, status}
    GET  /api/status/<id>      -> {status, headline, scorecard}
    GET  /api/leaderboard      -> every team's best scored entry, with the baselines
    GET  /scorecard/<id>       -> the scorecard page
"""
from __future__ import annotations

import argparse
import hashlib
import json
import queue
import threading
import time
import uuid
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
SUBMISSIONS_PER_DAY = 5
MAX_BYTES = 200 * 1024 * 1024


class Store:
    def __init__(self, root: Path, keys: Path):
        self.root = root
        self.keys = keys
        self.root.mkdir(parents=True, exist_ok=True)
        self.index_p = root / "index.json"
        self.lock = threading.Lock()
        try:
            self.index = json.loads(self.index_p.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            self.index = {}

    def save(self):
        tmp = self.index_p.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.index, indent=1), encoding="utf-8")
        tmp.replace(self.index_p)

    def key(self, token: str):
        """The team key's record, or None. Re-read every time: a revocation takes effect at once."""
        if not token:
            return None
        try:
            db = json.loads(self.keys.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return None
        h = hashlib.sha256(token.encode("utf-8")).hexdigest()
        for rec in db.get("keys", []):
            if rec.get("hash") != h or rec.get("role") != "team" or rec.get("revoked"):
                continue
            exp = rec.get("expires_at")
            if exp and datetime.fromisoformat(exp) < datetime.now(timezone.utc):
                return None
            return rec
        return None

    def used_today(self, key_id: str) -> int:
        day = time.strftime("%Y-%m-%d", time.gmtime())
        return sum(1 for s in self.index.values()
                   if s["key_id"] == key_id and s["received_utc"].startswith(day)
                   and s["status"] != "rejected")


class Evaluator(threading.Thread):
    """One submission at a time: the scores of record never race each other."""

    def __init__(self, store: Store, task: Path, answers: Path, site_dirs=()):
        super().__init__(daemon=True, name="evaluator")
        self.store, self.task, self.answers = store, task, answers
        self.site_dirs = [Path(d) for d in site_dirs]
        self.q: queue.Queue = queue.Queue()

    def run(self):
        import sys
        sys.path.insert(0, str(HERE))
        from sonair_benchmark import evaluate as ev
        while True:
            sid = self.q.get()
            with self.store.lock:
                rec = self.store.index[sid]
                rec["status"] = "scoring"
                self.store.save()
            d = self.store.root / sid
            try:
                card = ev.evaluate(d / "submission.zip", self.task, self.answers)
                ev.write_scorecard(card, d)
                if card.get("ok"):
                    status = "scored"
                    extra = {"headline": card["headline"]["value"],
                             "effort_tail": card["tracks"]["current"]["tail"]["gcr_p95"],
                             "effort_all": card["tracks"]["current"]["all"]["gcr_p95"],
                             "gyro_tail": card["tracks"]["gyro"]["tail"]["gcr_p95"],
                             "gyro_all": card["tracks"]["gyro"]["all"]["gcr_p95"],
                             "vis_tail": (card["tracks"].get("vis") or {}).get("tail", {}).get("gcr_p95"),
                             "cross_effort_tail": ((card.get("cross_site") or {}).get("tracks", {})
                                                   .get("current", {}).get("tail", {}).get("gcr_p95")),
                             "robust_drop": card["tracks"]["current"]["robustness"]["drop"],
                             "calibration": (card["tracks"]["current"].get("calibration") or {})
                             .get("coverage_90"),
                             "entry": card["entry"]}
                else:
                    status, extra = "rejected", {"problems": card.get("problems", [])[:20]}
            except Exception as e:                  # noqa: BLE001
                status, extra = "error", {"problems": [f"{type(e).__name__}: {e}"]}
            with self.store.lock:
                rec.update(status=status, scored_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                                                   time.gmtime()), **extra)
                self.store.save()
                self.write_board()

    def write_board(self):
        """Each team's best scored entry -- the public face of the hidden matrix."""
        best = {}
        for sid, s in self.store.index.items():
            if s["status"] != "scored" or s.get("headline") is None:
                continue
            k = s["institution"]
            if k not in best or s["headline"] > best[k]["headline"]:
                best[k] = {**{x: s.get(x) for x in ("headline", "effort_tail", "effort_all",
                                                    "gyro_tail", "gyro_all", "vis_tail",
                                                    "cross_effort_tail", "robust_drop",
                                                    "calibration", "scored_utc")},
                           **(s.get("entry") or {}), "institution": k, "id": sid,
                           "scorecard": f"/scorecard/{sid}", "kind": "submission"}
        out = {"updated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
               "entries": sorted(best.values(), key=lambda r: -r["headline"])}
        p = self.store.root / "leaderboard.json"
        p.write_text(json.dumps(out, indent=1), encoding="utf-8")
        # beside the site, so the benchmark page shows submissions with the
        # baselines -- only the site folders this server was started with
        for site in self.site_dirs:
            if site.exists():
                (site / "submissions.json").write_text(json.dumps(out, indent=1), encoding="utf-8")


def make_handler(store: Store, evaluator: Evaluator):
    class H(BaseHTTPRequestHandler):
        server_version = "SONAIR-evaluator/1"

        def _json(self, code, obj):
            body = json.dumps(obj).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, fmt, *a):         # quieter, and never logs the key
            pass

        def do_POST(self):
            if self.path.rstrip("/") != "/api/submit":
                return self._json(404, {"error": "not found"})
            n = int(self.headers.get("Content-Length") or 0)
            if n <= 0 or n > MAX_BYTES:
                self.close_connection = True
                return self._json(413, {"error": f"send a zip of at most {MAX_BYTES >> 20} MB"})
            # the body is read before any refusal: a client still sending when
            # the connection is closed sees "connection aborted", not the reason
            body = self.rfile.read(n)
            token = (self.headers.get("Authorization") or "").removeprefix("Bearer ").strip()
            rec = store.key(token)
            if rec is None:
                return self._json(401, {"error": "a valid team API key is required"})
            with store.lock:
                used = store.used_today(rec["id"])
            if used >= SUBMISSIONS_PER_DAY:
                return self._json(429, {"error": f"{SUBMISSIONS_PER_DAY} submissions a day per "
                                                 f"key; try again tomorrow (UTC)"})
            if body[:2] != b"PK":
                return self._json(400, {"error": "the body must be a .zip file"})
            sid = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + "-" + uuid.uuid4().hex[:6]
            d = store.root / sid
            d.mkdir(parents=True)
            (d / "submission.zip").write_bytes(body)
            with store.lock:
                store.index[sid] = {"key_id": rec["id"], "institution": rec["institution"],
                                    "received_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                                    "bytes": n, "sha256": hashlib.sha256(body).hexdigest(),
                                    "status": "queued"}
                store.save()
            evaluator.q.put(sid)
            return self._json(202, {"id": sid, "status": "queued",
                                    "status_url": f"/api/status/{sid}",
                                    "left_today": SUBMISSIONS_PER_DAY - used - 1})

        def do_GET(self):
            p = self.path.split("?")[0].rstrip("/")
            if p == "/api/leaderboard":
                f = store.root / "leaderboard.json"
                return self._json(200, json.loads(f.read_text(encoding="utf-8"))
                                  if f.exists() else {"entries": []})
            if p.startswith("/api/status/"):
                sid = p.rsplit("/", 1)[-1]
                s = store.index.get(sid)
                if s is None:
                    return self._json(404, {"error": "no such submission"})
                out = {k: v for k, v in s.items() if k not in ("key_id",)}
                if s["status"] == "scored":
                    out["scorecard"] = f"/scorecard/{sid}"
                return self._json(200, out)
            if p.startswith("/scorecard/"):
                sid = p.rsplit("/", 1)[-1]
                f = store.root / sid / "scorecard.html"
                if sid not in store.index or not f.exists():
                    return self._json(404, {"error": "no scorecard"})
                body = f.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            return self._json(404, {"error": "not found"})

    return H


def serve(port, task, answers, root, keys, site_dirs=()):
    store = Store(Path(root), Path(keys))
    ev = Evaluator(store, Path(task), Path(answers), site_dirs)
    ev.start()
    httpd = ThreadingHTTPServer(("0.0.0.0", port), make_handler(store, ev))
    print(f"SONAIR evaluator on :{port}  task={task}  answers={answers}")
    return httpd, store, ev


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--task", default="results/task_v2/public")
    ap.add_argument("--answers", default="results/task_v2/private")
    ap.add_argument("--store", default="submissions")
    ap.add_argument("--keys", default="sonair_keys.json")
    ap.add_argument("--site", action="append",
                    help="a site folder to write submissions.json into (repeatable); "
                         "default: site/ and site_build/site/ beside this file")
    a = ap.parse_args()
    sites = a.site or [HERE / "site", HERE / "site_build" / "site"]
    httpd, _, _ = serve(a.port, a.task, a.answers, a.store, a.keys, sites)
    httpd.serve_forever()


if __name__ == "__main__":
    main()

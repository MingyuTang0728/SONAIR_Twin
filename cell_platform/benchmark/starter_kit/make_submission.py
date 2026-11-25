"""
SONAIR starter kit: a complete, valid submission in about forty lines.

It learns one thing from the train runs -- a friction correction to the
reference simulator's current -- and passes the simulator's angular rate
through unchanged. That scores roughly where the F1 baseline does. Replace
`fit` and `predict` with your own model; keep the file format.

    python make_submission.py --task sonair_task_v2_public --out my_submission
    python -m sonair_benchmark validate my_submission --task sonair_task_v2_public

Needs only numpy.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def joint(head):
    """The joint whose current is scored: every run says which, in its header."""
    return int(head["_input"].get("effort_joint", 2))


def read(path):
    lines = Path(path).read_text(encoding="utf-8").splitlines()
    return json.loads(lines[0]), [json.loads(x) for x in lines[1:] if x.strip()]


def column(rows, side, key, k=None, fill=0.0):
    out = []
    for r in rows:
        v = r[side].get(key)
        out.append(fill if v is None else (v[k] if k is not None else v))
    return np.asarray(out, dtype=float)


def s0_current(head, rows):
    m = head["_input"]["effort_map"]
    return m["a"] * column(rows, "sim", "tau", joint(head)) + m["b"]


def features(head, rows):
    qd = column(rows, "cmd", "qd", joint(head))
    return np.column_stack([np.tanh(qd / 0.01), qd])


def fit(task):
    X, y = [], []
    for p in sorted((task / "train").glob("*.input.jsonl")):
        head, rows = read(p)
        _, tgt = read(str(p).replace(".input.", ".target."))
        cur = np.array([np.nan if t["current"] is None else t["current"] for t in tgt])
        ok = np.isfinite(cur)
        X.append(features(head, rows)[ok])
        y.append((cur - s0_current(head, rows))[ok])
    return np.linalg.lstsq(np.vstack(X), np.concatenate(y), rcond=None)[0]


def predict(head, rows, w):
    gyro = [r["sim"]["gyro"] or [0.0, 0.0, 0.0] for r in rows]
    return {"run_id": head["_input"]["run_id"], "t": [r["t"] for r in rows],
            "current": (s0_current(head, rows) + features(head, rows) @ w).round(5).tolist(),
            "gyro": gyro}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True, help="the unzipped public package")
    ap.add_argument("--out", default="my_submission")
    a = ap.parse_args()
    task, out = Path(a.task), Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    w = fit(task)
    with (out / "predictions.jsonl").open("w", encoding="utf-8") as fh:
        for p in sorted((task / "test").glob("*.input.jsonl")):
            head, rows = read(p)
            fh.write(json.dumps(predict(head, rows, w)) + "\n")
    (out / "meta.json").write_text(json.dumps({
        "name": "Starter: S0 + friction", "team": "your team",
        "description": "Coulomb and viscous friction added to the reference current",
        "runtime_s_per_run": 0.01, "params": 2}, indent=1), encoding="utf-8")
    print(f"wrote {out}/predictions.jsonl and meta.json")


if __name__ == "__main__":
    main()

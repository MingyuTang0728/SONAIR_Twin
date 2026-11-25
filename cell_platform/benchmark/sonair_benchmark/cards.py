"""
cards -- the Task, Dataset and Evaluation Cards, written from the numbers.

Each publish rewrites them from the task specification, the dataset that was
actually scored and the board that was actually computed, so a card can never
describe a benchmark other than the one on the site.
"""
from __future__ import annotations

import time
from pathlib import Path


def _f(v):
    return " - " if v is None else f"{v:+.3f}"


def task_card(spec: dict, board: dict) -> str:
    e = board.get("entries") or []
    rows = "\n".join(f"| {r['name']} | {r.get('track', '')} | {_f(r['headline'])} | "
                     f"{_f(r['effort_all'])} | {_f(r['gyro_tail'])} | {_f(r['gyro_all'])} |"
                     for r in e)
    return f"""# Task Card - SONAIR {spec['version']}

**Question.** {spec['question']}

**Why it matters.** Safety-critical robot behaviour is decided in rare moments
 -  starts, stops, reversals, heavy loads - of which real data holds very few
examples. If a model can turn simulation into what the real robot measures in
exactly those moments, simulation can supply the training data the real world
cannot. SONAIR measures how far that is true today. It is an unsolved
problem: the scores below are meant to be low.

## Input (per run, 125 Hz)
- `cmd`: commanded joint angles and speeds, the controller's own torque model.
- `sim`: the reference simulator S0 driven by the same command - joint angles,
  tool position, joint torques, IMU angular rate and acceleration (IMU axes).
- per run: condition (elbow speed, arm pose, trajectory, payload), the IMU's
  measured mounting, the published effort map.

## Output (on the input's timestamps)
- `current` - real elbow motor current, A (primary track)
- `gyro` - real IMU angular rate, rad/s (second track)
- optional `accel`, optional `sigma` (per-sample uncertainty)

## Splits
- **train**: {spec['splits']['train']}
- **test**: {spec['splits']['test']}
- **tail (headline)**: {spec['splits']['tail']}
- **stress**: {"; ".join(f"{k} - {v}" for k, v in spec['splits']['stress'].items())}

## Score
{spec['metric']}. **Headline: {spec['headline']}.**

## Baselines (scored like any entry)
| Entry | Track | **Effort tail** | Effort all | Gyro tail | Gyro all |
|---|---|---|---|---|---|
{rows}

## Out of scope
Tool position is reported as a kinematics check only: copying the command onto a
rigid model already closes ~98% of it, because the robot's own controller tracks
its command so closely.
"""


def dataset_card(dataset: dict, spec: dict, board: dict) -> str:
    return f"""# Dataset Card - SONAIR {spec['version']}

- **Platform**: a 6-DoF collaborative industrial arm with a tool-mounted inertial sensor and RGB-D
  camera; the same robot model at every site, each site its own robot unit.
- **Sites in this release**: {", ".join(f"{k} ({v} runs{', training site' if k == dataset.get('home_site') else ', cross-site test'})" for k, v in (dataset.get('sites') or {}).items()) or "one"}.
- **Signals**: joint angles, speeds, motor currents, the controller's torque model and tool pose
  at 125 Hz; a 9-axis IMU on an instrumented carrier at 100 Hz (resampled onto the 125 Hz clock);
  the tool's pose seen by the tool camera against a fixed ChArUco board, up to 30 Hz, where tracked
  ({(dataset.get('vision') or {}).get('runs', 0)} runs; vision gate: {(dataset.get('vision') or {}).get('verdict', 'not measured')}).
  Video of every run is kept as evidence and not released.
- **Design**: one joint (joint 3) at 6 speeds (0.2–0.9 rad/s) × 3 arm poses × 3 motion types = 54
  operating conditions, repeated over sessions with one deliberate sensor refit. 18 conditions are
  held out, chosen once with a fixed seed before any data existed.
- **In this release**: {dataset.get('runs', {})} runs over {dataset.get('cells', '?')} cells;
  sessions {dataset.get('sessions', {})}.
- **Calibration shipped**: per-session IMU mounting and timing; the effort map; the carrier's
  mass and centre of mass.
- **Released**: train runs with their targets; test inputs only. **Kept**: the targets of the
  held-out cells (with the evaluator) and the video.
- **Integrity**: every file has a SHA-256 in `SHA256SUMS.json`; only runs the acquisition
  system read back and accepted are included; superseded and rehearsal runs never are.
- **Licence**: data CC BY 4.0; code MIT. **Ethics**: no human subjects; no audio; machine IDs
  pseudonymised.
- **Known limits**: acceleration is provisional until the accelerometer calibration meets its
  0.5% target; the camera-seen tool position ranks only once the vision gate passes; the colour
  camera has a rolling shutter, so fast moves are where its frames are weakest.
"""


def eval_card(spec: dict, board: dict) -> str:
    return f"""# Evaluation Card - SONAIR {spec['version']}

**What is computed.** Per test run and per track, the per-sample error of the entry and of
S0 against the real target - |Δ current| in A; ‖Δ angular rate‖ in rad/s. GCR = 1 − p95(entry)
/ p95(S0) (and the same at the median), averaged over runs. Tail scores use only the tail
samples. Robustness is the same score on each stressed input. Calibration, when `sigma` is
given, is the share of samples whose error lies inside ±1.645σ (0.90 is calibrated).

**Headline.** {spec['headline']}.

**Fairness.** S0, the reference, is computed by the evaluator from the public inputs with the
same code as every entry. Baselines are made from the public package only and submitted as
entries. Anything fitted (the effort map, baseline coefficients) uses the train split only.

**Integrity.** The held-out targets never leave the evaluator. Submissions use a team API key
(a key that grants nothing else), at most 5 scored a day; the same validator runs at home and on
the server; the evaluator ships as a container so a score can be reproduced bit for bit.

**Reporting.** Every scorecard lists every held-out cell, the worst included.

**Versioning.** Changing the task, the splits or the metric is a new version; scores across
versions are not compared.

_Generated {time.strftime('%Y-%m-%d %H:%M UTC', time.gmtime())}._
"""


def write_cards(out_dir, spec: dict, dataset: dict, board: dict) -> list:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    files = {"TASK_CARD.md": task_card(spec, board),
             "DATASET_CARD.md": dataset_card(dataset, spec, board),
             "EVAL_CARD.md": eval_card(spec, board)}
    for n, t in files.items():
        (out / n).write_text(t, encoding="utf-8")
    return list(files)

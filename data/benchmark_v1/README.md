# benchmark-v1 real runs (public part)

Real UR5e runs recorded on the UoN cell by the cell platform (folder SONAIRCrossite33), in the platform's run format
`sonair-run-1` (JSON Lines, 125 Hz, line 0 is `{"_manifest": ...}`). Built by `tools/build_release_v1.py`; every
number below is in `release.json` and `index.csv`.

| Split | Folder | Runs | Cells | What each file holds |
|---|---|---|---|---|
| train (published cells) | `train/` | 180 | 36 | the original run file byte for byte, gzip |
| test (held-out cells) | `test_inputs/` | 90 | 18 | inputs only: `t`, `controller_t`, `target_q`, `target_qd`, `target_moment` |

A cell is (joint speed level, arm configuration, trajectory type). The 18 held-out cells are the ones whose manifest
notes say `HELD-OUT cell`; they are listed in `release.json`.

## Sessions

Sessions follow the platform's own numbering ("campaign session N" in each manifest). All three were recorded on
2026-10-09 (start times of the first and last run, UTC):

| Label | Campaign session | Repeats | Runs | Recorded | Role |
|---|---|---|---|---|---|
| benchmark-v1 | 0 | r00, r03 | 108 | 16:48 to 17:07 | scored on the benchmark site (72 published, 36 held-out) |
| trial-1 | 1 | r01, r04 | 108 | 10:33 to 10:52 | trial session, kept for repeatability studies |
| trial-2 | 2 | r02 | 54 | 14:59 to 15:09 | trial session, kept for repeatability studies |

Scores on the benchmark site use the benchmark-v1 session only.

## What is withheld, and why

- The measured channels of held-out runs (joints, currents, tool pose, IMU, cameras): they are the hidden targets.
  Anyone with them could fit to the test set, so they never go into a public repository.
- Raw controller and IMU logs (`ur_logs/`, `imu_logs/`): they cover the held-out periods too.
- API keys and submission secrets of the evaluation server.

Absolute paths to raw logs on the lab PC were removed from the metadata files.

## Files

- `index.csv`: one row per run (split, session, cell, rows, duration, SHA-256 of the original file and of the released
  file). `gunzip -c train/<run>.jsonl.gz | sha256sum` gives `sha256_original`.
- `calib/`: IMU-to-flange calibration and lag per session, accelerometer calibration, quaternion convention.
- `campaign/state.json`: the taught poses and the run list of the benchmark-v1 campaign.
- `site/leaderboard.json`: the benchmark site's leaderboard (baselines K0, B1, S0 on the held-out cells).
- `cards/`: dataset, task and evaluation cards of the platform.
- `analysis/`: per-run analysis tables, published cells only.
- `carrier.json`, `site.json`, `embodiments/ur5e.json`: carrier mass properties, site and robot description.

## Reading a run

```python
from sonair import realdata
man, A = realdata.load("data/benchmark_v1/train/v0p20_extended_contour_r00.jsonl.gz")
runs = realdata.load_dir("data/benchmark_v1/train")       # .jsonl and .jsonl.gz
```

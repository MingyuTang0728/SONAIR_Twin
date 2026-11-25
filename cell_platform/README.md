# cell_platform: the parts of the cell platform this repository runs on

Copied on 2026-11-25 from the cell platform folder (SONAIRCrossite33). The robot link is kept byte for byte; the rest
is the platform's benchmark package with the sonair-task/3 patch applied.

| Folder | What | Changed? |
|---|---|---|
| `ur/` | robot link: `ur_control` (URScript 30002, dashboard 29999, RTDE input), `ur_telemetry` (RTDE reader process), `ur_bridge_ext` (the message handler the console calls), `ur_jog` (hold-to-jog with watchdog, step), `ur_kin`, `scan3d` | **No. Locked.** `LOCKED.sha256` holds the hashes of the originals; `tests/test_console.py` and the console (at every start) check them |
| `calib/` | `imu_align`, `accel_cal`, `carrier`, `handeye`, `vision_track` | text only: em dashes in comments and messages replaced |
| `benchmark/` | `sonair_benchmark` (task, evaluator, cards, site, release), `submission_server`, `sonair_keygen`, `evaluator/Dockerfile`, `starter_kit/`, and the helpers it imports (`sim_mujoco`, `twin`, `campaign_runner`, `ident_set`, `sim_cell`) | `patches/sonair_benchmark_task3.patch` applied (task/3 rules as options); em dashes in comments and messages replaced |

The modules import each other by top-level name, as they did in the platform folder. `cell_platform/paths.py` puts
`ur/`, `calib/` and `benchmark/` on `sys.path` in that order; nothing in them was edited to change imports.

## Why the robot link is locked

It is the code that was debugged against the real controller (RTDE recipe negotiation, the 30003 fallback, the jog
watchdog, dashboard replies). The new console calls it exactly as the old console did, through
`ur_bridge_ext.handle_message`, and adds its own rules around that call, never inside it. If a locked file has to
change, change it in the platform first, test it on URSim and on the cell, then copy it here and regenerate
`LOCKED.sha256` in the same commit:

    cd cell_platform/ur && sha256sum ur_control.py ur_telemetry.py ur_bridge_ext.py ur_jog.py ur_kin.py scan3d.py > LOCKED.sha256

One thing the lock keeps from the platform on purpose: the reader falls back to the 30003 stream when RTDE is not
available. The console records only RTDE: a session with any 30003 packet fails QC.

## Not copied

The old console page and its bridge (`SONAIR_Console.html`, `multimodal_bridge.py`, `bench_agent.py`,
`lab_services.py`, `sensor_hub.py`), inspection, projector and depth, multi-view scanning, relay and FTP services,
the raw logs, videos and `sonair_keys.json` (API keys: never in a repository).

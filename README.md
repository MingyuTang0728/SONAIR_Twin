# SONAIR Twin

The dynamics side of the SONAIR sim-to-real benchmark (University of Nottingham UR5e inspection cell): the same
command runs on the real arm and in every simulator, is recorded in one format and scored by one piece of code.

* **MuJoCo twin** (the single reference simulator): `sonair/sim/mujoco_twin.py`, baselines B0 to B2.
* **Jetson acquisition**: RTDE at 500 Hz, LSM6DSV16X IMUs read from their FIFO with hardware timestamps, one host clock,
  session QC, export to the cell platform's run format. Runs without hardware in `--mock` mode.
* **Real-data audit** of benchmark-v1 (108 runs): data quality, real-real floors, simulator scores with bootstrap
  intervals, and two scoring-rule problems with their fix (`patches/`).
* **Cell console** (`console/`): robot, IMUs, sessions, campaigns, data and scores in one browser page, on top of the
  platform's robot link kept unchanged (`cell_platform/ur`, hash-locked). `python -m console --mock` runs it with no robot.
* **benchmark-v1 real data, public part** (`data/benchmark_v1/`): 180 runs of the 36 published cells, the 90 runs of
  the 18 held-out cells as command inputs only, index with hashes, calibration and campaign files.

## Quick start

```
pip install -r requirements.txt
python -m pytest -q                                   # no hardware needed
python -m sonair commands                             # command set (deterministic)
python -m sonair sim                                  # MuJoCo baselines on the command set
python -m sonair jetson-record --mock --cmd exc_gripper_open_v0.3_s1 --speed 1.0   # whole acquisition chain, simulated
python -m console --mock                              # the cell console with a stand-in controller: http://127.0.0.1:8765/
```

On the cell (Jetson Orin Nano, ur_rtde, smbus2, pyrealsense2):

```
python -m sonair jetson-imu-probe --bus 1 --addr 0x6b                  # bench check of one IMU
python -m sonair jetson-record --robot 192.168.1.10 --imu tool:1:0x6b --cmd <id> --speed 0.5 \
       --cell 0.50/mid_workspace/contour --rep 0 --unity <workstation>:5005
python -m sonair session-export runs_jetson/<run_id> --platform-out <SONAIRCrossite33>/bench_runs --lag-ms <imu lag>
```

## Results on the real benchmark-v1 data

`python -m sonair audit-v1 --runs <bench_runs> --cal <calib/imu_cal.json>` writes `reports/v1_real/`.
Held-out cells, 36 runs, elbow current, GCR at p95 against S0 (bootstrap 95 % intervals in `audit_v1.json`):

| entry | all samples | tail samples |
|---|---|---|
| CM, controller's own torque model | 0.06 | 0.20 |
| F1, S0 + Coulomb and viscous friction | 0.38 | 0.43 |
| B2 twin through the published S0 map | -0.12 | -0.11 |
| B2 twin, its own map | 0.04 | 0.15 |
| B2 + Coulomb and viscous friction | 0.63 | 0.67 |
| ceiling set by the real-real floor | 0.88 | 0.89 |

Gyro track, K0 (rigid kinematics of the command): GCR 0.56 with the current target rows, 0.82 when the target is
the IMU on its own timestamps minus the measured 12 ms lag. Real-real floors: current 0.17 A, gyro 0.026 rad/s,
tool position 0.07 mm (p95).

## Layout

See `CLAUDE.md`. Models: `assets/universal_robots_ur5e` is the MuJoCo Menagerie UR5e (Apache-2.0, see its LICENSE).
Licence of the rest of the repository: to be set by the owner.

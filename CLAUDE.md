# SONAIR twin: notes for Claude Code sessions

## Working with Mingyu
- Reply in Chinese. Never use the em dash character in any text (code comments included).
- Code, comments, UI text and repository docs are English (decided 2026-11-24). Documents for people (guides, plans,
  reports) are Word .docx, not markdown; Chinese docx go under docs/ with a `_zh` suffix.
- Show numbers that were actually computed; say plainly what was not run (no robot, no Isaac Sim, no Jetson here).

## What this repo is
The dynamics side of the SONAIR sim-to-real benchmark for the UoN UR5e inspection cell ("one camera tool, two scan
modes": 2F-85 gripper, camera tool with D405, collar with D435i, LSM6DSV16X IMUs, specimens S2 and S4, Jetson Orin
Nano for acquisition). Same command -> real UR5e and every simulator -> same run format -> same scorer.

It grew out of the cell platform (folder SONAIRCrossite33), which owns the benchmark definition (sonair-task/2) and
recorded the 270 benchmark-v1 runs. The parts still used are copied into cell_platform/ (robot link locked, benchmark
package with the task/3 patch applied); the public part of the 270 runs is in data/benchmark_v1/. This repo adds the
MuJoCo twin, the Jetson acquisition chain, the real-data audit, the task/3 rule changes and the new cell console, and
exports every session in the platform's run format (sonair-run-1).

## Decisions (with the evidence in reports/v1_real/audit_v1.json)
- ONE reference simulator: MuJoCo (S0 = Menagerie UR5e as shipped). Isaac Sim, Unity PhysX or anything else enters
  as a scored Track A entry, never as a second reference (two references = two GCR denominators).
- Unity is a viewer and authoring tool (sequences, live mirror with a commanded ghost, camera renders, XR). It never
  commands the robot and is not the reference simulator.
- Robot I/O: RTDE only (refuse the 30003 fallback), 500 Hz via ur_rtde's recording thread (jetson-record) or the
  platform link's own packets (console); motion via
  sonair.real.executor (servoJ of a checked command) or a pendant program; every sensor stream in its own file with
  its own timestamps, mapped to the host clock by sonair.jetson.clock.ClockMap.
- Scoring rules proposed as sonair-task/3 (patches/sonair_benchmark_task3.patch): torque -> current map fitted per
  simulator entry (the S0 map has the wrong physical sign), gyro target from the IMU's own timestamps minus the
  measured lag (the sample-and-hold rows turned half of the gyro gap into a timing artefact).
- The robot link is cell_platform/ur, byte-identical to the debugged platform files (LOCKED.sha256, checked by the
  tests and at console start). Never edit it here; new behaviour goes around ur_bridge_ext.handle_message (console/agent.py).
- The console (console/) replaces the old SONAIR_Console.html: one port for UI and WebSocket, interlock (manual motion
  refused while a command executes), one STOP, sessions recorded from the link's own 500 Hz packets (RTDE only),
  commands only through sonair.real.executor, campaigns step by step. Mock mode uses console/mockur.py on the
  controller's ports so the unchanged link is what gets tested.
- The console is the one-stop "SONAIR Lab" (2026-11-25, at the owner's request): the old console's robot controls are
  kept as they were (two pads, precise steps, keyboard, gamepad, power, programs, tool, speed, joint data, I/O, the
  live arm with held FREEDRIVE and the jog dock). Layout: tabs on top, the live arm stage left of every page with a
  divider the operator sizes freely (drag or wheel), tool drawer (move, gripper, camera, session) under the arm. The UI
  stays generic: no project-specific workflows, budgets or plan checklists in the pages. Around them: RTDE interface (read only, the link's own
  packets), D405/D435i cameras, 2F-85 gripper (URCap socket 63352), kit sequences, calibrations (platform calib
  modules), data analysis with replay, the benchmark scoreboard. The 3D twin is built in the browser from the Unity
  project's own scene data (console/lab.py serves it read-only); the Unity package stays the authoring tool.
  Gripper events are NOT synchronised with executor trajectories yet (M2). Calibration and gripper commands are
  behind the same interlock and STOP as manual motion.
- Public data never holds hidden targets: held-out cells go out as inputs only (tools/build_release_v1.py); raw logs,
  videos and sonair_keys.json are never committed. The repo is public.

## Conventions (do not change without bumping the schema)
- World = UR controller base frame B, metres, z up. MuJoCo Menagerie world == B (q=0 -> tool0 (-0.817,-0.234,0.063)).
- tool0 = flange = MuJoCo site attachment_site, TCP offset 0. Joint order = pendant order, radians.
- Commands are versioned in git: commands/<id>/command.json + knots.parquet (50 Hz), expanded to 500 Hz with a
  clamped cubic spline by sonair.commands.load(). Regenerate with `python -m sonair commands` (deterministic seeds).
- Runs: runs/<run_id>/meta.json + joints.parquet, schema sonair.run/0.1 (sonair/runlog.py). Jetson sessions:
  runs_jetson/<run_id>/ (schema sonair.session/0.1, sonair/jetson/session.py). Neither is committed.
- Payloads: config/payloads.json from config/masses.json (CAD solid masses until weighed).
- Baselines: B0_kinematic (Unity), B1_menagerie_pd (= S0 servo), B2_pd_invdyn; B2F adds Coulomb + viscous friction.
- Gap metrics in sonair/bench/metrics.py; the real-data audit in sonair/bench/v1_audit.py.

## Layout
console/ agent.py (server, interlock, STOP, sessions, campaigns, HTTP routes), recorder.py, jobs.py (executor, plans),
  data.py (runs, channels, scoreboard data), rtde.py (RTDE monitor, captures), cameras.py (RealSense + mock render),
  gripper.py (2F-85 URCap socket + mock), calibration.py (hand-eye, six-face, IMU mount, archive), lab.py (Unity scene
  served to the twin, the rig's kinematics in Python), mockur.py (stand-in controller and gripper);
  web/ index.html, css/sonair.css, js/ (core, main, jog, dock, twin, pages/*.js), vendor/ (three.js 0.160, fonts, licences);
  run: python -m console [--mock]
cell_platform/ ur/ (locked robot link), calib/, benchmark/ (sonair_benchmark + helpers), paths.py
data/benchmark_v1/ train/ (published cells, original bytes, gzip), test_inputs/ (held-out inputs), index.csv, release.json
tools/build_release_v1.py (rebuilds data/benchmark_v1 from the platform folder)
sonair/cell.py, payload.py, commands.py, kin.py, collision.py, runlog.py, realdata.py (reader of sonair-run-1, .jsonl or .gz)
sonair/sim/mujoco_twin.py, sonair/sim/isaac_replay_template.py (untested template)
sonair/real/executor.py (servoJ 500 Hz + RTDE recording, --mock), sonair/real/rtde_logger.py (read only)
sonair/jetson/ clock.py, lsm6dsv16x.py (FIFO + timestamps), session.py (recorder, --mock), qc.py, export.py
sonair/bench/ metrics.py, report_m1.py, v1_audit.py
patches/ changes proposed to the cell platform; tests/ (pytest, no hardware needed)

## Milestones
M0 repo (done)  M1 dynamics loop in MuJoCo + executor (done in sim)  M1.5 audit of benchmark-v1 real data (done)
M2 Jetson bring-up: imu probe, bench QC PASS, URSim dry run, low-speed pilot
M3 task/3 adopted in the platform; E1 identification incl. payload step (drive constant), friction, inertia (B3)
M4 campaign v2 with the kit payloads, frozen poses, sessions on different days, thermal sessions
M5 second site decision (UCL) and Task 2 decision   M6 release v1.0 (DOI, Croissant + RAI, hidden test, cards)
M7 paper and maintenance

## Safety rules for anything that moves the robot
URSim first. Executor defaults: speed x0.5, refuses peak joint speed > 0.6 rad/s, refuses commands that failed the
collision check, checks safety mode every 100 ms. The collision check is coarse (boxes, no self-collision).
Nothing in Unity or in the live link can move the robot. The console adds: manual motion refused while a command
executes, STOP (Esc) that stops campaign, executor, jog and robot, and a confirmation before every command or campaign.

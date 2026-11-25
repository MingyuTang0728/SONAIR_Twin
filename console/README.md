# SONAIR Lab console

One process serves the browser UI, its WebSocket, the camera frames and the Unity scene on one port. Plain Python
(asyncio, websockets) and plain JavaScript modules: no build step, no internet access needed in the lab (three.js and
the fonts are vendored under `web/vendor/`).

```
python -m console --mock                          # no robot: stand-in controller, gripper, IMU and cameras on 127.0.0.1
python -m console --robot <URSim IP>              # URSim first
python -m console --robot 192.168.1.10            # the cell
python -m console --robot 192.168.1.10 --bind 0.0.0.0     # reachable from the lab network: prints a URL with a token
```

Open `http://127.0.0.1:8765/`. Keys: `1` to `9`, `0`, `B`, `L` switch pages, `Esc` is STOP. With keyboard jog switched
on (Robot page) the arrow keys, `Q`/`E`, `A`/`D`, `Shift` and `Space` drive the arm instead.

## Layout of the page

* **Header**: mode (MOCK CELL, CELL), lamps for the robot link, the arm, the gripper, the cameras, the IMUs and the
  session, the clock, light or dark screen, and the STOP button.
* **Run bar**: what the cell is doing (job, step, progress, elapsed, recording), on every page.
* **Rail**: the pages, grouped Cell, Experiment, Analysis, System.
* **Live arm column**, on every page and the only place the 3D view lives, so the arm is in sight whatever the page:
  the twin (measured arm, commanded arm as a ghost; sequence previews and run replays play here too, labelled),
  tool position, joints, tracking error (commanded minus measured, RMS over 5 s), controller state and the held
  FREEDRIVE button. `Wide` widens it (remembered per page; wide by default on Robot and Kit sequences).
* **Tool drawer** at the foot of that column, on every page: Move (the two pads, frame, speed), Gripper (width, open,
  27 mm, close, stop), Camera (D435i or D405, colour or depth) and Session (state, passive start, stop). Experiments
  need several devices at once; the drawer keeps them next to the arm instead of on other pages. Buttons marked with a
  tool name on the Overview workflows and the experiment pages open it.
* **Navigation** follows the work: Prepare the cell, Run experiments (Calibrate, Record, Campaign, Kit sequences),
  Analyse, System. The Overview lists each experiment of the plan with its steps and the tools it needs.

## Pages

| Page | What it is for |
|---|---|
| Overview | ready-to-record checklist, one tile per device, the twin view large, campaign progress, data counts, robot link address |
| Robot | the cell console's controls: two pads (across the table, up and rotate), precise steps, table or tool axes, speed, keyboard, gamepad; power and recovery; programs on the pendant; payload (kit states), TCP, force sensor zero, tool voltage; speed slider; Unity mirror; joint data with a chart; electrical and I/O |
| RTDE | the robot data interface: the recipe the controller granted (type, unit, latest value of every field), packet timing on both clocks (rate, period, missing cycles, host jitter, transport delay, a period histogram, packets per source), up to 8 fields plotted at the full rate, raw CSV captures of every field |
| Cameras | D405 (camera tool) and D435i (collar): colour and depth, distance at a click, intrinsics, frame numbers and timestamp domain, the calibration board found live, a saved view with its pose |
| Motion sensors | LSM6DSV16X units (I2C bus, address, output rate), live readings and charts, the still bench check (gravity, gyro bias and noise, both clock maps) |
| Gripper and sequences | the 2F-85 (connect, activate, open, close, position, speed, force); the six kit sequences from the Unity package previewed in the twin with phases, gripper events, payload schedule and scan windows; run one as a recorded session |
| Calibrate | hand-eye for the collar camera (board, live detection, planned poses moved one by one or automatically, solve, save), accelerometer six faces, IMU mounting and lag from a recorded session, what the next session is recorded with, the archive of every run |
| Record | one session: a checked command through the executor or a passive recording; QC; export |
| Campaign | plans from the versioned commands (speed, repeats, settle time, presets); one step at a time or auto-advance |
| Data | sessions here and the released benchmark-v1 runs: any channel of any joint, a second run overlaid, QC and manifest, replay in the twin with the chart cursor following |
| Benchmark | tracks (A simulator fidelity, B sim-to-real correction) by channel (effort, gyro, vision, acceleration, position) with role, reference, floor, S0 error, best entry and ceiling; every effort entry with its bootstrap interval; scores by speed; the gyro rule change; the E2 condition grid (runs, floors, held-out cells); how an entry is scored; collection hours against the 300 h budget; release readiness |
| Log and diagnostics | the agent log with a filter, link health, the lock on the platform files, the safety rules, the folders written |

## How it moves the robot, and what stops it

* The robot link is the platform's, unchanged (`cell_platform/ur`, hash-locked). Every `jog_*` and `ur_*` message goes
  to `ur_bridge_ext.handle_message` as it did from the old console.
* A versioned command moves the robot only through `sonair.real.executor` (servoJ of a command that passed the
  collision check), started as a subprocess with the console's speed. Its refusals are the executor's: peak joint speed
  above 0.6 rad/s, a failed collision check, robot not RUNNING or safety not NORMAL.
* Calibration poses are single slow `movel` (50 mm/s) through the platform link, inside the cell envelope, confirmed in
  the page. In automatic mode the arm visits every planned pose on its own; poses the envelope rejects are skipped.
* While a command or calibration moves run, the agent refuses manual motion (jog, move, script, freedrive, program
  play) and gripper commands. Stop messages are never refused.
* STOP (button or `Esc`): aborts a running campaign, stops the executor (as Ctrl+C: servoStop, stopScript), cancels
  calibration moves, halts the jog, stops the gripper's fingers where they are, sends `ur_estop` through the platform
  (stopj and the dashboard stop), and closes the session with the reason.
* Jog: one velocity from the pads, the keyboard and a gamepad, sent every 100 ms with a 400 ms time to live; release,
  leaving the window or losing the socket stops it, and the platform's watchdog stops it if the page goes quiet.
  Tool axes are rotated into the base frame with the measured tool pose before they are sent.
* FREEDRIVE is held, never latched: pressed means loose, released (or the window losing focus) means locked.
* The twin view, Unity and the camera pages cannot move the robot.

## The 3D twin

`console/lab.py` serves the Unity project's own scene data read-only under `/lab/` (scene.json, lab.json,
sequences.json, the SNM1 meshes, the PNG textures; nothing else, gzip when the browser accepts it). `web/js/twin.js`
builds the same rig in three.js: every body, joint axis and pivot from scene.json, the 2F-85 linkage from its
closure table, the kit parts, the camera frusta, the coordinate frames and the room. Unity numbers sit under one root
whose matrix maps Unity to the robot base frame B (x_B = z_u, y_B = -x_u, z_B = y_u), so the view's world is B.
`tests/test_lab.py` checks the rig's flange against the platform's `ur_kin` on random joint sets.

Modes are written on the view: LIVE (measured joints, commanded ghost), PREVIEW (a kit sequence; the ghost is the
live arm) and REPLAY (a recorded run). `D435i eye` and `D405 eye` look through the cameras.

## Cameras and calibration in mock mode

Each mock camera is rendered from the same scene: pose from the rig and the measured joints, field of view from the
scene, and the ChArUco paddle where the scene puts it (the Unity board texture is a valid 9 x 6, 15 mm,
DICT_5X5_100 board). Calibration captures are rendered at 1280 px with 2 x 2 supersampling. In mock mode the true
camera mount is known, so the hand-eye result is compared with it: a full run (point at the board, 10 bootstrap
poses, 14 planned poses) gave 0.22 mm and 0.05 degrees from the truth here. This exercises detection, planning and
the solve; it says nothing about a real lens.

## Sessions

The recorder does not open a second robot connection: it subscribes to the link's own packets. At 500 Hz that is
every controller cycle, each row with the controller's timestamp, and each packet's host receive time gives the clock
pairs. IMUs are read through their FIFO with their own timestamps. The files are the Jetson session format
(`sonair.session/0.1`), so QC (`sonair.jetson.qc`) and export (`sonair.jetson.export`) are shared with
`python -m sonair jetson-record`. On top of that QC the console fails a session that has any 30003 packet or a link
that is not at 500 Hz, and marks every mock session. Camera frames are not part of a session yet (milestone M4).

Folders written (not committed): `runs_jetson/` sessions, `runs/` executor runs, `exports/` exported runs,
`campaigns/` plans with their step results, `calibration/` calibration results and their archive, `captures/` RTDE
captures and saved camera views.

## HTTP on the agent's port

| Path | What |
|---|---|
| `/` and `/css`, `/js`, `/vendor` | the page |
| `/ws` | the WebSocket (token required when bound to the lab network) |
| `/lab/<file>` | the Unity scene data, read-only, allow-listed names |
| `/cam/<D405 or D435i>/<color or depth>.jpg` | the latest camera frame (token as for the WebSocket) |
| `/files/<name>` | RTDE captures and saved camera views from `captures/`, download only |

## Mock mode

`--mock` starts `console/mockur.py` in its own process: a stand-in controller on the controller's ports (RTDE 30004,
dashboard 29999, script 30002) and the Robotiq URCap gripper socket (63352), so the unchanged platform modules connect
to it exactly as to the arm. It moves kinematically with an 8 ms lag; it is not a robot simulator. The executor runs
with `--mock` (a fake robot inside the executor) and the stand-in plays the same command in real time, so a mock
session has motion, an IMU that follows it, QC and export. Mock sessions are labelled and never enter a dataset.

## Optional packages

`opencv-python-headless` (4.7 or newer, with aruco) for board detection and hand-eye; `Pillow` or OpenCV for JPEG
camera frames (PNG otherwise); `pyrealsense2` for the real cameras (on the Jetson, built for the JetPack version).

## Not verified here

No real arm, URSim, gripper, RealSense camera, LSM6DSV16X or Jetson was available: the robot link, gripper and cameras
ran against the stand-ins only. The RealSense path (`console/cameras.py`, `RealSenseCamera`) has not been run. The
page was checked in headless Chromium with software WebGL at 1680 x 1000; a GPU browser renders the twin faster.

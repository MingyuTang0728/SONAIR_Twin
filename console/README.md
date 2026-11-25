# SONAIR console

One process serves the browser UI and its WebSocket on one port. Plain Python (asyncio, websockets) and plain
JavaScript: no build step, no internet access needed in the lab.

```
python -m console --mock                          # no robot: stand-in controller on 127.0.0.1 + simulated IMU
python -m console --robot <URSim IP>              # URSim first
python -m console --robot 192.168.1.10            # the cell
python -m console --robot 192.168.1.10 --bind 0.0.0.0     # reachable from the lab network: prints a URL with a token
```

Open `http://127.0.0.1:8765/`. Keys: `1` to `7` switch pages, `Esc` is STOP.

## Pages

| Page | What it is for |
|---|---|
| Overview | ready-to-record checklist (RTDE, 500 Hz, RUNNING, NORMAL, slider 100 %, IMU, nothing executing, robot link unchanged), log |
| Robot | connect, power and safety, payload to the controller, joint table and chart, hold-to-jog, step, speed slider, freedrive, Unity mirror |
| Sensors | LSM6DSV16X units (I2C bus, address, output rate), live readings, bench check (gravity, gyro bias and noise, both clock maps) |
| Sessions | one recorded session: a checked command through the executor, or passive (pendant program, jog); QC; export |
| Campaign | plans from the versioned commands (speed, repeats, settle time, presets); one step at a time or auto-advance |
| Data | sessions recorded here; the released benchmark-v1 runs with previews (held-out runs show inputs only) |
| Benchmark | scoring rules (task/2 package, task/3 changes), audit numbers, the benchmark site's leaderboard, the scoring commands |

## How it moves the robot, and what stops it

* The robot link is the platform's, unchanged (`cell_platform/ur`, hash-locked). Every `jog_*` and `ur_*` message goes
  to `ur_bridge_ext.handle_message` as it did from the old console.
* A versioned command moves the robot only through `sonair.real.executor` (servoJ of a command that passed the
  collision check), started as a subprocess with the console's speed. Its refusals are the executor's: peak joint speed
  above 0.6 rad/s, a failed collision check, robot not RUNNING or safety not NORMAL.
* While a command executes, the agent refuses manual motion messages (jog, move, script, freedrive). Stop messages are
  never refused.
* STOP (button or `Esc`): aborts a running campaign, stops the executor (as Ctrl+C: servoStop, stopScript), halts the
  jog, sends `ur_estop` through the platform (stopj and the dashboard stop), and closes the session with the reason.
* Jog: the page sends a velocity every 100 ms with a 400 ms time to live; release, leaving the window or losing the
  socket stops it, and the platform's watchdog stops it if the page goes quiet.
* Unity receives measured and commanded joints over UDP (display only). Nothing from Unity reaches the robot.

## Sessions

The recorder does not open a second robot connection: it subscribes to the link's own packets. At 500 Hz that is
every controller cycle, each row with the controller's timestamp, and each packet's host receive time gives the clock
pairs. IMUs are read through their FIFO with their own timestamps. The files are the Jetson session format
(`sonair.session/0.1`), so QC (`sonair.jetson.qc`) and export (`sonair.jetson.export`) are shared with
`python -m sonair jetson-record`. On top of that QC the console fails a session that has any 30003 packet or a link
that is not at 500 Hz, and marks every mock session.

Folders written (not committed): `runs_jetson/` sessions, `runs/` executor runs, `exports/` exported runs,
`campaigns/` plans with their step results.

## Mock mode

`--mock` starts `console/mockur.py` in its own process: a stand-in controller on the controller's ports (RTDE 30004,
dashboard 29999, script 30002), so the unchanged platform modules connect to it exactly as to the arm. It moves
kinematically with an 8 ms lag; it is not a robot simulator. The executor runs with `--mock` (a fake robot inside the
executor) and the stand-in plays the same command in real time, so a mock session has motion, an IMU that follows it,
QC and export. Mock sessions are labelled and never enter a dataset.

## What the old console had that this one does not

Inspection (vision), projector and depth, multi-view 3D scanning, the cloud relay and FTP, and the IMU transports of
the previous kit. The console is for the sim-to-real benchmark: robot, IMUs, sessions, campaigns, data and scores.
Cameras (D405, D435i) are not in this version; their capture belongs to the session format when the camera tool is
in the campaign (M4).

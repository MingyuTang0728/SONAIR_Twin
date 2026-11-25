# Dataset Card - SONAIR v2

- **Platform**: a 6-DoF collaborative industrial arm with a tool-mounted inertial sensor and RGB-D
  camera; the same robot model at every site, each site its own robot unit.
- **Sites in this release**: UoN (108 runs, training site).
- **Signals**: joint angles, speeds, motor currents, the controller's torque model and tool pose
  at 125 Hz; a 9-axis IMU on an instrumented carrier at 100 Hz (resampled onto the 125 Hz clock);
  the tool's pose seen by the tool camera against a fixed ChArUco board, up to 30 Hz, where tracked
  (0 runs; vision gate: not measured).
  Video of every run is kept as evidence and not released.
- **Design**: one joint (joint 3) at 6 speeds (0.2–0.9 rad/s) × 3 arm poses × 3 motion types = 54
  operating conditions, repeated over sessions with one deliberate sensor refit. 18 conditions are
  held out, chosen once with a fixed seed before any data existed.
- **In this release**: {'E2': 108} runs over 54 cells;
  sessions {'Session 1': 108}.
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

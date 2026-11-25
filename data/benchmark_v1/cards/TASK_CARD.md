# Task Card - SONAIR v2

**Question.** Given a robot's command and its simulation, predict what the real robot's sensors will read -- above all in the rare, demanding moments.

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
- **train**: published condition cells of the training site(s), input and target
- **test**: held-out condition cells, input only
- **tail (headline)**: test samples where |commanded elbow acceleration| >= 0.5 x its peak in the run (starts, stops, reversals); E3 runs in full
- **stress**: missing - three stretches of the simulated input (20% of samples) are absent; lag - the simulated input arrives 40 ms late; drift - the IMU calibration is off by 2 deg about x and S0's torque by +3%

## Score
GCR = 1 - err(entry) / err(ref), per run, at the 95th percentile (and median) of the per-sample error; the mean over runs. **Headline: Effort Tail GCR-p95: the share of S0's worst-moment error in the elbow current that an entry removes, on the long-tail samples of the held-out cells.**

## Baselines (scored like any entry)
| Entry | Track | **Effort tail** | Effort all | Gyro tail | Gyro all |
|---|---|---|---|---|---|
| L1 · learned (ridge regression) |  | +0.696 | +0.656 | +0.131 | +0.567 |
| F1 · S0 + friction, kinematic rate |  | +0.432 | +0.385 | +0.131 | +0.567 |
| CM · controller torque model |  | +0.201 | +0.065 | +0.000 | +0.000 |
| S0 · reference simulator, unchanged |  | -0.000 | -0.000 | +0.000 | +0.000 |
| K0 · rigid kinematics |  | -0.000 | -0.000 | +0.131 | +0.567 |

## Out of scope
Tool position is reported as a kinematics check only: copying the command onto a
rigid model already closes ~98% of it, because the robot's own controller tracks
its command so closely.

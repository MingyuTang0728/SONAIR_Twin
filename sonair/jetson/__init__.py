"""Acquisition on the Jetson Orin Nano: one host clock for the robot, the IMUs and the cameras.

Topology (one computer owns every stream, so every sample can be put on one clock):

    UR controller ──Ethernet (dedicated port)── Jetson ──USB 3── D405 / D435i
                                                  │
                                                  ├──I2C (Qwiic, differential extender for the arm run)── LSM6DSV16X IMUs
                                                  └──UDP── Unity twin on the workstation (display only)

Rules this package enforces, each learnt from the benchmark-v1 data (see reports/v1_real):
  * every stream is written to its own file with its OWN timestamps (device clock) and the host receive time;
    nothing is sample-and-held into another stream's rows (that lost 24 % of the IMU samples in benchmark-v1 and made
    half of the gyro "gap" a timing artefact);
  * the robot is read over RTDE only, at 500 Hz, through ur_rtde's own recording thread; a session on any fallback
    interface is refused;
  * device clocks are mapped to the host clock by a fitted offset + drift on the lower envelope of the delays
    (clock.ClockMap), and the residual is part of the session QC;
  * motion is commanded only by sonair.real.executor (servoJ of a checked command file) or by a URScript program the
    operator starts; Unity never commands the robot.
"""

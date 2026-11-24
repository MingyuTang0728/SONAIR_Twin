SONAIR cell twin for Unity, v2.0 (English)

What it is
  A viewer and authoring tool for the SONAIR two-mode inspection cell (UR5e, 2F-85 gripper, camera tool, specimens),
  inside a realistic lab. Unity does not command the robot and is not the reference simulator of the benchmark;
  the dynamics reference is MuJoCo (sonair-twin). Unity is used to author and check sequences, to mirror the real arm
  live (with the commanded arm as a ghost), to render camera views with exact poses, and for XR.

Quick start
  1. Unity Hub > Add > Add project from disk > SONAIR_Unity (Unity 2021.3 LTS or newer, or Unity 6).
  2. On first open the demo scene Assets/SONAIR/SONAIR_Demo.unity is created and opened.
     If not: menu SONAIR > 1 Create and Open Demo Scene. This also sets the colour space to Linear.
  3. Press Play. Keys: Space play/pause, 1..6 sequences, L lab on/off, G ghost, F frames, P camera insets,
     C save camera images with poses, V cycle views, Tab hide the panel.
  Existing project: Assets > Import Package > Custom Package > SONAIR_Unity_v2.unitypackage, then SONAIR > 1.

Key features (everything else was removed)
  Sequence  the six pre-computed sequences: Mode A and Mode B on S2 and S4, two hand-eye calibrations
  Manual    joint and gripper sliders, key poses, object in the hand
  Live      mirrors the real arm from UDP. Packet: q1..q6,closure,t1..t6 (rad); t = RTDE target_q, drawn as a
            translucent ghost, with the tracking error in degrees and millimetres. "#text" lines are shown as status.
            Senders: the Jetson recorder (python -m sonair jetson-record ... --unity HOST:5005) or
            SONAIR_Unity/Tools/sonair_live_bridge.py (--demo SONAIR_Trajectories_CSV/B_S2.csv works without a robot).
  Cameras   D405 and D435i views, saved as PNG + JSON (optical frame and tool0 poses in the robot base frame B,
            pinhole intrinsics, joint state)
  Lab       room, epoxy floor with the cell's hazard marking, guard fence, controller cabinet, e-stop, benches,
            whiteboard, cabinets, operator desk, LED ceiling panels. Built at Play from Resources/SONAIR/lab.json.

Editing the lab
  Source/make_lab.py writes lab.json and the procedural textures. Source/verify_blender.py (LAB=1) renders the same data
  in Blender; the images in Previews/ were made that way.

Frames
  Robot base frame B (UR controller base, metres, z up) to Unity: x_B = z_u, y_B = -x_u, z_B = y_u.

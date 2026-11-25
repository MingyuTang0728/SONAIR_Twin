"""The lab around the robot: the Unity scene the 3D view is built from, the RTDE monitor, the gripper socket, the mock
cameras and the hand-eye calibration they feed. No hardware needed."""
import socket
import time

import numpy as np
import pytest

from cell_platform import paths
from console import calibration, cameras, gripper, lab, rtde


def _free_port():
    s = socket.socket(); s.bind(("127.0.0.1", 0)); p = s.getsockname()[1]; s.close()
    return p


def test_unity_assets_are_served_read_only():
    if not lab.available():
        pytest.skip("Unity package not in this checkout")
    raw, gz, ctype = lab.asset("scene.json")
    assert ctype == "application/json" and raw.startswith(b"{") and len(gz) < len(raw)
    raw, _, _ = lab.asset("Meshes/ur_base_0.bytes")
    assert raw[:4] == b"SNM1"
    assert lab.asset("Textures/charuco.bytes")[2] == "image/png"
    for bad in ("../scene.json", "Meshes/../../../CLAUDE.md", "Meshes/x.y.bytes", "Scripts/Runtime/SonairRig.cs", "/etc/passwd", "lab.json.meta"):
        assert lab.asset(bad) is None, bad


def test_unity_rig_matches_the_ur_kinematics():
    """The twin's arm is the Unity rig; its flange must be where the robot's own kinematics put it."""
    if not lab.available():
        pytest.skip("Unity package not in this checkout")
    paths.add()
    import ur_kin
    rng = np.random.default_rng(1)
    for q in [np.zeros(6)] + [rng.uniform(-2.5, 2.5, 6) for _ in range(5)]:
        A, B = lab.tool0_B(q), np.asarray(ur_kin.fk(q))
        assert np.abs(A[:3, :3] - B[:3, :3]).max() < 5e-3                # nominal DH against the Menagerie geometry
        assert np.linalg.norm(A[:3, 3] - B[:3, 3]) < 2.5e-3
    T = lab.tool0_to_optical("G_D435i")
    assert abs(np.linalg.det(T[:3, :3]) - 1) < 1e-9                         # a proper rotation, not the Unity mirror


def test_rtde_monitor_counts_missing_cycles_and_captures(tmp_path, monkeypatch):
    monkeypatch.setattr(rtde, "CAPTURES", str(tmp_path))
    m = rtde.RtdeMonitor()
    assert m.start_capture(0.5, "unit test")["ok"]
    t0, h0 = 1000.0, 50.0
    for k in range(2000):
        if k == 1200:
            continue                                                        # one controller cycle lost
        m.push({"timestamp": t0 + 0.002 * k, "_mono": h0 + 0.002 * k + 0.0003 * (k % 3), "_source": "rtde",
                "actual_q": [0.1 * k] * 6, "target_q": [0.0] * 6, "actual_current": [1.0] * 6})
    s = m.stats(window_s=10)
    assert s["missing_cycles"] == 1 and abs(s["period_ms"] - 2.0) < 1e-6 and s["sources"] == {"rtde": 1999}
    assert abs(s["rate_ctrl_hz"] - 500) < 1
    r = m.get_series(max_points=100)
    assert r["names"] == ["actual_q[2]", "target_q[2]", "actual_current[2]"] and len(r["t"]) <= 100
    time.sleep(0.6)
    m.push({"timestamp": t0 + 10, "_mono": h0 + 10, "_source": "rtde", "actual_q": [0.0] * 6})     # ends the capture
    for _ in range(50):
        if m.captures_done:
            break
        time.sleep(0.1)
    done = m.captures_done[0]
    assert done["rows"] == 2000 and done["file"].endswith("_unit_test.csv")
    lines = open(tmp_path / done["file"]).read().splitlines()
    assert lines[0].split(",")[:3] == ["timestamp", "host_t", "_source"] and len(lines) == 2001


def test_mock_gripper_through_the_urcap_protocol():
    port = _free_port()
    srv = gripper.MockGripperServer("127.0.0.1", port).start()
    g = gripper.Gripper()
    try:
        assert g.connect("127.0.0.1", port)["ok"]
        assert g.activate()["ok"]
        for _ in range(50):
            if g.status().get("activated"):
                break
            time.sleep(0.1)
        assert g.status()["activated"]
        assert g.move(200, speed=255, force=50)["ok"]
        for _ in range(50):
            st = g.status()
            if st.get("pos") == 200:
                break
            time.sleep(0.1)
        assert st["pos"] == 200 and st["requested"] == 200 and 9 < st["width_mm"] < 11
    finally:
        g.close(); srv.stop()


def test_accelerometer_six_faces():
    six = calibration.SixFace()
    g = 9.80665
    bias, scale = np.array([0.05, -0.12, 0.2]), np.array([1.01, 0.98, 1.03])
    for i, ax in enumerate("xyz"):
        for s in (1, -1):
            v = np.zeros(3); v[i] = s * g
            six.faces.setdefault("t", {})[f"{ax}{'+' if s > 0 else '-'}"] = (v * scale + bias).tolist()
    r = six.ac.fit(six.faces["t"])
    assert r["ok"] and np.allclose(r["bias"], bias, atol=1e-4) and np.allclose(r["scale"], scale, atol=1e-5)


def test_mock_camera_hand_eye_recovers_the_mount():
    """Render the collar camera from eight poses around the paddle, run the platform's hand-eye, compare with the
    scene's camera mount (known exactly in mock mode)."""
    cv2 = pytest.importorskip("cv2")
    if not hasattr(cv2, "aruco") or not lab.available():
        pytest.skip("needs OpenCV with aruco and the Unity scene")
    pytest.importorskip("mujoco")
    from sonair import cell, kin
    b = cameras.board()
    centre = b["corners"].mean(0)
    q_now = [np.array([-0.9562, -2.2399, -1.1127, -2.9305, -4.0977, 1.2566])]
    cam = cameras.MockCamera("D435i", lambda: q_now[0])
    he = calibration.HandEye()
    s = he.session("test")
    for k, (tilt, az, roll, dist) in enumerate([(15, 0, 0, 0.30), (25, 60, 20, 0.33), (25, 150, -25, 0.27), (30, 240, 15, 0.36),
                                               (20, 320, -15, 0.30), (35, 100, 30, 0.38), (10, 200, -30, 0.26), (30, 20, 10, 0.32)]):
        t, a, r = np.radians(tilt), np.radians(az), np.radians(roll)
        f = np.array([np.sin(t) * np.cos(a), np.sin(t) * np.sin(a), -np.cos(t)])
        x = np.array([np.cos(r), np.sin(r), 0.0]); x -= f * (x @ f); x /= np.linalg.norm(x)
        T = np.eye(4); T[:3, :3] = np.column_stack([x, np.cross(f, x), f]); T[:3, 3] = centre - f * dist
        T_tool = T @ np.linalg.inv(cam.T_tool_opt)
        q, err = kin.ik(T_tool, q_now[0])
        assert err < 1e-4, k
        q_now[0] = q
        img, intr = cam.still_frame()
        tcp = list(T_tool[:3, 3]) + list(cell.rotvec(T_tool[:3, :3]))
        assert s.add(img, tcp, intr).get("ok"), k
    res = s.solve()
    assert res["ok"]
    X = np.asarray(res["T_tcp_cam"])
    assert np.linalg.norm(X[:3, 3] - cam.T_tool_opt[:3, 3]) < 1e-3                # under a millimetre
    dR = X[:3, :3].T @ cam.T_tool_opt[:3, :3]
    assert np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))) < 0.2

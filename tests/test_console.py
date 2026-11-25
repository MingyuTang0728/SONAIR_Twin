"""The console end to end against the stand-in controller: the unchanged platform robot link, the interlock, STOP,
a recorded session with QC and export, the executor's refusal, and the public data release. No hardware needed."""
import asyncio
import csv
import gzip
import json
import os
import socket

import pytest

from cell_platform import paths
from sonair import cell

ROOT = cell.ROOT


def test_platform_robot_link_unchanged():
    assert paths.verify_locked() == []


def _ports_free():
    for port in (29999, 30002, 30004):
        s = socket.socket()
        try:
            if s.connect_ex(("127.0.0.1", port)) == 0:
                return False
        finally:
            s.close()
    return True


@pytest.mark.skipif(not _ports_free(), reason="a controller (or a console in --mock) is already on 127.0.0.1")
def test_console_mock_end_to_end(tmp_path):
    import websockets
    from console.agent import Agent

    async def run():
        agent = Agent(mock=True, runs=str(tmp_path / "runs"))
        srv = asyncio.create_task(agent.serve("127.0.0.1", 8798))
        try:
            await asyncio.sleep(3.0)
            async with websockets.connect("ws://127.0.0.1:8798/ws", max_size=2 ** 24) as ws:
                rid = [0]

                async def req(d, timeout=60):
                    rid[0] += 1; d["rid"] = rid[0]; await ws.send(json.dumps(d))
                    while True:
                        m = json.loads(await asyncio.wait_for(ws.recv(), timeout))
                        if m.get("rid") == rid[0]:
                            return m

                hello = json.loads(await ws.recv())
                assert hello["mock"] and hello["locked_ok"] and len(hello["commands"]) > 10
                for _ in range(50):
                    st = agent.snapshot()
                    if not st["ready"] and st["imu"]:
                        break
                    await asyncio.sleep(0.2)
                assert st["link"]["source"] == "rtde" and st["robot"]["robot_mode_text"] == "RUNNING", st["ready"]

                # the RTDE page sees the link's own packets; the 3D view's scene and the camera frames are served
                ri = await req({"type": "rtde_info"})
                assert ri["stats"]["sources"].keys() == {"rtde"} and abs(ri["stats"]["period_ms"] - 2.0) < 1e-6
                assert {"actual_q", "target_q", "actual_current"} <= {f["name"] for f in ri["fields"]}
                import urllib.request

                def get(path):
                    with urllib.request.urlopen("http://127.0.0.1:8798" + path, timeout=10) as f:
                        return f.status, f.headers.get("Content-Type")
                assert (await asyncio.to_thread(get, "/lab/scene.json"))[0] == 200
                for _ in range(30):
                    code, ctype = await asyncio.to_thread(get, "/cam/D435i/color.jpg")
                    if code == 200:
                        break
                    await asyncio.sleep(0.3)
                assert code == 200 and ctype.startswith("image/")

                # a passive session with a short jog: every controller cycle recorded, QC without link failures
                r = await req({"type": "session_start", "motion": "passive", "cell": "0.3/extended/contour", "payload": "gripper_open"})
                assert r["ok"], r
                for _ in range(15):
                    await ws.send(json.dumps({"type": "jog_vel", "xd": [0, 0.03, 0, 0, 0, 0], "ttl_ms": 400}))
                    await asyncio.sleep(0.1)
                await ws.send(json.dumps({"type": "jog_stop"}))
                await asyncio.sleep(1.5)
                aq = await req({"type": "acq_status"})                     # the Acquisition page, mid-session
                se = aq["session"]
                assert se["state"] == "recording" and se["rows"] > 0.9 * se["expected_rows"] > 0
                assert aq["rtde"]["sources"].keys() == {"rtde"} and all(u["in_session"] > 0 for u in aq["imu"].values())
                r = await req({"type": "session_stop"})
                q = r["qc"]
                assert q["rtde"]["missing_samples"] == 0 and abs(q["rtde"]["median_period_ms"] - 2.0) < 1e-6
                assert q["console_checks"]["fails"] == [] and q["console_checks"]["packet_sources"].keys() == {"rtde"}
                assert q["verdict"] in ("PASS", "WARN")                       # mock sessions are never PASS-for-dataset
                run_id = r["run_id"]
                e = await req({"type": "session_export", "run_id": run_id}, 120)
                assert e["ok"] and os.path.exists(e["platform_run"]), e

                # the executor refuses a command above 0.6 rad/s, and the step records that
                r = await req({"type": "session_start", "motion": "executor", "cmd": "exc_gripper_open_v1.2_s1", "speed": 1.0})
                assert r["ok"]
                for _ in range(200):
                    if agent.step_task and agent.step_task.done():
                        break
                    await asyncio.sleep(0.1)
                assert agent.exec.refused and "REFUSED" in agent.exec.refused

                # while a checked command runs, manual motion is refused; STOP ends it and the session
                r = await req({"type": "session_start", "motion": "executor", "cmd": "exc_gripper_open_v0.3_s1", "speed": 1.0})
                assert r["ok"], r
                await asyncio.sleep(2.0)
                j = await req({"type": "jog_vel", "xd": [0.02, 0, 0, 0, 0, 0], "ttl_ms": 400})
                assert j["ok"] is False and "refused" in j["msg"]
                g = await req({"type": "grip_move", "pos": 120})
                assert g["ok"] is False and "refused" in g["error"]
                c = await req({"type": "he_next", "cam": "D435i"})
                assert c["ok"] is False
                s = await req({"type": "stop_all"}, 30)
                assert s["ok"] and "Stopped" in (s.get("estop") or {}).get("msg", "")
                await asyncio.sleep(1.0)
                assert not agent.motion_busy()
                assert agent.session.state in ("done", "failed")
        finally:
            srv.cancel()
            agent.shutdown()

    asyncio.run(run())


def test_public_release_has_no_hidden_targets():
    d = os.path.join(ROOT, "data", "benchmark_v1")
    if not os.path.isdir(d):
        pytest.skip("data/benchmark_v1 not built")
    idx = list(csv.DictReader(open(os.path.join(d, "index.csv"))))
    rel = json.load(open(os.path.join(d, "release.json")))
    assert len(idx) == rel["runs"] == 270
    held = set(rel["held_out_cell_list"])
    for r in idx:
        assert (r["cell"] in held) == (r["split"] == "test")
        assert os.path.exists(os.path.join(d, r["file"]))
    assert not any(f.endswith(".jsonl.gz") and not f.endswith(".cmd.jsonl.gz") for f in os.listdir(os.path.join(d, "test_inputs")))
    measured = {"q", "qd", "tcp_pos", "tcp_rot", "imu", "sensors", "speed_scaling", "robot_age_s"}
    for r in [x for x in idx if x["split"] == "test"][:10]:
        with gzip.open(os.path.join(d, r["file"]), "rt") as f:
            f.readline()
            for line in f:
                assert not (set(json.loads(line)) & measured)

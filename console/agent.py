"""The SONAIR console agent: one process, one port, the browser UI and a WebSocket on the same address.

    python -m console --mock                       no robot: a stand-in controller on 127.0.0.1 and a simulated IMU
    python -m console --robot 192.168.0.20         the cell (URSim first: --robot <URSim IP>)
    python -m console --bind 0.0.0.0               reachable from the lab network (a token is then required)

The robot link is the platform's, unchanged (cell_platform/ur, hash-locked): every jog_* and ur_* message is passed
to ur_bridge_ext.handle_message exactly as the old console did, so connecting, jogging, power and stop behave as they
were debugged. What this agent adds sits around that call, never inside it:
  - an interlock: while a checked command is executing, manual motion messages are refused (stops always pass)
  - one STOP that cancels the campaign, stops the executor, halts the jog and sends ur_estop through the platform
  - sessions recorded from the link's own 500 Hz packets (console.recorder), QC, export
  - campaigns: versioned commands run through sonair.real.executor, one step at a time unless auto-advance is on
  - display-only forwarding of the live joints to the Unity twin (UDP), never commands from it
  - the lab around the robot: the RTDE interface page (console.rtde, read only), the two RealSense cameras
    (console.cameras, simulated in --mock), the 2F-85 through the Robotiq URCap socket (console.gripper), the
    calibrations (console.calibration on the platform's calib modules), and the Unity scene for the 3D view (console.lab)
"""
from __future__ import annotations

import asyncio
import json
import logging
import math
import mimetypes
import os
import secrets
import socket
import threading
import time
from collections import deque
from urllib.parse import parse_qs, urlparse

from websockets.asyncio.server import serve
from websockets.datastructures import Headers
from websockets.http11 import Response

from cell_platform import paths
from sonair import cell
from . import calibration, cameras, data as datamod, gripper, jobs, lab, recorder, rtde

log = logging.getLogger("console")
WEB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
ROOT = cell.ROOT

# messages that stop something: never refused, never queued behind anything
STOP_TYPES = {"jog_stop", "jog_halt", "ur_stop", "ur_stopj", "ur_estop", "ur_stop_program", "ur_pause"}
# manual motion: refused while a checked command or a campaign step is moving the robot
MOTION_TYPES = {"jog_vel", "jog_step", "ur_movel", "ur_movej", "ur_movep", "ur_servoj", "ur_speedl", "ur_speedj",
                "ur_freedrive", "ur_script", "ur_play", "ur_load_program"}
FAST = {"jog_vel", "jog_stop", "jog_status"}          # cheap: called on the event loop, no thread hop
STATE_FIELDS = ("timestamp", "actual_q", "target_q", "actual_qd", "actual_current", "actual_TCP_pose", "actual_TCP_force",
                "robot_mode_text", "safety_mode_text", "runtime_state_text", "runtime_state", "speed_scaling",
                "joint_temperatures", "status_flags", "safety_flags", "tcp_force_magnitude", "_source", "robot_mode",
                "safety_mode", "actual_TCP_speed", "target_moment", "actual_digital_input_bits", "actual_digital_output_bits",
                "standard_analog_input0", "standard_analog_input1", "standard_analog_output0", "standard_analog_output1",
                "actual_robot_voltage", "actual_robot_current", "actual_main_voltage", "tool_output_voltage",
                "tool_output_current", "tool_temperature", "actual_joint_voltage", "actual_tool_accelerometer", "target_TCP_pose",
                "runtime_state", "tcp_force_scalar")


class Agent:
    def __init__(self, mock=False, robot=None, runs=None, unity=None, token=None, robot_hz=500.0):
        self.mock, self.robot_host, self.token, self.robot_hz = mock, robot, token, robot_hz
        self.runs = runs or os.path.join(ROOT, "runs_jetson")
        self.mockur = None
        if mock:
            self.mockur = _start_mockur("127.0.0.1")
            self.robot_host = "127.0.0.1"
        paths.add()
        problems = paths.verify_locked()
        if problems:
            log.error("platform robot link files changed: %s", problems)
        self.locked_ok = not problems
        import ur_bridge_ext
        self.X = ur_bridge_ext
        self.hist = recorder.RobotHistory()
        self.X.UR.subscribe(self.hist.push)
        self.imus = recorder.ImuHub(self.hist)
        self.rtde = rtde.RtdeMonitor()
        self.X.UR.subscribe(self.rtde.push)
        self.cams = cameras.CameraHub()
        self.grip = gripper.Gripper()
        self.cal_task = None
        self.cal_msg = ""
        self._he = None
        self._six = None
        self.session: recorder.Session | None = None
        self.exec: jobs.ExecJob | None = None
        self.campaign = None
        self.campaign_task = None
        self.campaign_go = None
        self.step_task = None
        self.clients = set()
        self.logbuf = deque(maxlen=400)
        self.unity = None
        self.unity_target = unity
        if unity:
            self.set_unity(unity)
        self.loop = None
        self.started = time.time()

    # ------------------------------------------------------------------------------------------ plumbing
    def log(self, msg, level="info"):
        ent = {"t": time.strftime("%H:%M:%S"), "level": level, "msg": str(msg)}
        self.logbuf.append(ent)
        getattr(log, "warning" if level == "warn" else level if level in ("info", "error") else "info")(msg)
        if self.loop:
            asyncio.run_coroutine_threadsafe(self.broadcast({"type": "log", **ent}), self.loop)

    async def broadcast(self, msg):
        if not self.clients:
            return
        s = json.dumps(msg, default=_json_default)
        for ws in list(self.clients):
            try:
                await ws.send(s)
            except Exception:
                self.clients.discard(ws)

    async def call(self, fn, *a):
        return await asyncio.get_running_loop().run_in_executor(None, fn, *a)

    # ------------------------------------------------------------------------------------------ state
    def motion_busy(self):
        return bool((self.exec and self.exec.running()) or (self.step_task and not self.step_task.done())
                    or (self.cal_task and not self.cal_task.done()))

    def robot_state(self):
        st = self.X.UR.state() or {}
        return {k: st.get(k) for k in STATE_FIELDS if k in st}

    def link_status(self):
        s = self.X.UR.status()
        h = s.get("health") or {}
        return {"enabled": s.get("enabled", False), "host": s.get("host"), "connected": h.get("connected"),
                "source": h.get("source"), "rate_hz": round(h["rate_hz"], 1) if h.get("rate_hz") else None,
                "controller_version": h.get("controller_version"), "errors": h.get("errors"), "last_error": h.get("last_error"),
                "degraded": s.get("degraded"), "mode": h.get("mode"), "error": s.get("error"),
                "requested_hz": getattr(self.X.UR.telemetry, "frequency", None) if self.X.UR.telemetry else None,
                "script_error": s.get("script_error")}

    def readiness(self, need_imu=False):
        """Everything that must hold before a recorded motion starts, as a list of plain sentences."""
        b = []
        L = self.link_status()
        st = self.robot_state()
        if not L["enabled"]:
            b.append("robot link not started")
        else:
            if not L["connected"]:
                b.append("robot link not connected")
            if L["source"] and L["source"] != "rtde":
                b.append(f"robot data from {L['source']}, not RTDE: fix RTDE before recording")
            if (L["requested_hz"] or 0) < 499:
                b.append(f"robot link at {L['requested_hz']:.0f} Hz: reconnect at 500 Hz to record" if L["requested_hz"] else "robot link rate unknown")
            if st.get("robot_mode_text") != "RUNNING":
                b.append(f"robot mode {st.get('robot_mode_text') or 'unknown'} (needs RUNNING)")
            if st.get("safety_mode_text") not in (None, "NORMAL"):
                b.append(f"safety mode {st.get('safety_mode_text')} (needs NORMAL)")
        if self.session and self.session.state == "recording":
            b.append(f"session {self.session.run_id} is recording")
        if self.motion_busy():
            b.append("a command is executing")
        if need_imu and not self.imus.units:
            b.append("no IMU streaming")
        return b

    def snapshot(self):
        return {"type": "state", "t": time.time(), "mock": self.mock, "robot": self.robot_state(), "link": self.link_status(),
                "imu": self.imus.status(), "session": self.session.status() if self.session else None,
                "exec": self.exec.status() if self.exec else None, "campaign": self._campaign_public(),
                "busy": self.motion_busy(), "ready": self.readiness(), "unity": self.unity_target if self.unity else None,
                "jog": (self.X.UR.jog.status() if self.X.UR.jog else None), "locked_ok": self.locked_ok,
                "gripper": self.grip.status(), "cams": {n: {k: c.get(k) for k in ("kind", "streaming", "fps", "error", "label")}
                                                        for n, c in self.cams.status().items()},
                "cal_busy": bool(self.cal_task and not self.cal_task.done()), "cal_msg": self.cal_msg,
                "rtde_capture": self.rtde.capture_status()["running"]}

    def _campaign_public(self):
        c = self.campaign
        if not c:
            return None
        return {k: c.get(k) for k in ("name", "steps", "settle_s", "state", "auto", "index", "file", "created_utc")}

    async def state_loop(self):
        while True:
            await asyncio.sleep(0.1)
            try:
                await self.broadcast(self.snapshot())
            except Exception as e:
                log.debug("state broadcast: %s", e)

    async def unity_loop(self):
        k = 0
        while True:
            await asyncio.sleep(0.02)
            if not self.unity:
                continue
            st = self.X.UR.state() or {}
            q, tq = st.get("actual_q"), st.get("target_q")
            if q and tq:
                status = None
                if k % 50 == 0:
                    status = (f"recording {self.session.run_id}" if self.session and self.session.state == "recording"
                              else "live" + (" (mock)" if self.mock else ""))
                self.unity.send(q, tq, status=status)
            k += 1

    def set_unity(self, target):
        from sonair.jetson.session import UnityLink
        if not target:
            self.unity, self.unity_target = None, None
            return {"ok": True, "unity": None}
        try:
            self.unity, self.unity_target = UnityLink(target), target
            return {"ok": True, "unity": target}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    # ------------------------------------------------------------------------------------------ the socket
    async def handler(self, ws):
        self.clients.add(ws)
        try:
            await ws.send(json.dumps(self.hello(), default=_json_default))
            async for raw in ws:
                try:
                    d = json.loads(raw)
                except ValueError:
                    continue
                rid = d.get("rid")
                try:
                    rep = await self.dispatch(d)
                except Exception as e:
                    log.exception("message %s", d.get("type"))
                    rep = {"type": "error", "ok": False, "error": f"{type(e).__name__}: {e}"}
                if rep is not None:
                    if rid is not None:
                        rep["rid"] = rid
                    await ws.send(json.dumps(rep, default=_json_default))
        finally:
            self.clients.discard(ws)

    def hello(self):
        return {"type": "hello", "mock": self.mock, "robot_host": self.robot_host, "version": recorder.CONSOLE_VERSION,
                "locked_ok": self.locked_ok, "log": list(self.logbuf)[-80:], "commands": jobs.command_index(),
                "presets": list(jobs.PRESETS), "payloads": _payloads(), "runs_dir": os.path.relpath(self.runs, ROOT),
                "lab": lab.available(), "token": bool(self.token), "cameras": cameras.CAMS, "paddle": calibration.PADDLE}

    async def dispatch(self, d):
        t = d.get("type")
        if not isinstance(t, str):
            return None
        X = self.X

        # ---- the platform's robot messages, unchanged, behind the interlock
        if t in STOP_TYPES:
            return await self.call(X.handle_message, d)
        if t in MOTION_TYPES and self.motion_busy():
            return {"type": "ur_cmd_res" if t.startswith("ur_") else "jog_res", "cmd": t, "ok": False,
                    "msg": "refused: a checked command is executing (press STOP to take over)"}
        if t == "ur_service_start":
            if self.session and self.session.state == "recording":
                return {"type": "ur_service_start_res", "ok": False, "error": "a session is recording: stop it first"}
            d = {**d, "host": d.get("host") or self.robot_host, "frequency": float(d.get("frequency") or self.robot_hz)}
            self.robot_host = d["host"]
            rep = await self.call(X.handle_message, d)
            self.log(f"robot link {'started' if rep.get('ok') else 'failed'}: {d['host']} at {d['frequency']:.0f} Hz"
                     + ("" if rep.get("ok") else f" ({rep.get('error')})"), "info" if rep.get("ok") else "error")
            return rep
        if t.startswith(("jog_", "ur_")):
            if t in FAST:
                return X.handle_message(d)
            rep = await self.call(X.handle_message, d)
            if rep is not None and t not in ("ur_service_status", "jog_status") and not (rep or {}).get("quiet"):
                self.log(f"{t}: {'ok' if rep.get('ok', True) else 'failed'} {rep.get('msg') or rep.get('error') or ''}".strip(),
                         "info" if rep.get("ok", True) else "warn")
            return rep

        # ---- the console's own messages
        h = getattr(self, "m_" + t, None)
        if h is None:
            return {"type": "error", "ok": False, "error": f"unknown message {t}"}
        r = h(d)
        if asyncio.iscoroutine(r):
            r = await r
        return {"type": t + "_res", **(r or {})}

    # ------------------------------------------------------------------------------------------ STOP
    async def m_stop_all(self, d):
        why = d.get("reason") or "operator STOP"
        self.log(f"STOP: {why}", "warn")
        if self.campaign and self.campaign.get("state") == "running":
            self.campaign["state"] = "aborted"
            if self.campaign_go:
                self.campaign_go.set()
        if self.exec and self.exec.running():
            await self.call(self.exec.stop)
        res = {}
        if self.X.UR.jog:
            res["jog"] = self.X.handle_message({"type": "jog_halt"})
        if self.X.UR.controller:
            res["estop"] = await self.call(self.X.handle_message, {"type": "ur_estop"})
        if self.step_task and not self.step_task.done():
            self.step_task.cancel()
        if self.cal_task and not self.cal_task.done():
            self.cal_task.cancel()
            self.cal_msg = "calibration poses stopped by STOP"
        if self.grip.status().get("connected"):
            res["gripper"] = await self.call(self.grip.stop)
        if self.session and self.session.state == "recording":
            self.session.meta["aborted"] = why
            res["session"] = await self.call(self.session.stop)
        return {"ok": True, **res}

    # ------------------------------------------------------------------------------------------ sensors
    def m_imu_start(self, d):
        if self.session and self.session.state == "recording":
            return {"ok": False, "error": "a session is recording"}
        units = d.get("units") or [{"name": "tool", "kind": "mock" if self.mock else "lsm6dsv16x", "bus": 1, "addr": "0x6b"}]
        r = self.imus.start(units, odr=int(d.get("odr") or 480), mock=self.mock or d.get("mock", False))
        self.log(f"IMU start {[u.get('name') for u in units]}: {'ok' if r['ok'] else r['errors']}", "info" if r["ok"] else "error")
        return r

    def m_imu_stop(self, d):
        if self.session and self.session.state == "recording":
            return {"ok": False, "error": "a session is recording"}
        self.imus.stop()
        return {"ok": True}

    async def m_bench_check(self, d):
        """Robot still, IMU on: gravity norm, gyro bias and noise, both clock maps. A few seconds, nothing moves."""
        if self.session and self.session.state == "recording":
            return {"ok": False, "error": "a session is recording"}
        secs = float(d.get("seconds") or 5.0)
        pairs = []
        sink = lambda st: pairs.append((st.get("timestamp"), st.get("_mono"))) if st.get("timestamp") is not None else None
        self.X.UR.subscribe(sink)
        for u in self.imus.units.values():
            u.begin_capture()
        await asyncio.sleep(secs)
        self.X.UR.unsubscribe(sink)
        caps = {n: u.end_capture() for n, u in self.imus.units.items()}
        return await self.call(_bench_check, pairs, caps, self.imus.units, self.hist)

    def m_unity_set(self, d):
        return self.set_unity(d.get("target") or None)

    # ------------------------------------------------------------------------------------------ sessions
    def _run_id(self, cmd, speed, rep):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        return f"{'mock' if self.mock else 'UoN'}_{cmd or 'passive'}_x{speed:g}_r{rep}_{stamp}"

    def _new_session(self, meta):
        rid = self._run_id(meta.get("cmd"), float(meta.get("speed") or 1.0), int(meta.get("rep") or 0))
        meta.setdefault("carrier", None)
        self.session = recorder.Session(self.runs, rid, meta, self.X.UR, self.imus, mock=self.mock)
        return self.session

    async def m_session_start(self, d):
        motion = d.get("motion", "passive")
        blockers = [b for b in self.readiness() if not (motion == "passive" and b.startswith("robot mode"))]
        if blockers:
            return {"ok": False, "error": "; ".join(blockers)}
        meta = {k: d.get(k) for k in ("cell", "rep", "cmd", "speed", "payload", "notes", "operator", "experiment", "imu_cal",
                                      "payload_on_pendant")}
        meta["speed"] = float(meta.get("speed") or (0.5 if motion == "executor" else 1.0))
        if motion == "executor":
            if not meta.get("cmd"):
                return {"ok": False, "error": "choose a command"}
            meta["motion_source"] = "sonair.real.executor servoJ" + (" (mock)" if self.mock else "")
            self.step_task = asyncio.create_task(self.run_step(meta))
            return {"ok": True, "started": "executor", "cmd": meta["cmd"]}
        meta["motion_source"] = "passive (pendant program or jog)"
        s = self._new_session(meta)
        r = await self.call(s.start)
        self.log(f"session {s.run_id} recording (passive)")
        return r

    async def m_session_stop(self, d):
        if self.step_task and not self.step_task.done():
            if self.exec and self.exec.running():
                await self.call(self.exec.stop)
            self.step_task.cancel()
            if self.session and self.session.state == "recording":
                self.session.meta["aborted"] = "stopped by the operator"
                return await self.call(self.session.stop)
            return {"ok": True}
        if not self.session or self.session.state != "recording":
            return {"ok": False, "error": "nothing is recording"}
        r = await self.call(self.session.stop)
        self.log(f"session {self.session.run_id}: QC {((r.get('qc') or {}).get('verdict')) or r.get('error')}")
        return r

    async def run_step(self, meta):
        """One recorded command: session on, executor (and in mock mode the stand-in plays the command), wait until the
        executor has exited and the controller is no longer playing, session off, QC."""
        s = self._new_session(meta)
        await self.call(s.start)
        await asyncio.sleep(0.5)                                  # a little still robot before the motion
        self.exec = jobs.ExecJob(meta["cmd"], speed=meta["speed"], max_qd=float(meta.get("max_qd") or 0.6), run_id=s.run_id + "_exec",
                                 robot=self.robot_host, mock=self.mock, rep=int(meta.get("rep") or 0),
                                 payload_on_pendant=meta.get("payload_on_pendant") or "", on_line=lambda l: self.log("executor: " + l))
        self.log(f"step {meta['cmd']} x{meta['speed']:g}: executor started, session {s.run_id}")
        try:
            await self.call(self.exec.start)
            while self.exec.running():
                await asyncio.sleep(0.1)
            if self.mock and self.exec.refused is None and self.exec.proc.returncode == 0:
                # the mock executor drives a fake robot and returns at once; the stand-in controller then plays the
                # same command in real time so the session has the motion it would have had
                ok, reply = await self.call(_dashboard, self.robot_host, f"sonair mock play {meta['cmd']} {meta['speed']:g}")
                self.log(f"mock controller: {reply}")
            await asyncio.sleep(0.4)
            t_wait = time.time()
            while (self.X.UR.state() or {}).get("runtime_state") == 2 and time.time() - t_wait < (3600 if self.mock else 5):
                await asyncio.sleep(0.1)
            await asyncio.sleep(0.5)
        except asyncio.CancelledError:
            meta["aborted"] = meta.get("aborted") or "cancelled"
            raise
        finally:
            if self.exec.refused:
                meta["aborted"] = self.exec.refused
            if s.state == "recording":
                r = await self.call(s.stop)
                self.log(f"session {s.run_id}: QC {((r.get('qc') or {}).get('verdict')) or r.get('error')}"
                         + (f" (executor: {self.exec.refused})" if self.exec.refused else ""))
        return s

    async def m_session_export(self, d):
        rd = os.path.join(self.runs, d["run_id"])
        try:
            r = await self.call(recorder.export, rd, os.path.join(ROOT, "exports", "platform"), os.path.join(ROOT, "exports", "twin"))
            self.log(f"exported {d['run_id']}: {os.path.relpath(r['platform_run'], ROOT)}")
            return {"ok": True, **r}
        except Exception as e:
            return {"ok": False, "error": f"{type(e).__name__}: {e}"}

    async def m_session_qc(self, d):
        rd = os.path.join(self.runs, d["run_id"])
        q = await self.call(recorder.run_qc, rd)
        return {"ok": True, "qc": q}

    # ------------------------------------------------------------------------------------------ campaign
    def m_campaign_build(self, d):
        if self.campaign and self.campaign.get("state") == "running":
            return {"ok": False, "error": "a campaign is running"}
        if d.get("preset"):
            plan = jobs.preset_plan(d["preset"], speed=float(d.get("speed") or 1.0))
        else:
            plan = jobs.plan_from(d.get("commands") or [], speed=float(d.get("speed") or 0.5), repeats=int(d.get("repeats") or 1),
                                  experiment=d.get("experiment") or "E2", payload_on_pendant=d.get("payload_on_pendant") or "",
                                  settle_s=float(d.get("settle_s") or 5.0), name=d.get("name") or "custom")
        for s in plan["steps"]:
            s["payload"] = d.get("payload") or ""
        plan.update(state="ready", auto=False, index=0)
        os.makedirs(os.path.join(ROOT, "campaigns"), exist_ok=True)
        plan["file"] = os.path.join("campaigns", f"{time.strftime('%Y%m%d_%H%M%S')}_{_slug(plan['name'])}.json")
        self.campaign = plan
        self._save_campaign()
        return {"ok": True, "steps": len(plan["steps"])}

    def _save_campaign(self):
        if self.campaign:
            json.dump(self._campaign_public(), open(os.path.join(ROOT, self.campaign["file"]), "w"), indent=1, default=_json_default)

    def m_campaign_start(self, d):
        c = self.campaign
        if not c or not c["steps"]:
            return {"ok": False, "error": "build a plan first"}
        if c.get("state") == "running":
            return {"ok": False, "error": "already running"}
        blockers = self.readiness()
        if blockers:
            return {"ok": False, "error": "; ".join(blockers)}
        c["auto"] = bool(d.get("auto"))
        c["state"] = "running"
        self.campaign_go = asyncio.Event()
        self.campaign_go.set()                                   # the first step starts on the Start click itself
        self.campaign_task = asyncio.create_task(self.campaign_loop())
        self.log(f"campaign '{c['name']}' started ({len(c['steps'])} steps, {'auto-advance' if c['auto'] else 'confirm each step'})")
        return {"ok": True}

    def m_campaign_next(self, d):
        if not self.campaign or self.campaign.get("state") != "running" or not self.campaign_go:
            return {"ok": False, "error": "no campaign waiting"}
        self.campaign_go.set()
        return {"ok": True}

    def m_campaign_auto(self, d):
        if self.campaign:
            self.campaign["auto"] = bool(d.get("auto"))
            if self.campaign["auto"] and self.campaign_go:
                self.campaign_go.set()
        return {"ok": True}

    async def m_campaign_abort(self, d):
        if not self.campaign:
            return {"ok": False, "error": "no campaign"}
        return await self.m_stop_all({"reason": "campaign aborted"})

    async def campaign_loop(self):
        c = self.campaign
        try:
            while c["index"] < len(c["steps"]) and c["state"] == "running":
                step = c["steps"][c["index"]]
                if not c["auto"]:
                    step["state"] = "waiting"
                    await self.campaign_go.wait()
                    self.campaign_go.clear()
                    if c["state"] != "running":
                        break
                blockers = self.readiness()
                if blockers:
                    step["state"] = "blocked"; step["note"] = "; ".join(blockers)
                    c["state"] = "paused"
                    self.log(f"campaign paused before step {c['index'] + 1}: {step['note']}", "warn")
                    break
                step["state"] = "running"
                meta = dict(cmd=step["cmd"], speed=step["speed"], rep=step["rep"], experiment=step["experiment"], payload=step.get("payload"),
                            payload_on_pendant=step.get("payload_on_pendant"), cell="", notes=f"campaign {c['name']} step {c['index'] + 1}",
                            motion_source="sonair.real.executor servoJ" + (" (mock)" if self.mock else ""))
                self.step_task = asyncio.create_task(self.run_step(meta))
                s = await self.step_task
                step["run_id"], step["qc"] = s.run_id, (s.qc or {}).get("verdict")
                step["state"] = "refused" if meta.get("aborted") else "done"
                step["note"] = meta.get("aborted")
                c["index"] += 1
                self._save_campaign()
                if c["index"] < len(c["steps"]) and c["auto"] and c["state"] == "running":
                    await asyncio.sleep(c["settle_s"])
            if c["state"] == "running":
                c["state"] = "finished"
                self.log(f"campaign '{c['name']}' finished")
        except asyncio.CancelledError:
            c["state"] = "aborted"
        finally:
            self._save_campaign()

    def m_campaign_resume(self, d):
        c = self.campaign
        if not c or c.get("state") not in ("paused", "aborted"):
            return {"ok": False, "error": "nothing to resume"}
        c["state"] = "ready"
        return self.m_campaign_start({"auto": c.get("auto")})

    # ------------------------------------------------------------------------------------------ data and benchmark
    async def m_data_sessions(self, d):
        return {"sessions": await self.call(datamod.list_sessions, self.runs)}

    async def m_data_bench(self, d):
        return {"runs": await self.call(datamod.bench_index)}

    async def m_data_preview(self, d):
        if d.get("kind") == "session":
            return await self.call(datamod.preview_session, d["run_id"], 800, self.runs)
        return await self.call(datamod.preview_bench, d["run_id"])

    async def m_bench_info(self, d):
        return await self.call(datamod.bench_info)

    def m_ping(self, d):
        return {"ok": True, "t": time.time()}

    # ------------------------------------------------------------------------------------------ RTDE interface (read only)
    def m_rtde_info(self, d):
        L = self.link_status()
        h = (self.X.UR.status() or {}).get("health") or {}
        return {"ok": True, "link": L, "health": {k: h.get(k) for k in ("source", "connected", "simulated", "controller_message",
                                                                          "controller_version", "rate_hz", "packets", "errors",
                                                                          "last_error", "reconnects", "dropped_fields", "rtde_drops",
                                                                          "rtde_unavailable_reason", "mode", "host_stall_max_s",
                                                                          "host_stalls")},
                "fields": self.rtde.fields(h.get("fields")), "stats": self.rtde.stats(float(d.get("window_s") or 10.0)),
                "capture": self.rtde.capture_status(), "captures": rtde.list_captures(),
                "watch": [f"{n}:{i}" if i is not None else n for n, i in self.rtde.watch]}

    def m_rtde_watch(self, d):
        return self.rtde.set_watch(d.get("fields") or [])

    def m_rtde_series(self, d):
        return {"ok": True, **self.rtde.get_series(int(d.get("max_points") or 1500), d.get("seconds"))}

    def m_rtde_capture(self, d):
        r = self.rtde.start_capture(d.get("seconds") or 10, d.get("name") or "")
        if r.get("ok"):
            self.log(f"RTDE capture {r['file']}: {r['seconds']:g} s of every field")
        return r

    def m_rtde_capture_stop(self, d):
        return self.rtde.stop_capture()

    # ------------------------------------------------------------------------------------------ cameras
    def _q(self):
        st = self.X.UR.state() or {}
        return st.get("actual_q")

    async def m_cam_start(self, d):
        r = await self.call(self.cams.start, self.mock, self._q)
        self.log(f"cameras: {r.get('cameras') or r.get('error')}", "info" if r.get("ok") else "warn")
        return r

    def m_cam_stop(self, d):
        self.cams.stop()
        return {"ok": True}

    def m_cam_status(self, d):
        return {"ok": True, "cams": self.cams.status(), "note": self.cams.note}

    def m_cam_depth_at(self, d):
        c = self.cams.get(d.get("name"))
        if not c:
            return {"ok": False, "error": "no such camera"}
        z = c.depth_at(d.get("u", 0), d.get("v", 0))
        return {"ok": z is not None, "depth_m": z}

    async def m_cam_snapshot(self, d):
        c = self.cams.get(d.get("name"))
        if not c:
            return {"ok": False, "error": "no such camera"}
        return await self.call(c.snapshot)

    # ------------------------------------------------------------------------------------------ gripper (2F-85)
    async def m_grip_connect(self, d):
        host = d.get("host") or self.robot_host
        if not host:
            return {"ok": False, "error": "no robot address: connect the robot first"}
        r = await self.call(self.grip.connect, host)
        self.log(f"gripper: {'connected on ' + host if r.get('ok') else r.get('error')}", "info" if r.get("ok") else "warn")
        return r

    def _grip_guard(self):
        if self.motion_busy():
            return {"ok": False, "error": "refused: a checked command is executing (press STOP to take over)"}
        if not self.grip.status().get("connected"):
            return {"ok": False, "error": "gripper not connected"}
        return None

    async def m_grip_activate(self, d):
        return self._grip_guard() or await self.call(self.grip.activate)

    async def m_grip_move(self, d):
        g = self._grip_guard()
        if g:
            return g
        r = await self.call(self.grip.move, d.get("pos", 0), d.get("speed", 128), d.get("force", 64))
        if r.get("ok"):
            self.log(f"gripper to {int(d.get('pos', 0))} (speed {int(d.get('speed', 128))}, force {int(d.get('force', 64))})")
        return r

    async def m_grip_stop(self, d):
        return await self.call(self.grip.stop)

    # ------------------------------------------------------------------------------------------ calibration
    @property
    def he(self):
        if self._he is None:
            self._he = calibration.HandEye()
        return self._he

    @property
    def six(self):
        if self._six is None:
            self._six = calibration.SixFace()
        return self._six

    def _tcp(self):
        return (self.X.UR.state() or {}).get("actual_TCP_pose")

    def m_cal_status(self, d):
        cam, unit = d.get("cam") or "D435i", d.get("unit") or "tool"
        ok, why = self.he.available()
        return {"ok": True, "cv": {"ok": ok, "error": why}, "handeye": self.he.status(cam),
                "sixface": self.six.status(unit, self.imus.units.get(unit)), "archive": calibration.list_archive(),
                "in_use": calibration.in_use(), "busy": bool(self.cal_task and not self.cal_task.done()), "msg": self.cal_msg,
                "mock_truth": self.mock}

    async def m_he_detect(self, d):
        return await self.call(self.he.detect, d.get("cam") or "D435i", self.cams.get(d.get("cam") or "D435i"), True)

    async def m_he_capture(self, d):
        cam = d.get("cam") or "D435i"
        st = self.X.UR.state() or {}
        if st.get("actual_qd") and max(abs(v) for v in st["actual_qd"]) > 2e-3:
            return {"ok": False, "error": "the arm is moving: capture with the robot still"}
        r = await self.call(self.he.capture, cam, self.cams.get(cam), self._tcp())
        return {k: v for k, v in r.items() if k != "corners"}

    def m_he_undo(self, d):
        return self.he.undo(d.get("cam") or "D435i")

    def m_he_clear(self, d):
        spec = d.get("spec")
        if spec:
            spec = {k: spec[k] for k in ("kind", "cols", "rows", "square_mm", "marker_mm", "dictionary") if k in spec}
        return self.he.clear(d.get("cam") or "D435i", spec)

    async def m_he_solve(self, d):
        cam = d.get("cam") or "D435i"
        truth = None
        if self.mock and cam in self.cams.cams and getattr(self.cams.cams[cam], "on_tool", False):
            truth = self.cams.cams[cam].T_tool_opt
        r = await self.call(self.he.solve, cam, truth)
        self.log(f"hand-eye {cam}: {r.get('verdict') or r.get('error')}" + (f", mock truth error {r['mock_truth']['translation_error_mm']} mm"
                                                                          if r.get("mock_truth") else ""))
        return json.loads(json.dumps(r, default=_json_default))

    def m_he_save(self, d):
        r = self.he.save(d.get("cam") or "D435i", self.mock)
        if r.get("ok"):
            self.log(f"hand-eye saved: {r['path']}" + (" (mock)" if self.mock else ""))
        return r

    def m_he_plan(self, d):
        env = getattr(self.X.UR.controller, "envelope", None) if self.X.UR.controller else None
        env_d = {"x": list(env.x), "y": list(env.y), "z": list(env.z)} if env else None
        return self.he.plan(d.get("cam") or "D435i", self._tcp(), env_d)

    def _cal_ready(self):
        b = [x for x in self.readiness() if not x.startswith("robot link at")]
        return b

    async def _move_and_wait(self, msg, timeout=40.0):
        """One slow move through the platform link, then wait until the arm is still."""
        rep = await self.call(self.X.handle_message, msg)
        if not (rep or {}).get("ok"):
            return False, (rep or {}).get("msg") or "move refused"
        await asyncio.sleep(0.6)
        t0 = time.time()
        while time.time() - t0 < timeout:
            st = self.X.UR.state() or {}
            qd = st.get("actual_qd") or [1]
            if max(abs(v) for v in qd) < 1e-3:
                await asyncio.sleep(0.8)
                return True, ""
            await asyncio.sleep(0.1)
        return False, "the arm did not come to rest"

    async def m_he_next(self, d):
        """Move to the next planned calibration pose (and capture there if asked). One move per click."""
        cam = d.get("cam") or "D435i"
        if self.motion_busy():
            return {"ok": False, "error": "refused: motion in progress"}
        b = self._cal_ready()
        if b:
            return {"ok": False, "error": "; ".join(b)}
        nxt = self.he.next_pose(cam)
        if not nxt:
            return {"ok": False, "error": "no planned pose left: plan first"}
        self.cal_task = asyncio.create_task(self._he_steps(cam, [nxt], capture=bool(d.get("capture", True))))
        return {"ok": True, "pose": nxt}

    async def m_he_auto(self, d):
        """Visit every remaining planned pose and capture at each, stopping at the first refusal. STOP ends it."""
        cam = d.get("cam") or "D435i"
        if self.motion_busy():
            return {"ok": False, "error": "refused: motion in progress"}
        b = self._cal_ready()
        if b:
            return {"ok": False, "error": "; ".join(b)}
        poses = []
        while True:
            p = self.he.next_pose(cam)
            if not p:
                break
            poses.append(p)
        if not poses:
            return {"ok": False, "error": "no planned pose left: plan first"}
        self.cal_task = asyncio.create_task(self._he_steps(cam, poses, capture=True))
        self.log(f"hand-eye {cam}: visiting {len(poses)} planned poses")
        return {"ok": True, "n": len(poses)}

    async def _he_steps(self, cam, poses, capture=True):
        got = skipped = 0
        try:
            for p in poses:
                self.cal_msg = f"pose {p['index']} of {p['of']} ({p['stage']}): moving"
                ok, why = await self._move_and_wait({"type": "ur_movel", "pose": p["tcp_pose"], "a": 0.2, "v": 0.05})
                tcp = self._tcp() or [0] * 6
                off_mm = 1e3 * math.dist(tcp[:3], p["tcp_pose"][:3])
                if not ok and "envelope" in why:
                    skipped += 1
                    self.cal_msg = f"pose {p['index']}: {why}, skipped"
                    self.log(f"hand-eye {cam}: {self.cal_msg}", "warn")
                    continue
                if not ok and "rest" not in why:
                    self.cal_msg = f"pose {p['index']}: {why}"
                    self.log(f"hand-eye {cam}: {self.cal_msg}", "warn")
                    return
                if not ok or off_mm > 3.0:
                    skipped += 1
                    self.cal_msg = f"pose {p['index']}: not reached ({off_mm:.0f} mm away), skipped"
                    self.log(f"hand-eye {cam}: {self.cal_msg}", "warn")
                    continue
                if capture:
                    r = await self.call(self.he.capture, cam, self.cams.get(cam), self._tcp())
                    got += bool(r.get("ok"))
                    self.cal_msg = f"pose {p['index']} of {p['of']}: " + ("captured" if r.get("ok") else (r.get("error") or "board not found"))
            self.cal_msg = (f"done: {got} of {len(poses)} poses captured" if capture else "arrived") + (f", {skipped} not reached" if skipped else "")
            self.log(f"hand-eye {cam}: {self.cal_msg}")
        except asyncio.CancelledError:
            self.cal_msg = "stopped"
            raise

    async def m_he_look(self, d):
        """Mock only: put the collar camera over the ChArUco paddle (the board pose is the scene's, known only in mock)."""
        if not self.mock:
            return {"ok": False, "error": "only in mock mode: on the cell, jog the camera until the board is in view"}
        if self.motion_busy():
            return {"ok": False, "error": "refused: motion in progress"}
        import numpy as np
        cam = self.cams.get("D435i")
        if not cam:
            return {"ok": False, "error": "start the cameras first"}
        b = cameras.board()
        c = b["corners"].mean(0)
        tilt = math.radians(float(d.get("tilt_deg", 18)))
        dist = float(d.get("distance", 0.32))
        f = np.array([math.sin(tilt), 0.0, -math.cos(tilt)])            # looking down, leaning towards +x
        x = np.array([1.0, 0.0, 0.0]) - f * f[0]; x /= np.linalg.norm(x)
        y = np.cross(f, x)
        T = np.eye(4); T[:3, :3] = np.column_stack([x, y, f]); T[:3, 3] = c - f * dist
        T_tool = T @ np.linalg.inv(cam.T_tool_opt)
        from sonair import kin
        q0 = np.array(self._q() or recorder.cell_home(), float)
        best = None
        for seed in (q0, np.array([0.0, -1.9, 1.9, -1.6, -1.57, 0.0]), np.array([-1.0, -1.6, 1.8, -1.8, -1.57, 0.0])):
            q, err = kin.ik(T_tool, seed)
            if err < 1e-4:
                q = q0 + (q - q0 + np.pi) % (2 * np.pi) - np.pi          # the nearest turn of every joint
                if best is None or np.abs(q - q0).max() < np.abs(best - q0).max():
                    best = q
        if best is None:
            return {"ok": False, "error": "no joint solution reaches the view over the board"}
        self.cal_task = asyncio.create_task(self._look(best.tolist()))
        return {"ok": True, "q": [round(v, 4) for v in best]}

    async def _look(self, q):
        self.cal_msg = "moving the camera over the board"
        ok, why = await self._move_and_wait({"type": "ur_movej", "q": q, "a": 0.6, "v": 0.4}, timeout=60)
        self.cal_msg = "the board is in view" if ok else why

    async def m_acc_capture(self, d):
        unit = d.get("unit") or "tool"
        return await self.call(self.six.capture, unit, self.imus.units.get(unit))

    def m_acc_reset(self, d):
        return self.six.reset(d.get("unit") or "tool")

    def m_acc_fit(self, d):
        r = self.six.fit_save(d.get("unit") or "tool", self.mock)
        self.log(f"accelerometer six-face: {'saved' if r.get('ok') else r.get('error')}")
        return r

    async def m_imu_mount(self, d):
        rd = os.path.join(self.runs, os.path.basename(d.get("run_id") or ""))
        if not os.path.isdir(rd):
            return {"ok": False, "error": "choose a recorded session"}
        try:
            r = await self.call(calibration.imu_mount_from_session, rd, d.get("unit") or "tool", self.mock)
        except Exception as e:
            return {"ok": False, "error": f"{type(e).__name__}: {e}"}
        self.log(f"IMU mount and lag from {d.get('run_id')}: {'lag ' + format(r['lag_s'] * 1e3, '.2f') + ' ms' if r.get('ok') else r.get('error')}")
        return r

    # ------------------------------------------------------------------------------------------ lab, sequences, analysis
    def m_seq_list(self, d):
        ids = {c["id"] for c in jobs.command_index()}
        out = []
        for s in lab.sequences_summary():
            k = s["key"]
            out.append({**s, "command": f"task_{k}" if f"task_{k}" in ids else None,
                        "command_half": f"task_{k}_x0.5" if f"task_{k}_x0.5" in ids else None})
        return {"ok": True, "sequences": out}

    async def m_data_series(self, d):
        return await self.call(datamod.run_series, d.get("kind") or "bench", d["run_id"], int(d.get("max_points") or 2500), self.runs)

    async def m_bench_overview(self, d):
        return await self.call(datamod.bench_overview, self.runs)

    # ------------------------------------------------------------------------------------------ run
    async def serve(self, bind="127.0.0.1", port=8765):
        self.loop = asyncio.get_running_loop()

        def resp(code, body, ctype="text/plain", extra=()):
            if isinstance(body, str):
                body = body.encode()
            return Response(code, {200: "OK", 403: "Forbidden", 404: "Not Found", 204: "No Content"}.get(code, "OK"),
                            Headers([("Content-Type", ctype), ("Content-Length", str(len(body))), *extra]), body)

        def process_request(connection, request):
            u = urlparse(request.path)
            tok_ok = not self.token or parse_qs(u.query).get("token", [""])[0] == self.token
            if u.path == "/ws":
                return None if tok_ok else resp(403, "token required\n")
            if u.path.startswith("/lab/"):                       # the Unity scene for the 3D view (public repo data)
                a = lab.asset(u.path[5:])
                if not a:
                    return resp(404, "not found\n")
                raw, gz, ctype = a
                accept = (request.headers.get("Accept-Encoding") or "").lower()
                if gz and "gzip" in accept:
                    return resp(200, gz, ctype, (("Content-Encoding", "gzip"), ("Cache-Control", "max-age=600")))
                return resp(200, raw, ctype, (("Cache-Control", "max-age=600"),))
            if u.path.startswith("/cam/"):                       # /cam/<name>/<color|depth>.jpg
                if not tok_ok:
                    return resp(403, "token required\n")
                parts = u.path.split("/")
                cam = self.cams.get(parts[2]) if len(parts) == 4 else None
                if not cam:
                    return resp(404, "no such camera\n")
                f = cam.frame(parts[3].split(".")[0])
                if not f:
                    return resp(204, b"", "text/plain", (("Cache-Control", "no-store"),))
                return resp(200, f[0], f[1], (("Cache-Control", "no-store"),))
            if u.path.startswith("/files/"):                     # captures/: RTDE CSV and camera snapshots, download only
                if not tok_ok:
                    return resp(403, "token required\n")
                rel = u.path[len("/files/"):]
                p = os.path.normpath(os.path.join(rtde.CAPTURES, rel))
                if not p.startswith(rtde.CAPTURES + os.sep) or not os.path.isfile(p) or not p.endswith((".csv", ".png", ".json")):
                    return resp(404, "not found\n")
                ctype = mimetypes.guess_type(p)[0] or "application/octet-stream"
                return resp(200, open(p, "rb").read(), ctype, (("Content-Disposition", f'attachment; filename="{os.path.basename(p)}"'),))
            name = "index.html" if u.path in ("", "/") else u.path.lstrip("/")
            p = os.path.normpath(os.path.join(WEB, name))
            if not p.startswith(WEB) or not os.path.isfile(p):
                return resp(404, "not found\n")
            ctype = mimetypes.guess_type(p)[0] or "application/octet-stream"
            if p.endswith((".js", ".mjs")):
                ctype = "text/javascript"
            body = open(p, "rb").read()
            cache = "max-age=86400" if "/vendor/" in p else "no-store"
            return resp(200, body, ctype + ("; charset=utf-8" if ctype.startswith("text") or "javascript" in ctype else ""),
                        (("Cache-Control", cache),))

        async with serve(self.handler, bind, port, process_request=process_request, max_size=4 * 2 ** 20):
            self.log(f"console on http://{bind}:{port}/" + (f"?token={self.token}" if self.token else "")
                     + ("  [MOCK: simulated controller and IMU]" if self.mock else ""))
            if self.mock or self.robot_host:
                rep = await self.dispatch({"type": "ur_service_start", "host": self.robot_host, "frequency": self.robot_hz}) \
                    if self.mock else None
                if self.mock:
                    for c in ("ur_power_on", "ur_brake_release"):
                        await asyncio.sleep(0.3)
                        await self.dispatch({"type": c})
                    self.m_imu_start({})
                    await self.m_cam_start({})
                    await self.m_grip_connect({"host": self.robot_host})
                    await asyncio.sleep(0.3)
                    await self.call(self.grip.activate)
            await asyncio.gather(self.state_loop(), self.unity_loop())

    def shutdown(self):
        try:
            if self.exec and self.exec.running():
                self.exec.stop()
            self.imus.stop()
            self.cams.stop()
            self.grip.close()
            self.X.UR.stop()
        finally:
            if self.mockur:
                self.mockur.terminate()
                try:
                    self.mockur.wait(3)
                except Exception:
                    self.mockur.kill()


# ---------------------------------------------------------------------------------------------- helpers
def _json_default(o):
    try:
        import numpy as np
        if isinstance(o, np.ndarray):
            return o.tolist()
        if isinstance(o, np.generic):
            return o.item()
    except Exception:
        pass
    return str(o)


def _slug(s):
    return "".join(ch if ch.isalnum() else "_" for ch in s.lower()).strip("_")[:40]


def _payloads():
    p = os.path.join(ROOT, "config", "payloads.json")
    if not os.path.exists(p):
        return {}
    d = json.load(open(p))
    return {k: {"label": v.get("label"), "mass_kg": v.get("mass_kg"), "cog_m": v.get("cog_m")} for k, v in d.items() if isinstance(v, dict)}


def _start_mockur(host):
    """The stand-in controller in its own process, so its 500 Hz clock does not share a GIL with the agent."""
    import subprocess
    import sys
    env = dict(os.environ)
    env["PYTHONPATH"] = ROOT + os.pathsep + env.get("PYTHONPATH", "")
    p = subprocess.Popen([sys.executable, "-m", "console.mockur", "--host", host], cwd=ROOT, env=env,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    t0 = time.time()
    while time.time() - t0 < 20:
        if p.poll() is not None:
            raise RuntimeError("the mock controller exited at start (ports 29999/30002/30004 in use?)")
        try:
            socket.create_connection((host, 29999), timeout=0.3).close()
            return p
        except OSError:
            time.sleep(0.2)
    p.kill()
    raise RuntimeError("the mock controller did not open its ports")


def _dashboard(host, cmd, timeout=3.0):
    """Talk to the dashboard port directly. Used only for the mock controller's play extension."""
    try:
        with socket.create_connection((host, 29999), timeout=timeout) as s:
            s.settimeout(timeout)
            s.recv(1024)
            s.sendall((cmd + "\n").encode())
            return True, s.recv(4096).decode(errors="ignore").strip()
    except Exception as e:
        return False, str(e)


def _bench_check(pairs, caps, units, hist):
    import numpy as np
    from sonair.jetson import lsm6dsv16x as imu_dev
    from sonair.jetson.clock import ClockMap
    out = {"ok": True, "robot": {}, "imu": {}}
    pr = [(a, b) for a, b in pairs if a is not None and b is not None]
    if len(pr) > 50:
        cm = ClockMap().fit(np.array([p[0] for p in pr]), np.array([p[1] for p in pr]))
        out["robot"] = {"packets": len(pr), "clock": cm.summary()}
    _, Q, QD = hist.arrays()
    still = bool(len(QD) and np.abs(QD[-min(len(QD), 2500):]).max() < 2e-3)
    out["robot"]["still"] = still
    for n, cap in caps.items():
        u = units[n]
        ts, acc, gyr, bdev, bhost = [], [], [], [], []
        for raw, th in cap:
            t, a, g, _ = imu_dev.decode_fifo(raw, u.acc_fs_g, u.gyr_fs_dps)
            if len(t):
                ts.append(t); acc.append(a); gyr.append(g); bdev.append(t[-1]); bhost.append(th)
        if not ts:
            out["imu"][n] = {"error": "no samples"}; continue
        t = np.concatenate(ts); a = np.concatenate(acc); g = np.concatenate(gyr)
        r = {"samples": int(len(t)), "rate_hz": round(float(1 / np.median(np.diff(t))), 2),
             "gravity_norm": round(float(np.linalg.norm(a.mean(0))), 4), "accel_mean": a.mean(0).round(4).tolist(),
             "gyro_bias_dps": np.degrees(g.mean(0)).round(4).tolist(), "gyro_noise_dps": np.degrees(g.std(0)).round(4).tolist()}
        if len(bdev) > 10:
            r["clock"] = ClockMap().fit(np.array(bdev), np.array(bhost)).summary()
        r["verdict"] = "PASS" if abs(r["gravity_norm"] - 9.80665) < 0.15 and max(abs(x) for x in r["gyro_bias_dps"]) < 1.0 else "CHECK"
        out["imu"][n] = r
    if not still:
        out["note"] = "the robot moved during the check: bias and gravity are not valid"
    return out


async def main_async(a):
    token = a.token
    if a.bind not in ("127.0.0.1", "localhost") and not token:
        token = secrets.token_urlsafe(9)
    agent = Agent(mock=a.mock, robot=a.robot, runs=a.runs, unity=a.unity, token=token, robot_hz=a.robot_hz)
    try:
        if a.robot and not a.mock:
            await agent.call(agent.X.handle_message, {"type": "ur_service_start", "host": a.robot, "frequency": a.robot_hz})
        await agent.serve(a.bind, a.port)
    finally:
        agent.shutdown()

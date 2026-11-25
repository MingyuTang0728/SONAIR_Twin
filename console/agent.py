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
"""
from __future__ import annotations

import asyncio
import json
import logging
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
from . import data as datamod, jobs, recorder

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
                "safety_mode")


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
        return bool((self.exec and self.exec.running()) or (self.step_task and not self.step_task.done()))

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
                "jog": (self.X.UR.jog.status() if self.X.UR.jog else None), "locked_ok": self.locked_ok}

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
                "presets": list(jobs.PRESETS), "payloads": _payloads(), "runs_dir": os.path.relpath(self.runs, ROOT)}

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

    # ------------------------------------------------------------------------------------------ run
    async def serve(self, bind="127.0.0.1", port=8765):
        self.loop = asyncio.get_running_loop()

        def process_request(connection, request):
            u = urlparse(request.path)
            if u.path == "/ws":
                if self.token and parse_qs(u.query).get("token", [""])[0] != self.token:
                    return Response(403, "Forbidden", Headers([("Content-Type", "text/plain")]), b"token required\n")
                return None
            name = "index.html" if u.path in ("", "/") else u.path.lstrip("/")
            p = os.path.normpath(os.path.join(WEB, name))
            if not p.startswith(WEB) or not os.path.isfile(p):
                return Response(404, "Not Found", Headers([("Content-Type", "text/plain")]), b"not found\n")
            ctype = mimetypes.guess_type(p)[0] or "application/octet-stream"
            body = open(p, "rb").read()
            return Response(200, "OK", Headers([("Content-Type", ctype + ("; charset=utf-8" if ctype.startswith("text") or "javascript" in ctype else "")),
                                                ("Content-Length", str(len(body))), ("Cache-Control", "no-store")]), body)

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
            await asyncio.gather(self.state_loop(), self.unity_loop())

    def shutdown(self):
        try:
            if self.exec and self.exec.running():
                self.exec.stop()
            self.imus.stop()
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

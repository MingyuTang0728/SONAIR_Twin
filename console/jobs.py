"""Motion jobs the console starts: the checked-command executor, and campaign plans built from versioned commands.

The console never streams a trajectory itself. A command moves the robot only through sonair.real.executor
(servoJ of a command that passed its speed and collision checks, half speed by default) run as a subprocess, so the
same executor and the same refusals apply whether it is started from a terminal or from the console.
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque

from sonair import cell

ROOT = cell.ROOT


def command_index():
    p = os.path.join(ROOT, "commands", "index.json")
    if not os.path.exists(p):
        return []
    out = []
    for c in json.load(open(p)):
        meta = {}
        mp = os.path.join(ROOT, "commands", c["id"], "command.json")
        if os.path.exists(mp):
            m = json.load(open(mp))
            meta = {k: m.get(k) for k in ("payload", "payload_schedule", "kind", "description", "collision_check")}
            if isinstance(meta.get("collision_check"), dict):
                meta["collision_check"] = "FAILED"
            pk = m.get("peak_qd")                      # per joint in command.json: the executor checks the largest
            if isinstance(pk, list) and pk:
                meta["peak_qd"] = round(max(abs(float(x)) for x in pk), 4)
            elif isinstance(pk, (int, float)):
                meta["peak_qd"] = float(pk)
        out.append({**c, **meta})
    return out


class ExecJob:
    """One run of sonair.real.executor. Lines of its output go to the console log as they come."""

    def __init__(self, cmd, speed=0.5, max_qd=0.6, run_id=None, robot=None, mock=False, rep=0, robot_id="UR5e-UoN-1",
                 payload_on_pendant="", on_line=None):
        self.cmd, self.speed, self.max_qd, self.run_id, self.robot, self.mock = cmd, speed, max_qd, run_id, robot, mock
        self.rep, self.robot_id, self.payload_on_pendant = rep, robot_id, payload_on_pendant
        self.on_line = on_line or (lambda s: None)
        self.lines = deque(maxlen=200)
        self.proc = None
        self.rc = None
        self.t0 = None
        self.refused = None

    def argv(self):
        a = [sys.executable, "-u", "-m", "sonair.real.executor", "--cmd", self.cmd, "--speed", f"{self.speed:g}",
             "--max-qd", f"{self.max_qd:g}", "--rep", str(self.rep), "--robot-id", self.robot_id, "--yes"]
        a += ["--mock"] if self.mock else ["--robot", self.robot]
        if self.run_id:
            a += ["--run-id", self.run_id]
        if self.payload_on_pendant:
            a += ["--payload-set-on-pendant", self.payload_on_pendant]
        return a

    def start(self):
        env = dict(os.environ)
        env["PYTHONPATH"] = ROOT + os.pathsep + env.get("PYTHONPATH", "")
        kw = {}
        if os.name == "nt":
            kw["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        self.t0 = time.time()
        self.proc = subprocess.Popen(self.argv(), cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, bufsize=1, **kw)
        threading.Thread(target=self._pump, daemon=True, name="exec-out").start()
        return self

    def _pump(self):
        for line in self.proc.stdout:
            line = line.rstrip()
            if not line:
                continue
            if line.startswith("REFUSED") or line.startswith("robot not ready"):
                self.refused = line
            self.lines.append(line)
            self.on_line(line)
        self.rc = self.proc.wait()

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def stop(self):
        """Ask the executor to stop the way Ctrl+C does (servoStop, stopScript, keep the recording), then insist."""
        if not self.running():
            return
        try:
            self.proc.send_signal(signal.CTRL_BREAK_EVENT if os.name == "nt" else signal.SIGINT)
            self.proc.wait(2.0)
        except Exception:
            try:
                self.proc.terminate(); self.proc.wait(2.0)
            except Exception:
                self.proc.kill()

    def status(self):
        return {"cmd": self.cmd, "speed": self.speed, "running": self.running(), "rc": self.rc, "refused": self.refused,
                "elapsed_s": round(time.time() - self.t0, 1) if self.t0 else 0, "tail": list(self.lines)[-6:], "mock": self.mock}


# ---------------------------------------------------------------------------------------------- campaign plans
PRESETS = {
    "E1 excitation (identification)": dict(match="exc_", experiment="E1", repeats=2),
    "Task A and B (half speed)": dict(match="task_", suffix="_x0.5", experiment="E2", repeats=3),
    "Calibration paths": dict(match="task_CAL_", suffix="_x0.5", experiment="CAL", repeats=1),
}


def plan_from(commands, speed=1.0, repeats=1, experiment="E2", payload_on_pendant="", settle_s=5.0, name="custom"):
    steps = []
    for r in range(int(repeats)):
        for cid in commands:
            steps.append(dict(cmd=cid, speed=float(speed), rep=r, experiment=experiment, payload_on_pendant=payload_on_pendant,
                              state="pending", run_id=None, qc=None))
    return dict(name=name, steps=steps, settle_s=float(settle_s), created_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()))


def preset_plan(name, speed=1.0):
    p = PRESETS[name]
    ids = [c["id"] for c in command_index() if c["id"].startswith(p["match"]) and
           (c["id"].endswith(p["suffix"]) if p.get("suffix") else "_x" not in c["id"])]
    return plan_from(ids, speed=speed, repeats=p["repeats"], experiment=p["experiment"], name=name)

"""A stand-in UR controller on localhost, so the console can be exercised end to end with no robot and no URSim.

It speaks the controller's own protocols on the controller's own ports, so the UNCHANGED platform modules
(ur_telemetry, ur_control, ur_jog, ur_bridge_ext) connect to it exactly as they connect to the real UR5e:

    30004  RTDE: protocol version, controller version, output recipe (any field in ur_telemetry.OUTPUT_RECIPE),
           input recipe (speed slider), start, data packages at the requested frequency (500 Hz max)
    29999  dashboard server: one line in, one line out (power on, brake release, robotmode, stop, ...)
    30002  secondary interface: URScript lines (speedl, speedj, movej, movel, servoj, stopl, stopj)

The plant is kinematic: a commanded joint motion plus an 8 ms first-order lag for the measured joints, a gravity-like
current model and the tool accelerometer. It is NOT a simulator of the robot and nothing it produces enters a dataset:
every session recorded against it is labelled mock by the console.

One extension that a real controller does not have, used only by the console in --mock mode:
    dashboard "sonair mock play <command_id> <speed>"   plays a versioned command (commands/<id>) like the executor would
    dashboard "sonair mock state"                        JSON with the plant state

    python -m console.mockur --host 127.0.0.1
"""
from __future__ import annotations

import argparse
import json
import math
import re
import socket
import struct
import threading
import time

import numpy as np

from sonair import cell, kin

_V, _v, _O, _I, _S, _P, _U, _M = 86, 118, 79, 73, 83, 80, 85, 77
DT = 0.002
HOME = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
VERSION = (5, 15, 2, 0)

# the field types a controller reports (a subset of what a real one knows is enough: unknown -> NOT_FOUND)
TYPES = {}
try:
    from cell_platform import paths as _paths
    _paths.add()
    import ur_telemetry as _urt
    TYPES = dict(_urt.OUTPUT_RECIPE)
    _FMT = _urt._FMT
except Exception:                                   # pragma: no cover - the platform travels with the repo
    _FMT = {"DOUBLE": ("d", 8), "UINT32": ("I", 4), "UINT64": ("Q", 8), "INT32": ("i", 4),
            "VECTOR3D": ("3d", 24), "VECTOR6D": ("6d", 48), "VECTOR6INT32": ("6i", 24)}
_N = {"VECTOR6D": 6, "VECTOR6INT32": 6, "VECTOR6UINT32": 6, "VECTOR3D": 3}


def _pkt(cmd: int, body: bytes = b"") -> bytes:
    return struct.pack(">HB", 3 + len(body), cmd) + body


def _recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        c = sock.recv(n - len(buf))
        if not c:
            raise ConnectionError("closed")
        buf += c
    return buf


def _recv_pkt(sock):
    size, cmd = struct.unpack(">HB", _recv_exact(sock, 3))
    return cmd, (_recv_exact(sock, size - 3) if size > 3 else b"")


def _encode(recipe, st):
    fmt, vals = ">", []
    for name, typ in recipe:
        fmt += _FMT[typ][0]
        n = _N.get(typ, 1)
        v = st.get(name)
        if v is None:
            v = [0] * n if n > 1 else 0
        if n > 1:
            vals.extend(int(x) if "INT" in typ else float(x) for x in list(v)[:n])
        else:
            vals.append(int(v) if typ in ("UINT32", "UINT64", "INT32") else float(v))
    return struct.pack(fmt, *vals)


def _floats(s):
    return [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", s)]


def _kw(line, key, default):
    m = re.search(rf"\b{key}\s*=\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", line)
    return float(m.group(1)) if m else default


def _pose(q):
    T = kin.fk(np.asarray(q, float))
    return np.r_[T[:3, 3], cell.rotvec(T[:3, :3])], T


def _jac(q):
    """Numerical geometric Jacobian of tool0: linear velocity and angular velocity in the base frame."""
    q = np.asarray(q, float)
    T0 = kin.fk(q)
    J = np.zeros((6, 6))
    for i in range(6):
        dq = np.zeros(6); dq[i] = 1e-6
        T1 = kin.fk(q + dq)
        J[:3, i] = (T1[:3, 3] - T0[:3, 3]) / 1e-6
        dR = T1[:3, :3] @ T0[:3, :3].T
        J[3:, i] = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / 2e-6
    return J


def _ik(q0, pose, iters=60):
    """Damped least squares from q0 to a tool0 pose (x y z rx ry rz). Good for the short moves a console makes."""
    q = np.asarray(q0, float).copy()
    p_goal = np.asarray(pose[:3], float)
    R_goal = cell.rotmat(np.asarray(pose[3:6], float)) if hasattr(cell, "rotmat") else _rv2m(pose[3:6])
    for _ in range(iters):
        T = kin.fk(q)
        e_p = p_goal - T[:3, 3]
        dR = R_goal @ T[:3, :3].T
        e_r = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / 2
        e = np.r_[e_p, e_r]
        if np.linalg.norm(e) < 1e-7:
            break
        J = _jac(q)
        q += J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e)
    return q


def _rv2m(rv):
    rv = np.asarray(rv, float); th = np.linalg.norm(rv)
    if th < 1e-12:
        return np.eye(3)
    k = rv / th; K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


class Plant:
    """Joint-space motion source and measured-joint model, stepped at 500 Hz."""

    TAU_LAG = 0.008

    def __init__(self):
        self.lock = threading.Lock()
        self.tq = np.array(HOME, float); self.tqd = np.zeros(6); self.tqdd = np.zeros(6)
        self.q = self.tq.copy(); self.qd = np.zeros(6)
        self.mode = 3                      # POWER_OFF until the dashboard powers it on
        self.safety = 1
        self.slider = 1.0
        self.src = ("idle",)               # ("speed", qd, a, deadline) | ("traj", Q, k) | ("stop", a)
        self.t_ctrl = 4321.0
        self.playing = False
        self.prev_pose = None

    # -- motion requests (called from the socket threads) --------------------------------------------------
    def running(self):
        return self.mode == 7 and self.safety == 1

    def speedj(self, qd, a, t):
        with self.lock:
            if self.running():
                self.src = ("speed", np.asarray(qd, float)[:6], max(0.1, a), time.monotonic() + max(0.02, t))

    def speedl(self, xd, a, t):
        with self.lock:
            q = self.tq.copy()
        J = _jac(q)
        qd = np.linalg.solve(J.T @ J + 1e-4 * np.eye(6), J.T @ np.asarray(xd, float)[:6])
        self.speedj(qd, a * 4.0, t)

    def move_to(self, q_goal, v):
        """A joint move with a smooth (cosine) profile, duration from the largest joint step and speed v."""
        with self.lock:
            if not self.running():
                return
            q0 = self.tq.copy()
        q_goal = np.asarray(q_goal, float)
        T = max(0.2, float(np.max(np.abs(q_goal - q0))) / max(0.01, v) * 1.5)
        s = 0.5 - 0.5 * np.cos(np.pi * np.linspace(0, 1, int(T / DT) + 1))
        with self.lock:
            self.src = ("traj", q0 + s[:, None] * (q_goal - q0), 0)

    def play(self, Q):
        with self.lock:
            if not self.running():
                return False
            q0 = self.tq.copy()
        T = max(0.5, float(np.max(np.abs(Q[0] - q0))) / 0.3 * 1.5)
        s = 0.5 - 0.5 * np.cos(np.pi * np.linspace(0, 1, int(T / DT) + 1))
        lead = q0 + s[:, None] * (Q[0] - q0)
        hold = np.repeat(Q[:1], int(0.5 / DT), axis=0)
        with self.lock:
            self.src = ("traj", np.vstack([lead, hold, Q, np.repeat(Q[-1:], int(0.3 / DT), axis=0)]), 0)
        return True

    def servoj(self, q):
        with self.lock:
            if self.running():
                self.src = ("traj", np.asarray([q], float), 0)

    def stop(self, a):
        with self.lock:
            self.src = ("stop", max(0.5, a))

    # -- one 2 ms step ----------------------------------------------------------------------------------------
    def step(self):
        with self.lock:
            self.t_ctrl += DT
            src = self.src
            tqd_prev = self.tqd.copy()
            if not self.running() and src[0] != "idle":
                src = ("stop", 3.0)
            if src[0] == "speed":
                want = src[1] * self.slider if time.monotonic() < src[3] else np.zeros(6)
                step = src[2] * DT
                self.tqd = self.tqd + np.clip(want - self.tqd, -step, step)
                if time.monotonic() >= src[3] and not np.any(self.tqd):
                    self.src = ("idle",)
                self.tq = self.tq + self.tqd * DT
            elif src[0] == "traj":
                Q, k = src[1], src[2]
                kk = min(int(k), len(Q) - 1)
                new = Q[kk]
                self.tqd = (new - self.tq) / DT if len(Q) > 1 else np.zeros(6)
                self.tq = new.copy()
                if k + self.slider >= len(Q):
                    self.src = ("idle",); self.tqd = np.zeros(6)
                else:
                    self.src = ("traj", Q, k + self.slider)
            elif src[0] == "stop":
                step = src[1] * DT
                self.tqd = self.tqd - np.clip(self.tqd, -step, step)
                self.tq = self.tq + self.tqd * DT
                if not np.any(np.abs(self.tqd) > 1e-9):
                    self.tqd = np.zeros(6); self.src = ("idle",)
            else:
                self.tqd = np.zeros(6)
            self.tqdd = (self.tqd - tqd_prev) / DT
            q_prev = self.q.copy()
            src_q = self.tq
            self.q = self.q + (src_q - self.q) * (DT / self.TAU_LAG)
            self.qd = (self.q - q_prev) / DT
            self.playing = self.src[0] != "idle"
            return self._state()

    def _state(self):
        q, qd = self.q, self.qd
        pose, T = _pose(q)
        tpose, _ = _pose(self.tq)
        speed = np.zeros(6) if self.prev_pose is None else (pose - self.prev_pose) / DT
        self.prev_pose = pose
        g = np.array([0, 0, -9.80665])
        grav = np.array([0.0, 20 * math.sin(q[1]) + 8 * math.sin(q[1] + q[2]), 8 * math.sin(q[1] + q[2]), 0.6, 0.2, 0.0])
        cur = 0.09 * grav + 0.7 * np.tanh(qd / 0.01) * (np.abs(qd) > 1e-5)
        on = self.mode >= 5
        return {
            "timestamp": self.t_ctrl, "actual_q": q, "actual_qd": qd, "actual_current": cur, "target_current": cur,
            "joint_temperatures": [31.0, 32.5, 32.0, 35.1, 35.4, 35.0], "target_q": self.tq, "target_qd": self.tqd,
            "target_qdd": self.tqdd, "target_moment": grav * 9.0, "actual_TCP_pose": pose, "actual_TCP_speed": speed,
            "actual_TCP_force": [0.0] * 6, "target_TCP_pose": tpose, "actual_digital_input_bits": 0,
            "actual_digital_output_bits": 0, "robot_mode": self.mode, "safety_mode": self.safety, "safety_status": self.safety,
            "runtime_state": 2 if self.playing else 1, "robot_status_bits": (1 if on else 0) | (2 if self.playing else 0),
            "safety_status_bits": 1, "actual_robot_voltage": 48.0 if on else 0.0, "actual_robot_current": 0.6 if on else 0.0,
            "actual_tool_accelerometer": T[:3, :3].T @ (-g), "tcp_force_scalar": 0.0, "speed_scaling": self.slider,
            "joint_control_output": cur, "actual_joint_voltage": [47.5] * 6, "joint_mode": [253 if self.mode == 7 else 251] * 6,
            "actual_main_voltage": 48.0, "actual_execution_time": 0.3, "actual_momentum": 0.0,
            "tool_output_voltage": 0, "tool_output_current": 0.0, "tool_temperature": 33.0,
        }


class MockUR:
    def __init__(self, host="127.0.0.1"):
        self.host = host
        self.plant = Plant()
        self.clients = []                  # [sock, recipe, decimation]
        self.clients_lock = threading.Lock()
        self.stop_ = threading.Event()
        self.servers = []
        self.k = 0

    # -- sockets ---------------------------------------------------------------------------------------------
    def _listen(self, port, target):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.host, port)); s.listen(8); self.servers.append(s)

        def accept():
            while not self.stop_.is_set():
                try:
                    c, _ = s.accept()
                except OSError:
                    return
                threading.Thread(target=target, args=(c,), daemon=True).start()
        threading.Thread(target=accept, daemon=True, name=f"mockur-{port}").start()

    def _rtde(self, c):
        recipe, decim, inputs = [], 1, None
        try:
            while not self.stop_.is_set():
                try:
                    cmd, body = _recv_pkt(c)
                except socket.timeout:            # a streaming client rarely talks; the timeout is for our sends
                    continue
                if cmd == _V:
                    c.sendall(_pkt(_V, b"\x01"))
                elif cmd == _v:
                    c.sendall(_pkt(_v, struct.pack(">IIII", *VERSION)))
                elif cmd == _O:
                    freq = struct.unpack(">d", body[:8])[0]
                    names = body[8:].decode().split(",")
                    recipe = [(n, TYPES.get(n, "NOT_FOUND")) for n in names]
                    c.sendall(_pkt(_O, b"\x01" + ",".join(t for _, t in recipe).encode()))
                    recipe = [(n, t) for n, t in recipe if t != "NOT_FOUND"]
                    decim = max(1, int(round(500.0 / min(500.0, max(1.0, freq)))))
                elif cmd == _I:
                    names = body.decode().split(",")
                    known = {"speed_slider_mask": "UINT32", "speed_slider_fraction": "DOUBLE"}
                    inputs = [known.get(n, "NOT_FOUND") for n in names]
                    c.sendall(_pkt(_I, b"\x02" + ",".join(inputs).encode()))
                elif cmd == _S:
                    c.sendall(_pkt(_S, b"\x01"))
                    if recipe and inputs is None:
                        c.settimeout(2.0)
                        with self.clients_lock:
                            self.clients.append([c, recipe, decim])
                elif cmd == _P:
                    with self.clients_lock:
                        self.clients = [x for x in self.clients if x[0] is not c]
                    c.sendall(_pkt(_P, b"\x01"))
                elif cmd == _U and inputs is not None and len(body) >= 13:
                    mask = struct.unpack(">I", body[1:5])[0]
                    frac = struct.unpack(">d", body[5:13])[0]
                    if mask & 1:
                        self.plant.slider = max(0.01, min(1.0, frac))
        except Exception:
            pass
        finally:
            with self.clients_lock:
                self.clients = [x for x in self.clients if x[0] is not c]
            try:
                c.close()
            except OSError:
                pass

    def _dashboard(self, c):
        p = self.plant
        try:
            c.sendall(b"Connected: Universal Robots Dashboard Server\n")
            f = c.makefile("r")
            for line in f:
                cmd = line.strip()
                low = cmd.lower()
                if low == "power on":
                    if p.mode < 5: p.mode = 5
                    r = "Powering on"
                elif low == "power off":
                    p.mode = 3; p.stop(5.0); r = "Powering off"
                elif low == "brake release":
                    if p.mode >= 5: p.mode = 7
                    r = "Brake releasing"
                elif low == "robotmode":
                    r = "Robotmode: " + {3: "POWER_OFF", 5: "IDLE", 7: "RUNNING"}.get(p.mode, str(p.mode))
                elif low == "safetystatus":
                    r = "Safetystatus: NORMAL" if p.safety == 1 else "Safetystatus: PROTECTIVE_STOP"
                elif low == "programstate":
                    r = ("PLAYING" if p.playing else "STOPPED") + " <unnamed>"
                elif low == "is in remote control":
                    r = "true"
                elif low == "get loaded program":
                    r = "No program loaded"
                elif low.startswith("load "):
                    r = f"Loading program: {cmd[5:]}"
                elif low == "play":
                    r = "Starting program"
                elif low == "stop":
                    p.stop(3.0); r = "Stopped"
                elif low == "pause":
                    p.stop(3.0); r = "Pausing program"
                elif low == "unlock protective stop":
                    p.safety = 1; r = "Protective stop releasing"
                elif low in ("close popup", "close safety popup"):
                    r = "closing popup"
                elif low.startswith("sonair mock play"):
                    r = self._mock_play(cmd.split()[3:])
                elif low == "sonair mock pstop":
                    p.safety = 3; p.stop(10.0); r = "mock protective stop"
                elif low == "sonair mock state":
                    with p.lock:
                        r = json.dumps(dict(mode=p.mode, safety=p.safety, playing=p.playing, q=p.q.round(6).tolist(),
                                            slider=p.slider, t_ctrl=round(p.t_ctrl, 3)))
                else:
                    r = f"could not understand: '{cmd}'"
                c.sendall((r + "\n").encode())
        except Exception:
            pass
        finally:
            try:
                c.close()
            except OSError:
                pass

    def _mock_play(self, args):
        from sonair import commands as cm
        try:
            cid = args[0]; speed = float(args[1]) if len(args) > 1 else 0.5
            c = cm.load(cid)
            if abs(speed - 1.0) > 1e-9:
                c = cm.time_scale(c, speed, cid=cid)
            Q = c["df"][cm.QCOLS].to_numpy()
        except Exception as e:
            return f"mock play failed: {e}"
        return "Playing " + cid if self.plant.play(Q) else "mock play refused: robot not RUNNING"

    def _script(self, c):
        p = self.plant
        try:
            f = c.makefile("r")
            for line in f:
                s = line.strip()
                if s.startswith("speedl("):
                    v = _floats(s.split("]")[0]); p.speedl(v[:6], _kw(s, "a", 0.5), _kw(s, "t", 0.1))
                elif s.startswith("speedj("):
                    v = _floats(s.split("]")[0]); p.speedj(v[:6], _kw(s, "a", 1.0), _kw(s, "t", 0.1))
                elif s.startswith(("stopl(", "stopj(")):
                    v = _floats(s); p.stop(v[0] if v else 2.0)
                elif s.startswith("movej("):
                    v = _floats(s.split("]")[0])[:6]
                    with p.lock:
                        q0 = p.tq.copy()
                    goal = _ik(q0, v) if "p[" in s.split("]")[0] else np.array(v)
                    p.move_to(goal, _kw(s, "v", 0.5))
                elif s.startswith(("movel(", "movep(")):
                    v = _floats(s.split("]")[0])[:6]
                    with p.lock:
                        q0 = p.tq.copy()
                    p.move_to(_ik(q0, v), 0.25)
                elif s.startswith("servoj("):
                    v = _floats(s.split("]")[0])[:6]; p.servoj(v)
        except Exception:
            pass
        finally:
            try:
                c.close()
            except OSError:
                pass

    # -- the 500 Hz loop -------------------------------------------------------------------------------------
    def _run(self):
        t_next = time.perf_counter()
        while not self.stop_.is_set():
            now = time.perf_counter()
            if now < t_next:
                time.sleep(max(0.0, t_next - now - 0.0002))
                continue
            t_next += DT
            if now - t_next > 0.2:                 # far behind (machine suspended): resynchronise
                t_next = now
            st = self.plant.step()
            self.k += 1
            with self.clients_lock:
                clients = list(self.clients)
            dead = []
            for cl in clients:
                if self.k % cl[2]:
                    continue
                try:
                    cl[0].sendall(_pkt(_U, b"\x01" + _encode(cl[1], st)))
                except Exception:
                    dead.append(cl[0])
            if dead:
                with self.clients_lock:
                    self.clients = [x for x in self.clients if x[0] not in dead]

    def start(self):
        self._listen(30004, self._rtde)
        self._listen(29999, self._dashboard)
        self._listen(30002, self._script)
        threading.Thread(target=self._run, daemon=True, name="mockur-plant").start()
        return self

    def stop(self):
        self.stop_.set()
        for s in self.servers:
            try:
                s.close()
            except OSError:
                pass


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--host", default="127.0.0.1")
    a = ap.parse_args(argv)
    m = MockUR(a.host).start()
    print(f"mock UR controller on {a.host}: RTDE 30004, dashboard 29999, script 30002 (Ctrl+C to stop)")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        m.stop()


if __name__ == "__main__":
    main()

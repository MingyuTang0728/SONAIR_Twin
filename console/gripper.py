"""The Robotiq 2F-85 on the flange, through the Robotiq URCap's socket on the controller (port 63352).

ASCII, one line each way: "SET POS 128 SPE 255 FOR 100 GTO 1" -> "ack", "GET POS" -> "POS 128". The variables used:
    ACT  1 activates (a reset gripper must be activated once after power-up; the fingers move fully during it)
    GTO  1 go to the requested position, 0 stop where it is
    POS  requested position 0 (open, 85 mm) .. 255 (closed); the fingers meet at about 227
    SPE  speed 0..255, FOR force 0..255
    STA  0 reset, 1 activating, 3 active      OBJ  0 moving, 1 or 2 stopped on an object, 3 at the position
    FLT  fault code (0 none)                  PRE  echo of the requested position

This is display and manual control only. A versioned command does not drive the gripper: the executor streams
joints, and synchronising gripper events with servoJ is milestone M2 (to be verified on URSim first).
"""
from __future__ import annotations

import socket
import threading
import time

PORT = 63352
CLOSED_COUNTS = 227.0          # where the 2F-85 fingers meet, approximately; the console's width display uses it
STROKE_MM = 85.0


class Gripper:
    def __init__(self):
        self.host = None
        self.sock = None
        self.lock = threading.Lock()
        self.state = {"connected": False}
        self.error = ""
        self.poll_thread = None
        self.stop_ = threading.Event()

    # -- link ---------------------------------------------------------------------------------------------------
    def connect(self, host, port=PORT):
        self.close()
        try:
            s = socket.create_connection((host, port), timeout=2.0)
            s.settimeout(1.0)
        except OSError as e:
            self.error = f"no gripper socket on {host}:{port} ({e}). Is the Robotiq URCap installed and the gripper on?"
            return {"ok": False, "error": self.error}
        with self.lock:
            self.sock, self.host = s, host
        self.error = ""
        self.stop_.clear()
        self.poll_thread = threading.Thread(target=self._poll, daemon=True, name="gripper-poll")
        self.poll_thread.start()
        return {"ok": True, "host": host}

    def close(self):
        self.stop_.set()
        with self.lock:
            if self.sock:
                try:
                    self.sock.close()
                except OSError:
                    pass
            self.sock = None
        self.state = {"connected": False}

    def _ask(self, line):
        with self.lock:
            if not self.sock:
                raise ConnectionError("gripper not connected")
            self.sock.sendall((line + "\n").encode())
            buf = b""
            while not buf.endswith(b"\n"):
                chunk = self.sock.recv(256)
                if not chunk:
                    raise ConnectionError("gripper socket closed")
                buf += chunk
            return buf.decode(errors="ignore").strip()

    def _set(self, **kv):
        r = self._ask("SET " + " ".join(f"{k.upper()} {int(v)}" for k, v in kv.items()))
        return r == "ack", r

    def _get(self, var):
        r = self._ask(f"GET {var}")
        parts = r.split()
        return int(parts[1]) if len(parts) == 2 and parts[0] == var and parts[1].lstrip("-").isdigit() else None

    def _poll(self):
        while not self.stop_.is_set():
            try:
                st = {v: self._get(v) for v in ("STA", "POS", "PRE", "OBJ", "FLT")}
                pos = st["POS"]
                self.state = {"connected": True, "host": self.host, "activated": st["STA"] == 3, "sta": st["STA"],
                              "pos": pos, "requested": st["PRE"], "obj": st["OBJ"], "fault": st["FLT"],
                              "object": st["OBJ"] in (1, 2), "moving": st["OBJ"] == 0,
                              "closure": None if pos is None else round(min(1.0, pos / CLOSED_COUNTS), 4),
                              "width_mm": None if pos is None else round(STROKE_MM * max(0.0, 1 - pos / CLOSED_COUNTS), 1),
                              "t": time.time()}
            except Exception as e:
                self.error = f"gripper: {e}"
                self.state = {"connected": False, "error": self.error}
                with self.lock:
                    try:
                        self.sock and self.sock.close()
                    except OSError:
                        pass
                    self.sock = None
                return
            time.sleep(0.15)

    # -- actions ------------------------------------------------------------------------------------------------
    def activate(self):
        try:
            if self._get("STA") == 3:
                return {"ok": True, "msg": "already active"}
            self._set(act=0)
            time.sleep(0.05)
            ok, r = self._set(act=1)
            return {"ok": ok, "msg": "activating: the fingers open and close once" if ok else r}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def move(self, pos, speed=128, force=64):
        try:
            pos, speed, force = (max(0, min(255, int(round(float(x))))) for x in (pos, speed, force))
            ok, r = self._set(pos=pos, spe=speed, **{"for": force}, gto=1)
            return {"ok": ok, "msg": f"to {pos}" if ok else r}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def stop(self):
        try:
            ok, r = self._set(gto=0)
            return {"ok": ok, "msg": "gripper stopped" if ok else r}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def status(self):
        s = dict(self.state)
        if self.error and not s.get("connected"):
            s["error"] = self.error
        return s


# ---------------------------------------------------------------------------------------------- the stand-in gripper
class MockGripperServer:
    """What the URCap socket answers, with fingers that move at the requested speed. Used by console.mockur."""

    def __init__(self, host="127.0.0.1", port=PORT):
        self.host, self.port = host, port
        self.lock = threading.Lock()
        self.v = {"ACT": 0, "GTO": 0, "STA": 0, "POS": 0, "PRE": 0, "SPE": 255, "FOR": 150, "OBJ": 3, "FLT": 0}
        self.pos = 0.0
        self.act_until = 0.0
        self.stop_ = threading.Event()
        self.srv = None

    def start(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.host, self.port)); s.listen(4); self.srv = s
        threading.Thread(target=self._accept, daemon=True, name="mockgrip-accept").start()
        threading.Thread(target=self._run, daemon=True, name="mockgrip-plant").start()
        return self

    def _accept(self):
        while not self.stop_.is_set():
            try:
                c, _ = self.srv.accept()
            except OSError:
                return
            threading.Thread(target=self._client, args=(c,), daemon=True).start()

    def _client(self, c):
        try:
            for line in c.makefile("r"):
                parts = line.strip().split()
                if not parts:
                    continue
                if parts[0] == "GET" and len(parts) == 2:
                    with self.lock:
                        r = f"{parts[1]} {int(self.v.get(parts[1], 0))}"
                elif parts[0] == "SET" and len(parts) >= 3 and len(parts) % 2 == 1:
                    with self.lock:
                        for k, val in zip(parts[1::2], parts[2::2]):
                            self.v[k] = int(val)
                            if k == "POS":
                                self.v["PRE"] = int(val)
                            if k == "ACT" and int(val) == 1 and self.v["STA"] != 3:
                                self.v["STA"] = 1; self.act_until = time.monotonic() + 1.5
                            if k == "ACT" and int(val) == 0:
                                self.v["STA"] = 0
                    r = "ack"
                else:
                    r = "?"
                c.sendall((r + "\n").encode())
        except OSError:
            pass
        finally:
            try:
                c.close()
            except OSError:
                pass

    def _run(self):
        dt = 0.02
        while not self.stop_.is_set():
            time.sleep(dt)
            with self.lock:
                v = self.v
                if v["STA"] == 1:
                    self.pos = 0.0
                    if time.monotonic() >= self.act_until:
                        v["STA"] = 3
                if v["STA"] == 3 and v["GTO"] == 1:
                    rate = 40 + 210 * v["SPE"] / 255.0          # counts per second
                    d = v["PRE"] - self.pos
                    step = max(-rate * dt, min(rate * dt, d))
                    self.pos += step
                    v["OBJ"] = 3 if abs(v["PRE"] - self.pos) < 0.5 else 0
                elif v["STA"] == 3:
                    v["OBJ"] = 3
                v["POS"] = int(round(self.pos))

    def stop(self):
        self.stop_.set()
        try:
            self.srv and self.srv.close()
        except OSError:
            pass

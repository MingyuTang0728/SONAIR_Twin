"""The RTDE interface page: what the controller grants, how well the stream arrives, any field live, raw captures.

Read only, and from the console's own robot link: the monitor subscribes to the packets of ur_bridge_ext.UR (the
platform's unchanged reader), exactly like the session recorder. It never opens a second RTDE connection, so it
cannot take the controller's RTDE slot away from the link or change what the link negotiated.

    packet timing     controller timestamp and host receive time of every packet (last 60 s): rate on both clocks,
                      the controller period, missing controller cycles, host jitter, the transport delay above its
                      lower envelope, packets per source (RTDE or the 30003 fallback)
    field series      any numeric field (or one element of a vector field) at the full packet rate, last 20 s
    capture           every field of every packet for N seconds, written to captures/<name>.csv
"""
from __future__ import annotations

import csv
import os
import threading
import time
from collections import deque

import numpy as np

from sonair import cell

ROOT = cell.ROOT
CAPTURES = os.path.join(ROOT, "captures")

# what each field is, for the page (the controller's own names; units as the RTDE guide gives them)
FIELD_DOC = {
    "timestamp": ("s", "controller time since boot"),
    "actual_q": ("rad", "measured joint positions"), "target_q": ("rad", "commanded joint positions"),
    "actual_qd": ("rad/s", "measured joint velocities"), "target_qd": ("rad/s", "commanded joint velocities"),
    "target_qdd": ("rad/s2", "commanded joint accelerations"),
    "actual_current": ("A", "measured joint currents"), "target_current": ("A", "commanded joint currents"),
    "target_moment": ("N m", "commanded joint torques (controller model)"),
    "joint_control_output": ("A", "joint control currents"), "joint_temperatures": ("C", "joint temperatures"),
    "actual_joint_voltage": ("V", "joint voltages"), "joint_mode": ("", "joint control modes"),
    "actual_TCP_pose": ("m, rad", "tool pose: x y z and rotation vector"), "target_TCP_pose": ("m, rad", "commanded tool pose"),
    "actual_TCP_speed": ("m/s, rad/s", "tool speed"), "actual_TCP_force": ("N, N m", "generalised tool force"),
    "tcp_force_scalar": ("N", "tool force magnitude"), "actual_tool_accelerometer": ("m/s2", "wrist accelerometer"),
    "speed_scaling": ("", "trajectory speed scaling (slider and limits)"), "robot_mode": ("", "robot mode"),
    "safety_mode": ("", "safety mode"), "safety_status": ("", "safety status"), "runtime_state": ("", "program state"),
    "robot_status_bits": ("", "robot status bits"), "safety_status_bits": ("", "safety status bits"),
    "actual_digital_input_bits": ("", "digital inputs"), "actual_digital_output_bits": ("", "digital outputs"),
    "standard_analog_input0": ("V or A", "analog input 0"), "standard_analog_input1": ("V or A", "analog input 1"),
    "standard_analog_output0": ("V or A", "analog output 0"), "standard_analog_output1": ("V or A", "analog output 1"),
    "actual_robot_voltage": ("V", "robot supply voltage"), "actual_robot_current": ("A", "robot supply current"),
    "actual_main_voltage": ("V", "safety board main voltage"), "actual_execution_time": ("ms", "controller real-time thread time"),
    "actual_momentum": ("kg m/s", "robot momentum norm"), "tool_output_voltage": ("V", "tool output voltage"),
    "tool_output_current": ("A", "tool output current"), "tool_temperature": ("C", "tool temperature"),
    "tool_analog_input0": ("V or A", "tool analog input 0"), "tool_analog_input1": ("V or A", "tool analog input 1"),
    "output_double_register_0": ("", "output double register 0"),
}


def _numeric(v):
    if isinstance(v, (int, float)) and not isinstance(v, bool):
        return [float(v)]
    if isinstance(v, (list, tuple, np.ndarray)) and len(v) and all(isinstance(x, (int, float, np.floating, np.integer)) for x in v):
        return [float(x) for x in v]
    return None


class RtdeMonitor:
    TIMING_S = 60.0
    SERIES_S = 20.0

    def __init__(self, hz=500.0):
        self.lock = threading.Lock()
        n = int(self.TIMING_S * hz)
        self.ctrl = deque(maxlen=n)
        self.host = deque(maxlen=n)
        self.src = deque(maxlen=n)
        self.watch: list[tuple[str, int | None]] = [("actual_q", 2), ("target_q", 2), ("actual_current", 2)]
        self.series = deque(maxlen=int(self.SERIES_S * hz))
        self.last = {}
        self.capture = None
        self.capture_meta = None
        self.captures_done = deque(maxlen=20)

    # -- the sink: called by the robot link for every packet, keep it cheap -------------------------------
    def push(self, st):
        ts, th = st.get("timestamp"), st.get("_mono")
        src = st.get("_source") or "?"
        row = None
        with self.lock:
            if ts is not None and th is not None:
                self.ctrl.append(float(ts)); self.host.append(float(th)); self.src.append(src)
            if self.watch and ts is not None:
                vals = []
                for name, idx in self.watch:
                    v = st.get(name)
                    try:
                        vals.append(float(v[idx]) if idx is not None else float(v))
                    except (TypeError, ValueError, IndexError):
                        vals.append(float("nan"))
                row = (float(ts), vals)
                self.series.append(row)
            self.last = st
            cap = self.capture
        if cap is not None:
            cap.append(st)
            if time.monotonic() >= self.capture_meta["until"]:
                self._finish_capture()

    # -- what the page asks for ------------------------------------------------------------------------------
    def set_watch(self, fields):
        w = []
        for f in fields[:8]:
            name, _, idx = str(f).partition(":")
            w.append((name, int(idx) if idx.strip().isdigit() else None))
        with self.lock:
            self.watch = w
            self.series.clear()
        return {"ok": True, "watch": [f"{n}:{i}" if i is not None else n for n, i in w]}

    def get_series(self, max_points=1500, seconds=None):
        with self.lock:
            rows = list(self.series)
            watch = list(self.watch)
        if seconds and rows:
            t_end = rows[-1][0]
            rows = [r for r in rows if r[0] >= t_end - float(seconds)]
        step = max(1, -(-len(rows) // max_points))          # ceiling: never more than max_points
        rows = rows[::step]
        t0 = rows[-1][0] if rows else 0.0
        t = [round(r[0] - t0, 4) for r in rows]
        ys = [[(None if not np.isfinite(r[1][k]) else round(r[1][k], 6)) for r in rows] for k in range(len(watch))]
        return {"t": t, "y": ys, "names": [f"{n}[{i}]" if i is not None else n for n, i in watch]}

    def fields(self, granted):
        """Every field the link was granted, with its type, unit, meaning and the latest value."""
        with self.lock:
            st = dict(self.last)
        types = {}
        try:
            import ur_telemetry
            types = dict(ur_telemetry.OUTPUT_RECIPE)
        except Exception:
            pass
        out = []
        for name in granted or [k for k in st if not k.startswith("_")]:
            unit, what = FIELD_DOC.get(name, ("", ""))
            v = st.get(name)
            nv = _numeric(v)
            out.append({"name": name, "type": types.get(name, ""), "unit": unit, "what": what,
                        "value": [round(x, 6) for x in nv] if nv else (v if isinstance(v, (str, int, float)) else None),
                        "n": len(nv) if nv else 0})
        return out

    def stats(self, window_s=10.0, nominal_hz=500.0):
        with self.lock:
            c = np.fromiter(self.ctrl, float, len(self.ctrl))
            h = np.fromiter(self.host, float, len(self.host))
            s = list(self.src)
        out = {"packets": int(len(c)), "window_s": window_s}
        if len(c) < 10:
            return out
        k = c >= c[-1] - window_s
        c, h, s = c[k], h[k], [x for x, kk in zip(s, k) if kk]
        dc, dh = np.diff(c), np.diff(h)
        period = float(np.median(dc)) if len(dc) else 0.0
        out.update({
            "packets_window": int(len(c)),
            "rate_ctrl_hz": round((len(c) - 1) / max(1e-9, c[-1] - c[0]), 2),
            "rate_host_hz": round((len(h) - 1) / max(1e-9, h[-1] - h[0]), 2),
            "period_ms": round(period * 1e3, 4),
            "missing_cycles": int(np.sum(np.maximum(0, np.round(dc / period) - 1))) if period > 0 else None,
            "max_gap_ms": round(float(dc.max()) * 1e3, 3),
            "repeats": int(np.sum(dc <= 0)),
            "host_jitter_p50_ms": round(float(np.percentile(np.abs(dh - dc), 50)) * 1e3, 3),
            "host_jitter_p99_ms": round(float(np.percentile(np.abs(dh - dc), 99)) * 1e3, 3),
            "host_jitter_max_ms": round(float(np.abs(dh - dc).max()) * 1e3, 3),
            "sources": {x: s.count(x) for x in set(s)},
            "nominal_hz": nominal_hz,
        })
        off = h - c
        delay = (off - off.min()) * 1e3                  # transport delay above its lower envelope
        out["delay_p50_ms"] = round(float(np.percentile(delay, 50)), 3)
        out["delay_p95_ms"] = round(float(np.percentile(delay, 95)), 3)
        out["delay_max_ms"] = round(float(delay.max()), 3)
        hist, edges = np.histogram(np.clip(dh * 1e3, 0, 10), bins=40, range=(0, 10))
        out["host_period_hist"] = {"counts": hist.tolist(), "edges_ms": edges.round(3).tolist()}
        return out

    # -- raw capture -------------------------------------------------------------------------------------------
    def start_capture(self, seconds, name=""):
        if self.capture is not None:
            return {"ok": False, "error": "a capture is already running"}
        seconds = max(0.5, min(600.0, float(seconds)))
        slug = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in (name or "rtde"))[:40]
        fn = f"{time.strftime('%Y%m%d_%H%M%S')}_{slug}.csv"
        self.capture_meta = {"file": fn, "seconds": seconds, "until": time.monotonic() + seconds, "started": time.time()}
        self.capture = []
        return {"ok": True, "file": fn, "seconds": seconds}

    def _finish_capture(self):
        with self.lock:
            rows, meta = self.capture, self.capture_meta
            self.capture = None
        if rows is None:
            return
        threading.Thread(target=self._write, args=(rows, meta), daemon=True, name="rtde-capture").start()

    def stop_capture(self):
        if self.capture is None:
            return {"ok": False, "error": "no capture running"}
        self._finish_capture()
        return {"ok": True}

    def _write(self, rows, meta):
        os.makedirs(CAPTURES, exist_ok=True)
        path = os.path.join(CAPTURES, meta["file"])
        cols = []
        if rows:
            for k, v in rows[0].items():
                if k.startswith("_") and k not in ("_mono", "_source"):
                    continue
                nv = _numeric(v)
                if k == "_source":
                    cols.append((k, None))
                elif nv is not None:
                    cols.extend([(k, i) for i in range(len(nv))] if len(nv) > 1 or isinstance(v, (list, tuple, np.ndarray)) else [(k, None)])
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([("host_t" if k == "_mono" else k) + (f"_{i}" if i is not None else "") for k, i in cols])
            for st in rows:
                line = []
                for k, i in cols:
                    v = st.get(k)
                    try:
                        line.append(v if k == "_source" else (v[i] if i is not None else v))
                    except (TypeError, IndexError):
                        line.append("")
                w.writerow(line)
        self.captures_done.appendleft({"file": meta["file"], "rows": len(rows), "seconds": meta["seconds"],
                                       "columns": len(cols), "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())})

    def capture_status(self):
        cm = self.capture_meta if self.capture is not None else None
        return {"running": cm is not None, "file": cm and cm["file"],
                "remaining_s": round(max(0.0, cm["until"] - time.monotonic()), 1) if cm else 0,
                "rows": len(self.capture) if self.capture is not None else 0, "done": list(self.captures_done)}


def list_captures():
    if not os.path.isdir(CAPTURES):
        return []
    out = []
    for f in sorted(os.listdir(CAPTURES), reverse=True)[:50]:
        p = os.path.join(CAPTURES, f)
        if f.endswith(".csv"):
            out.append({"file": f, "bytes": os.path.getsize(p)})
    return out

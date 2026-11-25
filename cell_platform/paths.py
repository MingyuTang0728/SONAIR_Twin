"""Put the cell platform modules on sys.path.

The platform modules import each other by top-level name (ur_telemetry, ur_kin, imu_align, sonair_benchmark,
sim_mujoco, ...), exactly as they did in the SONAIRCrossite33 folder. They are kept unchanged, so instead of
editing their imports we add their folders to sys.path in this order: ur, calib, benchmark.

The files in ur/ are the debugged robot link. They are locked: LOCKED.sha256 holds their hashes and
verify_locked() (also run by tests/test_platform_lock.py) reports any byte that changed.
"""
import hashlib
import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
UR_DIR = os.path.join(ROOT, "ur")
DIRS = [UR_DIR, os.path.join(ROOT, "calib"), os.path.join(ROOT, "benchmark")]


def add():
    for d in reversed(DIRS):
        if d not in sys.path:
            sys.path.insert(0, d)
    return DIRS


def verify_locked():
    """Return a list of problems; empty means every locked file matches its recorded hash."""
    bad = []
    with open(os.path.join(UR_DIR, "LOCKED.sha256")) as f:
        for line in f:
            if not line.strip():
                continue
            want, name = line.split()
            p = os.path.join(UR_DIR, name)
            if not os.path.exists(p):
                bad.append(f"{name}: missing")
                continue
            got = hashlib.sha256(open(p, "rb").read()).hexdigest()
            if got != want:
                bad.append(f"{name}: changed (sha256 {got[:12]} != {want[:12]})")
    return bad

"""
site -- which cell this is, and what robot it has.

The benchmark is one task recorded at more than one site. Everything that
differs between sites is in two small files, so the platform itself is the
same code everywhere:

    site.json                  this cell: site, robot unit, cell id, camera
    embodiments/<robot>.json   the robot model: joints, which joint the effort
                               track scores, its sensors and their mountings

A second site copies the platform and edits site.json; a new robot model adds
an embodiment card. Code reads the card instead of assuming joint 3 of six.
"""
from __future__ import annotations

import json
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
DEFAULT_SITE = {"site": "UoN", "unit": "", "robot": "ur5e", "cell": "uon-cell-1",
                "camera": ""}
_FALLBACK_CARDS = {
    "ur5e": {"robot": "ur5e", "dof": 6, "effort_joint": 2, "kinematics": "ur5e_dh",
             "joints": ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
             "rate_hz": 125.0},
}


def load_site(root=None) -> dict:
    """This cell's identity. A missing or broken site.json means the home site."""
    p = Path(root or HERE) / "site.json"
    try:
        d = json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        d = {}
    out = {**DEFAULT_SITE, **{k: v for k, v in d.items() if k in DEFAULT_SITE}}
    out["site"] = clean_id(out["site"]) or "UoN"
    return out


def clean_id(s) -> str:
    """Site ids go into file and run names: letters, digits, '-' only."""
    return "".join(c for c in str(s or "") if c.isalnum() or c == "-")[:24]


def embodiment(robot: str = "ur5e", root=None) -> dict:
    p = Path(root or HERE) / "embodiments" / f"{robot}.json"
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        if robot in _FALLBACK_CARDS:
            return dict(_FALLBACK_CARDS[robot])
        raise ValueError(f"no embodiment card for robot {robot!r} (embodiments/{robot}.json)")


def effort_joint(robot: str = "ur5e") -> int:
    return int(embodiment(robot).get("effort_joint", 2))

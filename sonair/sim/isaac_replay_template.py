"""TEMPLATE (not run in this environment): replay a SONAIR command or a recorded real target in Isaac Sim and write
a run in the SONAIR format, so it is scored by exactly the same code as MuJoCo and the real robot.

Run inside Isaac Sim's Python (5.x), e.g.   ./python.sh sonair/sim/isaac_replay_template.py --cmd exc_camtool_v0.6_s1
Contract (the only thing that matters for the benchmark):
  input   commands/<id>/ via sonair.commands.load()  or  runs/<real_run>/ via sonair.runlog.target_from_run()
  output  runs/<id>__isaac_<variant>/meta.json + joints.parquet with t, tq*, tqd*, q*, qd* (+ tau*), 500 Hz, source="isaac"
  check   python -m sonair validate runs/<id>__isaac_<variant>
Settings worth logging in meta (they drive the gap you saw above ~0.6 rad/s): physics_dt, solver position/velocity
iterations, drive stiffness/damping, max joint velocity, payload mass/CoG, articulation enable_self_collisions.
API names below follow Isaac Sim 4.5/5.x (isaacsim.core.*); adjust to your installed version.
"""
import argparse, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

ap = argparse.ArgumentParser(); ap.add_argument("--cmd", required=True); ap.add_argument("--target-run")
ap.add_argument("--payload", default=None); ap.add_argument("--variant", default="drive_default"); ap.add_argument("--physics-hz", type=float, default=2000)
a = ap.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.storage.native import get_assets_root_path
from sonair import commands as cm, runlog, payload as pl, cell

c = cm.load(a.cmd)
target = runlog.target_from_run(a.target_run)[0] if a.target_run else c["df"].rename(columns={f"q{i}": f"tq{i}" for i in range(6)} | {f"qd{i}": f"tqd{i}" for i in range(6)})
t = target["t"].to_numpy(); TQ = target[[f"tq{i}" for i in range(6)]].to_numpy(); TQD = target[[f"tqd{i}" for i in range(6)]].to_numpy()

world = World(physics_dt=1.0 / a.physics_hz, rendering_dt=1.0 / 60, stage_units_in_meters=1.0)
usd = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"   # or your own SONAIR twin USD
add_reference_to_stage(usd, "/World/ur5e")
robot = world.scene.add(SingleArticulation("/World/ur5e", name="ur5e"))
world.reset()

# TODO (M3): lumped end-effector payload. Two options, pick one and log it in meta:
#   (a) a rigid body "ee_payload" attached to wrist_3_link with a fixed joint at the tool0 pose,
#       mass = P["mass_kg"], centre of mass = P["cog_m"] (tool0 frame), diagonal inertia from P["inertia_kgm2"];
#   (b) wrist_3_link mass/CoG/inertia replaced by the combined link + payload values (convert tool0 -> wrist_3 frame).
P = pl.payloads()[a.payload or c["meta"].get("payload", "gripper_open")]

robot.set_joint_positions(TQ[0]); robot.set_joint_velocities(np.zeros(6))
ctrl = robot.get_articulation_controller()
steps = int(round(a.physics_hz * (t[1] - t[0])))
rows = []
for k in range(len(t)):
    q = robot.get_joint_positions()[:6]; qd = robot.get_joint_velocities()[:6]
    rows.append([t[k], *TQ[k], *TQD[k], *q, *qd])
    ctrl.apply_action(ArticulationAction(joint_positions=TQ[k], joint_velocities=TQD[k]))
    for _ in range(steps): world.step(render=False)

import pandas as pd
cols = ["t"] + [f"tq{i}" for i in range(6)] + [f"tqd{i}" for i in range(6)] + [f"q{i}" for i in range(6)] + [f"qd{i}" for i in range(6)]
df = pd.DataFrame(rows, columns=cols)
rid = f"{a.cmd}__isaac_{a.variant}" + ("__realtarget" if a.target_run else "")
runlog.write(os.path.join(cell.ROOT, "runs", rid), dict(run_id=rid, source="isaac", variant=a.variant, command_id=a.cmd,
             command_sha256=c["meta"].get("sha256"), payload=a.payload or c["meta"].get("payload"), physics_hz=a.physics_hz), df)
print("wrote", rid, runlog.validate(os.path.join(cell.ROOT, "runs", rid)) or "valid")
app.close()

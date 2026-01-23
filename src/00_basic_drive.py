import os
import numpy as np

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.prims import is_prim_path_valid

# -----------------------
# Config
# -----------------------
WORLD_USD = "/home/saman-aboutorab/projects/Robotics/isaac_diff_drive_nav/isaac/worlds/simple_obstacles_carter_lidar.usd"
ROBOT_PRIM_PATH = "/World/nova_carter"

# Carter wheel joints in your USD/URDF
LEFT_WHEEL_DOF  = "joint_wheel_left"
RIGHT_WHEEL_DOF = "joint_wheel_right"

# Simple demo drive (edit or set to 0 to stop)
LEFT_WHEEL_VEL  = 2.0   # rad/s
RIGHT_WHEEL_VEL = 2.0   # rad/s

WARMUP_STEPS = 60


def log(tag, msg):
    print(f"[{tag}] {msg}")


# -----------------------
# Load world
# -----------------------
if not os.path.exists(WORLD_USD):
    raise FileNotFoundError(WORLD_USD)

log("WORLD", f"Opening: {WORLD_USD}")
open_stage(WORLD_USD)
simulation_app.update()

world = World(stage_units_in_meters=1.0)
world.reset()

# -----------------------
# Load robot
# -----------------------
if not is_prim_path_valid(ROBOT_PRIM_PATH):
    raise RuntimeError(f"Robot not found at {ROBOT_PRIM_PATH}")

carter = Articulation(prim_path=ROBOT_PRIM_PATH, name="nova_carter")
world.scene.add(carter)

world.reset()
carter.initialize()

# Resolve DOF indices
dof_names = list(carter.dof_names) if hasattr(carter, "dof_names") else list(carter.get_dof_names())
if LEFT_WHEEL_DOF not in dof_names or RIGHT_WHEEL_DOF not in dof_names:
    raise RuntimeError(f"Wheel DOFs not found. Available DOFs: {dof_names}")

left_i = dof_names.index(LEFT_WHEEL_DOF)
right_i = dof_names.index(RIGHT_WHEEL_DOF)

vel = np.zeros(len(dof_names), dtype=np.float32)

log("ROBOT", f"Loaded Carter at {ROBOT_PRIM_PATH}")
log("ROBOT", f"DOFs: {dof_names}")
log("ROBOT", f"Wheel indices: left={left_i} right={right_i}")

# -----------------------
# Start sim
# -----------------------
timeline = omni.timeline.get_timeline_interface()
timeline.play()
simulation_app.update()

# Warm up (let physics settle)
for _ in range(WARMUP_STEPS):
    world.step(render=True)

log("SIM", "Running baseline (NO LIDAR). Close the window to stop.")

try:
    while simulation_app.is_running():
        # Apply constant wheel velocity (demo drive)
        vel[:] = 0.0
        vel[left_i] = LEFT_WHEEL_VEL
        vel[right_i] = RIGHT_WHEEL_VEL
        carter.apply_action(ArticulationAction(joint_velocities=vel))

        world.step(render=True)

finally:
    try:
        timeline.stop()
    except Exception:
        pass

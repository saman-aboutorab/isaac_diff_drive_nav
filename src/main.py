import os
import numpy as np

from omni.isaac.kit import SimulationApp

# --- 1) Start Isaac Sim UI ---
simulation_app = SimulationApp({"headless": False})

# import Isaac/Omni modules
from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import RigidPrim
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.prims import is_prim_path_valid

# --- Helper functions
class Log:
    BLUE = "\033[94m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RED = "\033[91m"
    BOLD = "\033[1m"
    END = "\033[0m"

def log_info(msg):
    print(f"{Log.GREEN}[INFO] {msg}{Log.END}")

def log_warn(msg):
    print(f"{Log.YELLOW}[WARN] {msg}{Log.END}")

def log_err(msg):
    print(f"{Log.RED}[ERR] {msg}{Log.END}")

def log_step(msg):
    print(f"{Log.BLUE}{msg}{Log.END}")



# --- 2) Point to saved world USD ---
# Change this if your world file has a different name/location:
WORLD_USD = "/home/saman-aboutorab/projects/Robotics/isaac_diff_drive_nav/isaac/worlds/simple_obstacles.usd"

if not os.path.exists(WORLD_USD):
    raise FileNotFoundError(f"World USD not found: {WORLD_USD}")

log_step(f"Loading world: {WORLD_USD}")
open_stage(WORLD_USD)

# --- 3) Spawn Nova Carter ---
assets_root = get_assets_root_path()
if assets_root is None:
    # Fallback if nucleus path isn't configured; you can replace with your real asset root later
    assets_root = "omniverse://localhost/NVIDIA/Assets/Isaac/5.0"
    log_warn(f"[WARN] get_assets_root_path() returned None. Using fallback: {assets_root}")
else:
    log_info(f"Assets root: {assets_root}")

CARTER_USD = "/home/saman-aboutorab/projects/Robotics/isaac_diff_drive_nav/isaac/robots/NovaCarter/nova_carter.usd"

simulation_app.update()
ROBOT_PRIM_PATH = "/World/NovaCarter"

log_info(f"Spawning Carter from: {CARTER_USD}")
create_prim(ROBOT_PRIM_PATH, "Xform")  # make sure prim path exists
add_reference_to_stage(usd_path=CARTER_USD, prim_path=ROBOT_PRIM_PATH)

# --- 4) Create a World and step simulation so we can see it ---
world = World(stage_units_in_meters=1.0)
world.reset()

# --- 5) Nova Carter Control Joints
# Wrap Nova Carter as an articulation
ROBOT_PATH = "/World/NovaCarter"

log_info(f"Checking prim exists: {ROBOT_PATH}")
log_info(f"prim valid? {is_prim_path_valid(ROBOT_PATH)}")

if not is_prim_path_valid(ROBOT_PATH):
    raise RuntimeError(f"Robot prim not found at {ROBOT_PATH}. Check the path in Stage.")

# Create articulation interface
carter = Articulation(prim_path=ROBOT_PATH, name="nova_carter")

world.scene.add(carter)

log_step("Carter articulation initialized")

# --- 6) Get joint indices
world.reset()
carter.initialize()

# DOF names
dof_names = None
if hasattr(carter, "dof_names"):
    dof_names = list(carter.dof_names)
elif hasattr(carter, "get_dof_names"):
    dof_names = list(carter.get_dof_names())
elif hasattr(carter, "_articulation_view") and hasattr(carter._articulation_view, "dof_names"):
    dof_names = list(carter._articulation_view.dof_names)

if dof_names is None:
    raise RuntimeError("Could not access DOF names from this Articulation object.")

log_info("DOF names:")
for i, n in enumerate(dof_names):
    log_info(f"  {i}: {n}")

# find wheel indices by name
left_name = "joint_wheel_left"
right_name = "joint_wheel_right"

left_i = dof_names.index(left_name)
right_i = dof_names.index(right_name)

log_info(f"wheel DOF indices:, {left_i}, {right_i}")

# apply velocity targets using a full array
num_dof = len(dof_names)
vel = np.zeros(num_dof, dtype=np.float32)

FORWARD_RAD_S = 5.0

log_step(f"Driving forward using joints, {dof_names[left_i]}, {dof_names[right_i]}")

for _ in range(300):  # ~5 seconds
    vel[:] = 0.0
    vel[left_i] = FORWARD_RAD_S
    vel[right_i] = FORWARD_RAD_S

    carter.apply_action(ArticulationAction(joint_velocities=vel))
    world.step(render=True)

log_info("Running sim for a few seconds. Close the window to exit.")
# for _ in range(3000):  # ~10 seconds if dt ~ 1/60
#     world.step(render=True)

# simulation_app.close()
# print("Done.")

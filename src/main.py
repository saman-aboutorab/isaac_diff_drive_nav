import os
import numpy as np
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})
import omni
import omni.replicator.core as rep
from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.sensor import _sensor
from omni.isaac.range_sensor import RangeSensorCreateLidar
from omni.isaac.core.utils.stage import get_current_stage
from pxr import Gf, UsdGeom
from omni.isaac.range_sensor import _range_sensor
from omni.isaac.core.utils.prims import create_prim, delete_prim
from isaacsim.sensors.rtx import LidarRtx

# -------------------
CANDIDATE_MODULES = [
    "omni.sensors.nv.lidar",
    "omni.sensors.nv.lidar._lidar",
    "isaacsim.sensors.rtx",
    "isaacsim.sensors.rtx._lidar",
    "omni.isaac.range_sensor",
    "omni.isaac.range_sensor._range_sensor",
]

def read_ranges_any(rs_iface, path: str):
    # common method names across builds
    for m in ["get_linear_depth_data", "get_depth_data", "get_ranges", "get_lidar_data"]:
        if hasattr(rs_iface, m):
            try:
                return m, getattr(rs_iface, m)(path)
            except Exception:
                pass
    return None, None
# -------------------
# Inputs
# -------------------
v = 0.2
w = 0.0
WHEEL_RADIUS = 0.05
WHEEL_BASE = 0.30

WORLD_USD = "/home/saman-aboutorab/projects/Robotics/isaac_diff_drive_nav/isaac/worlds/simple_obstacles_carter_lidar.usd"
ROBOT_PRIM_PATH = "/World/nova_carter"

ROBOT_PATH = "/World/nova_carter"
LIDAR_PATH = f"{ROBOT_PATH}/lidar_2d"

LIDAR_MOUNT_PATH = f"{ROBOT_PATH}/lidar_mount"
LIDAR_SENSOR_PATH = f"{LIDAR_MOUNT_PATH}/lidar_2d"

def v_w_to_wheels(v, w, r, L):
    wl = (v - w * L / 2.0) / r
    wr = (v + w * L / 2.0) / r
    return wl, wr

# -------------------
# Load world
# -------------------
if not os.path.exists(WORLD_USD):
    raise FileNotFoundError(WORLD_USD)

open_stage(WORLD_USD)
simulation_app.update()

world = World(stage_units_in_meters=1.0)
world.reset()

# -------------------
# Wrap robot
# -------------------
if not is_prim_path_valid(ROBOT_PRIM_PATH):
    raise RuntimeError(f"Robot not found at {ROBOT_PRIM_PATH}")

carter = Articulation(prim_path=ROBOT_PRIM_PATH, name="nova_carter")
world.scene.add(carter)

world.reset()
carter.initialize()

# DOF names -> wheel indices
dof_names = list(carter.dof_names) if hasattr(carter, "dof_names") else list(carter.get_dof_names())
left_i = dof_names.index("joint_wheel_left")
right_i = dof_names.index("joint_wheel_right")

vel = np.zeros(len(dof_names), dtype=np.float32)

# -------------------
# LiDAR
# -------------------

# Create a mounting Xform under the robot
stage = get_current_stage()

if not stage.GetPrimAtPath(LIDAR_MOUNT_PATH).IsValid():
    create_prim(LIDAR_MOUNT_PATH, "Xform")
    xform = UsdGeom.Xformable(stage.GetPrimAtPath(LIDAR_MOUNT_PATH))
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(0.25, 0.0, 0.25))  # front + up
simulation_app.update()

# RTX LiDAR uses an OmniLidar prim and Replicator-based annotators
# Create sensor
lidar = LidarRtx(
    prim_path=LIDAR_SENSOR_PATH,
    translation=np.array([0.0, 0.0, 0.0]),     # relative to mount
    orientation=np.array([1.0, 0.0, 0.0, 0.0]), # (w,x,y,z)
    config_file_name="Example_Rotary",
)
lidar.initialize()

# IMPORTANT: play timeline before attaching/reading replicator annotators
timeline = omni.timeline.get_timeline_interface()
timeline.play()
simulation_app.update()

# Attach an annotator. This one gives per-frame point cloud (fast to start with).
lidar.attach_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator")

# Step a bit so the render product + annotator graph initializes
for _ in range(30):
    world.step(render=True)

print("\033[95m[LIDAR]\033[0m RTX lidar created at:", LIDAR_SENSOR_PATH)

# -------------------
# Run loop
# -------------------
try:
    for step_i in range(11000):
        frame = lidar.get_current_frame()

        if step_i == 0:
            print("\n[LIDAR] frame keys:", list(frame.keys()))

        blob = frame.get("IsaacExtractRTXSensorPointCloudNoAccumulator", {})
        pts = blob.get("data", None)

        if pts is None or len(pts) == 0:
            print("\033[91m[LIDAR]\033[0m no RTX point cloud yet      ", end="\r")
        else:
            pts = np.asarray(pts, dtype=np.float32)
            d = np.linalg.norm(pts[:, :3], axis=1)
            d = d[np.isfinite(d)]
            if len(d) > 0:
                print(f"\033[95m[LIDAR]\033[0m min={float(d.min()):.2f} m      ", end="\r")

        wl, wr = v_w_to_wheels(v, w, WHEEL_RADIUS, WHEEL_BASE)
        vel[:] = 0.0
        vel[left_i] = wl
        vel[right_i] = wr
        carter.apply_action(ArticulationAction(joint_velocities=vel))
        world.step(render=True)

finally:
    # Clean shutdown prevents syntheticdata/graph crashes
    try:
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
    except Exception:
        pass
    simulation_app.close()


import os
import numpy as np
import math
from omni.isaac.kit import SimulationApp

# ------------------------------------------------------------
# App + core imports
# ------------------------------------------------------------
simulation_app = SimulationApp({"headless": False})

import omni
import omni.replicator.core as rep

from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage, get_current_stage
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.prims import is_prim_path_valid, create_prim
from pxr import Gf, UsdGeom

from isaacsim.sensors.rtx import LidarRtx

# ------------------------------------------------------------
# Pretty, colored logs
# ------------------------------------------------------------
class Log:
    RESET = "\033[0m"
    RED   = "\033[91m"
    GREEN = "\033[92m"
    YELL  = "\033[93m"
    BLUE  = "\033[94m"
    MAG   = "\033[95m"
    CYAN  = "\033[96m"
    GRAY  = "\033[90m"

def log(color, tag, msg, end="\n"):
    print(f"{color}[{tag}]{Log.RESET} {msg}", end=end)

def once_every(step_i: int, every: int) -> bool:
    return (every > 0) and (step_i % every == 0)

# ------------------------------------------------------------
# Helpers
# ------------------------------------------------------------
def v_w_to_wheels(v, w, r, L):
    wl = (v - w * L / 2.0) / r
    wr = (v + w * L / 2.0) / r
    return wl, wr

def fmt(x):
    return "inf" if not np.isfinite(x) else f"{x:.2f}"

def to_xyz_points(pts) -> np.ndarray | None:
    """Convert various RTX point formats to (N,3) float32 array."""
    if pts is None:
        return None
    arr = np.asarray(pts, dtype=np.float32)

    if arr.ndim == 2:
        if arr.shape[1] >= 3:
            return arr[:, :3]
        return None

    if arr.ndim == 1:
        n = arr.size
        if n % 3 == 0:
            return arr.reshape((-1, 3))
        if n % 4 == 0:
            return arr.reshape((-1, 4))[:, :3]
        return None

    return None

# ------------------------------------------------------------
# Step 1: sector mins from point cloud
# ------------------------------------------------------------
DEG = math.pi / 180.0
SECTORS = {
    "L": (10 * DEG, 90 * DEG),
    "C": (-10 * DEG, 10 * DEG),
    "R": (-90 * DEG, -10 * DEG),
}
MIN_VALID_RANGE = 0.15
MAX_VALID_RANGE = 8.0

def sector_mins_from_pointcloud(pts_xyz: np.ndarray):
    mins = {"L": float("inf"), "C": float("inf"), "R": float("inf")}

    if pts_xyz is None or len(pts_xyz) == 0:
        return mins

    x = pts_xyz[:, 0]
    y = pts_xyz[:, 1]

    d = np.sqrt(x * x + y * y)

    # Front only + ignore self hits + ignore far noise
    valid = (x > 0.05) & np.isfinite(d) & (d > MIN_VALID_RANGE) & (d < MAX_VALID_RANGE)
    if not np.any(valid):
        return mins

    x = x[valid]
    y = y[valid]
    d = d[valid]
    ang = np.arctan2(y, x)

    for k, (a0, a1) in SECTORS.items():
        # (kept robust for wrap-around sectors; your current sectors do not wrap)
        m = (ang >= a0) & (ang <= a1) if a0 <= a1 else ((ang >= a0) | (ang <= a1))
        if np.any(m):
            mins[k] = float(d[m].min())

    return mins

# ------------------------------------------------------------
# Inputs / paths
# ------------------------------------------------------------
v = 0.2
w = 0.0
WHEEL_RADIUS = 0.05
WHEEL_BASE = 0.30

READ_EVERY = 10              # read LiDAR every N sim steps (stability + realism)
LOG_EVERY_READ = True        # keep light logs at read time
LOG_GEOM_EVERY = READ_EVERY * 10
LOG_ANGLE_EVERY = READ_EVERY * 20

WORLD_USD = "/home/saman-aboutorab/projects/Robotics/isaac_diff_drive_nav/isaac/worlds/simple_obstacles_carter_lidar.usd"
ROBOT_PRIM_PATH = "/World/nova_carter"
ROBOT_PATH = "/World/nova_carter"
LIDAR_MOUNT_PATH = f"{ROBOT_PATH}/lidar_mount"
LIDAR_SENSOR_PATH = f"{LIDAR_MOUNT_PATH}/lidar_2d"

# ------------------------------------------------------------
# Load world
# ------------------------------------------------------------
if not os.path.exists(WORLD_USD):
    raise FileNotFoundError(WORLD_USD)

open_stage(WORLD_USD)
simulation_app.update()

world = World(stage_units_in_meters=1.0)
world.reset()

# ------------------------------------------------------------
# Robot
# ------------------------------------------------------------
if not is_prim_path_valid(ROBOT_PRIM_PATH):
    raise RuntimeError(f"Robot not found at {ROBOT_PRIM_PATH}")

carter = Articulation(prim_path=ROBOT_PRIM_PATH, name="nova_carter")
world.scene.add(carter)

world.reset()
carter.initialize()

dof_names = list(carter.dof_names) if hasattr(carter, "dof_names") else list(carter.get_dof_names())
left_i = dof_names.index("joint_wheel_left")
right_i = dof_names.index("joint_wheel_right")
vel = np.zeros(len(dof_names), dtype=np.float32)

# ------------------------------------------------------------
# RTX LiDAR setup
# ------------------------------------------------------------
stage = get_current_stage()

# Ensure mount exists
if not stage.GetPrimAtPath(LIDAR_MOUNT_PATH).IsValid():
    create_prim(LIDAR_MOUNT_PATH, "Xform")
    xform = UsdGeom.Xformable(stage.GetPrimAtPath(LIDAR_MOUNT_PATH))
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(0.25, 0.0, 0.25))  # front + up
    log(Log.MAG, "LIDAR", f"Created mount at {LIDAR_MOUNT_PATH}")
else:
    log(Log.GRAY, "LIDAR", f"Mount already exists: {LIDAR_MOUNT_PATH}")

simulation_app.update()

# Create LiDAR
lidar = LidarRtx(
    prim_path=LIDAR_SENSOR_PATH,
    translation=np.array([0.0, 0.0, 0.0]),
    orientation=np.array([1.0, 0.0, 0.0, 0.0]),  # (w,x,y,z)
    config_file_name="Example_Rotary",
    **{"omni:sensor:Core:scanRateBaseHz": 5},
)
lidar.initialize()

# Timeline must be playing for RTX/replicator annotators
timeline = omni.timeline.get_timeline_interface()
timeline.play()
simulation_app.update()

lidar.attach_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator")

# Warm up render/sensor graph
for _ in range(120):
    world.step(render=True)

log(Log.MAG, "LIDAR", f"RTX lidar ready at {LIDAR_SENSOR_PATH}")

# ------------------------------------------------------------
# Main loop
# ------------------------------------------------------------
last_xyz = None   # keep last valid point cloud between reads (so motion continues smoothly)

try:
    step_i = 0
    while simulation_app.is_running():
        # ----------------------------
        # LiDAR read (throttled)
        # ----------------------------
        if once_every(step_i, READ_EVERY):
            frame = lidar.get_current_frame()

            if step_i == 0:
                # One-time: frame keys
                if isinstance(frame, dict):
                    log(Log.CYAN, "LIDAR", f"frame keys: {list(frame.keys())}")
                else:
                    log(Log.RED, "LIDAR", f"frame is not dict (type={type(frame)})")

            pts = None
            if isinstance(frame, dict):
                blob = frame.get("IsaacExtractRTXSensorPointCloudNoAccumulator", {})
                if isinstance(blob, dict):
                    pts = blob.get("data", None)

            if LOG_EVERY_READ and pts is not None and once_every(step_i, READ_EVERY):
                arr = np.asarray(pts)
                # log format only occasionally so terminal doesn't spam
                if step_i == 0 or once_every(step_i, READ_EVERY * 30):
                    log(Log.CYAN, "LIDAR", f"raw pts ndim={arr.ndim}, shape={getattr(arr,'shape',None)}, size={arr.size}")

            xyz = to_xyz_points(pts)
            if xyz is None or len(xyz) == 0:
                # keep last_xyz; don't overwrite good data with empty frames
                if step_i == 0:
                    log(Log.YELL, "LIDAR", "No point cloud yet (warming up...)")
            else:
                last_xyz = xyz

        # ----------------------------
        # Step 1: sector mins (use last good data)
        # ----------------------------
        if last_xyz is None or len(last_xyz) == 0:
            log(Log.YELL, "LIDAR", "No RTX points yet", end="\r")
        else:
            xyz = last_xyz

            # Occasional geometry sanity logs
            if once_every(step_i, LOG_GEOM_EVERY):
                x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
                log(
                    Log.BLUE,
                    "LIDAR",
                    f"x:[{x.min():.2f},{x.max():.2f}]  y:[{y.min():.2f},{y.max():.2f}]  z:[{z.min():.2f},{z.max():.2f}]"
                )

            # Occasional angle sanity logs
            if once_every(step_i, LOG_ANGLE_EVERY):
                ang_deg = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0]))
                log(Log.GRAY, "LIDAR", f"angle deg: min={ang_deg.min():.1f}, max={ang_deg.max():.1f}")

            mins = sector_mins_from_pointcloud(xyz)
            log(
                Log.GREEN,
                "SECTOR",
                f"L={fmt(mins['L'])}  C={fmt(mins['C'])}  R={fmt(mins['R'])}",
                end="\r"
            )

        # ----------------------------
        # Drive robot (constant v,w for now)
        # ----------------------------
        wl, wr = v_w_to_wheels(v, w, WHEEL_RADIUS, WHEEL_BASE)
        vel[:] = 0.0
        vel[left_i] = wl
        vel[right_i] = wr
        carter.apply_action(ArticulationAction(joint_velocities=vel))
        world.step(render=True)

        step_i += 1

finally:
    # Clean shutdown helps avoid syntheticdata/graph crashes
    try:
        omni.timeline.get_timeline_interface().stop()
    except Exception:
        pass
    # If you want it to close automatically, uncomment:
    # simulation_app.close()

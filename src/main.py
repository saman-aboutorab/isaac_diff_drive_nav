import os
import numpy as np
import math
from omni.isaac.kit import SimulationApp

# ------------------------------------------------------------
# App + core imports
# ------------------------------------------------------------
simulation_app = SimulationApp({"headless": False})

from scipy.spatial.transform import Rotation as R

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
DEBUG_SECTORS = True

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
# IMPROVED: Wider sectors + better filtering
# ------------------------------------------------------------
DEG = math.pi / 180.0

# Wider overlapping sectors for better coverage
SECTORS = {
    "L": (25 * DEG, 155 * DEG),    # Wide left sector
    "C": (-35 * DEG, 35 * DEG),    # Wider center (70°)
    "R": (-155 * DEG, -25 * DEG),  # Wide right sector
}

MIN_VALID_RANGE = 0.15
MAX_VALID_RANGE = 8.0

def transform_points_to_robot_frame(pts_xyz, robot_orientation_quat):
    """
    Transform LiDAR points from sensor frame to robot base frame.
    
    Args:
        pts_xyz: (N, 3) array of points in LiDAR frame
        robot_orientation_quat: (4,) array [w, x, y, z] robot orientation
        
    Returns:
        (N, 3) array of points in world/robot frame
    """
    if pts_xyz is None or len(pts_xyz) == 0:
        return pts_xyz
    
    # Get rotation from robot orientation quaternion
    # Assuming LiDAR is mounted aligned with robot frame
    # If LiDAR has its own orientation, you'd need to compose rotations
    rot = R.from_quat([
        robot_orientation_quat[1],  # x
        robot_orientation_quat[2],  # y
        robot_orientation_quat[3],  # z
        robot_orientation_quat[0]   # w (scipy uses x,y,z,w order)
    ])
    
    # Apply rotation to all points
    pts_rotated = rot.apply(pts_xyz)
    
    return pts_rotated


def sector_mins_from_pointcloud_with_rotation(pts_xyz, robot_orientation, forward_axis="y"):
    """
    Compute sector minimums accounting for robot rotation.
    Sectors are now defined in WORLD frame, not robot frame.
    """
    mins = {"L": float("inf"), "C": float("inf"), "R": float("inf")}

    if pts_xyz is None or len(pts_xyz) == 0:
        return mins

    # Transform points to account for robot rotation
    # This keeps sectors aligned with world directions
    pts_world = transform_points_to_robot_frame(pts_xyz, robot_orientation)
    
    # Now use world-frame coordinates
    if forward_axis == "x":
        f = pts_world[:, 0]
        l = pts_world[:, 1]
    else:
        f = pts_world[:, 1]
        l = pts_world[:, 0]
    
    z = pts_world[:, 2]
    
    # Rest of your filtering logic...
    d = np.sqrt(f*f + l*l)
    valid = (
        (f > -0.5) &
        np.isfinite(d) &
        (d > MIN_VALID_RANGE) &
        (d < MAX_VALID_RANGE) &
        (z > -0.40) & (z < 1.0)
    )
    
    if not np.any(valid):
        return mins
    
    f = f[valid]
    l = l[valid]
    d = d[valid]
    
    ang = np.arctan2(l, f)
    
    for k, (a0, a1) in SECTORS.items():
        m = (ang >= a0) & (ang <= a1) if a0 <= a1 else ((ang >= a0) | (ang <= a1))
        if np.any(m):
            mins[k] = float(d[m].min())
    
    return mins

def sector_mins_from_pointcloud(pts_xyz: np.ndarray, forward_axis="y"):
    """
    forward_axis: "x" or "y"
    Returns minimum distance in each sector.
    """
    mins = {"L": float("inf"), "C": float("inf"), "R": float("inf")}

    if pts_xyz is None or len(pts_xyz) == 0:
        return mins

    # --- choose axes ---
    if forward_axis == "x":
        f = pts_xyz[:, 0]  # forward
        l = pts_xyz[:, 1]  # left
    else:
        f = pts_xyz[:, 1]  # forward (your case)
        l = -pts_xyz[:, 0]  # left

    z = pts_xyz[:, 2]

    # --- 2D distance in the horizontal plane ---
    d = np.sqrt(f*f + l*l)

    # --- IMPROVED FILTER: More lenient ---
    valid = (
        (f > -0.5) &  # Allow some backward points (robot might be angled)
        np.isfinite(d) &
        (d > MIN_VALID_RANGE) &
        (d < MAX_VALID_RANGE) &
        (z > -0.40) & (z < 1.0)  # More lenient z-range
    )
    
    if not np.any(valid):
        return mins

    if DEBUG_SECTORS:
        kept = int(np.sum(valid))
        total = int(len(d))
        log(Log.GRAY, "FILTER", f"kept={kept}/{total} ({100*kept/max(total,1):.1f}%)")

    f = f[valid]
    l = l[valid]
    d = d[valid]

    # angle relative to forward axis
    ang = np.arctan2(l, f)

    for k, (a0, a1) in SECTORS.items():
        m = (ang >= a0) & (ang <= a1) if a0 <= a1 else ((ang >= a0) | (ang <= a1))
        if np.any(m):
            mins[k] = float(d[m].min())
            if DEBUG_SECTORS:
                cnt = int(np.sum(m))
                log(Log.GRAY, "SECTOR_CNT", f"{k} count={cnt} min={mins[k]:.2f}")

    return mins


def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def compute_cmd_from_sectors(mins, w_prev=0.0):
    """
    IMPROVED: Better obstacle detection logic
    """
    L, C, R = mins["L"], mins["C"], mins["R"]
    L_ok, C_ok, R_ok = np.isfinite(L), np.isfinite(C), np.isfinite(R)

    # Find the closest obstacle in ANY direction
    all_dists = [d for d in [L, C, R] if np.isfinite(d)]
    closest_dist = min(all_dists) if all_dists else float("inf")
    
    # If center is missing, use the closest side distance (more conservative)
    if not C_ok and all_dists:
        Cm = min(all_dists)
    else:
        Cm = C if C_ok else float("inf")

    # --- SPEED CONTROL ---
    if closest_dist <= STOP_DIST:
        v_cmd = 0.0
    elif closest_dist <= SLOW_DIST:
        alpha = (closest_dist - STOP_DIST) / (SLOW_DIST - STOP_DIST)
        v_cmd = V_MIN + alpha * (V_MAX - V_MIN)
    else:
        v_cmd = V_MAX

    # --- TURNING LOGIC ---
    w_cmd = 0.0

    # If we're close to obstacles, turn away
    if closest_dist <= SLOW_DIST:
        if L_ok and R_ok:
            # Turn away from closer side
            diff = R - L
            w_cmd = -TURN_GAIN * diff
        elif R_ok and not L_ok:
            # Only right obstacle -> turn LEFT
            w_cmd = +0.8 * W_MAX
        elif L_ok and not R_ok:
            # Only left obstacle -> turn RIGHT
            w_cmd = -0.8 * W_MAX
        elif C_ok:
            # Only center obstacle -> turn based on previous direction
            w_cmd = -0.6 * W_MAX if w_prev >= 0 else 0.6 * W_MAX
        else:
            # Shouldn't happen, but keep previous turn
            w_cmd = w_prev * 0.5

        # Boost turn when very close
        if closest_dist <= STOP_DIST:
            w_cmd *= 1.3

    # Clamp and smooth
    w_cmd = clamp(w_cmd, -W_MAX, W_MAX)
    SMOOTH = 0.80
    w_cmd = SMOOTH * w_prev + (1.0 - SMOOTH) * w_cmd

    return float(v_cmd), float(w_cmd)

def log_cmd_state(v, w, mins, reason=""):
    color = Log.GREEN
    closest = min([d for d in mins.values() if np.isfinite(d)], default=float("inf"))
    
    if np.isfinite(closest) and closest < STOP_DIST:
        color = Log.RED
    elif np.isfinite(closest) and closest < SLOW_DIST:
        color = Log.YELL

    log(color, "CMD",
        f"{reason} v={v:.2f} w={w:.2f} | L={fmt(mins['L'])} C={fmt(mins['C'])} R={fmt(mins['R'])} | closest={fmt(closest)}"
    )

# ------------------------------------------------------------
# Configuration
# ------------------------------------------------------------
WHEEL_RADIUS = 0.05
WHEEL_BASE = 0.30

READ_EVERY = 5               # Read LiDAR more frequently
LOG_GEOM_EVERY = READ_EVERY * 20
LOG_ANGLE_EVERY = READ_EVERY * 30

WORLD_USD = "/home/saman-aboutorab/projects/Robotics/isaac_diff_drive_nav/isaac/worlds/simple_obstacles_carter_lidar.usd"
ROBOT_PRIM_PATH = "/World/nova_carter"
ROBOT_PATH = "/World/nova_carter"
LIDAR_MOUNT_PATH = f"{ROBOT_PATH}/lidar_mount"
LIDAR_SENSOR_PATH = f"{LIDAR_MOUNT_PATH}/lidar_2d"

# Controller params - TUNED
V_MAX = 0.20          # Slower max speed
V_MIN = 0.03          # Slower creep
W_MAX = 1.0           # Reduced turn speed

STOP_DIST = 0.65      # Stop further away
SLOW_DIST = 1.80      # Start slowing earlier

TURN_GAIN = 0.8       # More gentle turning

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

if not stage.GetPrimAtPath(LIDAR_MOUNT_PATH).IsValid():
    create_prim(LIDAR_MOUNT_PATH, "Xform")
    xform = UsdGeom.Xformable(stage.GetPrimAtPath(LIDAR_MOUNT_PATH))
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(0.25, 0.0, 0.25))
    log(Log.MAG, "LIDAR", f"Created mount at {LIDAR_MOUNT_PATH}")
else:
    log(Log.GRAY, "LIDAR", f"Mount already exists: {LIDAR_MOUNT_PATH}")

simulation_app.update()

lidar = LidarRtx(
    prim_path=LIDAR_SENSOR_PATH,
    translation=np.array([0.0, 0.0, 0.0]),
    orientation=np.array([1.0, 0.0, 0.0, 0.0]),
    config_file_name="Example_Rotary",
    **{"omni:sensor:Core:scanRateBaseHz": 10},  # Higher scan rate
)
lidar.initialize()

timeline = omni.timeline.get_timeline_interface()
timeline.play()
simulation_app.update()

lidar.attach_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator")

# Warm up
for _ in range(120):
    world.step(render=True)

log(Log.MAG, "LIDAR", f"RTX lidar ready at {LIDAR_SENSOR_PATH}")

# ------------------------------------------------------------
# Main loop
# ------------------------------------------------------------
last_xyz = None
v_prev, w_prev = 0.0, 0.0

# Initialize with safe defaults
mins = {"L": float("inf"), "C": float("inf"), "R": float("inf")}

try:
    step_i = 0
    while simulation_app.is_running():
        # ----------------------------
        # LiDAR read (more frequent)
        # ----------------------------
        if once_every(step_i, READ_EVERY):
            frame = lidar.get_current_frame()

            if step_i == 0:
                if isinstance(frame, dict):
                    log(Log.CYAN, "LIDAR", f"frame keys: {list(frame.keys())}")
                else:
                    log(Log.RED, "LIDAR", f"frame is not dict (type={type(frame)})")

            pts = None
            if isinstance(frame, dict):
                blob = frame.get("IsaacExtractRTXSensorPointCloudNoAccumulator", {})
                if isinstance(blob, dict):
                    pts = blob.get("data", None)

            xyz = to_xyz_points(pts)

            if step_i == 0 and xyz is not None and len(xyz) > 0:
                ang_x = np.degrees(np.arctan2(xyz[:,1], xyz[:,0]))
                ang_y = np.degrees(np.arctan2(xyz[:,0], xyz[:,1]))
                log(Log.CYAN, "AXIS", f"assume fwd=x angle range: {ang_x.min():.1f}..{ang_x.max():.1f}")
                log(Log.CYAN, "AXIS", f"assume fwd=y angle range: {ang_y.min():.1f}..{ang_y.max():.1f}")

            if xyz is None or len(xyz) == 0:
                if step_i == 0:
                    log(Log.YELL, "LIDAR", "No point cloud yet (warming up...)")
            else:
                last_xyz = xyz

        # ----------------------------
        # Process LiDAR data
        # ----------------------------



        if last_xyz is not None and len(last_xyz) > 0:
            xyz = last_xyz

            if once_every(step_i, LOG_GEOM_EVERY):
                x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
                log(Log.BLUE, "LIDAR",
                    f"x:[{x.min():.2f},{x.max():.2f}]  y:[{y.min():.2f},{y.max():.2f}]  z:[{z.min():.2f},{z.max():.2f}]"
                )

            if once_every(step_i, LOG_ANGLE_EVERY):
                ang_deg = np.degrees(np.arctan2(xyz[:, 0], xyz[:, 1]))
                log(Log.GRAY, "LIDAR", f"angle deg: min={ang_deg.min():.1f}, max={ang_deg.max():.1f}")

            mins = sector_mins_from_pointcloud(xyz, forward_axis="y")

            # Get robot's current pose
            # robot_position, robot_orientation = carter.get_world_pose()

            # Use rotation-aware function
            # mins = sector_mins_from_pointcloud_with_rotation(
            #     xyz, 
            #     robot_orientation,
            #     forward_axis="y"
            # )   
                     
        else:
            if once_every(step_i, 30):
                log(Log.YELL, "LIDAR", "No RTX points yet - using safe defaults")

        # ----------------------------
        # Compute commands
        # ----------------------------
        v, w = compute_cmd_from_sectors(mins, w_prev=w_prev)
        
        # Find closest obstacle for logging
        closest = min([d for d in mins.values() if np.isfinite(d)], default=float("inf"))
        near = np.isfinite(closest) and closest < SLOW_DIST
        turning = abs(w) > 0.15
        changed = abs(v - v_prev) > 0.02 or abs(w - w_prev) > 0.10

        # Log when relevant
        if near or turning or changed or once_every(step_i, READ_EVERY * 5):
            reason = ""
            if np.isfinite(closest) and closest <= STOP_DIST:
                reason = "STOP! "
            elif np.isfinite(closest) and closest <= SLOW_DIST:
                reason = "SLOW "
            
            log_cmd_state(v, w, mins, reason=reason)

        v_prev, w_prev = v, w

        # ----------------------------
        # Drive robot
        # ----------------------------
        wl, wr = v_w_to_wheels(v, w, WHEEL_RADIUS, WHEEL_BASE)
        vel[:] = 0.0
        vel[left_i] = wl
        vel[right_i] = wr
        carter.apply_action(ArticulationAction(joint_velocities=vel))
        world.step(render=True)

        step_i += 1

finally:
    try:
        omni.timeline.get_timeline_interface().stop()
    except Exception:
        pass
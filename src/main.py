import os
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# ------------------------------------------------------------
# App + core imports
# ------------------------------------------------------------
import numpy as np
import math

from scipy.spatial.transform import Rotation as R

from pxr import Usd, UsdGeom, Gf

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

# Controller params - RETUNED FOR SMOOTHER DRIVING
V_MAX = 0.15          # Increased: Let it move if safe
V_MIN = 0.05          # Don't stop completely unless necessary
W_MAX = 1.0           

STOP_DIST = 0.40      # Only stop if VERY close (was 0.65)
SLOW_DIST = 1.00      # Ignore walls > 1.0m away (was 1.80)
PANIC_DIST = 0.60     # Distance to trigger hard turns for Center obstacles

# drastically reduced gain to prevent spinning
TURN_GAIN = 0.25      # (was 0.8)

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
def get_world_pose(stage, prim_path: str):
    """
    Returns (pos_xyz, quat_wxyz) in WORLD frame for any prim path.
    """
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Invalid prim path: {prim_path}")

    xform = UsdGeom.Xformable(prim)
    mat = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    # translation
    t = mat.ExtractTranslation()
    pos = np.array([t[0], t[1], t[2]], dtype=np.float32)

    # rotation
    # ExtractRotationQuat gives Gf.Quatd (w, (x,y,z))
    q = mat.ExtractRotationQuat()
    quat_wxyz = np.array([q.GetReal(), q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2]], dtype=np.float32)

    return pos, quat_wxyz

from pxr import UsdGeom, Gf

def quat_gf_to_xyzw(q: Gf.Quatd):
    """Gf.Quatd -> (x,y,z,w) for scipy"""
    im = q.GetImaginary()
    return np.array([im[0], im[1], im[2], q.GetReal()], dtype=np.float64)

def get_world_pose_from_xformcache(stage, prim_path: str, cache: UsdGeom.XformCache):
    """Reliable world pose using XformCache (updates as sim runs)."""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Invalid prim: {prim_path}")

    xf = cache.GetLocalToWorldTransform(prim)  # Gf.Matrix4d
    t = xf.ExtractTranslation()                # Gf.Vec3d
    q = xf.ExtractRotationQuat()               # Gf.Quatd

    p = np.array([t[0], t[1], t[2]], dtype=np.float64)
    q_xyzw = quat_gf_to_xyzw(q)
    return p, q_xyzw

def world_points_to_robot_local(xyz_world: np.ndarray, robot_p: np.ndarray, robot_q_xyzw: np.ndarray):
    """
    xyz_world: (N,3) points in WORLD
    robot_p: (3,) WORLD position
    robot_q_xyzw: (4,) robot orientation as (x,y,z,w) in WORLD
    returns: (N,3) points in ROBOT local frame
    """
    if xyz_world is None or len(xyz_world) == 0:
        return xyz_world

    # translate so robot is origin
    rel = xyz_world - robot_p[None, :]

    # rotate world->robot using inverse rotation
    rot_rw = R.from_quat(robot_q_xyzw)   # robot->world
    rel_local = rot_rw.inv().apply(rel)  # world->robot

    return rel_local.astype(np.float32)

def v_w_to_wheels(v, w, r, L):
    # FIXED: Inverted w to match robot's physical wiring.
    # If w > 0 (Left Turn), we now make the Left wheel go faster 
    # (or whatever is needed to physically turn Left).
    wl = (v - w * L / 2.0) / r   # Changed from minus to plus
    wr = (v + w * L / 2.0) / r   # Changed from plus to minus
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
    "L": (20 * DEG, 160 * DEG),    # Starts at 20 deg (was 25)
    "C": (-20 * DEG, 20 * DEG),    # Narrower (-20 to 20)
    "R": (-160 * DEG, -20 * DEG),  # Starts at -20 deg (was -25)
}

MIN_VALID_RANGE = 0.6
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


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


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
# 1. FIXED: Coordinate Mapping (Left = +X)
# ------------------------------------------------------------
def sector_mins_from_pointcloud(pts_xyz: np.ndarray, forward_axis="y"):
    """
    Computes minimum distance in L, C, R sectors using arctan2 for correct angles.
    Assumption: Y is Forward, X is Left (Standard Right-Hand Rule).
    """
    mins = {"L": float("inf"), "C": float("inf"), "R": float("inf")}

    if pts_xyz is None or len(pts_xyz) == 0:
        return mins

    x = pts_xyz[:, 0]
    y = pts_xyz[:, 1]
    z = pts_xyz[:, 2]

    dist_all = np.sqrt(x*x + y*y)
    print("[DBG] raw dist min/med/p10:",
        float(np.min(dist_all)),
        float(np.median(dist_all)),
        float(np.percentile(dist_all, 10)))
    print("[DBG] near<0.6m:", int(np.sum(dist_all < 0.6)), "of", len(dist_all))
    print("[DBG] y>0:", int(np.sum(y > 0)), " y<0:", int(np.sum(y < 0)))

    # --- COORDINATE ROTATION ---
    # We calculate the angle relative to the Forward axis (Y).
    # ang = atan2(Opposite, Adjacent) = atan2(Side, Forward)
    # If Y is Forward and X is Left:
    #   Point(1, 1) -> Left-Forward -> atan2(1, 1) = +45 deg (Correct)
    #   Point(-1, 1) -> Right-Forward -> atan2(-1, 1) = -45 deg (Correct)
    if forward_axis == "y":
        ang = np.arctan2(x, y)  # ang > 0 is Left, ang < 0 is Right
        dist = np.sqrt(x*x + y*y)
    else:
        # Standard: X is Forward, Y is Left
        ang = np.arctan2(y, x)
        dist = np.sqrt(x*x + y*y)

    # --- FILTERING ---
    valid = (
        np.isfinite(dist) &
        (dist > MIN_VALID_RANGE) &
        (dist < MAX_VALID_RANGE) &
        (z > -0.40) & (z < 1.0)
    )
    
    # Ignore points behind the robot (angles > 135 deg) to prevent phantom stops
    valid = valid & (np.abs(ang) < (135 * DEG))

    if not np.any(valid):
        return mins

    ang = ang[valid]
    dist = dist[valid]

    # --- SECTOR MAPPING ---
    for k, (a0, a1) in SECTORS.items():
        # Check if angle falls within sector range
        m = (ang >= a0) & (ang <= a1)
        if np.any(m):
            mins[k] = float(dist[m].min())

    return mins


# ------------------------------------------------------------
# 2. FIXED: Steering Logic (Repulsive Force)
# ------------------------------------------------------------

def compute_cmd_from_sectors(mins, w_prev=0.0):
    L, C, R = mins["L"], mins["C"], mins["R"]
    
    SAFE = 10.0
    if not np.isfinite(L): L = SAFE
    if not np.isfinite(C): C = SAFE
    if not np.isfinite(R): R = SAFE

    closest_dist = min(L, C, R)

    # --- STEERING LOGIC ---
    w_cmd = 0.0
    
    # 1. EMERGENCY PANIC: Center is VERY close (< 0.6m)
    if C < PANIC_DIST:
        # Hard turn away from the block
        if R > L:
            w_cmd = -1.0 * W_MAX  # Turn Right
        else:
            w_cmd = 1.0 * W_MAX   # Turn Left

    # 2. STANDARD AVOIDANCE: Obstacles nearby (< 1.2m)
    # This handles both "Center" (if > 0.6m) and "Side" obstacles smoothly
    elif closest_dist < SLOW_DIST:
        
        # Calculate push forces
        # Force = 1 / Distance
        # Right wall pushes Left (+), Left wall pushes Right (-)
        push_r = (1.0 / R) 
        push_l = (1.0 / L)
        
        # If Center is visible (but not panic close), add it to the push
        # We assume Center pushes towards the previous turn direction or dominant side
        push_c = 0.0
        if C < SLOW_DIST:
            if abs(R - L) < 0.05 and abs(w_prev) < 0.1:
                push_c = 0.0
            elif w_prev > 0.1 or R < L:
                push_c = (1.0 / C) * 0.5
            else:
                push_c = -(1.0 / C) * 0.5
                
        # Total repulsive force
        total_force = push_r - push_l + push_c
        
        # Apply gain
        w_cmd = total_force * TURN_GAIN

    # 3. OPEN SPACE: Straighten out
    else:
        # Decay turn slower to avoid wobbling
        w_cmd = w_prev * 0.75

    # Clamp steering
    w_cmd = clamp(w_cmd, -W_MAX, W_MAX)

    # --- SPEED LOGIC ---
    # Base speed
    v_cmd = V_MAX

    # Slow down based on proximity
    if C < STOP_DIST:
        v_cmd = 0.0
    elif C < PANIC_DIST:
        v_cmd = 0.05 # Crawl
    else:
        # If turning hard, slow down slightly, but don't stop
        turn_factor = abs(w_cmd) / W_MAX
        v_cmd = V_MAX * (1.0 - 0.5 * turn_factor)
        v_cmd = max(v_cmd, V_MIN)

    return float(v_cmd), float(w_cmd)

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
    **{"omni:sensor:Core:scanRateBaseHz": 60},  # Higher scan rate
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

stage = omni.usd.get_context().get_stage()
xform_cache = UsdGeom.XformCache()

try:
    step_i = 0
    while simulation_app.is_running():
        # ----------------------------
        # LiDAR read (more frequent)
        # ----------------------------
        
        if once_every(step_i, READ_EVERY):
            rep.orchestrator.step()
            frame = lidar.get_current_frame()

            lidar_p, lidar_q = get_world_pose(stage, LIDAR_SENSOR_PATH)
            robot_p, robot_q = get_world_pose(stage, ROBOT_PRIM_PATH)

            print("[POSE] robot_p:", np.array(robot_p), " lidar_p:", np.array(lidar_p))            

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

            # if xyz is None or len(xyz) == 0:
            #     print("[LIDAR] no points this frame")
            # else:
                # lidar_p, _ = get_world_pose(stage, LIDAR_SENSOR_PATH)
                # xyz = xyz - np.array(lidar_p)

                # lp = np.array(lidar_p, dtype=np.float32)

                # dist_as_is = np.linalg.norm(xyz[:, :2], axis=1)              # assumes lidar-local
                # dist_minus_pose = np.linalg.norm((xyz[:, :2] - lp[:2]), axis=1)  # assumes world-frame
                # print("[FRAMECHK] min dist (as-is):", float(dist_as_is.min()),
                #     " | min dist (minus lidar pose):", float(dist_minus_pose.min()))

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

            # refresh cache each time step
            xform_cache.SetTime(omni.timeline.get_timeline_interface().get_current_time())

            robot_p, robot_q_xyzw = get_world_pose_from_xformcache(stage, ROBOT_PRIM_PATH, xform_cache)

            # Convert lidar point cloud (assumed WORLD) -> ROBOT local
            xyz_local = world_points_to_robot_local(last_xyz, robot_p, robot_q_xyzw)

            # mins = sector_mins_from_pointcloud(xyz_local, forward_axis="y")

            mins = sector_mins_from_pointcloud(last_xyz, forward_axis="y")
                     
            if once_every(step_i, 60):
                log(Log.CYAN, "POSE", f"robot_p={robot_p} robot_q_xyzw={robot_q_xyzw}")

            if once_every(step_i, 60) and last_xyz is not None:
                # if points are world-frame, their mean should move when robot moves
                mean_xy = last_xyz[:, :2].mean(axis=0)
                min_xy  = last_xyz[:, :2].min(axis=0)
                max_xy  = last_xyz[:, :2].max(axis=0)
                print("[PCD] mean_xy", mean_xy, " min_xy", min_xy, " max_xy", max_xy)

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
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
DEBUG_SECTORS = True  # set True temporarily when debugging

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
    "L": (15 * DEG, 90 * DEG),
    "C": (-20 * DEG, 20 * DEG),
    "R": (-90 * DEG, -15 * DEG),
}
MIN_VALID_RANGE = 0.15
MAX_VALID_RANGE = 8.0

def sector_mins_from_pointcloud(pts_xyz: np.ndarray, forward_axis="y"):
    """
    forward_axis: "x" or "y"
    We assume the other horizontal axis is left/right.
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
        l = pts_xyz[:, 0]  # left

    z = pts_xyz[:, 2]

    # --- 2D distance in the horizontal plane (forward-left plane) ---
    d = np.sqrt(f*f + l*l)

    # --- filter ---
    # only points in front, reasonable range, and near sensor height (reduce floor/ceiling noise)
    valid = (
        (f > 0.05) &
        np.isfinite(d) &
        (d > MIN_VALID_RANGE) &
        (d < MAX_VALID_RANGE) &
        (z > -0.30) & (z < 0.70)          # <-- keep wall points; reduce floor/ceiling noise
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
        if cnt > 0:
            log(Log.GRAY, "SECTOR_CNT", f"{k} count={cnt} min={float(d[m].min()):.2f}")

    return mins


def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def compute_cmd_from_sectors(mins, w_prev=0.0):
    L, C, R = mins["L"], mins["C"], mins["R"]
    L_ok, C_ok, R_ok = np.isfinite(L), np.isfinite(C), np.isfinite(R)

    # if C missing, be conservative instead of assuming clear
    Cm = C if C_ok else SLOW_DIST

    # --- speed ---
    if Cm <= STOP_DIST:
        v_cmd = 0.0
    elif Cm >= SLOW_DIST:
        v_cmd = V_MAX
    else:
        alpha = (Cm - STOP_DIST) / (SLOW_DIST - STOP_DIST)
        v_cmd = V_MIN + alpha * (V_MAX - V_MIN)

    # --- turning ---
    # Default straight
    w_cmd = 0.0

    # If something is within "slow zone", start steering away
    if Cm <= SLOW_DIST:
        if L_ok and R_ok:
            # obstacle closer on left => turn right (negative)
            w_cmd = -TURN_GAIN * (R - L)
        elif R_ok and not L_ok:
            # we only see right-side obstacles => turn LEFT
            w_cmd = +0.7 * W_MAX
        elif L_ok and not R_ok:
            # we only see left-side obstacles => turn RIGHT
            w_cmd = -0.7 * W_MAX
        else:
            w_cmd = 0.0

        # stronger when very close
        if Cm <= STOP_DIST:
            w_cmd *= 1.5

    # clamp + smooth
    w_cmd = clamp(w_cmd, -W_MAX, W_MAX)
    SMOOTH = 0.85
    w_cmd = SMOOTH * w_prev + (1.0 - SMOOTH) * w_cmd

    return float(v_cmd), float(w_cmd)

def log_cmd_if_changed(v, w, v_prev, w_prev, mins):
    dv = abs(v - v_prev)
    dw = abs(w - w_prev)

    C = mins["C"]
    near = np.isfinite(C) and (C < SLOW_DIST)

    # log if command changed meaningfully OR we are near something ahead
    if dv > 0.03 or dw > 0.10 or near:
        log(
            Log.YELL if near else Log.GRAY,
            "CMD",
            f"v={v:.2f} w={w:.2f} | L={fmt(mins['L'])} C={fmt(mins['C'])} R={fmt(mins['R'])}"
        )

def log_cmd_state(v, w, mins, reason=""):
    color = Log.GREEN
    C = mins.get("C", float("inf"))
    if np.isfinite(C) and C < STOP_DIST:
        color = Log.RED
    elif np.isfinite(C) and C < SLOW_DIST:
        color = Log.YELL

    log(color, "CMD",
        f"{reason} v={v:.2f} w={w:.2f} | L={fmt(mins['L'])} C={fmt(mins['C'])} R={fmt(mins['R'])}"
    )

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

# Reactive controller params

V_MAX = 0.25          # m/s
V_MIN = 0.05          # m/s (creep)
W_MAX = 1.2           # rad/s (turn speed cap)

STOP_DIST = 0.55      # if front closer than this -> strong avoid
SLOW_DIST = 1.60      # start slowing down here

TURN_GAIN = 1.0       # how strongly to turn away from obstacles
CENTER_GAIN = 0.6     # gentle centering when clear
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
v_prev, w_prev = 0.0, 0.0

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

            xyz = to_xyz_points(pts) # xyz shape: (n, 3)

            if step_i == 0:
                ang_x = np.degrees(np.arctan2(xyz[:,1], xyz[:,0]))  # assume forward=x
                ang_y = np.degrees(np.arctan2(xyz[:,0], xyz[:,1]))  # assume forward=y
                log(Log.CYAN, "AXIS", f"assume fwd=x angle range: {ang_x.min():.1f}..{ang_x.max():.1f}")
                log(Log.CYAN, "AXIS", f"assume fwd=y angle range: {ang_y.min():.1f}..{ang_y.max():.1f}")

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
                ang_deg = np.degrees(np.arctan2(xyz[:, 0], xyz[:, 1]))
                log(Log.GRAY, "LIDAR", f"angle deg: min={ang_deg.min():.1f}, max={ang_deg.max():.1f}")

            mins = sector_mins_from_pointcloud(xyz, forward_axis="y")

            log(
                Log.GREEN,
                "SECTOR",
                f"L={fmt(mins['L'])}  C={fmt(mins['C'])}  R={fmt(mins['R'])}",
                end="\r"
            )


        # ----------------------------
        # Drive robot (constant v,w for now)
        # ----------------------------
        v, w = compute_cmd_from_sectors(mins, w_prev=w_prev)
        
        log_cmd_if_changed(v, w, v_prev, w_prev, mins)
        near = np.isfinite(mins["C"]) and mins["C"] < SLOW_DIST
        turning = abs(w) > 0.05
        changed = abs(v - v_prev) > 0.02 or abs(w - w_prev) > 0.10

        if near or turning or changed:
            log(Log.YELL if near else Log.GRAY, "CMD",
                f"v={v:.2f} w={w:.2f} | L={fmt(mins['L'])} C={fmt(mins['C'])} R={fmt(mins['R'])}"
            )

        # reason labels
        reason = ""
        if np.isfinite(mins["C"]) and mins["C"] <= STOP_DIST:
            reason = "STOP_DIST "
        elif np.isfinite(mins["C"]) and mins["C"] <= SLOW_DIST:
            reason = "SLOW_DIST "

        # log when close OR turning meaningfully
        if reason or abs(w) > 0.15:
            log_cmd_state(v, w, mins, reason=reason)

        v_prev, w_prev = v, w

        log(
            Log.GREEN,
            "SECTOR",
            f"L={fmt(mins['L'])}  C={fmt(mins['C'])}  R={fmt(mins['R'])} | v={v:.2f} w={w:.2f}",
            end="\r"
        )

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

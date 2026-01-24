import os
import numpy as np

# --- CHECK ENV BEFORE STARTING ---
if "ROS_DISTRO" not in os.environ:
    print("\n[ERROR] ROS2 environment not found!")
    print("Please run: source /opt/ros/humble/setup.bash")
    sys.exit(1)

# 1. CONFIGURATION: Load extensions immediately
# This ensures the Lidar nodes are recognized when the USD is opened.
CONFIG = {
    "headless": False,
    "renderer": "RayTracedLighting",
    "extensions": [
        "omni.isaac.ros2_bridge",
        "omni.isaac.sensor",
        "omni.isaac.core_nodes",
        "omni.graph.bundle.action"
    ]
}

# 2. START SIMULATION APP
# Note: This import must happen before any omni imports
try:
    from isaacsim import SimulationApp
except ImportError:
    from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp(CONFIG)

# ---------------------------------------------------------
# NOW IMPORTS (Must be after SimulationApp)
# ---------------------------------------------------------
import omni.timeline
from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage, get_current_stage
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.prims import is_prim_path_valid

# ---------------------------------------------------------
# FORCE ENABLE EXTENSIONS
# ---------------------------------------------------------
import omni.kit.app
from omni.isaac.core.utils.extensions import enable_extension

# Explicitly force the bridge on
print("[STARTUP] Enabling ROS2 Bridge...")
enable_extension("omni.isaac.ros2_bridge")

# Wait one update frame to let the extension system register it
simulation_app.update()

# CHECK AGAIN
ext_manager = omni.kit.app.get_app().get_extension_manager()
if not ext_manager.is_extension_enabled("omni.isaac.ros2_bridge"):
    print("\n[FATAL] ROS2 Bridge failed to load even after force-enable.")
    print("Check looking at the terminal logs above for 'librclpy.so not found' or similar errors.")
    simulation_app.close()
    sys.exit(1)
else:
    print("[SUCCESS] ROS2 Bridge is ACTIVE.")

# ---------------------------------------------------------
# CONSTANTS
# ---------------------------------------------------------
# Update this path to your exact file location
WORLD_USD = "/home/saman-aboutorab/projects/Robotics/isaac_diff_drive_nav/isaac/worlds/simple_obstacles_carter_lidar.usd"
ROBOT_PRIM_PATH = "/World/nova_carter"
ACTION_GRAPH_PATH = "/Graph/ROS_LidarRTX" # Check your USD for the exact name

LEFT_WHEEL_DOF  = "joint_wheel_left"
RIGHT_WHEEL_DOF = "joint_wheel_right"
LEFT_WHEEL_VEL  = 4.0 
RIGHT_WHEEL_VEL = 4.0

def log(tag, msg):
    print(f"[{tag}] {msg}")

# ---------------------------------------------------------
# SETUP WORLD
# ---------------------------------------------------------
if not os.path.exists(WORLD_USD):
    simulation_app.close()
    raise FileNotFoundError(f"Could not find USD: {WORLD_USD}")

log("STARTUP", "Loading Stage...")
open_stage(WORLD_USD)

# Verify extensions are active
ext_manager = omni.kit.app.get_app().get_extension_manager()
ros_enabled = ext_manager.is_extension_enabled("omni.isaac.ros2_bridge")
sensor_enabled = ext_manager.is_extension_enabled("omni.isaac.sensor")
log("CHECK", f"ROS2 Bridge Enabled: {ros_enabled}")
log("CHECK", f"Isaac Sensor Enabled: {sensor_enabled}")

# Initialize World
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane() # Optional, if your USD doesn't have one

# ---------------------------------------------------------
# SETUP ROBOT
# ---------------------------------------------------------
if not is_prim_path_valid(ROBOT_PRIM_PATH):
    simulation_app.close()
    raise RuntimeError(f"Robot prim not found at: {ROBOT_PRIM_PATH}")

carter = Articulation(prim_path=ROBOT_PRIM_PATH, name="nova_carter")
world.scene.add(carter)

# Resetting the world allows the Articulation to register
world.reset()

# Locate Joints
dof_names = carter.dof_names
log("ROBOT", f"DOFs found: {dof_names}")

try:
    left_i = dof_names.index(LEFT_WHEEL_DOF)
    right_i = dof_names.index(RIGHT_WHEEL_DOF)
except ValueError as e:
    simulation_app.close()
    raise ValueError(f"Could not find wheel joints. Check names in USD. {e}")

# ---------------------------------------------------------
# EXECUTION LOOP
# ---------------------------------------------------------
log("SIM", "Starting Simulation loop...")
timeline = omni.timeline.get_timeline_interface()
timeline.play() # CRITICAL: Action Graphs require the timeline to be playing

# Tick once to ensure graphs initialize
world.step(render=True) 

vel = np.zeros(len(dof_names), dtype=np.float32)

try:
    step_count = 0
    while simulation_app.is_running():
        
        # Apply velocity
        vel[:] = 0.0
        vel[left_i] = LEFT_WHEEL_VEL
        vel[right_i] = RIGHT_WHEEL_VEL
        carter.apply_action(ArticulationAction(joint_velocities=vel))

        # Step the physics and rendering
        world.step(render=True)
        
        # Debug helper: Verify ROS bridge is actually ticking occasionally
        if step_count % 60 == 0:
            # You can check 'ros2 topic list' in a separate terminal now
            pass
            
        step_count += 1

except KeyboardInterrupt:
    log("SIM", "Stopping...")

finally:
    timeline.stop()
    simulation_app.close()
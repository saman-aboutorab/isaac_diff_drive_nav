import os
import sys

# ---------------------------------------------------------
# 0) ENV CHECK (optional but useful)
# ---------------------------------------------------------
# NOTE: We do NOT import rclpy here. This check is just to avoid
# "I forgot to source ROS" confusion when ROS graph nodes are used.
if "ROS_DISTRO" not in os.environ:
    print("\n[ERROR] ROS2 environment not found!")
    print("In a terminal run (example): source /opt/ros/jazzy/setup.bash")
    print("Then launch Isaac Sim from that same terminal.")
    sys.exit(1)

# ---------------------------------------------------------
# 1) SIMULATION APP CONFIG (extensions must load BEFORE omni imports)
# ---------------------------------------------------------
CONFIG = {
    "headless": False,
    "renderer": "RayTracedLighting",
    "extensions": [
        # ROS2 bridge + graph nodes
        "omni.isaac.ros2_bridge",
        "omni.graph.bundle.action",

        # Core Isaac / nodes (often needed by graph controllers)
        "omni.isaac.core_nodes",
        "omni.isaac.sensor",
    ],
}

try:
    from isaacsim import SimulationApp
except ImportError:
    from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp(CONFIG)

# ---------------------------------------------------------
# 2) IMPORTS (after SimulationApp!)
# ---------------------------------------------------------
import omni
import omni.timeline
import omni.kit.app

from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.core.utils.extensions import enable_extension

# ---------------------------------------------------------
# 3) CONSTANTS
# ---------------------------------------------------------
WORLD_USD = "/home/saman-aboutorab/projects/Robotics/isaac_diff_drive_nav/isaac/worlds/simple_obstacles_carter_lidar.usd"

Wheel_Distance = 0.413
Wheel_Radius = 0.14

# If you want, just for sanity checks/logging (not required):
ACTION_GRAPH_PATH = "/Graph/ROS_LidarRTX"  # update if your graph path is different

def log(tag, msg):
    print(f"[{tag}] {msg}")

# ---------------------------------------------------------
# 4) FORCE ENABLE EXTENSIONS (extra safety)
# ---------------------------------------------------------
log("STARTUP", "Enabling ROS2 Bridge extension...")
enable_extension("omni.isaac.ros2_bridge")
enable_extension("omni.graph.bundle.action")
enable_extension("omni.isaac.core_nodes")
enable_extension("omni.isaac.sensor")

# Tick one frame so extension manager registers everything
simulation_app.update()

ext_manager = omni.kit.app.get_app().get_extension_manager()
ros_ok = ext_manager.is_extension_enabled("omni.isaac.ros2_bridge")
log("CHECK", f"ROS2 Bridge Enabled: {ros_ok}")

if not ros_ok:
    log("FATAL", "ROS2 Bridge did not enable. Check terminal logs for missing rclpy/libraries.")
    simulation_app.close()
    sys.exit(1)

# ---------------------------------------------------------
# 5) LOAD STAGE + WORLD
# ---------------------------------------------------------
if not os.path.exists(WORLD_USD):
    simulation_app.close()
    raise FileNotFoundError(f"USD not found: {WORLD_USD}")

log("STARTUP", f"Opening USD: {WORLD_USD}")
open_stage(WORLD_USD)

world = World(stage_units_in_meters=1.0)

# Optional: only add ground plane if your USD does not already have one
# world.scene.add_default_ground_plane()

world.reset()

# ---------------------------------------------------------
# 6) RUN LOOP (timeline must be playing for Action Graph)
# ---------------------------------------------------------
timeline = omni.timeline.get_timeline_interface()
timeline.play()
log("SIM", "Timeline started. Action Graph should now run.")

try:
    while simulation_app.is_running():
        # Step physics/render. Action Graph evaluates while timeline is playing.
        world.step(render=True)

except KeyboardInterrupt:
    log("SIM", "KeyboardInterrupt - stopping")

finally:
    timeline.stop()
    simulation_app.close()
    log("SIM", "Closed cleanly.")

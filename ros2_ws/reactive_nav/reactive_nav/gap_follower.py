#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class ReactiveNavigator(Node):
    """
    Subscribes: /scan (sensor_msgs/LaserScan)
    Publishes:  /cmd_vel (geometry_msgs/Twist)

    Strategy: 3-sector reactive avoidance
    - Compute min range in left/front/right sectors
    - If front is unsafe -> turn to the clearer side
    - Else -> go forward, with small steering away from the closer side
    - Watchdog: if scan is stale -> publish zero cmd_vel
    """

    def __init__(self):
        super().__init__("reactive_navigator")

        # ---- Parameters (tune in one place) ----
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("control_hz", 10.0)

        self.declare_parameter("front_angle_deg", 25.0)   # +/- around 0 deg
        self.declare_parameter("side_angle_deg", 70.0)    # side sector width

        self.declare_parameter("stop_distance", 0.60)
        self.declare_parameter("slow_distance", 1.20)

        self.declare_parameter("max_linear", 0.40)
        self.declare_parameter("min_linear", 0.08)
        self.declare_parameter("max_angular", 1.20)

        self.declare_parameter("turn_speed", 0.8)          # rad/s
        self.declare_parameter("turn_angle_deg", 90.0)     # degrees
        self.declare_parameter("front_clearance", 0.75)    # start turning below this
        self.declare_parameter("resume_clearance", 1.10)   # optional hysteresis

        # ---- Debug / logging ----
        self.debug_every_n = 10     # log once every N control ticks
        self._tick_count = 0
        self._last_state = None    # for state-change logging

        self.scan_topic = self.get_parameter("scan_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value
        control_hz = float(self.get_parameter("control_hz").value)

        self.front_angle = math.radians(float(self.get_parameter("front_angle_deg").value))
        self.side_angle = math.radians(float(self.get_parameter("side_angle_deg").value))

        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.slow_distance = float(self.get_parameter("slow_distance").value)

        self.max_linear = float(self.get_parameter("max_linear").value)
        self.min_linear = float(self.get_parameter("min_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)

        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.turn_angle = math.radians(float(self.get_parameter("turn_angle_deg").value))
        self.front_clearance = float(self.get_parameter("front_clearance").value)
        self.resume_clearance = float(self.get_parameter("resume_clearance").value)

        self.state = "FORWARD"
        self.turn_dir = 0.0
        self.turn_end_time = None

        # ---- ROS I/O ----
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)

        # Latest scan cache
        self.last_scan: Optional[LaserScan] = None
        self.last_scan_time = self.get_clock().now()

        # Timer-based control loop (portfolio-friendly)
        self.timer = self.create_timer(1.0 / control_hz, self._control_tick)

        self.get_logger().info(
            f"ReactiveNavigator started | scan={self.scan_topic} cmd={self.cmd_topic} hz={control_hz}"
        )

    def _dbg(self, msg: str):
        self.get_logger().info(msg)

    def _on_scan(self, msg: LaserScan):
        self.last_scan = msg
        self.last_scan_time = self.get_clock().now()

    def _control_tick(self):
        self._tick_count += 1
        # Watchdog: stop if scan is stale
        if self.last_scan is None:
            self._publish_stop("no_scan_yet")
            if self._tick_count % self.debug_every_n == 0:
                self._dbg("[STATE] WAITING_FOR_SCAN")
            return

        age = self.get_clock().now() - self.last_scan_time
        if age > Duration(seconds=0.5):
            self._publish_stop("scan_stale")
            if self._tick_count % self.debug_every_n == 0:
                self._dbg(f"[STATE] SCAN_STALE age={age.nanoseconds/1e6:.1f} ms")
            return


        scan = self.last_scan
        ranges = scan.ranges
        if not ranges:
            self._publish_stop("empty_scan")
            return

        # Compute sectors distance
        front_perc = self._sector_dist(scan, 0.0, self.front_angle, pct=0.10)
        left_perc  = self._sector_dist(scan, +math.pi/2, self.side_angle, pct=0.10)
        right_perc = self._sector_dist(scan, -math.pi/2, self.side_angle, pct=0.10)

        if self._tick_count % self.debug_every_n == 0:
            self._dbg(
                f"[SCAN] front={front_perc:.2f} m | "
                f"left={left_perc:.2f} m | right={right_perc:.2f} m"
            )

        # --- Decision (simple state machine) ---

        now = self.get_clock().now()

        if self.state == "FORWARD":
            # drive straight
            if front_perc < self.front_clearance:
                # pick side with more space
                # (turn left if left is more open, else right)
                self.turn_dir = +1.0 if left_perc > right_perc else -1.0

                w = self.turn_dir * abs(self.turn_speed)
                duration = self.turn_angle / max(1e-6, abs(w))  # seconds to rotate ~90deg
                self.turn_end_time = now + Duration(seconds=float(duration))

                self.state = "TURNING"
                self._dbg(f"[STATE] TURNING start dir={'L' if self.turn_dir>0 else 'R'} "
                        f"front={front_perc:.2f} L={left_perc:.2f} R={right_perc:.2f} "
                        f"t={duration:.2f}s w={w:.2f}")
                self._publish_cmd(0.0, w)
                return

            # otherwise keep moving forward (no steering)
            v = self.max_linear if front_perc >= self.slow_distance else self.min_linear
            self._publish_cmd(v, 0.0)
            if self._tick_count % self.debug_every_n == 0:
                self._dbg(f"[DECISION] FORWARD | v={v:.2f} front={front_perc:.2f}")
            return


        if self.state == "TURNING":
            # keep turning until end time
            w = self.turn_dir * abs(self.turn_speed)

            if self.turn_end_time is not None and now >= self.turn_end_time and front_perc > self.resume_clearance:
                self.state = "FORWARD"
                self.turn_end_time = None
                self._dbg(f"[STATE] FORWARD resume front={front_perc:.2f}")
                self._publish_cmd(0.0, 0.0)
                return

            self._publish_cmd(0.0, w)
            if self._tick_count % self.debug_every_n == 0:
                self._dbg(f"[DECISION] TURNING | w={w:.2f} front={front_perc:.2f}")
            return

        # # Handle all-inf case
        # if math.isinf(front_perc) and math.isinf(left_perc) and math.isinf(right_perc):
        #     v = self.max_linear * 0.7
        #     self._publish_cmd(v, 0.0)
        #     if self._tick_count % self.debug_every_n == 0:
        #         self._dbg(f"[DECISION] CLEAR → v={v:.2f} w=0.00")
        #     return

        # # Decision
        # if front_perc < self.stop_distance:
        #     turn_dir = +1.0 if left_perc > right_perc else -1.0
        #     w = turn_dir * self.max_angular

        #     state = "TURN_LEFT" if turn_dir > 0 else "TURN_RIGHT"
        #     if state != self._last_state:
        #         self._dbg(
        #             f"[DECISION] {state} | "
        #             f"front={front_perc:.2f} < stop={self.stop_distance:.2f}"
        #         )
        #         self._last_state = state

        #     self._publish_cmd(0.0, w)
        #     return

        # # Otherwise: drive forward with speed scaled by front clearance
        # # scale in [stop_distance..slow_distance] -> [min_linear..max_linear]
        # if front_perc < self.slow_distance:
        #     t = (front_perc - self.stop_distance) / max(1e-6, (self.slow_distance - self.stop_distance))
        #     v = self.min_linear + t * (self.max_linear - self.min_linear)
        #     speed_mode = "SLOW"
        # else:
        #     v = self.max_linear
        #     speed_mode = "FAST"

        # # Gentle steering away from closer side
        # # If left is closer -> steer right (negative), if right is closer -> steer left (positive)
        # # Use difference ratio for smoothness
        # eps = 1e-6
        # if math.isinf(left_perc): left_perc = scan.range_max
        # if math.isinf(right_perc): right_perc = scan.range_max

        # diff = (left_perc - right_perc) / max(eps, (left_perc + right_perc))
        # w = _clamp(diff * self.max_angular, -self.max_angular, self.max_angular)

        # if self._tick_count % self.debug_every_n == 0:
        #     self._dbg(
        #         f"[DECISION] DRIVE({speed_mode}) | "
        #         f"v={v:.2f} m/s w={w:.2f} rad/s"
        #     )

        # self._last_state = "DRIVE"
        # self._publish_cmd(v, w)

    # def _sector_min(self, scan: LaserScan, center_angle: float, half_width: float) -> float:
    #     """
    #     Returns the minimum valid range within [center_angle - half_width, center_angle + half_width].
    #     center_angle is in robot frame where 0 is forward, +pi/2 left, -pi/2 right.
    #     """
    #     a_min = scan.angle_min
    #     a_inc = scan.angle_increment
    #     n = len(scan.ranges)

    #     def angle_to_index(a: float) -> int:
    #         return int(round((a - a_min) / a_inc))

    #     start_a = center_angle - half_width
    #     end_a = center_angle + half_width

    #     i0 = max(0, min(n - 1, angle_to_index(start_a)))
    #     i1 = max(0, min(n - 1, angle_to_index(end_a)))
    #     if i1 < i0:
    #         i0, i1 = i1, i0

    #     best = float("inf")
    #     rmin = scan.range_min
    #     rmax = scan.range_max

    #     for r in scan.ranges[i0 : i1 + 1]:
    #         if r <= 0.0 or math.isnan(r):
    #             continue
    #         # ignore out-of-range returns
    #         if r < rmin or r > rmax:
    #             continue
    #         if r < best:
    #             best = r

    #     return best

    def _sector_dist(self, scan: LaserScan, center_angle: float, half_width: float, pct: float = 0.10) -> float:
        a_min = scan.angle_min
        a_inc = scan.angle_increment
        n = len(scan.ranges)

        def angle_to_index(a: float) -> int:
            return int(round((a - a_min) / a_inc))

        i0 = max(0, min(n - 1, angle_to_index(center_angle - half_width)))
        i1 = max(0, min(n - 1, angle_to_index(center_angle + half_width)))
        if i1 < i0:
            i0, i1 = i1, i0

        vals = []
        rmin = scan.range_min
        rmax = scan.range_max

        # extra filter to avoid self-hits / weird near returns
        near_ignore = max(rmin, 0.25)

        for r in scan.ranges[i0:i1+1]:
            if r <= 0.0 or math.isnan(r):
                continue
            if r < near_ignore or r > rmax:
                continue
            vals.append(r)

        if not vals:
            return float("inf")

        vals.sort()
        k = int(pct * (len(vals) - 1))
        return vals[k]

    def _publish_stop(self, reason: str):
        self._publish_cmd(0.0, 0.0)
        # Don’t spam logs every tick; log occasionally if you want later.

    def _publish_cmd(self, v: float, w: float):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = ReactiveNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

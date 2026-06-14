#!/usr/bin/env python3
"""
scan_filter.py — LIDAR CLEAN-UP, built for this project to handle the ARM.

═══════════════════════════════════════════════════════════════════════════════
ROLE
═══════════════════════════════════════════════════════════════════════════════
Built specifically because of the swivelling arm.  On the A↔B carry leg the
arm/gripper swings FORWARD into the lidar's scan plane (~0.25-0.35 m ahead), and
those returns land INSIDE the robot's own footprint — so SLAM and the Nav2
costmaps mark the robot as permanently in collision (Nav2 reports "collision
ahead" for every motion and the leg can't run).  This node drops every return
below `min_range` and republishes the rest as /scan_filtered, clearing the arm
out of the scan.

NOTE: the chassis alone never needed this — plain mapping with the arm tucked maps
fine on the raw /scan (the mirte_navigation reference did exactly that).  The
filter earns its keep only because the arm intrudes; min_range is raised to 0.40 m
in the mission specifically to clear the extended arm.

═══════════════════════════════════════════════════════════════════════════════
HARDWARE  ⇄  PUBLISHED RESULT
═══════════════════════════════════════════════════════════════════════════════
  lidar sensor ─► /scan (sensor_msgs/LaserScan, BEST_EFFORT QoS)
        │
        ▼  drop ranges < min_range  (self-returns → +inf, i.e. "no obstacle")
  /scan_filtered ──► slam_toolbox  (builds the map + map→odom)
                 └─► Nav2 obstacle_layer  (marks real obstacles in both costmaps)

  This is the SINGLE shared sensor input to BOTH mapping (SLAM) and navigation
  (costmaps); everything the robot "sees" of the world passes through here.

LINKS / WHY:
  - Subscribes /scan with qos_profile_sensor_data (BEST_EFFORT): a default
    RELIABLE subscriber silently receives nothing from a best-effort lidar.
  - min_range is a ROS param (default 0.25, mission raises it to 0.40) because
    the cutoff depends on the unit's geometry and whether the arm sits in the
    lidar plane — see the inline note below.
  - raytrace_min_range in the Nav2 costmaps MUST match this cutoff, or obstacles
    inside the blind zone get erased right as the robot reaches them.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

MIN_RANGE = 0.25  # metres — LIDAR sits at base_link (x=+0.10, y=0), 10 cm
                  # forward of robot centre.  Self-return distances:
                  #   chassis rear : 0.10 + 0.14 = 0.24 m  ← worst case
                  #   left wheels  : 0.18 m
                  #   right wheels : 0.13 m
                  #   chassis front: 0.04 m
                  # 0.25 m clears all self-returns in every direction.


class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        self._min_range = float(self.declare_parameter('min_range', MIN_RANGE).value)
        self._pub = self.create_publisher(LaserScan, '/scan_filtered', 10)
        # Lidars publish with SensorData QoS (BEST_EFFORT).  A default RELIABLE
        # subscriber receives NOTHING from a BEST_EFFORT publisher (and gives no
        # error) — which silently starves SLAM.  Subscribe with sensor QoS so we
        # match any lidar (sim or the real MIRTE).
        self.create_subscription(LaserScan, '/scan', self._cb, qos_profile_sensor_data)
        self.get_logger().info(f'Filtering scan readings below {self._min_range} m')

    def _cb(self, msg):
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = self._min_range
        out.range_max = msg.range_max
        out.ranges = tuple(
            r if r >= self._min_range else float('inf') for r in msg.ranges
        )
        out.intensities = msg.intensities
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

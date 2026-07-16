#!/usr/bin/env python3
"""
tf_gate.py — STARTUP GATE: block until the map→odom transform is live, then exit.

═══════════════════════════════════════════════════════════════════════════════
ROLE
═══════════════════════════════════════════════════════════════════════════════
A one-shot helper used only by the launch file.  Nav2 must not start before its
real precondition exists — the map→odom transform (SLAM is up and localising).
The launch used to approximate that with a fixed 35 s timer; instead this node
polls TF and EXITS as soon as map→odom appears, and shuttle.launch.py keys Nav2's
start off this process exiting (RegisterEventHandler(OnProcessExit)).  So Nav2
comes up the instant the map exists, not on a wall-clock guess.

It exits (cleanly, code 0) on EITHER condition:
  • map→odom becomes available  → "transform live, releasing Nav2", OR
  • `timeout` seconds elapse     → exit anyway so the launch never hangs
    (Nav2 still starts; shuttle_manager's own slam_wait_timeout then aborts if
    SLAM truly never came up).

Uses Time() (latest-available) for the lookup so it's sim-time agnostic — same
tf2 pattern as shuttle_manager._robot_pose.
"""
import rclpy
from rclpy.node import Node
import rclpy.time
import tf2_ros


class TfGate(Node):
    def __init__(self):
        super().__init__('tf_gate')
        self._target = str(self.declare_parameter('target_frame', 'map').value)
        self._source = str(self.declare_parameter('source_frame', 'odom').value)
        self._timeout = float(self.declare_parameter('timeout', 60.0).value)
        self._buf = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buf, self)
        self._start = self.get_clock().now()
        self.done = False
        self.get_logger().info(
            f'Waiting for {self._target}→{self._source} before releasing Nav2 '
            f'(timeout {self._timeout:.0f}s)…')

    def check(self):
        """Return True once we should release Nav2 (transform live OR timed out)."""
        try:
            self._buf.lookup_transform(self._target, self._source, rclpy.time.Time())
            self.get_logger().info(
                f'{self._target}→{self._source} live — releasing Nav2.')
            self.done = True
            return True
        except Exception:
            pass
        if (self.get_clock().now() - self._start).nanoseconds / 1e9 > self._timeout:
            self.get_logger().warn(
                f'{self._target}→{self._source} not seen in {self._timeout:.0f}s — '
                'releasing Nav2 anyway.')
            self.done = True
            return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = TfGate()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)   # services the TF listener
            node.check()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

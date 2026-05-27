#!/usr/bin/env python3
"""
Depth Camera Safety Shield — Argo Mini

Pipeline:
    Nav2 controller → /cmd_vel_raw
    velocity_smoother → /cmd_vel_raw  →  /cmd_vel_smoothed
    [this node]       → /cmd_vel_smoothed  →  /cmd_vel
    serial_bridge     → /cmd_vel  →  ESP32

Behaviour:
    CLEAR  (no obstacle within slow_distance) — pass cmd_vel through unchanged
    SLOW   (obstacle between stop_distance and slow_distance) — scale linear.x
    STOP   (obstacle within stop_distance) — zero forward motion (reverse allowed)
    STALE  (no depth frame received in depth_timeout seconds) — pass through (fail-safe)

The node also re-publishes the depth PointCloud2 (downsampled) on /depth_filtered
so Nav2's local costmap can use it as an observation source for proactive planning.
"""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
import tf2_ros


class DepthSafetyShield(Node):

    # ── state constants ──────────────────────────────────────────────────────
    CLEAR = "CLEAR"
    SLOW  = "SLOW"
    STOP  = "STOP"
    STALE = "STALE"

    def __init__(self):
        super().__init__('depth_safety_shield')

        # ── parameters ───────────────────────────────────────────────────────
        self.declare_parameter('stop_distance',       0.70)   # m — hard stop
        self.declare_parameter('slow_distance',       1.0)   # m — begin scaling
        self.declare_parameter('slow_factor',         0.40)   # fraction of linear.x
        self.declare_parameter('lateral_margin',      0.40)   # m — half robot width + buffer
        self.declare_parameter('min_obstacle_height', 0.05)   # m — ignore floor reflections
        self.declare_parameter('max_obstacle_height', 1.60)   # m — ignore overhead structure
        self.declare_parameter('depth_timeout',       3.0)    # s — stale-data window
        self.declare_parameter('downsample_stride',   4)      # process every Nth point row/col
        self.declare_parameter('input_topic',  '/cmd_vel_smoothed')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('depth_topic',
            '/ascamera_hp60c/camera_publisher/depth0/points')

        p = self.get_parameter
        self.stop_dist   = p('stop_distance').value
        self.slow_dist   = p('slow_distance').value
        self.slow_factor = p('slow_factor').value
        self.lat_margin  = p('lateral_margin').value
        self.min_h       = p('min_obstacle_height').value
        self.max_h       = p('max_obstacle_height').value
        self.timeout     = p('depth_timeout').value
        self.stride      = p('downsample_stride').value
        in_topic         = p('input_topic').value
        out_topic        = p('output_topic').value
        depth_topic      = p('depth_topic').value

        # ── TF ───────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── state ────────────────────────────────────────────────────────────
        self.closest_fwd   = math.inf   # m — nearest obstacle in forward zone
        self.last_depth_ts = None       # monotonic time of last depth frame
        self.state         = self.STALE

        # Rate-limit depth processing: track last processed time
        self._last_proc    = 0.0
        self._proc_interval = 0.15      # ~7 Hz processing cap

        # ── publishers / subscribers ─────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, out_topic, 10)

        # Re-publish filtered cloud so Nav2 costmap can use it
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/depth_filtered', rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)

        # Use SENSOR_DATA QoS (best-effort, depth-1) to keep only latest frame
        self.depth_sub = self.create_subscription(
            PointCloud2, depth_topic, self._depth_cb,
            QoSPresetProfiles.SENSOR_DATA.value)

        self.cmd_sub = self.create_subscription(
            Twist, in_topic, self._cmd_cb, 10)

        # Watchdog: mark STALE if no depth arrives
        self.create_timer(1.0, self._watchdog)

        self.get_logger().info(
            f'DepthSafetyShield ready  |  '
            f'stop={self.stop_dist}m  slow={self.slow_dist}m  '
            f'in={in_topic}  out={out_topic}')

    # ── depth callback ────────────────────────────────────────────────────────
    def _depth_cb(self, msg: PointCloud2):
        now = time.monotonic()

        # Rate-limit heavy processing
        if now - self._last_proc < self._proc_interval:
            self.last_depth_ts = now   # still mark as alive
            return
        self._last_proc = now

        # Look up TF from camera frame → base_link
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.05))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=5.0)
            self.last_depth_ts = now
            return

        # Read XYZ points (downsampled by stride).
        # Use read_points with field_names and convert via list comprehension to
        # handle non-packed layouts (e.g. 16-byte stride with 4-byte padding).
        try:
            pts_raw = np.array(
                [(p[0], p[1], p[2])
                 for p in pc2.read_points(
                     msg, field_names=('x', 'y', 'z'), skip_nans=True)],
                dtype=np.float32)
        except Exception as e:
            self.get_logger().warn(f'PointCloud2 read error: {e}', throttle_duration_sec=5.0)
            self.last_depth_ts = now
            return

        self.last_depth_ts = now

        if pts_raw.size == 0:
            self.closest_fwd = math.inf
            self._update_state()
            return

        # Stride-downsample for efficiency
        pts = pts_raw[::self.stride]  # Nx3

        # ── Transform to base_link ────────────────────────────────────────────
        t  = tf_stamped.transform.translation
        r  = tf_stamped.transform.rotation
        qx, qy, qz, qw = r.x, r.y, r.z, r.w

        # Rotation matrix from quaternion (vectorised)
        R = np.array([
            [1 - 2*(qy*qy + qz*qz),   2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw)],
            [    2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz),  2*(qy*qz - qx*qw)],
            [    2*(qx*qz - qy*qw),    2*(qy*qz + qx*qw),  1 - 2*(qx*qx + qy*qy)]
        ], dtype=np.float64)

        pts_bl = (R @ pts.T).T + np.array([t.x, t.y, t.z])  # Nx3 in base_link

        x = pts_bl[:, 0]
        y = pts_bl[:, 1]
        z = pts_bl[:, 2]

        # ── Forward-zone filter ───────────────────────────────────────────────
        # Keep only: in front, within lateral footprint, above floor, below ceiling
        mask = (
            (x > 0.05) &
            (x < self.slow_dist) &
            (np.abs(y) < self.lat_margin) &
            (z > self.min_h) &
            (z < self.max_h)
        )
        fwd_x = x[mask]

        self.closest_fwd = float(np.min(fwd_x)) if fwd_x.size > 0 else math.inf
        self._update_state()

        # Re-publish filtered points (in base_link frame) for costmap
        if fwd_x.size > 0:
            keep = pts_bl[mask]
            header = Header()
            header.stamp    = msg.header.stamp
            header.frame_id = 'base_link'
            filtered_msg = pc2.create_cloud_xyz32(header, keep.tolist())
            self.cloud_pub.publish(filtered_msg)

    # ── cmd_vel callback ──────────────────────────────────────────────────────
    def _cmd_cb(self, msg: Twist):
        out = Twist()
        out.angular.z = msg.angular.z  # angular always passes through

        if self.state == self.STALE:
            # No depth data — fail-safe: pass through unchanged
            out.linear.x = msg.linear.x

        elif self.state == self.STOP:
            # Hard stop forward motion; allow reverse so Nav2 can recover
            out.linear.x = min(msg.linear.x, 0.0)

        elif self.state == self.SLOW:
            # Scale linear to slow_factor, always allow reverse
            if msg.linear.x > 0.0:
                out.linear.x = msg.linear.x * self.slow_factor
            else:
                out.linear.x = msg.linear.x

        else:  # CLEAR
            out.linear.x = msg.linear.x

        self.cmd_pub.publish(out)

    # ── helpers ───────────────────────────────────────────────────────────────
    def _update_state(self):
        d = self.closest_fwd
        prev = self.state

        if d <= self.stop_dist:
            self.state = self.STOP
        elif d <= self.slow_dist:
            self.state = self.SLOW
        else:
            self.state = self.CLEAR

        if self.state != prev:
            self.get_logger().info(
                f'Safety state: {prev} → {self.state}  '
                f'(closest={d:.2f}m)')

    def _watchdog(self):
        if self.last_depth_ts is None:
            if self.state != self.STALE:
                self.get_logger().warn('No depth data yet — safety in STALE (pass-through)')
                self.state = self.STALE
            return

        age = time.monotonic() - self.last_depth_ts
        if age > self.timeout:
            if self.state != self.STALE:
                self.get_logger().warn(
                    f'Depth data stale ({age:.1f}s) — reverting to STALE (pass-through)')
                self.state = self.STALE


def main(args=None):
    rclpy.init(args=args)
    node = DepthSafetyShield()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

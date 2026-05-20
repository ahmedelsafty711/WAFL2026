#!/usr/bin/env python3
"""
tf2_pose_transformer.py  –  Integration guide / drop-in helper
================================================================
Shows how a navigator node should:
  1. Subscribe to /pallet_pose (camera frame).
  2. Transform the pose to the 'map' frame using tf2.
  3. Use the map-frame pose to command Nav2 or publish /cmd_vel.

Drop this file into the pallet_vision package alongside the other nodes
and register it in setup.py under console_scripts if you want to run it
as a standalone node.

Dependencies (add to package.xml)
----------------------------------
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException


class TF2PoseTransformerNode(Node):
    """
    Minimal example: subscribe to /pallet_pose (camera frame) and
    republish it in the 'map' frame as /pallet_pose_map.

    ┌─────────────────────────────────────────────────────────────────┐
    │  HOW tf2 WORKS IN ROS2                                         │
    │                                                                 │
    │  tf2 maintains a tree of coordinate frames.  A transform       │
    │  T_map_camera tells you how to express a point that is in the  │
    │  camera frame as a point in the map frame.                     │
    │                                                                 │
    │  Buffer  – stores recent transforms (sliding window)           │
    │  TransformListener – subscribes to /tf and /tf_static and      │
    │                      fills the Buffer                          │
    │  buffer.transform() – does the actual math                     │
    └─────────────────────────────────────────────────────────────────┘
    """

    TARGET_FRAME = "map"          # frame we want the pose in
    TF_TIMEOUT   = Duration(seconds=0.1)   # how long to wait for a transform

    def __init__(self) -> None:
        super().__init__("tf2_pose_transformer")

        # ── tf2 setup ──────────────────────────────────────────────
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── Subscriber / Publisher ─────────────────────────────────
        self.create_subscription(
            PoseStamped, "/pallet_pose", self._on_pallet_pose, 10
        )
        self._pub = self.create_publisher(
            PoseStamped, "/pallet_pose_map", 10
        )

        self.get_logger().info("TF2PoseTransformerNode ready.")

    def _on_pallet_pose(self, msg: PoseStamped) -> None:
        """
        Transform incoming pose from its source frame to 'map'.

        Steps
        -----
        1.  msg.header.frame_id tells us the source frame (e.g. 'camera_link').
        2.  buffer.transform() looks up T_map_camera at msg.header.stamp
            (or the closest available time if timeout allows) and applies it.
        3.  The resulting PoseStamped has header.frame_id == 'map'.

        Notes on timing
        ---------------
        Using msg.header.stamp (the acquisition time) is most accurate but
        requires that the tf tree has data for that exact time.  If the
        camera driver timestamps slightly in the past, increase TF_TIMEOUT.
        Alternatively, use rclpy.time.Time() (= latest available transform)
        for robustness at the cost of a small temporal inconsistency.
        """
        try:
            pose_in_map: PoseStamped = self._tf_buffer.transform(
                msg,
                self.TARGET_FRAME,
                timeout=self.TF_TIMEOUT,
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Cannot transform {msg.header.frame_id} → {self.TARGET_FRAME}: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        self._pub.publish(pose_in_map)

        # ── At this point you can pass pose_in_map directly to Nav2 ──
        # goal = NavigateToPose.Goal()
        # goal.pose = pose_in_map
        # self._nav_client.send_goal_async(goal)
        #
        # Or compute a velocity command for direct /cmd_vel control:
        # dx = pose_in_map.pose.position.x - robot_x
        # dy = pose_in_map.pose.position.y - robot_y
        # ... PID / pure-pursuit logic here ...


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TF2PoseTransformerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

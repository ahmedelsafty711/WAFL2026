#!/usr/bin/env python3
"""
ROS2 Node: pallet_navigator
============================
State machine that:
  1. Waits in IDLE until a pallet marker is detected (/pallet_location_id).
  2. Looks up the destination in ID_TO_LOCATION.
  3. Waits for the /pallet_lifted signal (Bool=True).
  4. Sends a NavigateToPose goal to Nav2 and monitors it.
  5. Resets to IDLE after delivery (or on failure).

Integration note
----------------
For production use, replace the hardcoded ID_TO_LOCATION table with a
parameter loaded from a YAML file so the warehouse layout can be changed
without recompiling.  See _load_location_table() for the hook.
"""

from __future__ import annotations

import math
from enum import Enum, auto
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Bool, String


# ─────────────────────────────────────────────────────────────────────────────
# Location DB  (move to a YAML parameter for production)
# ─────────────────────────────────────────────────────────────────────────────

ID_TO_LOCATION: dict[int, dict] = {
    0: {"name": "Location_A1", "x":  5.0, "y": 2.0, "yaw": 0.0},
    1: {"name": "Location_A2", "x":  5.0, "y": 5.5, "yaw": 0.0},
    2: {"name": "Location_B1", "x": 12.0, "y": 2.0, "yaw": math.pi / 2},
    3: {"name": "Location_B2", "x": 12.0, "y": 5.5, "yaw": math.pi / 2},
}


# ─────────────────────────────────────────────────────────────────────────────
# State machine
# ─────────────────────────────────────────────────────────────────────────────

class State(Enum):
    IDLE        = auto()   # waiting for a marker detection
    MARKER_READ = auto()   # marker seen, waiting for pallet lifted signal
    NAVIGATING  = auto()   # Nav2 goal in flight


# ─────────────────────────────────────────────────────────────────────────────
# Node
# ─────────────────────────────────────────────────────────────────────────────

class PalletNavigator(Node):
    """
    Listens for pallet detections and autonomously navigates to deliver them.

    State transitions
    -----------------
    IDLE  --[marker detected]--> MARKER_READ
    MARKER_READ --[pallet lifted]--> NAVIGATING
    NAVIGATING  --[arrived / error]--> IDLE
    """

    def __init__(self) -> None:
        super().__init__("pallet_navigator")

        self._state: State = State.IDLE
        self._destination: Optional[dict] = None
        self._goal_handle: Optional[ClientGoalHandle] = None

        # Action client – Nav2
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Subscribers
        reliable_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String, "/pallet_location_id", self._on_detection, reliable_qos
        )
        self.create_subscription(
            Bool, "/pallet_lifted", self._on_lifted, reliable_qos
        )

        self.get_logger().info("PalletNavigator ready – state: IDLE")

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _on_detection(self, msg: String) -> None:
        """Handle a new pallet detection.  Only acts when IDLE."""
        if self._state != State.IDLE:
            return

        try:
            mid = int(msg.data)
        except ValueError:
            self.get_logger().warn(f"Could not parse marker ID: '{msg.data}'")
            return

        loc = ID_TO_LOCATION.get(mid)
        if loc is None:
            self.get_logger().warn(f"Marker ID {mid} not in location DB – ignoring.")
            return

        self._destination = loc
        self._state       = State.MARKER_READ
        self.get_logger().info(
            f"Marker {mid} → destination '{loc['name']}'. "
            "Waiting for /pallet_lifted …"
        )

    def _on_lifted(self, msg: Bool) -> None:
        """Start navigation once the pallet has been confirmed lifted."""
        if not msg.data or self._state != State.MARKER_READ:
            return

        self._state = State.NAVIGATING
        self.get_logger().info("Pallet lifted – sending Nav2 goal …")
        self._send_nav_goal(self._destination)

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------

    def _send_nav_goal(self, loc: dict) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server unavailable – resetting.")
            self._reset()
            return

        yaw = float(loc["yaw"])
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(loc["x"])
        pose.pose.position.y = float(loc["y"])
        pose.pose.position.z = 0.0
        # Encode yaw-only rotation as a quaternion: q = (cos(yaw/2), 0, 0, sin(yaw/2))
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        pose.pose.orientation.z = math.sin(yaw / 2.0)

        goal = NavigateToPose.Goal()
        goal.pose = pose

        send_future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        send_future.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future) -> None:
        handle: ClientGoalHandle = future.result()
        if not handle.accepted:
            self.get_logger().error("Nav2 rejected the goal – resetting.")
            self._reset()
            return

        self._goal_handle = handle
        self.get_logger().info("Nav2 accepted goal – forklift moving …")
        handle.get_result_async().add_done_callback(self._on_arrived)

    def _on_arrived(self, future) -> None:
        result = future.result()
        # NavigateToPose result has no payload; any non-exception means success.
        self.get_logger().info(
            f"Forklift arrived at '{self._destination['name']}' – delivery complete!"
        )
        self._reset()

    def _on_feedback(self, feedback_msg) -> None:
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(
            f"Distance remaining: {dist:.2f} m",
            throttle_duration_sec=2.0,
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _reset(self) -> None:
        self._destination  = None
        self._goal_handle  = None
        self._state        = State.IDLE
        self.get_logger().info("State reset → IDLE")


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = PalletNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

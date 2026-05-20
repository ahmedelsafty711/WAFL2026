"""
pallet_vision.launch.py
========================
Launches pallet_pose_node and pallet_navigator together.

Camera intrinsics should be overridden via a separate YAML file in
production; the values here are placeholder defaults for a 640×480 camera.

Usage
-----
  ros2 launch pallet_vision pallet_vision.launch.py

Override camera matrix at launch time:
  ros2 launch pallet_vision pallet_vision.launch.py \
      camera_matrix:=[fx,0,cx,0,fy,cy,0,0,1]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ── Declare overridable arguments ──────────────────────────────────
    args = [
        DeclareLaunchArgument("image_topic",   default_value="/image_raw"),
        DeclareLaunchArgument("camera_frame",  default_value="camera_link"),
        DeclareLaunchArgument("marker_length", default_value="0.105"),
    ]

    # ── Nodes ──────────────────────────────────────────────────────────
    pose_node = Node(
        package="pallet_vision",
        executable="pallet_pose_node",
        name="pallet_pose_node",
        output="screen",
        parameters=[{
            "image_topic":   LaunchConfiguration("image_topic"),
            "camera_frame":  LaunchConfiguration("camera_frame"),
            "marker_length": LaunchConfiguration("marker_length"),
            # Real calibration: replace with a loaded YAML or pass on CLI
            "camera_matrix": [600.0, 0.0, 320.0,
                                0.0, 600.0, 240.0,
                                0.0,   0.0,   1.0],
            "dist_coeffs":   [0.0, 0.0, 0.0, 0.0, 0.0],
        }],
    )

    navigator_node = Node(
        package="pallet_vision",
        executable="pallet_navigator",
        name="pallet_navigator",
        output="screen",
    )

    return LaunchDescription(args + [pose_node, navigator_node])

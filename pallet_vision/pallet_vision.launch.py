"""
pallet_vision.launch.py
========================
Launches pallet_pose_node and pallet_navigator together.

Configured for **Kinect v1 RGB camera** (640×480, plumb_bob distortion).
Calibration source: real kinect_rgb calibration file.

Kinect v1 ROS driver topics (freenect / openni2):
  Image : /camera/rgb/image_raw   (or /camera/rgb/image_color)
  Frame : camera_rgb_optical_frame

Usage
-----
  ros2 launch pallet_vision pallet_vision.launch.py

Switch back to a webcam at launch time without editing this file:
  ros2 launch pallet_vision pallet_vision.launch.py \
      image_topic:=/webcam/image_raw \
      camera_frame:=camera_link

Override marker size (metres):
  ros2 launch pallet_vision pallet_vision.launch.py marker_length:=0.105
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ── Kinect v1 RGB calibration ─────────────────────────────────────────────
#
#   Resolution : 640 × 480
#   Model      : plumb_bob  (radial k1, k2 + tangential p1, p2, k3)
#
#   Camera matrix (3×3, row-major):
#       fx    0   cx        526.607  0        318.525
#        0   fy   cy    =     0      526.607  241.181
#        0    0    1           0       0        1
#
#   Distortion vector [k1, k2, p1, p2, k3]:
#       [0.10265184, -0.18646532, 0.0, 0.0, 0.0]
#
# ─────────────────────────────────────────────────────────────────────────

KINECT_RGB_MATRIX = [
    526.60717328,   0.0,          318.52510740,
      0.0,        526.60717328,   241.18145973,
      0.0,          0.0,            1.0,
]

KINECT_RGB_DIST = [0.10265184, -0.18646532, 0.0, 0.0, 0.0]


def generate_launch_description() -> LaunchDescription:

    # ── Overridable CLI arguments ──────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            "image_topic",
            default_value="/image_raw",              # kinect_ros2 default
            description="Image topic published by the camera driver",
        ),
        DeclareLaunchArgument(
            "camera_frame",
            default_value="camera_rgb_optical_frame",  # kinect_ros2 default (verify with: ros2 topic echo /image_raw --field header.frame_id --once)
            description="TF frame attached to the camera",
        ),
        DeclareLaunchArgument(
            "marker_length",
            default_value="0.105",   # 105 mm – measure your printed marker!
            description="Physical side length of the ArUco marker in metres",
        ),
    ]

    # ── pallet_pose_node ──────────────────────────────────────────────
    pose_node = Node(
        package="pallet_vision",
        executable="pallet_pose_node",
        name="pallet_pose_node",
        output="screen",
        parameters=[{
            "image_topic":   LaunchConfiguration("image_topic"),
            "camera_frame":  LaunchConfiguration("camera_frame"),
            "marker_length": LaunchConfiguration("marker_length"),
            "camera_matrix": KINECT_RGB_MATRIX,
            "dist_coeffs":   KINECT_RGB_DIST,
            "fork_offset_m": 0.5,
        }],
    )

    # ── pallet_navigator ──────────────────────────────────────────────
    navigator_node = Node(
        package="pallet_vision",
        executable="pallet_navigator",
        name="pallet_navigator",
        output="screen",
    )

    return LaunchDescription(args + [pose_node, navigator_node])

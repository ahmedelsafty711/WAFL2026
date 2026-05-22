#!/usr/bin/env python3
"""
Pallet Pose Estimation via YOLO Keypoints + solvePnP
=====================================================
Detects 4 corner keypoints of a wooden pallet using a YOLO pose model,
then computes 3D pose (R, t) with cv2.solvePnP.

Pallet real-world dimensions : 68 cm × 13 cm
Keypoint order from the model :
    1 → Top-Right
    2 → Top-Left
    3 → Bottom-Left
    4 → Bottom-Right
"""

import cv2
import numpy as np
from ultralytics import YOLO

# ─────────────────────────────────────────────────────────────────────────────
# 1.  CAMERA INTRINSICS  ← paste your own values here
# ─────────────────────────────────────────────────────────────────────────────
# Replace the values below with your actual webcam calibration.
# Format: [fx,  0, cx,
#           0, fy, cy,
#           0,  0,  1]
CAMERA_MATRIX = np.array([
    [600.0,   0.0, 320.0],
    [  0.0, 600.0, 240.0],
    [  0.0,   0.0,   1.0],
], dtype=np.float64)

DIST_COEFFS = np.zeros((5, 1), dtype=np.float64)   # set your distortion if known


# ─────────────────────────────────────────────────────────────────────────────
# 2.  PALLET 3-D OBJECT POINTS  (in metres, Z = 0 plane)
# ─────────────────────────────────────────────────────────────────────────────
# Origin at the pallet centre.
# X-axis points right, Y-axis points down (image convention).
#
#   2 (TL) ──────── 1 (TR)
#      |                |
#   3 (BL) ──────── 4 (BR)
#
W = 0.68 / 2   # half-width  = 34 cm
H = 0.13 / 2   # half-height = 6.5 cm

OBJECT_POINTS = np.array([
    [ W, -H, 0.0],   # 1 – Top-Right
    [-W, -H, 0.0],   # 2 – Top-Left
    [-W,  H, 0.0],   # 3 – Bottom-Left
    [ W,  H, 0.0],   # 4 – Bottom-Right
], dtype=np.float64)


# ─────────────────────────────────────────────────────────────────────────────
# 3.  HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def extract_pallet_keypoints(results) -> np.ndarray | None:
    """
    Pull the 4 pallet corner keypoints from a YOLO pose result.
    Returns an (4, 2) float64 array ordered [TR, TL, BL, BR],
    or None if fewer than 4 confident keypoints are found.
    """
    if results[0].keypoints is None:
        return None

    kp_xy   = results[0].keypoints.xy[0].cpu().numpy()    # (N, 2)
    kp_conf = results[0].keypoints.conf[0].cpu().numpy()  # (N,)

    if len(kp_xy) < 4:
        return None

    # The model already outputs exactly 4 keypoints in order [TR, TL, BL, BR].
    # We just validate that all 4 have sufficient confidence.
    CONF_THRESHOLD = 0.10
    valid = kp_conf >= CONF_THRESHOLD

    if valid.sum() < 4:
        print(f"  ⚠  Only {valid.sum()} keypoints above confidence threshold.")
        return None

    # Take the first 4 in model order (TR=0, TL=1, BL=2, BR=3)
    return kp_xy[:4].astype(np.float64)


def solve_pallet_pose(image_points: np.ndarray):
    """
    Run solvePnP and return (rvec, tvec, R, euler_deg) or None on failure.
    image_points : (4, 2) array in pixel coords, ordered [TR, TL, BL, BR]
    """
    ok, rvec, tvec = cv2.solvePnP(
        OBJECT_POINTS,
        image_points,
        CAMERA_MATRIX,
        DIST_COEFFS,
        flags=cv2.SOLVEPNP_IPPE,   # IPPE is optimal for planar targets
    )
    if not ok:
        return None

    R, _ = cv2.Rodrigues(rvec)

    # Euler angles (ZYX / yaw-pitch-roll) in degrees
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6
    if not singular:
        roll  = np.degrees(np.arctan2( R[2, 1],  R[2, 2]))
        pitch = np.degrees(np.arctan2(-R[2, 0],  sy))
        yaw   = np.degrees(np.arctan2( R[1, 0],  R[0, 0]))
    else:
        roll  = np.degrees(np.arctan2(-R[1, 2],  R[1, 1]))
        pitch = np.degrees(np.arctan2(-R[2, 0],  sy))
        yaw   = 0.0

    return rvec, tvec, R, np.array([roll, pitch, yaw])


def draw_pose_axes(image, rvec, tvec, axis_length=0.10):
    """Draw XYZ axes on the pallet centre (axis_length in metres)."""
    axis_pts = np.float32([
        [0, 0, 0],                    # origin
        [axis_length, 0, 0],          # X → red
        [0, axis_length, 0],          # Y → green
        [0, 0, -axis_length],         # Z → blue  (out of pallet towards camera)
    ])
    projected, _ = cv2.projectPoints(axis_pts, rvec, tvec,
                                     CAMERA_MATRIX, DIST_COEFFS)
    projected = projected.reshape(-1, 2).astype(int)

    origin = tuple(projected[0])
    cv2.arrowedLine(image, origin, tuple(projected[1]), (0, 0, 255), 2, tipLength=0.2)  # X red
    cv2.arrowedLine(image, origin, tuple(projected[2]), (0, 255, 0), 2, tipLength=0.2)  # Y green
    cv2.arrowedLine(image, origin, tuple(projected[3]), (255, 0, 0), 2, tipLength=0.2)  # Z blue


def draw_keypoints_and_box(image, kpts: np.ndarray):
    """Draw the 4 keypoints and the bounding quadrilateral."""
    labels = ["TR", "TL", "BL", "BR"]
    colors = [(0, 0, 255), (0, 255, 255), (0, 255, 0), (255, 0, 255)]

    pts_int = kpts.astype(int)

    # Draw edges of the pallet rectangle
    order = [0, 1, 2, 3, 0]
    for i in range(4):
        cv2.line(image, tuple(pts_int[order[i]]), tuple(pts_int[order[i+1]]),
                 (255, 255, 0), 2)

    # Draw keypoints
    for i, (pt, lbl, col) in enumerate(zip(pts_int, labels, colors)):
        cv2.circle(image, tuple(pt), 7, col, -1)
        cv2.putText(image, f"{i+1}:{lbl}", (pt[0]+8, pt[1]-8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)


def overlay_pose_text(image, tvec, euler_deg):
    """Print translation + orientation in the top-left corner."""
    tx, ty, tz = tvec.flatten()
    roll, pitch, yaw = euler_deg

    lines = [
        f"X={tx*100:+.1f} cm   Y={ty*100:+.1f} cm   Z={tz*100:.1f} cm",
        f"Roll={roll:+.1f} deg   Pitch={pitch:+.1f} deg   Yaw={yaw:+.1f} deg",
    ]
    for i, line in enumerate(lines):
        y = 30 + i * 28
        cv2.putText(image, line, (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 4)
        cv2.putText(image, line, (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)


# ─────────────────────────────────────────────────────────────────────────────
# 4.  MAIN
# ─────────────────────────────────────────────────────────────────────────────

def process_image(image_path: str, model_path: str = "models/best_pose12.pt"):
    model  = YOLO(model_path)
    image  = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"Cannot read image: {image_path}")

    output = image.copy()

    # ── Run inference ──
    results = model(image, conf=0.10)

    if len(results[0].keypoints.xy) == 0:
        print("No pallet detected.")
        cv2.imshow("Pallet Pose", output)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return

    # ── Extract keypoints ──
    kpts = extract_pallet_keypoints(results)
    if kpts is None:
        print("Could not extract 4 valid keypoints.")
        return

    draw_keypoints_and_box(output, kpts)

    # ── Solve PnP ──
    pose = solve_pallet_pose(kpts)
    if pose is None:
        print("solvePnP failed.")
        return

    rvec, tvec, R, euler_deg = pose
    tx, ty, tz = tvec.flatten()
    roll, pitch, yaw = euler_deg

    # ── Console output ──
    print("\n" + "="*55)
    print("  PALLET POSE ESTIMATE")
    print("="*55)
    print(f"  Translation  X = {tx*100:+.2f} cm")
    print(f"               Y = {ty*100:+.2f} cm")
    print(f"               Z = {tz*100:.2f} cm  (distance to camera)")
    print(f"  Rotation  Roll  = {roll:+.2f}°")
    print(f"            Pitch = {pitch:+.2f}°")
    print(f"            Yaw   = {yaw:+.2f}°")
    print("-"*55)
    print("  Rotation Matrix R:\n", np.round(R, 4))
    print("="*55 + "\n")

    # ── Draw result ──
    draw_pose_axes(output, rvec, tvec)
    overlay_pose_text(output, tvec, euler_deg)

    cv2.imshow("Pallet Pose", output)
    cv2.imwrite("pallet_pose_result.jpg", output)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def process_webcam(model_path: str = "models/best_pose12.pt", camera_index: int = 0):
    """Live webcam version."""
    model = YOLO(model_path)
    cap   = cv2.VideoCapture(camera_index)

    print("Press 'q' to quit.")
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        output  = frame.copy()
        results = model(frame, conf=0.10, verbose=False)

        if (results[0].keypoints is not None
                and len(results[0].keypoints.xy) > 0):

            kpts = extract_pallet_keypoints(results)
            if kpts is not None:
                draw_keypoints_and_box(output, kpts)
                pose = solve_pallet_pose(kpts)
                if pose is not None:
                    rvec, tvec, R, euler_deg = pose
                    draw_pose_axes(output, rvec, tvec)
                    overlay_pose_text(output, tvec, euler_deg)

        cv2.imshow("Pallet Pose – Live", output)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # ── Choose one ──────────────────────────────────────────────────────────
    process_image("image/test.jpg")   # single image
    # process_webcam()                # live webcam

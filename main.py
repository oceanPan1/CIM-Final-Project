import cv2
from apriltag_detector import AprilTagDetector
from controller import PoseController
from smoother import EMAFilter
import numpy as np

TAG_SIZE_M      = 0.03
TARGET_DIST_M   = 1.0    
CAMERA_INDEX    = 0       


def draw_overlay(frame, pose, command):
    for i, corner in enumerate(pose['corners']):
        pt = tuple(corner.astype(int))
        cv2.circle(frame, pt, 5, (0, 255, 0), -1)

    cx = int(pose['corners'][:, 0].mean())
    cy = int(pose['corners'][:, 1].mean())

    lines = [
        f"ID: {pose['id']}",
        f"Z: {command['z_dist_m']:.3f}m",
        f"X: {command['x_offset_m']:.3f}m",
        f"Y: {command['y_offset_m']:.3f}m",  # add this
        f"Yaw: {command['yaw_deg']:.1f}deg",
        f"{command['rotation']}",
        f"{command['movement']}",
    ]

    for i, line in enumerate(lines):
        cv2.putText(frame, line, (cx - 60, cy - 80 + i * 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)


def main():
    cap_temp = cv2.VideoCapture(CAMERA_INDEX)
    w = cap_temp.get(cv2.CAP_PROP_FRAME_WIDTH)
    h = cap_temp.get(cv2.CAP_PROP_FRAME_HEIGHT)
    cap_temp.release()

    fx = fy = w  # good enough approximation for testing
    cam_matrix = np.array([
        [fx,  0, w/2],
        [ 0, fy, h/2],
        [ 0,  0,   1]
    ], dtype=np.float64)
    dist_coeffs = np.zeros((4, 1))
    
    detector   = AprilTagDetector(cam_matrix, dist_coeffs, TAG_SIZE_M)
    controller = PoseController(target_distance_m=TARGET_DIST_M)
    smoother   = EMAFilter(alpha=0.3)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera.")

    print("Running — press Q to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        detections = detector.detect(frame)

        for pose in detections:
            smoothed = smoother.smooth_pose(pose)
            command  = controller.interpret(smoothed)
            draw_overlay(frame, smoothed, command)
            print(f"Z: {command['z_dist_m']:.3f} | X: {command['x_offset_m']:.4f} | Y: {command['y_offset_m']:.4f} | Yaw: {command['yaw_deg']:.1f} | {command['rotation']} | {command['movement']}")

        cv2.imshow("PM Sensor", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
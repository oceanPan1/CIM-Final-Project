import cv2
import numpy as np
from pupil_apriltags import Detector

class AprilTagDetector:
    def __init__(self, cam_matrix, dist_coeffs, tag_size_m: float):
        """
        tag_size_m: physical size of the tag in meters,
                    measured edge-to-edge between the black/white border.
        """
        self.cam_matrix = cam_matrix
        self.dist_coeffs = dist_coeffs
        self.tag_size = tag_size_m
        self.detector = Detector(families="tagStandard41h12")

        # 3D object points for the tag corners (tag-centered, Z=0 plane)
        # Order: top-left, top-right, bottom-right, bottom-left
        half = tag_size_m / 2.0
        self.obj_points = np.array([
            [-half, -half, 0],
            [ half, -half, 0],
            [ half,  half, 0],
            [-half,  half, 0],
        ], dtype=np.float64)

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        fx = self.cam_matrix[0, 0]
        fy = self.cam_matrix[1, 1]
        cx = self.cam_matrix[0, 2]
        cy = self.cam_matrix[1, 2]

        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(fx, fy, cx, cy),
            tag_size=self.tag_size
        )
        results = []

        for det in detections:
            if det.pose_t is None:
                continue

            tvec = det.pose_t  # shape (3,1)
            rvec, _ = cv2.Rodrigues(det.pose_R)

            x_offset = tvec[0][0]
            y_offset = tvec[1][0]
            z_dist   = tvec[2][0]

            rot_mat = det.pose_R
            yaw_rad = np.arctan2(rot_mat[1, 0], rot_mat[0, 0])
            yaw_deg = np.degrees(yaw_rad)

            corners = det.corners
            img_points = np.array([
                corners[0], corners[1], corners[2], corners[3]
            ], dtype=np.float64)

            results.append({
                'id':       det.tag_id,
                'tvec':     tvec,
                'rvec':     rvec,
                'x_offset': x_offset,
                'y_offset': y_offset,
                'z_dist':   z_dist,
                'yaw_deg':  yaw_deg,
                'corners':  img_points,
            })

        return results
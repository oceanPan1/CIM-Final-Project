import cv2
import numpy as np
import glob

def calibrate_camera(images_glob: str, board_size=(9, 6), square_size_m=0.025):
    """
    Run once with a set of chessboard images to get camera intrinsics.
    images_glob: e.g. 'calib_images/*.jpg'
    Returns: (camera_matrix, dist_coeffs) or None if calibration fails.
    """
    obj_points = []  # 3D points in real world
    img_points = []  # 2D points in image plane

    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
    objp *= square_size_m

    images = glob.glob(images_glob)
    if not images:
        raise FileNotFoundError(f"No images found at {images_glob}")

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, board_size, None)
        if ret:
            obj_points.append(objp)
            img_points.append(corners)

    if not obj_points:
        print("No valid chessboard detections found.")
        return None, None

    h, w = gray.shape
    ret, cam_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
        obj_points, img_points, (w, h), None, None
    )
    print(f"Calibration RMS error: {ret:.4f}")
    print(f"fx={cam_matrix[0,0]:.2f}, fy={cam_matrix[1,1]:.2f}, "
          f"cx={cam_matrix[0,2]:.2f}, cy={cam_matrix[1,2]:.2f}")
    np.save("camera_matrix.npy", cam_matrix)
    np.save("dist_coeffs.npy", dist_coeffs)
    return cam_matrix, dist_coeffs


def load_calibration():
    """Load saved calibration from disk."""
    cam_matrix = np.load("camera_matrix.npy")
    dist_coeffs = np.load("dist_coeffs.npy")
    return cam_matrix, dist_coeffs
import glob
import cv2
import numpy as np
import cv2.aruco as aruco
import depthai as dai
from pathlib import Path

def detect_markers_corners(frame):
    # Use new OpenCV 4.7+ API
    arucoParams = cv2.aruco.DetectorParameters()
    refine_params = cv2.aruco.RefineParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dictionary, arucoParams, refine_params)
    marker_corners, ids, rejectedImgPoints = detector.detectMarkers(frame)

    # refineDetectedMarkers is now a method of ArucoDetector
    marker_corners, ids, refusd, recoverd = detector.refineDetectedMarkers(
        frame, charuco_board, marker_corners, ids, rejectedImgPoints
    )

    if marker_corners is None or len(marker_corners) <= 0:
        return marker_corners, ids, None, None

    # Use CharucoDetector for interpolation
    charuco_detector = cv2.aruco.CharucoDetector(charuco_board)
    # detectBoard returns (charucoCorners, charucoIds, markerCorners, markerIds)
    charuco_corners, charuco_ids, marker_corners_out, marker_ids_out = charuco_detector.detectBoard(
        frame, markerCorners=marker_corners, markerIds=ids
    )

    if charuco_corners is not None and len(charuco_corners) > 0:
        return marker_corners_out, marker_ids_out, charuco_corners, charuco_ids
    else:
        return marker_corners, ids, None, None

size = (1920, 1200)

# Load the images
calibration_handler = dai.CalibrationHandler('./resources/1844301021621CF500_05_26_23_16_31.json')
images = glob.glob('./dataset_cam7_may_18_merged/left/*.png')

# Prepare for camera calibration
k = np.array(calibration_handler.getCameraIntrinsics(calibration_handler.getStereoLeftCameraId(), size[0], size[1]))
D_left = np.array(calibration_handler.getDistortionCoefficients(calibration_handler.getStereoLeftCameraId()))
d = np.array(calibration_handler.getDistortionCoefficients(calibration_handler.getStereoLeftCameraId()))
r = np.eye(3, dtype=np.float32)

d_zero = np.zeros((14, 1), dtype=np.float32)
M_focal = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(k, d, size, np.eye(3), fov_scale=1.1)
mapXL, mapYL = cv2.fisheye.initUndistortRectifyMap(k, d[:4], r, M_focal, size, cv2.CV_32FC1)

aruco_dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
charuco_board = aruco.CharucoBoard(
                            (11, 8),
                            6.0,
                            4.6,
                            aruco_dictionary)
checkCorners3D = charuco_board.getChessboardCorners()
rvec = np.array([[0.0],
                 [0.0],
                 [0.0]], dtype=np.float32)

tvec = np.array([[0.0],
                 [0.0],
                 [0.0]], dtype=np.float32)
# count = 0
for im_name in images:
    ## Undistort it first
    path = Path(im_name)
    img = cv2.imread(im_name)
    img_rect = cv2.remap(img , mapXL, mapYL, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    marker_corners, ids, charuco_corners, charuco_ids = detect_markers_corners(img_rect)
    if charuco_corners is not None and charuco_ids is not None:
        # Use solvePnP instead of deprecated estimatePoseCharucoBoard
        objPoints = charuco_board.getChessboardCorners()[charuco_ids.flatten()]
        ret, rvec, tvec = cv2.solvePnP(objPoints, charuco_corners, M_focal, d_zero, rvec, tvec, useExtrinsicGuess=True)
    else:
        ret = False

    """ if len(charuco_ids) != len(checkCorners3D):
        print('charuco_ids shape -> ', charuco_ids.shape)
        print('charuco_corners shape -> ', charuco_corners.shape)
        print('checkCorners3D shape -> ', checkCorners3D.shape)
        raise ValueError("Number of ids does not match number of original corners") """

    points_2d, _ = cv2.fisheye.projectPoints(checkCorners3D[None], rvec, tvec, k, d)
    undistorted_points_2d = cv2.fisheye.undistortPoints(points_2d, k, d, R = r, P = M_focal)
    # print("undistorted_points_2d shape -> ", undistorted_points_2d.shape)
    # print("charuco_corners shape -> ", charuco_corners.shape)
    error = 0
    for i in range(len(charuco_ids)):
        error_vec = charuco_corners[i][0] - undistorted_points_2d[0][charuco_ids[i]]
        error += np.linalg.norm(error_vec)
    # print(im_name)
    print(f'Reprojection error is {error / len(charuco_ids)} avg of {len(charuco_ids)} of points in file {path.name} ')



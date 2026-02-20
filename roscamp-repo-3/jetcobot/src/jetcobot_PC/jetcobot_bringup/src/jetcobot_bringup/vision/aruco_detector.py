"""
ArUco marker detection and pose estimation
"""
import cv2
import numpy as np
from rclpy.node import Node


# ArUco dictionary constant
ARUCO_DICT = cv2.aruco.DICT_4X4_50


class ArucoDetector:
    """ArUco marker detector with pose estimation"""
    
    def __init__(self, camera_matrix, distortion_coeffs, marker_size_m, logger=None):
        """
        Initialize ArUco detector
        
        Args:
            camera_matrix: 3x3 camera intrinsic matrix
            distortion_coeffs: Camera distortion coefficients
            marker_size_m: Marker side length in meters
            logger: ROS logger for debugging
        """
        self.camera_matrix = camera_matrix
        self.distortion_coeffs = distortion_coeffs
        self.marker_size_m = marker_size_m
        self.logger = logger
        
        # Initialize ArUco dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        
    def detect(self, cv_image):
        """
        Detect ArUco markers in image
        
        Args:
            cv_image: OpenCV grayscale image
            
        Returns:
            tuple: (corners, ids, rvecs, tvecs) or None if no markers detected
        """
        # Undistort image
        h, w = cv_image.shape[:2]
        newK, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, 
            self.distortion_coeffs, 
            (w, h), 
            1.0, 
            (w, h)
        )
        gray = cv2.undistort(
            cv_image, 
            self.camera_matrix, 
            self.distortion_coeffs, 
            None, 
            newK
        )
        
        # Detect ArUco markers
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            # OpenCV 4.6 이하
            parameters = cv2.aruco.DetectorParameters_create()
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, 
                self.aruco_dict, 
                parameters=parameters
            )
        else:
            # OpenCV 4.7 이상
            parameters = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, parameters)
            corners, ids, rejected = detector.detectMarkers(gray)
        
        if ids is None or len(ids) == 0:
            if self.logger:
                self.logger.warn("❌ No ArUco markers detected")
            return None
        
        if self.logger:
            self.logger.info(f"✅ Detected {len(ids)} marker(s): {ids.flatten().tolist()}")
        
        # Estimate pose of markers
        rvecs, tvecs = self._estimate_poses(corners, newK)
        
        return corners, ids, rvecs, tvecs
    
    def _estimate_poses(self, corners, camera_matrix):
        """
        Estimate pose of detected markers
        
        Args:
            corners: Detected marker corners
            camera_matrix: Camera matrix (possibly optimized)
            
        Returns:
            tuple: (rvecs, tvecs) rotation and translation vectors
        """
        if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, 
                self.marker_size_m, 
                camera_matrix, 
                None
            )
        else:
            rvecs = []
            tvecs = []
            for corner in corners:
                obj_points = np.array([
                    [-self.marker_size_m/2,  self.marker_size_m/2, 0],
                    [ self.marker_size_m/2,  self.marker_size_m/2, 0],
                    [ self.marker_size_m/2, -self.marker_size_m/2, 0],
                    [-self.marker_size_m/2, -self.marker_size_m/2, 0]
                ], dtype=np.float32)
                success, rvec, tvec = cv2.solvePnP(
                    obj_points, 
                    corner, 
                    camera_matrix, 
                    None
                )
                rvecs.append(rvec)
                tvecs.append(tvec)
            rvecs = np.array(rvecs)
            tvecs = np.array(tvecs)
        
        return rvecs, tvecs
    
    def get_marker_axes(self, rvec):
        """
        Get marker coordinate axes from rotation vector
        
        Args:
            rvec: Rotation vector
            
        Returns:
            tuple: (x_axis, y_axis, z_axis) marker axes in camera frame
        """
        R_cam_to_marker, _ = cv2.Rodrigues(rvec)
        
        marker_x_axis = R_cam_to_marker[:, 0]  # Marker's +X axis
        marker_y_axis = R_cam_to_marker[:, 1]  # Marker's +Y axis
        marker_z_axis = R_cam_to_marker[:, 2]  # Marker's +Z axis (toward camera)
        
        return marker_x_axis, marker_y_axis, marker_z_axis

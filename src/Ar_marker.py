#!/usr/bin/env python3
import cv2
import numpy as np
import cv2.aruco as aruco
import logging
from typing import Dict, List, Tuple, Optional
import math

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ArUcoDetector:
    def __init__(self, camera_matrix=None, dist_coeffs=None, marker_size=0.068):
        """
        Initialize ArUco detector
        
        Args:
            camera_matrix: Camera intrinsic matrix (3x3)
            dist_coeffs: Camera distortion coefficients
            marker_size: Physical size of markers in meters
        """
        # ArUco dictionary - using 4x4 markers with 50 unique IDs
        # Support both old and new OpenCV versions
        try:
            # New OpenCV (4.7+)
            self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
            self.aruco_params = aruco.DetectorParameters()
        except AttributeError:
            # Old OpenCV
            self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
            self.aruco_params = aruco.DetectorParameters_create()
        
        # Camera parameters (use defaults if not provided)
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.marker_size = marker_size
        
        # Storage for detected markers
        self.detected_markers = {}
        
        # Define marker IDs for your setup
        self.BASEFRAME_ID = 0  # Marker on baseframe
        self.BOX_MARKERS = {
            1: "Box_Top",
            2: "Box_Front", 
            3: "Box_Left",
            4: "Box_Right",
            5: "Box_Back"
        }
        
        logger.info("ArUco detector initialized")
    
    def detect_markers(self, image: np.ndarray) -> Dict:
        """
        Detect ArUco markers in image
        
        Returns:
            Dictionary with marker info including positions and orientations
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Detect markers
        try:
            # Try new OpenCV API first
            detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, rejected = detector.detectMarkers(gray)
        except AttributeError:
            # Fall back to old API
            corners, ids, rejected = aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )
        
        detected_info = {}
        
        if ids is not None:
            # Process each detected marker
            for i, marker_id in enumerate(ids.flatten()):
                marker_corners = corners[i][0]
                
                # Calculate center point
                center = np.mean(marker_corners, axis=0)
                
                # Calculate orientation (angle of top edge)
                top_edge = marker_corners[1] - marker_corners[0]
                angle = np.arctan2(top_edge[1], top_edge[0]) * 180 / np.pi
                
                marker_info = {
                    'id': int(marker_id),
                    'corners': marker_corners,
                    'center': center,
                    'angle': angle,
                    'name': self.get_marker_name(marker_id)
                }
                
                # If camera calibration available, estimate pose
                if self.camera_matrix is not None and self.dist_coeffs is not None:
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                        [marker_corners], self.marker_size, 
                        self.camera_matrix, self.dist_coeffs
                    )
                    marker_info['rvec'] = rvec[0]
                    marker_info['tvec'] = tvec[0]
                    
                    # Convert rotation vector to euler angles
                    rmat, _ = cv2.Rodrigues(rvec[0])
                    euler = self.rotation_matrix_to_euler(rmat)
                    marker_info['euler_angles'] = euler
                
                detected_info[marker_id] = marker_info
                
        self.detected_markers = detected_info
        return detected_info
    
    def get_marker_name(self, marker_id: int) -> str:
        """Get descriptive name for marker ID"""
        if marker_id == self.BASEFRAME_ID:
            return "BaseFrame"
        elif marker_id in self.BOX_MARKERS:
            return self.BOX_MARKERS[marker_id]
        else:
            return f"Unknown_{marker_id}"
    
    def get_box_orientation(self) -> Optional[str]:
        """
        Determine which way the box is facing based on visible markers
        
        Returns:
            String indicating box orientation or None if not determinable
        """
        visible_box_markers = []
        
        for marker_id, info in self.detected_markers.items():
            if marker_id in self.BOX_MARKERS:
                visible_box_markers.append((marker_id, info))
        
        if not visible_box_markers:
            return None
        
        # Find the most prominent marker (largest area)
        max_area = 0
        dominant_marker = None
        
        for marker_id, info in visible_box_markers:
            corners = info['corners']
            area = cv2.contourArea(corners)
            if area > max_area:
                max_area = area
                dominant_marker = marker_id
        
        if dominant_marker:
            return self.BOX_MARKERS[dominant_marker]
        
        return None
    
    def get_relative_pose(self, marker1_id: int, marker2_id: int) -> Optional[Dict]:
        """
        Calculate relative pose between two markers
        
        Returns:
            Dictionary with relative position and rotation
        """
        if marker1_id not in self.detected_markers or marker2_id not in self.detected_markers:
            return None
        
        if 'tvec' not in self.detected_markers[marker1_id]:
            return None
        
        m1 = self.detected_markers[marker1_id]
        m2 = self.detected_markers[marker2_id]
        
        # Calculate relative translation
        rel_translation = m2['tvec'] - m1['tvec']
        
        # Calculate relative rotation
        r1, _ = cv2.Rodrigues(m1['rvec'])
        r2, _ = cv2.Rodrigues(m2['rvec'])
        rel_rotation = r2 @ r1.T
        rel_rvec, _ = cv2.Rodrigues(rel_rotation)
        
        return {
            'translation': rel_translation[0],
            'rotation': rel_rvec[0],
            'distance': np.linalg.norm(rel_translation)
        }
    
    def rotation_matrix_to_euler(self, R: np.ndarray) -> Tuple[float, float, float]:
        """Convert rotation matrix to Euler angles (roll, pitch, yaw)"""
        sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
        
        singular = sy < 1e-6
        
        if not singular:
            x = math.atan2(R[2,1], R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else:
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
        
        return np.array([x, y, z]) * 180.0 / np.pi
    
    def draw_markers(self, image: np.ndarray, draw_axes: bool = True) -> np.ndarray:
        """
        Draw detected markers on image
        
        Args:
            image: Input image
            draw_axes: Whether to draw 3D axes (requires camera calibration)
        
        Returns:
            Image with drawn markers
        """
        output = image.copy()
        
        # Draw detected markers
        for marker_id, info in self.detected_markers.items():
            corners = info['corners'].astype(int)
            center = info['center'].astype(int)
            
            # Draw marker outline
            cv2.polylines(output, [corners], True, (0, 255, 0), 2)
            
            # Draw marker ID and name
            cv2.putText(output, f"ID:{marker_id} - {info['name']}", 
                       (center[0] - 50, center[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Draw center point
            cv2.circle(output, tuple(center), 5, (0, 0, 255), -1)
            
            # Draw orientation arrow
            angle_rad = info['angle'] * np.pi / 180
            arrow_length = 50
            arrow_end = (
                int(center[0] + arrow_length * np.cos(angle_rad)),
                int(center[1] + arrow_length * np.sin(angle_rad))
            )
            cv2.arrowedLine(output, tuple(center), arrow_end, (255, 0, 0), 2)
            
            # Draw 3D axes if camera is calibrated
            if draw_axes and 'rvec' in info and self.camera_matrix is not None:
                aruco.drawAxis(output, self.camera_matrix, self.dist_coeffs,
                             info['rvec'], info['tvec'], self.marker_size * 0.5)
                
                # Show euler angles
                if 'euler_angles' in info:
                    euler = info['euler_angles']
                    cv2.putText(output, f"R:{euler[0]:.1f} P:{euler[1]:.1f} Y:{euler[2]:.1f}", 
                               (center[0] - 50, center[1] + 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        # Show box orientation
        box_orientation = self.get_box_orientation()
        if box_orientation:
            cv2.putText(output, f"Box facing: {box_orientation}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        return output
    
    def calibrate_from_stereo_params(self, stereo_params_path: str, camera: str = 'left'):
        """
        Load camera calibration from your stereo_params.yml file
        
        Args:
            stereo_params_path: Path to stereo_params.yml
            camera: 'left' or 'right'
        """
        import yaml
        
        try:
            with open(stereo_params_path, 'r') as f:
                params = yaml.safe_load(f)
            
            if camera == 'left':
                self.camera_matrix = np.array(params['mtxL'])
                self.dist_coeffs = np.array(params['distL'])
            else:
                self.camera_matrix = np.array(params['mtxR'])
                self.dist_coeffs = np.array(params['distR'])
                
            logger.info(f"Loaded camera calibration for {camera} camera")
            
        except Exception as e:
            logger.error(f"Failed to load calibration: {e}")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='ArUco Marker Detection')
    parser.add_argument('--camera', type=int, default=0, help='Camera index')
    parser.add_argument('--calibration', type=str, help='Path to stereo_params.yml')
    parser.add_argument('--marker-size', type=float, default=0.068, help='Marker size in meters (default: 0.068m = 6.8cm)')
    parser.add_argument('--camera-side', type=str, default='left', help='Camera side (left/right)')
    parser.add_argument('--width', type=int, default=3840, help='Camera width (default: 3840 for 4K)')
    parser.add_argument('--height', type=int, default=2160, help='Camera height (default: 2160 for 4K)')
    parser.add_argument('--fps', type=int, default=30, help='Camera FPS')
    
    args = parser.parse_args()
    
    # Initialize detector
    detector = ArUcoDetector(marker_size=args.marker_size)
    
    # Load calibration if provided
    if args.calibration:
        detector.calibrate_from_stereo_params(args.calibration, args.camera_side)
    
    # Open camera
    cap = cv2.VideoCapture(args.camera)
    
    # Set 4K resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    
    # Verify actual resolution
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = int(cap.get(cv2.CAP_PROP_FPS))
    
    logger.info(f"Camera settings: {actual_width}x{actual_height} @ {actual_fps}fps")
    
    if actual_width != args.width or actual_height != args.height:
        logger.warning(f"Could not set requested resolution. Using {actual_width}x{actual_height}")
    
    logger.info("Starting ArUco detection. Press 'q' to quit.")
    logger.info("Press 's' to save current frame")
    
    # Performance monitoring
    frame_count = 0
    start_time = cv2.getTickCount()
    
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        
        # Detect markers
        markers = detector.detect_markers(frame)
        
        # Draw results
        output = detector.draw_markers(frame, draw_axes=(args.calibration is not None))
        
        # Calculate and display FPS
        frame_count += 1
        if frame_count % 30 == 0:
            end_time = cv2.getTickCount()
            fps = 30 / ((end_time - start_time) / cv2.getTickFrequency())
            start_time = end_time
            cv2.putText(output, f"FPS: {fps:.1f}", (10, output.shape[0] - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Print detected markers info
        if markers and frame_count % 30 == 0:
            logger.info(f"Detected {len(markers)} markers")
            for marker_id, info in markers.items():
                logger.info(f"  {info['name']} (ID:{marker_id}) at {info['center']}")
            
            # Check relative pose to baseframe
            if detector.BASEFRAME_ID in markers:
                for box_id in detector.BOX_MARKERS:
                    if box_id in markers:
                        rel_pose = detector.get_relative_pose(detector.BASEFRAME_ID, box_id)
                        if rel_pose:
                            logger.info(f"  {detector.BOX_MARKERS[box_id]} relative to BaseFrame: "
                                      f"dist={rel_pose['distance']:.3f}m")
        
        # Resize for display (4K is too large for most screens)
        display_scale = 0.5  # Show at 1920x1080
        display_frame = cv2.resize(output, None, fx=display_scale, fy=display_scale)
        cv2.imshow('ArUco Detection', display_frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            filename = f'aruco_detection_4k_{frame_count}.jpg'
            cv2.imwrite(filename, output)
            logger.info(f"Saved full resolution image to {filename}")
    
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
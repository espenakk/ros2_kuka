#!/usr/bin/env python3
import cv2
import numpy as np
import yaml
import socket
import toml
import time
import logging
import cv2.aruco as aruco
import math
from typing import Tuple, Optional, List, Dict
from rclpy.node import Node
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class CombinedTracker(Node):
    def __init__(self):
        super().__init__("camera_node")
        self.declare_parameter("detection_config_path", "")
        config_path = self.get_parameter("detection_config_path").get_parameter_value().string_value
        self.config = toml.load(config_path)

        self.declare_parameter("stereo_params_path", "")
        params_path = self.get_parameter("stereo_params_path").get_parameter_value()
        self.load_stereo_params(params_path.string_value)

        self.setup_detection_params()
        self.setup_aruco()

        self.bridge = CvBridge()

        # Subscribe to left_camera
        self.left_cam_subscription = self.create_subscription(
            Image,
            "/camera_left/image_raw",
            self.left_image_callback,
            10)

        # Subscribe to right_camera
        self.right_cam_subscription = self.create_subscription(
            Image,
            "/camera_right/image_raw",
            self.right_image_callback,
            10)

        # Publish images with markers
        self.left_marker_img_publisher = self.create_publisher(
            Image,
            "/camera_left/marker_image",
            10)
        self.right_marker_img_publisher = self.create_publisher(
            Image,
            "camera_right/marker_image",
            10)

        # Publish ball position
        self.ball_publisher = self.create_publisher(
            Point,
            "/ball/position",
            10)
        
        # Storage for tracking data
        self.base_frame_pose = None
        self.box_poses = {}
        self.last_ball_world_pos = None

        self.prevLeftPt = None
        self.prevRightPt = None

    def left_image_callback(self, img):
        self.generic_camera_callback(img, self.prevRightPt, 'left', self.mtxL, self.distL)

    def right_image_callback(self, img):
        self.generic_camera_callback(img, self.prevLeftPt, 'right', self.mtxR, self.distR)

        
    def setup_aruco(self):
        """Initialize ArUco detector"""
        #try:
        #    # New OpenCV (4.7+)
        #    self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        #    self.aruco_params = aruco.DetectorParameters()
        #except AttributeError:
            # Old OpenCV
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()
        
        # Marker definitions
        self.BASEFRAME_ID = 0
        self.BOX_MARKERS = {
            1: "Box_Top",
            2: "Box_Front", 
            3: "Box_Left",
            4: "Box_Right",
            5: "Box_Back"
        }
        
        # Marker size (same as in ArUco detector)
        self.marker_size = 0.068  # 6.8cm
        
        # Camera offset from base frame center (in mm)
        # Camera is mounted 10cm to the right (positive Y) and 7cm above (positive Z) base frame center
        self.camera_offset = np.array([0.0, 100.0, 70.0])  # [X, Y, Z] in mm
        # X: 0cm (no forward/backward offset from base frame)
        # Y: 10cm to the right (positive Y direction for robot)  
        # Z: 7cm above base frame center
        
        logger.info("ArUco detector initialized")
        logger.info(f"Camera offset from base frame: {self.camera_offset} mm")
        
    def load_stereo_params(self, params_path: str):
        """Load stereo calibration parameters"""
        try:
            with open(params_path, 'r') as f:
                params = yaml.safe_load(f)
            
            self.mtxL = np.array(params['mtxL'])
            self.distL = np.array(params['distL'])
            self.mtxR = np.array(params['mtxR'])
            self.distR = np.array(params['distR'])
            self.R = np.array(params['R'])
            self.T = np.array(params['T'])
            
            # Calculate projection matrices
            self.P1 = self.mtxL @ np.hstack([np.eye(3), np.zeros((3,1))])
            self.P2 = self.mtxR @ np.hstack([self.R, self.T])
            
            logger.info("Stereo parameters loaded successfully")
            
        except FileNotFoundError:
            logger.error(f"Stereo parameters file {params_path} not found. Run calibration first.")
            raise
        except Exception as e:
            logger.error(f"Error loading stereo parameters: {e}")
            raise
    
    def setup_detection_params(self):
        """Setup detection parameters from config"""
        ball_config = self.config['ball']
        
        # Ball detection HSV values
        if 'hsv_min_left' in ball_config and 'hsv_min_right' in ball_config:
            self.hsv_min_left = tuple(ball_config['hsv_min_left'])
            self.hsv_max_left = tuple(ball_config['hsv_max_left'])
            self.hsv_min_right = tuple(ball_config['hsv_min_right'])
            self.hsv_max_right = tuple(ball_config['hsv_max_right'])
        else:
            self.hsv_min_left = self.hsv_min_right = tuple(ball_config['hsv_min'])
            self.hsv_max_left = self.hsv_max_right = tuple(ball_config['hsv_max'])
        
        self.min_radius = ball_config['min_radius']
        self.min_area = ball_config.get('min_area', 50)
        self.circularity_threshold = ball_config.get('circularity_threshold', 0.8)
        
        # Performance parameters
        perf_config = self.config.get('performance', {})
        self.blur_kernel_size = perf_config.get('blur_kernel_size', 7)
        self.morph_iterations = perf_config.get('morph_iterations', 2)
        
        logger.info(f"Detection params loaded")
    
    def detect_aruco_markers(self, image: np.ndarray, camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> Dict:
        """Detect ArUco markers in image"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self.get_logger().info("14")
        
        try:
            # Try new OpenCV API first
            self.get_logger().info("18")
            detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.get_logger().info("15")
            corners, ids, rejected = detector.detectMarkers(gray)
            self.get_logger().info("16")
        except AttributeError:
            self.get_logger().info("19")
            if self.aruco_dict is None:
                self.get_logger().info("Aruco dict none")
            if self.aruco_params is None:
                self.get_logger().info("Aruco params is none")
            if gray is None:
                self.get_logger().info("No gray today, the gray has gone away")
            
            # Fall back to old API
            corners, ids, rejected = aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )
            self.get_logger().info("17")
        
        detected_markers = {}
        self.get_logger().info("20")
        
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                marker_corners = corners[i][0]
                center = np.mean(marker_corners, axis=0)
                
                # Estimate pose
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    [marker_corners], self.marker_size, 
                    camera_matrix, dist_coeffs
                )
                
                detected_markers[int(marker_id)] = {
                    'id': int(marker_id),
                    'corners': marker_corners,
                    'center': center,
                    'rvec': rvec[0],
                    'tvec': tvec[0],
                    'name': self.get_marker_name(marker_id)
                }
                
        return detected_markers
    
    def get_marker_name(self, marker_id: int) -> str:
        """Get descriptive name for marker ID"""
        if marker_id == self.BASEFRAME_ID:
            return "BaseFrame"
        elif marker_id in self.BOX_MARKERS:
            return self.BOX_MARKERS[marker_id]
        else:
            return f"Unknown_{marker_id}"
    
    def detect_ball(self, image: np.ndarray, camera_side: str = 'left') -> Tuple[Optional[Tuple[float, float]], float]:
        """Detect ball using HSV color filtering"""
        try:
            if camera_side == 'left':
                hsv_min = self.hsv_min_left
                hsv_max = self.hsv_max_left
            else:
                hsv_min = self.hsv_min_right
                hsv_max = self.hsv_max_right
            
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, hsv_min, hsv_max)
            
            # Clean up mask
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=self.morph_iterations)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.medianBlur(mask, self.blur_kernel_size)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            best_ball = None
            best_circularity = 0
            
            if contours:
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > self.min_area:
                        perimeter = cv2.arcLength(contour, True)
                        if perimeter > 0:
                            circularity = 4 * np.pi * area / (perimeter * perimeter)
                            
                            if circularity > self.circularity_threshold:
                                (x, y), radius = cv2.minEnclosingCircle(contour)
                                
                                if radius > self.min_radius:
                                    if circularity > best_circularity:
                                        best_ball = ((float(x), float(y)), radius)
                                        best_circularity = circularity
            
            if best_ball:
                return best_ball[0], best_ball[1]
            
            return None, 0
            
        except Exception as e:
            logger.error(f"Error in ball detection ({camera_side}): {e}")
            return None, 0
    
    def triangulate_point(self, pt_left: np.ndarray, pt_right: np.ndarray) -> np.ndarray:
        """Triangulate 3D point from stereo correspondence"""
        try:
            points_4d = cv2.triangulatePoints(self.P1, self.P2, pt_left.T, pt_right.T)
            points_3d = (points_4d / points_4d[3])[:3].T
            return points_3d[0]
        except Exception as e:
            logger.error(f"Error in triangulation: {e}")
            return np.array([0, 0, 0])
    
    def transform_to_baseframe(self, camera_point: np.ndarray) -> Optional[np.ndarray]:
        """Transform 3D point from camera coordinate system to base frame"""
        if self.base_frame_pose is None:
            return None
        
        try:
            # Get base frame pose
            rvec = self.base_frame_pose['rvec']
            tvec = self.base_frame_pose['tvec']
            
            # Convert rotation vector to matrix
            R_base, _ = cv2.Rodrigues(rvec)
            
            # Convert camera point from meters to mm for consistency
            camera_point_mm = camera_point * 1000.0  # Convert m to mm
            
            # Apply camera offset correction
            # The camera sees the base frame at tvec, but camera is offset from base frame center
            corrected_base_position = tvec.flatten() * 1000.0  # Convert to mm
            
            # Transform point to base frame coordinates
            # 1. Translate by base frame position
            # 2. Rotate by base frame orientation  
            # 3. Apply camera offset correction
            relative_point = camera_point_mm - corrected_base_position
            transformed_point = R_base.T @ relative_point
            
            # Apply camera mounting offset
            # Since camera is offset from base frame center, we need to add this offset
            final_point = transformed_point + self.camera_offset
            
            return final_point
            
        except Exception as e:
            logger.error(f"Error transforming to base frame: {e}")
            return None
    
    def get_box_orientation_and_pose(self, detected_markers: Dict) -> Optional[Dict]:
        """Determine box orientation and pose from visible markers"""
        visible_box_markers = {}
        
        for marker_id, info in detected_markers.items():
            if marker_id in self.BOX_MARKERS:
                visible_box_markers[marker_id] = info
        
        if not visible_box_markers:
            return None
        
        # Find the most prominent marker (largest area)
        max_area = 0
        dominant_marker_id = None
        
        for marker_id, info in visible_box_markers.items():
            corners = info['corners']
            area = cv2.contourArea(corners)
            if area > max_area:
                max_area = area
                dominant_marker_id = marker_id
        
        if dominant_marker_id:
            marker_info = visible_box_markers[dominant_marker_id]
            
            # Transform marker pose to base frame if possible
            world_pose = None
            if self.base_frame_pose is not None:
                marker_pos = marker_info['tvec'].flatten()
                # For box markers, we also need to apply the camera offset correction
                corrected_pos = marker_pos * 1000.0  # Convert to mm
                base_pos = self.base_frame_pose['tvec'].flatten() * 1000.0
                R_base, _ = cv2.Rodrigues(self.base_frame_pose['rvec'])
                
                relative_pos = corrected_pos - base_pos
                world_pos = R_base.T @ relative_pos + self.camera_offset
                world_pose = world_pos
            
            return {
                'dominant_face': self.BOX_MARKERS[dominant_marker_id],
                'marker_id': dominant_marker_id,
                'camera_pose': marker_info['tvec'].flatten(),
                'world_pose': world_pose,
                'all_visible': list(visible_box_markers.keys())
            }
        
        return None

    def generic_camera_callback(self, img, other_pt, side : str, mtx, dist):
        # Getting images
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return

        self.get_logger().info("1")

        # Storage for detected points
        pt = None
        self.get_logger().info("2")

        # Detect ArUco markers in left camera (for base frame reference)
        markers = self.detect_aruco_markers(cv_img, mtx, dist)
        self.get_logger().info("3")

        # Update base frame pose if detected
        if self.BASEFRAME_ID in markers:
            self.base_frame_pose = markers[self.BASEFRAME_ID]
        self.get_logger().info("4")

        box_info = self.get_box_orientation_and_pose(markers)
        self.get_logger().info("5")

        detection, radius = self.detect_ball(cv_img, side)
        self.get_logger().info("6")

        if detection is not None:
            pt = detection
        self.get_logger().info("7")

        ball_world_pos = None
        if pt is not None and other_pt is not None:
            self.get_logger().info("8")
            xyz_camera = self.triangulate_point(
                np.array([pt]), 
                np.array([other_pt])
            )
            self.get_logger().info("9")
            
            # Transform to base frame coordinates
            ball_world_pos = self.transform_to_baseframe(xyz_camera)
            self.get_logger().info("10")
            
            if ball_world_pos is not None:
                self.last_ball_world_pos = ball_world_pos
                self.get_logger().info("11")

            ball_pos_msg = Point(ball_world_pos[0], ball_world_pos[1], ball_world_pos[2])
            self.get_logger().info("12")
            self.ball_publisher.publish(ball_pos_msg)
            self.get_logger().info("13")

        self.draw_visualization(cv_img, markers, detection, radius, box_info, mtx, dist, side)

    
    def draw_visualization(self, img, markers, ball_pos, radius, box_info, mtx, dist, side):
        """Draw visualization overlays"""
        # Draw ArUco markers
        for marker_id, info in markers.items():
            corners = info['corners'].astype(int)
            center = info['center'].astype(int)
            
            # Draw marker outline
            cv2.polylines(img, [corners], True, (0, 255, 0), 2)
            cv2.putText(img, f"{info['name']}", 
                       (center[0] - 30, center[1] - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Draw 3D axes (check OpenCV version compatibility)
            if 'rvec' in info:
                try:
                    # Try new OpenCV method
                    cv2.drawFrameAxes(img, mtx, dist,
                                    info['rvec'], info['tvec'], self.marker_size * 0.5)
                except AttributeError:
                    try:
                        # Try old aruco method
                        aruco.drawAxis(img, mtx, dist,
                                     info['rvec'], info['tvec'], self.marker_size * 0.5)
                    except AttributeError:
                        # Skip axis drawing if neither method works
                        pass
        
        # Draw ball detections
        if ball_pos is not None:
            cv2.circle(img, (int(ball_pos[0]), int(ball_pos[1])), 
                     int(radius), (0, 255, 255), 2)
            cv2.putText(img, "BALL", 
                       (int(ball_pos[0]-20), int(ball_pos[1]-25)), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        # Draw status info
        y_offset = 30
        if self.base_frame_pose is not None:
            cv2.putText(img, "BASE FRAME: OK", (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            y_offset += 25
        else:
            cv2.putText(img, "BASE FRAME: NOT FOUND", (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            y_offset += 25
        
        if box_info:
            cv2.putText(img, f"BOX: {box_info['dominant_face']}", (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            y_offset += 25
        
        if self.last_ball_world_pos is not None:
            pos_text = f"BALL: ({self.last_ball_world_pos[0]:.1f}, {self.last_ball_world_pos[1]:.1f}, {self.last_ball_world_pos[2]:.1f})"
            cv2.putText(img, pos_text, (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        # Publish markers and ball detections
        try:
            img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return


        if side == 'left':
            self.left_marker_img_publisher.publish(img_msg)
        else:
            self.right_marker_img_publisher.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    camera_node = CombinedTracker()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

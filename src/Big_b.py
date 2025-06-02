#!/usr/bin/env python3
import cv2
import numpy as np
import yaml
import argparse
import socket
import toml
import time
import logging
import cv2.aruco as aruco
import math
from typing import Tuple, Optional, List, Dict

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class CombinedTracker:
    def __init__(self, config_path: str = 'config.toml', params_path: str = 'stereo_params.yml'):
        self.config = toml.load(config_path)
        self.load_stereo_params(params_path)
        self.setup_network()
        self.setup_detection_params()
        self.setup_camera_params()
        self.setup_aruco()
        
        # Storage for tracking data
        self.base_frame_pose = None
        self.box_poses = {}
        self.last_ball_world_pos = None
        
    def setup_aruco(self):
        """Initialize ArUco detector"""
        try:
            # New OpenCV (4.7+)
            self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
            self.aruco_params = aruco.DetectorParameters()
        except AttributeError:
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
    
    def setup_network(self):
        """Initialize UDP socket for data transmission"""
        try:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.host = self.config['network']['host']
            self.port = self.config['network']['port']
            logger.info(f"UDP socket configured for {self.host}:{self.port}")
        except Exception as e:
            logger.error(f"Error setting up network: {e}")
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
        
        # Debug settings
        self.debug_config = self.config['debug']
        self.show_fps = self.debug_config['show_fps']
        self.log_detections = self.debug_config['log_detections']
        
        logger.info(f"Detection params loaded")
    
    def setup_camera_params(self):
        """Setup camera parameters from config"""
        camera_config = self.config['camera']
        self.left_cam_index = camera_config['left_index']
        self.right_cam_index = camera_config['right_index']
        self.cam_width = camera_config['width']
        self.cam_height = camera_config['height']
        self.cam_fps = camera_config['fps']
        self.auto_exposure = camera_config['auto_exposure']
        
        logger.info(f"Camera settings loaded")
    
    def setup_camera(self, cap, index):
        """Configure camera settings"""
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cam_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cam_height)
        cap.set(cv2.CAP_PROP_FPS, self.cam_fps)
        
        if not self.auto_exposure:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            
    def detect_aruco_markers(self, image: np.ndarray, camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> Dict:
        """Detect ArUco markers in image"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        try:
            # Try new OpenCV API first
            detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, rejected = detector.detectMarkers(gray)
        except AttributeError:
            # Fall back to old API
            corners, ids, rejected = aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )
        
        detected_markers = {}
        
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
    
    def send_data(self, data: Dict):
        """Send data via UDP"""
        try:
            import json
            # Convert numpy arrays to lists for JSON serialization
            def convert_numpy(obj):
                if isinstance(obj, np.ndarray):
                    return obj.tolist()
                elif isinstance(obj, dict):
                    return {k: convert_numpy(v) for k, v in obj.items()}
                elif isinstance(obj, list):
                    return [convert_numpy(v) for v in obj]
                else:
                    return obj
            
            serializable_data = convert_numpy(data)
            packet = json.dumps(serializable_data)
            self.udp_socket.sendto(packet.encode(), (self.host, self.port))
            
            if self.log_detections:
                logger.debug(f"Sent: {serializable_data}")
            
        except Exception as e:
            logger.error(f"Error sending data: {e}")
    
    def run(self, left_cam: int = None, right_cam: int = None, show_display: bool = True):
        """Main tracking loop"""
        try:
            if left_cam is None:
                left_cam = self.left_cam_index
            if right_cam is None:
                right_cam = self.right_cam_index
            
            # Initialize cameras
            cap_left = cv2.VideoCapture(left_cam)
            cap_right = cv2.VideoCapture(right_cam)
            
            if not (cap_left.isOpened() and cap_right.isOpened()):
                logger.error("Failed to open cameras")
                return
            
            self.setup_camera(cap_left, left_cam)
            self.setup_camera(cap_right, right_cam)
            
            logger.info("Starting combined ball and ArUco tracking...")
            
            # Storage for detected points
            pt_left = None
            pt_right = None
            
            frame_count = 0
            fps_start = time.time()
            
            while True:
                ret_left, img_left = cap_left.read()
                ret_right, img_right = cap_right.read()
                
                if not (ret_left and ret_right):
                    logger.warning("Failed to read frames")
                    continue
                
                frame_count += 1
                
                # Detect ArUco markers in left camera (for base frame reference)
                markers_left = self.detect_aruco_markers(img_left, self.mtxL, self.distL)
                
                # Update base frame pose if detected
                if self.BASEFRAME_ID in markers_left:
                    self.base_frame_pose = markers_left[self.BASEFRAME_ID]
                
                # Get box orientation and pose
                box_info = self.get_box_orientation_and_pose(markers_left)
                
                # Detect ball in both cameras
                detection_left, radius_left = self.detect_ball(img_left, 'left')
                detection_right, radius_right = self.detect_ball(img_right, 'right')
                
                # Update ball tracking
                if detection_left is not None:
                    pt_left = detection_left
                if detection_right is not None:
                    pt_right = detection_right
                
                # Triangulate ball position if both points available
                ball_world_pos = None
                if pt_left is not None and pt_right is not None:
                    xyz_camera = self.triangulate_point(
                        np.array([pt_left]), 
                        np.array([pt_right])
                    )
                    
                    # Transform to base frame coordinates
                    ball_world_pos = self.transform_to_baseframe(xyz_camera)
                    
                    if ball_world_pos is not None:
                        self.last_ball_world_pos = ball_world_pos
                
                # Prepare data packet
                data_packet = {
                    'timestamp': time.time(),
                    'frame': frame_count,
                    'base_frame_detected': self.base_frame_pose is not None,
                    'ball': {
                        'detected': ball_world_pos is not None,
                        'world_pos_mm': ball_world_pos.tolist() if ball_world_pos is not None else None,
                        'camera_pos_mm': xyz_camera.tolist() if 'xyz_camera' in locals() and xyz_camera is not None else None
                    },
                    'box': box_info if box_info else None
                }
                
                # Send data
                self.send_data(data_packet)
                
                # Display if requested
                if show_display:
                    self.draw_visualization(img_left, img_right, markers_left, 
                                          detection_left, detection_right, 
                                          radius_left, radius_right, box_info)
                    
                    # Show FPS
                    if self.show_fps and frame_count % 30 == 0:
                        fps = frame_count / (time.time() - fps_start)
                        logger.info(f"FPS: {fps:.1f}")
                    
                    if cv2.waitKey(1) & 0xFF == 27:  # ESC
                        logger.info("Exiting...")
                        break
            
        except Exception as e:
            logger.error(f"Error in main loop: {e}")
            
        finally:
            if 'cap_left' in locals():
                cap_left.release()
            if 'cap_right' in locals():
                cap_right.release()
            cv2.destroyAllWindows()
            self.udp_socket.close()
    
    def draw_visualization(self, img_left, img_right, markers, 
                          ball_left, ball_right, radius_left, radius_right, box_info):
        """Draw visualization overlays"""
        # Draw ArUco markers
        for marker_id, info in markers.items():
            corners = info['corners'].astype(int)
            center = info['center'].astype(int)
            
            # Draw marker outline
            cv2.polylines(img_left, [corners], True, (0, 255, 0), 2)
            cv2.putText(img_left, f"{info['name']}", 
                       (center[0] - 30, center[1] - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Draw 3D axes (check OpenCV version compatibility)
            if 'rvec' in info:
                try:
                    # Try new OpenCV method
                    cv2.drawFrameAxes(img_left, self.mtxL, self.distL,
                                    info['rvec'], info['tvec'], self.marker_size * 0.5)
                except AttributeError:
                    try:
                        # Try old aruco method
                        aruco.drawAxis(img_left, self.mtxL, self.distL,
                                     info['rvec'], info['tvec'], self.marker_size * 0.5)
                    except AttributeError:
                        # Skip axis drawing if neither method works
                        pass
        
        # Draw ball detections
        if ball_left is not None:
            cv2.circle(img_left, (int(ball_left[0]), int(ball_left[1])), 
                     int(radius_left), (0, 255, 255), 2)
            cv2.putText(img_left, "BALL", 
                       (int(ball_left[0]-20), int(ball_left[1]-25)), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        if ball_right is not None:
            cv2.circle(img_right, (int(ball_right[0]), int(ball_right[1])), 
                     int(radius_right), (0, 255, 255), 2)
            cv2.putText(img_right, "BALL", 
                       (int(ball_right[0]-20), int(ball_right[1]-25)), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        # Draw status info
        y_offset = 30
        if self.base_frame_pose is not None:
            cv2.putText(img_left, "BASE FRAME: OK", (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            y_offset += 25
        else:
            cv2.putText(img_left, "BASE FRAME: NOT FOUND", (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            y_offset += 25
        
        if box_info:
            cv2.putText(img_left, f"BOX: {box_info['dominant_face']}", (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            y_offset += 25
        
        if self.last_ball_world_pos is not None:
            pos_text = f"BALL: ({self.last_ball_world_pos[0]:.1f}, {self.last_ball_world_pos[1]:.1f}, {self.last_ball_world_pos[2]:.1f})"
            cv2.putText(img_left, pos_text, (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        cv2.imshow('Left Camera', img_left)
        cv2.imshow('Right Camera', img_right)

def main():
    parser = argparse.ArgumentParser(description='Combined Ball and ArUco Tracker')
    parser.add_argument('--left', type=int, default=None, help='Left camera index')
    parser.add_argument('--right', type=int, default=None, help='Right camera index')
    parser.add_argument('--show', action='store_true', default=True, help='Show camera feeds')
    parser.add_argument('--config', default='config.toml', help='Configuration file')
    parser.add_argument('--params', default='stereo_params.yml', help='Stereo parameters file')
    
    args = parser.parse_args()
    
    try:
        tracker = CombinedTracker(args.config, args.params)
        tracker.run(args.left, args.right, args.show)
    except Exception as e:
        logger.error(f"Failed to initialize tracker: {e}")

if __name__ == '__main__':
    main()
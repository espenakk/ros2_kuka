#!/usr/bin/env python3
import cv2
import yaml
import argparse
import numpy as np
from datetime import datetime

def collect_frames(camL, camR, board, square):
    """Collect calibration frames from both cameras"""
    # Create object points (3D points in real world space)
    objp = np.zeros((board[1]*board[0], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board[0], 0:board[1]].T.reshape(-1, 2)
    objp *= square
    
    # Arrays to store object points and image points from both cameras
    objpoints = []      # 3d point in real world space
    imgLpoints = []     # 2d points in left image plane
    imgRpoints = []     # 2d points in right image plane
    
    # Initialize cameras
    capL = cv2.VideoCapture(camL)
    capR = cv2.VideoCapture(camR)
    
    if not (capL.isOpened() and capR.isOpened()):
        print("Feil: Kunne ikkje opne kameraene")
        return None, None, None, None
    
    # Set resolution for both cameras
    capL.set(cv2.CAP_PROP_FRAME_WIDTH, 900)
    capL.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
    capR.set(cv2.CAP_PROP_FRAME_WIDTH, 900)
    capR.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
    
    print(f"Venstre kamera oppløysning: {int(capL.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(capL.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    print(f"Høgre kamera oppløysning: {int(capR.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(capR.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    
    collected = 0
    max_frames = 50
    frame_delay = 0.1  # Counter for spacing between captures
    print(f"Auto-capture aktivert! Samlar {max_frames} frames automatisk...")
    print("Trykk ESC for å stoppe tidleg")
    
    # Create named window once
    cv2.namedWindow('Auto Kalibrering', cv2.WINDOW_AUTOSIZE)
    
    while collected < max_frames:
        retL, imgL = capL.read()
        retR, imgR = capR.read()
        
        if not (retL and retR):
            print("Feil: Kunne ikkje lese frames")
            break
            
        grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)
        
        # Find chessboard corners with improved flags
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FILTER_QUADS
        retL, cornersL = cv2.findChessboardCorners(grayL, board, flags)
        retR, cornersR = cv2.findChessboardCorners(grayR, board, flags)
        
        # Create visualization
        vis = np.hstack([imgL.copy(), imgR.copy()])
        
        # If corners found in both images
        if retL and retR:
            # Refine corner positions with better criteria
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.0001)
            cornersL = cv2.cornerSubPix(grayL, cornersL, (11, 11), (-1, -1), criteria)
            cornersR = cv2.cornerSubPix(grayR, cornersR, (11, 11), (-1, -1), criteria)
            
            # Draw corners
            cv2.drawChessboardCorners(vis[:, :imgL.shape[1]], board, cornersL, retL)
            cv2.drawChessboardCorners(vis[:, imgL.shape[1]:], board, cornersR, retR)
            
            # Auto-capture when corners are found and enough delay has passed
            if frame_delay > 30:  # Wait 30 frames between captures (~1 second at 30fps)
                objpoints.append(objp)
                imgLpoints.append(cornersL)
                imgRpoints.append(cornersR)
                collected += 1
                frame_delay = 0  # Reset delay counter
                print(f"[{collected}/{max_frames}] frame automatisk samla")
            else:
                frame_delay += 1
                
            # Add text to show status
            cv2.putText(vis, f"Samlar automatisk... ({frame_delay}/30)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(vis, "Finn ikkje sjakkbrett i begge bilete", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            frame_delay = 0  # Reset delay if no corners found
        
        cv2.putText(vis, f"Samla: {collected}/{max_frames} frames", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Show progress bar
        progress_width = 400
        progress_height = 20
        progress_x = 10
        progress_y = 100
        progress_fill = int((collected / max_frames) * progress_width)
        
        cv2.rectangle(vis, (progress_x, progress_y), (progress_x + progress_width, progress_y + progress_height), (100, 100, 100), -1)
        cv2.rectangle(vis, (progress_x, progress_y), (progress_x + progress_fill, progress_y + progress_height), (0, 255, 0), -1)
        
        # Show in same window
        cv2.imshow('Auto Kalibrering', vis)
        
        key = cv2.waitKey(1) & 0xFF
        
        # Exit on ESC
        if key == 27:
            print("Stoppe tidleg av brukar")
            break
    
    print(f"Auto-capture fullført! Samla {collected} frames")
    capL.release()
    capR.release()
    cv2.destroyAllWindows()
    
    if collected < 15:
        print(f"Åtvaring: Berre {collected} frames samla. Anbefaler minst 15-20 for god kalibrering.")
    elif collected >= 50:
        print("Utmerka! 50 frames gir svært god kalibrering.")
    
    return objpoints, imgLpoints, imgRpoints, imgL.shape[:2]

def run_calibration():
    """Run the stereo calibration process"""
    # Hardcoded values
    camL = 4        # Left camera
    camR = 6        # Right camera
    board = (8, 6)  # Chessboard size
    square = 24.0   # Square size in mm
    
    print("Startar kalibreringsinnsamling...")
    obj, imgL, imgR, shape = collect_frames(camL, camR, board, square)
    
    if obj is None or len(obj) == 0:
        print("Feil: Ingen frames samla. Avsluttar.")
        return
    
    print(f"Kalibrerer med {len(obj)} frames...")
    
    # Calibrate individual cameras with better flags
    print("Kalibrerer venstre kamera...")
    flags_single = cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_THIN_PRISM_MODEL
    retL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(obj, imgL, shape[::-1], None, None, flags=flags_single)
    
    print("Kalibrerer høgre kamera...")
    retR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(obj, imgR, shape[::-1], None, None, flags=flags_single)
    
    print(f"Venstre kamera RMS feil: {retL:.3f}")
    print(f"Høgre kamera RMS feil: {retR:.3f}")
    
    # Stereo calibration with better flags and criteria
    print("Kalibrerer stereopar...")
    flags = cv2.CALIB_FIX_INTRINSIC + cv2.CALIB_RATIONAL_MODEL
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-6)
    
    retS, mtxL, distL, mtxR, distR, R, T, E, F = cv2.stereoCalibrate(
        obj, imgL, imgR, mtxL, distL, mtxR, distR, shape[::-1],
        criteria=criteria, flags=flags)
    
    print(f"Stereo kalibrering RMS feil: {retS:.3f}")
    
    # Save calibration data
    data = {
        'mtxL': mtxL.tolist(),
        'distL': distL.tolist(),
        'mtxR': mtxR.tolist(), 
        'distR': distR.tolist(),
        'R': R.tolist(),
        'T': T.tolist(),
        'E': E.tolist(),
        'F': F.tolist(),
        'image_size': shape[::-1],
        'rms_error': float(retS),
        'timestamp': datetime.utcnow().isoformat(),
        'frames_used': len(obj)
    }
    
    with open('stereo_params.yml', 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=False)
    
    print("=> stereo_params.yml lagra")
    print(f"Kalibrering fullført! RMS feil: {retS:.3f}")

if __name__ == '__main__':
    print(f"Konfiguration:")
    print(f"  Venstre kamera: 0")
    print(f"  Høgre kamera: 6")
    print(f"  Oppløysning: 900x800")
    print(f"  Sjakkbrett størrelse: (8, 6)")
    print(f"  Rutestørrelse: 25.0mm")
    
    run_calibration()
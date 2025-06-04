#!/usr/bin/env python3
import cv2
import numpy as np

print("HSV-KONTROLLER:")
print("Q/A = Hue Min +/-")
print("W/S = Sat Min +/-") 
print("E/D = Val Min +/-")
print("R/F = Hue Max +/-")
print("T/G = Sat Max +/-")
print("Y/H = Val Max +/-")
print("="*60)
print("FILTER-KONTROLLER:")
print("Z/X = Grønn kanal forsterking +/-")
print("C/V = Kontrast +/-")
print("B/N = Lysstyrke +/-")
print("M/, = Gamma korrigering +/-")
print("J/K = Saturasjon +/-")
print("U/I = Vibrance +/-")
print("O/P = Skarpheit +/-")


cap = cv2.VideoCapture(4)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
cap.set(cv2.CAP_PROP_FPS, 60)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)

print(f"Kamera setup: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")


h_min, s_min, v_min = 18, 10, 60
h_max, s_max, v_max = 101, 255, 135


green_boost = 1.0     
contrast = 1.1         
brightness = -10      
gamma = 1.0           
saturation = 2.4     
vibrance = 0.1         
sharpness = 0.90        

def apply_filters(frame, green_boost, contrast, brightness, gamma, saturation, vibrance, sharpness):
    """Anvend filter på kamerabildet"""
    filtered = frame.copy().astype(np.float32)
    
    # Grønn 
    if green_boost != 1.0:
        filtered[:, :, 1] = np.clip(filtered[:, :, 1] * green_boost, 0, 255)
    
    # Kontrast og lysstyrke
    if contrast != 1.0 or brightness != 0:
        filtered = np.clip(filtered * contrast + brightness, 0, 255)
    
    # Gamma
    if gamma != 1.0:
        filtered = np.clip(255 * (filtered / 255) ** gamma, 0, 255)
    
    # Konverter til HSV 
    if saturation != 1.0 or vibrance != 1.0:
        hsv = cv2.cvtColor(filtered.astype(np.uint8), cv2.COLOR_BGR2HSV).astype(np.float32)
        
        # Saturasjon 
        if saturation != 1.0:
            hsv[:, :, 1] = np.clip(hsv[:, :, 1] * saturation, 0, 255)
        
        # Vibrance 
        if vibrance != 1.0:
            low_sat_mask = hsv[:, :, 1] < 128
            hsv[low_sat_mask, 1] = np.clip(hsv[low_sat_mask, 1] * vibrance, 0, 255)
        
        filtered = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR).astype(np.float32)
    
    # Skarpheit
    if sharpness != 1.0:
        blurred = cv2.GaussianBlur(filtered.astype(np.uint8), (0, 0), 1.0)
        filtered = cv2.addWeighted(filtered.astype(np.uint8), 1.0 + (sharpness - 1.0), 
                                 blurred, -(sharpness - 1.0), 0).astype(np.float32)
    
    return np.clip(filtered, 0, 255).astype(np.uint8)

frame_count = 0
while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_count += 1

    # Anvend filter
    filtered_frame = apply_filters(frame, green_boost, contrast, brightness, gamma, saturation, vibrance, sharpness)
    display_frame = filtered_frame.copy()

    # Convert and create mask
    hsv = cv2.cvtColor(filtered_frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))

    # Process mask to reduce noise
    kernel = np.ones((5,5), np.uint8)
    mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)
    mask_clean = cv2.medianBlur(mask_clean, 7)

    # Count mask pixels for debugging
    white_pixels = np.sum(mask_clean == 255)
    total_pixels = mask_clean.shape[0] * mask_clean.shape[1]
    mask_percentage = 100 * white_pixels / total_pixels

    # Find contours
    contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    ball_found = False
    contour_info = []
    
    if contours:
        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area > 30:
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    
                    if circularity > 0.70:
                        (x, y), radius = cv2.minEnclosingCircle(contour)
                        if radius > 5:
                            cv2.circle(display_frame, (int(x), int(y)), int(radius), (0, 255, 0), 3)
                            cv2.putText(display_frame, f"BALL: A={int(area)} C={circularity:.2f} R={radius:.1f}", 
                                       (int(x-50), int(y-40)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            ball_found = True   
                            contour_info.append(f"BALL: A={int(area)} C={circularity:.2f} R={radius:.1f}")
                    else:
                        (x, y), radius = cv2.minEnclosingCircle(contour)
                        cv2.circle(display_frame, (int(x), int(y)), int(radius), (0, 0, 255), 1)
                        cv2.putText(display_frame, f"AVVIST: A={int(area)} C={circularity:.2f}", 
                                   (int(x-50), int(y+20)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                        contour_info.append(f"AVVIST: A={int(area)} C={circularity:.2f}")

    # Display HSV values
    cv2.putText(display_frame, f"HSV Min: ({h_min}, {s_min}, {v_min})", (10, 30), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    cv2.putText(display_frame, f"HSV Max: ({h_max}, {s_max}, {v_max})", (10, 50), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    cv2.putText(display_frame, f"Ball Found: {'YES' if ball_found else 'NO'}", (10, 80), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if ball_found else (0, 0, 255), 2)

    # Display mask statistics
    cv2.putText(display_frame, f"Mask: {mask_percentage:.1f}% ({white_pixels} px)", (10, 110), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
    cv2.putText(display_frame, f"Contours: {len(contours)}", (10, 130), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

    # Display filter values
    y_pos = 160
    cv2.putText(display_frame, f"Grønn: {green_boost:.2f}", (10, y_pos), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    cv2.putText(display_frame, f"Kontrast: {contrast:.2f}", (10, y_pos + 20), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    cv2.putText(display_frame, f"Lysstyrke: {brightness}", (10, y_pos + 40), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    cv2.putText(display_frame, f"Gamma: {gamma:.2f}", (10, y_pos + 60), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    
    cv2.putText(display_frame, f"Saturasjon: {saturation:.2f}", (200, y_pos), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    cv2.putText(display_frame, f"Vibrance: {vibrance:.2f}", (200, y_pos + 20), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    cv2.putText(display_frame, f"Skarpheit: {sharpness:.2f}", (200, y_pos + 40), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    # Display frame counter and FPS info
    cv2.putText(display_frame, f"Frame: {frame_count} (800x600)", (400, 30), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    cv2.imshow('Camera Tuning 800x600', display_frame)
    cv2.imshow('Mask', mask_clean)

    # Print debug info every 60 frames
    if frame_count % 60 == 0:
        print(f"\nFrame {frame_count}:")
        print(f"  Mask coverage: {mask_percentage:.1f}%")
        print(f"  Contours found: {len(contours)}")
        for info in contour_info:
            print(f"  {info}")

    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESC
        break
    
    # HSV-justeringar
    elif key == ord('q'): h_min = min(179, h_min + 1)
    elif key == ord('a'): h_min = max(0, h_min - 1)
    elif key == ord('w'): s_min = min(255, s_min + 5)
    elif key == ord('s'): s_min = max(0, s_min - 5)
    elif key == ord('e'): v_min = min(255, v_min + 5)
    elif key == ord('d'): v_min = max(0, v_min - 5)
    elif key == ord('r'): h_max = min(179, h_max + 1)
    elif key == ord('f'): h_max = max(0, h_max - 1)
    elif key == ord('t'): s_max = min(255, s_max + 5)
    elif key == ord('g'): s_max = max(0, s_max - 5)
    elif key == ord('y'): v_max = min(255, v_max + 5)
    elif key == ord('h'): v_max = max(0, v_max - 5)
    
    # Filter-justeringar
    elif key == ord('z'): green_boost = min(3.0, green_boost + 0.1)
    elif key == ord('x'): green_boost = max(0.1, green_boost - 0.1)
    elif key == ord('c'): contrast = min(3.0, contrast + 0.1)
    elif key == ord('v'): contrast = max(0.1, contrast - 0.1)
    elif key == ord('b'): brightness = min(100, brightness + 5)
    elif key == ord('n'): brightness = max(-100, brightness - 5)
    elif key == ord('m'): gamma = min(3.0, gamma + 0.1)
    elif key == ord(','): gamma = max(0.1, gamma - 0.1)
    elif key == ord('j'): saturation = min(3.0, saturation + 0.1)
    elif key == ord('k'): saturation = max(0.1, saturation - 0.1)
    elif key == ord('u'): vibrance = min(3.0, vibrance + 0.1)
    elif key == ord('i'): vibrance = max(0.1, vibrance - 0.1)
    elif key == ord('o'): sharpness = min(3.0, sharpness + 0.1)
    elif key == ord('p'): sharpness = max(0.1, sharpness - 0.1)

cap.release()
cv2.destroyAllWindows()

print("\n" + "="*60)
print("FINALE HSV-VERDIAR FOR 800x600:")
print(f"hsv_min = [{h_min}, {s_min}, {v_min}]")
print(f"hsv_max = [{h_max}, {s_max}, {v_max}]")
print("="*60)
print("FINALE FILTER-VERDIAR FOR 800x600:")
print(f"green_boost = {green_boost:.2f}")
print(f"contrast = {contrast:.2f}")
print(f"brightness = {brightness}")
print(f"gamma = {gamma:.2f}")
print(f"saturation = {saturation:.2f}")
print(f"vibrance = {vibrance:.2f}")
print(f"sharpness = {sharpness:.2f}")
print("="*60)
print("Kopiér desse nye 800x600 verdiane til Big_b.py!")
print("Dei gamle verdiane var for ein annan oppløsning!")
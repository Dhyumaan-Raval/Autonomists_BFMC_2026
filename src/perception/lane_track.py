import cv2
import numpy as np
import math
from collections import deque

# --- Stabilizers ---
lane_buffer = deque(maxlen=5)
angle_buffer = deque(maxlen=10)

def detect_pencil_lanes(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    # Adaptive Thresholding handles varying office light
    thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                   cv2.THRESH_BINARY_INV, 11, 2)
    kernel = np.ones((3,3), np.uint8)
    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    edges = cv2.Canny(opening, 50, 150)
    return edges

def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)
    # Focus on the lower 60% of the screen
    polygon = np.array([[(0, height), (0, int(height * 0.4)), 
                         (width, int(height * 0.4)), (width, height)]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    return cv2.bitwise_and(edges, mask)

def make_points(frame, line):
    height, _, _ = frame.shape
    slope, intercept = line
    y1 = height
    y2 = int(y1 * 0.6)
    # Prevent division by zero
    if slope == 0: slope = 0.001
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return [[x1, y1, x2, y2]]

def average_slope_intercept(frame, line_segments):
    if line_segments is None: 
        return list(lane_buffer)[-1] if lane_buffer else []

    height, width, _ = frame.shape
    left_fit, right_fit = [], []
    midline = width / 2

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2: continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope, intercept = fit[0], fit[1]
            
            if slope < -0.3:
                left_fit.append((slope, intercept))
            elif slope > 0.3:
                right_fit.append((slope, intercept))

    current_lanes = []
    if len(left_fit) > 0:
        current_lanes.append(make_points(frame, np.average(left_fit, axis=0)))
    if len(right_fit) > 0:
        current_lanes.append(make_points(frame, np.average(right_fit, axis=0)))
    
    if current_lanes:
        lane_buffer.append(current_lanes)
    return current_lanes

def get_steering_angle(frame, lane_lines):
    height, width, _ = frame.shape
    if len(lane_lines) == 2:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        x_offset = (left_x2 + right_x2) / 2 - (width / 2)
    elif len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    else:
        x_offset = 0
    
    y_offset = height / 2
    angle_to_mid_deg = math.degrees(math.atan(x_offset / y_offset))
    return angle_to_mid_deg + 90

def get_stabilized_steering(new_angle):
    angle_buffer.append(new_angle)
    return sum(angle_buffer) / len(angle_buffer)

def display_everything(frame, steering_angle, lane_lines):
    line_image = np.zeros_like(frame)
    for line in lane_lines:
        for x1, y1, x2, y2 in line:
            cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 10)
    
    angle_rad = math.radians(steering_angle)
    x1, y1 = int(frame.shape[1] / 2), frame.shape[0]
    
    # Safe tan calculation to avoid ZeroDivision
    tan_val = math.tan(angle_rad)
    if abs(tan_val) < 0.001: tan_val = 0.001
    
    x2 = int(x1 - (frame.shape[0]/2) / tan_val)
    y2 = int(frame.shape[0] / 2)
    cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 5)
    return cv2.addWeighted(frame, 0.8, line_image, 1, 1)

# --- Execution ---
video = cv2.VideoCapture(0)

while True:
    ret, frame = video.read()
    if not ret: break
    
    # Processing pipeline
    edges = detect_pencil_lanes(frame)
    roi = region_of_interest(edges)
    
    line_segments = cv2.HoughLinesP(roi, 1, np.pi/180, 40, np.array([]), 
                                    minLineLength=30, maxLineGap=100)
    
    lane_lines = average_slope_intercept(frame, line_segments)
    
    raw_angle = get_steering_angle(frame, lane_lines)
    smooth_angle = get_stabilized_steering(raw_angle)
    
    output = display_everything(frame, smooth_angle, lane_lines)

    cv2.imshow("Stark HUD - Lab Mode", output)
    
    if cv2.waitKey(1) & 0xFF == 27: # Press ESC to close
        break

video.release()
cv2.destroyAllWindows()

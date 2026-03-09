import cv2
import numpy as np
import math
import time
import threading
from collections import deque
from src.templates.workerprocess import WorkerProcess
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import SpeedMotor, SteerMotor
from src.statemachine.stateMachine import StateMachine
from src.brain.behaviors import BehaviorHandler

class processBrain(WorkerProcess):
    def __init__(self, queueList, logging, ready_event=None, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.ready_event = ready_event
        
        # --- P1M2 KINEMATIC PARAMETERS ---
        self.CRUISE_SPEED = 250  # mm/s
        self.MAX_STEER = 40.0    # Degrees
        self.DEADZONE = 0.0      # Ignore steer changes less than this
        
        self.FRAME_W, self.FRAME_H = 640, 480
        self.CENTER_X = self.FRAME_W // 2
        self.LD_ROI_Y = int(self.FRAME_H * 0.6) 
        
        self.lane_buffer = deque(maxlen=7)
        self.angle_buffer = deque(maxlen=4) 
        self.current_canny_low, self.current_canny_high = 50, 150
        
        self.Kp, self.Kd = 1.0, 0.4
        self.prev_error = 0.0
        self.dynamic_lane_width = 400.0
        
        # --- STATE TRACKING ---
        self.auto_start_time = None
        self.last_steer_sent = 0.0
        self.no_lane_counter = 0
        self.last_print_time = time.time()
        
        # --- ACTUATORS ---
        self.speed_sender = messageHandlerSender(self.queuesList, SpeedMotor)
        self.steer_sender = messageHandlerSender(self.queuesList, SteerMotor)
        
        # --- THE COMMAND INTERCEPTOR ---
        self.behavior_handler = BehaviorHandler()
        self.last_speed_sent = 0  
        self.last_cmd_time = 0.0
        
        # --- PHASE 2: YOLO SENTRY INIT ---
        self.current_detections = [] 
        self.latest_yolo_frame = None 
        self.yolo_status = "WAITING FOR CORE SHIFT..." 
        
        # We REMOVED the thread.start() from here!
        
        super(processBrain, self).__init__(self.queuesList, ready_event)

    def yolo_worker(self):
        """Runs continuously in the background, completely independent of the steering loop."""
        self.yolo_status = "LOADING MODEL..."
        
        try:
            from ultralytics import YOLO
            yolo_model = YOLO('/home/pi/Brain/models/best.pt')
            self.yolo_status = "WARMING UP..."
        except Exception as e:
            self.yolo_status = f"FATAL: {e}"
            self.logging.error(f"[ SENTRY FATAL ERROR ] : {e}")
            return 

        while True:
            try:
                if self.latest_yolo_frame is not None:
                    frame_to_process = self.latest_yolo_frame.copy()
                    self.latest_yolo_frame = None 
                    
                    yolo_crop = frame_to_process[0:268, 0:self.FRAME_W]
                    
                    # NO COLOR CONVERSION: The Vision queue is already RGB!
                    # Using .predict() to perfectly match your working test script
                    results = yolo_model.predict(yolo_crop, imgsz=320, conf=0.4, verbose=False)
                    
                    new_detections = []
                    for result in results:
                        for box in result.boxes:
                            cls_id = int(box.cls[0])
                            label = yolo_model.names[cls_id]
                            conf = float(box.conf[0])
                            coords = box.xyxy[0].tolist() 
                            
                            new_detections.append((label, conf, coords))
                            
                    self.current_detections = new_detections
                    self.yolo_status = f"DETECTED: {len(new_detections)} OBJECTS"
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.yolo_status = f"THREAD ERR: {e}"
                time.sleep(0.1)

    def run(self):
        sm = StateMachine.get_instance()
        if self.ready_event:
            self.ready_event.set()

        self.logging.info("[ BRAIN ] : P1M2 Active. Serial Comm Lines Open.")

        # --- IGNITE THE SENTRY ---
        # Now that we are safely inside the new process, we spin up the thread.
        self.yolo_thread = threading.Thread(target=self.yolo_worker, daemon=True)
        self.yolo_thread.start()

        while True:
            try:
                # 1. INTAKE (THE QUEUE DRAIN - ZERO LAG)
                frame = None
                while not self.queuesList["Vision"].empty():
                    try:
                        frame = self.queuesList["Vision"].get_nowait()
                    except Exception:
                        pass
                
                if frame is None:
                    try:
                        frame = self.queuesList["Vision"].get(timeout=0.1)
                    except Exception:
                        continue

                # --- FEED THE SENTRY ---
                self.latest_yolo_frame = frame

                # 2. PERCEPTION (HSV COLOR MASKING)
                ld_crop = frame[self.LD_ROI_Y:self.FRAME_H, 0:self.FRAME_W]
                blur = cv2.GaussianBlur(ld_crop, (5, 5), 0)
                hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
                
                lower_white = np.array([0, 0, 160])   
                upper_white = np.array([180, 50, 255])
                mask = cv2.inRange(hsv, lower_white, upper_white)
                edges = cv2.Canny(mask, 50, 150)

                edge_density = np.sum(edges == 255) / edges.size
                if edge_density > 0.03:
                    self.current_canny_low = min(self.current_canny_low + 2, 100)
                    self.current_canny_high = min(self.current_canny_high + 2, 250)
                elif edge_density < 0.01:
                    self.current_canny_low = max(self.current_canny_low - 2, 10)
                    self.current_canny_high = max(self.current_canny_high - 2, 100)

                line_segs = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=40, minLineLength=40, maxLineGap=100)
                lanes = self.average_slope_intercept(line_segs)
                x_offset = self.compute_offset(lanes)

                # 3. MATH & PID
                lookahead = self.FRAME_H * 0.3 
                raw_angle = math.degrees(math.atan2(x_offset, lookahead))
                self.angle_buffer.append(raw_angle)
                smooth_angle = sum(self.angle_buffer) / len(self.angle_buffer)

                error = smooth_angle 
                correction = (self.Kp * error) + (self.Kd * (error - self.prev_error))
                self.prev_error = error
                target_steer = np.clip(correction, -self.MAX_STEER, self.MAX_STEER)
                
                # --- THE COMMAND INTERCEPTOR ---
                # Pass vision and steering through the new behavior rules
                final_speed, final_steer, behavior_state = self.behavior_handler.process_commands(
                    self.current_detections, target_steer
                )

                # 4. HARDWARE EXECUTION
                current_mode = sm.get_mode().name
                if current_mode == "AUTO":
                    if self.auto_start_time is None:
                        self.auto_start_time = time.time()
                        self.is_cruising = False
                        self.last_steer_sent = 0.0
                        self.last_speed_sent = 0.0
                        self.last_cmd_time = time.time()
                        self.speed_sender.send("0")
                    
                    elapsed = time.time() - self.auto_start_time
                    if elapsed > 2.0:
                        if not lanes:
                            self.no_lane_counter += 1
                            if self.no_lane_counter > 10:
                                final_speed = 0  # Blind safety stop
                        else:
                            self.no_lane_counter = 0

                        # --- THE 10Hz THROTTLE FOR SPEED & STEER ---
                        if time.time() - self.last_cmd_time > 0.1:
                            if abs(final_steer - self.last_steer_sent) >= self.DEADZONE or final_speed != self.last_speed_sent:
                                self.steer_sender.send(str(int(final_steer * 10))) 
                                self.speed_sender.send(str(int(final_speed)))
                                
                                self.last_steer_sent = final_steer
                                self.last_speed_sent = final_speed
                                self.last_cmd_time = time.time()
                else:
                    self.auto_start_time = None  

                # 5. HUD GENERATION
                if "Hud" in self.queuesList:
                    hud_frame = self.draw_hud(frame, lanes, final_steer, x_offset, current_mode, self.current_detections)
                    ret, jpeg_buffer = cv2.imencode('.jpg', hud_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
                    if ret:
                        try:
                            if self.queuesList["Hud"].full():
                                self.queuesList["Hud"].get_nowait()
                            self.queuesList["Hud"].put_nowait(jpeg_buffer)
                        except Exception:
                            pass

            except Exception as e:
                pass
    # --- Helper Functions ---
    def average_slope_intercept(self, lines):
        if lines is None: return self.lane_buffer[-1] if self.lane_buffer else {}
        left_fit, right_fit = [], []
        for line in lines:
            for x1, y1, x2, y2 in line:
                if x1 == x2: continue
                y1_abs, y2_abs = y1 + self.LD_ROI_Y, y2 + self.LD_ROI_Y
                slope, intercept = np.polyfit((x1, x2), (y1_abs, y2_abs), 1)
                
                if abs(slope) < 0.3: continue
                if slope < 0: left_fit.append((slope, intercept))
                else: right_fit.append((slope, intercept))

        def make_pts(slope, intercept):
            y_bottom = self.FRAME_H 
            y_top = self.LD_ROI_Y
            
            x_bottom = int((y_bottom - intercept) / slope)
            x_top = int((y_top - intercept) / slope)
            return [[x_bottom, y_bottom, x_top, y_top]]

        lanes = {}
        if left_fit: lanes["left"] = make_pts(*np.mean(left_fit, axis=0))
        if right_fit: lanes["right"] = make_pts(*np.mean(right_fit, axis=0))
        if lanes: self.lane_buffer.append(lanes)
        return lanes

    def compute_offset(self, lanes):
        if not lanes: return 0.0
        
        if "left" in lanes and "right" in lanes:
            l, r = lanes["left"][0], lanes["right"][0]
            current_width = r[0] - l[0]
            if current_width > 100:  
                self.dynamic_lane_width = (0.8 * self.dynamic_lane_width) + (0.2 * current_width)
            
            mid_bottom, mid_top = (l[0] + r[0]) / 2, (l[2] + r[2]) / 2
            return ((mid_bottom + mid_top) / 2) - self.CENTER_X
            
        AGGRESSIVE_WIDTH = self.dynamic_lane_width * 0.90 
        
        if "right" in lanes: return (lanes["right"][0][0] - AGGRESSIVE_WIDTH) - self.CENTER_X
        if "left" in lanes: return (lanes["left"][0][0] + AGGRESSIVE_WIDTH) - self.CENTER_X
            
        return 0.0

    def draw_hud(self, frame, lanes, steer_angle, x_offset, mode, detections=[]):
        hud = frame.copy()

        # Draw Lanes
        for lane in lanes.values():
            for x1, y1, x2, y2 in lane:
                cv2.line(hud, (x1, y1), (x2, y2), (0, 255, 0), 4)

        # Draw Lane Targets
        cv2.line(hud, (0, self.LD_ROI_Y), (self.FRAME_W, self.LD_ROI_Y), (0, 255, 255), 1)
        cv2.line(hud, (self.CENTER_X, self.FRAME_H), (self.CENTER_X, self.LD_ROI_Y), (255, 255, 255), 1, cv2.LINE_AA)
        arrow_tip = (int(self.CENTER_X + x_offset * 0.5), int(self.FRAME_H * 0.65))
        cv2.arrowedLine(hud, (self.CENTER_X, self.FRAME_H), arrow_tip, (0, 0, 255), 4, tipLength=0.2)
        
        # --- DRAW YOLO BOXES ---
        try:
            for label, conf, coords in detections:
                x1, y1, x2, y2 = map(int, coords)
                cv2.rectangle(hud, (x1, y1), (x2, y2), (0, 0, 255), 2)
                text_y = y1 - 10 if y1 > 20 else y2 + 20
                cv2.putText(hud, f"{label} {conf:.2f}", (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        except Exception as e:
            # If the boxes fail to draw, print the error on the screen
            cv2.putText(hud, f"BOX ERR: {e}", (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Draw Status
        status_color = (0, 255, 0) if mode == "AUTO" else (0, 165, 255)
        cv2.putText(hud, f"SYSTEM: {mode}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        cv2.putText(hud, f"SENT STEER: {self.last_steer_sent:+.1f} deg", (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # --- SENTRY ON-SCREEN DIAGNOSTICS ---
        yolo_stat = getattr(self, 'yolo_status', 'WAITING FOR THREAD...')
        stat_color = (255, 255, 0) if "DETECTED" in yolo_stat else (0, 165, 255)
        cv2.putText(hud, f"SENTRY: {yolo_stat}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, stat_color, 2)
        
        return hud

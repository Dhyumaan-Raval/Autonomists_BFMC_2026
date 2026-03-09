import time

class BehaviorHandler:
    def __init__(self):
        # --- KINEMATIC TARGETS (mm/s) ---
        self.base_speed = 200.0
        self.motorway_speed = 400.0
        
        # --- STATE MEMORY ---
        self.current_target_speed = self.base_speed
        self.actual_speed = 0.0  
        self.last_update_time = time.time()

        # Priority 1: Emergency States
        self.obstacle_was_present = False
        self.obstacle_clear_time = 0.0

        # Priority 2: Stop Sign States
        self.stop_sign_active = False
        self.stop_sign_start = 0.0
        self.stop_sign_cooldown_end = 0.0  

        # --- THE PARKING SEQUENCER ---
        self.is_parking = False
        self.parking_step = 0
        self.parking_step_start = 0.0
        self.parking_cooldown_end = 0.0 # Prevents infinite loop re-parking

    def _next_park_step(self, current_time):
        """Helper to advance the parking sequence cleanly"""
        self.parking_step += 1
        self.parking_step_start = current_time

    def process_commands(self, detections, pilot_steer):
        """
        THE COMMAND INTERCEPTOR
        Evaluates YOLO vision against priority rules to authorize or override steering and speed.
        """
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        detected_labels = [d[0].lower() for d in detections]

# ==========================================================
        # ABSOLUTE PRIORITY 0: THE PARKING MANEUVER (God Mode)
        # ==========================================================
        if 'parking' in detected_labels and not self.is_parking and current_time > self.parking_cooldown_end:
            self.is_parking = True
            self.parking_step = 0
            self.parking_step_start = current_time

        if self.is_parking:
            elapsed = current_time - self.parking_step_start
            
            # Step 0: Forward 95cm @ 20cm/s (6.5 seconds)
            if self.parking_step == 0:
                if elapsed < 7.5: return 200, 0.0, "PARK 0: ALIGN FWD"
                else: self._next_park_step(current_time)

            # Step 1: Brake for 0.5s (ANTI-BROWNOUT PAUSE)
            elif self.parking_step == 1:
                if elapsed < 0.5: return 0, -40.0, "PARK 1: BRAKE"
                else: self._next_park_step(current_time)

            # Step 2: Turn Left Max, Reverse 55cm @ 15cm/s (3.67 seconds)
            elif self.parking_step == 2:
                if elapsed < 3.67: return -150, -40.0, "PARK 2: REV LEFT"
                else: self._next_park_step(current_time)

            # Step 3: Brake for 0.5s (ANTI-BROWNOUT PAUSE)
            elif self.parking_step == 3:
                if elapsed < 0.5: return 0, 40.0, "PARK 3: BRAKE"
                else: self._next_park_step(current_time)

            # Step 4: Turn Right Max, Reverse 55cm @ 15cm/s (3.67 seconds)
            elif self.parking_step == 4:
                if elapsed < 3.67: return -150, 40.0, "PARK 4: REV RIGHT"
                else: self._next_park_step(current_time)

            # Step 5: Stop, Wait for 2 seconds
            elif self.parking_step == 5:
                if elapsed < 2.0: return 0, 0.0, "PARK 5: IN SPOT WAIT"
                else: self._next_park_step(current_time)

            # Step 6: Turn Right Max, Forward 55cm @ 15cm/s (3.67 seconds)
            elif self.parking_step == 6:
                if elapsed < 3.67: return 150, 40.0, "PARK 6: EXIT RIGHT"
                else: self._next_park_step(current_time)

            # Step 7: Brake for 0.5s (ANTI-BROWNOUT PAUSE)
            elif self.parking_step == 7:
                if elapsed < 0.5: return 0, -40.0, "PARK 7: BRAKE"
                else: self._next_park_step(current_time)

            # Step 8: Turn Left Max, Forward 55cm @ 15cm/s (3.67 seconds)
            elif self.parking_step == 8:
                if elapsed < 3.67: return 150, -40.0, "PARK 8: EXIT LEFT"
                else: self._next_park_step(current_time)

            # Step 9: Final Brakes for 0.5 seconds
            elif self.parking_step == 9:
                if elapsed < 0.5: return 0, 0.0, "PARK 9: FINAL STOP"
                else:
                    self.is_parking = False
                    self.parking_cooldown_end = current_time + 10.0 

            if self.is_parking:
                return 0, 0.0, "PARKING: ERROR"


        # ==========================================================
        # PRIORITY 1: EMERGENCY OBSTACLE (Car, People)
        # ==========================================================
        if 'car' in detected_labels or 'people' in detected_labels or 'no entry' in detected_labels:
            self.obstacle_was_present = True
            self.obstacle_clear_time = 0.0  
            self.actual_speed = 0.0         
            return 0, pilot_steer, "EMERGENCY: OBSTACLE"

        if self.obstacle_was_present:
            if self.obstacle_clear_time == 0.0:
                self.obstacle_clear_time = current_time 
            
            if current_time - self.obstacle_clear_time < 2.0:
                self.actual_speed = 0.0     
                time_left = 2.0 - (current_time - self.obstacle_clear_time)
                return 0, pilot_steer, f"CLEARING: WAIT {time_left:.1f}s"
            else:
                self.obstacle_was_present = False
                self.obstacle_clear_time = 0.0

        # ==========================================================
        # PRIORITY 2: STOP SIGN (3 Seconds)
        # ==========================================================
        if 'stop' in detected_labels and not self.stop_sign_active and current_time > self.stop_sign_cooldown_end:
            self.stop_sign_active = True
            self.stop_sign_start = current_time
        
        if self.stop_sign_active:
            if current_time - self.stop_sign_start < 3.0:
                self.actual_speed = 0.0     
                time_left = 3.0 - (current_time - self.stop_sign_start)
                return 0, pilot_steer, f"STOP SIGN: WAIT {time_left:.1f}s"
            else:
                self.stop_sign_active = False
                self.stop_sign_cooldown_end = current_time + 5.0 

        # ==========================================================
        # PRIORITY 3: MOTORWAY (NON-BLOCKING SPEED RAMPING)
        # ==========================================================
        if 'priority road' in detected_labels:
            self.current_target_speed = self.motorway_speed
        elif 'end of motorway' in detected_labels:
            self.current_target_speed = self.base_speed

        if self.actual_speed < self.current_target_speed:
            self.actual_speed += 200.0 * dt
            if self.actual_speed > self.current_target_speed:
                self.actual_speed = self.current_target_speed
                
        elif self.actual_speed > self.current_target_speed:
            self.actual_speed -= 450.0 * dt
            if self.actual_speed < self.current_target_speed:
                self.actual_speed = self.current_target_speed

        # ==========================================================
        # PRIORITY 4: DEFAULT LANE FOLLOWING
        # ==========================================================
        state_str = "MOTORWAY" if self.current_target_speed == self.motorway_speed else "LANE FOLLOWING"
        
        if self.actual_speed < self.current_target_speed:
            state_str += " (ACCELERATING)"
        elif self.actual_speed > self.current_target_speed:
            state_str += " (BRAKING)"

        return int(self.actual_speed), pilot_steer, state_str

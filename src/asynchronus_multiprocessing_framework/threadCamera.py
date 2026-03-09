# ========================================================================================
# THE AUTONOMISTS - HIGH-SPEED VISION PORTAL (MJPEG STREAMER)
# ========================================================================================

import cv2
import threading
import picamera2
import time
import logging
from flask import Flask, Response

from src.utils.messages.allMessages import (
    mainCamera,
    serialCamera,
    Recording,
    Record,
    Brightness,
    Contrast,
)

from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import StateChange
from src.statemachine.systemMode import SystemMode

# --- FLASK STREAMER SETUP ---
LATEST_FRAME = None
FRAME_LOCK = threading.Lock()

# Suppress Flask's default terminal spam so it doesn't ruin your SSH Telemetry
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

app = Flask(__name__)

def generate_frames():
    """Generator function to yield JPEG frames for the web stream."""
    global LATEST_FRAME, FRAME_LOCK
    while True:
        with FRAME_LOCK:
            if LATEST_FRAME is None:
                frame = None
            else:
                frame = LATEST_FRAME.tobytes()
        
        if frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.01) # Wait for camera to warm up

@app.route('/')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
# ----------------------------


class threadCamera(ThreadWithStop):
    """Thread which handles camera capture and local Web Streaming."""

    def __init__(self, queuesList, logger, debugger):
        super(threadCamera, self).__init__(pause=0.001)
        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger
        self.frame_rate = 15 # Boosted framerate for smoother stream
        self.recording = False
        self.video_writer = ""

        # Senders
        self.recordingSender = messageHandlerSender(self.queuesList, Recording)
        # Note: Base64 senders left intact for compatibility, but bypassed in logic
        self.mainCameraSender = messageHandlerSender(self.queuesList, mainCamera)
        self.serialCameraSender = messageHandlerSender(self.queuesList, serialCamera)

        self.subscribe()
        self._init_camera()
        self.queue_sending()
        self.configs()

        # START THE VISION PORTAL (Runs in background)
        self.flask_thread = threading.Thread(
            target=lambda: app.run(host='0.0.0.0', port=4200, debug=False, use_reloader=False), 
            daemon=True
        )
        self.flask_thread.start()
        print(f"\n\033[1;96m[ VISION PORTAL ] : LIVE at http://<pi_ip>:4200/\033[0m\n")

    def subscribe(self):
        self.recordSubscriber = messageHandlerSubscriber(self.queuesList, Record, "lastOnly", True)
        self.brightnessSubscriber = messageHandlerSubscriber(self.queuesList, Brightness, "lastOnly", True)
        self.contrastSubscriber = messageHandlerSubscriber(self.queuesList, Contrast, "lastOnly", True)
        self.stateChangeSubscriber = messageHandlerSubscriber(self.queuesList, StateChange, "lastOnly", True)

    def queue_sending(self):
        if self._blocker.is_set():
            return
        self.recordingSender.send(self.recording)
        threading.Timer(1, self.queue_sending).start()

    def thread_work(self):
        if self.camera is None:
            time.sleep(0.1)
            return
            
        try:
            # 1. Capture Native Frame
            serialRequest = self.camera.capture_array("lores")  
            
            # 2. DISPATCH TO BRAIN (Non-Blocking)
            if "Vision" in self.queuesList:
                try:
                    if self.queuesList["Vision"].full():
                        self.queuesList["Vision"].get_nowait()
                    self.queuesList["Vision"].put_nowait(serialRequest)
                except Exception:
                    pass

            # 3. PULL FROM BRAIN (Non-Blocking)
            annotated_jpeg = None
            if "Hud" in self.queuesList:
                try:
                    while not self.queuesList["Hud"].empty():
                        annotated_jpeg = self.queuesList["Hud"].get_nowait()
                except Exception:
                    pass

            # 4. UPDATE WEB PORTAL
            global LATEST_FRAME, FRAME_LOCK
            if annotated_jpeg is not None:
                # Show AI HUD
                with FRAME_LOCK:
                    LATEST_FRAME = annotated_jpeg
            else:
                # Fallback: Show Raw Camera
                ret, jpeg_buffer = cv2.imencode('.jpg', serialRequest, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
                if ret:
                    with FRAME_LOCK:
                        LATEST_FRAME = jpeg_buffer

            if self._blocker.is_set():
                return

        except Exception as e:
            print(f"\033[1;97m[ Camera ] :\033[0m \033[1;91mERROR\033[0m - {e}")
    def state_change_handler(self):
        message = self.stateChangeSubscriber.receive()
        if message is not None:
            modeDict = SystemMode[message].value["camera"]["thread"]
            if "resolution" in modeDict:
                pass # Suppressing resolution change spam in terminal

    def _init_camera(self):
        try:
            if len(picamera2.Picamera2.global_camera_info()) == 0:
                print(f"\033[1;97m[ Camera Thread ] :\033[0m \033[1;91mERROR\033[0m - No camera detected.")
                self.camera = None
                return
            
            self.camera = picamera2.Picamera2()
            config = self.camera.create_preview_configuration(
                buffer_count=2, # Increased buffer slightly for stability
                queue=False,
                main={"format": "RGB888", "size": (1280, 720)},
                lores={"format": "RGB888", "size": (640, 480)}, # LOCKED TO 480p FOR FAST STREAMING
                encode="lores",
            )
            self.camera.configure(config)
            self.camera.start()
            print(f"\033[1;97m[ Camera Thread ] :\033[0m \033[1;92mINFO\033[0m - IMX708 Sensor Armed & Calibrated")
        except Exception as e:
            print(f"\033[1;97m[ Camera Thread ] :\033[0m \033[1;91mERROR\033[0m - Init Failed: {e}")
            self.camera = None

    def stop(self):
        if self.recording and self.video_writer:
            self.video_writer.release()
        if self.camera is not None:
            self.camera.stop()
        super(threadCamera, self).stop()

    def configs(self):
        if self._blocker.is_set():
            return
        threading.Timer(1, self.configs).start()

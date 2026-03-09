# ========================================================================================
# THE AUTONOMISTS - MAIN EXECUTIVE (RPi 5 OPTIMIZED)
# ========================================================================================

import sys
import time
import os
import psutil
import threading
import select  # NEW: For non-blocking SSH input
import tty     # NEW: For raw terminal mode
import termios # NEW: For capturing keystrokes over SSH
import signal
from multiprocessing import Queue, Event

# Pin to CPU cores 0-3 for RPi 5 stability
available_cores = list(range(psutil.cpu_count()))
psutil.Process(os.getpid()).cpu_affinity(available_cores)

sys.path.append(".")

from src.utils.bigPrintMessages import BigPrint
from src.utils.outputWriters import QueueWriter, MultiWriter
import logging
import logging.handlers

logging.basicConfig(level=logging.INFO)

# ===================================== PROCESS IMPORTS ==================================

from src.gateway.processGateway import processGateway
from src.hardware.camera.processCamera import processCamera
from src.hardware.serialhandler.processSerialHandler import processSerialHandler
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender # <-- ADDED SENDER
from src.utils.messages.allMessages import StateChange, SpeedMotor, SteerMotor, Klem, Brake # <-- ADDED BRAKE
from src.statemachine.stateMachine import StateMachine
from src.statemachine.systemMode import SystemMode

# ------ New component imports starts here ------#
from src.brain.processBrain import processBrain  # To be activated when Brain is built
# ------ New component imports ends here ------#

# ===================================== SHUTDOWN PROCESS =================================

def shutdown_process(process, timeout=1):
    """Helper function to gracefully shutdown a process."""
    process.join(timeout)
    if process.is_alive():
        print(f"The process {process} cannot normally stop, it's blocked somewhere! Terminate it!")
        process.terminate()  # force terminate if it won't stop
        process.join(timeout)  # give it a moment to terminate
        if process.is_alive():
            print(f"The process {process} is still alive after terminate, killing it!")
            process.kill()  # last resort
    print(f"The process {process} stopped")

# ===================================== PROCESS MANAGEMENT ===============================

def manage_process_life(process_class, process_instance, process_args, enabled, allProcesses):
    """Start or stop a process based on the enabled flag."""
    if enabled:
        if process_instance is None:
            process_instance = process_class(*process_args)
            allProcesses.append(process_instance)
            process_instance.start()
    else:
        if process_instance is not None and process_instance.is_alive():
            shutdown_process(process_instance)
            allProcesses.remove(process_instance)
            process_instance = None
    return process_instance 

# ===================================== SSH COMMANDER ====================================

def get_key(timeout=0.1):
    """Reads a single keypress from the SSH terminal non-blockingly."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        r, w, e = select.select([sys.stdin], [], [], timeout)
        if r:
            ch = sys.stdin.read(1)
        else:
            ch = None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    
    if ch == '\x03': # Catch Ctrl+C
        raise KeyboardInterrupt
        
    return ch

def keyboard_controller(queueList):
    """10Hz Terminal Command Center: True RC 'Hold-to-Steer' Logic"""
    sm = StateMachine.get_instance()
    
    klem_sender = messageHandlerSender(queueList, Klem)
    brake_sender = messageHandlerSender(queueList, Brake)
    speed_sender = messageHandlerSender(queueList, SpeedMotor)
    steer_sender = messageHandlerSender(queueList, SteerMotor)

    target_speed = 0.0
    target_steer = 0.0
    
    # Strictly adhering to Nucleo limits
    MAX_SPEED = 500.0 # mm/s
    MAX_STEER = 40.0  # Degrees

    # The Anti-Twitch Timer
    last_steer_time = time.time()

    print("\n" + "="*50)
    print("  AUTONOMISTS TERMINAL COMMANDER ACTIVE")
    print("  MODES  : [M] Manual | [O] Auto | [Q] STOP")
    print("  ENGINE : [1, 2, 3] Klem States")
    print("  DRIVE  : [W/S] Speed +/- 50 | [A/D] HOLD for Max Steer")
    print("  BRAKE  : [SPACEBAR] Zero Speed & Steer")
    print("="*50 + "\n")
    
    while True:
        try:
            cmd = get_key(timeout=0.1)
            
            if cmd:
                cmd = cmd.lower()
                # --- STATE & IGNITION COMMANDS ---
                if cmd == '1': 
                    klem_sender.send("0")
                    print("\n[INFO] Klem 0 (Power Off)")
                elif cmd == '2': 
                    klem_sender.send("15")
                    print("\n[INFO] Klem 15")
                elif cmd == '3': 
                    klem_sender.send("30")
                    print("\n[INFO] Engine Armed (Klem 30)")
                elif cmd == 'm': sm.request_mode("MANUAL")
                elif cmd == 'o': sm.request_mode("AUTO")
                elif cmd == 'q':
                    sm.request_mode("STOP")
                    klem_sender.send("0")
                    brake_sender.send("0.0")
                
                # --- WASD FLIGHT CONTROLS ---
                elif cmd == 'w':
                    target_speed = min(target_speed + 50.0, MAX_SPEED)
                elif cmd == 's':
                    target_speed = max(target_speed - 50.0, -MAX_SPEED)
                elif cmd == 'a':
                    target_steer = -MAX_STEER # Slam Left
                    last_steer_time = time.time()
                elif cmd == 'd':
                    target_steer = MAX_STEER  # Slam Right
                    last_steer_time = time.time()
                elif cmd == ' ': # Spacebar
                    target_speed = 0.0
                    target_steer = 0.0

            # --- AUTO-CENTER LOGIC ---
            # If 0.4 seconds pass without hearing 'A' or 'D', snap wheels to zero
            if time.time() - last_steer_time > 0.4:
                target_steer = 0.0

            # Print Telemetry cleanly (Fixed degree symbol error using 'deg')
            sys.stdout.write(f"\r Mode: {sm.get_mode().name} | Speed: {target_speed:>4.0f} mm/s | Steer: {target_steer:>3.0f} deg    ")
            sys.stdout.flush()

            # Transmit to Nucleo
            if sm.get_mode().name == "MANUAL":
                speed_sender.send(str(int(target_speed)))
                steer_sender.send(str(int(target_steer*10)))

        except KeyboardInterrupt:
            os.kill(os.getpid(), signal.SIGINT) 
            break
# ======================================== SETTING UP ====================================

print(BigPrint.PLEASE_WAIT.value)
allProcesses = list()
allEvents = list()

# Added Vision Queue (maxsize=1) for zero-latency frame passing to YOLO
queueList = {
    "Critical": Queue(),
    "Warning": Queue(),
    "General": Queue(),
    "Config": Queue(),
    "Log": Queue(),
    "Vision": Queue(maxsize=1), 
    "Hud": Queue(maxsize=1)
}
logger = logging.getLogger()

original_stdout = sys.stdout
original_stderr = sys.stderr

queue_writer = QueueWriter(queueList["Log"])
sys.stdout = MultiWriter(original_stdout, queue_writer)
sys.stderr = MultiWriter(original_stderr, queue_writer)

# ===================================== INITIALIZE =======================================

stateChangeSubscriber = messageHandlerSubscriber(queueList, StateChange, "lastOnly", True)
StateMachine.initialize_shared_state(queueList)

# Initializing gateway
processGateway = processGateway(queueList, logger)
processGateway.start()

# ===================================== INITIALIZE PROCESSES =============================

# Dummy event to bypass the Dashboard dependency for the Serial Handler
dummy_dash_ready = Event()
dummy_dash_ready.set() 

# Initializing camera
camera_ready = Event()
processCam = processCamera(queueList, logger, camera_ready, debugging=False)

# Initializing serial connection NUCLEO - > PI
serial_handler_ready = Event()
processSerialHandler = processSerialHandler(queueList, logger, serial_handler_ready, dummy_dash_ready, debugging=False)

# ------ New component initialize starts here ------#
brain_ready = Event()
processBrain = processBrain(queueList, logger, brain_ready, debugging=False)
# ------ New component initialize ends here ------#

# Adding all processes to the list
allProcesses.extend([processCam, processSerialHandler, processBrain])
allEvents.extend([camera_ready, serial_handler_ready, brain_ready])

# ===================================== START PROCESSES ==================================

for process in allProcesses:
    process.daemon = True
    process.start()

# ===================================== STAYING ALIVE ====================================

blocker = Event()
try:
    # wait for all events to be set
    logger.info("Waiting for Hardware Handshakes...")
    for event in allEvents:
        event.wait()

    # apply starting mode
    StateMachine.initialize_starting_mode()

    # Launch SSH Commander in a background thread
    cmd_thread = threading.Thread(target=keyboard_controller, args=(queueList,), daemon=True)
    cmd_thread.start()

    time.sleep(2)
    print(BigPrint.C4_BOMB.value)
    print(BigPrint.PRESS_CTRL_C.value)

    while True:
        message = stateChangeSubscriber.receive()
        if message is not None:
            # Camera logic toggling based on SystemMode can be mapped here later
            # For now, it simply observes the state changes safely.
            logger.info(f"Main loop detected Mode Change: {message}")
            
        blocker.wait(0.1)

except KeyboardInterrupt:
    print("\nCatching a KeyboardInterruption exception! Shutdown all processes.\n")

    for proc in reversed(allProcesses):
        proc.stop()
    processGateway.stop()

    # wait for all processes to finish before exiting
    for proc in reversed(allProcesses):
        shutdown_process(proc)
    shutdown_process(processGateway)
    sys.exit(0)

# Bosch Future Mobility Challenge (BFMC) — Team Autonomists

This repository contains the software architecture, embedded firmware, and mechanical designs developed by Team Autonomists for the Bosch Future Mobility Challenge. The project focuses on the development of a 1:10 scale autonomous vehicle capable of real-time perception, localized decision-making, and V2X communication.

---

## Technical Architecture

The system utilizes a distributed computing model, split between a high-level processing unit (Raspberry Pi/PC) and a low-level microcontroller (STM32 Nucleo).

### 1. Perception Module
* **Lane Detection (LD):** Computer vision algorithms for track boundary identification and ego-lane positioning.
* **Object Identification:** Convolutional Neural Networks (CNN) for real-time classification of traffic signs, obstacles, and pedestrian markers.

### 2. Control Module
* **State Machine:** High-level logic for intersection handling and traffic rule compliance.
* **Kinematic Modeling:** Calculation of required steering angles and longitudinal velocity.
* **Signal Processing:** PID control loops for stabilizing motor output and servo positioning.

### 3. Connectivity Module
* **V2X Communication:** Integration with the Bosch Environmental Server via MQTT/TCP-IP protocols.
* **Localization:** Processing of global positioning data and traffic light status provided by the server infrastructure.

### 4. Embedded Platform (Firmware)
* **Actuation:** Low-level PWM generation for the BLDC motor and steering servo.
* **Sensor Feedback:** High-frequency interrupt handling for rotary encoders and IMU data acquisition.

---

## Repository Structure

```text
Team_Autonomists/
├── src/                    # High-level processing (Python/C++)
│   ├── perception/         # Image processing and inference scripts
│   ├── control/            # Trajectory planning and decision logic
│   └── connectivity/       # Server-side communication scripts
├── firmware/               # STM32 Nucleo source code and binaries
├── 3dp/                    # CAD source files and STL geometries
├── models/                 # Trained weights and model definitions
├── docs/                   # Technical specifications and schematics
├── main.py                 # System integration and execution entry point
└── requirements.txt        # Dependency specification
```

---

## Team Autonomists

| Name | Role |
|------|------|
| Dhyumaan Raval | team lead, computer vision and AI/ML systems, vehicle control, CAD designer |
| Meet Jain | sensor fusion, microcontroller, communication protocols, github, firmware |
| Saumy Patel | mechanics optimization, sensor integration, vehicle control, CAD designer |
| Chintan Trivedi | data handling, communication protocols, data curator |
| Harshid Rawal | computer Vision and AI/ML systems, rpi related operations |



---


## Acknowledgments

Thanks to **Bosch Engineering Center Cluj** and the BFMC organizers for creating a platform that allows students to explore real-world autonomous driving challenges.


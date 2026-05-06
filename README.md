# 🌲 CPRE-288 — Tree Cutter AV

**Autonomous tree-cutting vehicle built on the ISU CyBot platform using a TM4C123GH6PM microcontroller.**

Final project for CPRE 288 — Embedded Systems I at Iowa State University.

---

## Overview

The Tree Cutter AV is a fully autonomous ground vehicle capable of navigating an environment, detecting obstacles (trees), and simulating a cutting operation. A companion desktop GUI allows an operator to monitor the robot in real time, send manual commands, and view a live-generated map of the scanned area.

---

## Features

### Autonomous Vehicle
- **IR-based distance sensing** — detects objects at close range using analog IR sensors with ADC
- **PING ultrasonic sensor** — measures precise distances for object classification (Timer3B edge-capture)
- **Servo-driven IR sweep** — rotates the sensor array to scan the full forward arc and build a distance profile
- **Autonomous navigation** — obstacle detection, approach, and avoidance logic with state-machine control
- **"Tree cutting" behavior** — identifies target objects within threshold distance and triggers the cutting routine

### GUI (Desktop Control Panel)
- Real-time serial communication with the CyBot over UART
- Manual drive controls (forward, reverse, turn left/right, stop)
- Live area map rendered from PING/IR scan data
- Object markers placed on the map when a target is detected
- Status display showing current robot state, distance readings, and mode

---

## Hardware

| Component | Description |
|---|---|
| **MCU** | TM4C123GH6PM (Tiva C Series) |
| **Platform** | ISU CyBot |
| **Distance (close)** | Analog IR sensors via onboard ADC |
| **Distance (far/precise)** | PING))) ultrasonic sensor (Timer3B edge-capture) |
| **Actuation** | DC drive motors via H-bridge, servo for sensor sweep |
| **Communication** | UART serial to host PC |
| **IDE** | Code Composer Studio (CCS) |

---

## Software Architecture

```
firmware/
├── main.c               # Top-level control loop & state machine
├── uart.c / uart.h      # UART driver for PC communication
├── ir_sensor.c/.h       # ADC-based IR distance reading
├── ping.c / ping.h      # PING ultrasonic driver (Timer3B)
├── servo.c / servo.h    # Servo sweep control
├── movement.c/.h        # Motor drive commands
├── scan.c / scan.h      # Area scan logic & data formatting
└── open_interface.c/.h  # CyBot Open Interface bindings

gui/
├── main.py              # GUI entry point
├── serial_comm.py       # Serial read/write handler
├── map_renderer.py      # Canvas-based map drawing
└── controls.py          # Button controls and command dispatch
```

---

## State Machine

```
IDLE → SCAN → NAVIGATE → APPROACH → CUT → SCAN (repeat)
                  ↓
             AVOID (if non-target obstacle detected)
```

- **IDLE** — waits for start command from GUI or button press
- **SCAN** — sweeps IR/PING array, builds distance profile, identifies objects
- **NAVIGATE** — drives toward the detected target
- **APPROACH** — slows and aligns as distance closes
- **CUT** — triggers cutting routine (motor/actuator sequence)
- **AVOID** — steers around non-target obstacles and resumes scan

---

## GUI

The operator GUI connects over a USB-UART serial link and provides:

- **Map view** — a top-down grid updated in real time from scan data; detected objects are marked and labeled
- **Drive controls** — WASD / on-screen buttons for manual override
- **Mode toggle** — switch between autonomous and manual modes without reflashing
- **Distance readout** — live PING and IR values during navigation
- **Log panel** — timestamped event feed (object detected, cut initiated, obstacle avoided, etc.)

---

## How to Build & Flash

### Firmware (CCS)
1. Open Code Composer Studio
2. Import the project from the `firmware/` directory
3. Build the project (`Ctrl+B`)
4. Connect CyBot via USB debugger
5. Flash and run (`F11`)

### GUI
```bash
cd gui
pip install pyserial tk
python main.py
```

Select the correct COM port when prompted. Default baud rate: **115200**.

---

## Team

Built for CPRE 288 — Embedded Systems I, Iowa State University.

---

## Notes

- Ensure the PING sensor's `volatile` flag is set on all ISR-shared variables to prevent compiler optimization bugs
- IR sensor readings require calibration per lighting environment — see `ir_sensor.c` for the lookup table
- The GUI map scale can be adjusted in `map_renderer.py` (`SCALE_FACTOR` constant)

# Embedded Sports Timing System

An ESP32-based sports timer developed for **DTU 02112 Embedded Systems Programming**, combining analog and digital timing elements with lap detection, environmental sensing, and remote-controlled race interaction.

## Overview

This project implements a **sports timing system** on an **ESP32-WROOM-32** using **C and ESP-IDF**.

The system combines multiple peripherals into one integrated prototype:

- a **stepper motor** acts as an analog-style seconds hand
- two **7-segment displays** show minutes
- an **HC-SR04 ultrasonic sensor** detects a runner or object crossing the start/finish line
- an **SSD1306 OLED display** shows system status, time, laps, and warning messages
- an **AM2320 sensor** measures temperature and humidity
- an **IR receiver + ELEGOO remote** provide user control
- a **piezo buzzer** and **RGB LED** provide feedback during countdowns, warnings, laps, and finish states

## Features

- ESP32-based embedded application written in C
- IR remote control for:
  - power on/off
  - race start
  - reset
  - stop
  - lap count configuration
- OLED UI with multiple states:
  - OFF
  - READY
  - COUNTDOWN
  - ARMED
  - RUNNING
  - STOPPED
  - FINISHED
  - SET LAPS
- Lap detection using an HC-SR04 ultrasonic sensor
- Temperature and humidity warning logic using AM2320
- Stepper motor timing output
- Dual 7-segment minute display via chained 74HC595 shift registers
- RGB LED status indication
- PWM buzzer feedback for countdown, warning, lap, and finish events

## Course Context

This repository contains the final project for **02112 Embedded Systems Programming** at the **Technical University of Denmark (DTU)**.

The project concept is a **sports timer** that combines both analog and digital timing elements in one embedded system.

## Hardware Used

- **ESP32-WROOM-32**
- **HC-SR04** ultrasonic sensor
- **SSD1306** OLED display
- **AM2320** temperature/humidity sensor
- **KY-022** IR receiver
- **ELEGOO IR remote**
- **2x 7-segment displays**
- **2x 74HC595** shift registers
- **RGB LED** (common anode)
- **PS1240 piezo buzzer**
- **28BYJ-48 stepper motor + ULN2003 driver**

## Software Stack

- **Language:** C
- **Framework:** ESP-IDF
- **Target:** ESP32
- **Build system:** CMake / `idf.py`


## Repository Structure

```text
.
├── README.md
├── Test/
│   ├── CMakeLists.txt
│   ├── .gitignore
│   ├── .clangd
│   ├── .devcontainer/
│   ├── .vscode/
│   └── main/
│       ├── CMakeLists.txt
│       └── main.c
└── docs/
    └── images/
        ├── system.jpg
        └── circuit-diagram.png

Current code is concentrated in `Test/main/main.c`, which contains:

- hardware initialization
- UI state handling
- IR decoding and event handling
- OLED drawing
- AM2320 reading
- HC-SR04 lap detection
- RGB LED control
- buzzer control
- 7-segment display driving
- stepper motor timing logic

## Pin Mapping

The following pin assignments are currently defined in `main.c`:

| Function | GPIO |
|---|---:|
| I2C SDA | 21 |
| I2C SCL | 22 |
| Ultrasonic TRIG | 18 |
| Ultrasonic ECHO | 19 |
| Buzzer | 14 |
| RGB Red | 16 |
| RGB Green | 13 |
| RGB Blue | 4 |
| Shift register DATA | 23 |
| Shift register CLOCK | 5 |
| Shift register LATCH | 17 |
| Stepper IN1 | 27 |
| Stepper IN2 | 26 |
| Stepper IN3 | 25 |
| Stepper IN4 | 33 |
| IR receiver | 32 |

I2C device addresses:

- OLED: `0x3C` or `0x3D`
- AM2320: `0x5C`

## Build and Flash

### Prerequisites

- ESP-IDF installed and exported in your shell
- ESP32 toolchain installed
- An ESP32 board connected over USB

### Build

```bash
cd Test
idf.py set-target esp32
idf.py build

Flash
idf.py -p <PORT> flash
Monitor serial output
idf.py -p <PORT> monitor

Example on Linux/macOS:

idf.py -p /dev/ttyUSB0 flash monitor

Example on Windows:

idf.py -p COM3 flash monitor
How the System Works
The system starts in POWER OFF mode.
The user turns it on using the IR remote.
The user can configure the target number of laps.
Pressing start begins a countdown on the OLED with buzzer and RGB feedback.
After GO!, the system enters an ARMED state.
When the ultrasonic sensor detects a crossing, the timer starts.
Each further detected crossing increments the lap counter.
When the configured lap target is reached, the race finishes and the system shows the final state.
The user can stop or reset the system from the remote.
User Interface / Feedback
OLED

The OLED shows:

system state
current race time
laps completed
warning messages
temperature and humidity
reaction time
RGB LED

The RGB LED is used for state feedback, including:

OFF
READY
COUNTDOWN
ARMED
RUNNING
STOPPED
FINISHED
warning blink behavior
Buzzer

The buzzer is used for:

countdown beeps
warning pattern
lap confirmation
finish pattern
Environmental Warning Logic

The AM2320 sensor is used to monitor track conditions.

Current warning thresholds in the code:

temperature warning at 30.0 °C
humidity warning at 40.0% RH

If either threshold is exceeded, the OLED shows a warning and the buzzer/RGB LED provide warning feedback before continuing.

Known Limitations

This is a student prototype and has several practical limitations:

The ultrasonic sensor can produce inconsistent triggers depending on distance, angle, reflection, clothing, and surrounding noise.
The stepper motor does not provide absolute position feedback, so homing is approximate.
The code is currently organized mainly in one large main.c file rather than separate modules.
Hardware setup is breadboard-based and intended as a prototype, not a robust product enclosure.
The exact ESP-IDF version used for the final project should be documented explicitly.
Future Improvements
Split the code into modules such as:
ir.c
oled.c
am2320.c
hcsr04.c
stepper.c
rgb.c
buzzer.c
ui.c
Replace or improve motion detection for more reliable lap sensing
Add clearer wiring documentation and schematic files
Improve code comments and reduce hardcoded values
Add demo media and architecture diagrams
Add a small CI build check for ESP-IDF
Authors

DTU 02112 Final Project, Group 5:

Emil M. Østergaard Knarreborg
Nitansh Thaker
Valdemar Kartikaya Mcclean
Rasmus Aasberg-Petersen

Reported contribution split in the final report:

Stepper motor code, hardware, debugging, and schematic: Valdemar
IR sensor, remote, and ultrasonic hardware/code: Nitansh
Shift registers / 7-segment display, general wiring, debugging, and schematic: Emil
I2C protocol and OLED display code: Rasmus

Notes

This repository is shared as a student course project and currently does not include an open-source license.

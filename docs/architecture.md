# Architecture

## High-Level System Flow

The sports timer is built around an ESP32 that coordinates multiple input and output components.

### Inputs
- IR receiver + remote for user commands
- HC-SR04 ultrasonic sensor for lap detection
- AM2320 sensor for temperature and humidity readings

### Outputs
- SSD1306 OLED display for status and timing information
- 2x 7-segment displays for minutes
- stepper motor for analog-style seconds indication
- RGB LED for visual feedback
- piezo buzzer for audio feedback

## Runtime Flow

1. The system starts in **POWER OFF** mode.
2. The user turns it on with the IR remote.
3. The user can configure the number of laps.
4. A countdown is shown on the OLED with buzzer and RGB feedback.
5. After `GO!`, the system waits for a crossing event.
6. The ultrasonic sensor starts the timer on the first crossing.
7. Additional crossings increment the lap count.
8. When the lap target is reached, the system finishes the race and displays the result.

## Current Firmware Structure

At the moment, the firmware is primarily implemented in a single file:

- `Test/main/main.c`

This file includes:
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

## Notes

This structure worked for the course project, but future cleanup could split the firmware into smaller modules once hardware testing is available again.
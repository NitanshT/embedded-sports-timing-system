# Hardware

This project was built around an ESP32-based sports timing prototype.

## Main Components

- ESP32-WROOM-32
- HC-SR04 ultrasonic sensor
- SSD1306 OLED display
- AM2320 temperature/humidity sensor
- KY-022 IR receiver
- ELEGOO IR remote
- 2x 7-segment displays
- 2x 74HC595 shift registers
- RGB LED
- PS1240 piezo buzzer
- 28BYJ-48 stepper motor with ULN2003 driver

## System Role of Each Component

- **ESP32**: central controller
- **Ultrasonic sensor**: detects lap crossings
- **OLED**: displays timer state, laps, warnings, and readings
- **AM2320**: provides temperature and humidity data
- **IR receiver + remote**: user input
- **7-segment displays**: minute display
- **Stepper motor**: analog-style seconds hand
- **RGB LED**: visual state feedback
- **Buzzer**: audio feedback

## Notes

This is a breadboard-based student prototype developed for DTU 02112 Embedded Systems Programming.
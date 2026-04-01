# Pinout

The following GPIO assignments are currently used in the project firmware.

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

## I2C Addresses

- OLED: `0x3C` or `0x3D`
- AM2320: `0x5C`
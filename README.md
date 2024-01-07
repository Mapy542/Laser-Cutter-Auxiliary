# Laser Cutter Auxiliary

Laser Control Program for K400 Laser Cutter.\
Arduino Mega 2560 Compatible.

Created by: Eli Bukoski

Arduino controls auxiliary features and bed lift. M2 Nano controls laser and laser power supply.

Auxiliary Features:

- Air Assist
- Water Chiller
- Water Pump
- Exhaust Fan
- Laser Manual Fire
- Cabinet LEDS

Bed Lift Modes:

- Auto Continuous Compensation
- Single Shot
- Manual

Work in Progress: Bed Lift PID
The control algorithm is basic and choppy.
Averaging sensor values would greatly improve the
output of the PID loop. VL6180X sensor is disappointingly inaccurate (+- 4 mm). However, it is the
only commodity sensor with a compatible range.

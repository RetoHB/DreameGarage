# Dreame robot vacuum automatic garage door
## Introduction
A WiFi-connected garage door controller that automatically opens and closes a garage door based on the state of a Dreame robot vacuum, using Adafruit IO as a cloud bridge and an ESP32-class microcontroller for real-time motor control.
The system is designed to be non-blocking, failsafe, and self-recovering, with OTA updates and a hardware watchdog to ensure continuous operation.
## General Description
The system uses an ESP-based microcontroller to control a DC motor through an L298N H-bridge. Motor movement is handled using a non-blocking state machine with controlled acceleration and deceleration, ensuring smooth and predictable motion.
On startup, the controller performs an automatic calibration cycle using a limit switch to establish a known reference position. This allows the door to recover correctly after resets or power interruptions.
The controller periodically reads the Dreame vacuum state from Adafruit IO, which is updated by Home Assistant. Based on this state, the door opens when the vacuum starts cleaning and closes when it returns to its base or enters charging mode, optionally after a configurable delay. Any manual command or state change immediately overrides pending actions.
A lightweight HTTP web interface provides local control and status monitoring, optimized for mobile devices. A hardware watchdog is used to automatically reset the controller if the main loop stalls, ensuring continued operation without user intervention.
## System Overview
### Control Flow
`Home Assistant → Adafruit IO → ESP Controller → L298N → DC Motor → Garage Door`
### Hardware components
- Electric tailgate trunk strut, pulleys and steel cable
- Xiao ESP32C3
- Joy-IT MotorDriver2
- Limit switch e.g. D2F-01L3
- 12V power supply
- Two drawer slides, shortened
### Wiring
<img width="800" alt="Dreame garage door schematic" src="https://github.com/user-attachments/assets/ac5dcd5c-bf36-4474-be62-4ccb120806eb" />

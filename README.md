# Dreame robot vacuum automatic garage door
## Introduction
A WiFi-connected garage door controller that automatically opens and closes a garage door based on the state of a Dreame Master robot vacuum, using Home Assistant, Adafruit IO as a cloud bridge and an ESP32-class microcontroller for real-time motor control.
## General Description
The system uses an ESP-based microcontroller to control a DC motor through an L298N H-bridge. Motor movement is handled using a non-blocking state machine with controlled acceleration and deceleration, ensuring smooth and predictable motion.
On startup, the controller performs an automatic calibration cycle using a limit switch to establish a known reference position. This allows the door to recover correctly after resets or power interruptions.
The controller periodically reads the Dreame vacuum state from Adafruit IO, which is updated by Home Assistant. Based on this state, the door opens when the vacuum starts cleaning and closes when it returns to its base or enters charging mode, optionally after a configurable delay.

A lightweight HTTP web interface provides local control and status monitoring. A hardware watchdog is used to automatically reset the controller if the main loop stalls, ensuring continued operation without user intervention.

The Dreame robot’s Master base is positioned lower than the surrounding floor level. To allow the robot to leave the base and return for reliable docking, an additional **ramp** is required. The ramp geometry ensures that the robot’s front wheel lowers early enough to achieve the correct docking angle.
The required 3D-print files for the ramp are included in this repository.
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
<br>
<br>

## Video and photos
### Video
[![Watch the video](https://img.youtube.com/vi/4hi8PueUa7Y/maxresdefault.jpg)](https://youtu.be/4hi8PueUa7Y)
<br>

<br>

<img width="500" alt="Dreame garage door controller" src="https://github.com/user-attachments/assets/e238ba79-9385-4cff-8f85-99c0bebd90c6" />
<img width="500" alt="Dreame garage door internal view" src="https://github.com/user-attachments/assets/86d4d1a6-bf0b-4e96-af54-8e6649bffd32" />

<video src="https://youtu.be/4hi8PueUa7Y" width=500/>

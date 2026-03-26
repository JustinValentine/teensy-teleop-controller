# Teleoperated Haptic CAN-FD Controller

This repository contains the Teensy 4.0 PlatformIO firmware for a custom, modular haptic controller carrier board. It is designed to act as a high-speed teleoperation input node on a CAN-FD network, specifically built to interface with mjbots Moteus controllers via a PC bridge.

## Overview

The controller leverages a PS5 trigger assembly and a 2-axis analog joystick to command a remote robotic gripper and rotational joints. Instead of static resistance, it features an admittance-based **"Smart Spring"** haptic feedback loop. The trigger dynamically changes its physical stiffness in real-time based on the torque/current data reported back from the remote Moteus controllers.


## Hardware Ecosystem

This firmware is designed to run on a custom carrier board with the following components:

- **Microcontroller:** Teensy 4.0
- **CAN Transceiver:** TI TCAN337GDR (3.3V Logic, 5 Mbps)
- **Main Haptic Driver:** TI DRV8833 Dual Motor Driver
- **Secondary Haptic Driver:** TI DRV2605L (I2C)
- **Inputs:** PS5 Trigger Module (Potentiometer + DC Motor), Adafruit 2-Axis Analog Thumb Stick, Tactile Bumper Switch

## Pinout & Wiring

| Component | Teensy 4.0 Pin | Notes |
| :--- | :--- | :--- |
| **DRV8833 (Motor A)** | `Pin 2` (PWM) | Pushes the trigger outward |
| **DRV8833 (Motor B)** | `Pin 3` (PWM) | Set to 0 (Grounded) |
| **Trigger Potentiometer** | `Pin 14` (A0) | |
| **Joystick X-Axis** | `Pin 15` (A1) | Includes software deadzone (±40 from 512) |
| **Joystick Y-Axis** | `Pin 17` (A3) | Includes software deadzone (±40 from 512) |
| **Bumper Button** | `Pin 16` | Configured as `INPUT_PULLUP` |
| **DRV2605L SDA** | `Pin 18` | Requires 4.7kΩ - 10kΩ pull-up to 3.3V |
| **DRV2605L SCL** | `Pin 19` | Requires 4.7kΩ - 10kΩ pull-up to 3.3V |
| **TCAN337 TXD** | `Pad 31` (CAN3) | Bottom pad on Teensy 4.0 |
| **TCAN337 RXD** | `Pad 30` (CAN3) | Bottom pad on Teensy 4.0 |

*Note: The TCAN337GDR `Silent` pin is hardwired locally to Teensy GND to ensure normal high-speed operation and protect against EMI.*

## Network Topology & Data Framing

Because the remote gripper is on an isolated CAN bus, a central PC acts as a bridge between the input controller and the Moteus network.

### Broadcasting (Teensy -> PC)

The controller broadcasts its physical state every 10ms (100Hz) on CAN ID `0x11`. 10-bit analog readings are split into High/Low 8-bit bytes to fit the CAN payload.

- `Byte 0/1`: Trigger Position (0-1023)
- `Byte 2/3`: Joystick X-Axis (0-1023)
- `Byte 4/5`: Joystick Y-Axis (0-1023)
- `Byte 6`: Bumper State (0 or 1)

### Listening (PC -> Teensy)

The controller listens for dynamic stiffness updates from the PC on CAN ID `0x22`.

- `Byte 0`: Stiffness Multiplier (0-255). `0` = moving through empty air (light spring). `255` = motor stalled/crushing (maximum pushback).


## Build & Upload (PlatformIO)

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

# Robotic Anti-Icing System (DT10)

Firmware for a senior-design autonomous anti-icing robot built on STM32F407 using STM32CubeIDE.

## Quick Overview

This project combines:

- Robot state management (`MANUAL`, `AUTO`, `PAUSE`, `ERROR`, `ESTOP`)
- Sensor stack (ICM-20948 IMU, GPS, HC-SR04 proximity)
- Safety layer (system health + watchdog + emergency stop)
- Mobility and actuation (Sabertooth drivetrain, salt/brine dispersion)
- Telemetry and remote control (LoRa)

## Repository Layout

- `Core/Inc`: Module interfaces (headers)
- `Core/Src`: Application logic and device drivers
- `Drivers`: STM32 HAL/CMSIS
- `Startup`: Startup assembly and linker support
- `Debug`, `Release`: STM32CubeIDE build outputs
- `Robotic Anti-Icing System.ioc`: CubeMX hardware/peripheral config

## Build & Flash

1. Open project in STM32CubeIDE.
2. Open `Robotic Anti-Icing System.ioc` and regenerate code if hardware config changed.
3. Build (`Debug` or `Release`).
4. Flash via ST-Link.

## Operational Notes

- Hardware watchdog is enabled during runtime.
- Proximity sensor effective range is software-capped to 250 cm.
- Test/diagnostic loops are watchdog-aware.

## Detailed Documentation

For full architecture, file-by-file responsibilities, boot/runtime flow, safety behavior, and integration details, see:

- `README_DETAILED.md`

## Repository

https://github.com/griffinMcConaha/Senior-Design-Project-DT10
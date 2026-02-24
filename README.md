# Robotic Anti-Icing System

Code home for our senior design project (DT10). This STM32CubeIDE firmware targets the STM32F407 and integrates sensor fusion, safety monitoring, proximity sensing, GPS navigation, LoRA telemetry, and Sabertooth motor control.

## What this repository contains

- **Core/**: Application sources and headers
  - `Src/`: main control loop, state machine, sensor drivers, diagnostics
  - `Inc/`: public headers for modules
- **Drivers/**: STM32 HAL/CMSIS drivers
- **Startup/**: Startup assembly and linker scripts
- **Debug/** and **Release/**: Build output folders from STM32CubeIDE
- **.ioc / .project / .cproject**: STM32CubeMX and STM32CubeIDE project configuration

## Key modules (Core/Src)

- `main.c`: System initialization and main control loop
- `robot_sm.c`: State machine (MANUAL/AUTO/PAUSE/ERROR/ESTOP)
- `robot_actions.c`: Manual and autonomous control behaviors
- `imu_icm20948.c`: ICM-20948 IMU driver (+ optional AK09916 mag)
- `heading_fusion.c`: IMU/GPS fusion for heading and attitude
- `proximity.c`: HC-SR04 ultrasonic proximity sensing
- `gps.c`: NMEA parsing and fix management
- `uart_lora.c`: LoRA telemetry/command link
- `sabertooth.c`: Sabertooth motor driver control
- `dispersion.c`: Salt/brine dispersion control
- `diagnostics.c`: On-device test menu and diagnostics
- `system_health.c`: Safety and health monitoring

## Build & flash

Open the .ioc file in STM32CubeIDE, generate code if needed, then build and flash from the IDE. Linker scripts are included for FLASH and RAM targets.

## Notes

- Watchdog is enabled; long-running loops must refresh it.
- Proximity range is software-capped to 250 cm for this platform.

## Repository

https://github.com/griffinMcConaha/Senior-Design-Project-DT10

# Mini Sumo Robot – Zephyr RTOS

This repository contains the firmware for a **Mini Sumo robot** developed using **Zephyr RTOS**, focused on embedded systems concepts such as real-time control, GPIO, PWM, and sensor integration.

## Features

* Zephyr RTOS–based embedded application
* Motor control using PWM (H-bridge)
* IR sensors for opponent and edge detection
* Modular code structure (motors, sensors, control logic)
* Target board: **STM32 Blackpill F401CC**

## Project Structure

```
├── src/            # Application source code
│   ├── main.c
│   ├── motor.c/.h
│   ├── sensors.c/.h
│   └── leds.c/.h
├── boards/         # Board-specific overlays
├── prj.conf        # Zephyr configuration
├── CMakeLists.txt
└── west-manifest/  # West workspace configuration
```

## Build & Debug

```bash
west build -b blackpill_f401cc
west debug
```

## Requirements

* Zephyr RTOS
* West tool
* ARM GCC toolchain
* STM32 programmer (ST-Link)

## Goal

Academic and practical study of **embedded systems** and **real-time operating systems** applied to autonomous robotics.

---

Developed for learning and experimentation with Zephyr RTOS.

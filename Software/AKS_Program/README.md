# AKS-Program

This directory contains the software for the Automatischer Kabelschneider (Automatic Cable Cutter).

## Requirements

- Download the `led-strip.h` library from the Espressif component registry: [led_strip v3.0.1](https://components.espressif.com/components/espressif/led_strip/versions/3.0.1)

## Overview

The code in this folder controls the automatic cable cutting process. Key features include:
- Interfacing with hardware components such as motors and sensors
- Controlling an LED strip for visual feedback
- Providing configuration options for cable length and quantity

## File Structure

- `main.c` (or similar): Main application logic for controlling the cable cutter
- `led-strip.h`: Required library for LED control (see above)
- Additional source and header files for hardware interface and user configuration

## Getting Started

1. Clone this repository.
2. Download and add `led-strip.h` to your project as described above.
3. Build and flash the firmware to your hardware using your preferred ESP32 development environment.

## Usage

- Configure your desired cable length and quantity in the software or via the user interface.
- Start the program to begin the automatic cutting process.
- The LED strip will provide status and progress feedback during operation.

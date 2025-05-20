# Automatic Cable Cutter – Software

## Overview

This directory contains the software for the Automatic Cable Cutter project.

- **Arduino File:** The Arduino file is intended exclusively for the display unit. It manages and controls the display and user interface.
- **AKS Program:** The AKS program is for the main unit of the cable cutter. It performs the main control functions for the cable cutting process.

## Structure

- `/Arduino/`  
  Code for the display unit (e.g., showing data, handling user input)
- `/AKS/`  
  Main program for controlling and automating the cable cutter

## Usage Notes

- Make sure to upload the Arduino firmware only to the display unit.
- The AKS program should be loaded onto the main control unit (e.g., microcontroller or control computer).
- Both programs are designed to work together, but are not interchangeable.

## Additional Information

- Please make changes or add features in separate branches and submit a pull request.
- If you have questions or encounter issues, create an issue in this repository.
- Documentation for hardware setup and wiring may be provided in additional files or the project Wiki.

---

**Note:** Any accidentally included information has been removed from this README.

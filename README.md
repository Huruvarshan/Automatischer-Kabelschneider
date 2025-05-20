# Automatischer Kabelschneider (Automatic Cable Cutter)

![Alt text](https://github.com/Huruvarshan/Automatischer-Kabelschneider/blob/main/Images/A6703971.JPG "a title")

## Overview

The **Automatischer Kabelschneider** is an open-source, automatic cable and heat shrink tube cutter. This project includes all hardware and software needed to build a programmable device for precise, automated processing of cables and tubes.

Designed for makers, professionals, and hobbyists, this tool streamlines cable and tube cutting, ensuring accuracy and efficiency in workshops or small production environments.

---

## Features

- **Programmable Length & Quantity**  
  Easily set the desired length and number of pieces using capacitive buttons and a built-in display.
- **Precision Cutting**  
  Clean, accurate cuts for various cable and tube types and diameters.
- **Material Flexibility**  
  Works with cables, heat shrink tubes, and similar materials.
- **Job Notification**  
  Visual indicators (LEDs or beacons) alert you when cutting is complete.
- **Safety**  
  Includes overheat protection, a “SawStop”-style safety system, and a run-out sensor.

---

## Repository Structure

```
Automatischer-Kabelschneider/
│
├── Hardware/
│   ├── PCB/           # PCB design files (KiCad)
│   ├── Mechanical/    # Mechanical design files
│   ├── BOM.md         # Bill of Materials
│   └── README.md      # Hardware documentation
│
├── Software/
│   └── AKS_Program/   # Microcontroller firmware and code
│       └── README.md  # Software usage and setup
│
├── images/
│   └── A6703971.JPG   # Project photo(s)
│
└── README.md          # Project overview (this file)
```

---

## Getting Started

### 1. Hardware

- Use the PCB and mechanical design files in `/Hardware` to manufacture and assemble your device.
- Reference the Bill of Materials (`Hardware/BOM.md`) for required components.
- Follow the assembly instructions in `Hardware/README.md`.

### 2. Software

- The microcontroller firmware is in `Software/AKS_Program/`.
- See `Software/AKS_Program/README.md` for setup and usage instructions.
- The software runs on an ESP32 microcontroller and includes code to control motors, sensors, and LED feedback.

### 3. Operation

- Set the desired cable/tube length and quantity via the device interface.
- Start the cutting process; LEDs provide progress and completion feedback.
- The device automatically stops and notifies you when the job is finished.

---

## Documentation

- **Hardware Details:** [Hardware/README.md](Hardware/README.md)
- **Software Instructions:** [Software/AKS_Program/README.md](Software/AKS_Program/README.md)

---

## Contributing

This project is open-source and welcomes contributions, bug reports, and suggestions!  
Please open an issue or submit a pull request.

---

## Safety & Disclaimer

- Always follow the included safety instructions before operating the device.
- The authors are not responsible for any injury or damage resulting from use or misuse of this project.

---

**Let’s build better tools together!**

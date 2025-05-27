# Automatischer Kabelschneider (Automatic Cable Cutter)

![Project in Action](https://github.com/Huruvarshan/Automatischer-Kabelschneider/blob/main/Images/A6703999.jpg "Automatic Cable Cutter")

## Overview

The **Automatischer Kabelschneider** is an open-source, programmable device for automatically cutting cables and heat shrink tubes to precise lengths and quantities. Designed for makers, professionals, and hobbyists, it streamlines repetitive workshop tasks, ensuring accuracy, safety, and efficiency in cable and tube preparation.

This repository includes all hardware and software resources needed to build, assemble, and operate the device.

---

## Features

- **Programmable Length & Quantity:**  
  Set the desired length and number of pieces using capacitive buttons and an integrated display.

- **Precision Cutting:**  
  Delivers clean, accurate cuts suitable for various cable and tube diameters.

- **Material Flexibility:**  
  Supports cables, heat shrink tubes, and similar flexible materials.

- **Job Notification:**  
  Visual indicators (LEDs or beacons) signal job progress and completion.

- **Safety:**  
  Built-in overheat protection, a "SawStop"-style emergency system, and run-out sensors to prevent accidents.

---

## Repository Structure

```
Automatischer-Kabelschneider/
├── Hardware/
│   ├── PCB/           # KiCad PCB design files
│   ├── Mechanical/    # Mechanical design files (e.g., 3D models)
│   ├── BOM.md         # Bill of Materials
│   └── README.md      # Hardware documentation
├── Software/
│   ├── AKS_Program/   # Microcontroller firmware (ESP32)
│   │   └── README.md  # Software setup and usage
│   └── README.md      # Software structure and notes
├── Images/            # Project and schematic images
│   └── A6703971.JPG
└── README.md          # Project overview (this file)
```

---

## Getting Started

### 1. Hardware

- Manufacture and assemble the PCB and mechanical parts using files in `/Hardware`.
- Reference the Bill of Materials (`Hardware/BOM.md`) for required components.
- Follow detailed assembly instructions in `Hardware/README.md`.

### 2. Software

- The main firmware for the microcontroller (ESP32) is located in `Software/AKS_Program/`.
- See `Software/AKS_Program/README.md` for build, installation, and configuration steps.
- The firmware controls motors, sensors, and LED feedback, and communicates with the device interface.

### 3. Operation

- Use the device's capacitive interface to configure cut length and quantity.
- Start the automatic cutting process. LEDs provide real-time feedback.
- The machine stops automatically and alerts you when the job is complete.

---

## Documentation

- **Hardware Details:** [Hardware/README.md](Hardware/README.md)
- **Software Instructions:** [Software/AKS_Program/README.md](Software/AKS_Program/README.md)

---

## Contributing

Contributions, bug reports, and suggestions are welcome!  
Please open an issue or submit a pull request via GitHub.

---

## Safety & Disclaimer

- Always follow safety instructions and observe caution while operating the device.
- The authors are not responsible for any injury or damage resulting from use or misuse of this project.

---

## License

This project is licensed under the MIT License.  
See [LICENSE](LICENSE) for details.

---

**Let’s build better tools together!**

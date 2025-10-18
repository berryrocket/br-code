# br-code


MicroPython code for BerryRocket-based board  
(“BerryRocket code”) ([github.com](https://github.com/berryrocket/br-code))

![License: CC BY-NC-SA](https://img.shields.io/badge/License-CC%20BY%20NC%20SA-green.svg)
![MicroPython](https://img.shields.io/badge/Language-MicroPython-blue)
![Platform: Raspberry Pi Pico](https://img.shields.io/badge/Platform-Raspberry%20Pi%20Pico-lightgrey)

[![BerryRocket Homepage](https://img.shields.io/badge/BerryRocket-Homepage-BC1544?style=for-the-badge&logo=rocket)](https://berryrocket.com)
[![BerryRocket Mini-Avionic Wiki](https://img.shields.io/badge/BerryRocket%20Mini%20Avionic-Wiki-BC1544?style=for-the-badge&logo=read-the-docs)](https://berryrocket.com/wiki/BR_Mini_Avionic)
[![BerryRocket Micro-Avionic Wiki](https://img.shields.io/badge/BerryRocket%20Micro%20Avionic-Wiki-BC1544?style=for-the-badge&logo=read-the-docs)](https://berryrocket.com/wiki/BR_Micro_Avionic)

## Overview

This repository contains the firmware and application logic for controlling a rocket’s avionics using MicroPython on a Raspberry Pi Pico (or equivalent board). The code handles:

- Flight sequencing (launch detection, apogee detection, parachute deployment)  
- Sensor acquisition (pressure, acceleration, angular velocity, etc.)  
- Payload operations (user‑customizable experiments, data logging)  
- Buzzer or audio feedback  
- Configuration and parameters  

It is intended for use in BerryRocket’s “Mini Avionic” / rocket avionics stack. ([berryrocket.com](https://berryrocket.com/wiki/BR_Mini_Avionic))

## Features

- Detect liftoff (via accelerometer or accel-contact)  
- Detect apogee / max altitude  
- Trigger parachute deployment  
- Read and log sensor data (IMU, pressure / temperature)  
- Buzzer / audio notifications  
- Configurable parameters for thresholds, timing, etc.  

## Repository Structure

```
.
├── .gitignore
├── LICENSE
├── README.md ← (this file)
├── board.py
├── buzzer.py
├── cu.py
├── main.py
├── parameters.py
└── lib/
    ├── icm20984.py
    ├── lps22.py
    └── (other sensor or utility modules)
```

Here’s a brief description of each important file:

| File / Module | Purpose |
|---|---|
| `main.py` | Rocket sequencer and payload logic — main entry point |
| `buzzer.py` | Buzzer control and notification routines |
| `cu.py` | Custom payload logic to be filled if needed |
| `parameters.py` | Configuration parameters, thresholds, constants |
| `lib/icm20984.py` | Driver for the ICM‑20984 IMU sensor |
| `lib/lps22.py` | Driver for the LPS22 pressure / temperature sensor |
| `lib/lsm6dsx.py` | Driver for the LSM6DSx IMU sensor |
| `lib/mpu9250.py` | Driver for the MPU9250 IMU sensor |

## Getting Started

### Prerequisites

- A BerryRocket-based board (e.g. Raspberry Pico + BR Interface + optionally BR Mini Sensor) ([berryrocket.com](https://berryrocket.com/wiki/BR_Mini_Avionic))  
- MicroPython firmware compatible with your board  
- A development environment (Thonny is a good choice for beginners) ([berryrocket.com](https://berryrocket.com/wiki/BR_Mini_Avionic))  

### Installation / Deployment

1. Flash MicroPython `.uf2` (or equivalent) to the board  
2. Upload all `.py` files from this repository onto the board (root level + `lib/`)  
3. Adjust `parameters.py` as needed (thresholds, sampling rates, timings)  
4. Reset / power-cycle the device to start execution  

### Configuration

- `parameters.py` contains all tunable constants (e.g. acceleration thresholds, delay times, data logging intervals)  
- You can modify behavior (e.g. how parachute is triggered) in `main.py`

### Usage

Once powered, the system:

1. Monitors acceleration or other signals to detect liftoff  
2. During ascent, reads sensors and logs data  
3. Wait for apogee time
4. Triggers parachute deployment (via servo, GPIO, etc.)  
5. Continues logging / optional payload operations until mission ends  

You can hook into or customize the payload operations inside `cu.py` or `main.py`.

## To go further ...

Here are a few ideas:

- Add new sensor drivers  
- Support additional board types  
- Improve stability / robustness / speed of sensor acquisitions
- Add logging/data upload support (e.g. via SD card, external flash, radio link)  

## License

This project is licensed under the **CC BY-NC-SA** (see `LICENSE`)

[![CC BY-NC-SA](https://user-images.githubusercontent.com/1367183/214925257-8b6ebb08-f1ee-49e5-85f3-be77d70f8bf6.png)](https://creativecommons.org/licenses/by-nc-sa/4.0/)

## References & Resources

[![BerryRocket Homepage](https://img.shields.io/badge/BerryRocket-Homepage-BC1544?style=for-the-badge&logo=rocket)](https://berryrocket.com)

[![BerryRocket Mini-Avionic Wiki](https://img.shields.io/badge/BerryRocket%20Mini%20Avionic-Wiki-BC1544?style=for-the-badge&logo=read-the-docs)](https://berryrocket.com/wiki/BR_Mini_Avionic)

[![BerryRocket Micro-Avionic Wiki](https://img.shields.io/badge/BerryRocket%20Micro%20Avionic-Wiki-BC1544?style=for-the-badge&logo=read-the-docs)](https://berryrocket.com/wiki/BR_Micro_Avionic)




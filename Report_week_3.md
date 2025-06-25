# Week #3

## Implemented MVP features

This week we completed the core MVP features focused on the Hardware and Firmware.

### Hardware

- Cart-Pole hardware assembly ([#27](https://github.com/IU-Capstone-Project-2025/total_control/issues/27))
  - Cart, rail, and pendulum arm assembled
  - Wiring completed and verified
  - End-button added and connected ([#28](https://github.com/IU-Capstone-Project-2025/total_control/issues/28))

- ESP32 physical integration ([#22](https://github.com/IU-Capstone-Project-2025/total_control/issues/22))
  - Mounted and connected to power supply and peripherals

- Case design and 3D modeling ([#20](https://github.com/IU-Capstone-Project-2025/total_control/issues/20), [#21](https://github.com/IU-Capstone-Project-2025/total_control/issues/21))

### Firmware & Backend

- Common firmware structure created ([#11](https://github.com/IU-Capstone-Project-2025/total_control/issues/11))
  - Shared firmware codebase for ESP32
    
- Init script implemented ([#12](https://github.com/IU-Capstone-Project-2025/total_control/issues/12))
  - Boots ESP32 and initializes pins and components

- State management logic added ([#25](https://github.com/IU-Capstone-Project-2025/total_control/issues/25))

### Software Design

- CartPole class structure implemented ([#14](https://github.com/IU-Capstone-Project-2025/total_control/issues/14))
  - Object-oriented design
  - Includes step(), reset(), and render() methods

- Repo structured and cleaned ([#1](https://github.com/IU-Capstone-Project-2025/total_control/issues/1))

### Functional User Journey

> A user connects the ESP32-powered cart-pole system, starts the firmware using the init script, and observes hardware movement while system states are logged for debugging. This demonstrates the full integration of software and hardware for a working MVP.

---

## Demonstration of the working MVP

TODO: Evgenii add Video

---

## Internal demo

- MVP reviewed by team with all hardware components assembled
- Firmware boot confirmed via serial output
- Class-based code design approved for extensibility

---

# Weekly commitments

## Individual contribution of each participant

Anastasia - Assist with hardware testing

Evgenii - 3D model and blueprints for controller container

Artyom - [Documentation via Sphinx](https://github.com/IU-Capstone-Project-2025/total_control/commit/760a3a5d4ea73bfbf63ce265c0089f90029d4c9a#diff-8ef404030a484d54db89ecc940a02d66d8cc4f55864528606d79e3f5547b433b)

Petr - [Write report](https://github.com/IU-Capstone-Project-2025/total_control/commit/5e8a2c6f1d7ce5efccc93700c9c3c01922524246)

Marat - [Firmware MVP](https://github.com/IU-Capstone-Project-2025/total_control/commit/921fc9ea2c93f5e39d8e4f7ab2d8c1fe815bc01e)


## Plan for Next Week

- Python library development
- CI/CD for documentation
- Hardware and Firmware documentation

## Confirmation of the code's operability

We confirm that the code in the main branch:
- [x] In working condition.
- [x] Run via docker-compose (or another alternative described in the README.md).

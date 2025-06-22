---
title: "Week #3"
---

# Week #3

## Implemented MVP features

This week we completed the core MVP features focused on the Cart-Pole system, which serves as the foundational element for more advanced systems like the double pendulum and butterfly mechanisms.

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
  - Structured for extension to multiple systems

- Init script implemented ([#12](https://github.com/IU-Capstone-Project-2025/total_control/issues/12))
  - Boots ESP32 and initializes pins and components

- State management logic added ([#25](https://github.com/IU-Capstone-Project-2025/total_control/issues/25))

### Software Design

- CartPole class structure implemented ([#14](https://github.com/IU-Capstone-Project-2025/total_control/issues/14))
  - Object-oriented design
  - Includes step(), reset(), and render() methods (compatible with RL algorithms)

- Repo structured and cleaned ([#1](https://github.com/IU-Capstone-Project-2025/total_control/issues/1))

### Functional User Journey

> A user connects the ESP32-powered cart-pole system, starts the firmware using the init script, and observes hardware movement while system states are logged for debugging or ML training. This demonstrates the full integration of software and hardware for a working MVP.

---

## Demonstration of the working MVP

---

## ML

Planned, not yet implemented.

- [#24](https://github.com/IU-Capstone-Project-2025/total_control/issues/24): Initial RL research underway. No training conducted yet.

---

## Internal demo

- MVP reviewed by team with all hardware components assembled
- Firmware boot confirmed via serial output
- Class-based code design approved for extensibility

---

# Weekly commitments

## Individual contribution of each participant

---

## Plan for Next Week

- Finalize firmware testing with sensor feedback
- Integrate RL training environment (simulator or live)
- Begin double pendulum hardware concept

---

## Confirmation of the code's operability

We confirm that the code in the main branch:
- [x] In working condition.
- [ ] Run via docker-compose (or another alternative described in the README.md).

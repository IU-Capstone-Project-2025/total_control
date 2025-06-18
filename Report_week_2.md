# Week #2

## Detailed Requirements Elaboration

This week our team focused on clarifying the key requirements for our platform, which aims to support educational and research activities in control theory, RL, trajectory shaping and robotics.

The objective is to develop a modular platform for simulating and controlling mechanical systems using embedded hardware, build both software (simulation, visualisation, control algorithms) and hardware (controllers and sensors) and to implement a first physical system: as a base class in class hierarchy that will include cart pole, double pendulum and butterfly systems in the future.  

### Prioritized backlog

[Link to the backlog](https://github.com/orgs/IU-Capstone-Project-2025/projects/17)

## Project specific progress

### Hardware

- It was decided empirically to use the esp32 controller to control the signals.
- The programs for testing the sensors and the motor were designed in a unified manner.
- All sensors and the motor were retested and now work properly.

### Software 

- The structure of the high-level library was outlined.
- The structure of the process of interaction between the controller and the user's computer via USB were discussed and outlined.
  
### Frontend

- The initial setup of the Sphinx project was completed to create project documentation in the form of a tree-like site.

### Minor chores

- The .gitignore file was generated in more detail.

# Weekly commitments

## Individual contribution of each participant

Anastasia - [Encoder test for ESP32]

Evgenii - [Python library structure](https://github.com/IU-Capstone-Project-2025/total_control/commit/81fd9744186f035bbc3ae94edd9649d1cfd495db)

Artyom - [Motor tests for ESP32](https://github.com/IU-Capstone-Project-2025/total_control/commit/f9f691522e77aa500441cd81726b9a6e3b78c9a4)

Petr - Write report and documentation

Marat - [Angle sensor test for ESP32]

## Plan for Next Week

- Combine sensor and motor tests into one file.
- Synchronize their operation using interrupts.
- Find the most suitable controller operating frequency.
- Add the ability to send signals to the controller to control the motor in real time.

## Confirmation of the code's operability

We confirm that the code in the main branch:
- [ ] In working condition.
- [ ] Run via docker-compose (or another alternative described in the `README.md`).

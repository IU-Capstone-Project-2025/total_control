---
title: "Week #2"
---


## Detailed Requirements Elaboration

This week our team focused on clarifying the key requirements for our platform, which aims to support educational and research activities in control theory, RL, trajectory shaping and robotics.

The objective is to develop a modular platform for simulating and controlling mechanical systems using embedded hardware, build both software (simulation, visualisation, control algorithms) and hardware (controllers and sensors) and to implement a first physical system: as a base class in class hierarchy that will include cart pole, double pendulum and butterfly systems in the future.  

### Prioritized backlog

https://github.com/orgs/IU-Capstone-Project-2025/projects/17

## Project specific progress

### Frontend

Early draft interface for user to set parameters using Sphinx.

### Backend

We implemented getters and setters for our state space equations and discrete time models as discussed. 


State space:

x' = Ax + Bu

y = Cx


Discrete time model:

x[k+1] = x[k] + u[k] * dt



f(x,u) system dynamics

getInitState() Returns initial state

getCurrentState() Returns current state

setSysParams() Loads the needed parameters from the sensors, like mass, length, angle etc.



|             | Soft                         | H.A. (Hardware Abstraction)  |
|-------------|------------------------------|------------------------------|
| I           | u or  θ'                     | x'                           |
| II          | (A, B, C), w = f(x)          | M, m, L, l, (A, B, C)        |

ESP32 board was selected and tested, USB connectivity was confirmed and GPIO mapping for motor and encoder is in planning phase.

Prototyping phase had started, physical cart pole setup being assembled

# Weekly commitments

## Individual contribution of each participant

*...*

## Plan for Next Week

Finalize cart pole hardware assembly and test with ESP32

Complete final function implementation

Extend backend to support basic RL experiments

Begin unit testing and simmulation integration

## Confirmation of the code's operability

We confirm that the code in the main branch:
- [ ] In working condition.
- [ ] Run via docker-compose (or another alternative described in the `README.md`).

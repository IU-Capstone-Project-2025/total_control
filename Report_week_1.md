---
title: "Week #1"
---

# Week #1

### Project name: *Total control*

**Code repository**: https://https://github.com/IU-Capstone-Project-2025/total_control



The Cart Pole system, also known as an inverted pendulum, is a control problem, where a pole is mounted on a wheeled cart that moves along a one-dimensional track. The goal is to control the movement of a cart such that the pole stays balanced around its vertical axis. The system is nonlinear and unstable, making it an excellent base for control theory, reinforcement learning and robotics.

The objective is to design a real-time control system that can dynamically balance an inverted pendulum mounted on a moving cart by controlling the cart position.


### **Team Members**

| Team Member                             | Telegram Alias   | Email Address                    | Track                                       | Responsibilities   |
|-----------------------------------------|------------------|----------------------------------|---------------------------------------------|--------------------|
| (Lead) Evgenii Shlomov                  | @mook003         | e.shlomov@innopolis.iniversity   | [Backend/Frontend/Fullstack/ML/Design/etc.] | [Responsibilities] |
| Artyom Tuzov                            | @artyomzifir     | a.tuzov@innopolis.iniversity     | [Backend/Frontend/Fullstack/ML/Design/etc.] | [Responsibilities] |
| Anastasia Malakhova                     | @stasia_hay      | an.malakhova@innopolis.university| [Backend/Frontend/Fullstack/ML/Design/etc.] | [Responsibilities] |
| Marat Shariev                           | @ficussss        | m.shariev@innopolis.university   | [Backend/Frontend/Fullstack/ML/Design/etc.] | [Responsibilities] |
| Petr Belayev                            | @pbel1           | p.belayev@innopolis.university   | [Backend/Frontend/Fullstack/ML/Design/etc.] | [Responsibilities] |



## Brainstorming

### Ideas during brainstorming

1. Educational Control System Kit
   - A modular hardware kit with software for teaching control systems, linear algebra, and system dynamics in universities.
   
2. RL & AI Research Platform
   - A benchmark system for reinforcement learning (RL) algorithms, allowing sim-to-real transfer and robustness testing.
   
3. Balance Control Testbed for Personal Transport Devices
   - A prototyping tool for companies developing self-balancing systems like Segways, monowheels, or hoverboards.
   
4. Digital Twin + Remote Lab for STEM Learning
   - An internet-connected cart pole with a digital twin, enabling students to run experiments remotely via a web interface.
   
5. Low-Cost Rehabilitation and Balance Training Robot
   - Adapt the inverted pendulum concept to assistive devices for posture training, with applications in physiotherapy.

### Brief market research / problem validation

Balance Control Testbed for Personal Mobility (Segways, Monowheels)
Problem:
Developing safe, robust control systems for personal mobility devices (e.g., Segways, electric unicycles) is challenging due to the cost and danger of real-world testing during early development.
Validation:
Companies like Segway-Ninebot, InMotion, and DIY e-mobility developers need small-scale testbeds to tune and validate control loops (pitch stabilization, tilt compensation).
Failures in self-balancing logic can lead to expensive hardware damage or injury; hence, early-stage prototyping on simplified analogs (like a cart-pole) is valuable.
The cart-pole is mathematically similar to self-balancing vehicle dynamics (inverted pendulum model).
Opportunity:
A scaled-down cart-pole system can serve as a safe development platform for core control algorithms (PID, LQR, Kalman filter, sensor fusion), before deploying them on full-size, high-inertia vehicles.


## Basic requirements

### Target users and their primary needs


The Cart Pole system serves as a versatile platform for control theory, robotics, and machine learning. It addresses the educational, research, and prototyping needs of a wide range of users. Below is a categorized list of key target users along with their primary motivations and use cases.

---

## 1. University Students (Engineering/Robotics)

Primary Needs:
- Understand real-world implementation of control algorithms.
- Perform hands-on experiments to complement theoretical coursework.
- Analyze system dynamics through physical modeling and simulation.

Use Case:
Students use the system during labs and final year projects to apply PID, LQR, and state estimation techniques.

---

## 2. Professors and Lab Instructors

Primary Needs:
- Provide safe, reliable, and repeatable lab experiments.
- Teach core concepts in control systems, feedback, and signal processing.
- Easily reset and monitor the system during student use.

Use Case:
Faculty integrate the cart-pole into structured lab modules or capstone projects for teaching applied control theory.

---

## 3. AI/ML Researchers

Primary Needs:
- Test reinforcement learning (RL) algorithms on real-world dynamic systems.
- Collect noisy, physical-world data to validate simulation results.
- Bridge the sim-to-real gap for policy transfer.

Use Case:
Researchers deploy and benchmark RL agents (e.g., DQN, PPO) to test robustness and sample efficiency.

---

## 4. Embedded Systems & Control Engineers

Primary Needs:
- Prototype real-time control loops on microcontrollers or FPGAs.
- Validate control firmware under physical constraints and disturbances.
- Integrate system into “hardware-in-the-loop” test environments.

Use Case:
Engineers use the cart-pole to verify embedded PID/LQR firmware for industrial or robotics applications.

---

## 5. Hobbyists and Makers

Primary Needs:
- Build a dynamic project using Arduino/Raspberry Pi.
- Experiment with sensors, actuators, and control logic.
- Engage with open-source designs and community contributions.

Use Case:
Makers assemble their own version of the system to explore DIY robotics and contribute improvements online.

---

## 6. Electric Vehicle Companies (e.g., Segway, Monowheels)

Primary Needs:
- Test and validate balance control algorithms for self-balancing vehicles.
- Use cart-pole as a simplified testbed for two-wheel or one-wheel dynamics.
- Rapidly prototype and simulate fall detection, recovery, and motor control strategies.

Use Case:
Companies use the cart-pole model to prototype early-stage control logic for personal transport systems like e-scooters, hoverboards, and monowheels.

---

## 7. EdTech Companies & STEM Curriculum Providers

Primary Needs:
- Provide scalable, engaging lab kits to educational institutions.
- Enable remote labs or digital twin access for distance learning.
- Offer modular kits that integrate with existing curriculum.

Use Case:
EdTech providers bundle the system with software and learning materials to deliver a complete “control systems lab-in-a-box.”

### User stories


User stories describe the goals and tasks that different types of users want to achieve using the Cart Pole system. These stories are organized by user type to inform feature development and prioritize use-case-driven design.

---

## University Students (Engineering/Robotics)

- As a student, I want to upload and test a PID or LQR controller so that I can see how different control strategies affect system stability.
- As a student, I want to collect and analyze real-time data so I can validate theoretical models with experimental results.
- As a student, I want to tune controller parameters (e.g., gain values) from a user interface so I can observe the effects without rewriting code.
- As a student, I want to simulate the cart-pole behavior before running it on hardware so I can avoid damaging the system.

---

## Professors and Lab Instructors

- As an instructor, I want to define pre-configured experiments so students can quickly start working without needing to set up the system from scratch.
- As an instructor, I want to remotely monitor student experiments so I can supervise labs in a hybrid or remote environment.
- As an instructor, I want to reset the system and apply limits to control parameters so I can ensure safety during classroom use.

---

##  AI/ML Researchers

- As a researcher, I want to train and test RL agents on the cart-pole hardware so I can evaluate real-world performance.
- As a researcher, I want to inject noise and disturbances to study the robustness of control algorithms.
- As a researcher, I want access to raw sensor data and control logs so I can analyze the learning process and failure cases.
- As a researcher, I want to compare sim-to-real performance so I can validate simulation-based methods in practice.

---

## 🛠 Embedded Systems & Control Engineers

- As an embedded engineer, I want to deploy my control firmware to the system so I can test real-time execution on microcontrollers or FPGAs.
- As an engineer, I want to run closed-loop control at high frequency (>50 Hz) so I can maintain stable performance under real conditions.
- As an engineer, I want access to a debug interface for monitoring variables in real-time so I can diagnose control issues.

---

## Hobbyists and Makers

- As a hobbyist, I want to assemble and customize the cart-pole system so I can learn about dynamics and control hands-on.
- As a hobbyist, I want to interface with Arduino or Raspberry Pi using open-source code so I can modify and extend the project.
- As a hobbyist, I want to follow detailed build guides and examples so I can replicate the system easily.

---

## EV & Micromobility Companies (e.g., Segway, Monowheels)

- As a vehicle control engineer, I want to use the cart-pole to prototype and validate balance algorithms in a safe, controlled environment.
- As a product developer, I want to simulate fall conditions and motor response so I can tune control systems for e-vehicles.
- As a team, we want to use the system to test variations of PID, sensor fusion, and failsafe recovery modes under different operating conditions.

---

## EdTech & STEM Lab Providers

- As an EdTech provider, I want to offer a ready-to-use cart-pole lab kit with documentation so schools can integrate it into STEM programs.
- As a digital learning provider, I want to connect users to the cart-pole remotely via the cloud so they can perform experiments online.
- As a curriculum designer, I want to map experiments to learning outcomes so educators can easily assess student performance.

### Initial scope

The initial scope of the Cart Pole Project is to design and implement a minimal viable product (MVP) that demonstrates real-time control of an inverted pendulum on a cart. The focus is on creating an educational and research-oriented system with core functionality, safety, and extensibility in mind.

---

## Objectives

- Develop a physical cart-pole system capable of balancing an inverted pole using real-time control.
- Implement fundamental control algorithms (e.g., PID and LQR).
- Provide real-time sensor data visualization and basic user interaction.
- Create a simulator for offline algorithm development and testing.
- Ensure system is modular, safe, and replicable for use in labs and learning environments.

---

## Hardware Features

- Cart and Rail System: Linear rail with motorized cart for 1D motion.
- Inverted Pole: Rigid pole mounted on a hinge, free to rotate in 1D plane.
- Sensors:
  - Cart position sensor (optical encoder or linear potentiometer).
  - Pole angle sensor (IMU or rotary encoder).
- Actuator: DC or BLDC motor with motor driver (e.g., L298N, BTS7960).
- Microcontroller: Raspberry Pi, Arduino, or STM32-based controller.
- Power Supply: Safe and sufficient for motor and control unit.
- Safety Mechanisms:
  - Physical end stops or limit switches.
  - Emergency shutdown button (optional).

---

## Software Features

- Control Loop:
  - Real-time control loop running ≥50 Hz.
  - Initial implementation of PID and LQR controllers.
- User Interface:
  - Desktop or web-based dashboard for:
    - Starting/stopping control
    - Viewing real-time graphs (pole angle, cart position)
    - Tuning control parameters
- Data Logging:
  - Save sensor and control data to CSV or JSON.
  - Timestamped logs for offline analysis.
- Offline Simulator:
  - Python or MATLAB-based simulator of the cart-pole dynamics.
  - Allows testing and tuning of control strategies before hardware deployment.

---

## Documentation

- Full assembly and wiring guide.
- Annotated code with comments and modular structure.
- Instructions for uploading code to the controller.
- Lab manual with 3–5 predefined experiments:
  - Open-loop falling behavior
  - Step response with PID
  - LQR stability analysis
- Troubleshooting checklist and safety guidelines.

---

## Out of Scope (Phase 1)

The following features are not included in the initial build but may be considered in future iterations:
- Reinforcement learning (RL) integration
- Cloud-based remote control or IoT features
- Wireless connectivity or mobile UI
- Industrial-grade casing/enclosure
- Multi-cart or multi-pole configurations

---

## Deliverables

- Functional cart-pole hardware prototype
- Controller software with real-time performance
- UI for visualization and parameter tuning
- Simulator for offline development
- Educational material and documentation


## Tech-stack

Microcontroller - STM32F407

Sensors:

Encoder - SEAVDAN H9730 

Raster - h9740 360 dpi

Angle sensor - Infineon TLE5012B

Motor driver - BTS7960 43A H-Bridge

Actuator - 12V or 24V gear motor with encoder (TBA)

Structure - Aluminum rail, custom cart, pole


# Weekly commitments

## Individual contribution of each participant

*...*

## Confirmation of the code's operability

We confirm that the code in the main branch:
- [ ] In working condition.
- [ ] Run via docker-compose (or another alternative described in the `README.md`).

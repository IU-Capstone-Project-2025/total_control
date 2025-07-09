

# Week #5

## Feedback

### Sessions

We conducted three feedback sessions with external users:

1. **1st year student** – Found the platform promising as an educational tool. Suggested adding better visualization for reinforcement learning progress.

2. **Graduate Student (Robotics)** – Tested the CartPole control interface. Recommended clearer examples for training agents using the provided API.

3. **Undergraduate (Control Theory)** – Emphasized the need for intuitive logging and understanding of real-time feedback from the system.

### Analyze

Key insights and actions:

| Feedback | Priority | Action Taken |
|----------|----------|--------------|
| Lack of real-time training feedback | High | Created issue to implement live training visualization |
| Onboarding too complex | Medium | Planned tutorial enhancements |
| Hardware timing unclear | Low | Will add logging of loop timing in a future iteration |

## Iteration & Refinement

### Implemented features based on feedback

- Added a working RL training loop based on real-time motor feedback
- Integrated structured control via `CartPole` abstraction

### Performance & Stability

We use standard Python libraries for development and performance/stability:

- **PyTorch** – For implementing and training the DQN agent
- **NumPy** – For efficient numerical operations
- **Matplotlib** (WIP) – Planned for visualizing training performance
- **serial / pyserial** – For hardware communication with the motor controller

Basic logging and exception handling are implemented. Training is currently stable for hardware episodes up to 300+ iterations.

### Documentation

All documentation is hosted at:  
**📘 https://iu-capstone-project-2025.github.io/total_control/**

The documentation contains:

- **Project Overview** – Architecture, communication flow, and control strategy
- **Quickstart Guide** – Instructions for running hardware and software components
- **API Reference** – Auto-generated documentation for interacting with the system

#### API Reference Structure:

- **LabDevice**  
  Base class for communication with lab hardware over serial interface. Provides methods for:
  - `connect()` / `disconnect()` for managing serial connection
  - Context manager support (`__enter__` / `__exit__`)
  
- **CartPole** *(inherits from LabDevice)*  
  Interface for controlling the Cart-Pole hardware:
  - `get_joint_state()` – Returns position and velocity as a string
  - `set_joint_efforts(effort)` – Applies a force to the cart (input: int or str)
  - `start_experimnet()` – Begins data flow and operation mode
  - `stop_experiment()` – Gracefully stops the experiment
  - `get_state()` – [WIP: internal state snapshot method]

This API is actively used by the Reinforcement Learning integration and will be further expanded to include reward shaping and safety checks.


### ML Model Refinement

We developed and integrated a Deep Q-Network (DQN) agent into the real CartPole system using the `CartPole` Python API. The RL agent observes position and velocity of the cart and applies motor effort as an action.

Improvements made:

- Switched to a real-time environment using `get_joint_state()` and `set_joint_efforts()` methods
- Refactored training loop to work asynchronously with hardware delays and serial communication
- Normalized the input state and clipped motor outputs to avoid unsafe operations
- Adjusted reward structure to favor longer balance time and penalize large accelerations

Planned refinements:

- Add reward shaping for smoother convergence
- Implement checkpoint saving/loading during training
- Tune hyperparameters: learning rate, epsilon decay, and discount factor
# Weekly commitments

## Individual contribution of each participant

  

## Plan for Next Week

- Add visual reward/loss graph to training loop  
- Validate the RL agent across multiple physical runs  
- Improve error handling for out-of-bounds behavior  
- Extend docs with ML training section and video tutorial (if possible)

## Confirmation of the code's operability

We confirm that the code in the main branch:
- [x] In working condition.
- [x] Run via docker-compose (or another alternative described in the README.md).

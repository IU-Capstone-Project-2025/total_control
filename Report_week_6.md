---
title: "Week #6"
---

# **Week #6**

## Links

- **Deployment**: TODO
- **API Docs**: https://iu-capstone-project-2025.github.io/total_control/
- **Design**: TODO
- **Demo**: TODO

## Final deliverables

### Project overview

Our project is a **Python-based control library** for interacting with and simulating a physical cart-pole mechanism. The library abstracts the complexity of serial communication and device management into a simple, user-friendly API.

The library supports both real hardware control and simulation environments, making it suitable for algorithm development and testing.

This solution targets engineers and researchers working with physical lab devices, helping them simplify the process of cart-pole control and experimentation.

### Features

- Control library (`inno_control`) for CartPole hardware interaction.
- Device connection management:
    - `connect()`
    - `disconnect()`
- Joint state retrieval:
    - `get_joint_state()` — returns positions and velocities.
- Force control:
    - `set_joint_efforts()` — sends effort/force commands to motors.
- Experiment control:
    - `start_experimnet()`
    - `stop_experiment()`
- Simulation capabilities:
    - Using same API interface for simulated cart-pole experiments (for algorithm testing without hardware).
- Public API documentation (via GitHub Pages).

### Tech stack

- **Python 3.10+**
- **Serial communication** via `pyserial`
- **NumPy** (optional, internal processing)
- **Sphinx** (for API documentation)
- **Docker** (optional, for packaging)
- **Markdown / GitHub Pages** (documentation hosting)

### Setup instructions

1. Clone repository:

```bash
git clone https://github.com/iu-capstone-project-2025/total_control.git
cd total_control
```
2. Install dependencies:
```bash
pip install -r requirements.txt
```

3. Connect cartpole via USB
   
4. Example usage:
  
```python

from inno_control.devices import CartPole

cart = CartPole('/dev/cu.usbserial-110')
cart.connect()
cart.start_experimnet()

state = cart.get_joint_state()
cart.set_joint_efforts(100)

cart.stop_experiment()
cart.disconnect()

```

5. Access API documentation:
   
https://iu-capstone-project-2025.github.io/total_control/

## Presentation draft

TODO

# Weekly commitments

## Individual contribution of each participant

TODO

## Plan for Next Week

Final project presentation.

Release library and documentation.

Optional: Extend simulation module.

## Confirmation of the code's operability

We confirm that the code in the main branch:
- [ ] In working condition.
- [ ] Run via docker-compose (or another alternative described in the `README.md`).

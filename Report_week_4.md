---
title: "Week #4"
---

# Week #4

## Testing and QA

We have started implementing tests for both the control system and the ML component. The strategy is to:
- Use unit tests for physics-based system classes (e.g. CartPole, EnvInterface)
- Test the RL agent’s interfaces (fit(), predict(), evaluate())
- Prepare for integration tests on simulated and physical setups (e.g., verifying response of hardware to predicted action)

All tests are written using pytest, and can be run via:
```
pytest tests/
```
### Evidence of test execution

<!-- Screenshots will go here later -->

## CI/CD

We configured GitHub Actions as our CI tool. It:
- Automatically runs unit tests on every push to the main and dev branches
- Lints the code using black and flake8
- Will later handle Docker image build and push for deployment

### Links to CI/CD configuration files

https://github.com/IU-Capstone-Project-2025/total_control/.github/workflows/docs.yml

https://github.com/IU-Capstone-Project-2025/total_control/.github/workflows/linters.yml


## Deployment

### Staging

A staging environment is being prepared on a local VDS. Current status:
- Docker container can run the backend and control logic
- The frontend connects to the backend for live control
- Currently tested with a simulation environment only

Commands used:

docker-compose up
Environment Variables:
- ENV=staging
- PORT=8000

### Production

*(Planned for Week #6, when the full stack is stable and safe for hardware control. Will be deployed to a VDS with a domain name and backup.)*

## Vibe Check

Progress: Team is moving forward with hardware design integration and ML. The DQN agent is functional in simulation, and environment interfaces are maturing.

Roadblocks: Hardware integration and accurate reward shaping are pending. Limited real-time feedback testing due to incomplete device interface.

Team dynamics: Good communication. Everyone has clear tasks, but we need to ensure syncing frontend/backend progress with the ML environment timeline.

# Weekly commitments

## Individual contribution of each participant


## Plan for Next Week

- Add real-time hardware interaction support  
- Finish deployment pipeline with automatic container builds  
- Tune hyperparameters of RL agent  
- Finalize reward shaping logic  
- Expand unit and integration test coverage

## Confirmation of the code's operability

We confirm that the code in the main branch:
- [x] In working condition.
- [x] Run via docker-compose (or another alternative described in the README.md).

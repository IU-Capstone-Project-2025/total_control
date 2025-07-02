---
title: "Week #4"
---

# Week #4

## Testing and QA

We have started implementing tests for both the control system and Hardware. The strategy is to:
- Use unit tests for physics-based system classes (e.g. CartPole, EnvInterface)
- Use hardware test to ckeck hardware system status.
- Verifying response of hardware to predicted action and check connectivity 

All tests for python lib are written using pytest, and can be run via in `inno_controll/` folder:
```
pytest tests/
```

or by docker:

```
docker compose up -build tests
```

### Evidence of test execution

<!-- Screenshots will go here later -->

## CI/CD

We configured GitHub Actions as our CI tool. It:
- Automatically runs unit tests on every push to the main and dev branches
- Lints the code using black and flake8
- Check style of documentation according to Google style
- Automaticly build documentation according new code structure
- Push this documentation on GitHub Pages

[Docs link](https://iu-capstone-project-2025.github.io/total_control/)

### Links to CI/CD configuration files

https://github.com/IU-Capstone-Project-2025/total_control/.github/workflows/docs.yml

https://github.com/IU-Capstone-Project-2025/total_control/.github/workflows/linters.yml

https://github.com/IU-Capstone-Project-2025/total_control/blob/main/.clang-tidy

https://github.com/IU-Capstone-Project-2025/total_control/blob/main/.ruff.toml

## Deployment

### Staging

For ease of use, there are two options for deploying software:

- A Docker image that is uploaded to the Docker Hub and automatically downloaded from the cloud to a local computer, where it is deployed using Docker Compose (this solution is primarily suitable for development, taking into account the use of additional tools for preparation, configuration, and management)

- A self-contained Python library that can automatically download additional dependencies and is currently being prepared for upload to the cloud for quick installation via Pip (currently only works locally)

### Production

*(Planned for Week #6, when the full stack is stable and safe for hardware control)*

## Vibe Check

Progress: Team is moving forward with hardware design integration and RL. The DQN agent is functional in simulation, and environment interfaces are maturing.

Roadblocks: Hardware integration and accurate reward shaping are pending. Limited real-time feedback testing due to incomplete API.

Team dynamics: Good communication. Everyone has clear tasks, but we need to ensure syncing frontend/backend progress with the ML environment timeline and documentation

General summary: All team members are very excited about the direction we are taking, especially the technologies we need to use and the future results

# Weekly commitments

## Individual contribution of each participant

- [Evgenii]() - Make PCB, box and tests for py
- [Artyom](https://github.com/IU-Capstone-Project-2025/total_control/commit/2d77f71c44007a1d604920881940c22b2d4c4f78) - Make CI/CD pipeline and docs
- [Anastasia](https://github.com/IU-Capstone-Project-2025/total_control/commit/ea22df737e36c3a01feb74946cd486dca1b6a630) - Read and write pipeline and data processing
- [Marat](https://github.com/IU-Capstone-Project-2025/total_control/commit/0273ab1e8bcf13460ab3e0583366fcf6cb4deb4d) - Firmware optimization and tests
- [Petr](https://github.com/IU-Capstone-Project-2025/total_control/commit/d87c531e82ee2dca2ba88e14a7f89c04cf4fd45b) - RL and Report


## Plan for Next Week

- Add real-time hardware data processing  
- Finish deployment pipeline with automatic container builds and docs processing 
- Tune hyperparameters of RL agent  
- Finalize reward shaping logic  
- Expand unit and integration test coverage
- Refactor entire code (Py, c++ and other)
- Imporove assembly

## Confirmation of the code's operability

We confirm that the code in the main branch:
- [x] In working condition.
- [x] Run via docker-compose (or another alternative described in the README.md).

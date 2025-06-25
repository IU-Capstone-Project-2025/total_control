# **Project 'Control'**  

🚀 *A software and hardware platform for studying **Control Theory, Reinforcement Learning (RL), Shapers, and Modern Robotics**.*  

---

## 📌 **Overview**  
This project provides a unified environment for:  
- **Real-world robotics experiments**  
- **Control Theory algorithms testing**  
- **Reinforcement Learning (RL) training**  
- **Hardware-in-the-Loop (HIL) testing**  

Supported platforms:  
- **Cart-pole** *(in work)*
- **Double pendulum** *(comming soon)*
- **Butterfly** *(comming soon)*

---

## 🛠 **Setup & Installation**  

### **Prerequisites**  
- **Docker** ([Install Guide](https://docs.docker.com/engine/install/))  


### **🚀 Quick Start with Docker**  

To open the documentation you need to run the following service
```bash
docker compose up --build get_docs
```

To run the test application, it is necessary to run the following service
```bash
docker compose up --build test
```

---

## 📂 **Project Structure**  
```
├── docker-compose.yaml      # Docker Compose configuration for multi-container setup
├── Dockerfile               # Docker image build instructions
├── docs/                    # Project documentation (Sphinx)
├── firmware/                # Device firmware code
│   ├── arduino/             # Arduino-specific firmware
│   └── shared/              # Shared firmware components
├── LICENSE                  # Project license file
├── python_lib/              # Python library package
│   ├── docs/                # Library documentation
│   ├── setup.py             # Python package configuration
│   ├── src/                 # Source code
│   │   ├── inno_control/    # Main package
│   │   │   ├── core.py      # Core functionality
│   │   │   └── __init__.py  # Package initialization
│   │   └── main.py          # Main application
│   └── tests/               # Unit tests
├── README.md                # Project overview and documentation
└── requirements.txt         # Python dependencies

```

---

## 🛠 **Development Guide**  

### **1. Local Development (Without Docker)**  
TO-DO
### **2. Development in Docker**

Run docker container
```bash
docker compose up --build terminal
```

Attach to the docker from other terminals, by running:
```bash
docker compose exec terminal bash
```

### **3. Build Docker**

After making changes to the dockerfile, you need to build a new image and upload it to the cloud

Build docker image
```bash
docker build -t image_name -f Dockerfile .
```

Tag docker image
```bash
docker tag image_name fabook/control:tag_name
```
Push docker image to Docker Hub (Optional)
```bash
docker push fabook/control:tag_name
```

> [!IMPORTANT]
> For reproducibility purposes, it is necessary to maintain the most current version of Docker on Docker Hub. But it is not necessary to call this command on every build.

> [!TIP]
> The tag can be anything, but by default docker compose calls the image with the tag 'latest'

### **5. Git & Commit Rules**  


### 🔹 **Core Tags (Commit Types)**  
| Tag         | Description                                                                 |
|-------------|----------------------------------------------------------------------------|
| **feat**    | A new feature. Example: `feat: add user authentication`                   |
| **fix**     | A bug fix. Example: `fix: resolve crash on null input`                    |
| **docs**    | Documentation changes. Example: `docs: update README.md`                  |
| **style**   | Code style/formatting (e.g., spaces, commas). Example: `style: format code per PEP8` |
| **refactor**| Code refactoring (no functional changes). Example: `refactor: optimize function X` |
| **perf**    | Performance improvements. Example: `perf: reduce data loading time`       |
| **test**    | Test-related changes. Example: `test: add API test coverage`              |
| **chore**   | Maintenance tasks (deps, configs). Example: `chore: update webpack`       |
| **ci**      | CI/CD changes (GitHub Actions, GitLab CI). Example: `ci: add staging deploy` |
| **build**   | Build system changes. Example: `build: add Dockerfile`                   |
| **revert**  | Reverts a previous commit. Example: `revert: undo commit 123abc`         |

### 🔹 **Additional Rules**  
1. **Message** should be clear and concise.  
   - ❌ Bad: `fix: bug`  
   - ✅ Good: `fix: prevent form submission error`  

2. **Body** (optional) — detailed description of changes.  
   ```  
   fix: fix memory leak in module X  

   The leak occurred due to unclosed DB connections during long sessions.  
   Added `cleanup()` to release resources properly.  
   ```  

3. **Footer** (optional) — issue links, breaking changes.  
   ```  
   feat: add WebSocket support  

   BREAKING CHANGE: Legacy `/chat` API is deprecated.  
   Closes #123  
   ```  

---

## 🤖 **Hardware Setup**  
### **Connecting a Real Robot**  
1. **Cart-pole**

TO-DO

---

## ❓ **Support & Contacts**  
- **Robotics Lab:** `Room 105a`  
- **Team contact:** [Evgenii Shlomov](@mook003)  

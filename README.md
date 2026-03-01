# 🏎️ RoboRacer Jetson-Sim Bridge

This project bridges a **Simulated F1TENTH Environment (x86)** with a **Jetson Orin Development Environment (ARM64)** using Docker and QEMU emulation. It allows for "Live Coding" where changes made on your host machine reflect instantly inside the emulated container.

## 🛠️ Prerequisites

Before starting, ensure your host machine has the following installed:

| Tool | Linux (Manjaro/Ubuntu) | Windows (WSL2) | Mac (M-Series) |
| :--- | :--- | :--- | :--- |
| **Docker** | `docker` & `docker-compose` | Docker Desktop | Docker Desktop |
| **Emulator** | `qemu-user-static` | `qemu-user-static` | Native (No QEMU needed) |
| **GPU** | NVIDIA Drivers + Toolkit | NVIDIA Windows Drivers | N/A (Runs on CPU) |
| **GUI** | `xhost` | WSLg (Already Integrated) | XQuartz |

---

## 🚀 Quick Start

### 1. Clone the Repositories
```bash
# Clone this repo
git clone <your-repo-url>
cd roboracer-project

# Clone the simulator (Must be in the root folder)
git clone [https://github.com/f1tenth/f1tenth_gym_ros.git](https://github.com/f1tenth/f1tenth_gym_ros.git)

```
### 2. Linux Users: Run this to allow the simulator window to open:
```bash
xhost +local:docker
```

### 3. Build & Launch
```bash
docker compose up -d --build
```

### 4. The "Magic Sync" (First Time Only)
You must build the ROS 2 workspace inside the racer container once to create the symlinks for live-coding. Run this in your host terminal:
```bash
docker exec -it roboracer-racer-1 bash -c "cd /opt/roboracer_ws && colcon build --symlink-install"
```

### 5. Running the simulator 
if you are in Linux, first do: 
```bash
xhost +local:docker
```

```bash
docker exec -it roboracer-racer-1 bash -c "cd /opt/roboracer_ws && colcon build --symlink-install"
```
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
git clone https://github.com/AaronClOr/roboracer.git
cd roboracer-project

# Clone the simulator (Must be in the root folder)
git clone https://github.com/f1tenth/f1tenth_gym_ros.git

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
Run the sim with this, this will create a terminal in the container of the simulator and then will run this commands: 

```bash
docker exec -it roboracer-racer-1 bash -c "cd /opt/roboracer_ws && colcon build --symlink-install"
```
This will pop up a window of the simulator on Rviz

### 6. Run the jetson container: 
On a new terminal, this will create a terminal inside the container: 
```bash
docker exec -it roboracer-racer-1 bash
```
Now that we are inside a terminal, we will be in the directory of "/opt/roboracer_ws" inside the container. We then source our workspace with: 

```bash
source install/setup.bash
```

Now we can check if we can see the topics, the simulator its publising with: 

```bash
ros2 topic list 
```

We can run a simple node for checking if we can subscribe and publish: 

```bash
ros2 run my_racer emergency_break 
```
Check the Rviz windows, it should move the car and then stop if there is a wall in front.

### 7. Close everything
Go to the termnial where you run the sim and just Ctr+c
in the terminal of the jetson orin, do the same for stopping the node and then type "exit". 

Since we did "docker compose up -d --build", this built and then run the containers in the background, so even if we do not have a process in the containers right now, the containers are still runing in the background. 
Stop the containers with: 
```bash
 docker compose down     
 ```

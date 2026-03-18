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
cd roboracer

# Clone the Auto drive simulator
git clone https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing.git

```

### 2. Build the simulator
Go inside "AutoDRIVE-RoboRacer-Sim-Racing" folder and build the simulator
```bash
docker build --tag autodriveecosystem/autodrive_roboracer_sim:latest -f autodrive_simulator.Dockerfile .

```

### 3. Build the ros brigde
Again, inside "AutoDRIVE-RoboRacer-Sim-Racing" folder, build the container for the ros bridge
```bash
docker build --tag autodriveecosystem/autodrive_roboracer_api:latest -f autodrive_devkit.Dockerfile .

```

### 4. Build the container for develpment
VSCode offers supports for containers develpment, using their tools is very practical because it allows us to debug inside the container so we are going to use it. 
1. Make sure you have install VS code
2. Add the extensions "Dev Containers" and "Containers Tools". 
3. To use it, we need a devcontainer.json which is already also in this repo. 

We only have to open the VS Code from the directory where this project is: 
```bash
cd Roboracer
code . 
```
 This will launch VSCode in this folder and because we have the devcontainer.json it will ask us if we would like to open as a container, and we accept. 

 it will build the container and once done, we are already inside the container ready for develpment. 



### 5. How to open simulator 

On a terminal, run these commands 
```bash
xhost local:root #For linux users
```

```bash
docker run --name autodrive_roboracer_sim --rm -it \
 --network=host \
 --ipc=host \
 -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
 --env DISPLAY \
 --privileged \
 --gpus all \
 --entrypoint /bin/bash \
 autodriveecosystem/autodrive_roboracer_sim:latest \
 -c "./AutoDRIVE\ Simulator.x86_64"
```
Now you should see the simulator on your screen. 



### 6. How to open ros bridge 

On a terminal, run these commands 
```bash
xhost local:root #For linux users
```

```bash
docker run --name autodrive_roboracer_api --rm -it \
 --network=host \
 --ipc=host \
 -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
 --env DISPLAY \
 --privileged \
 --gpus all \
 --entrypoint /bin/bash \
 autodriveecosystem/autodrive_roboracer_api:latest \
 -c "source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash && ros2 launch autodrive_roboracer bringup_graphics.launch.py"
```
Now you should see Rviz on your screen. 

### 6. How to open the container for develpment

Just open the folder where this repo is in VSCode and click on "reopen as a container". 
If your simulator and ros bridge are running, you should see the ros2 topics of the ros bridge in this container, just open a terminal with VScode (that terminal will be alreadt inside the container) and type ros2 topic list. 

Now you can develop in this workspace and everything will be store in your computer as well as in the container. 




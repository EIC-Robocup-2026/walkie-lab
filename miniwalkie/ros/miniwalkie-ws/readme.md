# MiniWalkie ROS 2 Workspace

This repository contains the ROS 2 workspace for the **MiniWalkie** onboard computer, developed as part of the EIC RoboCup 2026 project. It includes:

- 🦿 Robot Description (URDF, meshes, etc.)
- 🚀 Robot Bringup Packages
- 🧭 Sensor Integration (e.g., LiDAR)
- 🔌 Micro-ROS integration (for motor control, e.g., with walkie firmware, add later)

---

## 🛠️ Setup Instructions

### 1. Setup Micro-ROS and Build

First, set up the Micro-ROS workspace for communicating with the microcontroller.

```bash
# Set Micro-ROS workspace path
export MICROROS_WS_PATH="$HOME/microRos2ws"

cd
mkdir microRos2ws
cd $MICROROS_WS_PATH

git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```

### 2. Clone This Repo and Build

Now, clone and build this workspace.

```bash
# Set MiniWalkie workspace path
export MINIROCUP_WS_PATH="$HOME/minirobocup_ws"

# Clone the workspace
mkdir -p $minirobocup_ws_path/src
cd $minirobocup_ws_path/src
git clone https://github.com/EIC-Robocup-2026/miniwalkie-ws.git --recursive

# Install dependencies and build
cd $minirobocup_ws_path
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

### 3. Setup and Run the Robot!

Run the robot bringup program to start the robot's core systems.

```bash
# Run the robot bringup script
$MINIROCUP_WS_PATH/scripts/robot_bringup.sh
````

If you want the robot to start automatically on boot, set it up as a system service:

```bash
$MINIROCUP_WS_PATH/scripts/setup_robot_bringup_service.sh
```

---


## 📁 Repository Structure

This workspace currently contains two main components:

```
minirobocup_ws/
└── src/
    ├── miniwalkie_description/      # Robot URDF, meshes, etc.
    └── miniwalkie_bringup/          # Launch files, hardware integration (LiDAR, etc.)
```

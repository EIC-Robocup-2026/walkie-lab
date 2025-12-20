# Robot Arm MoveIt Ros2 Control

## Introduction

This repository contains a complete MoveIt2 configuration for a custom robotic arm with ROS2 control integration. The project provides motion planning, manipulation capabilities, and simulation support for robotic arm applications.

The project includes:

* **Robot Models**: URDF and Xacro files with mesh models (`.stl`) (mesh models still not work)
 **MoveIt Configuration**: Pre-configured motion planning setup with kinematics, controllers, and joint limits
* **ROS2 Control**: Topic-based hardware interface for seamless robot control
* **Python Commander**: Example scripts for programmatic robot control using MoveIt2 Python API
* **Simulation Ready**: Fully integrated with RViz for visualization and testing

### Tested on

- Ubuntu 24.04
- ROS Jazzy
- MoveIt2 (Jazzy distribution)

### Install Dependencies

Install required ROS2 packages by running the following command:

```bash
sudo apt install -y ros-jazzy-moveit \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  ros-jazzy-rviz2   
```

### Switching Between Mock Components and Topic-Based ROS2 Control

Changing ros2_control_hardware_type in my_robot_bringup/launch/my_robot_launch.xml

For Mock Components:

```bash
    ros2_control_hardware_type:=mock_components
```

For Topic-Based ROS2 Control:

```bash
    ros2_control_hardware_type:=isaac
```

## Usage

### 1. Visualize Robot in RViz

Launch the robot description to visualize the URDF model:

```bash
ros2 launch my_robot_description display.launch.xml
```

This will open RViz with the robot model and a joint state publisher GUI for manual joint control.

### 2. MoveIt Demo with Motion Planning

Launch the complete MoveIt demo including RViz motion planning interface:

```bash
ros2 launch my_robot_moveit_config demo.launch.py
```

Use the **MotionPlanning** panel in RViz to:
- Set goal poses using the interactive marker
- Plan trajectories to the goal
- Execute planned motions
- Save and load planning scenes

### 3. Control Robot Using Python Script

Start ROS2 Control + MoveIt:

```bash
ros2 launch my_robot_bringup my_robot_launch.xml
```

Run Python Script

```bash
ros2 run my_robot_commander_py test_moveit 
```

### 4. Control Robot Using Python Commander

Start ROS2 Control + MoveIt:

```bash
ros2 launch my_robot_bringup my_robot_launch.xml
```

Run the commander:

```bash
ros2 run my_robot_commander_py commander 
```

Example command to move to a pose:

```bash
ros2 topic pub -1 /pose_command my_robot_interfaces/msg/PoseCommand "{x: 0.0, y: 1.5, z: 5.0, roll: 0.0, pitch: 3.14, yaw: 0.0}"
```

Example command to move to a pose:

```bash
ros2 topic pub -1 /joint_command example_interfaces/msg/Float64MultiArray "data: [1.5,1.5,1.5,0.5,0.5,0.5]"
```

Example command to open/close the gripper (false = close, true = open):

```bash
ros2 topic pub -1 open_gripper example_interfaces/msg/Bool "data: false"
```

## Topic-Based ROS2 Control

This project uses **topic-based ros2_control** for hardware interface communication. The robot subscribes to joint command topics and publishes joint states, making it compatible with both simulation and real hardware.

### Key Topics

- `/MyArmControl/joint_states` - Joint state feedback from your hardware → ROS2 
- `/MyArmControl/joint_commands` - Joint command messages from ROS2 → your hardware
- `/joint_states` - Standard ROS 2 joint state topic published by joint_state_broadcaster
- `/joint_trajectory_controller/joint_trajectory` - Trajectory commands sent from MoveIt to ros2_control

### Controller Configuration

Controllers are defined in `my_robot_bringup/config/ros2_controllers.yaml` and include:

- **arm_controller** - For arm motion control
- **gripper_controller** - For gripper actuation
- **joint_state_broadcaster** - Publishes joint states

## Project Structure

```
├── my_robot_bringup
│   ├── CMakeLists.txt
│   ├── config
│   │   └── ros2_controllers.yaml
│   ├── launch
│   │   └── my_robot_launch.xml
│   └── package.xml
├── my_robot_commander_py
│   ├── my_robot_commander_py
│   │   ├── commander_template.py
│   │   ├── __init__.py
│   │   ├── test_moveit2.py
│   │   └── test_moveit.py
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
├── my_robot_description
│   ├── CMakeLists.txt
│   ├── launch
│   │   └── display.launch.xml
│   ├── meshes
│   │   ├── l1.stl
│   │   ├── l2.stl
│   │   ├── l3.stl
│   │   ├── l4.stl
│   │   └── l5.stl
│   ├── package.xml
│   ├── rviz
│   │   └── urdf_config.rviz
│   └── urdf
│       ├── armM.xacro
│       ├── arm.xacro
│       ├── common_properties.xacro
│       ├── gripper.xacro
│       ├── my_robot.ros2_control.xacro
│       └── my_robot.urdf.xacro
├── my_robot_interfaces
│   ├── CMakeLists.txt
│   ├── msg
│   │   └── PoseCommand.msg
│   └── package.xml
├── my_robot_moveit_config
│   ├── CMakeLists.txt
│   ├── config
│   │   ├── initial_positions.yaml
│   │   ├── joint_limits.yaml
│   │   ├── kinematics.yaml
│   │   ├── moveit_controllers.yaml
│   │   ├── moveit.rviz
│   │   ├── my_robot.ros2_control.xacro
│   │   ├── my_robot.srdf
│   │   ├── my_robot.urdf.xacro
│   │   ├── pilz_cartesian_limits.yaml
│   │   ├── ros2_controllers.yaml
│   │   └── sensors_3d.yaml
│   ├── launch
│   │   ├── demo.launch.py
│   │   ├── move_group.launch.py
│   │   ├── moveit_rviz.launch.py
│   │   ├── rsp.launch.py
│   │   ├── setup_assistant.launch.py
│   │   ├── spawn_controllers.launch.py
│   │   ├── static_virtual_joint_tfs.launch.py
│   │   └── warehouse_db.launch.py
│   └── package.xml
└── README.md
```

## Package Descriptions

### my_robot_description
Contains the URDF/Xacro robot model, mesh files, and visualization configuration.

### my_robot_moveit_config
MoveIt configuration package generated by the MoveIt Setup Assistant, including kinematics solvers, motion planning parameters, and controller configurations.

### my_robot_bringup
Launch files and configurations for bringing up the robot with ROS2 Control.

### my_robot_commander_py
Python scripts demonstrating programmatic control of the robot using the MoveIt2 Python API.

### my_robot_interfaces
Custom ROS2 message definitions for robot communication.

## Customization

### Modifying Robot Parameters

- **Joint Limits**: Edit `my_robot_moveit_config/config/joint_limits.yaml`
- **Kinematics**: Configure in `my_robot_moveit_config/config/kinematics.yaml`
- **Controllers**: Modify `my_robot_moveit_config/config/ros2_controllers.yaml`
- **Initial Pose**: Set in `my_robot_description/urdf/my_robot.ros2_control.xacro`

### Adding New Planning Groups

Use the MoveIt Setup Assistant to modify planning groups:

```bash
ros2 launch my_robot_moveit_config setup_assistant.launch.py
```

## Troubleshooting

**Issue**: Controllers not starting
- Check that all controller configurations are correct in `ros2_controllers.yaml`
- Verify controller_manager is running: `ros2 control list_controllers`

**Issue**: Motion planning fails
- Ensure robot model has valid collision
- Check joint limits are properly configured

**Issue**: Robot not moving in RViz
- Confirm controllers are loaded and active
- Check topic connections: `ros2 topic list`
- Verify joint_states are being published
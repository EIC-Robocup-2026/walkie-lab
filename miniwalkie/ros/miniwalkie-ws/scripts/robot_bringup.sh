#!/bin/bash

# ==========================================================
# This script is executed by the systemd service.
# It uses environment variables set in ~/.minirobocup_env.
# ==========================================================

echo "=================================================="
echo "    Starting Robot Bringup Process"
echo "=================================================="
echo "MiniRoboCup Workspace: $MINIROCUP_WS_PATH"
echo "Micro-ROS Workspace: $MICROROS_WS_PATH"

# Source the necessary ROS2 and workspace setup files
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$MINIROCUP_WS_PATH/install/setup.bash"
source "$MICROROS_WS_PATH/install/setup.bash"

# Now, you can use the paths to run your ROS2 commands
echo "Starting ROS2 launch file..."

ros2 launch robot_bringup real_omnibot.launch.py


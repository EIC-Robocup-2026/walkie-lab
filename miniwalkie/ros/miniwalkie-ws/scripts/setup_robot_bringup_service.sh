#!/bin/bash

# ==========================================================
# This script sets up the MiniRoboCup workspace environment variables
# and creates a systemd service that uses them.
# ==========================================================

# --- CONFIGURATION ---
# Set the absolute paths to your workspaces here.
# Change these values to match your specific setup.
minirobocup_ws_path="$HOME/minirobocup_ws"
microros_ws_path="$HOME/microRos2ws"
# ---------------------

# Define file paths
SERVICE_NAME="robot_bringup"
USER_NAME="$USER"
ENV_FILE="$HOME/.minirobocup_env"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"

echo "=================================================="
echo "        MiniRoboCup Environment Setup"
echo "=================================================="

# --- 1. Set up Environment Variables ---
echo "Configuring environment variables..."

# Write the environment variables to a file
echo "Creating environment file: $ENV_FILE"
cat > "$ENV_FILE" <<EOL
# MiniRoboCup environment variables
export ROS_DOMAIN_ID=23
export MINIROCUP_WS_PATH="$minirobocup_ws_path"
export MICROROS_WS_PATH="$microros_ws_path"
EOL

# Source the file in the user's shell profile to make it permanent
if [ -f "$HOME/.bashrc" ]; then
    grep -qF "source $ENV_FILE" "$HOME/.bashrc" || echo "source $ENV_FILE" >> "$HOME/.bashrc"
    echo "Added 'source $ENV_FILE' to ~/.bashrc"
elif [ -f "$HOME/.zshrc" ]; then
    grep -qF "source $ENV_FILE" "$HOME/.zshrc" || echo "source $ENV_FILE" >> "$HOME/.zshrc"
    echo "Added 'source $ENV_FILE' to ~/.zshrc"
else
    echo "Could not find .bashrc or .zshrc. Please manually add 'source $ENV_FILE' to your shell profile."
fi

# Make the script executable
chmod +x "$minirobocup_ws_path/src/scripts/robot_bringup.sh"

# --- 2. Create systemd service ---
echo "Creating systemd service for $SERVICE_NAME..."

# Write service file, referencing the environment file
sudo bash -c "cat > $SERVICE_FILE" <<EOL
[Unit]
Description=Robot Bringup ROS2 Service
After=network.target

[Service]
User=$USER_NAME
WorkingDirectory=$minirobocup_ws_path
# ExecStart now sources the environment file before running the script
ExecStart=/bin/bash -c "source $ENV_FILE && \$MINIROCUP_WS_PATH/src/scripts/robot_bringup.sh"
Restart=always
RestartSec=5
Environment=ROS_DOMAIN_ID=23

[Install]
WantedBy=multi-user.target
EOL

# Reload systemd daemon
echo "Reloading systemd..."
sudo systemctl daemon-reload

# Enable service at boot
echo "Enabling service..."
sudo systemctl enable $SERVICE_NAME.service

echo "✅ Setup complete!"
echo "Please run 'source $ENV_FILE' or open a new terminal to apply the changes."
echo "You can now start the service with:"
echo "  sudo systemctl start $SERVICE_NAME"
echo "Check status with:"
echo "  systemctl status $SERVICE_NAME"

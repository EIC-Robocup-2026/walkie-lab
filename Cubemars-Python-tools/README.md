# Cubemars Python Tools

This repository contains Python tools and examples for interacting with Cubemars motors. It provides a Python API to communicate with the motors and includes demonstration scripts for various functionalities.

## Installation & Setup

1. **Clone the repository.**
2. **Create and activate a virtual environment:**
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   ```
3. **Install the Cubemars Python API in editable mode:**
   ```bash
   pip install -e ./Cubemars-Python-API
   ```

## Directory Structure

- `Cubemars-Python-API/`: Contains the core Python API for controlling Cubemars motors.

## Scripts and Examples

### Control Examples

- `multi_motor_keyboard_input.py`: An example script demonstrating how to control multiple Cubemars motors using keyboard input. This can be useful for testing motor responses and basic control.

### Utilities

- `set_motor_origin.py`: A utility script to set the origin (zero) position of a Cubemars motor. This is crucial for proper calibration and control of the motor's absolute position.

- `can_listener.py`: A TUI tool to listen to the CAN bus, discover CubeMars motors, and display their feedback in a real-time table. This is useful for debugging and identifying motors on the network.
  - **Usage**: `python3 can_listener.py [OPTIONS]`
  - **Options**:
    - `--interface`: CAN interface type (e.g., `socketcan`, `pcan`, `gs_usb`). Default: `socketcan`.
    - `--channel`: CAN channel name (e.g., `can0`, `PCAN_USBBUS1`). Default: `can0`.
    - `--bitrate`: CAN bus speed in bits/s. Default: `1000000`.
  - **Example**: `python3 can_listener.py --interface socketcan --channel can0`

- `can_setup.sh`: A helper script for Linux to bring the CAN interface up or down.
  - **Usage**: `bash can_setup.sh {up|down} [interface] [bitrate]`
  - **Example**: `bash can_setup.sh up can0 1000000`

## Operating Instructions (Calibration & Movement)

Follow these steps to calibrate the motors and set the origin before using higher-level control software like MoveIt:

1. **Power on the robot.**
2. **Bring the CAN interface up:**
   ```bash
   bash can_setup.sh up
   ```
3. **Discover motors (Optional but Recommended):**
   Run the `can_listener.py` script to verify that all motors are powered on and detected on the CAN bus. Note their IDs.
   ```bash
   python3 can_listener.py
   ```
4. **Set initial origin:** Run `set_motor_origin.py` to set the initial zero position for all motors.
5. **Move motors to target joint positions:**
   Use `multi_motor_keyboard_input.py` to move each joint.
   - **For j1-j4:** These have a 10:1 gear ratio. To move 10 degrees, send a command for 100 (10 * target_degree).
   - **For j5-j8:** Input the target joint degree directly.
6. **Finalize Origin:** Once all motors are in their physical origin positions, run `set_motor_origin.py` again to save the final origin.
7. **Verify:** Use `multi_motor_keyboard_input.py` to test the movement and ensure everything is calibrated correctly.

Further instructions on setting up the environment and running these scripts would typically be found within the `Cubemars-Python-API/` directory or within each script's docstrings.

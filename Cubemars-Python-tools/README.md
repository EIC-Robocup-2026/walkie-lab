# Cubemars Python Tools

This repository contains Python tools and examples for interacting with Cubemars motors. It provides a Python API to communicate with the motors and includes demonstration scripts for various functionalities.

## Directory Structure

- `Cubemars-Python-API/`: This directory contains the core Python API for communicating with Cubemars motors. It provides the necessary classes and functions to control and read data from the motors.
- `scripts/`: Contains utility shell scripts for system configuration.

## Example Scripts

- `multi_motor_keyboard_input.py`: An example script demonstrating how to control multiple Cubemars motors using keyboard input. This can be useful for testing motor responses and basic control.
- `set_motor_origin.py`: A utility script to set the origin (zero) position of a Cubemars motor. This is crucial for proper calibration and control of the motor's absolute position.

## Utility Scripts

- `scripts/can_setup.sh`: A helper script to bring the CAN interface up or down.
  - Usage: `./scripts/can_setup.sh {up|down} [interface] [bitrate]`
  - Simple usage: `./scripts/can_setup.sh up` (defaults to `can0` at `1000000` bps)
  - Example: `./scripts/can_setup.sh up can0 1000000`

## Operating Instructions (Calibration & Movement)

Follow these steps to calibrate the motors and set the origin before using higher-level control software like MoveIt:

1. **Power on the robot.**
2. **Bring the CAN interface up:**
   ```bash
   ./scripts/can_setup.sh up
   ```
3. **Set initial origin:** Run `set_motor_origin.py` to set the initial zero position for all motors.
4. **Move motors to target joint positions:**
   Use `multi_motor_keyboard_input.py` to move each joint.
   - **For j1-j4:** These have a 10:1 gear ratio. To move 10 degrees, send a command for 100 (10 * target_degree).
   - **For j5-j8:** Input the target joint degree directly.
5. **Finalize Origin:** Once all motors are in their physical origin positions, run `set_motor_origin.py` again to save the final origin.
6. **Verify:** Use `multi_motor_keyboard_input.py` to test the movement and ensure everything is calibrated correctly.

Further instructions on setting up the environment and running these scripts would typically be found within the `Cubemars-Python-API/` directory or within each script's docstrings.

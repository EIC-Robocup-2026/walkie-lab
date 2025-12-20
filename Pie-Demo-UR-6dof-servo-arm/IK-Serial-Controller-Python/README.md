# Demo Arm Controller and Inverse Kinematics

A real-time keyboard controller for a UR-style robotic arm with inverse kinematics support. This project enables intuitive control of a 6-DOF robotic arm using keyboard inputs and is designed to interface with ESP32 via PySerial for wireless servo control.

![Demo Arm](../Demo%20Arm%20Image.png)

## Overview

This project provides a comprehensive real-time control system for a UR (Universal Robots) style robotic arm with:
- **Real-time Inverse Kinematics (IK)** - Jacobian matrix pseudo-inverse solver with convergence threshold
- **6-DOF Arm Control** - Full Cartesian XYZ movement with joint angle tracking
- **Gripper Management** - Gripper angle control (0-90°) and roll rotation
- **Keyboard Interface** - Non-blocking real-time keyboard input handling
- **State Persistence** - Automatic save/load of arm position to JSON for recovery after reboot
- **Live 3D Visualization** - Real-time matplotlib visualization of arm kinematics
- **Serial Communication** - Interface with ESP32 microcontroller (115200 baud)

## Dependencies

Install required packages:
```bash
pip install -r requirements.txt
```

Or manually install:
```bash
pip install numpy sympy pynput pyserial matplotlib
```

**Package Details:**
- **numpy** - Numerical computations and vector operations
- **sympy** - Symbolic mathematics for DH parameters and Jacobian matrix
- **pynput** - Non-blocking keyboard input capture
- **pyserial** - Serial communication with ESP32
- **matplotlib** - 3D visualization of arm kinematics

## Project Structure

```
Demo-Arm-Controller-and-Inverse-Kinematics/
├── src/
│   ├── main.py              # Main control loop orchestrating all systems
│   ├── keyboard_input.py    # URMController - keyboard input handling
│   ├── parameter.py         # URArmParameter - kinematics and gripper control
│   ├── const.py             # Physical parameters and control constants
│   └── theta_state.json     # Saved arm position and gripper state
├── DEMO ARM EXPORT.stp      # 3D CAD model of the arm (STEP format)
├── Demo Arm Image.png       # Image of Kinematics
├── requirements.txt         # Python dependencies
└── README.md                # This file
```

## Key Components

### 1. Keyboard Input Controller (`keyboard_input.py`)

Manages real-time keyboard input and control state through non-blocking listeners.

**Features:**
- Non-blocking keyboard listener using `pynput`
- Continuous direction control (WASD + Space/Shift for XYZ movement)
- Gripper control input (delta values, not state management)
- Control flow flags (start, pause, restore)
- Separate threads for input detection and main control loop

**Control Scheme:**

| Key | Function | Description |
|-----|----------|-------------|
| **W** | +Y Axis | Move forward |
| **A** | -X Axis | Move left |
| **S** | -Y Axis | Move backward |
| **D** | +X Axis | Move right |
| **Space** | +Z Axis | Move up |
| **Shift** | -Z Axis | Move down |
| **↑ Arrow** | Open Gripper | Increase gripper angle |
| **↓ Arrow** | Close Gripper | Decrease gripper angle |
| **← Arrow** | Roll Gripper CCW | Rotate gripper counter-clockwise (-1) |
| **→ Arrow** | Roll Gripper CW | Rotate gripper clockwise (+1) |
| **O** | Start Control | Enable serial sending and arm movement |
| **P** | Pause Control | Disable serial sending and arm movement |
| **I** | Restore Position | Load last saved arm configuration (only when paused) |
| **ESC** | Exit | Terminate program |

**Key Methods:**
- `start()` - Initialize keyboard listeners and print control instructions
- `returnDirection()` - Get current direction vector [x, y, z]
- `returnGripperValue()` - Get gripper delta (-1, 0, or +1)
- `returnGripperRollDirection()` - Get roll direction (-1, 0, or +1)
- `isControlStartRequested()` - Check if 'O' pressed
- `isPauseRequested()` - Check if 'P' pressed
- `isRestoreStateRequested()` - Check if 'I' pressed
- `stop()` - Stop keyboard listeners

### 2. Arm Kinematics Engine (`parameter.py`)

Core inverse kinematics solver with gripper management and state persistence.

**Features:**
- **DH Parameter-Based Kinematics** - Forward kinematics using Denavit-Hartenberg parameters
- **Jacobian Matrix Computation** - Symbolic differentiation of end-effector position
- **Iterative IK Solver** - Jacobian pseudo-inverse with configurable convergence
- **Gripper State Management** - Gripper angle (0-90°) and roll direction tracking
- **State Persistence** - Save/load configuration to `theta_state.json`
- **Compiled Numerical Functions** - SymPy lambda compilation for real-time performance

**Physical Parameters:**

| Parameter | Value | Unit |
|-----------|-------|------|
| l1 (base height) | 172 | mm |
| l2 (upper arm) | 350 | mm |
| l3 (forearm) | 350 | mm |
| l4 (wrist offset) | 95 | mm |
| a1 (offset 1) | 96.05 | mm |
| a2 (offset 2) | 95.8 | mm |
| a3 (offset 3) | 95 | mm |
| a4 (offset 4) | 50 | mm |

#### DH Parameter Table

The arm kinematics are defined using 8 DH (Denavit-Hartenberg) parameter rows. Each row represents a transformation between consecutive joint frames.

| Link | Frame | d (mm) | θ (rad) | a (mm) | α (rad) | Description |
|------|-------|--------|---------|--------|---------|-------------|
| 1 | Base → J1 | 172 | θ₁ *(variable)* | 0 | π/2 | Base vertical offset, first rotating joint |
| 2 | J1 → J2 | 96.05 | θ₂ *(variable)* | 0 | -π/2 | Joint 1 side offset, second rotating joint |
| 3 | J2 → J3 | 350 | 0 *(fixed)* | 0 | -π/2 | Upper arm length (fixed rotation) |
| 4 | J3 → J4 | 95.8 | θ₃ *(variable)* | 0 | π/2 | Upper arm offset, third rotating joint |
| 5 | J4 → J5 | 350 | 0 *(fixed)* | 0 | π/2 | Forearm length (fixed rotation) |
| 6 | J5 → J6 | 95 | θ₄ *(variable)* | 0 | -π/2 | Wrist offset, fourth rotating joint |
| 7 | J6 → EE | 95 | θ₅ *(variable)* | 0 | π/2 | Wrist roll offset, fifth rotating joint |
| 8 | EE (Gripper) | 50 | θ₆ *(variable)* | 0 | 0 | End effector/gripper attachment, sixth rotating joint |

**Parameter Explanation:**
- **d (displacement):** Distance along Z-axis of previous frame (mm)
- **θ (theta):** Rotation angle around Z-axis (radians) - 6 are variable, 2 are fixed
- **a (length):** Distance along X-axis of current frame (mm)
- **α (alpha):** Rotation angle around X-axis (radians)
- **Variable joints:** θ₁, θ₂, θ₃, θ₄, θ₅, θ₆ represent the 6 rotating joints
- **Fixed frames:** Rows 3 and 5 have θ = 0 (no rotation at those transformations)

**Initial Joint Configuration:**
- θ₁ = 0 rad (0°)
- θ₂ = π/4 rad (45°)
- θ₃ = π/3 rad (60°)
- θ₄ = 0 rad (0°)
- θ₅ = π/2 rad (90°)
- θ₆ = 0 rad (0°)

**Transformation Matrices:**
Each DH row generates a 4×4 transformation matrix T using:
```
T = Rz(θ) * Trans_z(d) * Trans_x(a) * Rx(α)
```
The complete forward kinematics is: **T_total = T₁ × T₂ × T₃ × ... × T₈**

**Control Parameters:**
- Gripper angle range: 0-90 degrees
- Gripper initial angle: 45 degrees
- Movement step size: 1 mm per input
- Gripper rotation step: 0.25 degrees per input
- IK max iterations: 5
- IK convergence threshold: 0.01 mm
- Theta offset angles: [-135, -135, -135, -135, -135, -135, 0] degrees

**Key Methods:**
- `move_by(unit_direction)` - Move end-effector in given direction using IK
- `update_gripper(delta, roll_direction)` - Update gripper angle and roll
- `rotate_gripper(roll_direction)` - Rotate gripper (modify θ₆)
- `get_gripper_angle()` - Get current gripper angle (0-90°)
- `set_gripper_angle(angle)` - Set gripper angle directly
- `get_end_effector_position()` - Get current position [x, y, z]
- `save_theta_state(filepath)` - Save arm configuration to JSON
- `load_theta_state(filepath)` - Load arm configuration from JSON
- `visualize_kinematic(thetas, title)` - Static 3D visualization

### 3. Constants (`const.py`)

Centralized configuration for physical dimensions and control parameters.

**Managed Settings:**
- DH parameter dimensions (l1-l4, a1-a4)
- Initial joint angles for all 6 joints
- Gripper angle range and initial position
- Control step sizes and IK convergence settings
- Theta offset angles for servo output

### 4. Main Control Loop (`main.py`)

Orchestrates real-time integration of keyboard input, kinematics solver, and serial communication.

**Functionality:**
- Initializes controller, arm parameters, and serial connection
- Runs at 20 Hz (50ms per loop iteration)
- Reads keyboard input (non-blocking)
- Manages control state (serial on/off, pause/resume)
- Executes inverse kinematics and gripper updates
- Outputs comma-separated joint angles and gripper angle
- Auto-saves arm configuration after each movement
- Updates 3D visualization (optional, every 10 iterations)
- Graceful fallback if matplotlib or serial unavailable

**Output Format:**
```
theta1_deg, theta2_deg, theta3_deg, theta4_deg, theta5_deg, theta6_deg, gripper_deg
```

Where each theta is computed as: `(theta_rad * 180/π) - THETA_OFFSET_ANGLE[i]`

**Key Features:**
- Control state machine: OFF → ON (press O) → ON → OFF (press P)
- Restore only available when paused
- Automatic state persistence on each movement
- Non-blocking visualization with Agg fallback
- Robust error handling for missing dependencies

## State Persistence

The system automatically saves arm position to `theta_state.json` after each movement while serial is active:

```json
{
  "thetas": [0.00797, 0.77750, 1.05100, -0.00049, 1.56682, 0.09163],
  "gripper_angle": 74,
  "timestamp": "2025-11-23T22:02:41.462515"
}
```

Press **I** (while paused) to restore this configuration, allowing the arm to return to its last position after reboot.

## Usage

### Quick Start

```bash
cd src
python main.py
```

### Control Workflow

1. **Launch program** - `python main.py`
2. **Start control** - Press 'O' to enable arm movement
3. **Move arm** - Use WASD + Space/Shift for XYZ translation
4. **Control gripper** - Use arrow keys for open/close/roll
5. **Auto-save** - Position saved automatically to JSON
6. **Pause** - Press 'P' to stop movement and serial sending
7. **Restore** - Press 'I' to restore last saved position
8. **Exit** - Press 'ESC' to quit

### Integration Example

```python
from keyboard_input import URMController
from parameter import URArmParameter
import time

# Initialize
controller = URMController()
arm = URArmParameter()
controller.start()

# Main loop
while controller.running:
    # Get inputs
    direction = controller.returnDirection()
    gripper_delta = controller.returnGripperValue()
    roll_dir = controller.returnGripperRollDirection()
    
    # Check state changes
    if controller.isControlStartRequested():
        print("Control started")
    if controller.isPauseRequested():
        print("Control paused")
    if controller.isRestoreStateRequested():
        arm.load_theta_state()
    
    # Move arm
    if direction.norm() > 0:
        arm.move_by(direction)
    
    # Update gripper
    if gripper_delta != 0 or roll_dir != 0:
        arm.update_gripper(gripper_delta, roll_dir)
    
    # Save state
    arm.save_theta_state()
    
    time.sleep(0.05)

controller.stop()
```

## Serial Communication

The system attempts to connect to ESP32 on **COM7** at **115200 baud**. If unavailable, the program continues with local control only.

**When Serial is Active:**
- Joint angles and gripper command sent as comma-separated values
- Format: `theta1, theta2, theta3, theta4, theta5, theta6, gripper`
- Updated every control loop iteration (20 Hz)
- Automatically disabled when paused

## Visualization

When enabled (`VISUALIZATION_ENABLED = True` in main.py):
- **Blue lines** - Arm links connecting joints
- **Red spheres** - Joint positions
- **Green star** - End effector position
- **Fixed axes** - X/Y: ±1000 mm, Z: 0-2000 mm (static workspace view)
- **Update rate** - Every 10 control loop iterations (2 Hz)
- **Fallback** - Gracefully disables if matplotlib unavailable

## Future Enhancements

- [ ] Real-time ESP32 communication with servo feedback
- [ ] Collision detection and path planning
- [ ] Web interface for remote monitoring
- [ ] IMU and encoder feedback integration
- [ ] Advanced IK with singularity avoidance
- [ ] Trajectory planning and execution

## Hardware Requirements

- 6-DOF robotic arm with servo motors
- ESP32 microcontroller for wireless control
- Power supply for servo motors
- USB cable or serial adapter (for COM7 communication)
- Computer with Python 3.7+ and dependencies

## Technical Notes

- All joint angles stored in radians, output in degrees
- Gripper state managed in `parameter.py` (not keyboard_input.py)
- Direction vectors auto-normalized by IK solver
- State JSON persists between sessions
- Visualization compatible with Windows/Linux/Mac via matplotlib backends
- Non-blocking design allows integration with other systems
- 20 Hz control loop = 50ms per iteration

## Troubleshooting

**Matplotlib Not Importing:**
- Program continues with visualization disabled
- Check: `pip install matplotlib`

**Serial Port Not Found:**
- Program continues without serial output
- Verify ESP32 connected to COM7
- Check: Device Manager (Windows) or `ls /dev/tty*` (Linux)

**Arm Not Moving:**
- Ensure 'O' key pressed first (SERIAL_IS_SENDING = True)
- Check direction vectors in terminal output
- Verify IK solver converging (check for error messages)

## License

This project is part of the EIC RoboCup 2026 initiative.

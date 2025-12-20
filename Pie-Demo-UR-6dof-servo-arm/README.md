# PIE Demo UR-Style 6-DOF Servo Robot Arm

![Demo Arm](Demo%20Arm%20Image.png)

A complete robotic arm control system featuring inverse kinematics, real-time keyboard control, ESP32-based servo control, and 3D CAD models. This project implements a UR (Universal Robots) style 6-DOF robotic arm with gripper control, designed for educational and research purposes.

## 🎯 Project Overview

This repository contains three main components:

1. **IK Serial Controller (Python)** - Real-time inverse kinematics solver with keyboard control
2. **ESP32 Firmware** - Dual-mode wireless servo controller (UART/ESP-NOW)
3. **CAD Model** - 3D mechanical design files (STEP format)

### Key Features

- ✅ **6-DOF + Gripper Control** - Full 6 degrees of freedom plus gripper manipulation
- ✅ **Real-time Inverse Kinematics** - Jacobian-based IK solver with position control
- ✅ **Dual Communication Modes** - UART serial or ESP-NOW wireless control
- ✅ **Motion Profiling** - Velocity and acceleration limits for smooth servo movement
- ✅ **State Persistence** - Auto-save/restore arm configuration
- ✅ **Live 3D Visualization** - Real-time matplotlib kinematic display
- ✅ **Configurable Servo Profiles** - Per-servo calibration and safety limits

## 📁 Project Structure

```
Pie-Demo-UR-6dof-servo-arm/
├── IK-Serial-Controller-Python/    # Python IK controller with keyboard interface
│   ├── src/
│   │   ├── main.py                 # Main control loop
│   │   ├── parameter.py            # IK solver and kinematics engine
│   │   ├── keyboard_input.py       # Non-blocking keyboard controller
│   │   ├── const.py                # Physical parameters and constants
│   │   └── theta_state.json        # Auto-saved arm state
│   ├── requirements.txt            # Python dependencies
│   └── README.md                   # Detailed IK controller documentation
│
├── firmware/                        # ESP32 servo controller firmware
│   ├── robot-arm-esp32-serial-reciver/   # Main servo controller (PlatformIO)
│   │   ├── include/
│   │   │   ├── Config.h            # Central configuration (servo profiles, modes)
│   │   │   ├── MotionControl.h     # Motion smoothing API
│   │   │   ├── CommandReceiver.h   # Abstract receiver interface
│   │   │   ├── UartReceiver.h      # UART CSV receiver
│   │   │   └── EspNowReceiver.h    # ESP-NOW wireless receiver
│   │   ├── src/
│   │   │   ├── main.cpp            # Firmware entry point
│   │   │   └── MotionControl.cpp   # PCA9685 control & motion profiling
│   │   ├── test/                   # ESP-NOW test examples
│   │   ├── platformio.ini          # Build configuration
│   │   └── README.md               # Firmware documentation
│   │
│   └── robot-arm-esp32-serial-sender/    # ESP-NOW sender (optional)
│       ├── src/main.cpp            # UART → ESP-NOW relay
│       ├── platformio.ini          # Build configuration
│       └── README.md               # Sender documentation
│
├── cad/
│   └── DEMO ARM EXPORT.stp         # 3D CAD model (STEP format)
│
├── Demo Arm Image.png              # Project image
└── README.md                       # This file
```

## 🤖 System Architecture

```
┌────────────────────────────────────────────────────────────────┐
│                    PC - Python Controller                      │
│  ┌────────────────┐    ┌──────────────┐    ┌────────────────┐  │
│  │ Keyboard Input │───▶│ IK Solver    │───▶│ Serial Output  │  │
│  │   (WASD+Keys)  │    │ (Jacobian)   │    │  (115200 baud) │  │
│  └────────────────┘    └──────────────┘    └────────┬───────┘  │
└─────────────────────────────────────────────────────┼──────────┘
                                                      │ USB/UART
                    ┌─────────────────────────────────┴──────────┐
                    │ CSV: θ1,θ2,θ3,θ4,θ5,θ6,gripper             │
                    │                                            │
                    ▼                                            ▼
    ┌───────────────────────────────┐         ┌─────────────────────────────────────┐
    │  ESP32 Sender (Optional)      │         │  Direct Connection                  │
    │  ┌──────────────────────────┐ │         │  (Recommended for testing)          │
    │  │ UART → ESP-NOW Relay     │ │         └─────────────────┬───────────────────┘
    │  │ (Wireless transmission)  │ │                           │
    │  └────────────┬─────────────┘ │                   ┌───────┘
    └───────────────┼───────────────┘                   │
                    │ ESP-NOW (2.4GHz Wireless)         │ UART Direct
                    └───────────────┐                   │
                                    ▼                   ▼
┌────────────────────────────────────────────────────────────────┐
│              ESP32 Receiver (Main Controller)                  │
│  ┌──────────────┐    ┌──────────────┐    ┌─────────────────┐   │
│  │ UART/ESP-NOW │───▶│ Motion       │───▶│ PCA9685 Driver  │   │
│  │  Receiver    │    │ Profiling    │    │ (I2C PWM)       │   │
│  └──────────────┘    └──────────────┘    └────────┬────────┘   │
└───────────────────────────────────────────────────┼────────────┘
                                                    │ I2C
                    ┌───────────────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────────────────────────────┐
│                    PCA9685 PWM Board                            │
│         16-channel 12-bit PWM servo controller                  │
│    CH0-CH6: 7 Servos (Base, Shoulder, Elbow, Wrist, Gripper)    │
└─────────────────────────────────────────────────────────────────┘
                    │
                    ▼
              [7x Servo Motors]
```

### Communication Flow

**Mode 1: Direct UART** (Recommended for testing)
```
PC Python ──USB──▶ ESP32 Receiver ──I2C──▶ PCA9685 ──PWM──▶ Servos
```

**Mode 2: Wireless ESP-NOW** (Optional, for untethered operation)
```
PC Python ──USB──▶ ESP32 Sender ~~ESP-NOW~~▶ ESP32 Receiver ──I2C──▶ PCA9685 ──PWM──▶ Servos
```

## 🚀 Quick Start Guide

### Prerequisites

**Hardware:**
- ESP32 development board (ESP32-DevKit or ESP32-C6)
- PCA9685 16-channel PWM servo driver board
- 7x servo motors (270° or 180° range recommended)
- 5V power supply for servos (5-10A recommended)
- USB cable for programming/UART communication

**Software:**
- Python 3.7+ (for IK controller)
- PlatformIO (for ESP32 firmware)
- USB-to-serial drivers (if required)

### Step 1: Install Python Controller

```bash
cd IK-Serial-Controller-Python
pip install -r requirements.txt
```

**Dependencies:**
- `numpy` - Numerical computations
- `sympy` - Symbolic mathematics for Jacobian
- `pynput` - Non-blocking keyboard input
- `pyserial` - UART communication
- `matplotlib` - 3D visualization

### Step 2: Flash ESP32 Firmware

```bash
cd firmware/robot-arm-esp32-serial-reciver

# Build and upload
platformio run --target upload

# Monitor serial output
platformio device monitor
```

**Configuration** (`include/Config.h`):
- Set `ACTIVE_COMMAND_MODE` to `COMMAND_MODE_UART` (default)
- Configure I2C pins: `I2C_SDA_PIN` and `I2C_SCL_PIN`
- Adjust `SERVO_PROFILES` array for your servos (pulse width, limits)

### Step 3: Wire the Hardware

**ESP32 to PCA9685 (I2C):**
```
ESP32 GPIO21 (SDA) ──▶ PCA9685 SDA
ESP32 GPIO22 (SCL) ──▶ PCA9685 SCL
ESP32 GND ──────────▶ PCA9685 GND
ESP32 3.3V ─────────▶ PCA9685 VCC
```

**PCA9685 to Servos:**
```
PCA9685 V+ Terminal ──▶ External 5V Power Supply (+)
PCA9685 GND Terminal ─▶ External 5V Power Supply (-)
Common GND: ESP32 GND = PCA9685 GND = Power Supply GND
```

**Servo Connections:**
- Channel 0: Base rotation
- Channel 1: Shoulder joint
- Channel 2: Elbow joint
- Channel 3: Wrist pitch
- Channel 4: Wrist yaw
- Channel 5: Wrist roll
- Channel 6: Gripper

### Step 4: Run the Controller

```bash
cd IK-Serial-Controller-Python/src
python main.py
```

**Control Commands:**

| Key | Action | Description |
|-----|--------|-------------|
| **O** | Start | Enable control and serial output |
| **P** | Pause | Stop movement and serial |
| **I** | Restore | Load last saved position (when paused) |
| **W** | +Y | Move forward |
| **A** | -X | Move left |
| **S** | -Y | Move backward |
| **D** | +X | Move right |
| **Space** | +Z | Move up |
| **Shift** | -Z | Move down |
| **↑** | Gripper | Open gripper |
| **↓** | Gripper | Close gripper |
| **←** | Roll | Rotate gripper CCW |
| **→** | Roll | Rotate gripper CW |
| **ESC** | Exit | Quit program |

## 📐 Robot Kinematics

### Physical Dimensions

| Parameter | Value (mm) | Description |
|-----------|------------|-------------|
| l1 | 172 | Base height |
| l2 | 350 | Upper arm length |
| l3 | 350 | Forearm length |
| l4 | 95 | Wrist offset |
| a1 | 96.05 | Joint 1 offset |
| a2 | 95.8 | Joint 2 offset |
| a3 | 95 | Joint 3 offset |
| a4 | 50 | End effector offset |

### Denavit-Hartenberg Parameters

The arm uses 8 transformation frames (6 actuated joints + 2 fixed links):

| Frame | d (mm) | θ (rad) | a (mm) | α (rad) | Type |
|-------|--------|---------|--------|---------|------|
| 1 | 172 | θ₁* | 0 | π/2 | Variable |
| 2 | 96.05 | θ₂* | 0 | -π/2 | Variable |
| 3 | 350 | 0 | 0 | -π/2 | Fixed |
| 4 | 95.8 | θ₃* | 0 | π/2 | Variable |
| 5 | 350 | 0 | 0 | π/2 | Fixed |
| 6 | 95 | θ₄* | 0 | -π/2 | Variable |
| 7 | 95 | θ₅* | 0 | π/2 | Variable |
| 8 | 50 | θ₆* | 0 | 0 | Variable (gripper roll) |

*Variable joints controlled by IK solver

### Workspace

- **Reach:** ~795 mm (l2 + l3 + l4)
- **Height:** 172-867 mm (base to max extension)
- **Base rotation:** 0-270° (configurable)
- **Gripper angle:** 0-90° (open-close)

### Inverse Kinematics Algorithm

The IK solver uses a **Jacobian pseudo-inverse** iterative method:

1. Compute forward kinematics to get current end-effector position
2. Calculate position error: `Δp = target - current`
3. Compute Jacobian matrix `J` (symbolic differentiation via SymPy)
4. Calculate joint update: `Δθ = J⁺ · Δp` (pseudo-inverse)
5. Update joint angles: `θ_new = θ_old + Δθ`
6. Repeat until convergence (error < 0.01 mm) or max iterations (5)

**Convergence:** Typically 2-4 iterations for 1mm movements

## ⚙️ Firmware Configuration

### Servo Profile Structure

Each servo is configured with the following parameters in `Config.h`:

```cpp
struct ServoProfile {
    uint8_t  pwmChannel;           // PCA9685 channel (0-15)
    float    initAngle;            // Initial angle on startup (degrees)
    float    servoRangeDegrees;    // Total servo range (180 or 270)
    float    minAngleLimit;        // Software minimum limit (degrees)
    float    maxAngleLimit;        // Software maximum limit (degrees)
    float    maxVelocity;          // Maximum speed (deg/s)
    float    maxAcceleration;      // Maximum acceleration (deg/s²)
    uint16_t minPulseUs;           // Min pulse width (microseconds)
    uint16_t maxPulseUs;           // Max pulse width (microseconds)
};
```

### Default Configuration

| Servo | Channel | Init° | Range° | Limits° | Vel (°/s) | Accel (°/s²) | Pulse (µs) |
|-------|---------|-------|--------|---------|-----------|--------------|------------|
| Base | 0 | 135 | 270 | 0-270 | 30 | 10 | 450-2420 |
| Shoulder | 1 | 180 | 270 | 0-270 | 30 | 10 | 450-2420 |
| Elbow | 2 | 225 | 270 | 0-270 | 30 | 10 | 450-2420 |
| Wrist Pitch | 3 | 135 | 270 | 0-270 | 30 | 10 | 450-2420 |
| Wrist Yaw | 4 | 135 | 180 | 0-180 | 30 | 10 | 450-2420 |
| Wrist Roll | 5 | 135 | 270 | 45-225 | 30 | 10 | 450-2420 |
| Gripper | 6 | 45 | 270 | 0-270 | 30 | 10 | 450-2420 |

### Communication Modes

**UART Mode (Default):**
- Baud rate: 115200
- Format: CSV string terminated with newline
- Example: `0.0,45.0,90.0,135.0,90.0,135.0,45.0\n`

**ESP-NOW Mode (Wireless):**
- Packet: 7 floats (28 bytes)
- Requires sender ESP32 with correct MAC address
- See `firmware/robot-arm-esp32-serial-sender/` for sender code

### Motion Profiling

The firmware implements **trapezoidal velocity profiling**:
- Smooth acceleration from rest to max velocity
- Constant velocity cruise phase
- Smooth deceleration to target
- Prevents mechanical stress and servo jitter

## 🔧 Calibration Guide

### 1. Servo Pulse Width Calibration

If servos don't reach expected angles:

1. Open `firmware/robot-arm-esp32-serial-reciver/include/Config.h`
2. Adjust `minPulseUs` and `maxPulseUs` for each servo
3. Common ranges:
   - Standard servos: 500-2500 µs
   - High-torque servos: 450-2420 µs
   - Digital servos: 600-2400 µs

**Test procedure:**
```cpp
// Test minimum angle (should be 0°)
servo.writeMicroseconds(minPulseUs);

// Test maximum angle (should be servoRangeDegrees)
servo.writeMicroseconds(maxPulseUs);
```

### 2. Mechanical Limit Calibration

Set software limits to protect mechanics:

```cpp
// Example: Wrist roll has mechanical stops at 45° and 225°
.minAngleLimit = 45.0,
.maxAngleLimit = 225.0,
```

### 3. IK Parameter Calibration

If end-effector position is inaccurate:

1. Measure actual link lengths with calipers
2. Update values in `IK-Serial-Controller-Python/src/const.py`
3. Verify DH parameters match mechanical design

## 🎨 3D CAD Model

The `cad/` directory contains the complete mechanical design:

- **Format:** STEP (.stp) - Universal 3D CAD format
- **Software:** Compatible with SolidWorks, Fusion 360, FreeCAD, OnShape
- **Contents:** All links, joints, mounting brackets, and gripper mechanism

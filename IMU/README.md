# IMU Sensor Fusion — Roll, Pitch & Yaw with Madgwick Filter

Reads gyroscope and accelerometer data from the **FXAS21002C** (gyro) and **FXOS8700** (accel/mag) sensors, applies a Gaussian noise filter, and fuses the data using a **Madgwick filter** to output stable roll, pitch, and yaw angles over Serial.

---

## Hardware Required

| Component | Part |
|-----------|------|
| Gyroscope | Adafruit FXAS21002C |
| Accelerometer / Magnetometer | Adafruit FXOS8700 |
| Microcontroller | Any Arduino-compatible board (e.g. Teensy, Arduino Mega) |

---

## Required Libraries

Install all three libraries via the PlatformIO Library Manager or Arduino Library Manager.

| Library | Source |
|---------|--------|
| `Adafruit FXAS21002C` | [GitHub](https://github.com/adafruit/Adafruit_FXAS21002C) |
| `Adafruit FXOS8700` | [GitHub](https://github.com/adafruit/Adafruit_FXOS8700) |
| `Adafruit Unified Sensor` | [GitHub](https://github.com/adafruit/Adafruit_Sensor) |

---

## PlatformIO Setup

### 1. Install PlatformIO

Install the [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode) for VS Code, or install the CLI:

```bash
pip install platformio
```

### 2. Create a New Project

Open VS Code → PlatformIO Home → **New Project**.

- **Name:** `imu_sensor_fusion` (or any name you like)
- **Board:** Select your board (e.g. `Teensy 4.0`, `Arduino Mega`)
- **Framework:** `Arduino`

### 3. Add the Source File

Copy `src/main.cpp` from this repository into the `src/` folder of your PlatformIO project, replacing the default `main.cpp`.

### 4. Add Libraries to `platformio.ini`

Open `platformio.ini` in the root of your project and add the library dependencies:

```ini
[env:your_board]
platform = espressif32
board = esp32dev        ; change to match your board
framework = arduino

lib_deps =
    adafruit/Adafruit FXAS21002C
    adafruit/Adafruit FXOS8700
    adafruit/Adafruit Unified Sensor
```

> **Note:** Replace `platform`, `board` with the values matching your hardware. For an Arduino Mega, use `platform = atmelavr` and `board = megaatmega2560`.

### 5. Build & Upload

```bash
# Build
pio run

# Upload to board
pio run --target upload

# Open Serial Monitor (115200 baud)
pio device monitor --baud 115200
```

---

## Serial Output

Once running, the board prints comma-separated roll, pitch, and yaw values at ~5ms intervals:

```
12.34,-5.67,90.12
12.35,-5.66,90.13
...
```

| Field | Description |
|-------|-------------|
| Roll | Rotation around the X-axis (degrees) |
| Pitch | Rotation around the Y-axis (degrees) |
| Yaw | Rotation around the Z-axis (degrees) |

---

## Optional: Sensor Calibration

The code ships with pre-measured default offsets and noise values for **Thailand** (gravity ≈ 9.826 m/s²).

To recalibrate for your environment, uncomment the `calibrateSensors()` call in `setup()`:

```cpp
void setup(void) {
  ...
  calibrateSensors();  // ← uncomment this line
  ...
}
```

Place the sensor **completely still** and flat. Calibration takes ~15 seconds and prints new offset values to Serial. You can then paste those values back into the default constants at the top of `main.cpp`.

---

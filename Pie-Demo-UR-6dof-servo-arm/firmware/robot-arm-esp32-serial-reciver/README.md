# Robot Arm — ESP32 Servo Controller

This project runs on an ESP32 and controls a 7-servo robotic arm using an Adafruit PCA9685 PWM driver. It supports receiving commands over UART (CSV of angles) or over ESP-NOW (packed packet with 7 floats). The code includes motion profiling (velocity/acceleration limits), configurable servo profiles, and a simple receiver abstraction.

## Project structure

```
robot-arm-esp32-serial-reciver/
├── platformio.ini                # PlatformIO config (envs, monitor_speed, lib_deps)
├── README.md                     # This file
├── include/                      # Public headers and configuration
│   ├── Config.h                  # Central configuration (modes, I2C pins, servo profiles)
│   ├── CommandReceiver.h         # Abstract receiver interface
│   ├── EspNowReceiver.h          # ESP-NOW receiver implementation
│   ├── UartReceiver.h            # UART CSV receiver implementation
│   ├── MotionControl.h           # Motion control API and types
│   └── Types.h                   # (placeholder for shared types)
├── src/                          # Application source
│   ├── main.cpp                  # Program entry: init + main loop
│   ├── MotionControl.cpp         # Motion smoothing & PCA9685 control
│   └── main.cpp.servotest.bak    # backup/test variants (not used at build)
├── lib/                          # Optional libraries or local code (README inside)
├── test/                         # Test & example sketches
│   ├── esp_now_sender_test.cpp   # Example ESP-NOW sender (sends 7 floats)
│   └── esp_now_receiver_test.cpp # Example ESP-NOW receiver/test harness
```


## Important configuration (edit `include/Config.h`)

Key knobs you will likely change:

- ACTIVE_COMMAND_MODE — choose `COMMAND_MODE_UART` or `COMMAND_MODE_ESPNOW`.
- UART_BAUD_RATE — set your preferred serial speed for UART control.
- I2C_SDA_PIN / I2C_SCL_PIN — pins used to initialize `Wire.begin(SDA, SCL)` on the ESP32.
- PCA9685_ADDRESS — I2C address of the PCA9685 (default `0x40`).
- PCA9685_FREQUENCY — PWM frequency for servos (usually `50` Hz).
- SERVO_PROFILES — array of 7 `ServoProfile` entries. Each profile includes:
  - `pwmChannel` — PCA9685 channel 0..15
  - `initAngle` — initial angle written on startup
  - `servoRangeDegrees` — physical servo range (commonly 180 or 270)
  - `minAngleLimit` / `maxAngleLimit` — software limits to protect mechanics
  - `maxVelocity` / `maxAcceleration` — motion profiling (deg/s, deg/s^2)
  - `minPulseUs` / `maxPulseUs` — servo pulse width mapping in microseconds (e.g., 450..2420)

Notes on pulse widths and calibration:
- If your servo does not reach expected angles, tune `minPulseUs` / `maxPulseUs` for that channel and confirm `servoRangeDegrees`. Many hobby servos expect ~500–2500 µs for ~0–180°.
- Keep mechanical limits (`minAngleLimit`/`maxAngleLimit`) smaller than physical stops to avoid stress.

### Default servo configuration (current settings)

Below are the default servo profiles currently defined in `include/Config.h` :

| Servo no. | Name   | PWM Ch | Init ° | Range ° | Min ° | Max ° | Max Vel (°/s) | Max Accel (°/s²) | MinPulse (µs) | MaxPulse (µs) |
|--------:|:------------------|:------:|:------:|:-------:|:-----:|:-----:|:-------------:|:-----------------:|:-------------:|:-------------:|
| 1       | Base rotation     | 0      | 135.0  | 270.0   | 0.0   | 270.0 | 30.0          | 10.0              | 450           | 2420          |
| 2       | Shoulder          | 1      | 180.0  | 270.0   | 0.0   | 270.0 | 30.0          | 10.0              | 450           | 2420          |
| 3       | Elbow             | 2      | 225.0  | 270.0   | 0.0   | 270.0 | 30.0          | 10.0              | 450           | 2420          |
| 4       | Wrist pitch       | 3      | 135.0  | 270.0   | 0.0   | 270.0 | 30.0          | 10.0              | 450           | 2420          |
| 5       | Wrist yaw         | 4      | 135.0  | 180.0   | 0.0   | 180.0 | 30.0          | 10.0              | 450           | 2420          |
| 6       | Wrist roll        | 5      | 135.0  | 270.0   | 45.0  | 225.0 | 30.0          | 10.0              | 450           | 2420          |
| 7       | Gripper           | 6      | 45.0   | 270.0   | 0.0   | 270.0 | 30.0          | 10.0              | 450           | 2420          |

Tips:
- If you need to re-center or change the default pose, update the `initAngle` for the corresponding servo in `Config.h` and re-upload.
- To protect the mechanics, tighten `Min °`/`Max °` for servos with limited travel (e.g., the gripper or wrist roll).
- If servo movement appears reversed, swap the `minPulseUs`/`maxPulseUs` or adjust `pwmChannel` wiring.

## Build & upload (PlatformIO)

You can build and upload using PlatformIO CLI or the PlatformIO extension in VSCode. PlatformIO envs are defined in `platformio.ini`.

From the project root, use PlatformIO CLI:

```sh
# Build the project
platformio run

# Build and upload to the default environment
platformio run --target upload

# Build and upload to a specific environment (example: esp32_c6_dev)
platformio run --environment esp32_c6_dev --target upload

# Open the serial monitor (uses monitor_speed from platformio.ini)
platformio device monitor
```

If you use the VSCode PlatformIO plugin you can choose the environment and click Build / Upload / Monitor buttons.

## Using UART control (quick test)

1. Set `ACTIVE_COMMAND_MODE` to `COMMAND_MODE_UART` in `include/Config.h`.
2. Build & upload.
3. Open a serial monitor at the configured baud rate (e.g. 115200).
4. Send a CSV of 7 angles, e.g. `90,90,90,90,90,90,90` followed by newline.

The `UartReceiver` will validate and apply the targets. It also prints debug information when `DEBUG_SERIAL` is enabled.

## Using ESP-NOW control (quick test)

1. Set `ACTIVE_COMMAND_MODE` to `COMMAND_MODE_ESPNOW` in `include/Config.h`.
2. Build & upload to the receiver device (your robot controller).
3. Open `test/esp_now_sender_test.cpp`, set `receiverMAC` to the receiver's MAC address (printed by the receiver on boot when ESP-NOW init runs).
4. Build & upload the sender sketch to another ESP32.
5. Use the sender serial monitor to type a comma-separated list of 7 floats or press `T` to send the built-in test pose.

Notes: ESP-NOW packets are lossy in wireless environments. The receiver tolerates out-of-range angles (skips them) — consider adding retries on the sender.


## Wiring / Hardware notes

- PCA9685 I2C pins: use `I2C_SDA_PIN` and `I2C_SCL_PIN` from `Config.h` when calling `Wire.begin(SDA, SCL)` on ESP32.
- PCA9685 VCC/V+:
  - The PCA9685 board has a separate servo power terminal (V+). Power servos from a proper 5V (or compatible) power supply capable of the total stall current for 7 servos.
  - Do NOT power servos from the ESP32 VIN/GPIO regulator.
  - Connect grounds: common ground between ESP32 GND, PCA9685 GND and servo power supply ground.
- PCA9685 SDA/SCL to ESP32 SDA/SCL pins. Also connect I2C SCL/SDA pull-ups if your breakout doesn’t include them.

## How commands are formatted

- UART mode (`UartReceiver`) expects a CSV line with exactly 7 numeric values (angles for servo 1..7). Example:

  90,90,90,10,20,30,40

  The receiver parses the 7 values, validates them against `SERVO_PROFILES` limits, and applies `setServoTarget` for each.

- ESP-NOW mode (`EspNowReceiver`) expects a single packet containing 7 `float`s (packed). The provided `test/esp_now_sender_test.cpp` is an example sender. Update the `receiverMAC` in that file to match the receiver device.

## High-level design / flow

- `main.cpp` initializes serial debug, motion control (I2C + PCA9685), then creates one `CommandReceiver` implementation depending on `Config.h` (`ACTIVE_COMMAND_MODE`).
- Receivers push target angles to the `MotionControl` API (`setServoTarget`).
- `MotionControl` runs a periodic update (called from `loop`) that enforces per-servo velocity and acceleration limits and writes PWM pulses to the PCA9685 using `writeMicroseconds`.

## Tests and examples

- `test/esp_now_sender_test.cpp` and `test/esp_now_receiver_test.cpp` provide simple test harnesses for validating ESP-NOW send/receive and packet structure. Use these to verify radio connectivity before integrating with the robot.

## Troubleshooting

- PCA9685 not found: check I2C wiring, correct `PCA9685_ADDRESS`, and that `Wire.begin(SDA,SCL)` uses the pins wired to the adapter.
- Servos jitter/no torque: ensure servo V+ is powered from a capable 5V supply and grounds are common.
- Angles not matching expected positions: calibrate `minPulseUs`/`maxPulseUs` for each servo and confirm `servoRangeDegrees`.
- ESP-NOW issues: confirm both devices are on the same WiFi channel (ESP-NOW/Channel in `Config.h`), set correct peer MAC on the sender.

## Safety notes

- Always power servos from an appropriate external supply. Disconnect power when adjusting mechanical linkages.
- Start with conservative `maxVelocity` and `maxAcceleration` values when first testing.

## Where to start

1. Wire the PCA9685 and servos with a proper power supply and common ground.
2. Edit `include/Config.h` to set `ACTIVE_COMMAND_MODE` and calibrate the `SERVO_PROFILES` (initial angles and pulse widths).
3. Build & upload with PlatformIO and test with `UartReceiver` CSV or the `esp_now_sender_test.cpp` sender.


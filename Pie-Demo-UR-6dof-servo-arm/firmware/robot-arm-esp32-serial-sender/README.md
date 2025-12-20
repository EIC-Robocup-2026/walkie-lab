
# ESP-NOW Sender — UART → 7-float CSV → ESP-NOW

Small sender firmware for an ESP32 that reads a newline-terminated CSV line of 7 float values from UART and forwards it over ESP-NOW to a receiver (robot controller).

## Overview

Behavior:
- Listen on the configured UART (default 115200) for lines containing 7 comma-separated floats (CSV).
- Parse the 7 floats, pack them into a small binary/JSON/CSV message (implementation detail) and send to a configured ESP-NOW peer MAC address.

This project is a minimal PlatformIO-based ESP32 sender used to control a robot by relaying commands received over a serial link.

## Serial input format

- Each command is a single text line terminated with a newline (\n).
- The line must contain 7 floating-point numbers separated by commas. Example:

	0.00,1.234,-0.5,90.0,0.1,0.0,2.5

- Whitespace around commas is allowed but not required. The implementation ignores blank lines.

If a line doesn't parse to 7 floats, it will be ignored (or logged) — check serial output for diagnostics.

## Hardware / wiring

- ESP32 board running this sender firmware.
- Power the ESP32 with a stable 5V/3.3V supply per board requirements.

ESP-NOW works over WiFi radio, so no physical wiring is needed between sender and receiver beyond power and the UART input.

## Configuration

- Set the peer (receiver) MAC address in the firmware before building. Edit `src/main.cpp` (search for `peer_mac`, `peer_addr` or similar) and set the 6-byte MAC address of the receiver, e.g.:

```cpp
// example: replace with your receiver MAC
uint8_t peer_mac[6] = { 0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC };
```

- If the project has a build flag option for peer MAC in `platformio.ini`, you can alternatively set it there; otherwise edit the source directly.

## Build & flash (PlatformIO)

Open a terminal and run (PowerShell example):

```powershell
# build and upload
pio run -t upload

# or build only
pio run
```

Replace upload target with the appropriate environment if multiple are defined.

## Run / Monitor

Start the serial monitor to view logs and confirm successful ESP-NOW sends:

```powershell
# open device monitor at 115200 baud (change port/baud as needed)
pio device monitor -b 115200
```

Send example input over the UART connected to the ESP32 (from another MCU or USB-to-UART adapter) in the 7-float CSV format shown above.

## Example

Send this line (ending with newline):

	0.0,45.0,-12.3,90.0,0.0,1.5,3.14

The sender will parse and forward the values via ESP-NOW to the configured peer MAC.

## Troubleshooting

- Nothing is sent: verify the UART wiring and that the serial source is sending newline-terminated lines.
- Parsing errors: ensure exactly 7 numeric values per line.
- ESP-NOW send failures: ensure the receiver MAC is correct and both devices are on compatible WiFi channels (ESP-NOW can require matching channels or explicit channel set).
- Use the serial monitor logs for diagnostic messages.

## Notes

- This repo provides the minimal sender logic. You may want to add a small checksum, sequence number, or acknowledgement layer if reliability is required.




import can
import time
import argparse
import sys
import os
from rich.live import Live
from rich.table import Table

# Add the Cubemars-Python-API directory to the path to find the cubemars package
sys.path.append(os.path.abspath("Cubemars-Python-API"))

from cubemars.protocol import unpack_motor_feedback


def main():
    parser = argparse.ArgumentParser(
        description="CAN bus listener for CubeMars motors."
    )
    parser.add_argument(
        "--interface",
        type=str,
        default="socketcan",
        help="CAN interface to use (e.g., socketcan, pcan, gs_usb).",
    )
    parser.add_argument(
        "--channel",
        type=str,
        default="can0",
        help="CAN channel to use (e.g., can0, PCAN_USBBUS1).",
    )
    parser.add_argument(
        "--bitrate", type=int, default=1000000, help="CAN bitrate in bits/s."
    )
    args = parser.parse_args()

    try:
        bus = can.Bus(
            channel=args.channel, interface=args.interface, bitrate=args.bitrate
        )
    except Exception as e:
        print(f"Error initializing CAN bus: {e}")
        print("\nPlease ensure the correct interface and channel are provided.")
        print(
            "Common interfaces: socketcan (Linux), pcan (Windows), gs_usb (Linux/Windows with candle-light fw)."
        )
        sys.exit(1)

    motor_data = {}  # motor_id -> feedback
    motor_last_seen = {}  # motor_id -> timestamp
    TIMEOUT = 1.0  # seconds before removing a motor from the list

    def purge_stale_motors():
        now = time.time()
        stale = [mid for mid, ts in motor_last_seen.items() if now - ts > TIMEOUT]
        for mid in stale:
            del motor_data[mid]
            del motor_last_seen[mid]

    def generate_table() -> Table:
        table = Table(title="CubeMars Motor Feedback (live)")
        table.add_column("Motor ID", justify="right", style="cyan", no_wrap=True)
        table.add_column("Position (deg)", justify="right", style="magenta")
        table.add_column("Velocity (RPM)", justify="right", style="green")
        table.add_column("Current (A)", justify="right", style="yellow")
        table.add_column("Temperature (C)", justify="right", style="red")
        table.add_column("Error Code", justify="right", style="bold red")
        table.add_column("Last Seen (ms ago)", justify="right", style="dim")

        now = time.time()
        sorted_motor_ids = sorted(motor_data.keys())
        for motor_id in sorted_motor_ids:
            feedback = motor_data[motor_id]
            age_ms = (now - motor_last_seen[motor_id]) * 1000
            table.add_row(
                str(motor_id),
                f"{feedback.position: 8.2f}",
                f"{feedback.velocity: 8.2f}",
                f"{feedback.current: 6.2f}",
                str(feedback.temperature),
                str(feedback.error_code),
                f"{age_ms: 6.0f}",
            )
        return table

    try:
        with Live(generate_table(), refresh_per_second=10, screen=True) as live:
            live.console.print(
                f"Starting CAN listener on {args.interface}:{args.channel}...",
                style="bold green",
            )
            live.console.print(
                "Listening for CubeMars motors... Press Ctrl+C to stop.",
                style="bold green",
            )
            while True:
                msg = bus.recv(timeout=0.05)

                # Always purge stale motors and refresh the table
                purge_stale_motors()

                if msg is not None and len(msg.data) == 8:
                    try:
                        feedback = unpack_motor_feedback(msg.data)
                        motor_id = msg.arbitration_id & 0xFF
                        motor_data[motor_id] = feedback
                        motor_last_seen[motor_id] = time.time()
                    except Exception:
                        pass

                live.update(generate_table())
    except KeyboardInterrupt:
        print("\nStopping listener.")
    finally:
        bus.shutdown()
        print("CAN bus shut down.")


if __name__ == "__main__":
    main()

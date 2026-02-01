import sys
import os
import time
from cubemars import CubeMarsMotor

BYPASS_JOINT_LIMIT = True

# --- User Configuration ---
INTERFACE = "socketcan"
CHANNEL = "can0"

# Per-motor configuration
MOTORS_CONFIG = [
    {
        "id": 11,
        "joint_name": "j1",
        "pos_limit_min": -1800.0,
        "pos_limit_max": 1800.0,
    },
    {
        "id": 12,
        "joint_name": "j2",
        "pos_limit_min": -1800.0,
        "pos_limit_max": 0.0,
    },
    {
        "id": 13,
        "joint_name": "j3",
        "pos_limit_min": -900.0,
        "pos_limit_max": 900.0,
    },
    {
        "id": 14,
        "joint_name": "j4",
        "pos_limit_min": -900.0,
        "pos_limit_max": 900.0,
    },
    {
        "id": 15,
        "joint_name": "j5",
        "pos_limit_min": -180.0,
        "pos_limit_max": 0.0,
    },
    {
        "id": 16,
        "joint_name": "j6",
        "pos_limit_min": -90.0,
        "pos_limit_max": 90.0,
    },
    {
        "id": 17,
        "joint_name": "j7",
        "pos_limit_min": -90.0,
        "pos_limit_max": 90.0,
    },
    {
        "id": 18,
        "joint_name": "j8",
        "pos_limit_min": -90.0,
        "pos_limit_max": 90.0,
    },
    # Add more motors here if needed
]

# Global movement parameters
VELOCITY_LIMIT = 1000.0  # rad/s
ACCEL_LIMIT = 100.0  # rad/s^2
# --------------------------


def main():
    """
    Initializes motors and runs an interactive loop to control them
    based on user keyboard input.
    """
    # if sys.platform == "win32":
    #     os.environ["PATH"] = os.getcwd() + os.pathsep + os.environ["PATH"]

    print("--- CubeMars Multi-Motor Keyboard Control ---")
    print(f"Connecting to CAN bus ({INTERFACE} on {CHANNEL})...")

    motors = {}  # Will store {motor_id: {"config": config, "object": motor_obj}}
    joint_name_to_id = {config["joint_name"]: config["id"] for config in MOTORS_CONFIG}

    try:
        for config in MOTORS_CONFIG:
            motor_id = config["id"]
            print(
                f"Initializing Motor ID: {motor_id} (Joint: {config['joint_name']})..."
            )
            motor_obj = CubeMarsMotor(
                interface=INTERFACE, channel=CHANNEL, motor_id=motor_id
            )
            motors[motor_id] = {"config": config, "object": motor_obj}
        print("All specified motors initialized.")

    except Exception as e:
        print(f"\nAn error occurred during initialization: {e}")
        print("Please ensure the CAN interface is connected and drivers are installed.")
        return

    if not motors:
        print("No motors were successfully initialized. Exiting.")
        return

    print("\n--- Control Interface ---")
    print(f"Global Velocity Limit: {VELOCITY_LIMIT:.2f} rad/s")
    print(f"Global Acceleration Limit: {ACCEL_LIMIT:.2f} rad/s^2")
    print(
        "Enter commands as 'ID_or_Name position' (e.g., '11 20.5' or 'front_left_hip -15')."
    )
    print("Type 'exit' to quit.")
    print("-" * 25)

    try:
        while True:
            # 1. Display current status of all motors
            status_line = "\r"
            for mid, motor_data in motors.items():
                fb = motor_data["object"].feedback
                joint = motor_data["config"]["joint_name"]
                status_line += f"{joint}({mid}): Pos={fb.position:6.2f} deg | "
            print(status_line, end="")

            # 2. Get user input
            try:
                user_input = input("\nEnter command: ")
            except (EOFError, KeyboardInterrupt):
                print("\nInterrupted by user.")
                break

            # 3. Parse and execute the command
            if user_input.lower().strip() == "exit":
                break

            parts = user_input.strip().split()
            if len(parts) != 2:
                print("Invalid format. Please use 'ID_or_Name position'.")
                continue

            try:
                # Resolve motor ID from either ID or joint name
                target_identifier = parts[0]
                target_pos_deg = float(parts[1])

                if target_identifier in joint_name_to_id:
                    target_id = joint_name_to_id[target_identifier]
                else:
                    try:
                        target_id = int(target_identifier)
                    except ValueError:
                        print(
                            f"Error: Identifier '{target_identifier}' is not a valid ID or joint name."
                        )
                        continue

                if target_id not in motors:
                    print(f"Error: Motor ID {target_id} has not been configured.")
                    continue

                motor_data = motors[target_id]
                motor_config = motor_data["config"]

                # Convert degrees to radians for the command
                # Assuming the library takes radians, which is standard.
                # target_pos_rad = target_pos_deg * (3.14159 / 180.0)

                # Check against per-motor limits (in degrees)
                min_lim = motor_config["pos_limit_min"]
                max_lim = motor_config["pos_limit_max"]
                if (not (min_lim <= target_pos_deg <= max_lim)) and (not(BYPASS_JOINT_LIMIT)):
                    print(
                        f"Error: Target position {target_pos_deg} for {motor_config['joint_name']} is outside its limits [{min_lim}, {max_lim}]."
                    )
                    continue

                # 4. Send the command to the motor
                print(
                    f"Sending command: Move {motor_config['joint_name']} to {target_pos_deg:.2f} deg."
                )
                motor_to_move = motor_data["object"]

                motor_to_move.set_pos(
                    target_pos_deg,
                    VELOCITY_LIMIT,
                    ACCEL_LIMIT,
                )

            except ValueError:
                print("Invalid input. Position must be a number.")
            except Exception as e:
                print(f"An error occurred while sending command: {e}")

    except Exception as e:
        print(f"\nA critical error occurred: {e}")

    finally:
        # 5. Clean shutdown
        print("\nShutting down. Stopping all motors...")
        for mid, motor_data in motors.items():
            try:
                motor_data["object"].set_current(0)  # Command to stop motion
                motor_data["object"].close()
                print(f"Motor {mid} connection closed.")
            except Exception as e:
                print(f"Error stopping/closing motor {mid}: {e}")
        print("Shutdown complete.")


if __name__ == "__main__":
    main()

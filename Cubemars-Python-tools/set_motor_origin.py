import sys
import os
import time
from cubemars import CubeMarsMotor
import math

# --- Configuration (from multi_motor_keyboard_input.py) ---
INTERFACE = "socketcan"
CHANNEL = "can0"

# Per-motor configuration to map IDs to joint names
MOTORS_CONFIG = [
    {"id": 11, "joint_name": "j1"},
    {"id": 12, "joint_name": "j2"},
    {"id": 13, "joint_name": "j3"},
    {"id": 14, "joint_name": "j4"},
    {"id": 15, "joint_name": "j5"},
    {"id": 16, "joint_name": "j6"},
    {"id": 17, "joint_name": "j7"},
    {"id": 18, "joint_name": "j8"},
]

# -------------------------------------------------------------


def get_joint_name(motor_id):
    """Helper function to find joint name from config."""
    for config in MOTORS_CONFIG:
        if config["id"] == motor_id:
            return config["joint_name"]
    return "Unknown"


def main():
    """
    Runs an interactive script to set the origin for a range of motors.
    """
    print("--- CubeMars Motor Origin Setting ---")

    # 1. Get motor ID range from user
    while True:
        try:
            range_input = input(
                "Enter the CAN bus ID range to calibrate (e.g., '11-14'): "
            )
            start_id_str, end_id_str = range_input.split("-")
            start_id = int(start_id_str)
            end_id = int(end_id_str)
            if start_id > end_id:
                print("Error: Start ID must be less than or equal to End ID.")
                continue
            motor_ids_to_calibrate = range(start_id, end_id + 1)
            break
        except ValueError:
            print("Invalid format. Please use 'start-end' (e.g., '11-14').")

    print(f"Will calibrate motors from ID {start_id} to {end_id}.\n")

    # 2. Iterate through each motor
    for motor_id in motor_ids_to_calibrate:
        joint_name = get_joint_name(motor_id)
        print(f"--- Calibrating Motor ID: {motor_id} (Joint: {joint_name}) ---")

        motor = None
        try:
            # Initialize motor
            motor = CubeMarsMotor(
                interface=INTERFACE, channel=CHANNEL, motor_id=motor_id
            )
            # motor.enable()

            # Ask for target origin position
            # while True:
            #     try:
            #         pos_input = input(
            #             f"Enter target origin position in degrees (or 's' to skip): "
            #         )
            #         if pos_input.lower() == "s":
            #             print("Skipping position setting.")
            #             target_pos_deg = motor.feedback.position * (
            #                 180.0 / math.pi
            #             )  # Use current pos
            #             break
            #         target_pos_deg = float(pos_input)
            #         target_pos_rad = math.radians(target_pos_deg)

            #         # Move motor to the approximate origin position
            #         print(f"Moving {joint_name} to {target_pos_deg:.2f} degrees...")
            #         motor.set_position(
            #             position=target_pos_rad,
            #             velocity_limit=VELOCITY_LIMIT,
            #             accel_limit=ACCEL_LIMIT,
            #         )
            #         # Give it a moment to reach the position
            #         time.sleep(3)
            #         print("Motor has moved.")
            #         break
            #     except ValueError:
            #         print("Invalid input. Please enter a number for the position.")
            #     except Exception as e:
            #         print(f"An error occurred while moving the motor: {e}")
            #         # Decide if we want to retry or skip
            #         break

            # User confirmation
            print("\n" + "=" * 40)
            print("Manually adjust the joint to its final, precise origin position.")
            print("--> Press ENTER to set this position as the TEMPORARY origin.")
            print(
                "--> Type 'p' and press ENTER to set it as the PERMANENT origin (for dual-encoder motors)."
            )
            print("=" * 40)

            confirm_input = input("Confirm origin setting: ")

            is_permanent = confirm_input.lower().strip() == "p"

            # NOTE: Assuming the API for setting zero position.
            # The actual method names might be different (e.g., `zero_encoder`, `set_home`).
            if is_permanent:
                print("Setting PERMANENT origin for dual-encoder motor...")
                # This is a hypothetical method. The actual one might be different.
                # e.g., motor.write_motor_parameter(Parameter.ENCODER_OFFSET, motor.feedback.position)
                # or motor.set_zero_position(permanent=True)
                motor.set_origin(1)
                print("Permanent origin set.")
            else:
                print("Setting TEMPORARY origin...")
                motor.set_origin(0)
                print("Temporary origin set.")

        except Exception as e:
            print(f"\nAn error occurred with motor {motor_id}: {e}")
            print("Please check connections and motor status.")
        finally:
            if motor:
                # motor.disable()
                motor.close()
            print(f"--- Finished with Motor ID: {motor_id} ---\n")

    print("All specified motors have been processed. Shutdown complete.")


if __name__ == "__main__":
    main()

from keyboard_input import URMController
import parameter
import const as c
import time
import serial
import math
import numpy as np


#enable or disable visualization
#note: visualization does lag the control loop slightly
VISUALIZATION_ENABLED = False

# Try to import matplotlib for visualization
try:
    import matplotlib
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    print("Matplotlib imported successfully")
except Exception as e:
    print(f"Warning: Could not import matplotlib ({type(e).__name__}: {e}). Visualization disabled.")
    VISUALIZATION_ENABLED = False
    plt = None
    Axes3D = None

# Setup live visualization (only if available)
fig = None
ax = None
if VISUALIZATION_ENABLED:
    try:
        import matplotlib
        matplotlib.use('TkAgg')  # Use TkAgg backend for better compatibility
        plt.ion()  # Enable interactive mode
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        print("Visualization window created successfully")
    except Exception as e:
        print(f"Warning: Could not initialize visualization: {type(e).__name__}: {e}")
        VISUALIZATION_ENABLED = False

# Visualization update counter
viz_update_counter = 0
viz_update_frequency = 10  # Update visualization every N iterations

# Fixed axis limits for stable visualization (based on arm workspace)
FIXED_AXIS_LIMIT = 1000  # mm, adjusted based on arm dimensions


# Initialize controller once
controller = URMController()
controller.start()  # Starts background listener (non-blocking)

#if true serial is sending data (printing)
SERIAL_IS_SENDING = False

# Initialize UR Arm parametersoddddddddasd
parameter = parameter.URArmParameter()
print("UR Arm parameters initialized.")

# Initialize serial communication with ESP32 on COM7
ser = None
try:
    ser = serial.Serial(port='COM6', baudrate=115200, timeout=1)
    print(f"Serial port opened: {ser.name}")
except Exception as e:
    print(f"Serial port not available (continuing without): {e}")


while True:
    # Get current direction and gripper control inputs
    direction = controller.returnDirection()
    gripper_angle_delta = controller.returnGripperValue()
    gripper_roll_direction = controller.returnGripperRollDirection()
    pitch_rotation_direction = controller.returnPitchRotationDirection()
    yaw_rotation_direction = controller.returnYawRotationDirection()

    # Check for control start and pause requests
    start = controller.isControlStartRequested()
    pause = controller.isPauseRequested()
    restore = controller.isRestoreStateRequested()
    reset = controller.isResetRequested()

    # Get current of all 7 servo angle state (in rad)
    end_pos = parameter.get_end_effector_position()
    thetas = parameter.get_current_thetas()
    gripper = parameter.get_gripper_angle()

    #start or pause serial sending
    
    if start:
        SERIAL_IS_SENDING = True
    if pause:
        SERIAL_IS_SENDING = False
    
    # Restore arm to last saved position, only available when not sending serial
    if restore and not SERIAL_IS_SENDING:
        parameter.load_theta_state()
        # print("Arm position restored successfully")
    
    # Reset arm to initial position, only available when not sending serial
    if reset and not SERIAL_IS_SENDING:
        parameter.reset_to_initial_position()

    # Send all 7 servo deg value via serial if available
    if SERIAL_IS_SENDING:

        # Update gripper state based on keyboard input
        parameter.update_gripper(gripper_angle_delta, gripper_roll_direction)
        
        # Update pitch angle based on keyboard input
        if pitch_rotation_direction != 0:
            parameter.rotate_pitch(pitch_rotation_direction)
        
        # Update yaw angle based on keyboard input
        if yaw_rotation_direction != 0:
            parameter.rotate_yaw(yaw_rotation_direction)

        # Move arm based on direction input
        parameter.move_by(direction)

        values = []
        for i in range(6):
            theta_deg = math.degrees(float(thetas[i])) - c.THETA_OFFSET_ANGLE[i]
            values.append(f"{theta_deg:.4f}")
        gripper_val = gripper - c.THETA_OFFSET_ANGLE[6]
        values.append(f"{gripper_val:.4f}")
        output_string = ",".join(values) + "\n"
        print(output_string)
        if ser is not None and ser.is_open:
            ser.write(output_string.encode("UTF-8"))
        
        # Save theta state after each successful movement
        parameter.save_theta_state()
    
    if not controller.running:
        break
    

    ##############################################################
    #VISUALIZATION CODE, NOT RELEVANT TO CONTROL AND SERIAL SENDING
    ##############################################################

    # Update visualization periodically (only if enabled)
    if VISUALIZATION_ENABLED:
        viz_update_counter += 1
        if viz_update_counter >= viz_update_frequency:
            viz_update_counter = 0
            
            try:
                ax.clear()
                
                # Calculate joint positions for visualization
                dh_params = parameter.dh_parameters
                import sympy as sp
                T_total = sp.eye(4)
                joint_positions = [np.array([0, 0, 0])]  # Start at origin
                
                # Substitute theta values
                theta_values = {parameter.theta1: thetas[0],
                                parameter.theta2: thetas[1],
                                parameter.theta3: thetas[2],
                                parameter.theta4: thetas[3],
                                parameter.theta5: thetas[4],
                                parameter.theta6: thetas[5]}
                
                # Calculate position of each joint
                for i in range(dh_params.rows):
                    d, theta, a, alpha = dh_params.row(i)
                    T = parameter.dh_to_transformation(d, theta, a, alpha)
                    T_substituted = T.subs(theta_values)
                    T_total = T_total * T_substituted
                    
                    # Extract position from transformation matrix
                    pos = T_total[:3, 3]
                    pos_numeric = np.array([float(pos[0]), float(pos[1]), float(pos[2])])
                    joint_positions.append(pos_numeric)
                
                # Convert to numpy array
                joint_positions = np.array(joint_positions)
                
                # Plot the arm links
                ax.plot(joint_positions[:, 0], joint_positions[:, 1], joint_positions[:, 2], 
                        'b-o', linewidth=2, markersize=8, label='Arm links')
                
                # Plot joint positions
                ax.scatter(joint_positions[:, 0], joint_positions[:, 1], joint_positions[:, 2], 
                           c='red', s=100, label='Joints')
                
                # Plot end effector
                ax.scatter(joint_positions[-1, 0], joint_positions[-1, 1], joint_positions[-1, 2], 
                           c='green', s=150, marker='*', label='End Effector')
                
                # Labels and formatting
                ax.set_xlabel('X (mm)')
                ax.set_ylabel('Y (mm)')
                ax.set_zlabel('Z (mm)')
                ax.set_title('UR Arm Live Kinematic Visualization')
                ax.legend()
                ax.grid(True, alpha=0.3)
                
                # Set FIXED axis limits (graph does not translate)
                ax.set_xlim(-FIXED_AXIS_LIMIT, FIXED_AXIS_LIMIT)
                ax.set_ylim(-FIXED_AXIS_LIMIT, FIXED_AXIS_LIMIT)
                ax.set_zlim(0, 2 * FIXED_AXIS_LIMIT)
                
                fig.canvas.draw()
                fig.canvas.flush_events()
            except Exception as e:
                print(f"Visualization error: {e}")
                VISUALIZATION_ENABLED = False
    

    time.sleep(0.01)

controller.stop()
if VISUALIZATION_ENABLED:
    plt.close('all')

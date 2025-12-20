import const as c
import sympy as sp
import numpy as np
import time
import json
import os
import math
from datetime import datetime

class URArmParameter:
    """Class to hold UR Arm parameters"""
    def __init__(self):
        print("Starting initialization...")
        
        # parameter for links, width, angle of the Demo UR Arm
        # unit in miliimeters and radians
        # reference in Demo Arm image
        self.l1, self.l2, self.l3, self.l4 = c.l1, c.l2, c.l3, c.l4
        self.a1, self.a2, self.a3, self.a4 = c.a1, c.a2, c.a3, c.a4
        self.theta1, self.theta2, self.theta3, self.theta4, self.theta5, self.theta6 = sp.symbols('t1 t2 t3 t4 t5 t6')

        # initial angle for joint 1 - 6 and gripper respectively
        self.joint_initializing_angle = sp.Matrix([c.initial_angle[0], c.initial_angle[1], c.initial_angle[2], c.initial_angle[3], c.initial_angle[4], c.initial_angle[5]])

        # inital theta parameters for joints 1-6
        self.theta_parameters = sp.Matrix([self.theta1, self.theta2, self.theta3, self.theta4, self.theta5, self.theta6]).T

        #initialize thetas, these will be used both controlling the arm and calculating DH parameters
        self.thetas = self.joint_initializing_angle.copy()  # in rad
        
        # Initialize gripper parameters
        self.gripper_angle = c.GRIPPER_INITIAL_ANGLE  # 0-90 degrees
        self.gripper_roll_direction = 0  # -1, 0, or +1
        
        # Initialize reset flag
        self.is_resetting = False

        #initialize DH parameters
        print("Computing DH parameters...")
        t0 = time.time()
        self.dh_parameters = self.dh()
        print(f"  DH parameters took {time.time() - t0:.2f}s")

        #initialize forward kinematics
        print("Computing forward kinematics...")
        t0 = time.time()
        self.forward_kinematics = self.dh_to_forward_kinematics()
        print(f"  Forward kinematics took {time.time() - t0:.2f}s")

        #initialize jacobian matrix (from end-effector position - last column rows 0-2)
        print("Computing Jacobian matrix...")
        t0 = time.time()
        self.end_effector_position = self.forward_kinematics[:3, 3]
        self.jacobian_matrix = self.end_effector_position.jacobian(self.theta_parameters)
        print(f"  Jacobian matrix took {time.time() - t0:.2f}s")
        
        # Compile numerical functions for fast computation
        print("Compiling numerical functions...")
        t0 = time.time()
        self.fk_func = sp.lambdify((self.theta1, self.theta2, self.theta3, self.theta4, self.theta5, self.theta6), 
                                    self.end_effector_position, 'numpy')
        self.jac_func = sp.lambdify((self.theta1, self.theta2, self.theta3, self.theta4, self.theta5, self.theta6), 
                                     self.jacobian_matrix, 'numpy')
        print(f"  Compilation took {time.time() - t0:.2f}s")
        print("Initialization complete!")
    
    #reset thetas to initial position
    def reset_thetas(self):
        """Start gradual reset of thetas to initial position"""
        self.is_resetting = True
    
    def update_reset(self):
        """Update reset progress - call this every loop iteration"""
        if not hasattr(self, 'is_resetting'):
            self.is_resetting = False
        
        if self.is_resetting:
            # Calculate distance to initial position
            distance = (self.thetas - self.joint_initializing_angle).norm()
            
            if distance > c.RESET_STEP_SIZE:
                # Calculate direction towards initial position
                direction = self.joint_initializing_angle - self.thetas
                normalized_direction = direction / direction.norm()
                
                # Take a step towards initial position
                self.thetas += normalized_direction * c.RESET_STEP_SIZE
            else:
                # Reached initial position
                self.thetas = self.joint_initializing_angle.copy()
                self.is_resetting = False

    #return current thetas
    def get_current_thetas(self):
        return self.thetas
    
    #return DH parameters as sp.Matrix
    def dh(self):
        """Return the DH parameters as np.array (d, theta, a, alpha)"""
        dh_parameter = sp.Matrix([[self.l1, self.theta_parameters[0], 0, sp.pi/2],
                              [self.a1, 0, 0, -sp.pi/2],
                              [self.l2, 0, 0, -sp.pi/2],
                              [self.a2, self.theta_parameters[2], 0, sp.pi/2],
                              [self.l3, 0, 0, sp.pi/2],
                              [self.l4, self.theta_parameters[3], 0, -sp.pi/2],
                              [self.a3, self.theta_parameters[4], 0, sp.pi/2],
                              [self.a4, self.theta_parameters[5], 0, 0] ])
        
        return dh_parameter
    
    #compute transformation matrix from DH parameters
    def dh_to_transformation(self, d, theta, a, alpha):
        """Convert DH parameters to transformation matrix"""
        T = sp.Matrix([[sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
                       [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
                       [0, sp.sin(alpha), sp.cos(alpha), d],
                       [0, 0, 0, 1]])
        return T
    
    #calculate forward kinematics from DH parameters
    def dh_to_forward_kinematics(self):
        """Calculate forward kinematics using DH parameters"""
        dh_params = self.dh_parameters
        T_total = sp.eye(4)
        
        for i in range(dh_params.rows):
            d, theta, a, alpha = dh_params.row(i)
            T = self.dh_to_transformation(d, theta, a, alpha)
            T_total = T_total * T
    
        return T_total
    
    # Substitute theta values into any symbolic matrix
    def substitute_thetas(self, matrix, thetas=None):
        """Substitute theta values into any symbolic matrix
        
        Args:
            matrix: Symbolic matrix to substitute into (e.g., forward_kinematics, jacobian_matrix, etc.)
            thetas: sp.Matrix or list of 6 theta values. If None, uses current self.thetas
            
        Returns:
            Matrix with substituted theta values
        """
        if thetas is None:
            thetas = self.thetas
        
        theta_values = {self.theta1: thetas[0],
                        self.theta2: thetas[1],
                        self.theta3: thetas[2],
                        self.theta4: thetas[3],
                        self.theta5: thetas[4],
                        self.theta6: thetas[5]}
        
        substituted = matrix.subs(theta_values)
        return substituted
    
    #return current end-effector position
    def get_end_effector_position(self):
        """Return the current end-effector position using compiled numerical function"""
        pos = self.fk_func(float(self.thetas[0]), float(self.thetas[1]), float(self.thetas[2]), 
                           float(self.thetas[3]), float(self.thetas[4]), float(self.thetas[5]))
        return sp.Matrix(pos.flatten())

    def rotate_gripper(self, roll_direction):
        """Rotate the gripper by a small angle in the given direction
        
        Args:
            roll_direction: +1 to increase angle, -1 to decrease angle
        """
        # Define small angle step (in radians)
        angle_step = sp.rad(5)  # 5 degrees in radians
        
        # Update theta6 (gripper rotation)
        new_theta6 = self.thetas[5] + roll_direction * sp.rad(c.GRIPPER_ROTATION_STEP)
        
        # Update thetas
        self.thetas[5] = new_theta6
    
    def rotate_pitch(self, pitch_direction):
        """Rotate the pitch angle (theta5) by a small angle in the given direction
        
        Args:
            pitch_direction: +1 to increase angle, -1 to decrease angle
        """
        # Update theta5 (pitch angle / wrist roll offset)
        new_theta5 = self.thetas[4] + pitch_direction * sp.rad(c.PITCH_ROTATION_STEP)
        
        # Update thetas
        self.thetas[4] = new_theta5
    
    def rotate_yaw(self, yaw_direction):
        """Rotate the yaw angle (theta4) by a small angle in the given direction
        
        Args:
            yaw_direction: +1 to increase angle, -1 to decrease angle
        """
        # Update theta4 (yaw angle / wrist)
        new_theta4 = self.thetas[3] + yaw_direction * sp.rad(c.YAW_ROTATION_STEP)
        
        # Update thetas
        self.thetas[3] = new_theta4
    
    def reset_to_initial_position(self):
        """Reset all theta values to their initial positions"""
        self.thetas = self.joint_initializing_angle.copy()
        print("Arm reset to initial position")
        print(f"Thetas: {[f'{float(t):.4f}' for t in self.thetas]}")
    
    def update_gripper(self, gripper_angle_delta=0, roll_direction=0):
        """Update gripper angle and roll direction
        
        Args:
            gripper_angle_delta: Change in gripper angle (-1 to close, +1 to open)
            roll_direction: Gripper roll direction (-1, 0, or +1)
        """
        # Update gripper angle
        if gripper_angle_delta != 0:
            self.gripper_angle = max(c.GRIPPER_MIN_ANGLE, 
                                    min(c.GRIPPER_MAX_ANGLE, 
                                        self.gripper_angle + gripper_angle_delta))
        
        # Update gripper roll direction
        self.gripper_roll_direction = roll_direction
        
        # Apply roll rotation to theta6
        if roll_direction != 0:
            self.rotate_gripper(roll_direction)
    
    def get_gripper_angle(self):
        """Return the current gripper angle (0-90 degrees)"""
        return self.gripper_angle
    
    def get_gripper_roll_direction(self):
        """Return the current gripper roll direction (-1, 0, or +1)"""
        return self.gripper_roll_direction
    
    def set_gripper_angle(self, angle):
        """Set gripper angle directly
        
        Args:
            angle: Gripper angle in degrees (0-90)
        """
        self.gripper_angle = max(c.GRIPPER_MIN_ANGLE, min(c.GRIPPER_MAX_ANGLE, angle))
    
    def save_theta_state(self, filepath=None):
        """Save current theta values and gripper state to JSON file
        
        Args:
            filepath: Path to save JSON file. If None, uses default theta_state.json
        """
        if filepath is None:
            # Get the directory of this file
            script_dir = os.path.dirname(os.path.abspath(__file__))
            filepath = os.path.join(script_dir, 'theta_state.json')
        
        # Convert sympy Matrix to list of floats
        thetas_list = [float(t) for t in self.thetas]
        
        state_data = {
            "thetas": thetas_list,
            "gripper_angle": self.gripper_angle,
            "timestamp": datetime.now().isoformat()
        }
        
        try:
            with open(filepath, 'w') as f:
                json.dump(state_data, f, indent=2)
            # print(f"Theta state saved to {filepath}")
            return True
        except Exception as e:
            # print(f"Error saving theta state: {e}")
            return False
    
    def load_theta_state(self, filepath=None):
        """Load theta values and gripper state from JSON file
        
        Args:
            filepath: Path to load JSON file from. If None, uses default theta_state.json
            
        Returns:
            True if successfully loaded, False otherwise
        """
        if filepath is None:
            # Get the directory of this file
            script_dir = os.path.dirname(os.path.abspath(__file__))
            filepath = os.path.join(script_dir, 'theta_state.json')
        
        try:
            with open(filepath, 'r') as f:
                state_data = json.load(f)
            
            # Load thetas
            if "thetas" in state_data:
                thetas_list = state_data["thetas"]
                if len(thetas_list) == 6:
                    self.thetas = sp.Matrix([sp.Float(t) for t in thetas_list])
                    
                    # Load gripper angle if available
                    if "gripper_angle" in state_data:
                        self.gripper_angle = state_data["gripper_angle"]
                    
                    print(f"Theta state loaded from {filepath}")
                    print(f"Thetas: {[f'{float(t):.4f}' for t in self.thetas]}")
                    print(f"Gripper angle: {self.gripper_angle}")
                    return True
                else:
                    print(f"Error: Expected 6 thetas, got {len(thetas_list)}")
                    return False
            else:
                print("Error: 'thetas' key not found in JSON file")
                return False
        
        except FileNotFoundError:
            print(f"Error: theta_state.json file not found at {filepath}")
            return False
        except Exception as e:
            print(f"Error loading theta state: {e}")
            return False
    

    #If movement by unit direction is possible, update thetas and return True, else return False
    def move_by(self, unit_direction):
        """Move the end-effector by a small step in the given unit direction using Jacobian inverse
        
        Args:
            unit_direction: Direction vector to normalize and scale by CONTROL_STEP_SIZE
        """
        
        # Normalize direction and scale by step size
        normalized_dir = unit_direction.normalized()
        step_direction = normalized_dir * c.CONTROL_STEP_SIZE
        
        # Move the end-effector in the given direction
        self.current_pos = self.get_end_effector_position()
        self.target_pos = self.current_pos + step_direction

        #temp theta copy for iteration
        self.theta_copy = self.thetas.copy()

        self.error = self.target_pos - self.current_pos

        #keeping track of iterations
        self.iteration = 0

        while (self.error.norm() > c.CONVERGENCE_THRESHOLD) and (self.iteration < c.MAX_ITERATIONS):
            # Use compiled Jacobian function for speed
            jac_numpy = self.jac_func(float(self.theta_copy[0]), float(self.theta_copy[1]), float(self.theta_copy[2]),
                                      float(self.theta_copy[3]), float(self.theta_copy[4]), float(self.theta_copy[5]))
            dir_numpy = np.array(step_direction.evalf(), dtype=float).flatten()
            
            # Use numerical pseudo-inverse (much faster)
            jac_pinv = np.linalg.pinv(jac_numpy)
            delta_thetas_numpy = jac_pinv @ dir_numpy
            
            # Convert back to sympy
            self.delta_thetas = sp.Matrix(delta_thetas_numpy)

            #update theta copy
            self.theta_copy += self.delta_thetas

            #update current position using compiled function
            pos_numpy = self.fk_func(float(self.theta_copy[0]), float(self.theta_copy[1]), float(self.theta_copy[2]),
                                     float(self.theta_copy[3]), float(self.theta_copy[4]), float(self.theta_copy[5]))
            self.current_pos = sp.Matrix(pos_numpy.flatten())

            #update error
            self.error = self.target_pos - self.current_pos

            #increment iteration
            self.iteration += 1

        if self.iteration < c.MAX_ITERATIONS:
            #update thetas if converged
            self.thetas = self.theta_copy
            return True  # movement successful
        
        else:
            return False  # movement failed due to max iterations

    def visualize_kinematic(self, thetas=None, title="UR Arm Kinematic Visualization"):
        """Visualize the kinematic chain of the robot arm using matplotlib
        
        Args:
            thetas: sp.Matrix of 6 theta values. If None, uses current self.thetas
            title: Title for the visualization plot
        """
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D
        except ImportError:
            print("Error: matplotlib is required for visualization. Install it using: pip install matplotlib")
            return
        
        if thetas is None:
            thetas = self.thetas
        
        # Calculate all joint positions by computing transformation matrices
        dh_params = self.dh_parameters
        T_total = sp.eye(4)
        joint_positions = [np.array([0, 0, 0])]  # Start at origin
        
        # Substitute theta values
        theta_values = {self.theta1: thetas[0],
                        self.theta2: thetas[1],
                        self.theta3: thetas[2],
                        self.theta4: thetas[3],
                        self.theta5: thetas[4],
                        self.theta6: thetas[5]}
        
        # Calculate position of each joint
        for i in range(dh_params.rows):
            d, theta, a, alpha = dh_params.row(i)
            T = self.dh_to_transformation(d, theta, a, alpha)
            T_substituted = T.subs(theta_values)
            T_total = T_total * T_substituted
            
            # Extract position from transformation matrix
            pos = T_total[:3, 3]
            pos_numeric = np.array([float(pos[0]), float(pos[1]), float(pos[2])])
            joint_positions.append(pos_numeric)
        
        # Convert to numpy array for easier plotting
        joint_positions = np.array(joint_positions)
        
        # Create 3D plot
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
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
        ax.set_title(title)
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Set equal aspect ratio for better visualization
        max_range = np.array([joint_positions[:, 0].max()-joint_positions[:, 0].min(),
                              joint_positions[:, 1].max()-joint_positions[:, 1].min(),
                              joint_positions[:, 2].max()-joint_positions[:, 2].min()]).max() / 2.0
        mid_x = (joint_positions[:, 0].max()+joint_positions[:, 0].min()) * 0.5
        mid_y = (joint_positions[:, 1].max()+joint_positions[:, 1].min()) * 0.5
        mid_z = (joint_positions[:, 2].max()+joint_positions[:, 2].min()) * 0.5
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        plt.show()

    def move_absolute(self, target_position):
        """Move the end-effector to an absolute target position using iterative Jacobian inverse
        
        Args:
            target_position: sp.Matrix of target (x, y, z) position
        """
        # Initialize movement state variables
        self.move_absolute_active = True
        self.current_pos = self.get_end_effector_position()
        self.target_pos = target_position
        
        # Define error and movement parameters
        self.error = self.target_pos - self.current_pos
        self.error_magnitude = self.error.norm()

        # print(self.error_magnitude)
        
        if self.error_magnitude < c.CONVERGENCE_THRESHOLD:
            print("Already at target position")
            self.move_absolute_active = False
            return
        
        self.error_direction = self.error.normalized()
        
        # Step size based on MOVE_SPEED (mm/s) and control loop frequency (20 Hz = 0.05s)
        # step_size = MOVE_SPEED * 0.05 (mm per control loop iteration)
        # self.move_speed_step = (c.MOVE_SPEED * 0.05)
        self.move_speed_step = c.CONTROL_STEP_SIZE
        
        # Determine number of waypoints based on distance and move speed step
        self.waypoints_number = int(math.ceil(self.error_magnitude / self.move_speed_step))
        
        # Generate waypoint positions
        self.waypoint_positions = []
        for i in range(1, self.waypoints_number + 1):
            waypoint_pos = self.current_pos + (self.error_direction * self.move_speed_step * i)
            self.waypoint_positions.append(waypoint_pos)
        
        # Solve IK for each waypoint and store theta configurations
        self.waypoint_joint_angles = []
        theta_copy_prev = self.thetas.copy()  # Start from current thetas
        
        for waypoint_index, waypoint_pos in enumerate(self.waypoint_positions):
            # Use previous waypoint's theta as starting point for IK solver
            theta_copy_iter = theta_copy_prev.copy()
            
            # Calculate initial error for this waypoint
            pos_numpy = self.fk_func(float(theta_copy_iter[0]), float(theta_copy_iter[1]), 
                                    float(theta_copy_iter[2]), float(theta_copy_iter[3]), 
                                    float(theta_copy_iter[4]), float(theta_copy_iter[5]))
            current_pos_iter = sp.Matrix(pos_numpy.flatten())
            error_iter = waypoint_pos - current_pos_iter
            
            # Use IK solver to find theta configuration for this waypoint
            # Starting from previous waypoint's theta configuration
            iteration = 0
            
            while (error_iter.norm() > c.CONVERGENCE_THRESHOLD) and (iteration < c.MAX_ITERATIONS):
                # Use compiled Jacobian function for speed
                jac_numpy = self.jac_func(float(theta_copy_iter[0]), float(theta_copy_iter[1]), 
                                         float(theta_copy_iter[2]), float(theta_copy_iter[3]), 
                                         float(theta_copy_iter[4]), float(theta_copy_iter[5]))
                error_numpy = np.array(error_iter.evalf(), dtype=float).flatten()
                
                # Use numerical pseudo-inverse
                jac_pinv = np.linalg.pinv(jac_numpy)
                delta_thetas_numpy = jac_pinv @ error_numpy
                
                # Convert back to sympy
                delta_thetas = sp.Matrix(delta_thetas_numpy)
                
                # Update theta copy
                theta_copy_iter += delta_thetas
                
                # Update current position using compiled function with new theta
                pos_numpy = self.fk_func(float(theta_copy_iter[0]), float(theta_copy_iter[1]), 
                                        float(theta_copy_iter[2]), float(theta_copy_iter[3]), 
                                        float(theta_copy_iter[4]), float(theta_copy_iter[5]))
                current_pos_iter = sp.Matrix(pos_numpy.flatten())
                
                # Update error from current waypoint position
                error_iter = waypoint_pos - current_pos_iter
                
                # Increment iteration
                iteration += 1
            
            # Store the converged theta configuration for this waypoint
            if iteration < c.MAX_ITERATIONS:
                self.waypoint_joint_angles.append(theta_copy_iter)
            else:
                print(f"Warning: IK did not converge for waypoint {waypoint_index + 1}/{len(self.waypoint_positions)}")
                self.waypoint_joint_angles.append(theta_copy_iter)  # Use best effort
            
            # Use this waypoint's theta as starting point for next waypoint
            theta_copy_prev = theta_copy_iter.copy()
        
        # Initialize waypoint traversal state
        self.current_waypoint_index = 0
        self.waypoint_theta_target = self.waypoint_joint_angles[0] if self.waypoint_joint_angles else self.thetas
        print(f"Move absolute initialized: {self.waypoints_number} waypoints to traverse")
    
    def update_move_absolute(self):
        """Update theta values smoothly along the waypoint path without blocking
        Call this every control loop iteration to progress toward target
        Returns True if movement complete, False if still in progress
        """
        if not hasattr(self, 'move_absolute_active') or not self.move_absolute_active:
            return False
        
        if not hasattr(self, 'current_waypoint_index'):
            return False
        
        # Check if we've completed all waypoints
        if self.current_waypoint_index >= len(self.waypoint_joint_angles):
            print("Move absolute complete")
            self.move_absolute_active = False
            return True
        
        # Get target theta for current waypoint
        self.waypoint_theta_target = self.waypoint_joint_angles[self.current_waypoint_index]
        
        # Calculate difference between current and target theta
        theta_error = self.waypoint_theta_target - self.thetas
        theta_error_norm = theta_error.norm()
        
        # If close enough to waypoint, move to next waypoint
        if theta_error_norm < 0.01:  # Small threshold in radians
            self.current_waypoint_index += 1
            
            if self.current_waypoint_index >= len(self.waypoint_joint_angles):
                print("Move absolute complete")
                self.move_absolute_active = False
                return True
            
            return False
        
        # Smoothly interpolate toward target waypoint theta
        # Use small step to avoid blocking other control processes
        interpolation_step = 0.1  # Interpolation ratio per loop iteration
        theta_direction = theta_error.normalized()
        step_distance = min(theta_error_norm * interpolation_step, theta_error_norm)
        
        # Update thetas toward current waypoint
        self.thetas = self.thetas + (theta_direction * step_distance)
        
        return False
    

            








# parameter = URArmParameter()
# print(parameter.get_end_effector_position())
# parameter.move_absolute(sp.Matrix([-50,-150, 800]))  # Example usage

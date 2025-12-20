import sympy as sp
import const as c
from pynput import keyboard
import time

class URMController:
    """Controller for UR Arm using pynput keyboard input"""
    
    def __init__(self):
        # Direction vector [x, y, z]
        self.direction = sp.Matrix([0.0, 0.0, 0.0])
        
        # Gripper control inputs (direction of change, not state)
        self.gripper_angle_delta = 0  # +1 to open, -1 to close
        self.gripper_roll_direction = 0  # -1, 0, or +1
        
        # Pitch angle control input (direction of change, not state)
        self.pitch_rotation_direction = 0  # -1 to decrease pitch, +1 to increase pitch, 0 for no change
        
        # Yaw angle control input (direction of change, not state)
        self.yaw_rotation_direction = 0  # -1 to decrease yaw, +1 to increase yaw, 0 for no change

        # Running state
        self.running = True
        
        # Control and reset flags
        self.control_start_requested = False
        self.pause_requested = False
        self.restore_state_requested = False
        self.reset_requested = False
        self.absolute_move_requested = False  # "/" key pressed during pause
        self.absolute_coordinate = None  # Stores valid absolute coordinate [x, y, z]
        
        # Key states to track continuous movement
        self.keys_pressed = {
            'w': False,
            'a': False,
            's': False,
            'd': False,
            'shift': False,
            'space': False,
            'left': False,
            'right': False,
            'up': False,
            'down': False,
            'j': False,
            'k': False,
            'n': False,
            'm': False
        }
        
        # Setup listeners
        self.keyboard_listener = keyboard.Listener(
            on_press=self.on_key_press,
            on_release=self.on_key_release
        )
    
    def on_key_press(self, key):
        """Handle keyboard press events"""
        try:
            # Character keys (w, a, s, d) - handle both lowercase and uppercase
            if hasattr(key, 'char'):
                char = key.char.lower() if key.char else None
                if char in self.keys_pressed:
                    self.keys_pressed[char] = True
                elif char == 'o':
                    self.control_start_requested = True
                elif char == 'p':
                    self.pause_requested = True
                elif char == 'i':
                    self.restore_state_requested = True
                elif char == 'r':
                    self.reset_requested = True
                elif char == '/':
                    self.absolute_move_requested = True
                elif char == 'j':
                    self.keys_pressed['j'] = True
                elif char == 'k':
                    self.keys_pressed['k'] = True
                elif char == 'n':
                    self.keys_pressed['n'] = True
                elif char == 'm':
                    self.keys_pressed['m'] = True
            
            # Special keys
            elif key == keyboard.Key.shift_l or key == keyboard.Key.shift_r:
                self.keys_pressed['shift'] = True
            elif key == keyboard.Key.space:
                self.keys_pressed['space'] = True
            elif key == keyboard.Key.left:
                self.keys_pressed['left'] = True
            elif key == keyboard.Key.right:
                self.keys_pressed['right'] = True
            elif key == keyboard.Key.up:
                self.keys_pressed['up'] = True
            elif key == keyboard.Key.down:
                self.keys_pressed['down'] = True
            elif key == keyboard.Key.esc:
                self.running = False
                return False
        
        except AttributeError:
            pass
    
    def on_key_release(self, key):
        """Handle keyboard release events"""
        try:
            if hasattr(key, 'char'):
                char = key.char.lower() if key.char else None
                if char in self.keys_pressed:
                    self.keys_pressed[char] = False
            elif key == keyboard.Key.shift_l or key == keyboard.Key.shift_r:
                self.keys_pressed['shift'] = False
            elif key == keyboard.Key.space:
                self.keys_pressed['space'] = False
            elif key == keyboard.Key.left:
                self.keys_pressed['left'] = False
            elif key == keyboard.Key.right:
                self.keys_pressed['right'] = False
            elif key == keyboard.Key.up:
                self.keys_pressed['up'] = False
            elif key == keyboard.Key.down:
                self.keys_pressed['down'] = False
            elif hasattr(key, 'char'):
                char = key.char.lower() if key.char else None
                if char == 'j':
                    self.keys_pressed['j'] = False
                elif char == 'k':
                    self.keys_pressed['k'] = False
                elif char == 'n':
                    self.keys_pressed['n'] = False
                elif char == 'm':
                    self.keys_pressed['m'] = False
        
        except AttributeError:
            pass
    
    def update_gripper(self):
        """Update gripper control signals based on arrow key states"""
        # Gripper angle control (up/down arrows) - send delta values
        self.gripper_angle_delta = 0
        if self.keys_pressed['up']:
            # Up arrow: open gripper (increase angle)
            self.gripper_angle_delta = 1
        elif self.keys_pressed['down']:
            # Down arrow: close gripper (decrease angle)
            self.gripper_angle_delta = -1
        
        # Gripper roll direction control (left/right arrows)
        if self.keys_pressed['right']:
            self.gripper_roll_direction = 1
        elif self.keys_pressed['left']:
            self.gripper_roll_direction = -1
        else:
            self.gripper_roll_direction = 0
    
    def update_pitch(self):
        """Update pitch angle control signals based on j/k key states"""
        # Pitch angle control (j/k keys) - send direction values
        self.pitch_rotation_direction = 0
        if self.keys_pressed['k']:
            # K key: increase pitch angle (theta5)
            self.pitch_rotation_direction = 1
        elif self.keys_pressed['j']:
            # J key: decrease pitch angle (theta5)
            self.pitch_rotation_direction = -1
    
    def update_yaw(self):
        """Update yaw angle control signals based on n/m key states"""
        # Yaw angle control (n/m keys) - send direction values
        self.yaw_rotation_direction = 0
        if self.keys_pressed['m']:
            # M key: increase yaw angle (theta4)
            self.yaw_rotation_direction = 1
        elif self.keys_pressed['n']:
            # N key: decrease yaw angle (theta4)
            self.yaw_rotation_direction = -1
    
    def update_direction(self):
        """Update direction vector based on current key states"""
        # Reset direction vector
        self.direction = sp.Matrix([0.0, 0.0, 0.0])
        
        # X-axis control (A/D keys)
        if self.keys_pressed['d']:
            self.direction[0] += 1.0
        if self.keys_pressed['a']:
            self.direction[0] -= 1.0
        
        # Y-axis control (W/S keys)
        if self.keys_pressed['w']:
            self.direction[1] += 1.0
        if self.keys_pressed['s']:
            self.direction[1] -= 1.0
        
        # Z-axis control (Space/Shift keys)
        if self.keys_pressed['space']:
            self.direction[2] += 1.0
        if self.keys_pressed['shift']:
            self.direction[2] -= 1.0
        
        # Update gripper angle
        self.update_gripper()
        
        # Update pitch angle
        self.update_pitch()
        
        # Update yaw angle
        self.update_yaw()
    
    def start(self):
        """Start the controller listeners"""
        self.keyboard_listener.start()
        print("UR Arm Controller Started")
        print("Controls:")
        print("  W/A/S/D - Move in X/Y plane")
        print("  Space/Shift - Move up/down (Z axis)")
        print("  ↑ (Up Arrow) - Open gripper")
        print("  ↓ (Down Arrow) - Close gripper")
        print("  <- (Left Arrow) - Gripper roll direction -1")
        print("  -> (Right Arrow) - Gripper roll direction +1")
        print("  J - Decrease pitch angle (theta5)")
        print("  K - Increase pitch angle (theta5)")
        print("  N - Decrease yaw angle (theta4)")
        print("  M - Increase yaw angle (theta4)")
        print("  O - Start/Resume control")
        print("  P - Pause control")
        print("  I - Restore arm to last saved position")
        print("  R - Reset arm to initial position")
        print("  ESC - Exit")
        print()
    
    def get_state(self):
        """Get current state: (direction_vector, gripper_angle, gripper_roll_direction)"""
        self.update_direction()
        return {
            'direction': self.direction,
            'gripper_angle': self.gripper_angle,
            'gripper_roll_direction': self.gripper_roll_direction
        }
    
    def returnDirection(self):
        """Return the current direction vector"""
        self.update_direction()
        return self.direction
    
    def returnGripperValue(self):
        """Return the gripper angle delta value (+1 to open, -1 to close, 0 for no change)"""
        return self.gripper_angle_delta
    
    def returnGripperRollDirection(self):
        """Return the current gripper roll direction (1, 0, or -1)"""
        return self.gripper_roll_direction
    
    def returnPitchRotationDirection(self):
        """Return the current pitch rotation direction (+1 to increase, -1 to decrease, 0 for no change)"""
        return self.pitch_rotation_direction
    
    def returnYawRotationDirection(self):
        """Return the current yaw rotation direction (+1 to increase, -1 to decrease, 0 for no change)"""
        return self.yaw_rotation_direction
    
    def isControlStartRequested(self):
        """Check if control start was requested ('o' key) and clear the flag"""
        if self.control_start_requested:
            self.control_start_requested = False
            return True
        return False
    
    def isPauseRequested(self):
        """Check if pause was requested ('p' key) and clear the flag"""
        if self.pause_requested:
            self.pause_requested = False
            return True
        return False
    
    def isRestoreStateRequested(self):
        """Check if restore state was requested ('i' key) and clear the flag"""
        if self.restore_state_requested:
            self.restore_state_requested = False
            return True
        return False
    
    def isResetRequested(self):
        """Check if reset was requested ('r' key) and clear the flag"""
        if self.reset_requested:
            self.reset_requested = False
            return True
        return False
    
    def isAbsoluteMoveRequested(self):
        """Check if absolute move was requested ('/' key) and clear the flag"""
        if self.absolute_move_requested:
            self.absolute_move_requested = False
            return True
        return False
    
    def getAbsoluteCoordinate(self):
        """Get the stored absolute coordinate and clear it"""
        coord = self.absolute_coordinate
        self.absolute_coordinate = None
        return coord
    
    def inputAbsoluteCoordinate(self):
        """Prompt user to input absolute coordinate during pause and validate
        
        Returns:
            sp.Matrix([x, y, z]) if input is valid
            None if input is invalid
        """
        try:
            user_input = input("Enter target position (format: 100,100,100): ").strip()
            
            # Parse input
            parts = user_input.split(',')
            if len(parts) != 3:
                print("[INVALID] Input must contain exactly 3 comma-separated values (x,y,z)")
                return None
            
            # Convert to floats
            x, y, z = float(parts[0].strip()), float(parts[1].strip()), float(parts[2].strip())
            
            # Basic validation: check if coordinates are within reasonable workspace
            # Adjust these limits based on your arm's actual workspace
            if abs(x) > 2000 or abs(y) > 2000 or z > 2000 or z < 0:
                print(f"[INVALID] Coordinates ({x:.1f}, {y:.1f}, {z:.1f}) outside workspace bounds")
                print("  Expected ranges: x,y in [-2000,2000], z in [0,2000]")
                return None
            
            # Valid input
            self.absolute_coordinate = sp.Matrix([x, y, z])
            print(f"[VALID] Target position set to: ({x:.1f}, {y:.1f}, {z:.1f}) mm")
            return self.absolute_coordinate
        
        except ValueError as e:
            print(f"[INVALID] Failed to parse input: {e}")
            print("  Please use format: 100,100,100 (numbers only, separated by commas)")
            return None
        except Exception as e:
            print(f"[ERROR] Unexpected error during input: {e}")
            return None
    
    def run(self, update_rate=20):
        """Run the controller update loop"""
        self.start()
        
        try:
            while self.running:
                state = self.get_state()
                print(f"Direction: {state['direction']} | Gripper: {state['gripper_angle']:.1f}°")
                print(controller.get_state())
                time.sleep(1.0 / update_rate)
        
        except KeyboardInterrupt:
            print("\nController interrupted")
        
        finally:
            self.stop()
    
    def stop(self):
        """Stop the controller"""
        self.keyboard_listener.stop()
        print("UR Arm Controller Stopped")


if __name__ == "__main__":
    controller = URMController()
    controller.run()
    

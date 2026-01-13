"""
Keyboard/Mouse Control Interface for MuJoCo Car Simulation
Provides UI input controls for driving the car
"""

import time
import mujoco
import mujoco.viewer
from control import CarController


class KeyboardController:
    def __init__(self, model_path):
        """
        Initialize the keyboard controller

        Args:
            model_path: Path to the MuJoCo XML model file
        """
        # Load MuJoCo model
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Create car controller
        self.controller = CarController(self.model, self.data)

        # Control state
        self.forward_speed = 0.0  # -1 to 1
        self.turn_speed = 0.0     # -1 to 1

        # Control parameters
        self.acceleration = 0.05   # How fast the car accelerates
        self.turn_rate = 0.05      # How fast the car turns
        self.max_speed = 1.0       # Maximum forward speed

        # Key states
        self.keys = {
            'up': False,
            'down': False,
            'left': False,
            'right': False,
            'space': False
        }

    def handle_keyboard(self):
        """
        Handle keyboard input for car control

        Controls:
        - Arrow Up / W: Move forward
        - Arrow Down / S: Move backward
        - Arrow Left / A: Turn left
        - Arrow Right / D: Turn right
        - Space: Stop
        """
        # Update control based on key states
        if self.keys['space']:
            # Emergency stop
            self.forward_speed = 0.0
            self.turn_speed = 0.0
        else:
            # Forward/backward control
            if self.keys['up']:
                self.forward_speed += self.acceleration
            elif self.keys['down']:
                self.forward_speed -= self.acceleration
            else:
                # Gradually slow down when no key pressed
                self.forward_speed *= 0.95

            # Clamp forward speed
            self.forward_speed = max(-self.max_speed, min(self.max_speed, self.forward_speed))

            # Turn control
            if self.keys['left']:
                self.turn_speed -= self.turn_rate
            elif self.keys['right']:
                self.turn_speed += self.turn_rate
            else:
                # Gradually return to center
                self.turn_speed *= 0.9

            # Clamp turn speed
            self.turn_speed = max(-1.0, min(1.0, self.turn_speed))

        # Apply control to car
        self.controller.set_control(self.forward_speed, self.turn_speed)

    def on_key_press(self, key):
        """Handle key press events"""
        key_map = {
            mujoco.mjtButton.mjBUTTON_UP: 'up',
            mujoco.mjtButton.mjBUTTON_DOWN: 'down',
            mujoco.mjtButton.mjBUTTON_LEFT: 'left',
            mujoco.mjtButton.mjBUTTON_RIGHT: 'right',
        }

        # Map arrow keys
        if key in key_map:
            self.keys[key_map[key]] = True

        # Map WASD keys (using keycode)
        # These are common keycodes for W, A, S, D
        wasd_map = {
            87: 'up',      # W
            83: 'down',    # S
            65: 'left',    # A
            68: 'right',   # D
            32: 'space',   # Space
        }

        if key in wasd_map:
            self.keys[wasd_map[key]] = True

    def on_key_release(self, key):
        """Handle key release events"""
        key_map = {
            mujoco.mjtButton.mjBUTTON_UP: 'up',
            mujoco.mjtButton.mjBUTTON_DOWN: 'down',
            mujoco.mjtButton.mjBUTTON_LEFT: 'left',
            mujoco.mjtButton.mjBUTTON_RIGHT: 'right',
        }

        # Map arrow keys
        if key in key_map:
            self.keys[key_map[key]] = False

        # Map WASD keys
        wasd_map = {
            87: 'up',      # W
            83: 'down',    # S
            65: 'left',    # A
            68: 'right',   # D
            32: 'space',   # Space
        }

        if key in wasd_map:
            self.keys[wasd_map[key]] = False

    def print_controls(self):
        """Print control instructions"""
        print("\n" + "="*50)
        print("Car Control System")
        print("="*50)
        print("Controls:")
        print("  Arrow Up / W     - Move Forward")
        print("  Arrow Down / S   - Move Backward")
        print("  Arrow Left / A   - Turn Left")
        print("  Arrow Right / D  - Turn Right")
        print("  Space            - Stop")
        print("="*50)
        print("\nSimulation running... Press Ctrl+C to exit\n")

    def get_status(self):
        """Get current control status"""
        return {
            'forward': self.forward_speed,
            'turn': self.turn_speed,
            'position': self.controller.get_car_position(),
        }

    def run(self, duration=None):
        """
        Run the simulation with keyboard control

        Args:
            duration: Duration in seconds (None for infinite)
        """
        self.print_controls()

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            start_time = time.time()

            while viewer.is_running():
                # Check duration
                if duration and (time.time() - start_time > duration):
                    break

                step_start = time.time()

                # Handle keyboard input and update control
                self.handle_keyboard()

                # Apply control signals
                self.controller.apply_control()

                # Step physics
                mujoco.mj_step(self.model, self.data)

                # Print status every 100 steps
                if int(self.data.time * 100) % 100 == 0:
                    status = self.get_status()
                    print(f"\rTime: {self.data.time:.2f}s | "
                          f"Forward: {status['forward']:.2f} | "
                          f"Turn: {status['turn']:.2f} | "
                          f"Position: [{status['position'][0]:.2f}, "
                          f"{status['position'][1]:.2f}, "
                          f"{status['position'][2]:.2f}]", end="")

                # Sync viewer
                viewer.sync()

                # Time keeping
                time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)


def main():
    """Main entry point"""
    model_path = '../model/car.xml'

    try:
        controller = KeyboardController(model_path)
        controller.run()
    except KeyboardInterrupt:
        print("\n\nSimulation stopped by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()

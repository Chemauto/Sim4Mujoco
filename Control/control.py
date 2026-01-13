"""
Car Controller for MuJoCo Simulation
Controls the forward and turn motors for the car
"""

import mujoco
import numpy as np


class CarController:
    def __init__(self, model, data):
        """
        Initialize the car controller

        Args:
            model: MuJoCo model instance
            data: MuJoCo data instance
        """
        self.model = model
        self.data = data

        # Get actuator IDs
        self.forward_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'forward')
        self.turn_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'turn')

        # Control signals (-1 to 1)
        self.forward_control = 0.0
        self.turn_control = 0.0

    def set_control(self, forward, turn):
        """
        Set control signals for the car

        Args:
            forward: Forward motor control value (-1 to 1)
                    -1 = full reverse, 0 = stop, 1 = full forward
            turn: Turn motor control value (-1 to 1)
                  -1 = full left, 0 = straight, 1 = full right
        """
        # Clamp values to valid range
        self.forward_control = np.clip(forward, -1.0, 1.0)
        self.turn_control = np.clip(turn, -1.0, 1.0)

    def get_forward_control(self):
        """Get current forward control value"""
        return self.forward_control

    def get_turn_control(self):
        """Get current turn control value"""
        return self.turn_control

    def stop(self):
        """Stop the car"""
        self.forward_control = 0.0
        self.turn_control = 0.0

    def apply_control(self):
        """
        Apply the control signals to the MuJoCo actuators

        This should be called before each mj_step
        """
        self.data.ctrl[self.forward_actuator_id] = self.forward_control
        self.data.ctrl[self.turn_actuator_id] = self.turn_control

    def get_car_position(self):
        """
        Get the current position of the car

        Returns:
            numpy array of [x, y, z] position
        """
        car_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'car')
        return self.data.xpos[car_body_id].copy()

    def get_car_orientation(self):
        """
        Get the current orientation of the car

        Returns:
            numpy array of [w, x, y, z] quaternion
        """
        car_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'car')
        return self.data.xquat[car_body_id].copy()

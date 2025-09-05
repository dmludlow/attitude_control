"""
This file contains a base controller class for spacecraft attitude control.
"""

import numpy as np
from abc import ABC, abstractmethod

class Controller(ABC):
    max_torque: float

    def __init__(self, max_torque: float):
        """
        Initializes the controller.

        Args:
            max_torque (float): Maximum allowable torque for the controller.
        """
        self.max_torque = max_torque

    @abstractmethod
    def torque(self, state_in, state_desired, I: np.ndarray) -> np.ndarray:
        """
        Abstract method to compute the control torque.

        Args:
            state_in: Current state of the spacecraft.
            state_desired: Desired state of the spacecraft.
            I (np.ndarray): Inertia tensor of the spacecraft.

        Returns:
            np.ndarray: Control torque vector.
        """
        pass
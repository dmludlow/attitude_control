"""
This file contains a base controller class for spacecraft attitude control.
"""

import numpy as np
from abc import ABC, abstractmethod

class controller(ABC):

    max_torque: float

    def __init__(self, max_torque: float):
        """
        Initializes the controller.
        
        Args:
            max_torque (float): Maximum allowable torque for the controller.
        """
        self.max_torque = max_torque
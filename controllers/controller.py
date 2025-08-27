"""
This file contains implimentation of a PD controller for spacecraft attitude control 
"""

import numpy as np
import attitude_control.spacecraft as sc
import attitude_control.quaternion as qm
import attitude_control.dynamics as dy
import attitude_control.state as st

class controller:

    craft: sc.Spacecraft
    T_c: np.ndarray
    
    def __init__(self, craft: sc.Spacecraft):
        """
        Initializes the controller with the spacecraft to be controlled.
        """
        self.craft = craft
        self.T_c = np.array([0, 0, 0])
"""
This file defines the Spacecraft class for simulating spacecraft dynamics.
"""

import attitude_control.state as st
import attitude_control.quaternion as qm
import numpy as np


class Spacecraft:

    state: st.State

    def __init__(self, state_in: st.State = None):
        if state_in is None:
            # Initialize with default state: no rotation and zero angular velocity
            self.state = st.State(
                qm.Quaternion(np.array([1, 0, 0, 0])),
                np.array([0, 0, 0])
            )
        else:
            self.state = state_in

    def step(self, a: np.ndarray, dt: float):
        """
        Updates the spacecraft state based on angular acceleration vector and time step.

        Args:
            a (np.ndarray): Angular acceleration vector.
            dt (float): Time step in seconds.
        """
        self.state.step(a, dt)
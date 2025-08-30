"""
This file contains implimentation of a PD controller for spacecraft attitude control 
"""

import numpy as np
import attitude_control.controllers as ctrl
import attitude_control.state as st

class PD_control(ctrl.controller):

    # Proportional and Derivative gains
    Kp: float
    Kd: float

    def __init__(self, Kp: float, Kd: float):
        """
        Initializes the PD controller with specified gains.

        Args:
            Kp (float): Proportional gain.
            Kd (float): Derivative gain.
            craft (sc.Spacecraft): The spacecraft to be controlled.
        """
        self.Kp = Kp
        self.Kd = Kd

    # Very incomplete, just a starting point to make sure controller can act on the spacecraft
    def torque(self, state_in: st.State, state_desired: st.State, I: np.ndarray) -> np.ndarray:
        """
        Computes the control torque.

        Args:
            state_in (sc.State): Current state of the spacecraft.
            state_desired (sc.State): Desired state of the spacecraft.
            I (np.ndarray): Inertia tensor of the spacecraft.

        Returns:
            np.ndarray: Control torque vector.
        """
        # Quaternion error (((((not done yet)))))
        q_e = state_desired.q - state_in.q  # Placeholder for quaternion error calculation

        # Angular velocity error
        w_e = state_desired.w - state_in.w

        # Controller torque calculation
        T_c = self.Kp * q_e - self.Kd * w_e
        return T_c
    
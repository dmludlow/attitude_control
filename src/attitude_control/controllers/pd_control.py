"""
This file contains implimentation of a PD controller for spacecraft attitude control 
"""

import numpy as np
import src.attitude_control.controllers as ctrl
import src.attitude_control.plant.state as st

class PD_control(ctrl.Controller):

    # Proportional and Derivative gains
    Kp: np.ndarray
    Kd: np.ndarray

    def __init__(self, max_torque: float, Kp: np.ndarray, Kd: np.ndarray):
        """
        Initializes the PD controller with specified gains and max torque.

        Args:
            max_torque (float): Maximum allowable torque for the controller.
            Kp (np.ndarray): Proportional gain.
            Kd (np.ndarray): Derivative gain.
        """
        super().__init__(max_torque)
        self.Kp = Kp
        self.Kd = Kd

    # Simple implimentation
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
        # Quaternion error (desired * conj(current))
        q_e = state_desired.q.q_prod(state_in.q.q_conj)
        # Enforce shortest rotation: flip sign if scalar part negative
        if q_e.q[0] < 0:
            q_e = q_e.negative
        # Vector part of the error; negative feedback uses -e
        e = q_e.q[1:]
        e = -e

        # Body-rate error as current - desired for negative feedback
        w_err = state_in.w - state_desired.w

        # Controller torque calculation (all negative feedback terms)
        T_c = -(self.Kp @ e) - (self.Kd @ w_err)

        # Clip each axis to +/- max_torque
        T_c = np.clip(T_c, -self.max_torque, self.max_torque)

        return T_c

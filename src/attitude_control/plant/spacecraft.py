"""
This file defines the Spacecraft class for simulating spacecraft dynamics.
"""

from . import state as st
from . import quaternion as qm
from . import dynamics as dy
from ..controllers import controller as ctrl
import numpy as np


class Spacecraft:


    I: np.ndarray
    state: st.State
    controller: ctrl.Controller

    def __init__(self, I: np.ndarray, state_in: st.State, controller: ctrl.Controller = None):
        """
        Initializes the Spacecraft with inertia tensor and initial state.

        Inertia tensor follows the form:
        [Ixx, Ixy, Ixz]
        [Iyx, Iyy, Iyz]
        [Izx, Izy, Izz]
        """
        self.I = I
        self.state = state_in
        self.controller = controller

    def step(self, T: np.ndarray, dt: float):
        """
        Updates the spacecraft state based on applied torque vector and time step.

        Args:
            T (np.ndarray): Applied torque vector.
            dt (float): Time step in seconds.

        Using I*w_dot + w x (Iw) = T to account for high rates and gyroscopic effects.
        """
        # Extract w from current state
        w = self.state.w
        # Compute angular acceleration: alpha = I^{-1} * (T - w × (I * w))
        a = dy.torque_to_ang_accel(self.I, T, w)
        # Update state with computed angular acceleration
        self.state.step(a, dt)
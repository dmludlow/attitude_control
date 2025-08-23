"""
This file defines the State class for simulating spacecraft dynamics.

It manages the spacecraft's quaternion and angular velocity state.

It includes methods for updating the state based on angular acceleration and time step,
normalizing the quaternion, and converting the state to a string representation.
"""

import attitude_control.quaternion as qm
import numpy as np
import attitude_control.dynamics as dy


class State:

    q: qm.Quaternion
    w: np.ndarray

    def __init__(self, q: qm.Quaternion, w: np.ndarray):
        # Ensure q is always a Quaternion object
        if isinstance(q, qm.Quaternion):
            self.q = q
        else:
            self.q = qm.Quaternion(np.array(q))
        #np array with angular velocity [x, y, z]
        self.w = np.asarray(w, dtype=float)

    def step(self, a: np.ndarray, dt: float):
        """
        Updates the state based on angular acceleration vector and time step.

        Args:
            a (np.ndarray): Angular acceleration vector.
            dt (float): Time step in seconds.
        """
        self.w = dy.integrate_omega(self.w, a, dt)
        self.q = dy.integrate_angle(self.q, self.w, a, dt)
    
    @property
    def norm(self):
        """Normalizes the state's quaternion."""
        self.q = qm.q_normalize(self.q)

    def  __str__(self):
        """Returns a string representation of the state."""
        return (
            f"State(   q = {self.q},     w = {self.w})\n"
            f"         euler = {self.q.to_euler_angles}"
        )
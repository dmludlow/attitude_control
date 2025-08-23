"""
This file contains quaternion math functions used for spacecraft attitude simulation.

Expected quaternion format: [q0, q1, q2, q3] where q0 is the scalar.
"""

import numpy as np


class Quaternion:
    """
    Represents a quaternion for spacecraft attitude simulation.

    Attributes:
        q (np.ndarray): The quaternion as a 4-element numpy array [q0, q1, q2, q3].
    """

    q: np.ndarray

    def __init__(self, q: np.ndarray):
        """
        Initializes a Quaternion object.

        Args:
            q (np.ndarray): A 4-element array representing the quaternion [q0, q1, q2, q3].
        """
        self.q = q

    @property
    def norm(self):
        """
        Returns the normalized quaternion.

        Returns:
            Quaternion: The normalized quaternion as a Quaternion object.
        """
        return Quaternion(self.q / np.linalg.norm(self.q))

    @property
    def q_conj(self):
        """
        Returns the conjugate of the quaternion.

        Returns:
            Quaternion: The conjugate quaternion as a Quaternion object.
        """
        return Quaternion(np.array([self.q[0], -self.q[1], -self.q[2], -self.q[3]]))

    def q_prod(self, q2: 'Quaternion'):
        """
        Returns the product of this quaternion and another.

        Args:
            q2 (Quaternion): The second quaternion.

        Returns:
            Quaternion: The product quaternion as a Quaternion object.
        """
        prod = np.array([
            self.q[0]*q2.q[0] - self.q[1]*q2.q[1] - self.q[2]*q2.q[2] - self.q[3]*q2.q[3],
            self.q[0]*q2.q[1] + self.q[1]*q2.q[0] + self.q[2]*q2.q[3] - self.q[3]*q2.q[2],
            self.q[0]*q2.q[2] - self.q[1]*q2.q[3] + self.q[2]*q2.q[0] + self.q[3]*q2.q[1],
            self.q[0]*q2.q[3] + self.q[1]*q2.q[2] - self.q[2]*q2.q[1] + self.q[3]*q2.q[0]
        ])
        return Quaternion(prod)

    @property
    def to_euler_angles(self):
        """
        Converts the quaternion to Euler angles (roll, pitch, yaw).

        Returns:
            np.ndarray: Euler angles in radians as [roll, pitch, yaw].
        """
        return np.array([
            np.arctan2(2*(self.q[0]*self.q[1]), 1 - 2*(self.q[1]**2 - self.q[2]**2)),
            np.arcsin(2*(self.q[0]*self.q[2] + self.q[1]*self.q[3])),
            np.arctan2(2*(self.q[0]*self.q[3]), 1 - 2*(self.q[2]**2 - self.q[3]**2))
        ])
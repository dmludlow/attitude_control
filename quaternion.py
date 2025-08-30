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
    
    ###### Unverified equation #######
    @staticmethod
    def from_euler_angles(roll: float, pitch: float, yaw: float) -> 'Quaternion':
        """
        Converts Euler angles (roll, pitch, yaw) to a quaternion.

        Args:
            roll (float): Roll angle in radians.
            pitch (float): Pitch angle in radians.
            yaw (float): Yaw angle in radians.

        Returns:
            Quaternion: The quaternion as a Quaternion object.
        """
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        q0 = cr * cp * cy + sr * sp * sy
        q1 = sr * cp * cy - cr * sp * sy
        q2 = cr * sp * cy + sr * cp * sy
        q3 = cr * cp * sy - sr * sp * cy

        return Quaternion(np.array([q0, q1, q2, q3])).norm

    def rotate_by_quat(self, a: float, e: np.ndarray) -> 'Quaternion':
        """
        Rotates the quaternion by an angle around a given axis.

        Args:
            a (float): The angle in radians to rotate.
            e (np.ndarray): The axis of rotation as a 3-element numpy array.

        Returns:
            Quaternion: The rotated quaternion as a Quaternion object.
        """
        dq = Quaternion(np.array([np.cos(a/2), 
                                 np.sin(a/2) * e[0],
                                 np.sin(a/2) * e[1],
                                 np.sin(a/2) * e[2]]))
        return (self.q_prod(dq)).norm
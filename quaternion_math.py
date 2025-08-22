'''
This file contains quaternion math functions used for spacecraft attitude simulation.
'''

import numpy as np


def q_normalize(q):
    '''Normalizes a quaternion.'''
    return q / np.linalg.norm(q)

def q_conj(q):
    '''Returns the conjugate of a quaternion.'''
    return np.array([q[0], -q[1], -q[2], -q[3]])

def q_prod(q1, q2):
    '''Returns the product of two quaternions.'''
    return np.array([
        q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3],
        q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2],
        q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1],
        q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]
    ])

def q_to_euler(q):
    '''Converts a quaternion to Euler angles (roll, pitch, yaw).
    q: quaternion vector
    Returns: Euler angles in radians as a numpy array [roll, pitch, yaw]'''
    return np.array([
        np.arctan2(2*(q[0]*q[1]), 1 - 2*(q[1]**2 - q[2]**2)),
        np.arcsin(2*(q[0]*q[2] + q[1]*q[3])),
        np.arctan2(2*(q[0]*q[3]), 1 - 2*(q[2]**2 - q[3]**2))
    ])
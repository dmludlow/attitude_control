import numpy as np

def q_normalize(q):
    return q / np.linalg.norm(q)

def q_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def q_prod(q1, q2):
    return np.array([
        q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3],
        q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2],
        q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1],
        q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]
    ])

def q_to_euler(q):
    return np.array([
        np.arctan2(2*(q[0]*q[1]), 1 - 2*(q[1]**2 - q[2]**2)),
        np.arcsin(2*(q[0]*q[2] + q[1]*q[3])),
        np.arctan2(2*(q[0]*q[3]), 1 - 2*(q[2]**2 - q[3]**2))
    ])
import numpy as np

def q_normalize(q):
    length = np.linalg.norm(q)
    q[0] = q[0] / length
    q[1] = q[1] / length
    q[2] = q[2] / length
    q[3] = q[3] / length
    return q

def q_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


import numpy as np

def q_normalize(q):
    length = np.sqrt( q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)
    q[0] = q[0] / length
    q[1] = q[1] / length
    q[2] = q[2] / length
    q[3] = q[3] / length
    return q


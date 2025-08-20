import numpy as np

def integrate_omega(w,alpha,dt):     #updates w according euler integration (change to rk4 later)
    w = np.array(w)
    alpha = np.array(alpha)

    w = w + alpha * dt

    return w

def integrate_angle():
    pass
'''
This file contains dynamics and other physics functions required for attitude simulation
'''

import numpy as np
import quaternion_math as qm


def integrate_omega(w,alpha,dt):
    '''Integrates angular velocity using euler's method.

    w: current angular velocity vector
    alpha: angular acceleration vector
    dt: time step in seconds
    Returns: updated angular velocity vector

    **Update to rk4 method later**
    '''
    # Euler's method for integrating angular velocity
    w = w + alpha * dt

    return w

def integrate_angle(q,w,a,dt):
    '''Integrates quaternion position using euler's method.
    
    q: current quaternion vector
    w: current angular velocity vector
    a: angular acceleration vector
    dt: time step in seconds
    Returns: updated quaternion vector
    '''
    # Average angular velocity over the time step
    w_ave = (w + integrate_omega(w, a, dt)) / 2

    # Calculate the change in quaternion based on angular velocity
    # Using the formula dq = 0.5 * q * w * dt
    theta = np.linalg.norm(w_ave * dt)
    
    # If theta is very small, return the original quaternion
    if theta < 1e-6:
        return q
    else:
        # Calculate the quaternion change
        dq = np.array([np.cos(theta/2), np.sin(theta/2) * (w_ave[0] * dt) / theta,
                       np.sin(theta/2) * (w_ave[1] * dt) / theta,
                       np.sin(theta/2) * (w_ave[2] * dt) / theta])
        return qm.q_normalize(qm.q_prod(q,dq))
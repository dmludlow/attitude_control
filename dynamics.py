'''
This file contains dynamics and other physics functions required for attitude simulation
'''

import numpy as np
import quaternion_math as qm

def integrate_omega(w,alpha,dt):
    '''Integrates angular velocity using euler's method.
    **Update to rk4 method later**
    '''
    alpha = np.array(alpha)

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
    w_ave = (w + integrate_omega(w, a, dt)) / 2

    theta = np.linalg.norm(w_ave * dt)
    
    if theta < 1e-6:  # If theta is very small, return the original quaternion
        return q
    else:
        dq = np.array([np.cos(theta/2), np.sin(theta/2) * (w_ave[0] * dt) / theta,
                       np.sin(theta/2) * (w_ave[1] * dt) / theta,
                       np.sin(theta/2) * (w_ave[2] * dt) / theta])
        return qm.q_normalize(qm.q_prod(q,dq))
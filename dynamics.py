import numpy as np
import quaternion_math as qm

def integrate_omega(w,alpha,dt):     #updates w according euler integration (change to rk4 later)
    alpha = np.array(alpha)

    w = w + alpha * dt

    return w

def integrate_angle(q,w,a,dt):
    w_ave = (w + integrate_omega(w, a, dt)) / 2

    theta = np.linalg.norm(w_ave * dt)
    
    if theta < 1e-6:  # If theta is very small, return the original quaternion
        return q
    else:
        dq = np.array([np.cos(theta/2), np.sin(theta/2) * (w_ave[0] * dt) / theta,
                       np.sin(theta/2) * (w_ave[1] * dt) / theta,
                       np.sin(theta/2) * (w_ave[2] * dt) / theta])
        return qm.q_normalize(qm.q_prod(q,dq))
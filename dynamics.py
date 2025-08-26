"""
This file contains dynamics and other physics functions required for attitude simulation
"""

import numpy as np
import attitude_control.quaternion as qm


def integrate_omega(w: np.ndarray, alpha: np.ndarray, dt: float) -> np.ndarray:
    """
    Integrates angular velocity using Euler's method.
    
    Args:
    w: current angular velocity vector
    alpha: angular acceleration vector
    dt: time step in seconds

    Returns: 
    updated angular velocity vector

    **Update to rk4 method later**
    """
    # Euler's method for integrating angular velocity
    w = w + alpha * dt

    return w

def integrate_angle(q: qm.Quaternion, w: np.ndarray, a: np.ndarray, dt: float) -> qm.Quaternion:
    """
    Integrates quaternion position using euler's method.

    Args:
        q: current quaternion vector
        w: current angular velocity vector
        a: angular acceleration vector
        dt: time step in seconds

    Returns:
        Quaternion: updated quaternion vector
    """
    # Average angular velocity over the time step
    w_ave = (w + integrate_omega(w, a, dt)) / 2

    # Calculate the change in quaternion based on angular velocity
    # Using the formula dq = 0.5 * q * w * dt
    theta = np.linalg.norm(w_ave * dt)
    
    # If theta is very small, use small angle approximation
    if theta < 1e-8:
        dq = qm.Quaternion(np.array([1, 
                                    theta/2 * (w_ave[0] * dt),
                                    theta/2 * (w_ave[1] * dt),
                                    theta/2 * (w_ave[2] * dt)]
                                    ))
    else:
        # Calculate the quaternion change
        dq = qm.Quaternion(np.array([np.cos(theta/2), 
                                    np.sin(theta/2) * (w_ave[0] * dt) / theta,
                                    np.sin(theta/2) * (w_ave[1] * dt) / theta,
                                    np.sin(theta/2) * (w_ave[2] * dt) / theta]
                                    ))
    return (q.q_prod(dq)).norm
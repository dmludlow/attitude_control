"""
Slew generator for spacecraft attitude maneuvers.
"""

import numpy as np
from src.attitude_control.plant import quaternion as qm
from src.attitude_control.plant import state as st
from src.attitude_control.command_shaping import trajectory as traj


class Slew:
    
    initial_state: st.State
    desired_state: st.State
    dt: float
    w_max: float
    a_max: float

    def __init__(self, initial_state: st.State,
                 desired_state: st.State,
                 dt: float,
                 w_max: float,
                 a_max: float):
        """
        Initializes the slew generator with initial and desired states, time array, time step, max angular velocity, and max angular acceleration.

        Args:
            initial_state (st.State): Initial state of the spacecraft.
            desired_state (st.State): Desired state of the spacecraft.
            dt (float): Time step in seconds.
            w_max (float): Maximum allowable angular velocity (rad/s).
            a_max (float): Maximum allowable angular acceleration (rad/s^2).
        """
        self.initial_state = initial_state
        self.desired_state = desired_state
        self.dt = dt
        self.w_max = w_max
        self.a_max = a_max

        # Ensure quaternions are normalized
        self.initial_state.norm
        self.desired_state.norm

    def generate_trap(self) -> traj.Trajectory:
        """
        Generates a trapezoidal slew trajectory.

        Returns:
            traj.Trajectory: Generated trajectory with states over time.
        """

        # Plan the rotation
        # Quaternion error (desired * conj(current))
        path = self.desired_state.q.q_prod(self.initial_state.q.q_conj)

        # Enforce shortest rotation: flip entire quaternion if scalar < 0
        if path.q[0] < 0:
            path = path.negative
        # Ensure path is normalized
        path = path.norm
        
        # Extract rotation angle and axis from quaternion
        angle = 2 * np.arccos(path.q[0])
        # Handle edge case of zero rotation
        if np.isclose(angle, 0):
            # No rotation, arbitrary axis
            axis = np.array([1, 0, 0])
        else:
            # Extract rotation axis form quaternion using standard formula
            axis = np.array([path.q[1], path.q[2], path.q[3]]) / np.sin(angle / 2)
        
        # Determine if trapezoidal or triangular profile is needed
        if angle < (self.w_max**2) / (2 * self.a_max):
            # Max angular velocity is not reached before midpoint
            profile_type = 'triangular'
        else:
            # Max angular velocity is reached, coast phase exists
            profile_type = 'trapezoidal'

        # Time intervals
        # Triangular profile
        if profile_type == 'triangular':
            # Acceleration to midpoint
            t_accel = np.sqrt(angle / self.a_max)
            # Deceleration is the same
            t_decel = t_accel
            # No coast phase
            t_coast = 0
            # Total time
            t_total = 2 * t_accel
        # Trapezoidal profile
        else:
            # Acceleration to midpoint
            t_accel = self.w_max / self.a_max
            # Deceleration is the same
            t_decel = t_accel
            # Coast phase
            t_coast = (angle - self.w_max**2/self.a_max) / self.w_max
            # Total time
            t_total = t_accel + t_coast + t_decel

        # Create maneuver time array
        t_maneuver = np.arange(0, t_total, self.dt)

        # Generate angular velocity profile
        # Scalar angular velocity profile 
        w_scalar = np.zeros_like(t_maneuver)
        # Scalar angular velocity profile    
        # Triangular case
        if profile_type == 'triangular':
            for i, t in enumerate(t_maneuver):
                if t < t_accel:
                    # Acceleration phase
                    w_scalar[i] = self.a_max * t
                elif t < t_total:
                    # Deceleration phase
                    w_scalar[i] = self.a_max * (t_total - t)
                else:
                    # End of maneuver
                    w_scalar[i] = 0
        # Trapezoidal case   
        else:
            for i, t in enumerate(t_maneuver):
                if t < t_accel:
                    # Acceleration phase
                    w_scalar[i] = self.a_max * t
                elif t < (t_accel + t_coast):
                    # Coast phase
                    w_scalar[i] = self.w_max
                elif t < t_total:
                    # Deceleration phase
                    w_scalar[i] = self.a_max * (t_total - t)
                else:
                    # End of maneuver
                    w_scalar[i] = 0

        # Multiply by axis to get vector profile
        w_profile = np.outer(w_scalar, axis)

        # Integrate to get attitude profile
        # Preallocate quaternion array
        q_profile = np.empty(len(t_maneuver), dtype=object)

        for i in range(len(t_maneuver)):
            w = w_profile[i]
            # Integrate quaternion (no angular acceleration, so pass zeros)
            if i == 0:
                # First step uses initial state
                q = self.initial_state.q
            else:
                rotation_angle = np.linalg.norm(w * self.dt)
                if rotation_angle < 1e-8:
                    # Small angle approximation
                    dq = qm.Quaternion(np.array([1, 
                                                rotation_angle/2 * (w[0] * self.dt),
                                                rotation_angle/2 * (w[1] * self.dt),
                                                rotation_angle/2 * (w[2] * self.dt)]
                                                ))
                else:
                    dq = qm.Quaternion(np.array([np.cos(rotation_angle/2), 
                                                np.sin(rotation_angle/2) * (w[0] * self.dt) / rotation_angle,
                                                np.sin(rotation_angle/2) * (w[1] * self.dt) / rotation_angle,
                                                np.sin(rotation_angle/2) * (w[2] * self.dt) / rotation_angle]
                                                ))
                q = (q.q_prod(dq)).norm
            q_profile[i] = q

        # Final trajectory
        return traj.Trajectory(t_maneuver, q_profile, w_profile)
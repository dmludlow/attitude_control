"""
This file contains implementation of a PID controller for spacecraft attitude control
"""

"""
Possibly imporvements:
- Add eta max parameter to constructor
- Revisit anti-windup strategy
"""

import numpy as np
import src.attitude_control.controllers as ctrl
import src.attitude_control.plant.state as st

class PID_control(ctrl.Controller):
    
    # Controller gain attributes.
    Kp: np.ndarray
    Ki: np.ndarray
    Kd: np.ndarray

    # Integral error
    integral_err: np.ndarray

    def __init__(self, max_torque: float, Kp: np.ndarray, Ki: np.ndarray, Kd: np.ndarray):
        """
        Initializes the PID controller with specified gains and max torque.
        Args:
            max_torque (float): Maximum allowable torque for the controller.
            Kp (np.ndarray): Proportional gain.
            Ki (np.ndarray): Integral gain.
            Kd (np.ndarray): Derivative gain.
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral_err = np.zeros(3)
        super().__init__(max_torque)

    def torque(self, state_in: st.State, state_desired: st.State, I: np.ndarray, dt: float) -> np.ndarray:
        """
        Computes the control torque.
        Args:
            state_in (st.State): Current state of the spacecraft.
            state_desired (st.State): Desired state of the spacecraft.
            I (np.ndarray): Inertia tensor of the spacecraft.
            dt (float): Time step in seconds.
        Returns:
            np.ndarray: Control torque vector.
        """
        # Quaternion error (desired * conj(current))
        q_e = state_desired.q.q_prod(state_in.q.q_conj)

        # Enforce shortest rotation: flip entire quaternion if scalar < 0
        if q_e.q[0] < 0:
            q_e = q_e.negative
        e = q_e.q[1:]
        # Required torque is the negative of the error for feedback
        # τ = −Kp e we set e = −q_e_vec so a positive desired angle yields
        # a positive body torque in the correct direction.
        e = -e

        # Body-rate error: current - desired (negative feedback for D term)
        w_err = state_in.w - state_desired.w

        # Integral update with angle-meaningful clamp and simple anti-windup
        # Clamp integral growth to +/- eta_max on each axis
        eta_max = 0.2 
        prev_int = self.integral_err.copy()
        cand_int = np.clip(self.integral_err + e * dt, -eta_max, eta_max)

        # Form raw torque with candidate integral (all negative feedback)
        T_raw = -(self.Kp @ e) - (self.Kd @ w_err) - (self.Ki @ cand_int)

        # Add gyroscopic compensation (computed-torque feedforward)
        T_raw = T_raw + np.cross(state_in.w, I @ state_in.w)

        # Saturate to actuator limits
        T_c = np.clip(T_raw, -self.max_torque, self.max_torque)

        # Simple anti-windup: if saturated, revert the integral growth for this step
        if np.any(T_c != T_raw):
            self.integral_err = prev_int
        else:
            self.integral_err = cand_int

        return T_c
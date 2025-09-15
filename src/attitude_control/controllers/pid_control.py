"""
This file contains implimentation of a PID controller for spacecraft attitude control
"""

import numpy as np
import src.attitude_control.controllers as ctrl
import src.attitude_control.object_state.state as st

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
            state_desired (sc.State): Desired state of the spacecraft.
            I (np.ndarray): Inertia tensor of the spacecraft.
            dt (float): Time step in seconds.
        Returns:
            np.ndarray: Control torque vector.
        """
        # Quaternion error
        q_e = state_in.q.q_prod(state_desired.q.q_conj)

        # Take the vector part of the quaternion error for control
        # FIXED: Ensure we take the "short way" rotation
        if q_e.q[0] < 0:
            q_e_vec = -q_e.q[1:]  # Flip to take shorter path
        else:
            q_e_vec = q_e.q[1:]
            
        # Angular velocity error
        w_e = state_desired.w - state_in.w

        # Update integral error
        self.integral_err += q_e_vec * dt
        print(self.integral_err)

        self.integral_err = np.clip(self.integral_err, -1, 1)  # Anti-windup

        # Controller torque calculation
        T_c =  - self.Kp @ q_e_vec + self.Kd @ w_e + self.Ki @ self.integral_err

        # Clip each axis to +/- max_torque
        T_c = np.clip(T_c, -self.max_torque, self.max_torque)
        # print(f"Control Torque: {T_c}")
        # print(self.max_torque)
        return T_c
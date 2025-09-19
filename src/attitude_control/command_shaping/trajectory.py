"""
A module for defining and handling spacecraft trajectories.
"""

import numpy as np
import src.attitude_control.utils.visualization as vis
import src.attitude_control.plant as plant

class Trajectory:
    """
    A class to represent a spacecraft trajectory.

    Attributes:
    time (np.ndarray): Array of time points.
    attitude (np.ndarray): Array of attitude quaternions at each time point.
    angular_velocity (np.ndarray): Array of angular velocities at each time point.
    """
    time: np.ndarray
    attitude: np.ndarray
    angular_velocity: np.ndarray
    
    def __init__ (self, time: np.ndarray, attitude: np.ndarray, angular_velocity: np.ndarray):
        self.time = time
        self.attitude = attitude
        self.angular_velocity = angular_velocity

    def plot_trajectory(self, dt: float):
        """
        Plots and saves the angular velocity and Euler angles over time using the visualization utilities.
        Args:
            dt (float): Time step in seconds.
        """
        import src.attitude_control.utils.visualization as vis
        # Build a list of State objects for visualization
        states = []
        for i in range(len(self.time)):
            q = self.attitude[i]
            w = self.angular_velocity[i]
            # If q is not a Quaternion object, skip or convert
            states.append(
                type('State', (), {'q': q, 'w': w})()
            )
        vis.plot_w(states, dt, filename='angular_velocity_slew.png')
        vis.plot_q(states, dt, filename='quaternion_euler_angles_slew.png')

    def get_state(self, index: int):
        """
        Returns the state at a specific index in the trajectory.

        Args:
            index (int): Index of the desired state.

        Returns:
            A simple object with attributes 'q' (quaternion) and 'w' (angular velocity).
        """
        if index < 0 or index >= len(self.time):
            raise IndexError("Index out of bounds for trajectory states.")
        q = self.attitude[index]
        w = self.angular_velocity[index]
        return plant.state.State(q,w)
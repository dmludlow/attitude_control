"""
This file contains functions for visualizing spacecraft dynamics simulation results.
"""

import matplotlib.pyplot as plt
import numpy as np
import src.attitude_control.plant.quaternion as qm


def plot_w(states_arr: list, dt: float, filename: str):
    """
    Plots the angular velocity over time from the states array.

    Args:
        states_arr: List of State objects containing the angular velocity.
        dt: Time step in seconds.
        filename: Name of the file to save the plot.
    """
    # Extract angular velocities from states
    angular_velocities = np.array([state.w for state in states_arr])
    num_steps = angular_velocities.shape[0]
    # Time array in seconds
    time = np.arange(num_steps) * dt

    # Plotting the angular velocities
    plt.plot(time, angular_velocities[:, 0], label='w_x')
    plt.plot(time, angular_velocities[:, 1], label='w_y')
    plt.plot(time, angular_velocities[:, 2], label='w_z')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.title('Angular Velocity Over Time')
    plt.legend()
    plt.tight_layout()
    plt.savefig(filename)
    plt.close()

def plot_q(states_arr: list, dt: float, filename: str):
    """
    Plots the Euler angles over time from the states array.

    Args:
        states_arr: List of State objects containing the quaternion.
        dt: Time step in seconds.
        filename: Name of the file to save the plot.
    """
    # Extract quaternions from states
    quaternions = np.array([state.q for state in states_arr])
    # Time array in seconds
    time = np.arange(quaternions.shape[0]) * dt

    # Convert quaternions to Euler angles (in radians)
    euler_angles = np.array([q.to_euler_angles for q in quaternions])
    # Convert to degrees
    euler_angles_deg = np.rad2deg(euler_angles)

    # Plotting the Euler angles in degrees
    plt.plot(time, euler_angles_deg[:, 0], label='roll')
    plt.plot(time, euler_angles_deg[:, 1], label='pitch')
    plt.plot(time, euler_angles_deg[:, 2], label='yaw')
    plt.xlabel('Time (s)')
    plt.ylabel('Euler Angles (deg)')
    plt.title('Euler Angles Over Time')
    plt.legend()
    plt.tight_layout()
    plt.savefig(filename)
    plt.close()
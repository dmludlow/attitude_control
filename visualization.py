'''
This file contains functions for visualizing spacecraft dynamics simulation results.
'''

import matplotlib.pyplot as plt
import numpy as np
import spacecraft
import state
import quaternion_math as qm


def plot_w(states_arr, dt):
    '''Plots the angular velocity over time from the states array.

    Inputs:
    states_arr: List of State objects containing the angular velocity.
    dt: Time step in seconds.
    '''

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
    plt.show()

def plot_q(states_arr, dt):
    '''Plots the Euler angles over time from the states array.
    
    Inputs:
    states_arr: List of State objects containing the angular velocity.
    dt: Time step in seconds.
    '''
    # Extract quaternions from states
    quaternions = np.array([state.q for state in states_arr])
    # Time array in seconds
    time = np.arange(quaternions.shape[0]) * dt

    # Convert quaternions to Euler angles
    euler_angles = np.array([qm.q_to_euler(q) for q in quaternions])

    # Plotting the Euler angles
    plt.plot(time, euler_angles[:, 0], label='w_x')
    plt.plot(time, euler_angles[:, 1], label='w_y')
    plt.plot(time, euler_angles[:, 2], label='w_z')
    plt.xlabel('Time (s)')
    plt.ylabel('Euler Angles (rad)')
    plt.title('Euler Angles Over Time')
    plt.legend()
    plt.show()
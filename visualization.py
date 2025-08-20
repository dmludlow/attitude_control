import matplotlib.pyplot as plt
import numpy as np
import spacecraft
import state
import quaternion_math as qm

def plot_w(states_arr, dt):
    angular_velocities = np.array([state.w for state in states_arr])
    num_steps = angular_velocities.shape[0]
    time = np.arange(num_steps) * dt  # Time array in seconds

    plt.plot(time, angular_velocities[:, 0], label='w_x')
    plt.plot(time, angular_velocities[:, 1], label='w_y')
    plt.plot(time, angular_velocities[:, 2], label='w_z')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.title('Angular Velocity Over Time')
    plt.legend()
    plt.show()

def plot_q(states_arr, dt):
    quaternions = np.array([state.q for state in states_arr])
    time = np.arange(quaternions.shape[0]) * dt  # Time array in seconds

    euler_angles = np.array([qm.q_to_euler(q) for q in quaternions])

    plt.plot(time, euler_angles[:, 0], label='w_x')
    plt.plot(time, euler_angles[:, 1], label='w_y')
    plt.plot(time, euler_angles[:, 2], label='w_z')
    plt.xlabel('Time (s)')
    plt.ylabel('Euler Angles (rad)')
    plt.title('Euler Angles Over Time')
    plt.legend()
    plt.show()
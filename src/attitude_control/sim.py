"""
This file contains the main simulation loop for a spacecraft attitude control system.
"""

import numpy as np
import copy
import src.attitude_control.spacecraft as sc
import src.attitude_control.utils.visualization as vis
import src.attitude_control.object_state.state as st
import src.attitude_control.object_state.quaternion as qm
import src.attitude_control.controllers as ctrl


initial_state = st.State(
    # Initial quaternion representing no rotation
    qm.Quaternion(np.array([1, 0, 0, 0])), 
    np.array([3, 3, 3])    
)

# Time for the simulation to run in seconds.
time = 200
# Time step in seconds
dt = 0.001
# Total simulation steps
steps = int(time / dt)

# Define Inertia Tensor
# Example: 
I = np.array([
    [1009.86, 18.05, -21.26],
    [18.05, 811.94, -37.83],
    [-21.26, -37.83, 803.24]
])

# Create a PD controller instance
# Hard coded gains for now, tune later
Kp = np.diag([2e-3, 2e-3, 2e-3])
Kd = np.diag([5e-4, 5e-4, 5e-4])

Kp = Kp * 1.0
Kd = Kd * 1.0


# Tuned PID controller gains
kp = 0.06
ki = 0
kd = 1.8

Kp = np.diag([kp, kp, kp])
Ki = np.diag([ki, ki, ki])
Kd = np.diag([kd, kd, kd])


# Max torque the controller can apply (N*m)
max_torque = 100

test_cont = ctrl.PID_control(max_torque, Kp, Ki, Kd)

# Create a spacecraft instance with the initial state
test_craft = sc.Spacecraft(I, initial_state, test_cont)

# Desired state: roll, pitch, and yaw of 30 degrees
roll = np.deg2rad(10)
pitch = np.deg2rad(10)
yaw = np.deg2rad(10)
q_desired = qm.Quaternion.from_euler_angles(roll, pitch, yaw)

# Desired angular velocity of zero
w_desired = np.array([0, 0, 0])

state_desired = st.State(q_desired, w_desired)

# List to store states for visualization
states = []

# Run the simulation for a number of steps
for i in range(steps):
    # Compute control torque using the PD controller
    T_cur = test_craft.controller.torque(test_craft.state,
                                         state_desired,
                                         test_craft.I,
                                         dt)

    # Update the spacecraft state with the computed torque
    test_craft.step(T_cur, dt)
    # Store a copy of the current state
    states.append(copy.deepcopy(test_craft.state))

# Visualize the results
vis.plot_w(states,dt)
vis.plot_q(states,dt)

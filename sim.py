"""
This file contains the main simulation loop for a spacecraft attitude control system.
"""

import numpy as np
import copy
import attitude_control.spacecraft as sc
import attitude_control.visualization as vis
import attitude_control.state as st
import attitude_control.quaternion as qm
import attitude_control.controllers as ctrl


initial_state = st.State(
    # Initial quaternion representing no rotation
    qm.Quaternion(np.array([1, 0, 0, 0])), 
    np.array([0, 0, 0])    
)

# Time for the simulation to run in seconds.
time = 20
# Time step in seconds
dt = 0.001
# Total simulation steps
steps = int(time / dt)

# Define Inertia Tensor
# Example: 1U CubeSat 
I = np.array([
    [2.3e-3, -1e-5, 2e-5],
    [-1e-5, 2.1e-3, -0.5e-5],
    [2e-5, -0.5e-5, 2.25e-3]
])

# Create a PD controller instance
# Hard coded gains for now, tune later
Kp = np.diag([5.1*10e-3, 4.7*10e-3, 5.0*10e-3])
Kd = np.diag([6.2*10e-3, 5.7*10e-3, 6.0*10e-3])

Kp = Kp / 200
Kd = Kd / 200

# Max torque the controller can apply (N*m)
max_torque = 0.01

test_cont = ctrl.PD_control(max_torque,Kp, Kd)

# Create a spacecraft instance with the initial state
test_craft = sc.Spacecraft(I, initial_state, test_cont)

# Desired state: roll, pitch, and yaw of 30 degrees
roll = np.deg2rad(30)
pitch = np.deg2rad(30)
yaw = np.deg2rad(30)
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
                                         test_craft.I)

    # Update the spacecraft state with the computed torque
    test_craft.step(T_cur, dt)
    # Store a copy of the current state
    states.append(copy.deepcopy(test_craft.state))

# Visualize the results
vis.plot_w(states,dt)
vis.plot_q(states,dt)

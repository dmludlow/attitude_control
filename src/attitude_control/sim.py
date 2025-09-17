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
    q=qm.Quaternion(np.array([1, 0, 0, 0])), 
    w=np.array([-0.2, 0.1, -0.1])    
)

# Time for the simulation to run in seconds.
time = 200
# Time step in seconds
dt = 0.01
# Total simulation steps
steps = int(time / dt)

# Define Inertia Tensor
# Diagonal inertia tensor (off-diagonal elements set to zero)
I = np.array([
    [1009.86, 18.05, -21.26],
    [18.05, 811.94, -37],
    [-21.26, -37, 803.24]
])

# Tuned PID controller gains
kp = 230
ki = 10
kd = 800

Kp = np.diag([kp, kp*0.75, kp*0.75])
Ki = np.diag([ki, ki, ki])
Kd = np.diag([kd, kd*0.75, kd*0.75])

#Kp = np.diag([80.7888, 64.9552, 64.2592])
#Kd = np.diag([363.5496, 292.2984, 289.1664])
#Ki = np.diag([1.615776, 1.299104, 1.285184])

# Max torque the controller can apply (N*m)
max_torque = 100

test_cont = ctrl.PID_control(max_torque, Kp, Ki, Kd)

# Create a spacecraft instance with the initial state
test_craft = sc.Spacecraft(I, initial_state, test_cont)

# Desired state: roll, pitch, and yaw of __ degrees
roll = np.deg2rad(30)
pitch = np.deg2rad(40)
yaw = np.deg2rad(20)
q_desired = qm.Quaternion.from_euler_angles(roll, pitch, yaw)

# Desired angular velocity of zero
w_desired = np.array([0, 0, 0])

state_desired = st.State(q_desired, w_desired)

# List to store states for visualization
states = []

# Run the simulation for a number of steps
for i in range(steps):

    # Optional: Add some random disturbance torque
    random_disturbance = np.random.normal(0, 5, 3)

    # Compute control torque using the PD controller
    T_cur = test_craft.controller.torque(test_craft.state,
                                         state_desired,
                                         test_craft.I,
                                         dt)
    # Update the spacecraft state with the computed torque
    test_craft.step(T_cur + random_disturbance, dt)
    # Store a copy of the current state
    states.append(copy.deepcopy(test_craft.state))

# Visualize the results
vis.plot_w(states,dt)
vis.plot_q(states,dt)
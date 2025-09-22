"""
This file contains the main simulation loop for a spacecraft attitude control system.
"""

"""
Improvments to make:
- Convert into a class (with time as attibute)
- Revisit and streamline random disturbance torques
- Add basic front end to call sim, streamline input of inertia, initial conditions, controller type, etc.
- Make actuators more realistic (rate limits and startup torques)
- Make sure angles are mapped to correct angles
- Add orbital dynamics, propagation, and major disturbance torques
- Investigate different coordinate systems
"""

import numpy as np
import copy
import src.attitude_control.plant.spacecraft as sc
import src.attitude_control.utils.visualization as vis
import src.attitude_control.plant.state as st
import src.attitude_control.plant.quaternion as qm
import src.attitude_control.controllers as ctrl
import src.attitude_control.command_shaping as cmd


initial_state = st.State(
    # Initial quaternion representing no rotation
    q=qm.Quaternion(np.array([1, 0, 0, 0])), 
    w=np.array([0, 0, 0])    
)

# Time for the simulation to run in seconds.
time = 150
# Time step in seconds
dt = 0.001
# Total simulation steps
steps = int(time / dt)
# Generate time array
t = np.arange(0, time, dt)

# Define Inertia Tensor
# Diagonal inertia tensor (off-diagonal elements set to zero)
I = np.array([
    [1009.86, 18.05, -21.26],
    [18.05, 811.94, -37],
    [-21.26, -37, 803.24]
])

# Tuned PID controller gains
kp = 400
ki = 3
kd = 1500

diag_gain = 0

Kp = np.array([[kp, kp*diag_gain, kp*diag_gain],
               [kp*diag_gain, kp, kp*diag_gain],
               [kp*diag_gain, kp*diag_gain, kp]])

Ki = np.array([[ki, ki*diag_gain, ki*diag_gain],
               [ki*diag_gain, ki, ki*diag_gain],
               [ki*diag_gain, ki*diag_gain, ki]])

Kd = np.array([[kd, kd*diag_gain, kd*diag_gain],
               [kd*diag_gain, kd, kd*diag_gain],
               [kd*diag_gain, kd*diag_gain, kd]])

# Max torque the controller can apply (N*m)
max_torque = 100

test_cont = ctrl.PID_control(max_torque, Kp, Ki, Kd)

# Create a spacecraft instance with the initial state
test_craft = sc.Spacecraft(I, initial_state, test_cont)

# Desired state: roll, pitch, and yaw of __ degrees
roll = np.deg2rad(60)
pitch = np.deg2rad(70)
yaw = np.deg2rad(-40)
q_desired = qm.Quaternion.from_euler_angles(roll, pitch, yaw)

# Desired angular velocity of zero
w_desired = np.array([0, 0, 0])

state_desired = st.State(q_desired, w_desired)
state_desired_og = copy.deepcopy(state_desired)

# Slew generation
slew_gen = cmd.Slew(initial_state,
                    state_desired,
                    dt,
                    # Max angular velocity (rad/s)
                    w_max=np.deg2rad(5),
                    # Max angular acceleration (rad/s^2)
                    a_max=np.deg2rad(1)
                    )
# Generate the trapezoidal slew trajectory
test_traj = slew_gen.generate_trap()

# Plot trajectory details
test_traj.plot_trajectory(dt)


# List to store states for visualization
states = []

# Run the simulation for a number of steps
for i in range(steps):

    # Optional: Add some random disturbance torque
    random_disturbance = np.random.normal(0, 5, 3)
    #random_disturbance = np.array([0,0,0])

    #"""
    # Get the desired state from the trajectory at the current time step
    if i < len(test_traj.time):
        state_desired = test_traj.get_state(i)
    else:
        state_desired = state_desired_og
    #"""
        
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
vis.plot_w(states,dt, 'angular_velocity.png')
vis.plot_q(states,dt, 'quaternion_euler_angles.png')
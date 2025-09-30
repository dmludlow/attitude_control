"""
This is the main file to run the spacecraft attitude control simulation.
"""

"""
TODO:
- Revisit and streamline random disturbance torques
- Add basic front end to call sim, streamline input of inertia, initial conditions, controller type, etc.
- Make actuators more realistic (rate limits and startup torques)
- Add orbital dynamics, propagation, and major disturbance torques
- Look into various coordinate systems
- Improve efficiency

Ultimate goal is to make two spacecraft point at eachother in orbit.
"""

import numpy as np
import src.attitude_control.plant.spacecraft as sc
import src.attitude_control.simulation as sim
import src.attitude_control.plant.state as st
import src.attitude_control.plant.quaternion as qm
import src.attitude_control.controllers as ctrl

# Time step in seconds
dt = 0.01

# Time for the simulation to run in seconds.
time = 200

# Initial state of the spacecraft
initial_state = st.State(
    # Initial quaternion representing no rotation
    q=qm.Quaternion(np.array([1, 0, 0, 0])), 
    w=np.array([0, 0, 0])    
)

# Define Inertia Tensor
I = np.array([
    [1009.86, 18.05, -21.26],
    [18.05, 811.94, -37],
    [-21.26, -37, 803.24]
])

# Making the controller
# Tuned PID controller gains
kp = 400
ki = 3
kd = 1500

Kp = np.diag([kp, kp, kp])
Ki = np.diag([ki, ki, ki])
Kd = np.diag([kd, kd, kd])

# Max torque the controller can apply (N*m)
max_torque = 100

# Make PID controller object
controller1 = ctrl.PID_control(max_torque, Kp, Ki, Kd)

roll1 = np.deg2rad(30)
pitch1 = np.deg2rad(30)
yaw1 = np.deg2rad(30)
q_desired1 = qm.Quaternion.from_euler_angles(roll1, pitch1, yaw1)

roll2 = np.deg2rad(70)
pitch2 = np.deg2rad(70)
yaw2 = np.deg2rad(70)
q_desired2 = qm.Quaternion.from_euler_angles(roll2, pitch2, yaw2)

roll3 = np.deg2rad(-60)
pitch3 = np.deg2rad(-60)
yaw3 = np.deg2rad(-60)
q_desired3 = qm.Quaternion.from_euler_angles(roll3, pitch3, yaw3)

goal_state1 = st.State(q_desired1,np.array([0, 0, 0]))
goal_state2 = st.State(q_desired2,np.array([0, 0, 0]))
goal_state3 = st.State(q_desired3,np.array([0, 0, 0]))

# Define corresponding times to reach each goal state
goal_times = np.array([50, 100, 150])  # seconds

# Combine states and times into state_goal
state_goal = np.array([[goal_state1, goal_state2, goal_state3], goal_times])

# Make spacecraft object
craft1 = sc.Spacecraft(I,initial_state, controller1, state_goal)

state_goal = np.array([[goal_state3, goal_state2, goal_state1], goal_times])
craft2 = sc.Spacecraft(I,initial_state, controller1, state_goal)

# List of spacecraft to simulate
crafts = [craft1, craft2]

simulation = sim.Simulation(dt, time, crafts)

# Run the simulation
simulation.run()

# Visualize the results
simulation.visualize()
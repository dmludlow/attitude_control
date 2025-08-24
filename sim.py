"""
This file contains the main simulation loop for a spacecraft attitude control system.
"""

import numpy as np
import copy
import attitude_control.spacecraft as sc
import attitude_control.visualization as vis
import attitude_control.state as st
import attitude_control.quaternion as qm


initial_state = st.State(
    # Initial quaternion representing no rotation
    qm.Quaternion(np.array([1, 0, 0, 0])), 
    np.array([0, 0, 0])    
)

# Create a spacecraft instance with the initial state
test_craft = sc.Spacecraft(initial_state)

# Time for the simulation to run in seconds.
time = 20
# Time step in seconds
dt = 0.1
# Total simulation steps
steps = int(time / dt)

# Define angular acceleration
# Testing angular acceleration that varies over time
a = []
for j in range(steps):
    current_time = j * dt
    if current_time < 5:
        a_cur = 2
    elif current_time < 10:
        a_cur = 6
    else:
        a_cur = -4
        
    a.append(np.array([0,a_cur,0]))

# List to store states for visualization
states = []

# Run the simulation for a number of steps
for i in range(steps):
    test_craft.step(a[i], dt)
    # Store a copy of the current state
    states.append(copy.deepcopy(test_craft.state))
    # Print the final state of the spacecraft for testing
    print(test_craft.state)
    print("\n")

# Visualize the results
vis.plot_w(states,dt)
vis.plot_q(states,dt)

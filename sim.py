'''
This file contains the main simulation loop for a spacecraft attitude control system.
'''

import numpy as np
import spacecraft as sc
import visualization as vis
import copy
import state


initial_state = state.State(
    # Initial quaternion representing no rotation
    np.array([1, 0, 0, 0]), 
    np.array([0, 0, 0])    
)

# Create a spacecraft instance with the initial state
test_craft = sc.Spacecraft(initial_state)
# Define angular acceleration vector and time step
a = np.array([0,2,0])
dt = 0.1

# List to store states for visualization
states = []

# Run the simulation for a number of steps
for i in range(80):
    test_craft.step(a, dt)
    # Store a copy of the current state
    states.append(copy.deepcopy(test_craft.state))
    # Print the final state of the spacecraft for testing
    print(test_craft.state)
    print("\n")

# Visualize the results
vis.plot_w(states,dt)
vis.plot_q(states,dt)
import numpy as np
import spacecraft as sc
import visualization as vis
import copy
import state

initial_state = state.State(
    np.array([1, 0, 0, 0]), 
    np.array([0, 0, 0])    
)

test_craft = sc.Spacecraft(initial_state)
a = np.array([0,2,0])
dt = 0.1

states = []

for i in range(80):
    test_craft.step(a, dt)
    states.append(copy.deepcopy(test_craft.state))  # Store a copy of the state
    print(test_craft.state)
    print("\n")

vis.plot_w(states,dt)
vis.plot_q(states,dt)

print(states)
import numpy as np
import spacecraft as sc

test_craft = sc.Spacecraft()
a = np.array([2,2,2])
dt = 0.1

for i in range(20):
    test_craft.step(a, dt)
    print(test_craft.state)
    # This will print the state of the spacecraft after each step

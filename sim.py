import numpy as np
import spacecraft as sc
import visualization as vis

test_craft = sc.Spacecraft()
a = np.array([2,2,2])
dt = 0.1

sim_storage = []

for i in range(20):
    test_craft.step(a, dt)
    sim_storage.append(test_craft)
    print(test_craft.state)
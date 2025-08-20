import matplotlib.pyplot as plt
import numpy as np
import spacecraft
import state

# working, but not what i want, keep as refernce
def plot_w(craft_arr):  
    for i, craft in enumerate(craft_arr):
        w = np.array(craft.state.w)
        if w.ndim == 2:  # shape: (timesteps, 3)
            for j in range(w.shape[1]):
                plt.plot(w[:, j], label=f'Craft {i} w{j}')
        else:
            plt.plot(w, label=f'Craft {i}')
    plt.xlabel('Timestep')
    plt.ylabel('Angular Velocity')
    plt.legend()
    plt.show()
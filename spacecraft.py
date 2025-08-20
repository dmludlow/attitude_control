import state
import numpy as np

class Spacecraft:

    def __init__(self,state_in = None):
        if state_in is None:
            default_state = state.State(
                np.array([0,0,0]),
                np.array([0,0,0]),
                np.array([1,0,0,0]),
                np.array([2,3,4]))
            self.state = default_state
        else:
            self.state = state_in

    def step(self, a, dt):
        self.state.step(a, dt)
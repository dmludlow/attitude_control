'''
This file defines the Spacecraft class for simulating spacecraft dynamics.
'''

import state
import numpy as np

class Spacecraft:

    def __init__(self,state_in = None):   # have to add i matrix
        if state_in is None:
            default_state = state.State(
                np.array([1,0,0,0]),
                np.array([0,0,0]))
            self.state = default_state
        else:
            self.state = state_in

    def step(self, a, dt):
        '''Updates the spacecraft state based on angular acceleration vector and time step.'''
        self.state.step(a, dt)
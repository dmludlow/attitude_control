'''
This file defines the State class for simulating spacecraft dynamics.

It manages the spacecraft's quaternion and angular velocity state.

It includes methods for updating the state based on angular acceleration and time step,
normalizing the quaternion, and converting the state to a string representation.
'''

import quaternion_math as qm
import numpy as np
import dynamics as dy

class State:
    def __init__(self,q,w):  
        self.q = np.asarray(q, dtype=float)                #np array with quaternion [r,x,y,z]
        self.w = np.asarray(w, dtype=float)                #np array with angular velocity [x,y,z]

    def step(self,a,dt):
        '''Updates the state based on angular acceleration vector and time step.'''
        self.w = dy.integrate_omega(self.w,a,dt)              # update angular velocity
        self.q = dy.integrate_angle(self.q, self.w, a, dt)    # update quaternion
    
    def normalize(self):
        '''Normalizes the quaternion.'''
        self.q = qm.q_normalize(self.q)

    def  __str__(self):
        '''Returns a string representation of the state.'''
        return (
            f"State(   q = {self.q},     w = {self.w})\n"
            f"         euler = {qm.q_to_euler(self.q)}"
        )

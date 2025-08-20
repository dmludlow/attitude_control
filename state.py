import quaternion_math as qm
import numpy as np
import dynamics as dy

class State:
    def __init__(self,r,v,q,w):
        self.r = np.asarray(r, dtype=float)                #np array with position [x,y,z]
        self.v = np.asarray(v, dtype=float)                #np array with translational velocity [x,y,z]
        self.q = np.asarray(q, dtype=float)                #np array with quaternion [r,x,y,z]
        self.w = np.asarray(w, dtype=float)                #np array with angular velocity [x,y,z]

    def step(self,a,dt):
        self.w = dy.integrate_omega(self.w,a,dt)              # update angular velocity
        self.q = dy.integrate_angle(self.q, self.w, a, dt)    # update quaternion
    
    def normalize(self):                                        # normalizes current state quaternion
        self.q = qm.q_normalize(self.q)

    def  __str__(self):
        return (
            f"State(   q = {self.q},     w = {self.w})\n"
            f"         euler = {qm.q_to_euler(self.q)}"
        )

import quaternion_math as qm
import numpy as np

class State:
    def __init__(self,r,v,q,w):
        self.r = np.asarray(r, dtype=float)                        #np array with position [x,y,z]
        self.v = np.asarray(v, dtype=float)                        #np array with translational velocity [x,y,z]
        self.q = np.asarray(q, dtype=float)                        #np array with quaternion [r,x,y,z]
        self.w = np.asarray(w, dtype=float)                        #np array with angular velocity [x,y,z]

    def vector_to_q(self):              # converts vector to quaternion
        pass
    
    def q_to_vector(self):              # expresses state as vector
        pass
    
    def normalize(self):                # normalizes current state quaternion
        self.q = qm.q_normalize(self.q)

    def  __str__(self):
        return f"State(q = {self.q}, w = {self.w})"

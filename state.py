import quaternion_math as qm

class State:
    def __init__(self,q,w):
        self.q = q                  #np array with [r,x,y,z]
        self.w = w                  #np array with roll rates [x,y,z]

    def vector_to_q(self):              # converts vector to quaternion
        pass
    
    def q_to_vector(self):              # expresses state as vector
        pass
    
    def normalize(self):                # normalizes current state quaternion
        self.q = qm.q_normalize(self.q)

    def  __str__(self):
        return f"State(q = {self.q}, w = {self.w})"

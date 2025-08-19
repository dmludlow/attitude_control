class State:
    def __init__(self,q,w):
        self.q = q                  #np array with [r,x,y,z]
        self.w = w                  #np array with roll rates [x,y,z]

    def vector_to_q():              # converts vector to quaternion
        pass
    
    def q_to_vector():              # expresses state as vector
        pass
    
    def normalize():                # normalizes current state quaternion
        pass
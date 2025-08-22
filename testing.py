'''
This file is for small scale testing of the ACS_sim package.
'''

import state as st
import quaternion_math as qm
import dynamics as dy
import numpy as np
import visualization as vis


w = np.array([2,3,4])
a = np.array([2,2,2])
t = 0.002

print(dy.w_step(w,a,t))

"""
Test the functionality of the Bezier polynomial class in "class_bz.py"

@author: rt-2pm2
"""

import numpy as np
import sys
import os
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../trjgen')))

import trjgen.class_bz as bz

np.set_printoptions(precision=6)
np.set_printoptions(suppress=True)

#Solution Position: [3.115, -0.890, 0.113]
#Solution Velocity: [1.493, 0.083, -0.079]
#Solution Acceleration: [-2.488, -0.138, -9.602]
#Final Relative Position: [3.016, -0.895, 0.113]
#Final Velocity: [1.990, 0.110, -0.079]
#Final Acceleration: [-2.488, -0.138, -9.602]

# Build the waypoint matrix
x0 = 0.0
X = np.array([
        [ 0,  3.101], # p
        [ 0,  1.99], # v
        [ 0,  -2.48], # a
        ])

x_lim = [3.5, 2.5, 7.0]
x_cnstr = np.array([[-x0, x_lim[0]], [-x_lim[1], x_lim[1]], [-9.90, x_lim[2]]])

# Generate the polynomial
T = 2.5
bz_x = bz.Bezier(waypoints=X, constraints=x_cnstr, degree=17, s=T)

print("Evaluation of the bezier polynomial")
print(bz_x.eval(T, [0,1,2]))



#### PLOT
N = 100
test_y = np.zeros((N, 3), dtype=float)
t = np.zeros((N), dtype=float)

Xtj = np.zeros((N, 3), dtype=float)
for i in range(N):
    t[i] = T/(N - 1) * i
    Xtj[i, :] = bz_x.eval(t[i], [0,1,2])

fig, axs = plt.subplots(3, 1)
axs[0].plot(t, Xtj[:, 0], t, np.ones(N) * x_cnstr[0,0], t, np.ones(N) * x_cnstr[0,1], T, X[0,1], 'o')
axs[0].set_title("p")
axs[1].plot(t, Xtj[:, 1], t, np.ones(N) * x_cnstr[1,0], t, np.ones(N) * x_cnstr[1,1], T, X[1,1], 'o')
axs[1].set_title("v")
axs[2].plot(t, Xtj[:, 2], t, np.ones(N) * x_cnstr[2,0], t, np.ones(N) * x_cnstr[2,1], T, X[2,1], 'o')
axs[2].set_title("a")

plt.show()



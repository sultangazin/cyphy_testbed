# Guidance class
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
import numpy as np

from guidance_helper import *

DT = 1.0

Vi = np.array([1.0, 0.0, 0.0])

Xf = np.zeros(3, dtype=float)
Af = np.array([0.0, 0.0, -9.8])
Vf = np.array([1.0, 0.0, Vi[2] + DT * Af[2]])

angle = -math.pi/2

qt = np.array([math.cos(angle), 0, math.sin(angle), 0]

Y = Integration(Xf, Vf, Af, DT, 0.001, -1, qt)  

print("X0 = ", Y[0:3])
print("V0 = ", Y[3:6])



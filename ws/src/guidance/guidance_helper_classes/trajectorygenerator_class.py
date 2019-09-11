# Guidance class
import numpy as np    

################# TRAJECTORY GENERATOR ###############
# This class incapsulates the polynomial trajectory
# generators.
class TrajectoryGenerator:
    
    def __init__(self, trj_obj):
        # Set the trajectory object
        self.trj_obj = trj_obj
        # Set the duration of the current trajectory
        self.t_stop = trj_obj.duration

    def eval(self, t): 
        """
        Eval the trajectory object at time 't'
        """
        X = np.zeros(3, dtype=float)
        Y = np.zeros(3, dtype=float)
        Z = np.zeros(3, dtype=float)
        W = np.zeros(3, dtype=float)
        R = np.eye(3)
        Omega = np.zeros(3, dtype=float)
   
        if (t <= self.t_stop):
            # Update the current reference
            (X, Y, Z, W, R, Omega) = self.trj_obj.eval(t, [0, 1, 2, 3])
        else:
            (X, Y, Z, W, R, Omega) = self.trj_obj.eval(self.t_stop, [0, 1, 2, 3])

        return (X, Y, Z, W, R, Omega)

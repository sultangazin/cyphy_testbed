# Guidance class
import numpy as np
import enum

class TrajectoryType(enum.Enum):
    FullTrj = 0
    AttTrj = 1
    Landing = 2
    Takeoff = 4

class MissionType(enum.Enum):
    Simple = 0
    Composite = 1


class ConstAttitudeTrj:
    def __init__(self):
        self.r = 0
        self.p = 0
        self.y = 0
        
    def set(self, r, p, y):
        self.r = r
        self.p = p
        self.y = y

    def eval(self, t):
        return (self.r, self.p, self.y)


################ MISSION CLASS ##################
# This class contains the information about the 
# current mission.
class Mission:
    def __init__(self):
        self.start_pos = np.zeros(3, dtype=float)        
        self.start_vel = np.zeros(3, dtype=float) 
        self.start_acc = np.zeros(3, dtype=float)
        
        self.end_pos = np.zeros(3, dtype=float)        
        self.end_vel = np.zeros(3, dtype=float) 
        self.end_acc = np.zeros(3, dtype=float)

        self.t_start = 0.0
        self.t_stop = 0.0

        # Variables for storing the current control ref
        self.X = np.zeros(3, dtype=float)
        self.Y = np.zeros(3, dtype=float)
        self.Z = np.zeros(3, dtype=float)
        self.W = np.zeros(3, dtype=float)
        self.R = np.eye(3)
        self.Omega = np.zeros(3, dtype=float)

        self.Euler = np.zeros(3, dtype=float)

        self.isActive = False 
        self.TrjType = TrajectoryType.FullTrj 
 
    def update(self, p, tg_p, trj_gen, start_time, stop_time,
            v = None, a = None, tg_v = None, tg_a = None, ttype = TrajectoryType.FullTrj):
        # Update the starting point
        self.start_pos = p
        if (v is not None):
            self.start_vel = v
        else:
            self.start_vel = np.zeros(3, dtype=float)
        if (a is not None):
            self.start_acc = a
        else:
            self.start_acc = np.zeros(3, dtype=float)

         # Update the target point
        self.end_pos = tg_p
        if (tg_v is not None):
            self.end_vel = tg_v
        else:
            self.end_vel = np.zeros(3, dtype=float)
        if (tg_a is not None):
            self.end_acc = tg_a
        else:
            self.end_acc = np.zeros(3, dtype=float)
    
        self.trj_gen = trj_gen
         
        self.t_start = start_time
        self.t_stop = stop_time

        self.TrjType = ttype 
    
    def getRef(self, t):
        """ 
        This will return the trajectory reference, updating the 
        trajectory type giving the relative time from the start.
        """
        rel_t = t - self.t_start

        # If the mission request just the control of the attitude...
        if (self.TrjType == TrajectoryType.AttTrj):
            (self.Euler[0], self.Euler[1], self.Euler[2]) = self.trj_gen.eval(rel_t)
            return (self.Euler[0], self.Euler[1], self.Euler[2])
        else:
            (self.X, self.Y, self.Z, self.W, self.R, self.Omega) = self.trj_gen.eval(rel_t)
            self.X[0] = self.X[0] + self.start_pos[0]
            self.Y[0] = self.Y[0] + self.start_pos[1]
            self.Z[0] = self.Z[0] + self.start_pos[2]

            return (self.X, self.Y, self.Z, self.W, self.R, self.Omega)
        

    def queryStatus(self, t):
        """
        Return the status of the Mission
        looking at the time of the planned
        trajectory.
        """
        if (t > np.max(self.t_stop)):
            self.isActive = False
            return self.isActive
        else:
            self.isActive = True 
            return self.isActive
        
    def getTrjType(self):
        """
        Return the type of trajectory to track: Full or Att
        """
        return self.TrjType

    def getStartTime(self):
        return self.t_start

    def getStopTime(self):
        return self.t_stop
    
    def getStart(self):
        return self.start_pos

    def getEnd(self):
        return (self.end_pos, self.end_vel, self.end_acc)

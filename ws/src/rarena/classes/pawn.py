from arena import *
from testbed_msgs.msg import CustOdometryStamped
import rospy
import numpy as np

from classes import ROSArenaObject

###### HELPERS #####
def posFromStateMsg(msg):
    pos = np.array([msg.p.x - 0.045, msg.p.z, -msg.p.y - 0.15], dtype=float)
    return pos

def quatFromStateMsg(msg):
    quat = np.array([-msg.q.x, 
                     msg.q.z,
                     msg.q.y,
                     msg.q.w], dtype=float)
    return quat


class PawnObject(ROSArenaObject):
    """
    Class inheriting from the Pawn class but with more advanced features.
    """
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.mission_active = False

        path = [Position(0,0,0), Position(1,1,1), Position(2,1,1)]
        self.trajectory = ThickLine(path = path, lineWidth = 5)

        self.final_pos = self.location

        # Deal with the ROS stuff
        self.register_sources()
        self.register_services()

        self.arena_obj_.update_attributes(
                evt_handler = self.arena_callback)

        # Add the object to the ARENA server
        #self.arena_srv.add_object(self.arena_obj_)
        self.arena_srv.add_object(self.trajectory)

    def __del__(self):
        print("{} Destructor...\n".format(self.id))
        if self.trajectory is not None:
            self.arena_srv.delete_object(self.trajectory)

        super().__del__()
        #if self.arena_obj_ is not None:
        #    self.arena_srv.delete_object(self.arena_obj_)



    def delete_obj(self):
        print("{} Deleting...\n".format(self.id))
        if self.trajectory is not None:
            self.arena_srv.delete_object(self.trajectory)
        #self.arena_srv.delete_object(self.arena_obj_)
        super().delete_obj()


    # Register to the source of object's pose data
    def register_sources(self):
        self.state_topic = "/{}/external_codom".format(self.id)
        self.trj_topic = "/{}/trajectory".format(self.id)
        rospy.Subscriber(self.state_topic,
                CustOdometryStamped,
                self.state_callback)


    def get_arena_obj(self):
        return [self.arena_obj_, self.trajectory]


    # Callback for the pose messages from ROS
    def state_callback(self, state_msg):
        # Update pose information
        p = posFromStateMsg(state_msg)
        r = quatFromStateMsg(state_msg)

        super().set_position(p)
        super().set_rotation(r)

        #if np.linalg.norm(self.location - self.final_pos) < 0.1:
        #    self.mission_active = False
        #    self.arena_srv.delete_object(self.trajectory)
            
            
    def arena_update(self):
        if (self.mission_active and self.trajectory is not None):
            self.arena_srv.update_object(self.trajectory)

        super().arena_update()


    def arena_callback(self, evt):
        """
        Callback for the events triggered in the ARENA
        """
        if (evt.event_type  == mouseup):
            if self.selected:
                self.selected = False
                print("deactivate ({})".format(self.objName))
            else:
                self.selected = True
                print("activate ({})".format(self.objName))

    
    def traj_callback(self, msg):
        self.mission_active = True
        self.remove_trajectory()
        #self.start_time = rospy.get_time()
        init_pos = self.position
        
        for curve in msg.mission:
            self.final_pos = self.draw_curve(curve, Nsamples, init_pos)
            

    def draw_curve(self, curve, Nsamples, init_pos=0):
        c_x = Bezier(curve.coeff_x)
        c_y = Bezier(curve.coeff_y)
        c_z = Bezier(curve.coeff_z)

        dt = 1/Nsamples
        time = 0

        path = []
        while time <= 1.0:
            pos = [c_x.eval(time)[0] + init_pos[0], 
                   c_z.eval(time)[0] + init_pos[1],
                   -c_y.eval(time)[0] + init_pos[2]]
            time = time + dt
            path = path + [[pos]]

        pos = [c_x.eval(1.0)[0] + init_pos[0], 
                   c_z.eval(1.0)[0] + init_pos[1],
                   -c_y.eval(1.0)[0] + init_pos[2]]
        path = path + [[pos]]

        self.trajectory = ThickLine(path, 5, "#FF0000")

        return pos 

    # Virtual Function for the services (Interface)
    def register_services(self):
        pass


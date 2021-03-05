from arena import *
from testbed_msgs.msg import CustOdometryStamped
import rospy
import numpy as np

from classes import ROSArenaManager

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../guidance/trjgen')))
from trjgen.class_bz import Bezier
from testbed_msgs.msg import BZCurve

###### HELPERS #####
def posFromStateMsg(msg):
    pos = np.array([msg.p.x, msg.p.y, msg.p.z], dtype=float)
    return pos

def quatFromStateMsg(msg):
    quat = np.array([msg.q.x, 
                     msg.q.y,
                     msg.q.z,
                     msg.q.w], dtype=float)
    return quat


class RingBuffer:
    def __init__(self, size):
        self.data = [None for i in range(size)]

    def append(self, x):
        self.data.pop(0)
        self.data.append(x)

    def get(self):
        return self.data


class DroneObject(object):
    """
    Class for a drone object
    """
    def __init__(self, **kwargs):
        # Extract Arguments from the kwargs
        if 'object_id' in kwargs:
            self.object_id = kwargs.get('object_id')
        else:
            print("Object ID not provided!")

        if 'enable_trace' in kwargs:
            self.enable_trace_ = kwargs.get('enable_trace')
            del kwargs['enable_trace']
        else:
            self.enable_trace_ = False

        if 'enable_trajctory' in kwargs:
            self.enable_trajectory_ = kwargs.get( 'enable_trajectory')
            del kwargs['enable_trajectory']
        else:
            self.enable_trajectory_ = False

        self.state_topic = ""
        self.trj_topic = ""

        self.scene = kwargs.get('arena_srv')

        # Build the base object (the body) 
        self.rarena_manager = None
        self.rarena_manager = ROSArenaManager(self.object_id)
        self.rarena_manager.add_object(
                self.object_id,
                **kwargs
        #        ROSArenaObject(**kwargs)
                )

        self.mission_active = False

        self.trace_ = RingBuffer(25) 

        # Deal with the ROS stuff
        self.register_sources()
        self.register_services()

        self.update_event_handler(self.arena_callback)

        self.old_pos = np.array([0,0,0])


    def __del__(self):
        print("{} Destructor...".format(self.object_id))
        if (self.rarena_manager):
            self.rarena_manager.delete_objects()
        return


    def delete(self):
        self.rarena_manager.delete_objects()
        del self.rarena_manager
        return


    # Register to the source of object's pose data
    def register_sources(self):
        self.state_topic = "/{}/external_codom".format(self.object_id)
        self.trj_topic = "/{}/trajectory".format(self.object_id)
        rospy.Subscriber(self.state_topic,
                CustOdometryStamped,
                self.state_callback)

        rospy.Subscriber(self.trj_topic,
                BZCurve,
                self.traj_callback)

    
    def set_pose(self, ros_pos, ros_rot):
        self.rarena_manager.update_object(
                self.object_id,
                position = ros_pos,
                rotation = ros_rot
                )
        return


    def set_opacity(self, opacity):
        self.rarena_manager.update_object(
                self.object_id,
                opacity = opacity
                )
        

    def update_event_handler(self, clback):
        self.rarena_manager.update_object(
                self.object_id, event_handler = clback)


    # Callback for the pose messages from ROS
    def state_callback(self, state_msg):
        pos = posFromStateMsg(state_msg)
        rot = quatFromStateMsg(state_msg)
        self.set_pose(pos, rot)

        if (np.linalg.norm(pos - self.old_pos) > 0.05):
            self.old_pos = pos
            self.trace_.append(pos)
        
        wake = self.trace_.get()

        for (index, el) in enumerate(wake):
            if el is not None:
                temp_id = self.object_id + 'wake' + str(index)
                self.rarena_manager.add_object(
                        temp_id,
                        #ROSArenaObject(
                            arena_srv = self.scene,
                            position = np.array(el),
                            object_id = temp_id,
                            object_type = 'sphere',
                            color = [255, 255, 255],
                            scale = [0.01, 0.01, 0.01]
                        #    )
                        )
            
            
    def update(self):
        """
        Trigger the update of the arena objects
        associated with this object 
        """
        self.rarena_manager.update()


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
        Nsamples = 6
        self.final_pos = self.draw_curve(msg, Nsamples)
            

    def draw_curve(self, curve, Nsamples):
        init_pos = curve.p0
        c_x = Bezier(curve.coeff_x)
        c_y = Bezier(curve.coeff_y)
        c_z = Bezier(curve.coeff_z)

        dt = 1/Nsamples
        time = 0

        path = []
        while time <= 1.0:
            pos = np.array([c_x.eval(time)[0] + init_pos.x, 
                   c_z.eval(time)[0] + init_pos.z,
                   -c_y.eval(time)[0] - init_pos.y], dtype = float)
            time = time + dt
            path = path + [[pos]]

        pos = np.array([c_x.eval(1.0)[0] + init_pos.x, 
                   c_z.eval(1.0)[0] + init_pos.z,
                   -c_y.eval(1.0)[0] - init_pos.y], dtype = float)
        path = path + [[pos]]


        # Convert the list of numpy array into a 'x0 y0 z0, x1 y1 z1, ..."
        str_path = ''
        for el in path:
            str_path += (np.array2string(el[0]).lstrip('[')).rstrip(']') + ","
        str_path = str_path.rstrip(',')

        self.rarena_manager.add_object(
                self.object_id + 'traj',
                #ROSArenaObject(
                    arena_srv = self.scene,
                    object_id = self.object_id + 'traj',
                    object_type = 'thickline',
                    color = [255, 0, 0],
                    path = str_path,
                    lineWidth = 5
                #    )
                )
        return pos 

    # Virtual Function for the services (Interface)
    def register_services(self):
        pass


from inspect import isfunction
from arena import *
from testbed_msgs.msg import CustOdometryStamped
import rospy
import numpy as np
import math

from classes import ROSArenaManager

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../guidance/trjgen')))
from trjgen.class_bz import Bezier
from testbed_msgs.msg import BZCurve
from testbed_msgs.msg import ControlSetpoint

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
        self.data = list([None for i in range(size)])
        self.tail = 0
        self.head = 0
        self.size = size
        self.curr_size = 0

    def append(self, x):
        self.data[self.tail] = x;
        self.tail = (self.tail + 1) % self.size;
        if (self.tail == self.head):
            self.head = (self.head + 1) % self.size

        self.curr_size = min(self.curr_size + 1, self.size)
        #self.data.pop(0)
        #self.data.append(x)

    def get(self):
        return self.data

    def get_tail(self):
        return self.tail

    def get_head(self):
        return self.head

    def is_full(self):
        return self.curr_size == self.size



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
                )

        self.mission_active = False

        self.trace_ = RingBuffer(80) 

        # Deal with the ROS stuff
        self.register_sources()
        self.register_services()

        self.update_event_handler(self.arena_callback)

        self.old_pos = np.array([0,0,0])

        self.draw_eight(100.0, 10.0, 1.0, 1.0, 0.7)


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
        self.state_topic = "/{}/external_odom".format(self.object_id)
        self.trj_topic = "/{}/trajectory".format(self.object_id)
        self.setp_topic = "/{}/setpoint".format(self.object_id)

        rospy.Subscriber(self.state_topic,
                CustOdometryStamped,
                self.state_callback)

        rospy.Subscriber(self.setp_topic,
                ControlSetpoint,
                self.sp_callback)


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

        head = self.trace_.get_head()
        tail = self.trace_.get_tail()

        if (np.linalg.norm(pos - self.old_pos) > 0.05):
            self.old_pos = pos
            self.trace_.append(pos)

            wake = self.trace_.get()

            new_p = wake[tail]

            if (not self.trace_.is_full()):
                temp_id = self.object_id + 'wake' + str(tail)
                self.rarena_manager.add_object(
                        temp_id,
                        arena_srv = self.scene,
                        position = np.array(new_p),
                        object_id = temp_id,
                        object_type = 'sphere',
                        color = [255, 255, 255],
                        scale = [0.01, 0.01, 0.01]
                        )
            else:
                temp_id = self.object_id + 'wake' + str(head)
                self.rarena_manager.add_object(
                        temp_id,
                        arena_srv = self.scene,
                        position = np.array(new_p),
                        object_id = temp_id,
                        object_type = 'sphere',
                        color = [255, 255, 255],
                        scale = [0.01, 0.01, 0.01]
                        )

        #for (index, el) in enumerate(wake):
        #    if el is not None:
        #        temp_id = self.object_id + 'wake' + str(index)
        #        self.rarena_manager.add_object(
        #                temp_id,
        #                    arena_srv = self.scene,
        #                    position = np.array(el),
        #                    object_id = temp_id,
        #                    object_type = 'sphere',
        #                    color = [255, 255, 255],
        #                    scale = [0.01, 0.01, 0.01]
        #                )
            
            

    # Callback for the pose messages from ROS
    def sp_callback(self, setpoint_msg):
        pos = posFromStateMsg(setpoint_msg)
        #self.set_pose(pos, np.array([0,0,0,1]))

        temp_id = self.object_id + 'setpoint'
        self.rarena_manager.add_object(
                temp_id,
                arena_srv = self.scene,
                position = pos,
                object_id = temp_id,
                object_type = 'sphere',
                color = [100, 50, 255],
                scale = [0.03, 0.03, 0.03]
                )


    def update(self):
        """
        Trigger the update of the arena objects
        associated with this object 
        """
        self.rarena_manager.update()


    def set_color(self, col):
        self.rarena_manager.update_object(self.object_id, color = col)

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
        Nsamples = 8
        #self.final_pos = self.draw_curve(msg, Nsamples)
            

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
                    arena_srv = self.scene,
                    object_id = self.object_id + 'traj',
                    object_type = 'thickline',
                    color = [255, 0, 0],
                    path = str_path,
                    lineWidth = 5
                )
        return pos 

    def draw_eight(self, Nsamples, t2go, rx, ry, rz):
        dt = t2go/Nsamples
        r2z  = 0.1
        omega_x = 4.0 * math.pi / t2go
        omega_y = 2.0 * math.pi / t2go
        omega_z = omega_y

        path = []
        time = 0
        while time <= t2go:
            x = rx * math.sin(omega_x * time)
            y = ry * math.sin(omega_y * time)
            z = rz + r2z * math.sin(omega_z * time)
            pos = np.array([x,z,-y], dtype = float)
            time = time + dt
            path = path + [[pos]]

        # Convert the list of numpy array into a 'x0 y0 z0, x1 y1 z1, ...'
        str_path = ''
        for el in path:
            str_path += (np.array2string(el[0]).lstrip('[')).rstrip(']')+','
        str_path = str_path.rstrip(',')

        self.rarena_manager.add_object(
            self.object_id + 'eight',
            arena_srv = self.scene,
            object_type = 'thickline',
            color = [255, 0, 0],
            path = str_path,
            linewidth = 5
        )

    # Virtual Function for the services (Interface)
    def register_services(self):
        pass


#!/usr/bin/env python

import paho.mqtt.client as mqtt
import rospy
import os
import sys
import time
import json
import math

from arena_playground.json_arena import *

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../guidance/trjgen')))

from commander_interface.srv import GoTo, Land
from guidance.srv import GenImpTrajectoryAuto

from geometry_msgs.msg import PoseStamped
from testbed_msgs.msg import MissionMsg
from trjgen.class_bz import Bezier

import numpy as np

y_offset=0

###### HELPERS #####
def posFromPoseMsg(pose_msg):
    pos = np.array([pose_msg.pose.position.x, 
                pose_msg.pose.position.y + y_offset, 
                pose_msg.pose.position.z])
    return pos

def quatFromPoseMsg(pose_msg):
    quat = np.array([pose_msg.pose.orientation.x, 
                     pose_msg.pose.orientation.y,
                     pose_msg.pose.orientation.z,
                     pose_msg.pose.orientation.w])
    return quat 

# Convert from quaternion to euler angles
def ToEulerAngles(q):
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (q[0] * q[1] + q[2] * q[3]);
    cosr_cosp = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
    roll = math.atan2(sinr_cosp, cosr_cosp);

    # pitch (y-axis rotation)
    sinp = 2.0 * (q[0] * q[2] - q[3] * q[1]);

    pitch = math.asin(sinp);

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2]);
    cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);

    yaw = math.atan2(siny_cosp, cosy_cosp);

    return np.array([roll, pitch, yaw])

def Rx(a):
    # Roll Matrix
    R = np.eye(3)
    R[1][1] = math.cos(a)
    R[1][2] = -math.sin(a)
    R[2][1] = math.sin(a)
    R[2][2] = math.cos(a)
    return R
    
def Ry(a):
    # Pitch Matrix
    R = np.eye(3)
    R[0][0] = math.cos(a)
    R[0][2] = math.sin(a)
    R[2][0] = -math.sin(a)
    R[2][2] = math.cos(a)
    return R

def Rz(a):
    # Yaw Matrix
    R = np.eye(3)
    R[0][0] = math.cos(a)
    R[0][1] = -math.sin(a)
    R[1][0] = math.sin(a)
    R[1][1] = math.cos(a)
    return R



def quat2Rot(q):
    # Quaternion to Rotation matrix
    R = np.zeros((3,3))
    eul = ToEulerAngles(q)

    # Yaw * Pitch * Roll (Robotic convention)
    R = np.matmul(np.matmul(Rz(eul[2]), Ry(eul[1])), Rx(eul[0]))
    return R



################# ROS ARENA CLASS #####################
class RArenaClass(object):
    """
    Bridge class between Ros and Arena.
    """
    def __init__(self, client, scene, name, id, shape="cube", color="#AAAAAA", opacity=False, animate=False,
        source=None, listen=True, on_click_clb=None, visible=True, active=False):

        self.client = client
        self.scene = scene
        self.name = name
        self.id = id
        self.shape = shape
        self.basecolor = color
        self.color = color
        self.opacity = opacity
        self.animate = animate
        self.source = source
        self.listen = listen
        self.on_click = on_click_clb
        self.visible = visible
        self.active=active

        self.plotted = False

        self.node_object_id = self.shape + "_{}".format(self.id)
        self.text_object_id = "text"+ "_{}".format(self.id)

        self.base_topic = "realm/s/" + self.scene + "/"  + self.node_object_id
        self.text_topic = "realm/s/" + self.scene + "/"  + self.text_object_id
        
        self.registerCallbacks()
        self.registerServices()
        self.initArenaObject()

        print("Created Arena Object: {}".format(self.name))

    def registerCallbacks(self):
        pass

    def registerServices(self):
        pass

    def initArenaObject(self):
        pass

    def on_click_input(self, client, userdata, msg):
        pass

    def remove_sub(self, topic, obj_id):
        """ 
        Publish the MQTT message to remove the entity from the scene
        """
        #print("Removing: Full Topic = {} | Object ID = {}\n".format(topic, obj_id))
        # Delete the object
        del_msg = genDelJsonMsg(obj_id)
        self.client.publish(topic, json.dumps(del_msg), retain=True)

    def remove(self):
        # Remove the shape and the text
        self.remove_sub(self.base_topic, self.node_object_id)
        self.remove_sub(self.text_topic, self.text_object_id)

    def __del__(self):
        self.remove()

    def set_color(self, color_string):
        print("[{}]({}) Setting color to {}\n".format(self.__class__.__name__, self.name, color_string))
        self.color = color_string

    def start_animation(self):  
        self.animate = True

    def stop_animation(self):
        self.animate = False
        self.client.publish(self.base_topic + "/animation", "", retain=True)

    def deactivate(self):
        self.active = False

    def activate(self):
        self.active = True

    def isActive(self):
        return self.active



###### NODE ARENA CLASS ####### 
class NodeArenaClass(RArenaClass):
    def __init__(self,
            client, scene, name, id,
            shape="cube", color="#AAAAAA",
            opacity=None, animate=False,
            source=None, listen=True, on_click_clb=None,
            visible=True, active=False,
            pos=[0,1,0], quat=[0,0,0,0], scale=[0.5, 0.5, 0.5],
            text_offset=[0,0.5,0], text_visible=False,
            text_quat=[0,3,0,1], text_scale=[0.08,0.08,0.08]):

        self.pos = pos
        self.quat = quat
        self.scale = scale
        self.text_offset = text_offset
        self.text_visible = text_visible
        self.text_quat = text_quat
        self.text_scale = text_scale

        super(NodeArenaClass, self).__init__(
                client, scene, name, id,
                shape=shape, color=color, opacity=opacity,
                animate=animate, source=source,
                listen=listen, on_click_clb=on_click_clb,
                visible=visible, active=active)


    def initArenaObject(self):
        # Redraw object
        self.remove()
        self.draw()

        if self.opacity:
            mess = genTransJsonMsg(self.shape, self.id, self.opacity)
            self.client.publish(self.base_topic, json.dumps(mess), retain=True)

        # Enable click listener for object (allows it to be clickable)
        if self.listen:
            self.client.subscribe(self.base_topic)
            self.client.message_callback_add(self.base_topic, self.on_click_input)

    def registerCallbacks(self):
        if self.source:
            self.pose_topic = "/" + self.source + "/" + self.name + "/pose" 
            # Subscribe to vehicle state update
            rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback)
            rospy.loginfo("Subscribed to: {}".format(self.pose_topic))


    def on_click_input(self, client, userdata, msg): 
        pass
            

    def draw(self):
        # Fill json mqtt message  
        if (not self.plotted):
            self.plotted = True
            json_message = genJsonMessage(
                    shape = self.shape,
                    identifier = self.id,
                    action = 1,
                    pos = self.pos,
                    quat = self.quat,
                    scale = self.scale,
                    color = self.color,
                    interactive=self.listen) 
        else: 
            json_message = genJsonMessage(
                    shape = self.shape,
                    identifier = self.id,
                    action = 0,
                    pos = self.pos,
                    quat = self.quat,
                    scale = self.scale,
                    color = self.color) 

        # Draw object
        self.client.publish(self.base_topic,
                json.dumps(json_message), retain=True)
    
    def draw_text(self): 
        text_pos = np.array(self.pos) + np.array(self.text_offset)
        json_message = genJsonMessage(
                shape = "text",
                identifier = self.id,
                action = 1,
                pos = text_pos,
                quat = self.text_quat,
                scale = [0.5,0.5,0.5],
                color = self.color,
                text = self.name) 
        # Draw text
        self.client.publish(self.text_topic, 
                json.dumps(json_message),
                retain=True)

    def trigger_animation(self):
        mess = animationJsonMsg(self.shape, self.id);
        self.client.publish(self.base_topic, json.dumps(mess), retain=True)


    def update(self):
        # Update pose
        if self.visible and self.source:
            self.draw()

        # Update text
        if self.text_visible:
            self.draw_text()
        
        # Update animation
        if self.animate:
            self.trigger_animation()
 

    def remove(self):
        super(NodeArenaClass, self).remove()


    def pose_callback(self, pose_msg):
        # Update pose information
        self.pos = posFromPoseMsg(pose_msg)
        self.quat = quatFromPoseMsg(pose_msg)


    def show_text(self):
        self.text_visible=True


    def hide_text(self):
        self.remove_sub(self.text_topic, self.text_object_id)
        self.text_visible=False


    def show_object(self):
        self.visible=True


    def hide_object(self):
        self.remove_sub(self.base_topic, self.node_object_id)
        self.visible=False



###### DRONE ARENA CLASS ########
class DroneArenaClass(NodeArenaClass):
    def __init__(self,
            client,
            scene, name, id,
            shape="sphere", color="#AAAAAA",
            animate=False,
            source=None,
            listen=True,
            on_click_clb=None,
            visible=True,
            active=False,
            pos=[0,1,0], quat=[0,0,0,0], scale=[0.5, 0.5, 0.5],
            opacity=None,
            text_offset=[0,0.5,0], text_visible=False,
            text_quat=[0,3,0,1], text_scale=[0.08,0.08,0.08]):

        super(DroneArenaClass, self).__init__(client,
                scene, name, id,
                shape=shape, color=color, opacity=opacity,
                animate=animate, source=source,
                listen=listen, on_click_clb=on_click_clb,
                visible=visible, pos=pos, quat=quat, scale=scale,
                text_offset=text_offset, text_visible=text_visible,
                text_quat=text_quat, text_scale=text_scale)

        
    def registerServices(self):
        #rospy.wait_for_service("/" + self.name + "/Commander_Node/goTo_srv")
        self.goTo = rospy.ServiceProxy("/" + self.name + "/Commander_Node/goTo_srv", GoTo)
        self.inTer = rospy.ServiceProxy("/" + self.name + "/gen_TrajectoryAuto", GenImpTrajectoryAuto)
        self.land = rospy.ServiceProxy("/" + self.name + "/Commander_Node/land_srv", Land)


    def on_click_input(self, client, userdata, msg):
        """
        Event handler on the DroneClass
        """
        (sanity, click_x, click_y, click_z, user) = parseEventJsonMsg(msg);

        if (not sanity):
            return

        print("{}[{}] | Got click: {} {} {}".format(self.__class__.__name__,
            self.name, click_x, click_y, click_z))

        if (self.on_click is not None):
            self.on_click(self.name) 
        else:
            print("Click callback not specified ({})".format(self.name))
 

    def deactivate(self):
        self.active = False
        super(DroneArenaClass, self).set_color(self.basecolor)
        print("Drone {} Deselected\n".format(self.name))


    def activate(self):
        self.active = True
        super(DroneArenaClass, self).set_color('#00FF00')
        print("Drone {} Selected\n".format(self.name))


    def isActive(self):
        return self.active


###### TARGET ARENA CLASS #######
class TargetArenaClass(NodeArenaClass):
    def __init__(self, client, scene, name, id, shape="cube", color="#AAAAAA", opacity=None, animate=False, source=None, 
        listen=True, on_click_clb=None, visible=True, pos=[0,1,0], quat=[0,0,0,0], scale=[0.5, 0.5, 0.5], 
        text_offset=[0,0.5,0], text_visible=False, text_quat=[0,3,0,1], text_scale=[0.08,0.08,0.08],
        marker_color="#FF0000", marker_scale=[.05,.05,.05], marker_offset=[0,0,0], timeout=2):

        self.marker_color = marker_color
        self.marker_scale = marker_scale
        self.marker_offset = marker_offset
        self.timeout = timeout

        self.last_time=None    # Time since last click event

        super(TargetArenaClass, self).__init__(client, scene, name, id, shape=shape, color=color, opacity=opacity, 
            animate=animate, source=source, listen=listen, on_click_clb=on_click_clb, visible=visible, pos=pos, 
            quat=quat, scale=scale, text_offset=text_offset, text_visible=text_visible, text_quat=text_quat, 
            text_scale=text_scale)


    def initArenaObject(self):
        #   x  y  z  qx qy qz qw sx sy sz col
        #self.marker_message = "sphere_00{}".format(self.id)  + ",{},{},{},0,0,0,0,{},{},{},{},{}"

        self.marker_id = "00{}".format(self.id)
        self.marker_topic = "realm/s/" + self.scene + "/"  + "sphere" + "_" + self.marker_id
        # Redraw object
        self.remove()
        self.draw()
        self.draw_marker(status="off")

        if self.opacity:
            mess = genTransJsonMsg(self.shape, self.id, self.opacity)
            self.client.publish(self.base_topic, json.dumps(mess), retain=True)
            self.client.publish(self.marker_topic, json.dumps(mess), retain=True)

            
        # Enable click listener for object (allows it to be clickable)
        if self.listen:
            self.client.subscribe(self.base_topic)
            self.client.message_callback_add(self.base_topic, self.on_click_input)

            
    def on_click_input(self, client, userdata, msg):
        (sanity, click_x, click_y, click_z, user) = parseEventJsonMsg(msg);

        if (not sanity):
            return

        # Create array with the target position
        print( "Target Position ({}): ".format(self.name) + str(click_x) + "," + str(click_y) + "," + str(click_z) )
        
        self.draw_marker([float(click_x) + self.marker_offset[0],
                          float(click_y) + self.marker_offset[1],
                          float(click_z) + self.marker_offset[2]])

        if self.on_click:
            tg_p = np.array([float(click_x), -float(click_z), float(click_y)])
            self.on_click(tg_p)

        self.last_time = rospy.get_time()


    def draw_marker(self, pos=[0,0,0], status="on"):
        # I suppose that marker is a text object
        if (status == "on"):
            json_message = genJsonMessage(
                    shape = "sphere", identifier = self.marker_id,
                    action = 1,
                    pos = pos,
                    quat = self.quat,
                    scale = self.marker_scale,
                    color = self.marker_color) 
            print(json_message)
        else:
            json_message = genDelJsonMsg("sphere_" + self.marker_id)

        # Draw object
        self.client.publish(self.marker_topic, json.dumps(json_message), retain=True)


    def update(self):
        self.draw()
        # Clean up
        if self.last_time!=None and (rospy.get_time() - self.last_time) > self.timeout:
            self.draw_marker(status="off")
            self.last_time = None


    def remove(self):
        super(TargetArenaClass, self).remove() 



###### EDGE ARENA CLASS #######
class EdgeArenaClass(RArenaClass):
    def __init__(self, client, scene, name, id, color="#AAAAAA", opacity=None, animate=False,
        source=None, listen=True, on_click_clb=None, visible=True, start_node=None, end_node=None, 
        packet_interval=1000, packet_duration=200, packet_scale=[0.05,0.05,0.05], active=True):

        self.packet_interval = packet_interval
        self.packet_duration = packet_duration
        self.packet_scale = packet_scale
        self.active = active


        self.last_time = rospy.get_time()

        if start_node and end_node:
            self.start_node = start_node
            self.end_node = end_node
        else:
            rospy.loginfo("Error: Must supply start and end nodes to edge constructor")
        # Note: Should probably shut down here.
        super(EdgeArenaClass, self).__init__(client, scene, name, id, shape="line", color=color, opacity=opacity,
            animate=animate, source=source, listen=listen, on_click_clb=on_click_clb, visible=visible, active=active)

        print("Created Arena Edge Object")


    def initArenaObject(self):
        self.packet_base_topic = "/topic/" + self.scene + "/"  + "sphere_00{}".format(self.id)
        #                                       x  y  z  x  y  z          col
        self.message = "line_{}".format(self.id) + ",{},{},{},{},{},{},0,0,0,0,{},{}"
        #                                                    x  y  z  qx qy qz qw sx sy sz col
        self.packet_message = "sphere_00{}".format(self.id)  + ",{},{},{},0,0,0,0,{},{},{},{},on"

        # Redraw objects
        self.remove()
        self.draw()

        if self.opacity:
            mess = genTransJsonMsg(self.shape, self.id, self.opacity)
            self.client.publish(self.packet_base_topic, json.dumps(mess), retain=True)
            #self.client.publish(self.packet_base_topic + "/material", 
            #"transparent: true; opacity: {}".format(self.opacity), retain=True)


    def registerCallbacks(self):
        if self.source:
            topic = "/" + self.source
            # Subscribe to vehicle state update
            print("Subscribing to: {}".format(topic))
            rospy.Subscriber(topic, PoseStamped, self.topic_callback)


    def topic_callback(self, msg):
        #If enough time has passed, trigger single packet animationg
        if rospy.get_time() > (self.last_time + self.packet_interval/1000.0) and self.animate: 
            mqtt_topic = "/topic/" + self.scene + "/"  + "sphere_00{}".format(self.id) + "/animation"
            message = "property: position; from: {} {} {}; to: {} {} {}; dur:{};".format(
                self.start_node.pos[0],self.start_node.pos[1],self.start_node.pos[2],
                self.end_node.pos[0],self.end_node.pos[1],self.end_node.pos[2],
                self.packet_duration)

            self.client.publish(mqtt_topic, message, retain=True)
            self.last_time = rospy.get_time() #reset time

        
    def draw(self):
        if self.active:
            color = self.color
            # self.visible = "on"
        else:
            color = "#FFFFFF"
            # self.visible = "off"

        if self.visible:
            vstring = "on"
        else:
            vstring = "off"

        mqtt_string = self.message.format(
            self.start_node.pos[0], self.start_node.pos[1], self.start_node.pos[2],
            self.end_node.pos[0], self.end_node.pos[1], self.end_node.pos[2],
            color, vstring)

        # Draw object
        self.client.publish(self.base_topic, mqtt_string, retain=True)

        # Fill packet message
        packet_string = self.packet_message.format(
            self.start_node.pos[0], self.start_node.pos[1], self.start_node.pos[2], 
            self.packet_scale[0], self.packet_scale[1], self.packet_scale[2], self.color)

        # Update packet start position
        self.client.publish(self.packet_base_topic, packet_string, retain=True)
            

    def update(self):
        # Update pose
        self.draw()


    def remove(self):
        self.client.publish(self.base_topic, "", retain=True)
        self.client.publish(self.base_topic + "/click-listener", "", retain=True)
        self.client.publish(self.base_topic + "/animation", "", retain=True)
        self.client.publish(self.base_topic + "/material", "", retain=True)

        self.client.publish(self.packet_base_topic, "", retain=True)
        self.client.publish(self.packet_base_topic + "/click-listener", "", retain=True)
        self.client.publish(self.packet_base_topic + "/animation", "", retain=True)
        self.client.publish(self.packet_base_topic + "/material", "", retain=True)


    def start_animation(self):  
        self.animate = True

    
    def stop_animation(self):
        self.animate = False
        self.client.publish(self.base_topic + "/animation", "", retain=True)
        self.client.publish(self.packet_base_topic + "/animation", "", retain=True)


    def activate(self):
        self.active = True


    def deactivate(self):
        self.active = False

    
    def show(self):
        self.visible = True


    def hide(self):
        self.visible = False


###### TRAJECTORY ARENA CLASS ########
class TrajectoryArenaClass(RArenaClass):
    def __init__(self, client, scene, name, id, shape="sphere", color="#AAAAAA", opacity=False, 
        animate=False, source=None, listen=False, on_click_clb=None, visible=True, 
        scale=[0.2,0.2,0.2], points=20, duration=2, point_interval=0, tracked_object=None):

        self.scale = scale
        self.points = points
        self.duration = duration
        self.point_interval=point_interval
        self.tracked_object = tracked_object
        self.tracked_pos = [0,0,0]

        self.ros_topic = None
        self.start_time=None
        self.point_count=0

        super(TrajectoryArenaClass, self).__init__(client, scene, name, id, shape=shape, color=color, opacity=opacity,
            animate=animate, source=source, listen=listen, on_click_clb=on_click_clb, visible=visible)


    def initArenaObject(self):
        self.trj_id = "{}"+"0{}".format(self.id) 
        self.obj_id = self.shape + "_" + self.trj_id
        self.topic_frame = "realm/s/" + self.scene + "/" + self.shape + "_{}" + "0{}".format(self.id)
        self.registerCallbacks()
        self.remove_trajectory(100)

    def registerCallbacks(self):
        if self.source:
            self.ros_topic = "/" + self.source
            # Subscribe to vehicle state update
            rospy.Subscriber(self.ros_topic, MissionMsg, self.traj_callback)
            rospy.loginfo("Subscribed to: {}".format(self.ros_topic))

        if self.tracked_object:
            self.tracked_topic = "/" + self.tracked_object
            rospy.Subscriber(self.tracked_topic, PoseStamped, self.tracked_callback)
            rospy.loginfo("Subscribed to: {}".format(self.tracked_topic))


    def tracked_callback(self, pose_msg):
        self.tracked_pos = posFromPoseMsg(pose_msg)


    def traj_callback(self, msg):
        self.remove_trajectory()
        self.start_time = rospy.get_time()
        self.duration = 2
        self.point_count = 0
        time_offset = 0.0
        init_pos = self.tracked_pos
        
        #rospy.loginfo("Received trajectory! ({0}): Time = {1:f}".format(self.name, self.start_time))
        #rospy.loginfo("Starting position ({}): {}".format(self.name, init_pos))
        #print(msg)

        for curve in msg.mission:
            #rospy.loginfo("Got Curve ({})".format(self.name))
            # normed_interval = float(self.point_interval) / curve.duration # Calculate scaled interval
            # normed_offset = float(time_offset) / curve.duration
            normed_offset, points, final_pos = self.draw_curve(curve, .1, self.point_count, 
                                                            0, init_pos)
            # time_offset = normed_offset * curve.duration # Re-scale to real time
            init_pos = final_pos
            self.point_count = self.point_count + points 
            self.duration = self.duration + curve.duration # Add to the total duration of the trajectory

            #rospy.loginfo("Point Count = {}".format(self.point_count))
            
    def draw_curve(self, curve, interval, idx_start=0, time_offset=0, init_pos=0):
        
        c_x = Bezier(curve.coeff_x)
        c_y = Bezier(curve.coeff_y)
        c_z = Bezier(curve.coeff_z)
        time = float(time_offset)
        idx = idx_start
        #rospy.loginfo("Drawing curve at time: {}".format(time))

        # Draw curve with points at specified interval and indices starting from idx_start
        while time <= 1.0:  # Curve assumes a time length of 1.0s
            # Remeber to transform coordinates
            pos = [c_x.eval(time)[0] + init_pos[0], 
                   c_z.eval(time)[0] + init_pos[1],
                   -c_y.eval(time)[0] + init_pos[2]]
            topic = self.topic_frame.format(idx)

            json_message = genJsonMessage(
                    shape = self.shape,
                    identifier = self.trj_id.format(idx),
                    action = 1,
                    pos = pos, scale = self.scale, color=self.color)  
            self.client.publish(topic, json.dumps(json_message), retain=False)

            if self.opacity:
                mess = genTransJsonMsg(self.shape, self.trj_id.format(idx), self.opacity)
                self.client.publish(topic, json.dumps(mess), retain=False)
                
            idx = idx + 1
            time = time + interval

        offset = time - 1.0
        points = idx - idx_start
        final_pos = [c_x.eval(1.0)[0] + init_pos[0], 
                   c_z.eval(1.0)[0] + init_pos[1],
                   -c_y.eval(1.0)[0] + init_pos[2]]

        return offset, points, final_pos

    def sweep_trajectory(self): 
        if self.start_time!=None and self.point_count > 0:
            i = 0
            while (self.point_count > 0 and ((rospy.get_time() - self.start_time) > self.duration * (i/float(self.point_count)))):
                topic = self.topic_frame.format(i)
                json_message = genDelJsonMsg(self.obj_id.format(i))
                self.client.publish(topic, json.dumps(json_message), retain=False)
                i = i + 1


    def remove_trajectory(self, max=None):
        #rospy.loginfo("Removing Trajectory ({})".format(self.name))
        if max:
            points = max
        else:
            points = self.point_count
        for i in range(points):
            topic = self.topic_frame.format(i)
            json_message = genDelJsonMsg(self.obj_id.format(i))
            self.client.publish(topic, json.dumps(json_message), retain=False)

        # evaluate trajectory and update drawing
        self.start_time = None


    def update(self):
        self.sweep_trajectory()
        if self.start_time!=None and (rospy.get_time() - self.start_time) > self.duration:
            #print("Got to Remove")
            self.remove_trajectory()
            self.start_time = None


    def remove(self):
        self.remove_trajectory()


###### TRACKER ARENA CLASS ########
class TrackerArenaClass:
    def __init__(self, client, scene, name, source, active=True, rate=10):
        self.client = client
        self.scene = scene
        self.name = name
        self.camera_id = "camera_" + self.name + "_" + self.name
        self.source = source
        self.source_pos = np.array([0,0,0], dtype=float)
        self.source_quat = np.array([0,0,0,0], dtype=float)
        self.camera_pos = np.array([0,0,0], dtype=float)
        self.camera_quat = np.array([0,0,0,0], dtype=float)
        self.active = active
        self.rate = rate
        self.last_time = rospy.get_time()
        self.diff = np.array([0,0,0], dtype=float)
        self.got_initial_pos = False

        self.source_topic = "/" + self.source + "/" + self.name + "/pose"
        self.camera_topic = "realm/s/" + self.scene + "/" + self.camera_id

        self.client.subscribe(self.camera_topic)
        self.client.message_callback_add(self.camera_topic, self.camera_callback)

        self.registerCallbacks()


    def registerCallbacks(self):
        # Subscribe to vehicle state update
        rospy.Subscriber(self.source_topic, PoseStamped, self.source_callback)
        rospy.loginfo("Subscribed to: {}".format(self.source))


    def source_callback(self, pose_msg):
        # Update the position of the Motive Tracker 
        self.source_pos = posFromPoseMsg(pose_msg)
        self.source_quat = quatFromPoseMsg(pose_msg)
        #print("vrpn camera update: pos = {}".format(self.source_pos))


    def camera_callback(self, client, userdata, msg):
        #print( "arena camera update ({}): {}".format(self.name, msg) )
        # I can receive either RIG message either CAMERA messages.
        (flag, pos, quat) = parseCameraJsonMsg(msg)
        
        self.camera_quat = quat 
        #print("[{}] Camera position = {} \n".format(self.name, pos))
        #if (flag and (not self.got_initial_pos) and pos.any()):
        if (flag and pos.any()):
            #print("[{}] Initial Offset = {}\n".format(self.name, pos))
            self.camera_pos = pos 
            self.got_initial_pos = True
        else:
            #print("Not a camera message!")
            return
            pass


    def update(self):
        dt_last = (rospy.get_time() - self.last_time) 
        if (self.active and dt_last > (1.0/self.rate)):
            self.publish_correction()
            pass


    def publish_correction(self):
        # Compute the difference between the real position and the
        # initial position in the Aframe.
        pos_diff = self.source_pos

        if (self.name == "tablet"):
            quat_diff = [0,0,0,1] 
            pos_diff = np.array([0, -1.0, 0]) 
            #quat_diff = self.source_quat
            #pos_diff = pos_diff - self.camera_pos

        if (self.name == "phone"):    
            quat_diff = [0,0,0,1] 
            pos_diff = pos_diff - self.camera_pos

        json_message = genCameraJsonMsg(
                self.camera_id,
                pos_diff,
                quat_diff)

        if (self.got_initial_pos):
            self.client.publish("realm/s/" + self.scene,
                    json.dumps(json_message),
                    retain=False)

        #print("Published Diff: {},{}".format(pos_diff, quat_diff))
        #print("Camera Position = {}".format(self.camera_pos))
        #print("Source Position = {}".format(self.source_pos))


    def compute_quat_diff(self, quat_camera, quat_source):
        q1 = np.array([-quat_camera[0], -quat_camera[1], -quat_camera[2], quat_camera[3]]) # get conjugate
        q2 = np.array(quat_source)

        scalar = q1[3]*q2[3] - np.dot(q1[0:3], q2[0:3])
        vector = q1[3]*q2[0:3] + q2[3]*q1[0:3] + np.cross(q1[0:3],q2[0:3])

        qdiff = np.array([vector[0], vector[1], vector[2], scalar])

        return qdiff/np.linalg.norm(qdiff)


    def remove_sub(self, topic, obj_id):
        """ 
        Publish the MQTT message to remove the entity from the scene
        """
        # Delete the object
        del_msg = genDelJsonMsg(obj_id)
        self.client.publish(topic, json.dumps(del_msg), retain=True)

    def remove(self):
        pass
        self.remove_sub(self.camera_topic, self.camera_id)


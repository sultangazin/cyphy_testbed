#!/usr/bin/env python

import paho.mqtt.client as mqtt
import rospy
import os
import sys
import time
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
        self.color = color
        self.opacity = opacity
        self.animate = animate
        self.source = source
        self.listen = listen
        self.on_click = on_click_clb
        self.visible = visible
        self.active=active

        self.base_topic = "/topic/" + self.scene + "/"  + self.shape + "_{}".format(self.id) 
        self.text_topic = "/topic/" + self.scene + "/"  + "text"+ "_{}".format(self.id) 

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

    def remove(self):
        self.client.publish(self.base_topic, "", retain=True)

    def __del__(self):
        self.remove()

    def set_color(self, color_string):
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
    def __init__(self, client, scene, name, id, shape="cube", color="#AAAAAA", opacity=None, animate=False,
        source=None, listen=True, on_click_clb=None, visible=True, active=False, pos=[0,1,0], quat=[0,0,0,0], 
        scale=[0.5, 0.5, 0.5], text_offset=[0,0.5,0], text_visible=False, text_quat=[0,3,0,1], 
        text_scale=[0.08,0.08,0.08]):

        self.pos = pos
        self.quat = quat
        self.scale = scale
        self.text_offset = text_offset
        self.text_visible = text_visible
        self.text_quat = text_quat
        self.text_scale = text_scale

        super(NodeArenaClass, self).__init__(client, scene, name, id, shape=shape, color=color, opacity=opacity,
        animate=animate, source=source, listen=listen, on_click_clb=on_click_clb, visible=visible, active=active)


    def initArenaObject(self):
        #                                                      x  y  z  qx qy qz qw sx sy sz col
        self.message = self.shape + "_{}".format(self.id)  + ",{},{},{},{},{},{},{},{},{},{},{},on"
        #                           x  y  z  qx qy qz qw sx sy sz txt
        self.text_message = "text" + "_{}".format(self.id) + ",{},{},{},{},{},{},{},{},{},{},{},on"

        # Redraw object
        self.remove()
        self.draw()

        if self.opacity:
            self.client.publish(self.base_topic + "/material", "transparent: true; opacity: {}".format(self.opacity), retain=True)

        # Enable click listener for object (allows it to be clickable)
        if self.listen:
            self.client.publish(self.base_topic + "/click-listener", "enable", retain=True)
            self.client.subscribe(self.base_topic + "/mousedown")
            self.client.message_callback_add(self.base_topic + "/mousedown", self.on_click_input)


    def registerCallbacks(self):
        if self.source:
            self.pose_topic = "/" + self.source + "/" + self.name + "/pose" 
            # Subscribe to vehicle state update
            rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback)
            rospy.loginfo("Subscribed to: {}".format(self.pose_topic))


    def on_click_input(self, client, userdata, msg):
        click_x, click_y, click_z, user = msg.payload.split(',')

        if str(msg.topic).find("mousedown") != -1:
            print("Got click ({}): {} {} {}".format(self.name, click_x, click_y, click_z))
            pass
            

    def draw(self):
        # Fill mqtt message
        message = self.message.format(
            self.pos[0], self.pos[1], self.pos[2],
            self.quat[0], self.quat[1], self.quat[2], self.quat[3],
            self.scale[0], self.scale[1], self.scale[2], 
            self.color)

        # Draw object
        self.client.publish(self.base_topic, message, retain=True)
    
    def draw_text(self):
        message = self.text_message.format(
            self.pos[0] + self.text_offset[0], self.pos[1] + self.text_offset[1], self.pos[2] + self.text_offset[2],
            self.text_quat[0], self.text_quat[1], self.text_quat[2], self.text_quat[3],
            self.text_scale[0], self.text_scale[1], self.text_scale[2], 
            self.name)
        
        self.client.publish(self.text_topic, message, retain=True)

    def trigger_animation(self):
        tg_scene_string = self.base_topic + "/animation"
        cmd_string = "property: color; to: #FF0000; loop:true; dur: 500;"
        self.client.publish(tg_scene_string, cmd_string, retain=True)


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
        self.client.publish(self.base_topic, "", retain=True)
        self.client.publish(self.base_topic + "/click-listener", "", retain=True)
        self.client.publish(self.base_topic + "/animation", "", retain=True)
        self.client.publish(self.base_topic + "/material", "", retain=True)

        self.client.publish(self.text_topic, "", retain=True)
        self.client.publish(self.text_topic + "/click-listener", "", retain=True)
        self.client.publish(self.text_topic + "/animation", "", retain=True)
        self.client.publish(self.text_topic + "/material", "", retain=True)


    def pose_callback(self, pose_msg):
        # Update pose information
        self.pos = posFromPoseMsg(pose_msg)
        self.quat = quatFromPoseMsg(pose_msg)


    def show_text(self):
        self.text_visible=True


    def hide_text(self):
        self.client.publish(self.text_topic, "", retain=True)
        self.text_visible=False


    def show_object(self):
        self.visible=True


    def hide_object(self):
        self.client.publish(self.text_topic, "", retain=True)
        self.visible=False



###### DRONE ARENA CLASS ########
class DroneArenaClass(NodeArenaClass):
    def __init__(self, client, scene, name, id, shape="sphere", color="#AAAAAA", animate=False, source=None, 
        listen=True, on_click_clb=None, visible=True, active=False, pos=[0,1,0], quat=[0,0,0,0], scale=[0.5, 0.5, 0.5], 
        opacity=None, text_offset=[0,0.5,0], text_visible=False, text_quat=[0,3,0,1], text_scale=[0.08,0.08,0.08]):

        super(DroneArenaClass, self).__init__(client, scene, name, id, shape=shape, color=color, opacity=opacity, 
            animate=animate, source=source, listen=listen, on_click_clb=on_click_clb, visible=visible, pos=pos, 
            quat=quat, scale=scale, text_offset=text_offset, text_visible=text_visible, text_quat=text_quat, 
            text_scale=text_scale)

        
    def registerServices(self):
        #rospy.wait_for_service("/" + self.name + "/Commander_Node/goTo_srv")
        self.goTo = rospy.ServiceProxy("/" + self.name + "/Commander_Node/goTo_srv", GoTo)
        self.inTer = rospy.ServiceProxy("/" + self.name + "/gen_TrajectoryAuto", GenImpTrajectoryAuto)
        self.land = rospy.ServiceProxy("/" + self.name + "/Commander_Node/land_srv", Land)


    def on_click_input(self, client, userdata, msg):
        #click_x, click_y, click_z, user = msg.payload.split(',')
        if (self.on_click is not None):
            self.on_click(self.name) 
        else:
            print("Click callback not specified ({})".format(self.name))
 

    def deactivate(self):
        self.active = False
        super(DroneArenaClass, self).set_color('#000022')
        self.color = "#000022"
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
        #                                                      x  y  z  qx qy qz qw sx sy sz col
        self.message = self.shape + "_{}".format(self.id)  + ",{},{},{},{},{},{},{},{},{},{},{},on"
        self.text_message = "text" + "_{}".format(self.id) + ",{},{},{},{},{},{},{},{},{},{},{},on"
        self.marker_message = "sphere_00{}".format(self.id)  + ",{},{},{},0,0,0,0,{},{},{},{},{}"

        self.marker_topic = "/topic/" + self.scene + "/"  + "sphere_00{}".format(self.id)

        # Redraw object
        self.remove()
        self.draw()
        self.draw_marker(status="off")

        if self.opacity:
            self.client.publish(self.base_topic + "/material", 
                "transparent: true; opacity: {}".format(self.opacity), retain=True)

            self.client.publish(self.marker_topic + "/material", 
                "transparent: true; opacity: {}".format(self.opacity), retain=True)

        # Enable click listener for object (allows it to be clickable)
        if self.listen:
            self.client.publish(self.base_topic + "/click-listener", "enable", retain=True)
            self.client.subscribe(self.base_topic + "/mousedown")
            self.client.message_callback_add(self.base_topic + "/mousedown", self.on_click_input)

            
    def on_click_input(self, client, userdata, msg):
        click_x, click_y, click_z, user = msg.payload.split(',')
        # Create array with the target position
        print( "Target Position ({}): ".format(self.name) + str(click_x) + "," + str(click_y) + "," + str(click_z) )
        
        self.draw_marker([float(click_x) + self.marker_offset[0],
                          float(click_y) + self.marker_offset[1],
                          float(click_z) + self.marker_offset[2]])

        if self.on_click:
            tg_p = np.array([float(click_x), -float(click_z), float(click_y)])
            self.on_click(tg_p)

        self.last_time = rospy.get_time()


    def draw_marker(self, pos=[0,0,0],status="on"):
        mqtt_string = self.marker_message.format(pos[0], pos[1], pos[2], 
            self.marker_scale[0], self.marker_scale[1], self.marker_scale[2], self.marker_color, status)

        # Draw object
        self.client.publish(self.marker_topic, mqtt_string, retain=True)


    def update(self):
        self.draw()
        # Clean up
        if self.last_time!=None and (rospy.get_time() - self.last_time) > self.timeout:
            self.draw_marker(status="off")
            self.last_time = None


    def remove(self):
        self.client.publish(self.base_topic, "", retain=True)
        self.client.publish(self.base_topic + "/click-listener", "", retain=True)
        self.client.publish(self.base_topic + "/animation", "", retain=True)
        self.client.publish(self.base_topic + "/material", "", retain=True)

        self.client.publish(self.text_topic, "", retain=True)
        self.client.publish(self.text_topic + "/click-listener", "", retain=True)
        self.client.publish(self.text_topic + "/animation", "", retain=True)
        self.client.publish(self.text_topic + "/material", "", retain=True)

        self.client.publish(self.marker_topic, "", retain=True)
        self.client.publish(self.marker_topic + "/click-listener", "", retain=True)
        self.client.publish(self.marker_topic + "/animation", "", retain=True)
        self.client.publish(self.marker_topic + "/material", "", retain=True)




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
            self.client.publish(self.packet_base_topic + "/material", 
            "transparent: true; opacity: {}".format(self.opacity), retain=True)


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
        self.topic_frame = "/topic/" + self.scene + "/" + self.shape + "_{}" + "0{}".format(self.id)
        self.message_frame = self.shape + "_{}" + "0{}".format(self.id) + ",{},{},{}" + ",0,0,0,0,{},{},{}".format(self.scale[0],
            self.scale[1],self.scale[2]) + ",{},on".format(self.color)

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
        print(msg)

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
            message = self.message_frame.format(idx, pos[0], pos[1], pos[2])
            topic = self.topic_frame.format(idx)
            self.client.publish(topic, message, retain=True)
            if self.opacity:
                self.client.publish(topic + "/material", 
                "transparent: true; opacity: {}".format(self.opacity), retain=True)
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
            while (rospy.get_time() - self.start_time) > self.duration * (i/float(self.point_count)):
                topic = self.topic_frame.format(i)
                self.client.publish(topic, "", retain=True)
                i = i + 1


    def remove_trajectory(self, max=None):
        #rospy.loginfo("Removing Trajectory ({})".format(self.name))
        if max:
            points = max
        else:
            points = self.point_count
        for i in range(points):
            message = ""
            topic = self.topic_frame.format(i)
            self.client.publish(topic, message, retain=True)
            self.client.publish(topic + "/material", "", retain=True)
        # evaluate trajectory and update drawing
        self.start_time = None


    def update(self):
        #self.sweep_trajectory()
        if self.start_time!=None and (rospy.get_time() - self.start_time) > self.duration:
            #print("Got to Remove")
            self.remove_trajectory()
            self.start_time = None


    def remove(self):
        self.remove_trajectory()


###### TRAJECTORY ARENA CLASS ########
class TrackerArenaClass:
    def __init__(self, client, scene, name, source, active=True, rate=10):

        self.client = client
        self.scene = scene
        self.name = name
        self.source = source
        self.source_pos = np.array([0,0,0], dtype=float)
        self.source_quat = np.array([0,0,0,0], dtype=float)
        self.camera_pos = np.array([0,0,0], dtype=float)
        self.camera_quat = np.array([0,0,0,0], dtype=float)
        self.active = active
        self.rate = rate
        self.last_time = rospy.get_time()
        self.diff = np.array([0,0,0], dtype=float)

        self.source_topic = "/" + self.source + "/" + self.name + "/pose"

        self.camera_topic = "/topic/vio/camera_" + self.name + "_" + self.name

        self.rig_message_topic= "/topic/" + self.scene + "/camera_" + self.name + "_" + self.name + "/rig"
        self.rig_message_frame = "{}, {}, {}, {}, {}, {}, {}"

        #self.client.publish(self.camera_topic + "/click-listener", "enable", retain=True)
        self.client.subscribe(self.camera_topic)
        self.client.message_callback_add(self.camera_topic, self.camera_callback)

        # self.client.subscribe(self.rig_topic)
        # self.client.message_callback_add(self.rig_topic, self.rig_callback)

        self.registerCallbacks()


    def registerCallbacks(self):
        # Subscribe to vehicle state update
        rospy.Subscriber(self.source_topic, PoseStamped, self.source_callback)
        rospy.loginfo("Subscribed to: {}".format(self.source))


    def source_callback(self, pose_msg):
        # Set new pose
        self.source_pos = posFromPoseMsg(pose_msg)
        self.source_quat = quatFromPoseMsg(pose_msg)


    def camera_callback(self, client, userdata, msg):
        #print( "Arena camera update ({}): {}".format(self.name, msg.payload) )
        data = msg.payload.split(',')
        
        self.camera_pos[0] = float(data[1])
        self.camera_pos[1] = float(data[2])
        self.camera_pos[2] = float(data[3])

        self.camera_quat[0] = float(data[4])
        self.camera_quat[1] = float(data[5])
        self.camera_quat[2] = float(data[6])
        self.camera_quat[3] = float(data[7])

        #print(self.camera_pos, self.camera_quat)


    def rig_callback(self, client, userdata, msg):
        #print( "Arena rig update ({}): {}".format(self.name, msg.payload) )
        data = msg.payload.split(',')

        self.rig_pos[0] = float(data[1])
        self.rig_pos[1] = float(data[2])
        self.rig_pos[2] = float(data[3])


    def update(self):
        if self.active: #and (rospy.get_time() - self.last_time) > (1.0/self.rate):
            # self.publish_correction()
            pass


    def publish_correction(self):
        pos_diff = self.source_pos - self.camera_pos
        quat_diff = [0,0,0,1] #self.compute_quat_diff(self.camera_quat, self.source_quat)
        rig_message = self.rig_message_frame.format(pos_diff[0], pos_diff[1], pos_diff[2],
                                        quat_diff[0], quat_diff[1], quat_diff[2], quat_diff[3])
        self.client.publish(self.rig_message_topic, rig_message, retain=True)

        print("Published Diff: {},{}".format(pos_diff, quat_diff))

    def compute_quat_diff(self, quat_camera, quat_source):
        q1 = np.array([-quat_camera[0], -quat_camera[1], -quat_camera[2], quat_camera[3]]) # get conjugate
        q2 = np.array(quat_source)

        scalar = q1[3]*q2[3] - np.dot(q1[0:3], q2[0:3])
        vector = q1[3]*q2[0:3] + q2[3]*q1[0:3] + np.cross(q1[0:3],q2[0:3])

        qdiff = np.array([vector[0], vector[1], vector[2], scalar])

        return qdiff/np.linalg.norm(qdiff)


    def remove(self):
        self.client.publish(self.camera_topic + "/click-listener", "", retain=True)

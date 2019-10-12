#!/usr/bin/env python

import paho.mqtt.client as mqtt
import rospy
import os
import sys
import time
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), './')))

from commander_interface.srv import GoTo, Land
from guidance.srv import GenImpTrajectoryAuto

from geometry_msgs.msg import PoseStamped

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

        self.base_topic = self.scene + "/"  + self.shape + "_{}".format(self.id) 
        self.text_topic = self.scene + "/"  + "text"+ "_{}".format(self.id) 

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
        if self.visible:
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
        self.inTer = rospy.ServiceProxy("/" + self.name + "/gen_ImpTrajectoryAuto", GenImpTrajectoryAuto)
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

        self.marker_topic = self.scene + "/"  + "sphere_00{}".format(self.id)

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
        self.packet_base_topic = self.scene + "/"  + "sphere_00{}".format(self.id)
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
            mqtt_topic = self.scene + "/"  + "sphere_00{}".format(self.id) + "/animation"
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
        animate=False, source=None, listen=True, on_click_clb=None, visible=True, 
        scale=[0.2,0.2,0.2], points=20, duration=2, draw_interval=0):

        self.scale = scale
        self.points = points
        self.duration = duration
        self.draw_interval=draw_interval

        self.ros_topic = None
        self.start_time=None

        super(TrajectoryArenaClass, self).__init__(client, scene, name, id, shape=shape, color=color, opacity=opacity,
            animate=animate, source=source, listen=listen, on_click_clb=on_click_clb, visible=visible)


    def initArenaObject(self):
        self.topic_frame = self.scene + "/" + self.shape + "_{}" + "0{}".format(self.id)
        self.message_frame = self.shape + "_{}" + "0{}".format(self.id) + ",{},{},{}" + ",0,0,0,1,{},{},{}".format(self.scale[0],
            self.scale[1],self.scale[2]) + ",{},on".format(self.color)

        # Redraw object
        self.remove_trajectory()

        # TEMPORARY: Enable click listener for object (allows it to be clickable)
        if self.listen:
            self.client.message_callback_add(self.scene + "/cube_6/mousedown", self.on_click_input)


    def registerCallbacks(self):
        if self.source:
            self.ros_topic = "/" + self.source
            # Subscribe to vehicle state update
            rospy.Subscriber(self.ros_topic, PoseStamped, self.traj_callback)
            rospy.loginfo("Subscribed to: {}".format(self.traj_callback))


    def on_click_input(self, client, userdata, msg):
        self.draw_trajectory(msg)


    def traj_callback(self, traj_msg):
        print("Received trajectory!: {}".format(traj_msg))


    def draw_trajectory(self, msg):
        self.start_time = rospy.get_time()
        click_x, click_y, click_z, user = msg.payload.split(',')
        for i in range(self.points):
            pos = [float(click_x),0,float(click_z) + int(i)/5.] # To be changed
            message = self.message_frame.format(i,pos[0],pos[1],pos[2])
            topic = self.topic_frame.format(i)
            self.client.publish(topic, message, retain=True)
            if self.opacity:
                self.client.publish(topic + "/material", 
                "transparent: true; opacity: {}".format(self.opacity), retain=True)
            rospy.sleep(self.draw_interval)


    def sweep_trajectory(self): 
        if self.start_time:
            i = 0
            while (rospy.get_time() - self.start_time) > self.duration * (i/float(self.points)):
                topic = self.topic_frame.format(i)
                self.client.publish(topic, "", retain=True)
                i = i + 1


    def remove_trajectory(self):
        for i in range(self.points):
            message = ""
            topic = self.topic_frame.format(i)
            self.client.publish(topic, message, retain=True)
            self.client.publish(topic + "/material", "", retain=True)
        # evaluate trajectory and update drawing


    def update(self):
        self.sweep_trajectory()
        if self.start_time!=None and (rospy.get_time() - self.start_time) > self.duration:
            print("Got to Remove")
            self.remove_trajectory()
            self.start_time=None


    def remove(self):
        self.remove_trajectory()

    
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

y_offset=-0.21

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
    def __init__(self, client, scene, name, id, shape="cube", color="#AAAAAA", animate=False, source=None, 
        listen=True, on_click_clb=None, visible=True):

        self.client = client
        self.scene = scene
        self.name = name
        self.id = id
        self.shape = shape
        self.color = color
        self.animate = animate
        self.source = source
        self.listen = listen
        self.on_click = on_click_clb
        self.visible = visible

        self.base_topic = self.scene + "/"  + self.shape + "_{}".format(self.id) 
        self.text = name
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



###### NODE ARENA CLASS ####### 
class NodeArenaClass(RArenaClass):
    def __init__(self, client, scene, name, id, shape="cube", color="#AAAAAA", animate=False, source=None, 
        listen=True, on_click_clb=None, visible=True, pos=[0,1,0], quat=[0,0,0,0], scale=[0.5, 0.5, 0.5], 
        opacity=None, text_offset=[0,0.5,0], text_visible=False, text_quat=[0,3,0,1], text_scale=[0.08,0.08,0.08]):

        self.pos = pos
        self.quat = quat
        self.scale = scale
        self.opacity = opacity
        self.text_offset = text_offset
        self.text_visible = text_visible
        self.text_quat = text_quat
        self.text_scale = text_scale

        super(NodeArenaClass, self).__init__(client, scene, name, id, shape=shape, color=color, animate=animate, source=source, listen=listen,
        on_click_clb=on_click_clb, visible=visible)


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
            self.text)
        
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
        self.text_visible=True


    def hide_object(self):
        self.client.publish(self.text_topic, "", retain=True)
        self.text_visible=False



###### DRONE ARENA CLASS ########
class DroneArenaClass(NodeArenaClass):
    def __init__(self, client, scene, name, id, shape="sphere", color="#AAAAAA", animate=False,
        pos=[0,0.05,0], scale=[0.5, 0.5, 0.5], source=None, on_click_clb=None, opacity=False):

        self.Active = False

        super(DroneArenaClass, self).__init__(client, scene, name, id, shape=shape, pos=pos, 
            scale=scale, color=color, animate=animate, source=source, opacity=opacity)

        
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
            print("Click allback not specified ({})".format(self.name))
 

    def deactivate(self):
        self.Active = False
        super(DroneArenaClass, self).set_color('#000022')
        self.color = "#000022"
        print("Drone {} Deselected\n".format(self.name))


    def activate(self):
        self.Active = True
        super(DroneArenaClass, self).set_color('#00FF00')
        print("Drone {} Selected\n".format(self.name))


    def isActive(self):
        return self.Active


###### TARGET ARENA CLASS #######
class TargetArenaClass(NodeArenaClass):
    def __init__(self, client, scene, name, id, shape="cube", color="#AAAAAA", animate=False, opacity=False,
        pos=[0,1,0], quat=[0,0,0,0], scale=[0.5, 0.5, 0.5], source=None, on_click_clb=None):

        self.on_click = on_click_clb
        self.duration = 2
        self.marker_color = "#FF0000"
        self.marker_scale = [.05,.05,.05]

        self.marker_base_topic = scene + "/"  + "sphere_{}00".format(id)
        #                                                    x  y  z  qx qy qz qw sx sy sz col
        self.marker_message = "sphere_{}00".format(id)  + ",{},{},{},0,0,0,0,{},{},{},{},{}"

        self.last_time=None

        super(TargetArenaClass, self).__init__(client, scene, name, id, shape=shape, color=color, animate=animate, 
        pos=pos, quat=quat, scale=scale, source=source, opacity=opacity)

        self.draw_marker(status="off")

        if self.opacity:
            self.client.publish(self.marker_base_topic + "/material", 
                "transparent: true; opacity: {}".format(self.opacity), retain=True)

        #self.registerServices()

        print("Created Arena Target Object")

    # def registerServices(self):
    #     pass
            

    def on_click_input(self, client, userdata, msg):
        click_x, click_y, click_z, user = msg.payload.split(',')

        # Create array with the target position

        print( "Target Position ({}): ".format(self.name) + str(click_x) + "," + str(click_y) + "," + str(click_z) )
        
        if (self.id == 6):
            self.draw_marker([click_x, str(float(click_y) + 0.7), click_z])
        else:
            self.draw_marker([click_x, click_y, click_z])

        if self.on_click:
            tg_p = np.array([float(click_x), -float(click_z), float(click_y)])
            self.on_click(tg_p)

        self.last_time = rospy.get_time()


    def draw_marker(self, pos=[0,0,0],status="on"):
        mqtt_string = self.marker_message.format(
            pos[0], pos[1], (pos[2]),
            self.marker_scale[0], self.marker_scale[1], self.marker_scale[2],
            self.marker_color,status)

        # Draw object
        self.client.publish(self.marker_base_topic, mqtt_string, retain=True)


    def update(self):
        # Update pose
        self.draw()
        # Clean up
        if self.last_time!=None and (rospy.get_time() - self.last_time) > self.duration:
            self.client.publish(self.marker_base_topic, self.marker_message.format(
            0,0,0,0,0,0,0,"off"), retain=True)
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

        self.client.publish(self.marker_base_topic, "", retain=True)
        self.client.publish(self.marker_base_topic + "/click-listener", "", retain=True)
        self.client.publish(self.marker_base_topic + "/animation", "", retain=True)
        self.client.publish(self.marker_base_topic + "/material", "", retain=True)




###### EDGE ARENA CLASS #######
class EdgeArenaClass(RArenaClass):
    def __init__(self, client, scene, name, id, start_node, end_node, color="#AAAAAA", 
        animate=False, ros_topic=None, interval=1000, packet_duration=200, active=True):

        self.last_time = rospy.get_time()
        self.visible = "on"
        self.packet_scale = [0.05,0.05,0.05]
        self.packet_duration = packet_duration
        self.ros_topic = ros_topic
        self.interval = interval
        self.active = active
        self.transparent=True
        self.opacity=0.5

        if start_node and end_node:
            self.start_node = start_node
            self.end_node = end_node
        else:
            rospy.loginfo("Error: Must supply start and end nodes to edge constructor")
        # Note: Should probably shut down here.
        super(EdgeArenaClass, self).__init__(client, scene, name, id, shape="line", color=color, animate=animate)

        print("Created Arena Edge Object")


    def initArenaObject(self):
        self.packet_base_topic = self.scene + "/"  + "sphere_{}00".format(self.id)
        #                                       x  y  z  x  y  z          col
        self.message = "line_{}".format(self.id) + ",{},{},{},{},{},{},0,0,0,0,{},{}"
        #                                                    x  y  z  qx qy qz qw sx sy sz col
        self.packet_message = "sphere_{}00".format(self.id)  + ",{},{},{},0,0,0,0,{},{},{},{},on"

        # Redraw objects
        self.remove()
        self.draw()


    def registerCallbacks(self):
        if self.ros_topic:
            self.ros_topic = "/" + self.ros_topic
            # Subscribe to vehicle state update
            print("Subscribing to: {}".format(self.ros_topic))
            rospy.Subscriber(self.ros_topic, PoseStamped, self.topic_callback)


    def topic_callback(self, msg):
        #If enough time has passed, trigger single packet animationg
        if rospy.get_time() > (self.last_time + self.interval/1000.0) and self.animate:
            tg_scene_string = self.scene + "/"  + "sphere" + "_{}00".format(self.id) + "/animation"
            cmd_string = "property: position; from: {} {} {}; to: {} {} {}; dur:{};".format(
                self.start_node.pos[0],self.start_node.pos[1],self.start_node.pos[2],
                self.end_node.pos[0],self.end_node.pos[1],self.end_node.pos[2],
                self.packet_duration)

            self.client.publish(tg_scene_string, cmd_string, retain=True)
            self.last_time = rospy.get_time()

        
    def draw(self):
        if self.active:
            color = self.color
            # self.visible = "on"
        else:
            color = "#000000"
            # self.visible = "off"


        mqtt_string = self.message.format(
            self.start_node.pos[0], self.start_node.pos[1], self.start_node.pos[2],
            self.end_node.pos[0], self.end_node.pos[1], self.end_node.pos[2],
            color, self.visible)

        # Draw object
        self.client.publish(self.base_topic, mqtt_string, retain=True)

        # Fill packet message
        packet_string = self.packet_message.format(
            self.start_node.pos[0], self.start_node.pos[1], self.start_node.pos[2], 
            self.packet_scale[0], self.packet_scale[1], self.packet_scale[2], self.color)
        
        self.client.publish(self.packet_base_topic, packet_string, retain=True)

        if self.transparent:
            self.client.publish(self.packet_base_topic + "/material", 
            "transparent: true; opacity: {}".format(self.opacity), retain=True)
            

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


###### TRAJECTORY ARENA CLASS ########
class TrajectoryArenaClass(RArenaClass):
    def __init__(self, client, scene, name, id, points=20, shape="sphere", opacity=0.5,
        scale=[0.2,0.2,0.2], color="#AAAAAA", source=None, on_click_clb=None, listen=True):

        self.Active = False

        self.on_click = on_click_clb
        self.source = source
        self.points = points
        
        self.ros_topic = None
        self.shape = shape
        self.scale = scale
        self.color = color
        self.start_time=None
        self.duration = 2
        self.opacity = opacity
        self.listen = listen
        self.time_interval=0


        self.topic_frame = scene + "/" + self.shape + "_{}"
        self.message_frame = self.shape + "_{},{},{},{}" + ",0,0,0,1,{},{},{}".format(self.scale[0],
            self.scale[1],self.scale[2]) + ",{},on".format(self.color)

        super(TrajectoryArenaClass, self).__init__(client, scene, name, id, color=color)

        self.registerCallbacks()

        print("Created Arena Trajectory Object")


    def initArenaObject(self):
        #                                                      x  y  z  qx qy qz qw sx sy sz col
        self.message = self.shape + "_{}".format(self.id)  + ",{},{},{},{},{},{},{},{},{},{},{},on"
        #                           x  y  z  qx qy qz qw sx sy sz txt
        self.text_message = "text" + "_{}".format(self.id) + ",{},{},{},{},{},{},{},{},{},{},{},on"

        # Redraw object
        # self.remove()

        # Enable click listener for object (allows it to be clickable)
        if self.listen:
            self.client.message_callback_add(self.scene + "/cube_6/mousedown", self.on_click_input)


    def registerCallbacks(self):
        if self.source:
            self.ros_topic = "/" + self.source + "/" + self.name + "/pose" 
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
            rospy.sleep(self.time_interval)


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
            self.remove_trajectory()
            self.start_time=None


    def remove(self):
        self.remove_trajectory()

    
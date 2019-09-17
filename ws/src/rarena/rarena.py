#!/usr/bin/env python

import paho.mqtt.client as mqtt
import rospy
import os
import sys
import time
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), './')))

from commander_interface.srv import GoTo 

from geometry_msgs.msg import PoseStamped

import numpy as np


###### HELPERS #####
def posFromPoseMsg(pose_msg):
    pos = np.array([pose_msg.pose.position.x, 
                pose_msg.pose.position.y, 
                pose_msg.pose.position.z])
    return pos

def quatFromPoseMsg(pose_msg):
    quat = np.array([pose_msg.pose.orientation.w,
        pose_msg.pose.orientation.x, 
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z])
    return quat 



################# ROS ARENA CLASS #####################
class RArenaClass(object):
    """
    Bridge class between Ros and Arena.
    """
    def __init__(self, mqtt_client, scene, obj_name, color, position=[0,0,0], scale=[1,1,1]):

        self.client = mqtt_client

        self.initData(scene, obj_name, position, scale, color)
        self.initArenaObject()
        rospy.loginfo("\n [%s] RArena Object Initialized!"%rospy.get_name()) 

    def __del__(self):
        self.client.publish(self.scene + self.object_name, "", retain=True)  

    def initData(self, scene, obj_name, pos, scale, color):
        self.scale = np.array([scale[0], scale[1], scale[2]])

        self.time_old = 0
        self.scene_path = scene
        self.object_name = obj_name  
        self.color = color 

        #                 x  y  z  qx qy qz qw sx sy sz col
        self.base_str = ",{},{},{},{},{},{},{},{},{},{},{},on"

        self.obj_str = self.object_name + self.base_str 
        self.p = np.array(pos)
        self.q = np.zeros(4)
        self.q[0] = 1.0


    def initArenaObject(self):        
        ############
        # Delete the object from the scene to get a fresh start with a null message
        self.client.publish(self.scene_path + self.object_name, "")  
        
        # Publish a cube with x,y,z and color parameters
        # retain=True makes it persistent
        mqtt_string = self.obj_str.format(
                str(self.p[0]), str(self.p[1]), str(self.p[2]),
                str(0.0), str(0.0), str(0.0), str(0.0),
                self.scale[0], self.scale[1], self.scale[2], 
                self.color)

        self.client.publish(self.scene_path + self.object_name, mqtt_string, retain=True)
        print("Creating: ", self.scene_path + self.object_name)
        print("Mqtt Command: ", mqtt_string)

        # Enable click listener for object (allows it to be clickable)
        self.client.publish(self.scene_path + self.object_name + "/click-listener", 
                "enable", retain=True)

        #self.client.subscribe(self.scene_path + self.object_name + "/mouseup")
        self.client.subscribe(self.scene_path + self.object_name + "/mousedown")

        #self.client.message_callback_add(self.scene_path + self.object_name + "/mouseup", self.on_click_input)
        self.client.message_callback_add(self.scene_path + self.object_name + "/mousedown", 
                self.on_click_input)


 
    def plot_arenaObj(self, pos, quaternion, scale = None, color = None):
        """
        Update the coordinates of an object in arena and
        plot it in the synthetic environment
        """
        # Update the pos of the object
        self.p[0] = float(pos[0])
        self.p[1] = float(pos[1])
        self.p[2] = float(pos[2])

        if (scale is not None):
            sc = scale
        else:
            sc = self.scale

        if (color is not None):
            cl = color
        else:
            cl = self.color

        tg_scene_string = self.scene_path + self.object_name
        cmd_string = self.obj_str.format(
                    str(pos[0]), 
                    str(pos[2]), 
                    str(-pos[1]), 
                    str(quaternion[1]), str(quaternion[3]), str(-quaternion[2]), str(quaternion[0]), 
                    str(sc[0]), str(sc[1]), str(sc[2]), cl)

        time_now = time.time()
        diff = time_now - self.time_old
        if (diff) > 0.3:
            self.time_old = time_now 
            self.client.publish(tg_scene_string, 
                    cmd_string, 
                    retain=True)

        return

    def on_click_input(self, client, userdata, msg):
        print("Got click: %s \"%s\"" % (msg.topic, msg.payload))
        click_x, click_y, click_z, user = msg.payload.split(',')
        print("Clicked by: " + user)    

    def set_color(self, new_color):
        self.color = new_color
       



###### DRONE AREAN CLASS ########
class DroneArenaClass(RArenaClass):
    # Constructor
    def __init__(self, on_click_clb, mqtt_clt, scene_path, name, isMovable = True):
        print("Creating Arena Drone Object")

        self.name = name
        self.isActive = False
        
        self.on_click = on_click_clb
 
        # Name of the topic containing the pose of the object       
        self.obj_pose_topic_ =  "/" + name + "/external_pose"

        self.position = [0,0.01,0]
        self.quaternion = [0,0,0,1]

        super(DroneArenaClass, self).__init__(mqtt_clt, scene_path, obj_name = self.name, 
               color="#0000FF", position=self.position, scale=[0.3,0.1,0.2])

        if (isMovable):
            self.registerCallbacks()

        self.registerServices()


    def registerCallbacks(self):
        # Subscribe to vehicle state update
        print("Subscribing to ", self.obj_pose_topic_)
        rospy.Subscriber(self.obj_pose_topic_, 
                PoseStamped, self.pose_callback)
       

    def pose_callback(self, pose_msg):
        self.position = posFromPoseMsg(pose_msg)
        self.quaternion = quatFromPoseMsg(pose_msg)
        
        # plot the object in arena
        super(DroneArenaClass, self).plot_arenaObj(self.position, self.quaternion, color=self.color)

        
    def registerServices(self):
        rospy.wait_for_service("/" + self.name + "/Commander_Node/goTo_srv")
        self.goTo = rospy.ServiceProxy("/" + self.name + "/Commander_Node/goTo_srv", GoTo)
        pass


    def on_click_input(self, client, userdata, msg):
        click_x, click_y, click_z, user = msg.payload.split(',')
    
        if (self.isActive):
            self.isActive = False
            super(DroneArenaClass, self).set_color('#000022')
            self.color = "#000022"
            print("Drone {} Deselected\n".format(self.name))
            super(DroneArenaClass, self).plot_arenaObj(self.position, self.quaternion)
        else:
            self.isActive = True
            super(DroneArenaClass, self).set_color('#00FF00')
            print("Drone {} Selected\n".format(self.name))
            super(DroneArenaClass, self).plot_arenaObj(self.position, self.quaternion)

        self.on_click(self.name) 
 

            
###### TARGET ARENA CLASS #######
class TargetArenaClass(RArenaClass):
    def __init__(self, on_click_clb, mqtt_client, scene, name, c = "#FFFFFF",
            p = [0,0,0], s = [0.5, 0.5, 0.5], isMovable=False):

        print("Creating Arena Target Object")
        self.on_click = on_click_clb
        super(TargetArenaClass, self).__init__(mqtt_client, scene, obj_name = name, 
                color = c, position = p, scale=s)

        if (isMovable):
            self.obj_pose_topic_ =  "/" + name + "/external_pose"
            self.registerCallbacks()
            
            
    def on_click_input(self, client, userdata, msg):
        click_x, click_y, click_z, user = msg.payload.split(',')

        # Create array with the target position

        if str(msg.topic).find("mousedown") != -1:
            print( "Target Position: " + str(click_x) + "," + str(click_y) + "," + str(click_z) )
            
            tg_string = "TargetGoto" + self.base_str;
            client.publish(self.scene_path + "TargetGoto", 
                    tg_string.format(click_x, click_y, click_z, 0.0, 0.0, 0.0, 0.0, 
                    0.1, 0.1, 0.1, "#FF0000"), retain=True) 

            tg_p = np.array([float(click_x), -float(click_z), float(click_y)])
            self.on_click(tg_p)
                        
    def registerCallbacks(self):
        # Subscribe to vehicle state update
        print("Subscribing to ", self.obj_pose_topic_)
        rospy.Subscriber(self.obj_pose_topic_, 
                PoseStamped, self.pose_callback)

    def pose_callback(self, pose_msg):
        self.position = posFromPoseMsg(pose_msg)
        self.quaternion = quatFromPoseMsg(pose_msg)
      
        # plot the object in arena
        super(TargetArenaClass, self).plot_arenaObj(self.position, self.quaternion, color=self.color)


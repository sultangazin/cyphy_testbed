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


activeDrone = None

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


def toggle_active(name):
    if (activeDrone == name):
        activeDrone = None
    else:
        activeDrone = name


def issue_command(tg_p):
    if (activeDrone == None):
        print("No drone selected!")
    else:
        try:
            resp1 = drones[active].goTo([tg_p[0], tg_p[1], tg_p[0]], 2.0)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))



################# ROS ARENA CLASS #####################
class RArenaClass:
    """
    Bridge class between Ros and Arena.
    """
    def __init__(self, mqtt_client, scene, obj_name, obj_type, position=[0,0,0], scale=[1,1,1]):

        self.client = mqtt_client

        self.initData(scene, obj_name, obj_type, position, scale)

        self.initArenaObject()

        rospy.loginfo("\n [%s] RArena Object Initialized!"%rospy.get_name()) 

    def __del__(self):
        self.client.publish(self.scene + self.object_name, "", retain=True)  

    def initData(self, scene, obj_name, obj_type, pos, scale):
        self.scale = np.array([scale[0], scale[1], scale[2]])

        self.counter = 0
        self.scene_path = scene

        self.object_name = obj_name
        self.object_type = obj_type 
        
        self.base_str = ",{},{},{},{},{},{},{},{},{},{},{},on"

        self.obj_str = self.object_name + self.base_str 
        self.p = np.array(pos)
        self.q = np.zeros(4)
        self.q[0] = 1.0

        self.has_datasource = False
 
        

    def initArenaObject(self):
        
        ############
        # Setup box
        # Delete the object from the scene to get a fresh start with a null message
        self.client.publish(self.scene_path + self.object_name, "")  
        
        # Publish a cube with x,y,z and color parameters
        # retain=True makes it persistent
        self.client.publish(self.scene_path + self.object_name, 
            self.obj_str.format(str(self.p[0]), str(self.p[1]), str(self.p[2]),
                str(0.0), str(0.0), str(0.0), str(0.0),
                self.scale[0], self.scale[1], self.scale[2], 
                self.color), 
            retain=True)

        # Enable click listener for object (allows it to be clickable)
        self.client.publish(self.scene_path + self.object_name + "/click-listener", "enable", retain=True)

        #self.client.subscribe(self.scene_path + self.object_name + "/mouseup")
        self.client.subscribe(self.scene_path + self.object_name + "/mousedown")

        #self.client.message_callback_add(self.scene_path + self.object_name + "/mouseup", self.on_click_input)
        self.client.message_callback_add(self.scene_path + self.object_name + "/mousedown", self.on_click_input)


               
    def plot_arenaObj(pos, quaternion):
        """
        Update the coordinates of an object in arena and
        plot it in the synthetic environment
        """
        # Update the pos of the object
        self.p[0] = float(pos[0])
        self.p[1] = float(pos[1])
        self.p[2] = float(pos[2])

        tg_scene_string = self.scene_path + self.object_name
        cmd_string = self.obj_str.format(
                    str(pos[0]), 
                    str(pos[2]), 
                    str(-pos[1]), 
                    str(quaterion[1]), str(-quaternion[3]), str(quaternion[2]), str(quaternion[0]), 
                    self.scale[0], self.scale[1], self.scale[2],
                    str(self.color))


        self.counter = self.counter + 1
        if (self.counter == 10):
            self.counter = 0
            self.client.publish(tg_scene_string, 
                    cmd_string, 
                    retain=True)

        return

    def on_click_input(self, client, userdata, msg):

        print("Got click: %s \"%s\"" % (msg.topic, msg.payload))
        click_x, click_y, click_z, user = msg.payload.split(',')
        print("Clicked by: " + user)    

        

###### DRONE AREAN CLASS ########
def DroneArenaClass(RArenaClass):
    def __init__(self, mqtt_client, scene, obj_name):
        print("Creating Arena Drone Object")

        self.name = name
        self.isActive = False
 
        # Name of the topic containing the pose of the object       
        self.obj_pose_topic_ =  "/" + name + "/external_pose"

        super().__init__(self, mqtt_client, scene, obj_name = self.name, obj_type = "drone", position=[0,0,0], scale=[1,1,1]):  
        super().color = "#0000FF"

        self.registerCallbacks()

        self.registerServices()


    def registerCallbacks(self):
        # Subscribe to vehicle state update
        print("Subscribing to ", self.obj_pose_topic_)
        rospy.Subscriber(self.obj_pose_topic_, 
                PoseStamped, self.pose_callback)
       

    def pose_callback(self, pose_msg):
        position = posFromPoseMsg(pose_msg)
        quaternion = quatFromPoseMsg(pose_msg)
        
        # Plot the object in Arena
        super().plot_arenaObj(position, quaternion)

        
    def registerServices(self):
        rospy.wait_for_service("/" + self.name + "/Commander_Node/goTo_srv")
        self.goTo = rospy.ServiceProxy("/" + self.name + "/Commander_Node/goTo_srv", GoTo)
        pass


    def on_click_input(self, client, userdata, msg):
        click_x, click_y, click_z, user = msg.payload.split(',')
    
        if (self.isActive):
            self.isActive = False
            super().color("#000022")
            print("Drone {} Deselected\n".format(self.name))
        else:
            self.isActive = True
            super().color("#0000FF")
            print("Drone {} Selected\n".format(self.name))

        toggle_active(self.name) 
 

            
###### TARGET ARENA CLASS #######
def TargetArenaClass(RArenaClass):
    def __init__(self, name):
        super().color = "#FFFFFF"
        print("Creating Arena Target Object")

        self.name = name

        super().__init__(self, mqtt_client, scene, obj_name = self.name, obj_type = "target", position=[0,0,0], scale=[1,1,1]):  
        super().color = "#0000FF"


    def on_click_input(self, client, userdata, msg):
        click_x, click_y, click_z, user = msg.payload.split(',')

        # Create array with the target position

        if str(msg.topic).find("mousedown") != -1:
            print( "Target Position: " + str(click_x) + "," + str(click_y) + "," + str(click_z) )
            
            tg_string = "TargetGoto" + self.base_str;
            super().client.publish(self.scene_path + "TargetGoto", tg_string.format(click_x, click_y, click_z), 0.0, 0.0, 0.0, 0.0,
                0.1, 0.1, 0.1, "#FF0000"), retain=True) 

            tg_p = np.array([float(click_x), -float(click_z), float(click_y)])
            issue_command(tg_p)
                        


drones = dict('cf1', DroneArenaClass(mqtt_client, "topic/luigi", "cf1"))

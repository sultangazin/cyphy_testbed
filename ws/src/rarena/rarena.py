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
class RArenaClass:

    def __init__(self, mqtt_client, scene, obj_name, obj_type, position=[0,0,0], scale=[1,1,1]):

        self.client = mqtt_client

        self.initData(scene, obj_name, obj_type, position, scale)

        self.loadParameters()

        self.registerCallbacks()

        self.registerServices()

        self.initArena()

        rospy.loginfo("\n [%s] RArena Initialized!"%rospy.get_name()) 

    def __del__(self):
        self.client.publish(self.scene + self.object_name, "", retain=True)  

    def initData(self, scene, obj_name, obj_type, pos, scale):
        self.scale = np.array([scale[0], scale[1], scale[2]])

        self.counter = 0
        self.scene_path = scene

        self.object_name = obj_name
        self.object_type = obj_type
        
        if (self.object_type == "drone"):
            print("Creating Arena Drone Object")
            self.color = "#0000FF"
        else:
            print("Creating Arena Target Object")
            self.color = "#FFFFFF"

        self.base_str = ",{},{},{},{},{},{},{},{},{},{},{},on"

        self.obj_str = self.object_name + self.base_str 
        self.p = np.array(pos)
        self.q = np.zeros(4)
        self.q[0] = 1.0

        self.has_datasource = False
 
        
    def loadParameters(self):
        # Check whether the param specifying the pose topic is defined
        #if rospy.has_param('/' + self.object_name + '/aggregator/topics/out_ext_pose_topic'):
        if (self.object_type == "drone"):
            self.has_datasource = True
            self.obj_pose_topic_ =  "/cf1/external_pose"


    def initArena(self):
        
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


    def registerCallbacks(self):
        if (self.has_datasource):
            # Subscribe to vehicle state update
            print("Subscribing to ", self.obj_pose_topic_)
            rospy.Subscriber(self.obj_pose_topic_, 
                    PoseStamped, self.pose_callback)

    def registerServices(self):
        rospy.wait_for_service('/cf1/Commander_Node/goTo_srv')
        self.goTo = rospy.ServiceProxy('/cf1/Commander_Node/goTo_srv', GoTo)
        #self.service_callback =  srv_call
        pass
        

    ###### CALLBACKS
    # On new target information
    def pose_callback(self, pose_msg):
        pos = posFromPoseMsg(pose_msg)
        quat = quatFromPoseMsg(pose_msg)

        # Update the position of the object
        self.p[0] = float(pos[0])
        self.p[1] = float(pos[1])
        self.p[2] = float(pos[2])
        
        tg_scene_string = self.scene_path + self.object_name
        cmd_string = self.obj_str.format(
                    str(pos[0]), 
                    str(pos[2]), 
                    str(-pos[1]), 
                    str(quat[1]), str(-quat[3]), str(quat[2]), str(quat[0]), 
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

        if (self.object_type == "target"):
            # Create array with the target position
            tg_p = np.array([float(click_x), float(click_y), float(click_z)])

            if str(msg.topic).find("mousedown") != -1:
                print( "Target Position: " + str(tg_p[0]) + "," + str(tg_p[1]) + "," + str(tg_p[2]) )
                
                try:
                    resp1 = self.goTo([tg_p[0], -tg_p[2], tg_p[1] + 1.0], 2.0)
                except rospy.ServiceException as exc:
                    print("Service did not process request: " + str(exc))

                tg_string = "TargetGoto" + self.base_str;
                self.client.publish(self.scene_path + "TargetGoto", tg_string.format(str(tg_p[0]), str(tg_p[1] + 1.0), str(tg_p[2]), 0.0, 0.0, 0.0, 0.0,
                    0.1, 0.1, 0.1, "#FF0000"), retain=True) 

        if (self.object_type == "drone"):
            print("Drone Selected\n")

